#!/usr/bin/env python3
"""Pack a UAV submission without importing or executing strategy code."""

import argparse
import ast
import hashlib
import json
import os
import re
import stat
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path


PACKAGE_NAME = "uav_combat_control"
FIXED_LAUNCH_FILE = Path("launch/control.launch.py")
MAX_FILES = 2000
MAX_ARCHIVE_BYTES = 64 * 1024 * 1024
MAX_EXPANDED_BYTES = 256 * 1024 * 1024
MAX_XML_BYTES = 512 * 1024
MAX_PATH_BYTES = 512
MAX_PATH_DEPTH = 20
MAX_COMPRESSION_RATIO = 200
FIXED_ZIP_TIME = (1980, 1, 1, 0, 0, 0)
EXCLUDED_DIRECTORIES = {
    ".git", ".idea", ".vscode", ".venv", ".pytest_cache", ".mypy_cache",
    "__pycache__", "build", "install", "log", "dist", "node_modules", "venv",
}


def resolve_package_root(source):
    source = source.absolute()
    if source.is_symlink() or not source.is_dir():
        raise ValueError("所选工作区的策略目录不可用，请重新选择工作区")
    if (source / "package.xml").is_file():
        return source.resolve()
    roots = []
    for entry in source.iterdir():
        if (entry.name not in EXCLUDED_DIRECTORIES
                and not entry.is_symlink() and entry.is_dir()
                and (entry / "package.xml").is_file()):
            roots.append(entry)
    if len(roots) != 1:
        raise ValueError("工作区中应有一个且仅有一个包含 package.xml 的策略目录")
    return roots[0].resolve()


def collect_files(root, output):
    files = []
    expanded_bytes = 0
    output = output.resolve() if output is not None else None

    def visit(directory):
        nonlocal expanded_bytes
        for path in sorted(directory.iterdir()):
            relative = path.relative_to(root)
            name = relative.as_posix()
            metadata = path.lstat()
            if stat.S_ISLNK(metadata.st_mode):
                raise ValueError(f"工作区包含链接文件（{name}），请移除后重试")
            if stat.S_ISDIR(metadata.st_mode) and path.name in EXCLUDED_DIRECTORIES:
                continue
            if "\\" in name or re.match(r"^[A-Za-z]:", name):
                raise ValueError(f"工作区包含无效文件路径（{name}）")
            if (len(name.encode("utf-8")) > MAX_PATH_BYTES
                    or len(relative.parts) > MAX_PATH_DEPTH):
                raise ValueError(f"文件路径过长或目录层级过深（{name}）")
            if stat.S_ISDIR(metadata.st_mode):
                visit(path)
                continue
            if not stat.S_ISREG(metadata.st_mode):
                raise ValueError(f"工作区包含不支持的文件（{name}）")
            lowered = path.name.lower()
            if (lowered == ".env" or lowered.startswith(".env.")
                    or path.suffix.lower() in {".pem", ".key"}):
                raise ValueError(f"请移除可能包含凭据的文件（{name}）后重试")
            if (path.name in {".git", ".DS_Store"}
                    or path.suffix.lower() in {".pyc", ".pyo"}):
                continue
            if output is not None and path == output:
                continue
            if metadata.st_size > MAX_ARCHIVE_BYTES:
                raise ValueError(f"文件超过 64 MiB（{name}）")
            expanded_bytes += metadata.st_size
            files.append((path, metadata))
            if len(files) > MAX_FILES or expanded_bytes > MAX_EXPANDED_BYTES:
                raise ValueError("工作区文件过多或总体积过大，请精简后重试")

    visit(root)
    return files


def validate(root, files):
    paths = {path.relative_to(root).as_posix(): metadata for path, metadata in files}
    if "package.xml" not in paths:
        raise ValueError("package.xml 必须位于策略包根目录")
    if any(name.endswith("/package.xml") for name in paths):
        raise ValueError("策略包内只能有一个 package.xml，且必须位于顶层")
    if paths["package.xml"].st_size > MAX_XML_BYTES:
        raise ValueError("package.xml 文件过大，请精简后重试")
    package = ET.parse(root / "package.xml").getroot()
    if (package.findtext("name") or "").strip() != PACKAGE_NAME:
        raise ValueError(f"package.xml 中的包名须为 {PACKAGE_NAME}")
    if (package.findtext("export/build_type") or "").strip() != "ament_python":
        raise ValueError("package.xml 中的构建类型须为 ament_python")
    if FIXED_LAUNCH_FILE.as_posix() not in paths:
        raise ValueError(f"策略包内缺少 {FIXED_LAUNCH_FILE.as_posix()}")
    try:
        tree = ast.parse((root / FIXED_LAUNCH_FILE).read_text(encoding="utf-8"))
    except (SyntaxError, UnicodeDecodeError) as error:
        raise ValueError("control.launch.py 必须是有效的 UTF-8 Python 文件") from error
    if not any(
        isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "generate_launch_description" for node in ast.walk(tree)
    ):
        raise ValueError("control.launch.py 必须定义 generate_launch_description")


def write_zip(root, files, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.is_symlink():
        raise ValueError("输出 ZIP 不能是链接文件")
    with tempfile.NamedTemporaryFile(
        dir=output.parent, suffix=".zip", delete=False
    ) as temporary:
        temporary_path = Path(temporary.name)
    try:
        with zipfile.ZipFile(temporary_path, "w") as archive:
            for path, metadata in files:
                info = zipfile.ZipInfo(
                    path.relative_to(root).as_posix(), FIXED_ZIP_TIME
                )
                info.compress_type = zipfile.ZIP_DEFLATED
                info.create_system = 3
                mode = 0o755 if metadata.st_mode & 0o111 else 0o644
                info.external_attr = (stat.S_IFREG | mode) << 16
                flags = (os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
                         | getattr(os, "O_NONBLOCK", 0))
                with os.fdopen(os.open(path, flags), "rb") as source:
                    before = os.fstat(source.fileno())
                    if (not stat.S_ISREG(before.st_mode)
                            or before.st_size != metadata.st_size
                            or before.st_mtime_ns != metadata.st_mtime_ns):
                        raise ValueError("打包期间策略文件发生变化，请保存后重试")
                    copied = 0
                    with archive.open(info, "w") as target:
                        while True:
                            chunk = source.read(1024 * 1024)
                            if not chunk:
                                break
                            copied += len(chunk)
                            if copied > metadata.st_size:
                                raise ValueError("打包期间策略文件发生变化，请保存后重试")
                            target.write(chunk)
                    after = os.fstat(source.fileno())
                    if (copied != metadata.st_size
                            or after.st_mtime_ns != before.st_mtime_ns):
                        raise ValueError("打包期间策略文件发生变化，请保存后重试")
        if temporary_path.stat().st_size > MAX_ARCHIVE_BYTES:
            raise ValueError("ZIP 文件超过 64 MiB，请缩减后重试")
        with zipfile.ZipFile(temporary_path) as archive:
            if any(item.file_size / max(1, item.compress_size) > MAX_COMPRESSION_RATIO
                   for item in archive.infolist()):
                raise ValueError("ZIP 内有文件解压后体积异常，请精简后重试")
        temporary_path.replace(output)
    finally:
        temporary_path.unlink(missing_ok=True)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        root = resolve_package_root(args.source)
        output = (
            args.output or root / "dist" / f"{PACKAGE_NAME}-submission.zip"
        ).absolute()
        files = collect_files(root, output)
        validate(root, files)
        result = {"package": PACKAGE_NAME, "launch": FIXED_LAUNCH_FILE.as_posix(),
                  "fileCount": len(files)}
        if args.check_only:
            result["validation"] = "passed"
        else:
            write_zip(root, files, output)
            result.update(output=str(output), sha256=sha256(output))
        if args.json:
            print(json.dumps(result, ensure_ascii=False))
        else:
            for key, value in result.items():
                print(f"{key}={value}")
        return 0
    except (ValueError, OSError, ET.ParseError) as error:
        message = str(error)
        print(
            json.dumps({"error": message}, ensure_ascii=False)
            if args.json else message, file=sys.stderr
        )
        return 1


if __name__ == "__main__":
    sys.exit(main())
