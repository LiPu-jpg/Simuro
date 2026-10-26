#!/usr/bin/env python3

import ast
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from setuptools import find_packages, setup


PACKAGE_NAME = "uav_combat_control"
ROOT = Path(__file__).resolve().parent


def package_metadata():
    package = ET.parse(ROOT / "package.xml").getroot()
    maintainer = package.find("maintainer")
    return {
        "version": (package.findtext("version") or "").strip(),
        "description": (package.findtext("description") or "").strip(),
        "maintainer": (
            ""
            if maintainer is None or maintainer.text is None
            else maintainer.text.strip()
        ),
        "maintainer_email": (
            "" if maintainer is None else maintainer.attrib.get("email", "")
        ),
        "license": (package.findtext("license") or "").strip(),
    }


def discover_node_entry_points():
    entries = []
    node_dir = ROOT / PACKAGE_NAME / "nodes"
    for source_path in sorted(node_dir.glob("*.py")):
        if source_path.name.startswith("_"):
            continue
        tree = ast.parse(
            source_path.read_text(encoding="utf-8"), filename=str(source_path)
        )
        if not any(
            isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == "main"
            for node in tree.body
        ):
            raise RuntimeError(
                "ROS node module must define main(): "
                f"{source_path.relative_to(ROOT)}"
            )
        module_name = source_path.stem
        entries.append(f"{module_name} = {PACKAGE_NAME}.nodes.{module_name}:main")
    if not entries:
        raise RuntimeError(
            f"at least one ROS node module is required under {PACKAGE_NAME}/nodes"
        )
    return entries


def data_files(directory, pattern):
    paths = sorted((ROOT / directory).glob(pattern))
    if not paths:
        return []
    destination = os.path.join("share", PACKAGE_NAME, directory)
    return [(destination, [str(path.relative_to(ROOT)) for path in paths])]


metadata = package_metadata()

setup(
    name=PACKAGE_NAME,
    version=metadata["version"],
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        *data_files("launch", "*.launch.py"),
        *data_files("config", "*.yaml"),
        *data_files("config", "*.yml"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    python_requires=">=3.10,<3.11",
    maintainer=metadata["maintainer"],
    maintainer_email=metadata["maintainer_email"],
    description=metadata["description"],
    license=metadata["license"],
    tests_require=["pytest"],
    entry_points={"console_scripts": discover_node_entry_points()},
)
