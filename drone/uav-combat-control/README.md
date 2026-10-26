# uav_combat_control

SIMURO AIC 无人机对抗赛 1v1 策略包（ROS 2 Humble / ament_python）。

## 策略概述

- 起飞后快速爬升到 1.45 m，满足 1.2 m 最低高度与 15 s 起飞期限。
- 使用 `enemy_pose` 估计对手速度和角速度，预测对手未来位置与机头方向。
- 以对手机尾后方 1.0 m 为目标，采用平滑速度场 + 切向绕行尾追，不频繁切换航点。
- 近距离自动后退保持识别距离；远离时预测追击；超时 90 s 后切换顺时针绕柱积累净圈数。
- 中心柱排斥、墙体排斥、场地边界钳制、对手碰撞保护、最低高度钳制。
- 二维码识别：整帧 pyzbar、中心 ROI 放大、自适应阈值三级回退；上报间隔 40 ms，严格低于 30 Hz。
- 扫码成功后在安全距离悬停 10 s，避免上报后碰撞/撞墙。

## 本地实测

- 环境：Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + PX4 SITL + SIMURO env 2.0.10 / sample 0.3.4。
- 静止二维码目标：开赛后 **9.00 s** 完成扫码上报。
- 对官方 sample 0.3.4：开赛后 **8.70 s** 完成扫码上报，官方样例未上报。
- 二维码内容校验：正确读取 `FEDCBA0987654321`。
- 起飞阶段水平移动与爬升并行，`takeoff_hold_sec` 缩短到 0.5 s，避免先垂直爬升再追击。

## 入口

- `launch/control.launch.py`：平台固定入口，启动 `flight_strategy_node`。
- `flight_strategy_node`：主策略节点，并自动拉起独立的 `qr_scanner_node` 子进程，避免图像解码阻塞飞控。
- `sample_strategy_node`：兼容入口，行为与 `flight_strategy_node` 相同。
- `qr_scanner_node`：独立二维码节点，也可单独运行用于调试。

## 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `strategy_mode` | `pursuit` | 持续追尾扫描 |
| `altitude` | `1.45` | ENU 目标高度 |
| `minimum_altitude` | `1.3` | 安全最低高度 |
| `takeoff_hold_sec` | `0.5` | 到达高度后的保持时间 |
| `scan_follow_distance` | `1.0` | 尾后目标距离 |
| `scan_approach_velocity` | `0.9` | 水平速度上限 |
| `scan_minimum_opponent_clearance` | `0.85` | 对手最小净空 |
| `scan_opponent_slowdown_distance` | `1.6` | 对手减速距离 |
| `center_obstacle_clearance_radius` | `0.8` | 中心柱净空 |
| `field_margin` | `0.6` | 场地边界安全距离 |
| `enemy_prediction_horizon` | `0.5` | 对手预测时长 |
| `scan_timeout_sec` | `90.0` | 未扫码转绕圈兜底时间 |
| `post_submit_hold_sec` | `10.0` | 上报后悬停时间 |

参数在 `launch/control.launch.py` 与 `nodes/flight_strategy_node.py` 中同步维护。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_combat_control
source install/setup.bash
ros2 pkg executables uav_combat_control
```

## 本地运行

```bash
ros2 launch uav_combat_control run_match.launch.py headless:=true run_rosbag:=false
ros2 topic pub --once /game_start std_msgs/msg/Bool "{data: true}"
```

本地无 Launcher 裁判时，需要额外提供 `local_pose` / `enemy_pose`；官方 Launcher 环境会自动提供这些公开接口。

## 提交

```bash
python3 tools/pack_submission.py --source . --output uav_combat_control-submission.zip
```

ZIP 根目录直接包含 `package.xml`、`launch/control.launch.py` 和策略源码。
