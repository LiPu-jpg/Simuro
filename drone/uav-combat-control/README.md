# uav_combat_control

SIMURO AIC 1v1 无人机对抗赛策略包（ROS 2 Humble / ament_python）。

## 策略

- 固定低空绕圈：起飞到安全高度后，保持约 1.35 m 顺时针绕中心柱，避免高度切换导致掉高。
- 防触地：当前高度低于 1.05 m 时，暂停绕圈并爬升到 1.55 m；绕圈半径过小、靠近中心柱时优先向外恢复。
- 防被扫：固定低空避开常见的高飞对手，二维码板不进入对手尾后相机的有效高度。
- 机会扫码：二维码扫描节点始终运行，遇到可见二维码正常上报；飞行路径不为扫码牺牲圈速。
- 固定入口 `launch/control.launch.py`，适配平台传入的 `/px4_1`、`/px4_2` namespace。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `race_altitude` | `1.35` | 绕圈固定高度 |
| `takeoff_altitude` | `1.5` | 起飞目标高度 |
| `takeoff_height` | `1.15` | 进入绕圈的最低高度 |
| `orbit_radius` | `1.4` | 顺时针绕圈半径 |
| `orbit_speed` | `1.0` | 绕圈水平速度 |
| `center_clearance` | `1.0` | 中心柱最小恢复半径 |
| `minimum_flight_altitude` | `1.05` | 触地保护阈值 |
| `recovery_altitude` | `1.55` | 触地保护恢复高度 |
| `enemy_clearance` | `0.85` | 同高度对手横向净空 |
| `enemy_vertical_range` | `0.45` | 触发横向避让的高度差 |
| `field_half_x` / `field_half_y` | `4.5` / `3.0` | 场地半尺寸 |
| `field_margin` | `0.6` | 场地边界安全距离 |
| `enemy_pose_timeout` | `0.6` | 对手位姿有效期 |

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_combat_control
source install/setup.bash
ros2 launch uav_combat_control control.launch.py namespace:=/px4_1
```
