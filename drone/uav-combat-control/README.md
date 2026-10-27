# uav_combat_control

SIMURO AIC 1v1 无人机对抗赛策略包（ROS 2 Humble / ament_python）。

## 策略

- 绕圈优先：起飞后保持约 1.9 m 顺时针绕中心柱，优先保证净圈数。
- 高度安全：最低飞行高度 1.5 m，低于 1.5 m 自动恢复到 2.0 m，避免 1.2 m 高度违规和触地。
- 防被扫：对手进入 2.5 m 内时，机头转向对手，让尾后二维码板背离对方相机。
- 中心柱保护：当前半径小于 1.2 m 时暂停绕圈、向外恢复，避免切柱。
- 机会扫码：二维码扫描节点始终运行，遇到可见二维码正常上报；飞行不追尾。
- 固定入口 `launch/control.launch.py`，适配平台传入的 `/px4_1`、`/px4_2` namespace。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `base_altitude` | `1.9` | 绕圈固定高度 |
| `takeoff_altitude` | `2.05` | 起飞目标高度 |
| `takeoff_threshold` | `1.7` | 进入绕圈的最低高度 |
| `orbit_radius` | `1.5` | 顺时针绕圈半径 |
| `orbit_speed` | `1.0` | 绕圈水平速度 |
| `flight_floor` | `1.5` | 低空保护阈值 |
| `recovery_altitude` | `2.0` | 低空恢复高度 |
| `center_clearance` | `1.2` | 中心柱恢复半径 |
| `enemy_clearance` | `0.85` | 同高度对手横向净空 |
| `enemy_vertical_range` | `0.45` | 触发横向避让的高度差 |
| `engage_distance` | `2.5` | 触发机头对准的距离 |
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
