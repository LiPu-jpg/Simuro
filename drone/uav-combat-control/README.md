# uav_combat_control

SIMURO AIC 1v1 无人机对抗赛策略包（ROS 2 Humble / ament_python）。

## 策略

- 绕圈优先：起飞后立即顺时针绕中心柱，优先保证净圈数。
- 高度错开：对手低于 1.8 m 时飞 2.3 m；对手高飞时保持 1.35 m，避免被尾后相机读取二维码。
- 二维码扫描节点始终运行，遇到可见二维码会正常上报；飞行路径不因扫码牺牲圈速。
- 固定入口为 `launch/control.launch.py`，适配平台传入的 `/px4_1`、`/px4_2` namespace。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `altitude` | `1.35` | 对手高飞时的低空高度 |
| `lap_high_altitude` | `2.3` | 对手低飞时的高空高度 |
| `horizontal_velocity` | `1.0` | 绕圈水平速度 |
| `radius` | `1.3` | 顺时针绕圈半径 |
| `takeoff_hold_sec` | `0.5` | 起飞后绕圈前等待 |
| `minimum_altitude` | `1.3` | 安全最低高度 |
| `maximum_altitude` | `2.5` | 安全最高高度 |
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
