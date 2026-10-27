from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # 固定入口：使用平台传入的 namespace（蓝 /px4_1，黄 /px4_2）。
    # 保留此入口和 /game_start 订阅；不要把 namespace 编号当作 PX4 system_id。
    # 启动约定见 flight_strategy_node.py 文件头；AI 修改也须遵守。
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='ROS namespace assigned to this controller application',
        ),
        GroupAction([
            PushRosNamespace(namespace),
            Node(
                package='uav_combat_control',
                executable='flight_strategy_node',
                output='screen',
                parameters=[{
                    'strategy_mode': 'lap_race',
                    'altitude': 1.35,
                    'horizontal_velocity': 1.0,
                    'radius': 1.3,
                    'orientation_mode': 1,
                    'orbit_laps': 0.25,
                    'takeoff_hold_sec': 0.5,
                    'scan_role_duration_sec': 45.0,
                    'scan_follow_distance': 1.0,
                    'scan_approach_velocity': 1.0,
                    'scan_minimum_opponent_clearance': 0.85,
                    'scan_opponent_slowdown_distance': 1.6,
                    'center_obstacle_clearance_radius': 0.8,
                    'center_obstacle_detour_offset': 1.1,
                    'scan_waypoint_tolerance': 0.15,
                    'enemy_prediction_horizon': 0.5,
                    'enemy_pose_timeout': 0.6,
                    'enemy_velocity_smoothing': 0.45,
                    'enemy_max_speed': 2.0,
                    'field_half_x': 4.5,
                    'field_half_y': 3.0,
                    'field_margin': 0.6,
                    'minimum_altitude': 1.3,
                    'maximum_altitude': 2.5,
                    'lap_high_altitude': 2.3,
                    'scan_route_refresh_sec': 0.8,
                    'scan_route_refresh_distance': 0.35,
                    'collision_avoid_distance': 0.55,
                    'post_submit_hold_sec': 10.0,
                    'scan_timeout_sec': 20.0,
                }],
            ),
        ]),
    ])
