from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
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
                    'altitude': 1.35,
                    'lap_high_altitude': 2.3,
                    'horizontal_velocity': 1.0,
                    'radius': 1.3,
                    'takeoff_hold_sec': 0.5,
                    'minimum_altitude': 1.3,
                    'maximum_altitude': 2.5,
                    'field_half_x': 4.5,
                    'field_half_y': 3.0,
                    'field_margin': 0.6,
                    'enemy_pose_timeout': 0.6,
                }],
            ),
        ]),
    ])
