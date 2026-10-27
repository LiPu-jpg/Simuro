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
                    'race_altitude': 1.35,
                    'takeoff_altitude': 1.5,
                    'takeoff_height': 1.15,
                    'orbit_radius': 1.4,
                    'orbit_speed': 1.0,
                    'center_clearance': 1.0,
                    'field_half_x': 4.5,
                    'field_half_y': 3.0,
                    'field_margin': 0.6,
                    'minimum_flight_altitude': 1.05,
                    'recovery_altitude': 1.55,
                    'enemy_clearance': 0.85,
                    'enemy_vertical_range': 0.45,
                    'enemy_pose_timeout': 0.6,
                }],
            ),
        ]),
    ])
