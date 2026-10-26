import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('uav_combat_sim')
    control_share = get_package_share_directory('uav_combat_control')
    sim_launch_path = os.path.join(sim_share, 'launch', 'run_sim.launch.py')
    control_launch_path = os.path.join(
        control_share, 'launch', 'control.launch.py')
    ros_home = os.environ.get('ROS_HOME', os.path.expanduser('~/.ros'))
    default_rootfs = os.path.join(ros_home, 'uav_combat_sim', 'px4', 'rootfs')

    sim_arguments = {
        'run_uxrce': LaunchConfiguration('run_uxrce'),
        'run_rosbag': LaunchConfiguration('run_rosbag'),
        'headless': LaunchConfiguration('headless'),
        'rootfs_dir': LaunchConfiguration('rootfs_dir'),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_uxrce', default_value='true',
            description='Start the Micro XRCE-DDS Agent'),
        DeclareLaunchArgument(
            'run_rosbag', default_value='true',
            description='Record game events and state'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo without gzclient'),
        DeclareLaunchArgument(
            'rootfs_dir', default_value=default_rootfs,
            description='Writable PX4 rootfs overlay directory'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path),
            launch_arguments=sim_arguments.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path),
            launch_arguments={'namespace': '/px4_1'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path),
            launch_arguments={'namespace': '/px4_2'}.items(),
        ),
    ])
