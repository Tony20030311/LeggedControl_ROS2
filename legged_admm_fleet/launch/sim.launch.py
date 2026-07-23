import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution


def generate_launch_description():
    roster = os.path.join(get_package_share_directory('legged_admm_fleet'), 'config', 'fleet_robots.yaml')
    world = EnvironmentVariable('FLEET_ARENA', default_value='empty')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                get_package_share_directory('legged_gazebo'), 'launch', 'gazebo.launch.py'])),
            launch_arguments={'world': world, 'gazebo_package': 'legged_admm_fleet',
                              'robots_config_file': roster}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                get_package_share_directory('vision60_description'), 'launch', 'generate_urdf.launch.py'])),
            launch_arguments={'robot_type': 'vision60'}.items()),
    ])
