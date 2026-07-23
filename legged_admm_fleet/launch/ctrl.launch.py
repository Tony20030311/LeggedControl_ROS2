import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('legged_admm_fleet'), 'config')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                get_package_share_directory('legged_robot_controller'), 'launch', 'load_controller.launch.py'])),
            launch_arguments={
                'robots_config_file': os.path.join(cfg, 'fleet_robots.yaml'),
                'controller_param_file': os.path.join(cfg, 'vision60_fleet_controller.yaml'),
                'base_target_command': 'false',
            }.items()),
    ])
