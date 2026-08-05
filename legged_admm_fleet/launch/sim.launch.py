import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution


def _lidar_bridge(context, share, roster):
    """gz -> ROS bridge for the lidar, added only for rosters whose dogs actually carry one.

    Both rosters use robot_type "vision60" (legged_robot_controller supports no other name), and
    gz_bridge.launch.py keys its template lookup on robot_type alone, so it cannot tell them
    apart. Started unconditionally, the plain roster gets three bridge nodes advertising six ROS
    lidar topics that no gz publisher will ever feed: a consumer subscribes and waits forever
    with nothing to say it is waiting on a sensor that does not exist. What does distinguish the
    rosters is description_package, since only the lidar one points the URDF at this package.

    (The bridge_node exit -11 seen on 2026-08-05 was not this: it happens on both rosters, after
    gz sim is SIGKILLed out from under the bridge, and is shutdown noise.)
    """
    path = roster.perform(context)
    with open(path, 'r', encoding='utf-8') as f:
        robots = yaml.safe_load(f) or {}
    carries_lidar = any(isinstance(c, dict) and c.get('description_package') == 'legged_admm_fleet'
                        for c in robots.values())
    if not carries_lidar:
        return []
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('legged_gazebo'), 'launch', 'gz_bridge.launch.py')),
        launch_arguments={'robots_config_file': path,
                          'bridge_config_root': os.path.join(share, 'config')}.items())]


def generate_launch_description():
    share = get_package_share_directory('legged_admm_fleet')
    # FLEET_ROSTER picks the roster: the default 3x vision60 one, or fleet_robots_lidar.yaml for
    # the same three dogs carrying a lidar.
    roster = PathJoinSubstitution([share, 'config',
                                   EnvironmentVariable('FLEET_ROSTER', default_value='fleet_robots.yaml')])
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
        # gazebo.launch.py runs its own bridge pass, but only against legged_gazebo's config root,
        # where vision60 has no gz_bridge.yaml at all. This is the pass that reaches ours.
        OpaqueFunction(function=_lidar_bridge, args=[share, roster]),
    ])
