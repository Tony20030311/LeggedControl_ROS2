# RViz view for the distributed fleet: arena markers (obstacles + walls, door jamb posts hidden)
# + each dog's MPC horizon + the 3 robot models, in one command. Keeps the gz GUI (does NOT go
# headless — that is only for automated log-only testing, via scripts/rviz_view.sh HEADLESS=1).
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('legged_admm_fleet')
    marker = os.path.join(get_package_prefix('legged_admm_fleet'),
                          'lib', 'legged_admm_fleet', 'fleet_viz_markers.py')
    return LaunchDescription([
        DeclareLaunchArgument('robot_ids', default_value='1 2 3'),
        DeclareLaunchArgument('arena', default_value=EnvironmentVariable('FLEET_ARENA', default_value='')),
        ExecuteProcess(
            cmd=['python3', marker, LaunchConfiguration('robot_ids'), LaunchConfiguration('arena'),
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen'),
        Node(
            package='rviz2', executable='rviz2', name='fleet_rviz',
            arguments=['-d', os.path.join(share, 'rviz', 'fleet.rviz')],
            parameters=[{'use_sim_time': True}], output='screen'),
    ])
