# ADMM upper-control launch. mode:=distributed (default, P4/G4) runs one admm_agent_node
# per robot; mode:=centralized (P2 baseline oracle) runs the single fleet_centralized_node.
import ast

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _reference_file():
    return PathJoinSubstitution([
        get_package_share_directory('legged_robot_controller'),
        'config', 'vision60', 'reference.info'])


def _distributed_agents(context, *_a, **_k):
    ids = ast.literal_eval(LaunchConfiguration('robot_ids').perform(context))
    v = float(LaunchConfiguration('v').perform(context))
    deadline = int(LaunchConfiguration('hop_deadline_ms').perform(context))
    ref = _reference_file()
    # arena obstacle-CBF + A* apply per-agent identically to the centralized node.
    arena = LaunchConfiguration('arena').perform(context)
    use_astar = LaunchConfiguration('use_astar').perform(context).lower() in ('true', '1')
    nodes = []
    # standalone formation slot allocator (was hosted on dog1; now its own process, no dog special)
    nodes.append(Node(
        package='legged_admm_fleet',
        executable='fleet_coordinator_node',
        name='fleet_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_ids': [int(x) for x in ids],
            'formation': LaunchConfiguration('formation').perform(context),
        }],
    ))
    for i in ids:
        nodes.append(Node(
            package='legged_admm_fleet',
            executable='admm_agent_node',
            name=f'admm_agent_{i}',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'reference_file': ref,
                'robot_id': int(i),
                'robot_ids': [int(x) for x in ids],
                'v': v,
                'hop_deadline_ms': deadline,
                'formation': LaunchConfiguration('formation').perform(context),
                'arena': arena,
                'use_astar': use_astar,
                'hard_through': int(LaunchConfiguration('hard_through').perform(context)),
                'corpse_anchor_knot': int(
                    LaunchConfiguration('corpse_anchor_knot').perform(context)),
                'astar_robot_radius': float(LaunchConfiguration('astar_robot_radius').perform(context)),
                'astar_x_min': float(LaunchConfiguration('astar_x_min').perform(context)),
                'astar_x_max': float(LaunchConfiguration('astar_x_max').perform(context)),
                'astar_y_min': float(LaunchConfiguration('astar_y_min').perform(context)),
                'astar_y_max': float(LaunchConfiguration('astar_y_max').perform(context)),
            }],
            remappings=[
                # The observation channel is the observer's own sensor, subscribed under a LOCAL
                # name so a real perception source can be swapped in later without touching the
                # node. The remap itself grants no security -- the resolved DDS topic is still one
                # global name -- the publisher-GID pin in admm_agent_node.cpp is what actually
                # rejects an impostor writer (see the subscription's comment).
                (f'observed/robot{int(j)}', f'/robot{int(j)}/hardware/odom')
                for j in ids if int(j) != int(i)
            ],
        ))
    return nodes


def generate_launch_description():
    common = {
        'use_sim_time': True,
        'reference_file': _reference_file(),
        'arena': LaunchConfiguration('arena'),
        'use_astar': LaunchConfiguration('use_astar'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='distributed'),
        DeclareLaunchArgument('arena', default_value=EnvironmentVariable('FLEET_ARENA', default_value='')),
        DeclareLaunchArgument('use_astar', default_value='true'),
        DeclareLaunchArgument('astar_robot_radius', default_value='0.60'),
        # generous defaults covering both arenas (plum goals reach x=12.6, y=5.7; door to x=10):
        DeclareLaunchArgument('astar_x_min', default_value='-2.0'),
        DeclareLaunchArgument('astar_x_max', default_value='20.0'),  # plum goals now reach x=18
        DeclareLaunchArgument('astar_y_min', default_value='-7.0'),
        DeclareLaunchArgument('astar_y_max', default_value='7.0'),
        DeclareLaunchArgument('robot_ids', default_value='[1, 2, 3]'),
        DeclareLaunchArgument('v', default_value='0.4'),
        DeclareLaunchArgument('hop_deadline_ms', default_value='20'),
        # Number of edge-CBF steps k=0..hard_through-1 with slack forced to 0. Was reachable only
        # as a node default; exposed so the safety/feasibility trade can be swept. Raising it
        # tightens the published reference but risks an infeasible QP -> NaN -> fleet cold start.
        DeclareLaunchArgument('hard_through', default_value='1'),
        # Which knot of a dead peer's last plan predicts where its body parks. K_SEND (10) is
        # the correct and default value; N (20) is the known-unsafe setting kept only so the
        # before/after can be measured with ONE binary. The node warns loudly if it is not 10.
        DeclareLaunchArgument('corpse_anchor_knot', default_value='10'),
        DeclareLaunchArgument('formation', default_value='V'),
        Node(
            package='legged_admm_fleet',
            executable='fleet_centralized_node',
            name='fleet_centralized',
            output='screen',
            condition=LaunchConfigurationEquals('mode', 'centralized'),
            parameters=[dict(common, robot_ids=LaunchConfiguration('robot_ids'))],
        ),
        OpaqueFunction(
            function=_distributed_agents,
            condition=LaunchConfigurationEquals('mode', 'distributed'),
        ),
    ])
