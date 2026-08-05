# ADMM upper-control launch. mode:=distributed (default, P4/G4) runs one admm_agent_node
# per robot; mode:=centralized (P2 baseline oracle) runs the single fleet_centralized_node.
import ast
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _observation_sources():
    """The observation sources, from config/observation_sources.yaml.

    Data, not classes: the sources differ in a topic template, a history window and a flag,
    and two implementations of an interface would share every line of behaviour. Being a
    file also puts them within reach of scripts/_perception.sh, so the bring-up sequence is
    written once instead of once per gate.
    """
    path = os.path.join(get_package_share_directory('legged_admm_fleet'),
                        'config', 'observation_sources.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


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
    # Task 10 item 4 (inject_forged_obs), "first mover" case ONLY: the attacker's publisher must
    # exist before the real Gazebo one to win the GID-pin race, so it has to be a launch value
    # rather than a later `ros2 param set` (that is the "late impostor" case and needs no launch
    # support -- it fires the same parameter, just after bring-up). Default '0' on both -> every
    # agent gets inject_forged_obs=0 -> inert.
    obs_name = LaunchConfiguration('observation').perform(context).strip().lower()
    obs = _observation_sources().get(obs_name)
    if obs is None:
        raise ValueError(f"unknown observation source '{obs_name}'; "
                         f"config/observation_sources.yaml defines "
                         f"{sorted(_observation_sources())}")
    forged_obs_attacker = int(LaunchConfiguration('forged_obs_attacker').perform(context))
    forged_obs_target = int(LaunchConfiguration('forged_obs_target').perform(context))
    # Task 11 item 6 (arm selector, scripts/d_run.sh): which detector is armed and whether it
    # blocks are per-run, launch-time decisions -- design spec section 9 protocol item 1 requires
    # A1's mechanism to survive the whole matrix unchanged, so which one runs cannot be a
    # mid-run `ros2 param set` (obs_gate2 is deliberately absent from admm_agent_node.cpp's
    # runtime callback allowlist; see its REFUSED-param branch). detection_log_only IS runtime-
    # settable too (kept that way for the d_run.sh LIE_LOGONLY pre-arm sequence), so this is only
    # its bring-up default, not its only entry point.
    obs_gate2 = LaunchConfiguration('obs_gate2').perform(context).lower() in ('true', '1')
    detection_log_only = LaunchConfiguration('detection_log_only').perform(context).lower() in (
        'true', '1')
    obs_noise_seed = int(LaunchConfiguration('obs_noise_seed').perform(context))
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
                'enable_peer_keepout': LaunchConfiguration('enable_peer_keepout').perform(
                    context).lower() in ('true', '1'),
                'inject_forged_obs': forged_obs_target if int(i) == forged_obs_attacker else 0,
                'obs_window_s': float(obs['window_s']),
                'obs_gate2': obs_gate2,
                'detection_log_only': detection_log_only,
                'obs_noise_seed': obs_noise_seed,
            }],
            remappings=[
                # The observation channel is the observer's own sensor, subscribed under a LOCAL
                # name so a real perception source can be swapped in without touching the node.
                # The remap itself grants no security -- the resolved DDS topic is still one
                # global name -- the publisher-GID pin in admm_agent_node.cpp is what actually
                # rejects an impostor writer (see the subscription's comment).
                #
                # Which topic that is comes from config/observation_sources.yaml, keyed by
                # observation:=<name>. The sources and their trade-offs are documented there,
                # next to the values, rather than here.
                (f'observed/robot{int(j)}',
                 obs['topic'].format(self=int(i), peer=int(j)))
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
        DeclareLaunchArgument(
            'observation', default_value='truth',
            description="Where 'I observe peer j' comes from: 'truth' = the Gazebo model "
                        "pose (the stand-in), 'lidar' = this dog's own lidar via "
                        "lidar_peer_tracker_node.py. 'lidar' needs the trackers and the "
                        "gz bridge running; nothing here starts them."),
        DeclareLaunchArgument('formation', default_value='V'),
        # task 4b spike: each agent's own local pairwise safety net against every peer it can
        # observe, bypassing edge_owner entirely (see AgentCore ctor in agent_core.hpp). Default
        # off -- this is a feasibility spike, not yet the shipped behaviour.
        DeclareLaunchArgument('enable_peer_keepout', default_value='false'),
        # Task 10 item 4 (inject_forged_obs) "first mover" case: which agent (robot_id, 0 = none)
        # impersonates forged_obs_target's ground-truth observation channel from t=0. The "late
        # impostor" case needs neither of these -- it is the same parameter, set later via
        # `ros2 param set` on whichever agent you choose, after bring-up.
        DeclareLaunchArgument('forged_obs_attacker', default_value='0'),
        DeclareLaunchArgument('forged_obs_target', default_value='0'),
        # Task 11 item 6: the arm selector's three bring-up knobs (see scripts/d_run.sh's ARM
        # case). Defaults reproduce pre-Task-11 behaviour exactly: gate2()'s single-shot EMA
        # (A1), blocking, seed 0.
        DeclareLaunchArgument('obs_gate2', default_value='false'),
        DeclareLaunchArgument('detection_log_only', default_value='false'),
        # Task 11 item 7: must vary per run, or a false-positive/refutation-rate measurement is
        # n=1 in the only dimension that produces one -- see admm::obs_noise.
        DeclareLaunchArgument('obs_noise_seed', default_value='0'),
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
