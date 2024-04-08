import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros
import os
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)

def setup_launch(context, *args, **kwargs):
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    configured_params = launch.substitutions.LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))

    remap = LaunchConfiguration('remap')

    remap_str = str(remap.perform(context))

    # get the namespace
    namespace = LaunchConfiguration('namespace').perform(context)
    
    remappings = [('/cmd_vel', remap_str),
                  ('/'+namespace + '/map', '/map')]


    use_respawn = False
    map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        parameters=[configured_params],
        remappings=remappings,

    )

    controller = launch_ros.actions.Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                namespace=namespace,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings
    )
    smoother = launch_ros.actions.Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                namespace=namespace,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
    )
    planner = launch_ros.actions.Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                namespace=namespace,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
    )
    behaviours = launch_ros.actions.Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                namespace=namespace,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings

    )
    navigator = launch_ros.actions.Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                namespace=namespace,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
    )
    waypoint_follower = launch_ros.actions.Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                namespace=namespace,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
    )

    lifecycle_nodes = ['map_server',
                       'controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    lifecycle_manager_namespaced = launch_ros.actions.Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                namespace=namespace,
                output='screen',
                parameters=[
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}
                            ])
    
    lifecycle_manager = launch_ros.actions.Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                            {'autostart': True},
                            {'node_names': ['map_server']}
                            ])

    return [
        map_server,
        controller,
        smoother,
        planner,
        behaviours,
        navigator,
        waypoint_follower,
        lifecycle_manager_namespaced,
        # lifecycle_manager,
    
    ]

def generate_launch_description():

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'remap',
            # default_value must be iterable
            default_value="/sam_bot/cmd_vel",
            description='List of topics to remap'),
        DeclareLaunchArgument(
            'namespace',
            default_value='sam_bot',
            description='Namespace for the navigation stack'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='nav2_params.yaml',
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        # Run a ros2 command - the map server lifecycle node
        # ExecuteProcess(
        #     cmd=['ros2', 
        #     output='screen'
        # ),
        # run the setup_launch function
        OpaqueFunction(function=setup_launch)
    ])