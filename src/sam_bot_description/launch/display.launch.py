import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    nav2_bringup_pkg_share = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')

    # nav_launch_file = os.path.join(nav2_bringup_pkg_share, 'launch', 'navigation_launch.py')
    nav2_custom_launch_file = os.path.join(pkg_share, 'launch', 'nav2_launch.py')

    default_model_path = os.path.join(pkg_share, 'src/description/nexus_4wd_mecanum.xacro')

    world_path=os.path.join(pkg_share, 'world/my_world.sdf'),
    map_file = os.path.join(pkg_share, 'maps/test_map.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    nav2_params_file = os.path.join(pkg_share, 'config/nav2_params.yaml')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='nexus',
        name='nexus_robot_state_publisher',
        # remappings=[('/joint_states', 'nexus/joint_states')],
        parameters=[{'robot_description': Command(['xacro ', default_model_path])},
                    {'frame_prefix': 'nexus/'}],
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='nexus',
        # remappings=[('/joint_states', 'nexus/joint_states')],
        parameters=[{'source_list': ['sam_bot_wheel_states']},
                    {'robot_description': Command(['xacro ', default_model_path])}],
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')) # Commented out for Gazebo simulation
    )
    # Commented out for Gazebo simulation
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('run_rviz')),

    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    
    map_server_node = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_file}],
        output='screen',
    )

    # Map->odom broadcaster
    # map_odom_tf_broadcaster = launch_ros.actions.Node(
    #     package='tf_broadcast',
    #     executable='map_odom_publisher',
    #     output='screen'
    # )

    sam_bot_node = launch_ros.actions.Node(
        package='sam_bot',
        executable='sam_bot',
        namespace='nexus',
        output='screen'
    )
  
    

    # nav2_bringup_launch = launch.actions.TimerAction(
    #     period=2.0,
    #     actions=[
    #         launch.actions.IncludeLaunchDescription(
    #             launch.launch_description_sources.PythonLaunchDescriptionSource(nav_launch_file),
    #             launch_arguments={
    #                 'use_sim_time': 'True',
    #                 'params_file': nav2_params_file,
    #                 'autostart': 'True',
    #                 # 'namespace': 'nexus',
    #                 'remappings': "/cmd_vel:=/nexus/cmd_vel",
    #             }.items()
    #         )
    #     ]
    # )

    nav2_custom_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(nav2_custom_launch_file),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'remappings': "/cmd_vel:=/nexus/cmd_vel",
        }.items()
    )



        
            

    return launch.LaunchDescription([
        # Commented out for Gazebo simulation
        # launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
        #                                     description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='run_rviz', default_value='False',
                                            description="whether or not this launch file runs RVIZ2"),
        launch.actions.DeclareLaunchArgument(name="tf_broadcast", default_value="True",
                                             description="whether to publush tf in this namespace or not"),
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        # launch.actions.ExecuteProcess(cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server']),
        joint_state_publisher_node, # Commented out because gazebo plugin publishes joint states
        # joint_state_publisher_gui_node, # Commented out for Gazebo simulation
        robot_state_publisher_node,
        # spawn_entity,
        map_server_node,
        sam_bot_node,
        # robot_localization_node,
        rviz_node,
        nav2_custom_launch,
    ])