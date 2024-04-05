import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution
import launch_ros
import os
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    nav2_bringup_pkg_share = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')

    # nav_launch_file = os.path.join(nav2_bringup_pkg_share, 'launch', 'navigation_launch.py')
    nav2_custom_launch_file = os.path.join(pkg_share, 'launch', 'nav2_launch.py')

    default_model_path = os.path.join(pkg_share, 'src/description/nexus_4wd_mecanum.xacro')

    map_file = os.path.join(pkg_share, 'maps/test_map.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config/nav2_params.yaml')

    # Launch configuration variables
    robot_name = LaunchConfiguration('robot_name')

    # Launch arguments
    robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='nexus',
        description='Name of the robot, also doubles as the namespace and frame_prefix name'
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        name='nexus_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])},
                    {'frame_prefix': PathJoinSubstitution([robot_name, '/'])}],
    )
    # Joint State publisher - for the wheels simulation, also needed by robot_state_publisher
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_name,
        parameters=[{'source_list': ['sam_bot_wheel_states']},
                    {'robot_description': Command(['xacro ', default_model_path])}],
    )
    
    # map_server_node = launch_ros.actions.Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     parameters=[{'yaml_filename': map_file}],
    #     output='screen',
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
            'remappings': "/cmd_vel:=/"+str(robot_name)+"/cmd_vel",
        }.items()
    )



        
            

    return launch.LaunchDescription([
        # Commented out for Gazebo simulation
        # launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
        #                                     description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        # launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                     description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        # launch.actions.DeclareLaunchArgument(name='run_rviz', default_value='False',
        #                                     description="whether or not this launch file runs RVIZ2"),
        launch.actions.DeclareLaunchArgument(name="tf_broadcast", default_value="False",
                                             description="whether to publush tf in this namespace or not"),
        robot_name_cmd,

        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        # launch.actions.ExecuteProcess(cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server']),
        joint_state_publisher_node, # Commented out because gazebo plugin publishes joint states
        # joint_state_publisher_gui_node, # Commented out for Gazebo simulation
        robot_state_publisher_node,
        # spawn_entity,
        # map_server_node,
        sam_bot_node,
        # robot_localization_node,
        # rviz_node,
        nav2_custom_launch,
    ])