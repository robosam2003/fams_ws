import launch
from launch import LaunchContext
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from nav2_common.launch import RewrittenYaml, ReplaceString
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

def launch_setup(context: LaunchContext):
    # Files etc
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    # nav2_bringup_pkg_share = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    # nav_launch_file = os.path.join(nav2_bringup_pkg_share, 'launch', 'navigation_launch.py')
    nav2_custom_launch_file = os.path.join(pkg_share, 'launch', 'nav2_launch.py')
    default_model_path = os.path.join(pkg_share, 'src/description/nexus_4wd_mecanum.xacro')
    nav2_params_file = os.path.join(pkg_share, 'config/nav2_params.yaml')


    # Arguments
    robot_name = LaunchConfiguration('robot_name')
    robot_name_str = robot_name.perform(context) # A string

    # Get the last number in the robot_name string
    robot_number = int(robot_name_str[-1])
    other_nexus_number = 1 if robot_number == 2 else 2

    nav2_params_file = ReplaceString(
        source_file=nav2_params_file,
        replacements={'<namespace>': robot_name_str,
                        '<other_nexus>': 'nexus' + str(other_nexus_number)}

    )

    robot_type = 'sam_bot'
    
    initial_base_link_pos = LaunchConfiguration('initial_base_link_pos')
    initial_base_link_pos_list = initial_base_link_pos.perform(context) # A string of the form "x y yaw"


    new_cmd_vel_topic = robot_name_str + '/cmd_vel'

    # Robot State Publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        name='nexus_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])},
                    {'frame_prefix': PathJoinSubstitution([robot_name_str, ''])}],
    )
    # Joint State publisher - for the wheels simulation, also needed by robot_state_publisher
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_name,
        parameters=[{'source_list': ["wheel_states"]},
                    {'robot_description': Command(['xacro ', default_model_path])}],
    )
    
    # TF Broadcaster
    tf_broadcaster_node = launch_ros.actions.Node(
        package='tf_broadcast',
        executable='tf_broadcaster',
        namespace=robot_name,
        name='tf_broadcaster',
        output='screen',
        parameters=[{'robot_name': robot_name_str},
                    {'robot_type': robot_type},
                    {'initial_base_link_pos': initial_base_link_pos_list}, # x, y, yaw
                ]
    )

    sam_bot_node = launch_ros.actions.Node(
        package='sam_bot',
        executable='sam_bot',
        namespace=robot_name,
        output='screen'
    )


    
    nav2_custom_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(nav2_custom_launch_file),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'namespace': robot_name,
            'remap': new_cmd_vel_topic,
        }.items()
    )
    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        tf_broadcaster_node,
        sam_bot_node,
        nav2_custom_launch
    ]


def generate_launch_description():     
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='nexus',
            description='Name of the robot, also doubles as the namespace and frame_prefix name'
        ),
        DeclareLaunchArgument(
            'initial_base_link_pos',
            default_value='[0.0, 0.0, 0.0]',
            description='Initial position of the robot in the map frame'
        ),
        # Run the OpaqueFunction
        OpaqueFunction(function=launch_setup),
    ])
