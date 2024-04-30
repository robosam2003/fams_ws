import launch
from launch import LaunchContext
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
import launch_ros
from launch_ros.actions import PushRosNamespace 
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
    pkg_share = launch_ros.substitutions.FindPackageShare(package='mover6_description').find('mover6_description')

    # Excecute the can setup command
    modprobe_can_command = ExecuteProcess(
        cmd=['sudo', 'modprobe', 'peak_usb', '&&', 'sudo', 'modprobe', 'peak_pci'],
        output='screen'
    )
    can_setup_command = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', 'can0', 'type', 'can', 'bitrate', '500000'],
        output='screen'
    )

    # Robot URDF file
    default_model_path = os.path.join(pkg_share, 'src/description/CPRMover6WithGripper.urdf.xacro')
    # Pass name param to the xacro file
    # load_param = launch.actions.ExecuteProcess(
    #     cmd=['xacro', default_model_path, 'prefix:=mover6', '-o', processed_model_path],
    #     output='screen'
    # )

    # Rviz config file
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config_mover6.rviz')

    robot_name = LaunchConfiguration('robot_name')
    robot_name_str = robot_name.perform(context) # A string

    robot_type = "mover6"

    initial_base_link_pos = LaunchConfiguration('initial_base_link_pos')
    initial_base_link_pos_list = initial_base_link_pos.perform(context) # A string of the form "x y yaw"

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,        
        name="mover6_robot_state_publisher",
        parameters=[{'robot_description': Command(['xacro ', default_model_path])},
                    {'frame_prefix': PathJoinSubstitution([robot_name_str, ''])}],
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='mover6_joint_state_publisher',
        namespace=robot_name,
        parameters=[{'source_list': ['mover6_joint_states']},
                    {'robot_description': Command(['xacro ', default_model_path])}],
    )

    tf_broadcast_node = launch_ros.actions.Node(
        package='tf_broadcast',
        executable='tf_broadcaster',
        namespace=robot_name,
        name='tf_broadcaster',
        output='screen',
        parameters=[{'robot_name': robot_name_str},
                    {'robot_type': robot_type},
                    {'initial_base_link_pos': initial_base_link_pos_list}
                ]
    )

    mover6_node = launch_ros.actions.Node( 
        package='mover6',
        executable='mover6',
        namespace=robot_name,
        name='mover6',
        output='screen',
    )

    workstation_node = launch_ros.actions.Node(
        package='workstation',
        executable='workstation',
        namespace=robot_name,
        name='workstation',
        output='screen',
    )


    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    # )

    return [
        modprobe_can_command,
        can_setup_command,
        robot_state_publisher_node,
        joint_state_publisher_node,
        tf_broadcast_node,
        mover6_node,
        workstation_node,
        # rviz_node,
    ]
    

def generate_launch_description():


    return launch.LaunchDescription([
        # Commented out for Gazebo simulation
        # launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                     description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        # launch.actions.DeclareLaunchArgument(name='run_rviz', default_value='True',
        #                                      description="whether or not this launch file runs RVIZ2"),

        launch.actions.DeclareLaunchArgument(
            'robot_name',
            default_value='mover6',
            description='Name of the robot, also doubles as the namespace and frame_prefix name'
        ),
        launch.actions.DeclareLaunchArgument(
            'initial_base_link_pos',
            default_value='0.0 0.0 0.0',
            description='Initial position of the base link in the world frame'
        ),
        OpaqueFunction(function=launch_setup),
    ])

