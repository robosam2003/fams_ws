import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import PushRosNamespace 
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='mover6_description').find('mover6_description')

    # Robot URDF file
    default_model_path = os.path.join(pkg_share, 'src/description/CPRMover6WithGripper.urdf.xacro')
    # Pass name param to the xacro file
    # load_param = launch.actions.ExecuteProcess(
    #     cmd=['xacro', default_model_path, 'prefix:=mover6', '-o', processed_model_path],
    #     output='screen'
    # )

    # Rviz config file
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config_mover6.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="mover6_robot_state_publisher",
        namespace='mover6',
        # remappings=[('/joint_states', 'mover6/joint_states')],
        
        parameters=[{'robot_description': Command(['xacro ', default_model_path])},
                    {'frame_prefix': 'mover6/'}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='mover6_joint_state_publisher',
        # arguments=[default_model_path],
        namespace='mover6',
        # remappings=[('/joint_states', 'mover6/joint_states')],
        parameters=[{'source_list': ['mover6_joint_states']},
                    {'robot_description': Command(['xacro ', default_model_path])}],
    )
    # Commented out for Gazebo simulation
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    mover6_node = launch_ros.actions.Node(
        package='mover6_sim',
        executable='mover6',
        namespace='mover6',
        name='mover6',
        output='screen',
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration("run_rviz")),
    )


    return launch.LaunchDescription([
        # Commented out for Gazebo simulation
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='run_rviz', default_value='False',
                                             description="whether or not this launch file runs RVIZ2"),
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        # load_param,
        mover6_node,
        joint_state_publisher_node, # Commented out because gazebo plugin publishes joint states
        robot_state_publisher_node,
        rviz_node,
    ])

