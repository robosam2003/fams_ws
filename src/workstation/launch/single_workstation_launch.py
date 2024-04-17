import launch
from launch.actions import IncludeLaunchDescription
from launch import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
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
    # A workstation consists of the mover6 nodes and the workstation node
    # This launch file will be run on the raspberry pi
    # ----------- MOVER6 ------------
    workstation_name = LaunchConfiguration('workstation_name')
    workstation_name_str = workstation_name.perform(context) # A string

    mover6_description_pkg_share = launch_ros.substitutions.FindPackageShare(package='mover6_description').find('mover6_description')
    mover6_pkg_share = launch_ros.substitutions.FindPackageShare(package='mover6').find('mover6')

    # Robot URDF file
    default_model_path = os.path.join(mover6_description_pkg_share, 'src/description/CPRMover6WithGripper.urdf.xacro')

    # Robot State Publisher for RVIZ
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="mover6_robot_state_publisher",
        namespace=workstation_name,        
        parameters=[{'robot_description': Command(['xacro ', default_model_path])},
                    {'frame_prefix': PathJoinSubstitution([workstation_name_str, ''])}]
    )

    # Joint State publisher - for the rviz simulation, also needed by robot_state_publisher?
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='mover6_joint_state_publisher',
        namespace=workstation_name,
        parameters=[{'source_list': ['mover6_joint_states']},
                    {'robot_description': Command(['xacro ', default_model_path])}],
    )



    # ----------- Workstation ------------
    workstation_pkg_share = launch_ros.substitutions.FindPackageShare(package='workstation').find('workstation')



    return [
        ]


def generate_launch_description():
    return launch.LaunchDescription(
        DeclareLaunchArgument(
            'workstation_name',
            default_value='workstationX',
            description='Name of the workstation'
        ),
        DeclareLaunchArgument(
            'Initial_pase_link_pos',
            default_value='0 0 0',
            description='Initial position of the base link, x, y, yaw'
        ),

    )