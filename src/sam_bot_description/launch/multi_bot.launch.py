import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import PushRosNamespace
import os

"""  Build, source, launch command: 
colcon build --symlink-install --packages-select mover6 mover6_description sam_bot sam_bot_description tf_broadcast && source install/setup.bash  && ros2 launch sam_bot_description multi_bot.launch.py
"""

def generate_launch_description():
    sam_bot_description_pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    sam_bot_launch_file = os.path.join(sam_bot_description_pkg_share, 'launch', 'single_amr_launch.py')

    mover6_pkg_share = launch_ros.substitutions.FindPackageShare(package='mover6_description').find('mover6_description')
    mover6_launch_file = os.path.join(mover6_pkg_share, 'launch', 'mover6_launch.py')

    # GUI 
    gui_pkg_share = launch_ros.substitutions.FindPackageShare(package='gui').find('gui')

    # Scheduler
    scheduler_pkg_share = launch_ros.substitutions.FindPackageShare(package='scheduler').find('scheduler')

    # Fleet Controller
    fleet_controller_pkg_share = launch_ros.substitutions.FindPackageShare(package='fleet_controller').find('fleet_controller')

    # Workstation Controller
    workstation_controller_pkg_share = launch_ros.substitutions.FindPackageShare(package='workstation').find('workstation')

    default_rviz_config_path = os.path.join(sam_bot_description_pkg_share, 'rviz/urdf_config.rviz')



    sam_bot_launch1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sam_bot_launch_file),
        launch_arguments={
            'robot_name': 'nexus1',
            'initial_base_link_pos': "1.77 1.015 0"
        }.items()
    )

    gui_launch = launch_ros.actions.Node(
        package='gui',
        executable='gui',
        name='gui',
        output='screen',
    )

    scheduler_launch = launch_ros.actions.Node(
        package='scheduler',
        executable='scheduler',
        name='scheduler',
        output='screen',
    )

    fleet_controller_launch = launch_ros.actions.Node(
        package='fleet_controller',
        executable='fleet_controller',
        name='fleet_controller',
        output='screen',
    )

    workstation_controller_launch = launch_ros.actions.Node(
        package='workstation',
        executable='workstation_controller',
        name='workstation_controller',
        output='screen',
    )

    mover6_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mover6_launch_file),
        launch_arguments={
            'robot_name': 'mover61',
            'initial_base_link_pos': "3.31 2.20 -1.57079"
        }.items()
    )


      
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )




    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        sam_bot_launch1,
        gui_launch,
        scheduler_launch,
        fleet_controller_launch,
        mover6_launch, # This will run on the raspberry pi
        # workstation_controller_launch,
        rviz_node,
    ])