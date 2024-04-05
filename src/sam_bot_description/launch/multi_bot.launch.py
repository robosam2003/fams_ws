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

    default_rviz_config_path = os.path.join(sam_bot_description_pkg_share, 'rviz/urdf_config.rviz')

    sam_bot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sam_bot_launch_file)
    )
    
    mover6_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mover6_launch_file),
    )

    # sam_bot_launch_with_namespace = launch.actions.GroupAction(
    #     actions=[
    #         PushRosNamespace('nexus'),
    #         sam_bot_launch,
    #     ]
    # )

    # mover6_launch_with_namespace = launch.actions.GroupAction(
    #     actions=[
    #         PushRosNamespace('mover6'),
    #         mover6_launch,
    #     ]
    # )
      
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    # This is just a simulation thing for now, in reality, every robot will publish it's own tf
    tf_broadcaster = launch_ros.actions.Node(
        package='tf_broadcast',
        executable='map_odom_publisher',
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        tf_broadcaster,
        sam_bot_launch,
        mover6_launch,
        rviz_node,
    ])