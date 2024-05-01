from launch import LaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import PushRosNamespace
import os
 
def generate_launch_description():
    
    

    maincam=Node(
        package='machine_vis',
        executable='marker',
        name='maincam'
        launch.arguments=['camindex': 0]
        
        )
    
    workcam1=Node(
        package='machine_vis',
        executable='marker',
        name='workcam1'
    )
    workcam2=Node(
        package='machine_vis',
        executable='marker',
        name='workcam2'
    )
    return LaunchDescription([
        # d
    ])