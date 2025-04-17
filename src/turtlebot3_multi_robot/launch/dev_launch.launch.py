import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

def generate_launch_description():

    ld = LaunchDescription()
    world = PathJoinSubstitution([
        FindPackageShare('turtlebot3_multi_robot'),
        'worlds',
        'multi_empty_world.world'
    ])
    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args' : {world}}.items(),
    )

    ld.add_action(gz_sim_cmd)
    return ld