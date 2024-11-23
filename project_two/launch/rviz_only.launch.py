#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import  PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

     # RVIZ Configuration
    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("project_two"), "rviz", "display_proj2.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])

    

    # Launch Description 
    return LaunchDescription([
        rviz_node
        
    ])