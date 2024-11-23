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

    # Get Gazebo ROS interface package
    pkg_proj_two   = get_package_share_directory('project_two')

    # Remove old joint trajectory file (fail-safe)
    old_joint_cmds = os.path.join(pkg_proj_two, 'csv', 'joint_angles.csv')
    if os.path.exists(old_joint_cmds) and os.path.isfile(old_joint_cmds):
        os.remove(old_joint_cmds)

    subscriber_node = Node(
        package='project_two',  
        executable='node_img_subscriber.py',  # writes camera img to file and processes it to contours.csv
        output='screen',
        name='image_subscriber_node',
        parameters=[{'use_sim_time': True}],
    )
    
    publisher_node = Node(
        package='project_two',  
        executable='joint_angle_publisher.py',  # commands end effector
        output='screen',
        name='joint_angle_publisher',
        parameters=[{'use_sim_time': True}],
    )
    
    
    # Launch Description 
    return LaunchDescription([
        subscriber_node,
        publisher_node
        
    ])