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

    subscriber_node = Node(
        package='project_two',  
        executable='node_img_subscriber.py',  # writes camera img to file and processes it to contours.csv
        output='screen',
        name='image_subscriber_node',
        parameters=[{'use_sim_time': True}],
    )

    ik_node = Node(
        package='project_two',  
        executable='inverse_kinematics.py',
        output='screen',
        name='inverse_kinematics_node',
        parameters=[{'use_sim_time': True}],
    )

    # Delay start of ik after img process
    delay_ik_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=subscriber_node,
            on_exit=[ik_node],
        )
    )

    path_publisher_node = Node(
        package='project_two',  
        executable='path_publisher.py',  
        output='screen',
        name='path_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # Delay start of path publisher after ik
    delay_path_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ik_node,
            on_exit=[path_publisher_node],
        )
    )
    
    joint_publisher_node = Node(
        package='project_two',  
        executable='joint_angle_publisher.py',  # commands end effector
        output='screen',
        name='joint_angle_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # Delay start of joint publisher after ik
    delay_joint_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_publisher_node,
            on_exit=[ik_node],
        )
    )
    
    # Launch Description 
    return LaunchDescription([
        subscriber_node,
        delay_ik_node,
        ik_node,
        delay_path_publisher,
        path_publisher_node,
        delay_joint_publisher,
        joint_publisher_node,
        
    ])
