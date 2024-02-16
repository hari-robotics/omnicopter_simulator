#! /usr/bin/python3
from launch import LaunchDescription
from launch.actions import  ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os
import xacro

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    omc_description_path = os.path.join(
        get_package_share_directory('omc_description')
    )

    xacro_file = os.path.join(
        omc_description_path, 'urdf', 'omc.xacro'
    )
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'omc',
                                   "-x", "0.0", "-y", "0.0", "-z", "0.5"],
                        output='screen'
    )
        
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_velocity_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])