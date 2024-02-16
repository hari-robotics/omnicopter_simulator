#! /usr/bin/python3
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import os
import xacro

def generate_launch_description():
    package_name = 'omc_description'
    package_path = get_package_share_directory(package_name)
    
    robot_xacro = os.path.join(package_path, 'urdf', 'omc.xacro')
    robot_description = xacro.process_file(robot_xacro).toxml()

    # Launch scripts
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true',
            'use_sim_time': 'true',
        }.items()
    )

    # Creating nodes
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'omc',
            '-topic', 'robot_description',
            "-x", "0.0", "-y", "0.0", "-z", "0.5"
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo,
        urdf_spawn_node,
    ])
