#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package paths
    pkg_quadcopter = FindPackageShare('quadcopter_simulation')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Files
    urdf_file = PathJoinSubstitution([pkg_quadcopter, 'urdf', 'quadcopter.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_quadcopter, 'rviz', 'drone_config.rviz'])
    world_file = PathJoinSubstitution([pkg_quadcopter, 'worlds', 'drone_world.world'])
    controller_config = PathJoinSubstitution([pkg_quadcopter, 'config', 'drone_controllers.yaml'])
    
    # Robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                  '-entity', 'quadcopter',
                  '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'use_sim_time': use_sim_time}],
        output='both'
    )
    
    # Load controllers
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    drone_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drone_controller'],
        output='screen'
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )
    
    # Drone controller node
    drone_control_node = Node(
        package='quadcopter_simulation',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Teleop node
    teleop_node = Node(
        package='quadcopter_simulation',
        executable='drone_teleop.py',
        name='drone_teleop',
        output='screen'
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        
        # Launch nodes
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        TimerAction(period=2.0, actions=[spawn_robot]),
        TimerAction(period=4.0, actions=[controller_manager]),
        TimerAction(period=6.0, actions=[joint_state_broadcaster, drone_controller]),
        TimerAction(period=8.0, actions=[drone_control_node]),
        TimerAction(period=2.0, actions=[rviz]),
        TimerAction(period=10.0, actions=[teleop_node])
    ])