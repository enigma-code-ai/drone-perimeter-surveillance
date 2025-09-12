#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package paths
    pkg_quadcopter = FindPackageShare('quadcopter_simulation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    headless = LaunchConfiguration('headless', default='false')
    
    # Paths
    urdf_file = os.path.join(
        get_package_share_directory('quadcopter_simulation'),
        'urdf',
        'quadcopter.urdf'
    )
    
    world_file = os.path.join(
        get_package_share_directory('quadcopter_simulation'),
        'worlds',
        'drone_world.sdf'
    )
    
    rviz_config = os.path.join(
        get_package_share_directory('quadcopter_simulation'),
        'rviz',
        'drone_config.rviz'
    )
    
    # Read URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Start Gazebo with or without GUI
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen',
        condition=UnlessCondition(headless)
    )
    
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', '-s', world_file],
        output='screen',
        condition=IfCondition(headless)
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 50
        }]
    )
    
    # Spawn the drone in Gazebo
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'quadcopter',
            '-string', robot_description,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # Bridge for clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Bridge for IMU
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/drone_world/model/quadcopter/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        remappings=[('/world/drone_world/model/quadcopter/link/imu_link/sensor/imu_sensor/imu', '/drone/imu/data')],
        output='screen'
    )
    
    # Bridge for cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/quadcopter/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        remappings=[('/model/quadcopter/cmd_vel', '/cmd_vel')],
        output='screen'
    )
    
    # Bridge for pose (using world state)
    bridge_pose = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/drone_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/world/drone_world/pose/info', '/tf')],
        output='screen'
    )
    
    # TF publisher for world to base_link
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen'
    )
    
    # Drone controller
    drone_controller = Node(
        package='quadcopter_simulation',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mass': 1.5,
            'arm_length': 0.15,
            'thrust_coefficient': 8.54858e-06,
            'max_thrust': 15.0
        }]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        
        # Launch Gazebo
        gazebo_gui,
        gazebo_headless,
        
        # Launch robot description nodes
        robot_state_publisher,
        joint_state_publisher,
        
        # Static transforms
        static_tf_world,
        
        # Delayed actions to ensure Gazebo is ready
        TimerAction(period=2.0, actions=[spawn_drone]),
        TimerAction(period=3.0, actions=[bridge_clock, bridge_imu, bridge_cmd_vel, bridge_pose]),
        TimerAction(period=5.0, actions=[drone_controller]),
        TimerAction(period=2.0, actions=[rviz])
    ])