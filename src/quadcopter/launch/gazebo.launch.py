import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('quadcopter')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'quadcopter.urdf')
    
    # Path to controllers config file
    controllers_config = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Set the GZ_SIM_RESOURCE_PATH environment variable
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join(pkg_dir, '..')
    
    # Include Gazebo launch file (using ros_gz_sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}, {'use_sim_time': True}],
    )

    # Spawn robot in Gazebo (using ros_gz_sim)
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'quadcopter'],
    )

    # Static transform publisher for base_footprint (already exists as root in URDF)
    # This is now redundant since base_footprint is the root link
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_footprint_tf',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    # )

    # Bridge for camera topics
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
    )

    # Bridge for IMU topics
    imu_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
    )

    # Bridge for joint states from Gazebo to ROS
    joint_state_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        output='screen',
        arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
    )

    # Joint states are published by gz-sim JointStatePublisher system; no bridge needed

    # RViz configuration file path
    rviz_config_file = os.path.join(pkg_dir, 'config', 'quadcopter_view.rviz')
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': True}],
    )


    # Fallback joint state publisher (disabled; using Gazebo joint state bridge)

    # Load controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_propeller_1_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'propeller_1_controller'],
        output='screen'
    )

    load_propeller_2_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'propeller_2_controller'],
        output='screen'
    )

    load_propeller_3_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'propeller_3_controller'],
        output='screen'
    )

    load_propeller_4_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'propeller_4_controller'],
        output='screen'
    )

    # Controller manager node with configuration (disabled for now)
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[controllers_config, {'use_sim_time': True}],
    #     output='screen'
    # )

    # Controller manager spawner to start all controllers (disabled for now)
    # controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', 'propeller_1_controller', 
    #               'propeller_2_controller', 'propeller_3_controller', 'propeller_4_controller'],
    #     output='screen'
    # )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        bridge_node,
        joint_state_bridge_node,
        imu_bridge_node,
        rviz_node,
    ])