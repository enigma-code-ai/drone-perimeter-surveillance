#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_quadcopter = FindPackageShare("quadcopter_simulation")

    # Launch arguments
    world_name = DeclareLaunchArgument(
        "world_name",
        default_value="drone_world.sdf",
        description="Name of the world to load",
    )

    # Paths
    world_file = PathJoinSubstitution(
        [pkg_quadcopter, "worlds", LaunchConfiguration("world_name")]
    )

    urdf_file = PathJoinSubstitution([pkg_quadcopter, "urdf", "quadcopter.urdf.xacro"])

    # Robot description using xacro
    robot_description = Command(["xacro ", urdf_file])

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, {"use_sim_time": True}],
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Gazebo Server (gz sim)
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-v4", world_file],
        name="gazebo_server",
        output="screen",
    )

    # Gazebo Client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=["gz", "sim", "-g", "-v4"],
        name="gazebo_client",
        output="screen",
        condition=IfCondition(PythonExpression(["'true' == 'true'"])),
    )

    # Spawn robot
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_quadcopter",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "quadcopter",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "2.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        output="screen",
        respawn=False,
        parameters=[{"use_sim_time": True}],
    )

    # Clock bridge
    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # IMU bridge with correct topic path (fixed joint collapses to base_link)
    bridge_imu = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="imu_bridge",
        arguments=[
            "/world/drone_world/model/quadcopter/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            (
                "/world/drone_world/model/quadcopter/link/base_link/sensor/imu_sensor/imu",
                "/drone/imu/data",
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Pose bridge with correct message type
    bridge_pose = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pose_bridge",
        arguments=["/model/quadcopter/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        remappings=[("/model/quadcopter/pose", "/tf")],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Command velocity bridge for direct control
    bridge_cmd_vel = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="cmd_vel_bridge",
        arguments=["/model/quadcopter/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
        remappings=[("/model/quadcopter/cmd_vel", "/drone/cmd_vel")],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )  # Python drone controller
    drone_controller = Node(
        package="quadcopter_simulation",
        executable="drone_controller.py",
        name="drone_controller",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Motor command bridges - connect ROS motor commands to Gazebo motor speeds
    bridge_motor_0 = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="motor_0_bridge",
        arguments=[
            "/model/quadcopter/motor_speed_0@std_msgs/msg/Float64[gz.msgs.Double"
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    bridge_motor_1 = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="motor_1_bridge",
        arguments=[
            "/model/quadcopter/motor_speed_1@std_msgs/msg/Float64[gz.msgs.Double"
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    bridge_motor_2 = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="motor_2_bridge",
        arguments=[
            "/model/quadcopter/motor_speed_2@std_msgs/msg/Float64[gz.msgs.Double"
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    bridge_motor_3 = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="motor_3_bridge",
        arguments=[
            "/model/quadcopter/motor_speed_3@std_msgs/msg/Float64[gz.msgs.Double"
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            world_name,
            robot_state_publisher,
            joint_state_publisher,
            gazebo_server,
            gazebo_client,
            TimerAction(period=2.0, actions=[spawn_entity]),
            TimerAction(
                period=3.0,
                actions=[
                    bridge_clock,
                    bridge_imu,
                    bridge_pose,
                    bridge_cmd_vel,
                    bridge_motor_0,
                    bridge_motor_1,
                    bridge_motor_2,
                    bridge_motor_3,
                ],
            ),
            TimerAction(period=4.0, actions=[drone_controller]),
        ]
    )
