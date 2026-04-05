"""Launch Gazebo Harmonic with the PerceptionBOT sorting world and Franka FR3."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg_desc = get_package_share_directory("perception_bot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_file = os.path.join(pkg_desc, "worlds", "sorting_world.sdf")
    controllers_file = os.path.join(pkg_desc, "config", "fr3_controllers.yaml")

    # Process xacro to get robot URDF
    xacro_file = os.path.join(pkg_desc, "urdf", "sorting_cell.urdf.xacro")
    robot_description_raw = xacro.process_file(
        xacro_file,
        mappings={
            "hand": "true",
            "gazebo": "true",
            "ros2_control": "true",
        },
    ).toxml()

    # Launch Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw}],
    )

    # Spawn FR3 into Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "fr3",
            "-topic", "robot_description",
        ],
        output="screen",
    )

    # ros_gz_bridge for joint states and clock
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    # ros2_control manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_raw},
            controllers_file,
        ],
        output="screen",
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn arm trajectory controller (after joint_state_broadcaster)
    arm_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["fr3_arm_controller", "--controller-manager", "/controller_manager"],
            )
        ],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        gz_bridge,
        controller_manager,
        TimerAction(period=4.0, actions=[joint_state_broadcaster_spawner]),
        arm_controller_spawner,
    ])
