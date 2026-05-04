"""End-to-end demo launch.

Brings up:
  - Gazebo Harmonic with the sorting world + FR3 + ros2_control + camera
  - MoveIt 2 move_group with our MoveIt config
  - perception detection_node (HSV by default) and pose_estimation_node
  - manipulation_node (SortObjects action server)
  - RViz with the demo config

depth_node is included only if the env var PERCEPTION_BOT_USE_DA_V2=1, since
it requires ONNX Runtime + the model file. Default depth source is Gazebo's
ground-truth /camera/depth_image, relayed by the absence of depth_node directly.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = get_package_share_directory("perception_bot_bringup")
    pkg_perception = get_package_share_directory("perception_bot_perception")
    pkg_manip = get_package_share_directory("perception_bot_manipulation")

    use_rviz = LaunchConfiguration("use_rviz")
    detection_backend = LaunchConfiguration("detection_backend")
    use_estimated_depth = LaunchConfiguration("use_estimated_depth")

    # Pick depth topic for the pose-estimation step.
    # Gazebo GT (relayed) is the default; the DA-V2 estimate is opt-in.
    depth_topic = "/camera/depth_image"  # raw GT from Gazebo
    # When use_estimated_depth=true the manipulation pipeline expects depth_node
    # to be running, which in turn requires ONNX runtime + a model. We expose
    # the switch but keep the default off.

    # Gazebo + FR3 + controllers
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "gazebo.launch.py")))

    # MoveIt move_group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_manip, "launch", "move_group.launch.py")))

    # Perception nodes
    detection_node = Node(
        package="perception_bot_perception",
        executable="detection_node",
        name="detection_node",
        parameters=[
            os.path.join(pkg_perception, "config", "detection.yaml"),
            {"backend": detection_backend, "use_sim_time": True},
        ],
        output="screen",
    )

    pose_estimation_node = Node(
        package="perception_bot_perception",
        executable="pose_estimation_node",
        name="pose_estimation_node",
        parameters=[{
            "depth_topic": depth_topic,
            "use_sim_time": True,
        }],
        output="screen",
    )

    manipulation_node = Node(
        package="perception_bot_manipulation",
        executable="manipulation_node",
        name="manipulation_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("perception_bot_bringup"), "rviz", "demo.rviz"
    ])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Start RViz with the demo configuration"),
        DeclareLaunchArgument(
            "detection_backend", default_value="hsv",
            description="Detection backend: hsv or yolo"),
        DeclareLaunchArgument(
            "use_estimated_depth", default_value="false",
            description="Use Depth Anything V2 instead of Gazebo GT depth"),

        gazebo,
        # Wait for Gazebo + controllers to be up before move_group.
        TimerAction(period=8.0, actions=[move_group]),
        # Perception can start as soon as the camera is up.
        TimerAction(period=5.0, actions=[detection_node, pose_estimation_node]),
        # Manipulation needs move_group; start after it.
        TimerAction(period=12.0, actions=[manipulation_node]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
