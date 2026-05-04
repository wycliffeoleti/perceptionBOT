"""Launch MoveIt 2 move_group with the PerceptionBOT MoveIt config.

Assumes Gazebo + ros2_control are launched separately (see
perception_bot_bringup/launch/gazebo.launch.py) so the controllers exist
and joint_states are flowing.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def _load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_desc = get_package_share_directory("perception_bot_description")
    pkg_manip = get_package_share_directory("perception_bot_manipulation")

    # robot_description (URDF)
    xacro_file = os.path.join(pkg_desc, "urdf", "sorting_cell.urdf.xacro")
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={
            "hand": "true",
            "gazebo": "true",
            "ros2_control": "true",
            "xyz": "0 0 0.81",
        },
    ).toxml()

    # robot_description_semantic (SRDF)
    srdf_file = os.path.join(pkg_manip, "config", "fr3.srdf")
    with open(srdf_file, "r") as f:
        robot_description_semantic_xml = f.read()

    # kinematics, joint limits, ompl, controllers
    kinematics_yaml = _load_yaml(
        os.path.join(pkg_manip, "config", "kinematics.yaml"))
    joint_limits_yaml = _load_yaml(
        os.path.join(pkg_manip, "config", "joint_limits.yaml"))
    ompl_yaml = _load_yaml(
        os.path.join(pkg_manip, "config", "ompl_planning.yaml"))
    pilz_yaml = _load_yaml(
        os.path.join(pkg_manip, "config", "pilz_planning.yaml"))
    moveit_controllers_yaml = _load_yaml(
        os.path.join(pkg_manip, "config", "moveit_controllers.yaml"))
    move_group_yaml = _load_yaml(
        os.path.join(pkg_manip, "config", "move_group.yaml"))

    move_group_params = [
        {
            "robot_description": robot_description_xml,
            "robot_description_semantic": robot_description_semantic_xml,
            "robot_description_kinematics": kinematics_yaml,
            "robot_description_planning": joint_limits_yaml,
            "ompl": ompl_yaml,
            "pilz_industrial_motion_planner": pilz_yaml,
            "use_sim_time": True,
        },
        moveit_controllers_yaml,
        move_group_yaml,
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=move_group_params,
    )

    return LaunchDescription([move_group])
