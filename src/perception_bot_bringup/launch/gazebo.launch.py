"""Launch Gazebo Harmonic with the PerceptionBOT sorting world and Franka FR3."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_desc = get_package_share_directory("perception_bot_description")
    pkg_franka_desc = get_package_share_directory("franka_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_file = os.path.join(pkg_desc, "worlds", "sorting_world.sdf")

    # Gazebo needs to find franka_description meshes via model://franka_description/...
    # GZ_SIM_RESOURCE_PATH must contain the *parent* of the franka_description directory.
    franka_desc_parent = os.path.dirname(pkg_franka_desc)

    # Process xacro → URDF string
    xacro_file = os.path.join(pkg_desc, "urdf", "sorting_cell.urdf.xacro")
    robot_description_raw = xacro.process_file(
        xacro_file,
        mappings={
            "hand": "true",
            "gazebo": "true",
            "ros2_control": "true",
            "xyz": "0 0 0.81",  # arm base at table height (0.81m above world origin)
        },
    ).toxml()

    # Set Gazebo resource path so it can find franka_description meshes
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=franka_desc_parent,
    )

    # Launch Gazebo Harmonic with the sorting world
    # gz_ros2_control plugin inside the URDF creates the controller_manager
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Robot state publisher — publishes TF from URDF joints
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw}],
    )

    # Spawn the FR3 URDF into Gazebo (triggers gz_ros2_control plugin → creates controller_manager)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "sorting_cell",
            "-topic", "robot_description",
        ],
        output="screen",
    )

    # ros_gz_bridge: clock + camera topics
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

    # Controller spawners — wait for controller_manager (created by Gazebo plugin)
    # 10s delay gives Gazebo time to start and create the controller_manager
    joint_state_broadcaster_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    arm_controller_spawner = TimerAction(
        period=13.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "fr3_arm_controller",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        gz_bridge,
        TimerAction(period=3.0, actions=[spawn_robot]),
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
