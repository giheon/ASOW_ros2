from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def controller_spawner(controller_name, delay):
    return TimerAction(
        period=delay,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                name=f"spawner_{controller_name}",
                output="screen",
                arguments=[
                    controller_name,
                    "-c",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
            )
        ],
    )


def generate_launch_description():
    gazebo_package = FindPackageShare("asow_gazebo")

    hw_resource_root = os.path.dirname(
        get_package_share_directory(
            "asow_hw_description"
        )
    )

    gazebo_resource_root = os.path.dirname(
        get_package_share_directory(
            "asow_gazebo"
        )
    )

    append_gz_hw_resources = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=hw_resource_root,
    )

    append_gz_project_resources = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gazebo_resource_root,
    )

    append_ign_hw_resources = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=hw_resource_root,
    )

    append_ign_project_resources = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=gazebo_resource_root,
    )

    world_file = PathJoinSubstitution(
        [
            gazebo_package,
            "worlds",
            "asow_empty.sdf",
        ]
    )

    robot_xacro = PathJoinSubstitution(
        [
            gazebo_package,
            "urdf",
            "robot_a_gazebo.urdf.xacro",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    robot_xacro,
                ]
            ),
            value_type=str,
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r -v 4 ",
                world_file,
            ]
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": True,
            },
        ],
    )


    # Gazebo simulation clock을 ROS2 /clock으로 전달
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_robot_a",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "robot_a",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.0",
                ],
            )
        ],
    )

    joint_state_broadcaster = controller_spawner(
        "joint_state_broadcaster",
        6.0,
    )

    left_arm_controller = controller_spawner(
        "left_arm_controller",
        7.0,
    )

    right_arm_controller = controller_spawner(
        "right_arm_controller",
        8.0,
    )

    return LaunchDescription(
        [
            append_gz_hw_resources,
            append_gz_project_resources,
            append_ign_hw_resources,
            append_ign_project_resources,
            gazebo,
            clock_bridge,
            robot_state_publisher,
            spawn_robot,
            joint_state_broadcaster,
            left_arm_controller,
            right_arm_controller,
        ]
    )
