from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
    add_debuggable_node,
)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "ASOW",
            package_name="asow_moveit_config",
        )
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(
                moveit_config.package_path
                / "config"
                / "moveit.rviz"
            ),
        )
    )

    # RViz의 MoveIt plugin이 /robot_description topic으로
    # fallback하지 않도록 MoveIt 전용 RobotModel을 명시적으로 전달한다.
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config"),
        ],
        parameters=rviz_parameters,
    )

    return ld
