import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import SetParameter


def include_launch(package_name, launch_file):
    package_share = get_package_share_directory(
        package_name
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                launch_file,
            )
        )
    )


def generate_launch_description():
    # Gazebo, robot_state_publisher, ros2_control,
    # robot spawn 및 좌우 Controller 실행
    gazebo = include_launch(
        "asow_gazebo",
        "asow_gazebo.launch.py",
    )

    # MoveIt도 Gazebo의 /clock을 사용
    move_group = TimerAction(
        period=10.0,
        actions=[
            GroupAction(
                [
                    SetParameter(
                        name="use_sim_time",
                        value=True,
                    ),
                    include_launch(
                        "asow_moveit_config",
                        "move_group.launch.py",
                    ),
                ]
            )
        ],
    )

    # RViz도 동일한 simulation clock 사용
    moveit_rviz = TimerAction(
        period=12.0,
        actions=[
            GroupAction(
                [
                    SetParameter(
                        name="use_sim_time",
                        value=True,
                    ),
                    include_launch(
                        "asow_moveit_config",
                        "moveit_rviz.launch.py",
                    ),
                ]
            )
        ],
    )

    return LaunchDescription(
        [
            gazebo,
            move_group,
            moveit_rviz,
        ]
    )
