# ASOW ROS2 팀 표준 환경

## 표준 환경

- OS: Ubuntu 22.04 LTS Desktop 64-bit
- ROS: ROS 2 Humble Hawksbill
- Python: Python 3.10.x
- Gazebo: Gazebo Fortress LTS
- ROS-Gazebo 연동: ros-humble-ros-gz
- RViz: RViz2 Humble
- MoveIt: MoveIt2 Humble
- Build: colcon, rosdep, vcstool
- Control: ros2_control, ros2_controllers
- Dynamixel: Dynamixel SDK, dynamixel_hardware_interface
- IDE: VSCode

## Workspace

작업 경로:

    ~/asow_ws

Workspace 구조:

    asow_ws/
    ├── src/
    ├── build/
    ├── install/
    └── log/

직접 수정하는 위치:

    ~/asow_ws/src

## 공유 대상 패키지

- asow_hw_description
- asow_moveit_config
- asow_gazebo

## 실행 모드

URDF 및 RViz 검증:

    ros2 launch asow_hw_description display.launch.py

MoveIt2 FakeSystem:

    ros2 launch asow_moveit_config demo.launch.py

MoveIt2-Gazebo 통합:

    ros2 launch asow_gazebo asow_moveit_gazebo.launch.py

주의:

demo.launch.py와 asow_moveit_gazebo.launch.py는 동시에 실행하지 않는다.

## 환경 상세 파일

- system-info.txt
- ros2-package-list.txt
- installed-packages.txt
- pip-freeze.txt
