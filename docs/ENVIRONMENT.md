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
- Dynamixel: DYNAMIXEL Shield + Dynamixel2Arduino (Axis 3 Arduino Bridge)
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
- asow_dynamixel

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

## Axis 3 Arduino / Dynamixel 환경

실제 Dynamixel Hardware 연동용 Arduino Toolchain:

- Arduino CLI: 1.5.1
- Arduino AVR Core: 1.8.8
- Target Board: Arduino UNO
- Board FQBN: `arduino:avr:uno`
- Dynamixel2Arduino: 0.8.1
- SoftwareSerial: 1.0
- Dynamixel Protocol: 1.0
- Pi-Arduino Serial: 115200 bps (Hardware 검증 전 가정)
- Dynamixel Bus: 1000000 bps (Hardware 검증 전 가정)

Arduino 의존성 재현:

    arduino-cli core install arduino:avr@1.8.8
    arduino-cli lib install "Dynamixel2Arduino@0.8.1"

`SoftwareSerial 1.0`은 Arduino AVR Core에 포함된다.

Arduino Firmware:

    src/asow_dynamixel/firmware/asow_dynamixel_bridge/asow_dynamixel_bridge.ino

Firmware Compile:

    cd ~/asow_ws

    arduino-cli compile \
      --fqbn arduino:avr:uno \
      src/asow_dynamixel/firmware/asow_dynamixel_bridge

현재 검증 결과:

    Sketch uses 19484 bytes (60%) of program storage space.
    Global variables use 923 bytes (45%) of dynamic memory.

현재 Axis 3 Hardware 통신 전제:

    Raspberry Pi
    → USB-to-TTL UART
    → Arduino D7/D8 SoftwareSerial
    → Arduino UNO
    → D0/D1 + Direction
    → ROBOTIS DYNAMIXEL Shield
    → AX-12A / AX-18A

Pi ↔ Arduino 통신은 D7/D8 SoftwareSerial을 사용하고,
Arduino ↔ Dynamixel 통신은 DYNAMIXEL Shield의 Hardware UART 경로를 사용한다.

실제 Motor ID, Baudrate, Direction, Zero Offset, Joint Mapping은
Hardware 연결 후 검증하여 확정한다.
