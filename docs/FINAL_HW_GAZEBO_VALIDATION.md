# Final HW Gazebo Validation

## 1. 목적

Final HW 모델 migration 이후 다음 전체 실행 경로의 일관성을 검증한다.

```text
Canonical URDF
→ MoveIt2
→ Controller
→ gz_ros2_control
→ Gazebo
→ Joint State Feedback
```

기능 구현 기준 commit:

```text
1601366 feat(gazebo): migrate simulation to final HW model
```

---

## 2. Final Model 기준

```text
Robot: ASOW
Root: base_link

Left:
left_joint1
left_joint2
left_joint3
left_joint4
left_joint5

Right:
right_joint1
right_joint2
right_joint3
right_joint4
right_joint5

Tool:
left_tool0
right_tool0
```

Canonical Description:

```text
src/asow_hw_description/urdf/asow_robot.urdf
```

Gazebo Description:

```text
src/asow_gazebo/urdf/asow_gazebo.urdf.xacro
```

Gazebo simulation 전용 TF:

```text
world
└── world_fixed_joint
    └── base_link
        z = 0.15 m
```

MoveIt Model Frame:

```text
base_link
```

RViz Fixed Frame:

```text
base_link
```

---

## 3. Static Validation

검증 항목:

- Xacro generation
- URDF parsing
- 10 Revolute Joint 이름
- 10 Revolute Joint 순서
- Gazebo ros2_control Joint 이름
- Controller Joint 이름 및 순서
- Canonical URDF와 Gazebo Position Limit 일치
- `git diff --check`

Final Position Limit:

```text
left_joint1   -0.523599 ~ +1.570796
left_joint2   -1.570796 ~ +1.570796
left_joint3   -1.570796 ~ +1.570796
left_joint4   -1.570796 ~ +1.570796
left_joint5   -1.570796 ~ +1.570796

right_joint1  -1.570796 ~ +0.523599
right_joint2  -1.570796 ~ +1.570796
right_joint3  -1.570796 ~ +1.570796
right_joint4  -1.570796 ~ +1.570796
right_joint5  -1.570796 ~ +1.570796
```

결과:

```text
PASS
```

---

## 4. Gazebo Standalone

Hardware:

```text
GazeboSimSystem
state = active
```

Command Interface:

```text
left_joint1~5/position
right_joint1~5/position
```

10개 모두:

```text
available
claimed
```

Controller:

```text
joint_state_broadcaster active
left_arm_controller     active
right_arm_controller    active
```

직접 FollowJointTrajectory Goal을 사용하여 좌우 팔의 명령 전달과 `/joint_states` Feedback을 확인하였다.

결과:

```text
PASS
```

---

## 5. MoveIt2–Gazebo Integration

통합 실행:

```bash
ros2 launch \
  asow_gazebo \
  asow_moveit_gazebo.launch.py
```

검증 시나리오:

```text
left_arm  Random Valid → Plan & Execute
right_arm Random Valid → Plan & Execute
both_arms Home         → Plan & Execute
```

Left Arm:

```text
sending trajectory to left_arm_controller
Accepted new action goal
Goal reached, success
Controller successfully finished
Completed trajectory execution with status SUCCEEDED
Solution was found and executed
```

Right Arm:

```text
sending trajectory to right_arm_controller
Accepted new action goal
Goal reached, success
Controller successfully finished
Completed trajectory execution with status SUCCEEDED
Solution was found and executed
```

Both Arms:

```text
left_arm_controller goal accepted
right_arm_controller goal accepted

left_arm_controller goal reached
right_arm_controller goal reached

Completed trajectory execution with status SUCCEEDED
Solution was found and executed
```

결과:

```text
PASS
```

---

## 6. Frame Consistency

최종 MoveIt RobotModel:

```text
root = base_link
```

최종 RViz MoveIt RobotModel:

```text
root = base_link
```

Gazebo RobotModel:

```text
root = world

world
└── world_fixed_joint
    └── base_link
```

Gazebo의 `world → base_link`는 simulation TF에서만 담당한다.

MoveIt과 RViz는 `base_link`를 RobotModel 기준 frame으로 사용한다.

통합 실행 중 다음 Frame Error:

```text
Given transform is to frame 'base_link',
but frame 'world' was expected.
```

최종 regression 결과:

```text
0건
```

Standalone MoveIt FakeSystem에서도 frame 관련 오류가 발생하지 않음을 확인하였다.

결과:

```text
PASS
```

---

## 7. Exact Tool0 IK Regression

Left:

```text
group = left_arm
tip = left_tool0
base = base_link
RC = 0
```

Right:

```text
group = right_arm
tip = right_tool0
base = base_link
RC = 0
```

현재 Tool0 Pose에 대한 Exact IK가 좌우 모두 성공하였다.

5DOF 팔이므로 임의의 6D Cartesian Pose에 대한 해가 항상 존재하는 것은 아니다.

결과:

```text
PASS
```

---

## 8. Self-Collision Regression

Deterministic Sample:

```text
home                     1
single_joint_extreme     20
all_joint_extreme      1024
left_uniform           1000
right_uniform          1000
both_uniform           5000
boundary_stress        2000
```

Total:

```text
10045
```

최종 결과:

```text
tested:               10045
valid:                 4371
collision:             5674
noncollision_invalid:  0
service_failure:       0
collision detection:   56.49%
RC:                    0
```

Final HW migration 이전 Phase 3B deterministic baseline과 동일한 결과를 확인하였다.

56.49%는 정의된 sampling distribution에서 collision으로 판정된 비율이며 실제 운용 중 충돌 발생률을 의미하지 않는다.

결과:

```text
PASS
```

---

## 9. MoveIt FakeSystem Regression

실행:

```bash
ros2 launch \
  asow_moveit_config \
  demo.launch.py
```

Controller:

```text
left_arm_controller     active
right_arm_controller    active
joint_state_broadcaster active
```

RViz RobotModel:

```text
ASOW
```

Frame 관련 오류:

```text
0건
```

결과:

```text
PASS
```

---

## 10. Build Regression

실행 위치:

```text
~/asow_ws
```

명령:

```bash
colcon build \
  --symlink-install \
  --packages-select \
  asow_hw_description \
  asow_moveit_config \
  asow_gazebo
```

결과:

```text
Summary: 3 packages finished
```

Build는 반드시 Workspace root인 다음 위치에서 실행한다.

```text
~/asow_ws
```

결과:

```text
PASS
```

---

## 11. 최종 검증 상태

```text
Canonical Model            PASS
MoveIt Semantic Model      PASS
Gazebo Model               PASS
GazeboSimSystem            PASS
ros2_control Controller    PASS
Joint State Feedback       PASS

left_arm Execution         PASS
right_arm Execution        PASS
both_arms Execution        PASS

MoveIt / RViz Frame        PASS

Left Exact IK              PASS
Right Exact IK             PASS

Self-Collision Regression  PASS
MoveIt FakeSystem          PASS
3-package Build            PASS
```

Final HW 기반 MoveIt2–Gazebo Simulation 경로 검증 완료.

다음 개발 단계:

```text
Axis 3
→ Dynamixel Hardware Interface
→ Actual Joint State Feedback
→ Actual Trajectory Execution
```
