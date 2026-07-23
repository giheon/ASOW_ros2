#!/usr/bin/env python3

import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


class ComputeIKTest(Node):
    def __init__(self) -> None:
        super().__init__('compute_ik_test')

        self.declare_parameter('group_name', 'left_arm')
        self.declare_parameter('ik_link_name', 'left_tool0')
        self.declare_parameter('base_frame', 'base_link-v1')
        self.declare_parameter('dx', 0.0)
        self.declare_parameter('dy', 0.0)
        self.declare_parameter('dz', 0.0)
        self.declare_parameter('avoid_collisions', False)

        self.latest_joint_state = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
        )

    def joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        start = time.monotonic()

        while rclpy.ok() and time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_joint_state is not None:
                return True

        return False

    def get_current_tool_transform(
        self,
        base_frame: str,
        ik_link_name: str,
        timeout_sec: float = 5.0,
    ):
        start = time.monotonic()

        while rclpy.ok() and time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            try:
                return self.tf_buffer.lookup_transform(
                    base_frame,
                    ik_link_name,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
            except Exception:
                pass

        return None

    def run_test(self) -> int:
        group_name = str(self.get_parameter('group_name').value)
        ik_link_name = str(self.get_parameter('ik_link_name').value)
        base_frame = str(self.get_parameter('base_frame').value)

        dx = float(self.get_parameter('dx').value)
        dy = float(self.get_parameter('dy').value)
        dz = float(self.get_parameter('dz').value)

        avoid_collisions = bool(
            self.get_parameter('avoid_collisions').value
        )

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/compute_ik 서비스를 찾지 못했습니다.')
            return 1

        if not self.wait_for_joint_state():
            self.get_logger().error('/joint_states를 받지 못했습니다.')
            return 1

        transform = self.get_current_tool_transform(
            base_frame,
            ik_link_name,
        )

        if transform is None:
            self.get_logger().error(
                f'{base_frame} → {ik_link_name} TF를 찾지 못했습니다.'
            )
            return 1

        current = transform.transform

        target_x = current.translation.x + dx
        target_y = current.translation.y + dy
        target_z = current.translation.z + dz

        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = ik_link_name
        request.ik_request.avoid_collisions = avoid_collisions

        seed_state = RobotState()
        seed_state.joint_state = self.latest_joint_state
        seed_state.is_diff = False
        request.ik_request.robot_state = seed_state

        pose = request.ik_request.pose_stamped
        pose.header.frame_id = base_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.position.z = target_z

        pose.pose.orientation.x = current.rotation.x
        pose.pose.orientation.y = current.rotation.y
        pose.pose.orientation.z = current.rotation.z
        pose.pose.orientation.w = current.rotation.w

        request.ik_request.timeout.sec = 2
        request.ik_request.timeout.nanosec = 0

        self.get_logger().info(
            f'그룹: {group_name}, IK 링크: {ik_link_name}'
        )
        self.get_logger().info(
            f'현재 위치: '
            f'x={current.translation.x:.4f}, '
            f'y={current.translation.y:.4f}, '
            f'z={current.translation.z:.4f}'
        )
        self.get_logger().info(
            f'목표 위치: '
            f'x={target_x:.4f}, '
            f'y={target_y:.4f}, '
            f'z={target_z:.4f}'
        )

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if not future.done() or future.result() is None:
            self.get_logger().error('IK 서비스 응답을 받지 못했습니다.')
            return 1

        response = future.result()

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'IK 실패: error_code={response.error_code.val}'
            )
            return 1

        self.get_logger().info('IK 성공: 계산된 관절각')

        names = response.solution.joint_state.name
        positions = response.solution.joint_state.position

        for name, position in zip(names, positions):
            if group_name == 'left_arm' and ('_L_' in name or 'linkL' in name):
                self.get_logger().info(f'  {name}: {position:.6f} rad')
            elif group_name == 'right_arm' and ('_R_' in name or 'linkR' in name):
                self.get_logger().info(f'  {name}: {position:.6f} rad')

        return 0


def main() -> None:
    rclpy.init()
    node = ComputeIKTest()

    try:
        result = node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(result)


if __name__ == '__main__':
    main()
