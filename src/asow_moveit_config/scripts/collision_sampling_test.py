#!/usr/bin/env python3

import itertools
import random
import sys
from collections import Counter

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_LIMITS = {
    'base_link-v1_linkL1': (-1.570796, 1.570796),
    'link_L_1-v1_linkL2': (-1.745329, 1.745329),
    'link_L_2-v1_linkL3': (-1.570796, 1.570796),
    'link_L_3-v1_linkL4': (-1.570796, 1.570796),
    'link_L_4-v1_linkL5': (-3.141593, 0.0),

    'base_link-v1_linkR1': (-1.570796, 1.570796),
    'link_R_1-v1_linkR2': (-1.745329, 1.745329),
    'link_R_2-v1_linkR3': (-1.570796, 1.570796),
    'link_R_3-v1_linkR4': (-1.570796, 1.570796),
    'link_R_4-v1_linkR5': (-3.141593, 0.0),
}


class CollisionSamplingTest(Node):
    def __init__(self) -> None:
        super().__init__('collision_sampling_test')

        self.declare_parameter('random_samples', 5000)
        self.declare_parameter('seed', 42)
        self.declare_parameter('max_examples', 10)

        self.client = self.create_client(
            GetStateValidity,
            '/check_state_validity',
        )

        self.joint_names = list(JOINT_LIMITS.keys())
        self.collision_pairs = Counter()
        self.collision_examples = []

    def check_state(self, positions):
        request = GetStateValidity.Request()

        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = self.joint_names
        state.joint_state.position = list(positions)
        state.is_diff = False

        request.robot_state = state

        # 빈 문자열이면 전체 로봇 검사
        request.group_name = ''

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=3.0,
        )

        if not future.done() or future.result() is None:
            return None

        return future.result()

    def record_collision(self, response, positions) -> None:
        pairs = set()

        for contact in response.contacts:
            body1 = getattr(contact, 'contact_body_1', 'unknown')
            body2 = getattr(contact, 'contact_body_2', 'unknown')
            pair = tuple(sorted((body1, body2)))
            pairs.add(pair)

        for pair in pairs:
            self.collision_pairs[pair] += 1

        max_examples = int(self.get_parameter('max_examples').value)

        if len(self.collision_examples) < max_examples:
            self.collision_examples.append(
                {
                    'positions': list(positions),
                    'pairs': sorted(pairs),
                }
            )

    def run(self) -> int:
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/check_state_validity 서비스를 찾지 못했습니다.'
            )
            return 1

        random_samples = int(
            self.get_parameter('random_samples').value
        )
        seed = int(self.get_parameter('seed').value)

        random.seed(seed)

        tested = 0
        invalid = 0
        service_failures = 0

        # 1. Home 자세 검사
        test_sets = [
            ('home', [0.0] * len(self.joint_names)),
        ]

        # 2. 모든 관절 최소/최대 조합: 2^10 = 1024개
        extreme_states = itertools.product(
            *[
                (JOINT_LIMITS[name][0], JOINT_LIMITS[name][1])
                for name in self.joint_names
            ]
        )

        # 3. 관절 범위 내부 무작위 샘플
        random_states = (
            [
                random.uniform(
                    JOINT_LIMITS[name][0],
                    JOINT_LIMITS[name][1],
                )
                for name in self.joint_names
            ]
            for _ in range(random_samples)
        )

        self.get_logger().info('Home 자세 검사 시작')

        for label, positions in test_sets:
            response = self.check_state(positions)
            tested += 1

            if response is None:
                service_failures += 1
            elif not response.valid:
                invalid += 1
                self.record_collision(response, positions)
                self.get_logger().warning(
                    f'{label} 자세가 충돌 상태입니다.'
                )

        self.get_logger().info(
            '관절 최소/최대 조합 1024개 검사 시작'
        )

        for positions in extreme_states:
            response = self.check_state(positions)
            tested += 1

            if response is None:
                service_failures += 1
            elif not response.valid:
                invalid += 1
                self.record_collision(response, positions)

        self.get_logger().info(
            f'무작위 자세 {random_samples}개 검사 시작'
        )

        for index, positions in enumerate(random_states, start=1):
            response = self.check_state(positions)
            tested += 1

            if response is None:
                service_failures += 1
            elif not response.valid:
                invalid += 1
                self.record_collision(response, positions)

            if index % 500 == 0:
                self.get_logger().info(
                    f'무작위 검사 진행: {index}/{random_samples}'
                )

        valid = tested - invalid - service_failures

        print('\n========== 충돌 샘플링 결과 ==========')
        print(f'전체 검사 자세: {tested}')
        print(f'유효 자세:       {valid}')
        print(f'충돌 자세:       {invalid}')
        print(f'서비스 실패:     {service_failures}')

        if tested - service_failures > 0:
            ratio = invalid / (tested - service_failures) * 100.0
            print(f'충돌 검출 비율:  {ratio:.2f}%')

        print('\n충돌 링크 조합:')

        if not self.collision_pairs:
            print('  발견되지 않음')
        else:
            for pair, count in self.collision_pairs.most_common():
                print(f'  {pair[0]} <-> {pair[1]}: {count}회')

        if self.collision_examples:
            print('\n충돌 자세 예시:')

            for example_index, example in enumerate(
                self.collision_examples,
                start=1,
            ):
                print(f'\n[예시 {example_index}]')

                for name, position in zip(
                    self.joint_names,
                    example['positions'],
                ):
                    print(f'  {name}: {position:.6f}')

                print('  접촉 링크:')

                for pair in example['pairs']:
                    print(f'    {pair[0]} <-> {pair[1]}')

        print('=====================================\n')

        return 0 if service_failures == 0 else 1


def main() -> None:
    rclpy.init()
    node = CollisionSamplingTest()

    try:
        result = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(result)


if __name__ == '__main__':
    main()
