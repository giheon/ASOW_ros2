#!/usr/bin/env python3

import itertools
import random
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    'left_joint1',
    'left_joint2',
    'left_joint3',
    'left_joint4',
    'left_joint5',
    'right_joint1',
    'right_joint2',
    'right_joint3',
    'right_joint4',
    'right_joint5',
]

LEFT_JOINTS = JOINT_NAMES[:5]
RIGHT_JOINTS = JOINT_NAMES[5:]


class CollisionSamplingTest(Node):
    def __init__(self) -> None:
        super().__init__('collision_sampling_test')

        self.declare_parameter('left_random_samples', 1000)
        self.declare_parameter('right_random_samples', 1000)
        self.declare_parameter('both_random_samples', 5000)
        self.declare_parameter('boundary_samples', 2000)
        self.declare_parameter('boundary_fraction', 0.10)
        self.declare_parameter('include_all_extremes', True)
        self.declare_parameter('seed', 42)
        self.declare_parameter('max_examples', 3)
        self.declare_parameter('progress_interval', 500)

        self.validity_client = self.create_client(
            GetStateValidity,
            '/check_state_validity',
        )

        self.parameter_client = self.create_client(
            GetParameters,
            '/move_group/get_parameters',
        )

        self.joint_limits = {}

        self.stats = defaultdict(
            lambda: {
                'tested': 0,
                'valid': 0,
                'collision': 0,
                'noncollision_invalid': 0,
                'service_failure': 0,
            }
        )

        self.collision_pairs = Counter()
        self.category_pairs = defaultdict(Counter)
        self.examples = defaultdict(list)

        self.home_valid = False

    def load_runtime_joint_limits(self) -> bool:
        if not self.parameter_client.wait_for_service(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                '/move_group/get_parameters 서비스를 '
                '찾지 못했습니다.'
            )
            return False

        request = GetParameters.Request()
        request.names = ['robot_description']

        future = self.parameter_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if not future.done() or future.result() is None:
            self.get_logger().error(
                'robot_description 조회에 실패했습니다.'
            )
            return False

        response = future.result()

        if not response.values:
            self.get_logger().error(
                'robot_description 응답이 비어 있습니다.'
            )
            return False

        urdf_xml = response.values[0].string_value

        try:
            root = ET.fromstring(urdf_xml)
        except ET.ParseError as exc:
            self.get_logger().error(
                f'robot_description XML parse 실패: {exc}'
            )
            return False

        limits = {}

        for joint in root.findall('joint'):
            name = joint.attrib.get('name', '')

            if name not in JOINT_NAMES:
                continue

            if joint.attrib.get('type') != 'revolute':
                self.get_logger().error(
                    f'{name}: revolute joint가 아닙니다.'
                )
                return False

            limit = joint.find('limit')

            if limit is None:
                self.get_logger().error(
                    f'{name}: <limit>가 없습니다.'
                )
                return False

            limits[name] = (
                float(limit.attrib['lower']),
                float(limit.attrib['upper']),
            )

        missing = [
            name
            for name in JOINT_NAMES
            if name not in limits
        ]

        if missing:
            self.get_logger().error(
                f'런타임 URDF에서 joint 누락: {missing}'
            )
            return False

        self.joint_limits = limits

        print('\n===== RUNTIME JOINT LIMITS =====')

        for name in JOINT_NAMES:
            lower, upper = limits[name]
            print(
                f'{name:13s}: '
                f'{lower:+.6f} ~ {upper:+.6f}'
            )

        return True

    def check_state(self, positions):
        request = GetStateValidity.Request()

        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = list(JOINT_NAMES)
        state.joint_state.position = [
            float(positions[name])
            for name in JOINT_NAMES
        ]
        state.is_diff = False

        request.robot_state = state

        # 빈 group_name: 전체 로봇 기준 PlanningScene 검사
        request.group_name = ''

        future = self.validity_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=3.0,
        )

        if not future.done() or future.result() is None:
            return None

        return future.result()

    @staticmethod
    def extract_pairs(response):
        pairs = set()

        for contact in response.contacts:
            body1 = getattr(
                contact,
                'contact_body_1',
                'unknown',
            )
            body2 = getattr(
                contact,
                'contact_body_2',
                'unknown',
            )

            pairs.add(
                tuple(sorted((body1, body2)))
            )

        return pairs

    def record_state(
        self,
        category,
        positions,
    ) -> None:
        stats = self.stats[category]
        stats['tested'] += 1

        response = self.check_state(positions)

        if response is None:
            stats['service_failure'] += 1
            return

        if response.valid:
            stats['valid'] += 1

            if category == 'home':
                self.home_valid = True

            return

        pairs = self.extract_pairs(response)

        if pairs:
            stats['collision'] += 1

            for pair in pairs:
                self.collision_pairs[pair] += 1
                self.category_pairs[category][pair] += 1
        else:
            stats['noncollision_invalid'] += 1

        max_examples = int(
            self.get_parameter('max_examples').value
        )

        if len(self.examples[category]) < max_examples:
            self.examples[category].append(
                {
                    'positions': dict(positions),
                    'pairs': sorted(pairs),
                }
            )

    def uniform_state(
        self,
        rng,
        active_joints,
    ):
        positions = {
            name: 0.0
            for name in JOINT_NAMES
        }

        for name in active_joints:
            lower, upper = self.joint_limits[name]
            positions[name] = rng.uniform(
                lower,
                upper,
            )

        return positions

    def boundary_state(
        self,
        rng,
        fraction,
    ):
        positions = {}

        for name in JOINT_NAMES:
            lower, upper = self.joint_limits[name]
            span = upper - lower
            width = span * fraction

            if rng.random() < 0.5:
                positions[name] = rng.uniform(
                    lower,
                    lower + width,
                )
            else:
                positions[name] = rng.uniform(
                    upper - width,
                    upper,
                )

        return positions

    def run_random_category(
        self,
        category,
        count,
        generator,
    ):
        progress_interval = int(
            self.get_parameter(
                'progress_interval'
            ).value
        )

        self.get_logger().info(
            f'{category}: {count}개 검사 시작'
        )

        for index in range(1, count + 1):
            self.record_state(
                category,
                generator(),
            )

            if (
                progress_interval > 0
                and index % progress_interval == 0
            ):
                self.get_logger().info(
                    f'{category}: {index}/{count}'
                )

    def print_results(self):
        category_order = [
            'home',
            'single_joint_extreme',
            'all_joint_extreme',
            'left_uniform',
            'right_uniform',
            'both_uniform',
            'boundary_stress',
        ]

        print(
            '\n'
            '========== SELF-COLLISION REGRESSION =========='
        )

        total = {
            'tested': 0,
            'valid': 0,
            'collision': 0,
            'noncollision_invalid': 0,
            'service_failure': 0,
        }

        for category in category_order:
            if category not in self.stats:
                continue

            stats = self.stats[category]

            for key in total:
                total[key] += stats[key]

            completed = (
                stats['tested']
                - stats['service_failure']
            )

            ratio = (
                stats['collision']
                / completed
                * 100.0
                if completed > 0
                else 0.0
            )

            print(f'\n[{category}]')
            print(
                f"  tested:               "
                f"{stats['tested']}"
            )
            print(
                f"  valid:                "
                f"{stats['valid']}"
            )
            print(
                f"  collision:            "
                f"{stats['collision']}"
            )
            print(
                f"  noncollision_invalid: "
                f"{stats['noncollision_invalid']}"
            )
            print(
                f"  service_failure:      "
                f"{stats['service_failure']}"
            )
            print(
                f"  collision detection:  "
                f"{ratio:.2f}%"
            )

        total_completed = (
            total['tested']
            - total['service_failure']
        )

        total_ratio = (
            total['collision']
            / total_completed
            * 100.0
            if total_completed > 0
            else 0.0
        )

        print('\n[TOTAL]')
        print(f"  tested:               {total['tested']}")
        print(f"  valid:                {total['valid']}")
        print(
            f"  collision:            "
            f"{total['collision']}"
        )
        print(
            f"  noncollision_invalid: "
            f"{total['noncollision_invalid']}"
        )
        print(
            f"  service_failure:      "
            f"{total['service_failure']}"
        )
        print(
            f"  collision detection:  "
            f"{total_ratio:.2f}%"
        )

        print(
            '\n주의: 위 비율은 정의된 샘플링 분포에서의 '
            'collision detection 비율이며 실제 운용 중 '
            '충돌 발생률이 아닙니다.'
        )

        print('\n===== COLLISION PAIRS =====')

        if not self.collision_pairs:
            print('발견되지 않음')
        else:
            for pair, count in (
                self.collision_pairs.most_common()
            ):
                print(
                    f'{pair[0]} <-> {pair[1]}: '
                    f'{count} states'
                )

        print('\n===== EXAMPLES =====')

        for category in category_order:
            examples = self.examples.get(
                category,
                [],
            )

            if not examples:
                continue

            print(f'\n[{category}]')

            for index, example in enumerate(
                examples,
                start=1,
            ):
                print(f'  example {index}')

                for name in JOINT_NAMES:
                    print(
                        f'    {name}: '
                        f'{example["positions"][name]:+.6f}'
                    )

                if example['pairs']:
                    print('    pairs:')

                    for pair in example['pairs']:
                        print(
                            f'      {pair[0]} '
                            f'<-> {pair[1]}'
                        )
                else:
                    print(
                        '    no collision contact '
                        '(non-collision invalid)'
                    )

        print(
            '\n=============================================='
        )

        return total

    def run(self) -> int:
        if not self.validity_client.wait_for_service(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                '/check_state_validity 서비스를 '
                '찾지 못했습니다.'
            )
            return 1

        if not self.load_runtime_joint_limits():
            return 1

        seed = int(
            self.get_parameter('seed').value
        )
        rng = random.Random(seed)

        left_count = int(
            self.get_parameter(
                'left_random_samples'
            ).value
        )
        right_count = int(
            self.get_parameter(
                'right_random_samples'
            ).value
        )
        both_count = int(
            self.get_parameter(
                'both_random_samples'
            ).value
        )
        boundary_count = int(
            self.get_parameter(
                'boundary_samples'
            ).value
        )
        boundary_fraction = float(
            self.get_parameter(
                'boundary_fraction'
            ).value
        )
        include_all_extremes = bool(
            self.get_parameter(
                'include_all_extremes'
            ).value
        )

        if not (0.0 < boundary_fraction <= 0.5):
            self.get_logger().error(
                'boundary_fraction은 '
                '0 < value <= 0.5 이어야 합니다.'
            )
            return 1

        home = {
            name: 0.0
            for name in JOINT_NAMES
        }

        self.get_logger().info(
            'Home baseline 검사'
        )
        self.record_state(
            'home',
            home,
        )

        self.get_logger().info(
            'Single-joint extrema 검사'
        )

        for name in JOINT_NAMES:
            lower, upper = self.joint_limits[name]

            for value in (lower, upper):
                state = dict(home)
                state[name] = value

                self.record_state(
                    'single_joint_extreme',
                    state,
                )

        if include_all_extremes:
            self.get_logger().info(
                'All-joint extrema 2^10=1024 검사'
            )

            choices = [
                self.joint_limits[name]
                for name in JOINT_NAMES
            ]

            for values in itertools.product(
                *choices
            ):
                state = dict(
                    zip(JOINT_NAMES, values)
                )

                self.record_state(
                    'all_joint_extreme',
                    state,
                )

        self.run_random_category(
            'left_uniform',
            left_count,
            lambda: self.uniform_state(
                rng,
                LEFT_JOINTS,
            ),
        )

        self.run_random_category(
            'right_uniform',
            right_count,
            lambda: self.uniform_state(
                rng,
                RIGHT_JOINTS,
            ),
        )

        self.run_random_category(
            'both_uniform',
            both_count,
            lambda: self.uniform_state(
                rng,
                JOINT_NAMES,
            ),
        )

        self.run_random_category(
            'boundary_stress',
            boundary_count,
            lambda: self.boundary_state(
                rng,
                boundary_fraction,
            ),
        )

        total = self.print_results()

        if not self.home_valid:
            self.get_logger().error(
                'Home state가 valid하지 않습니다.'
            )
            return 2

        if total['service_failure'] > 0:
            self.get_logger().error(
                '서비스 실패가 발생했습니다.'
            )
            return 3

        if total['noncollision_invalid'] > 0:
            self.get_logger().error(
                'collision contact 없이 invalid인 '
                'state가 발견되었습니다.'
            )
            return 4

        return 0


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
