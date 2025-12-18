#!/usr/bin/env python3
"""
Gripper Node
Franka Hand 그리퍼 제어 노드 (libfranka SDK via franka_gripper actions)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from std_srvs.srv import Trigger
from std_msgs.msg import Bool, Float64
from franka_msgs.action import Move, Grasp
from enum import Enum


class GripperState(Enum):
    """그리퍼 상태"""
    IDLE = 0
    OPENING = 1
    CLOSING = 2
    HOLDING = 3
    ERROR = 4


class GripperNode(Node):
    """
    Franka Hand 그리퍼 제어 노드

    libfranka SDK를 franka_gripper 액션 서버를 통해 사용합니다.

    Services:
        /gripper/open: 그리퍼 열기
        /gripper/close: 그리퍼 닫기
        /gripper/grasp: 물체 파지

    Publishers:
        /gripper/is_grasping: 파지 여부
        /gripper/width: 현재 그리퍼 너비

    Action Clients:
        /<namespace>/move: Move 액션 (열기/닫기)
        /<namespace>/grasp: Grasp 액션 (파지)
    """

    DEFAULT_MAX_WIDTH = 0.08
    DEFAULT_SPEED = 0.1
    DEFAULT_FORCE = 40.0
    DEFAULT_EPSILON_INNER = 0.005
    DEFAULT_EPSILON_OUTER = 0.005
    STATE_PUBLISH_RATE = 10.0
    ACTION_TIMEOUT = 10.0

    def __init__(self):
        super().__init__('gripper_node')

        self._callback_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self._init_state()
        self._create_action_clients()
        self._create_services()
        self._create_publishers()
        self._create_timers()

        self.get_logger().info('Gripper Node initialized (with franka_gripper actions)')

    def _declare_parameters(self):
        """파라미터 선언"""
        self.declare_parameter('gripper_max_width', self.DEFAULT_MAX_WIDTH)
        self.declare_parameter('gripper_default_speed', self.DEFAULT_SPEED)
        self.declare_parameter('gripper_default_force', self.DEFAULT_FORCE)
        self.declare_parameter('gripper_epsilon_inner', self.DEFAULT_EPSILON_INNER)
        self.declare_parameter('gripper_epsilon_outer', self.DEFAULT_EPSILON_OUTER)
        self.declare_parameter('gripper_action_namespace', 'fr3_gripper')

        self._max_width = self.get_parameter('gripper_max_width').value
        self._default_speed = self.get_parameter('gripper_default_speed').value
        self._default_force = self.get_parameter('gripper_default_force').value
        self._epsilon_inner = self.get_parameter('gripper_epsilon_inner').value
        self._epsilon_outer = self.get_parameter('gripper_epsilon_outer').value
        self._action_namespace = self.get_parameter('gripper_action_namespace').value

    def _create_action_clients(self):
        """franka_gripper 액션 클라이언트 생성"""
        self._move_action_client = ActionClient(
            self,
            Move,
            f'{self._action_namespace}/move',
            callback_group=self._callback_group
        )
        self._grasp_action_client = ActionClient(
            self,
            Grasp,
            f'{self._action_namespace}/grasp',
            callback_group=self._callback_group
        )

        self.get_logger().info(f'Waiting for gripper action servers on {self._action_namespace}...')
        move_ready = self._move_action_client.wait_for_server(timeout_sec=5.0)
        grasp_ready = self._grasp_action_client.wait_for_server(timeout_sec=5.0)

        if move_ready and grasp_ready:
            self.get_logger().info('Gripper action servers connected')
        else:
            self.get_logger().warn('Gripper action servers not available - running in simulation mode')

    def _init_state(self):
        """상태 변수 초기화"""
        self._current_state = GripperState.IDLE
        self._current_width = 0.0
        self._is_grasping = False

    def _create_services(self):
        """서비스 생성"""
        self._srv_open = self.create_service(
            Trigger, 'gripper/open', self._handle_open,
            callback_group=self._callback_group
        )
        self._srv_close = self.create_service(
            Trigger, 'gripper/close', self._handle_close,
            callback_group=self._callback_group
        )
        self._srv_grasp = self.create_service(
            Trigger, 'gripper/grasp', self._handle_grasp,
            callback_group=self._callback_group
        )

    def _create_publishers(self):
        """퍼블리셔 생성"""
        self._pub_is_grasping = self.create_publisher(Bool, 'gripper/is_grasping', 10)
        self._pub_width = self.create_publisher(Float64, 'gripper/width', 10)

    def _create_timers(self):
        """타이머 생성"""
        period = 1.0 / self.STATE_PUBLISH_RATE
        self._state_timer = self.create_timer(
            period, self._publish_state, callback_group=self._callback_group
        )

    def _publish_state(self):
        """상태 퍼블리시"""
        msg_grasping = Bool()
        msg_grasping.data = self._is_grasping
        self._pub_is_grasping.publish(msg_grasping)

        msg_width = Float64()
        msg_width.data = self._current_width
        self._pub_width.publish(msg_width)

    def _handle_open(self, request, response):
        """그리퍼 열기"""
        self.get_logger().info('Opening gripper...')

        try:
            self._current_state = GripperState.OPENING
            success = self._execute_open()

            if success:
                self._current_state = GripperState.IDLE
                self._is_grasping = False
                self._current_width = self._max_width
                response.success = True
                response.message = 'Gripper opened'
            else:
                self._current_state = GripperState.ERROR
                response.success = False
                response.message = 'Failed to open gripper'

        except Exception as e:
            self._current_state = GripperState.ERROR
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def _handle_close(self, request, response):
        """그리퍼 닫기"""
        self.get_logger().info('Closing gripper...')

        try:
            self._current_state = GripperState.CLOSING
            success = self._execute_close()

            if success:
                self._current_state = GripperState.IDLE
                self._current_width = 0.0
                response.success = True
                response.message = 'Gripper closed'
            else:
                self._current_state = GripperState.ERROR
                response.success = False
                response.message = 'Failed to close gripper'

        except Exception as e:
            self._current_state = GripperState.ERROR
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def _handle_grasp(self, request, response):
        """물체 파지"""
        self.get_logger().info('Grasping object...')

        try:
            self._current_state = GripperState.CLOSING
            success, grasped = self._execute_grasp()

            if success and grasped:
                self._current_state = GripperState.HOLDING
                self._is_grasping = True
                response.success = True
                response.message = 'Object grasped'
            elif success and not grasped:
                self._current_state = GripperState.IDLE
                self._is_grasping = False
                response.success = False
                response.message = 'No object detected'
            else:
                self._current_state = GripperState.ERROR
                response.success = False
                response.message = 'Grasp failed'

        except Exception as e:
            self._current_state = GripperState.ERROR
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def _execute_open(self) -> bool:
        """그리퍼 열기 실행 (Move 액션)"""
        self.get_logger().info(f'Opening to width: {self._max_width}m, speed: {self._default_speed}m/s')

        if not self._move_action_client.server_is_ready():
            self.get_logger().warn('Move action server not ready - simulating open')
            return True

        goal = Move.Goal()
        goal.width = self._max_width
        goal.speed = self._default_speed

        future = self._move_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.ACTION_TIMEOUT)

        if not future.done():
            self.get_logger().error('Failed to send open goal')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Open goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.ACTION_TIMEOUT)

        if not result_future.done():
            self.get_logger().error('Open action timed out')
            return False

        result = result_future.result().result
        if result.success:
            self._current_width = self._max_width
            self.get_logger().info('Gripper opened successfully')
        else:
            self.get_logger().error(f'Open failed: {result.error}')

        return result.success

    def _execute_close(self) -> bool:
        """그리퍼 닫기 실행 (Move 액션)"""
        self.get_logger().info(f'Closing gripper, speed: {self._default_speed}m/s')

        if not self._move_action_client.server_is_ready():
            self.get_logger().warn('Move action server not ready - simulating close')
            return True

        goal = Move.Goal()
        goal.width = 0.0
        goal.speed = self._default_speed

        future = self._move_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.ACTION_TIMEOUT)

        if not future.done():
            self.get_logger().error('Failed to send close goal')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Close goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.ACTION_TIMEOUT)

        if not result_future.done():
            self.get_logger().error('Close action timed out')
            return False

        result = result_future.result().result
        if result.success:
            self._current_width = 0.0
            self.get_logger().info('Gripper closed successfully')
        else:
            self.get_logger().error(f'Close failed: {result.error}')

        return result.success

    def _execute_grasp(self) -> tuple[bool, bool]:
        """파지 실행 (Grasp 액션)"""
        self.get_logger().info(
            f'Grasping with force: {self._default_force}N, speed: {self._default_speed}m/s'
        )

        if not self._grasp_action_client.server_is_ready():
            self.get_logger().warn('Grasp action server not ready - simulating grasp')
            return True, True

        goal = Grasp.Goal()
        goal.width = 0.0
        goal.speed = self._default_speed
        goal.force = self._default_force
        goal.epsilon.inner = self._epsilon_inner
        goal.epsilon.outer = self._epsilon_outer

        future = self._grasp_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.ACTION_TIMEOUT)

        if not future.done():
            self.get_logger().error('Failed to send grasp goal')
            return False, False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Grasp goal rejected')
            return False, False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.ACTION_TIMEOUT)

        if not result_future.done():
            self.get_logger().error('Grasp action timed out')
            return False, False

        result = result_future.result().result
        grasped = result.success

        if grasped:
            self.get_logger().info('Object grasped successfully')
        else:
            self.get_logger().warn(f'Grasp completed but object not detected: {result.error}')

        return True, grasped


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
