#!/usr/bin/env python3
"""
Gripper Node
Franka Hand 그리퍼 제어 노드 (libfranka SDK via franka_gripper actions)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading
from concurrent.futures import Future as ConcurrentFuture

from std_srvs.srv import Trigger
from std_msgs.msg import Bool, Float64, String
from franka_msgs.action import Move, Grasp
from enum import Enum


class GripperState(Enum):
    """그리퍼 상태"""
    IDLE = 0
    OPENING = 1
    CLOSING = 2
    HOLDING = 3
    ERROR = 4


class GripperError(Enum):
    """그리퍼 에러 코드"""
    NONE = 0
    ACTION_SERVER_NOT_READY = 1
    GOAL_REJECTED = 2
    ACTION_TIMEOUT = 3
    EXECUTION_FAILED = 4
    COMMUNICATION_ERROR = 5
    BUSY = 6


class GripperNode(Node):
    """
    Franka Hand 그리퍼 제어 노드

    libfranka SDK를 franka_gripper 액션 서버를 통해 사용합니다.
    콜백 기반 비동기 처리로 서비스 콜백 내 블로킹 방지.

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
    DEFAULT_ACTION_TIMEOUT = 10.0
    MAX_RETRY_COUNT = 3

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
        self.declare_parameter('gripper_action_timeout', self.DEFAULT_ACTION_TIMEOUT)

        self._max_width = self.get_parameter('gripper_max_width').value
        self._default_speed = self.get_parameter('gripper_default_speed').value
        self._default_force = self.get_parameter('gripper_default_force').value
        self._epsilon_inner = self.get_parameter('gripper_epsilon_inner').value
        self._epsilon_outer = self.get_parameter('gripper_epsilon_outer').value
        self._action_namespace = self.get_parameter('gripper_action_namespace').value
        self._action_timeout = self.get_parameter('gripper_action_timeout').value

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
        self._last_error = GripperError.NONE
        self._last_error_message = ''
        self._is_busy = False
        self._retry_count = 0
        # 콜백 기반 비동기 처리용
        self._action_result = None
        self._action_event = threading.Event()

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
        self._srv_recover = self.create_service(
            Trigger, 'gripper/recover', self._handle_recover,
            callback_group=self._callback_group
        )
        self._srv_stop = self.create_service(
            Trigger, 'gripper/stop', self._handle_stop,
            callback_group=self._callback_group
        )

    def _create_publishers(self):
        """퍼블리셔 생성"""
        self._pub_is_grasping = self.create_publisher(Bool, 'gripper/is_grasping', 10)
        self._pub_width = self.create_publisher(Float64, 'gripper/width', 10)
        self._pub_state = self.create_publisher(String, 'gripper/state', 10)
        self._pub_error = self.create_publisher(String, 'gripper/error', 10)

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

        msg_state = String()
        msg_state.data = self._current_state.name
        self._pub_state.publish(msg_state)

        msg_error = String()
        if self._last_error != GripperError.NONE:
            msg_error.data = f'{self._last_error.name}: {self._last_error_message}'
        else:
            msg_error.data = ''
        self._pub_error.publish(msg_error)

    def _set_error(self, error: GripperError, message: str):
        """에러 상태 설정"""
        self._last_error = error
        self._last_error_message = message
        self._current_state = GripperState.ERROR
        self.get_logger().error(f'[{error.name}] {message}')

    def _clear_error(self):
        """에러 상태 클리어"""
        self._last_error = GripperError.NONE
        self._last_error_message = ''
        if self._current_state == GripperState.ERROR:
            self._current_state = GripperState.IDLE

    def _check_busy(self) -> bool:
        """바쁜 상태 체크 - True면 요청 거부해야 함"""
        if self._is_busy:
            self._set_error(GripperError.BUSY, 'Gripper is busy with another operation')
            return True
        return False

    def _handle_open(self, request, response):
        """그리퍼 열기"""
        self.get_logger().info('Opening gripper...')

        if self._check_busy():
            response.success = False
            response.message = 'Gripper is busy'
            return response

        try:
            self._is_busy = True
            self._clear_error()
            self._current_state = GripperState.OPENING
            success = self._execute_open()

            if success:
                self._current_state = GripperState.IDLE
                self._is_grasping = False
                self._current_width = self._max_width
                response.success = True
                response.message = 'Gripper opened'
            else:
                response.success = False
                response.message = f'Failed to open gripper: {self._last_error_message}'

        except Exception as e:
            self._set_error(GripperError.COMMUNICATION_ERROR, str(e))
            response.success = False
            response.message = f'Error: {str(e)}'
        finally:
            self._is_busy = False

        return response

    def _handle_close(self, request, response):
        """그리퍼 닫기"""
        self.get_logger().info('Closing gripper...')

        if self._check_busy():
            response.success = False
            response.message = 'Gripper is busy'
            return response

        try:
            self._is_busy = True
            self._clear_error()
            self._current_state = GripperState.CLOSING
            success = self._execute_close()

            if success:
                self._current_state = GripperState.IDLE
                self._current_width = 0.0
                response.success = True
                response.message = 'Gripper closed'
            else:
                response.success = False
                response.message = f'Failed to close gripper: {self._last_error_message}'

        except Exception as e:
            self._set_error(GripperError.COMMUNICATION_ERROR, str(e))
            response.success = False
            response.message = f'Error: {str(e)}'
        finally:
            self._is_busy = False

        return response

    def _handle_grasp(self, request, response):
        """물체 파지"""
        self.get_logger().info('Grasping object...')

        if self._check_busy():
            response.success = False
            response.message = 'Gripper is busy'
            return response

        try:
            self._is_busy = True
            self._clear_error()
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
                response.success = False
                response.message = f'Grasp failed: {self._last_error_message}'

        except Exception as e:
            self._set_error(GripperError.COMMUNICATION_ERROR, str(e))
            response.success = False
            response.message = f'Error: {str(e)}'
        finally:
            self._is_busy = False

        return response

    def _handle_recover(self, request, response):
        """에러 복구"""
        self.get_logger().info('Recovering from error...')

        if self._current_state != GripperState.ERROR:
            response.success = True
            response.message = 'No error to recover from'
            return response

        self._clear_error()
        self._is_busy = False
        self._retry_count = 0
        self._current_state = GripperState.IDLE

        response.success = True
        response.message = 'Error cleared, gripper ready'
        self.get_logger().info('Gripper recovered successfully')
        return response

    def _handle_stop(self, request, response):
        """그리퍼 동작 중지"""
        self.get_logger().info('Stopping gripper...')

        self._is_busy = False
        self._retry_count = 0

        if self._current_state in [GripperState.OPENING, GripperState.CLOSING]:
            self._current_state = GripperState.IDLE
            response.success = True
            response.message = 'Gripper stopped'
        else:
            response.success = True
            response.message = 'Gripper was not moving'

        return response

    def _execute_open(self) -> bool:
        """그리퍼 열기 실행 (Move 액션) - 콜백 기반 비동기 처리"""
        self.get_logger().info(f'Opening to width: {self._max_width}m, speed: {self._default_speed}m/s')

        if not self._move_action_client.server_is_ready():
            self.get_logger().warn('Move action server not ready - simulating open')
            return True

        goal = Move.Goal()
        goal.width = self._max_width
        goal.speed = self._default_speed

        # 이벤트 초기화
        self._action_event.clear()
        self._action_result = None

        # 콜백 기반 비동기 goal 전송
        send_goal_future = self._move_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_move_goal_response)

        # 타임아웃과 함께 이벤트 대기 (executor에서 콜백 처리)
        if not self._action_event.wait(timeout=self._action_timeout):
            self._set_error(GripperError.ACTION_TIMEOUT, 'Open action timed out')
            return False

        if self._action_result is None:
            self._set_error(GripperError.COMMUNICATION_ERROR, 'Failed to get open result')
            return False

        if self._action_result.success:
            self._current_width = self._max_width
            self.get_logger().info('Gripper opened successfully')
        else:
            self._set_error(GripperError.EXECUTION_FAILED, f'Open failed: {self._action_result.error}')

        return self._action_result.success

    def _on_move_goal_response(self, future):
        """Move 액션 goal 응답 콜백"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._set_error(GripperError.GOAL_REJECTED, 'Move goal rejected by action server')
                self._action_event.set()
                return

            # 결과 비동기 대기
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_move_result)
        except Exception as e:
            self._set_error(GripperError.COMMUNICATION_ERROR, f'Goal response error: {e}')
            self._action_event.set()

    def _on_move_result(self, future):
        """Move 액션 결과 콜백"""
        try:
            self._action_result = future.result().result
        except Exception as e:
            self._set_error(GripperError.COMMUNICATION_ERROR, f'Result error: {e}')
            self._action_result = None
        finally:
            self._action_event.set()

    def _execute_close(self) -> bool:
        """그리퍼 닫기 실행 (Move 액션) - 콜백 기반 비동기 처리"""
        self.get_logger().info(f'Closing gripper, speed: {self._default_speed}m/s')

        if not self._move_action_client.server_is_ready():
            self.get_logger().warn('Move action server not ready - simulating close')
            return True

        goal = Move.Goal()
        goal.width = 0.0
        goal.speed = self._default_speed

        # 이벤트 초기화
        self._action_event.clear()
        self._action_result = None

        # 콜백 기반 비동기 goal 전송
        send_goal_future = self._move_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_move_goal_response)

        # 타임아웃과 함께 이벤트 대기
        if not self._action_event.wait(timeout=self._action_timeout):
            self._set_error(GripperError.ACTION_TIMEOUT, 'Close action timed out')
            return False

        if self._action_result is None:
            self._set_error(GripperError.COMMUNICATION_ERROR, 'Failed to get close result')
            return False

        if self._action_result.success:
            self._current_width = 0.0
            self.get_logger().info('Gripper closed successfully')
        else:
            self._set_error(GripperError.EXECUTION_FAILED, f'Close failed: {self._action_result.error}')

        return self._action_result.success

    def _execute_grasp(self) -> tuple[bool, bool]:
        """파지 실행 (Grasp 액션) - 콜백 기반 비동기 처리"""
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

        # 이벤트 초기화
        self._action_event.clear()
        self._action_result = None

        # 콜백 기반 비동기 goal 전송
        send_goal_future = self._grasp_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_grasp_goal_response)

        # 타임아웃과 함께 이벤트 대기
        if not self._action_event.wait(timeout=self._action_timeout):
            self._set_error(GripperError.ACTION_TIMEOUT, 'Grasp action timed out')
            return False, False

        if self._action_result is None:
            self._set_error(GripperError.COMMUNICATION_ERROR, 'Failed to get grasp result')
            return False, False

        grasped = self._action_result.success

        if grasped:
            self.get_logger().info('Object grasped successfully')
        else:
            self.get_logger().warn(f'Grasp completed but object not detected: {self._action_result.error}')

        return True, grasped

    def _on_grasp_goal_response(self, future):
        """Grasp 액션 goal 응답 콜백"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._set_error(GripperError.GOAL_REJECTED, 'Grasp goal rejected by action server')
                self._action_event.set()
                return

            # 결과 비동기 대기
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_grasp_result)
        except Exception as e:
            self._set_error(GripperError.COMMUNICATION_ERROR, f'Goal response error: {e}')
            self._action_event.set()

    def _on_grasp_result(self, future):
        """Grasp 액션 결과 콜백"""
        try:
            self._action_result = future.result().result
        except Exception as e:
            self._set_error(GripperError.COMMUNICATION_ERROR, f'Result error: {e}')
            self._action_result = None
        finally:
            self._action_event.set()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()

    # MultiThreadedExecutor 사용으로 콜백 병렬 처리 가능
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
