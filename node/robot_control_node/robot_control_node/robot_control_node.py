#!/usr/bin/env python3
"""
Robot Control Node
MoveIt과의 인터커넥션 레이어 - 모션 명령 큐 관리

기능:
- 모션 명령 큐 관리 (순차 실행)
- 실행 상태 모니터링 (pending/executing/success/failed)
- 에러 핸들링 및 재시도
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Callable
from collections import deque
from threading import Lock
import uuid


class MotionStatus(Enum):
    """모션 실행 상태"""
    PENDING = 'pending'
    EXECUTING = 'executing'
    SUCCESS = 'success'
    FAILED = 'failed'
    CANCELLED = 'cancelled'


class MotionType(Enum):
    """모션 타입"""
    MOVE_TO_POSE = 'move_to_pose'
    MOVE_TO_JOINT = 'move_to_joint'
    MOVE_CARTESIAN = 'move_cartesian'
    MOVE_HOME = 'move_home'


@dataclass
class MotionCommand:
    """모션 명령"""
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    motion_type: MotionType = MotionType.MOVE_TO_POSE
    target_pose: Optional[Pose] = None
    target_joints: Optional[list] = None
    status: MotionStatus = MotionStatus.PENDING
    retry_count: int = 0
    max_retries: int = 3
    callback: Optional[Callable] = None

    def __str__(self):
        return f'Motion[{self.id}] type={self.motion_type.value} status={self.status.value}'


class MotionQueue:
    """모션 명령 큐"""

    def __init__(self, logger):
        self._queue: deque[MotionCommand] = deque()
        self._lock = Lock()
        self._logger = logger

    def enqueue(self, command: MotionCommand) -> str:
        with self._lock:
            self._queue.append(command)
            self._logger.info(f'Enqueued: {command}')
            return command.id

    def dequeue(self) -> Optional[MotionCommand]:
        with self._lock:
            return self._queue.popleft() if self._queue else None

    def peek(self) -> Optional[MotionCommand]:
        with self._lock:
            return self._queue[0] if self._queue else None

    def clear(self):
        with self._lock:
            self._queue.clear()
            self._logger.info('Queue cleared')

    def size(self) -> int:
        with self._lock:
            return len(self._queue)

    def is_empty(self) -> bool:
        with self._lock:
            return len(self._queue) == 0


class RobotControlNode(Node):
    """
    로봇 제어 노드 - MoveIt 인터커넥션 레이어

    역할:
    - 상위 노드로부터 모션 명령 수신
    - 명령을 큐에 저장하고 순차 실행
    - MoveIt에 명령 전달 및 결과 모니터링
    - 실패 시 재시도 처리

    Services:
        /robot/move_home: 홈 위치 이동
        /robot/stop: 현재 동작 중지
        /robot/clear_queue: 큐 비우기

    Publishers:
        /robot/status: 현재 상태 (idle/executing/queued)
        /robot/current_command: 실행 중인 명령 정보

    Subscribers:
        /robot/target_pose: 목표 포즈 수신
    """

    DEFAULT_MAX_RETRIES = 3
    QUEUE_PROCESS_RATE = 10.0

    def __init__(self):
        super().__init__('robot_control_node')

        self._reentrant_group = ReentrantCallbackGroup()
        self._exclusive_group = MutuallyExclusiveCallbackGroup()

        self._declare_parameters()
        self._init_state()
        self._create_services()
        self._create_publishers()
        self._create_subscribers()
        self._create_timers()

        self.get_logger().info('Robot Control Node initialized')

    def _declare_parameters(self):
        self.declare_parameter('max_retries', self.DEFAULT_MAX_RETRIES)
        self.declare_parameter('queue_process_rate', self.QUEUE_PROCESS_RATE)

        self._max_retries = self.get_parameter('max_retries').value
        self._queue_process_rate = self.get_parameter('queue_process_rate').value

    def _init_state(self):
        self._motion_queue = MotionQueue(self.get_logger())
        self._current_command: Optional[MotionCommand] = None
        self._is_executing = False

    def _create_services(self):
        self._srv_move_home = self.create_service(
            Trigger, 'robot/move_home', self._handle_move_home,
            callback_group=self._reentrant_group
        )
        self._srv_stop = self.create_service(
            Trigger, 'robot/stop', self._handle_stop,
            callback_group=self._reentrant_group
        )
        self._srv_clear_queue = self.create_service(
            Trigger, 'robot/clear_queue', self._handle_clear_queue,
            callback_group=self._reentrant_group
        )

    def _create_publishers(self):
        self._pub_status = self.create_publisher(String, 'robot/status', 10)
        self._pub_current_command = self.create_publisher(String, 'robot/current_command', 10)

    def _create_subscribers(self):
        self._sub_target_pose = self.create_subscription(
            PoseStamped, 'robot/target_pose', self._handle_target_pose, 10,
            callback_group=self._reentrant_group
        )

    def _create_timers(self):
        # 큐 처리 타이머 (순차 실행 보장)
        self._queue_timer = self.create_timer(
            1.0 / self._queue_process_rate,
            self._process_queue,
            callback_group=self._exclusive_group
        )
        # 상태 퍼블리시 타이머
        self._status_timer = self.create_timer(
            0.1, self._publish_status,
            callback_group=self._reentrant_group
        )

    def _publish_status(self):
        """상태 퍼블리시"""
        status_msg = String()
        if self._is_executing and self._current_command:
            status_msg.data = f'executing:{self._current_command.id}'
        elif not self._motion_queue.is_empty():
            status_msg.data = f'queued:{self._motion_queue.size()}'
        else:
            status_msg.data = 'idle'
        self._pub_status.publish(status_msg)

        if self._current_command:
            cmd_msg = String()
            cmd_msg.data = str(self._current_command)
            self._pub_current_command.publish(cmd_msg)

    def _process_queue(self):
        """큐에서 명령을 꺼내 순차 실행"""
        if self._is_executing:
            return

        if self._motion_queue.is_empty():
            return

        command = self._motion_queue.dequeue()
        if command:
            self._execute_command(command)

    def _execute_command(self, command: MotionCommand):
        """명령 실행"""
        self._is_executing = True
        self._current_command = command
        command.status = MotionStatus.EXECUTING

        self.get_logger().info(f'Executing: {command}')

        try:
            # TODO: MoveIt 액션 클라이언트로 실제 명령 전송
            success = self._send_to_moveit(command)

            if success:
                command.status = MotionStatus.SUCCESS
                self.get_logger().info(f'Success: {command.id}')
                if command.callback:
                    command.callback(True, command)
            else:
                self._handle_failure(command)

        except Exception as e:
            self.get_logger().error(f'Execution error: {str(e)}')
            self._handle_failure(command)

        finally:
            self._is_executing = False
            self._current_command = None

    def _handle_failure(self, command: MotionCommand):
        """실패 처리 - 재시도 또는 최종 실패"""
        command.retry_count += 1

        if command.retry_count < command.max_retries:
            self.get_logger().warn(
                f'Retry {command.retry_count}/{command.max_retries}: {command.id}'
            )
            command.status = MotionStatus.PENDING
            # 큐 맨 앞에 다시 추가 (우선 재시도)
            self._motion_queue._queue.appendleft(command)
        else:
            command.status = MotionStatus.FAILED
            self.get_logger().error(f'Failed after {command.max_retries} retries: {command.id}')
            if command.callback:
                command.callback(False, command)

    def _send_to_moveit(self, command: MotionCommand) -> bool:
        """MoveIt으로 명령 전송"""
        # TODO: MoveIt MoveGroup 액션 클라이언트 구현
        # - move_to_pose: MoveGroup.action 으로 목표 포즈 전송
        # - move_to_joint: joint target 설정
        # - move_cartesian: cartesian path 계획

        self.get_logger().info(f'[MoveIt] Sending: {command.motion_type.value}')

        # 스켈레톤: 항상 성공 반환
        return True

    def _handle_target_pose(self, msg: PoseStamped):
        """목표 포즈 수신"""
        command = MotionCommand(
            motion_type=MotionType.MOVE_TO_POSE,
            target_pose=msg.pose,
            max_retries=self._max_retries
        )
        self._motion_queue.enqueue(command)

    def _handle_move_home(self, request, response):
        """홈 이동"""
        command = MotionCommand(
            motion_type=MotionType.MOVE_HOME,
            max_retries=self._max_retries
        )
        cmd_id = self._motion_queue.enqueue(command)

        response.success = True
        response.message = f'Move home queued: {cmd_id}'
        return response

    def _handle_stop(self, request, response):
        """정지"""
        self.get_logger().warn('Stop requested')

        if self._current_command:
            self._current_command.status = MotionStatus.CANCELLED

        self._is_executing = False
        self._current_command = None

        response.success = True
        response.message = 'Stopped'
        return response

    def _handle_clear_queue(self, request, response):
        """큐 비우기"""
        self._motion_queue.clear()

        response.success = True
        response.message = 'Queue cleared'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
