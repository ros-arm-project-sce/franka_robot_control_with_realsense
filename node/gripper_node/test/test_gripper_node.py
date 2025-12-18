#!/usr/bin/env python3
"""
gripper_node 단위 테스트
"""

import pytest
import rclpy
from rclpy.node import Node
from unittest.mock import MagicMock, patch, AsyncMock
from std_srvs.srv import Trigger

from gripper_node.gripper_node import GripperNode, GripperState, GripperError


class TestGripperState:
    """GripperState 열거형 테스트"""

    def test_gripper_states_exist(self):
        """모든 그리퍼 상태가 정의되어 있는지 확인"""
        assert GripperState.IDLE.value == 0
        assert GripperState.OPENING.value == 1
        assert GripperState.CLOSING.value == 2
        assert GripperState.HOLDING.value == 3
        assert GripperState.ERROR.value == 4


class TestGripperError:
    """GripperError 열거형 테스트"""

    def test_gripper_errors_exist(self):
        """모든 에러 코드가 정의되어 있는지 확인"""
        assert GripperError.NONE.value == 0
        assert GripperError.ACTION_SERVER_NOT_READY.value == 1
        assert GripperError.GOAL_REJECTED.value == 2
        assert GripperError.ACTION_TIMEOUT.value == 3
        assert GripperError.EXECUTION_FAILED.value == 4
        assert GripperError.COMMUNICATION_ERROR.value == 5
        assert GripperError.BUSY.value == 6


class TestGripperNodeInit:
    """GripperNode 초기화 테스트"""

    @pytest.fixture(autouse=True)
    def setup(self):
        """테스트 전 ROS 2 초기화"""
        rclpy.init()
        yield
        rclpy.shutdown()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_node_initialization(self, mock_action_client):
        """노드가 올바르게 초기화되는지 확인"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()

        assert node.get_name() == 'gripper_node'
        assert node._current_state == GripperState.IDLE
        assert node._current_width == 0.0
        assert node._is_grasping == False
        assert node._last_error == GripperError.NONE

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_default_parameters(self, mock_action_client):
        """기본 파라미터가 올바르게 설정되는지 확인"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()

        assert node._max_width == 0.08
        assert node._default_speed == 0.1
        assert node._default_force == 40.0
        assert node._epsilon_inner == 0.005
        assert node._epsilon_outer == 0.005
        assert node._action_namespace == 'fr3_gripper'
        assert node._action_timeout == 10.0

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_async_state_variables_initialized(self, mock_action_client):
        """비동기 처리용 상태 변수가 초기화되는지 확인"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()

        assert node._action_result is None
        assert node._action_event is not None
        assert not node._action_event.is_set()

        node.destroy_node()


class TestGripperNodeErrorHandling:
    """에러 핸들링 테스트"""

    @pytest.fixture(autouse=True)
    def setup(self):
        """테스트 전 ROS 2 초기화"""
        rclpy.init()
        yield
        rclpy.shutdown()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_set_error(self, mock_action_client):
        """에러 설정이 올바르게 동작하는지 확인"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()

        node._set_error(GripperError.ACTION_TIMEOUT, 'Test timeout error')

        assert node._last_error == GripperError.ACTION_TIMEOUT
        assert node._last_error_message == 'Test timeout error'
        assert node._current_state == GripperState.ERROR

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_clear_error(self, mock_action_client):
        """에러 클리어가 올바르게 동작하는지 확인"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()

        # 에러 설정
        node._set_error(GripperError.EXECUTION_FAILED, 'Test error')
        assert node._current_state == GripperState.ERROR

        # 에러 클리어
        node._clear_error()

        assert node._last_error == GripperError.NONE
        assert node._last_error_message == ''
        assert node._current_state == GripperState.IDLE

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_check_busy_when_idle(self, mock_action_client):
        """바쁘지 않을 때 check_busy가 False 반환"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()
        node._is_busy = False

        result = node._check_busy()

        assert result == False

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_check_busy_when_busy(self, mock_action_client):
        """바쁠 때 check_busy가 True 반환하고 에러 설정"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()
        node._is_busy = True

        result = node._check_busy()

        assert result == True
        assert node._last_error == GripperError.BUSY

        node.destroy_node()


class TestGripperNodeServices:
    """서비스 핸들러 테스트"""

    @pytest.fixture(autouse=True)
    def setup(self):
        """테스트 전 ROS 2 초기화"""
        rclpy.init()
        yield
        rclpy.shutdown()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_recover_no_error(self, mock_action_client):
        """에러 없을 때 recover 서비스 호출"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_recover(request, response)

        assert result.success == True
        assert 'No error' in result.message

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_recover_with_error(self, mock_action_client):
        """에러 있을 때 recover 서비스 호출"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()
        node._current_state = GripperState.ERROR
        node._last_error = GripperError.ACTION_TIMEOUT

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_recover(request, response)

        assert result.success == True
        assert node._current_state == GripperState.IDLE
        assert node._last_error == GripperError.NONE

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_stop_when_moving(self, mock_action_client):
        """이동 중일 때 stop 서비스 호출"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()
        node._current_state = GripperState.OPENING

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_stop(request, response)

        assert result.success == True
        assert node._current_state == GripperState.IDLE
        assert 'stopped' in result.message

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_stop_when_idle(self, mock_action_client):
        """대기 중일 때 stop 서비스 호출"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()
        node._current_state = GripperState.IDLE

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_stop(request, response)

        assert result.success == True
        assert 'not moving' in result.message

        node.destroy_node()


class TestGripperNodeSimulationMode:
    """시뮬레이션 모드 테스트 (액션 서버 미연결 시)"""

    @pytest.fixture(autouse=True)
    def setup(self):
        """테스트 전 ROS 2 초기화"""
        rclpy.init()
        yield
        rclpy.shutdown()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_execute_open_simulation(self, mock_action_client):
        """시뮬레이션 모드에서 open 실행"""
        mock_action_client.return_value.wait_for_server.return_value = False
        mock_action_client.return_value.server_is_ready.return_value = False

        node = GripperNode()

        result = node._execute_open()

        assert result == True  # 시뮬레이션 모드에서는 항상 성공

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_execute_close_simulation(self, mock_action_client):
        """시뮬레이션 모드에서 close 실행"""
        mock_action_client.return_value.wait_for_server.return_value = False
        mock_action_client.return_value.server_is_ready.return_value = False

        node = GripperNode()

        result = node._execute_close()

        assert result == True  # 시뮬레이션 모드에서는 항상 성공

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_execute_grasp_simulation(self, mock_action_client):
        """시뮬레이션 모드에서 grasp 실행"""
        mock_action_client.return_value.wait_for_server.return_value = False
        mock_action_client.return_value.server_is_ready.return_value = False

        node = GripperNode()

        success, grasped = node._execute_grasp()

        assert success == True  # 시뮬레이션 모드에서는 항상 성공
        assert grasped == True

        node.destroy_node()


class TestGripperNodeServiceHandlers:
    """서비스 핸들러 통합 테스트"""

    @pytest.fixture(autouse=True)
    def setup(self):
        """테스트 전 ROS 2 초기화"""
        rclpy.init()
        yield
        rclpy.shutdown()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_open_simulation_success(self, mock_action_client):
        """시뮬레이션 모드에서 open 서비스 성공"""
        mock_action_client.return_value.wait_for_server.return_value = False
        mock_action_client.return_value.server_is_ready.return_value = False

        node = GripperNode()

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_open(request, response)

        assert result.success == True
        assert 'opened' in result.message
        assert node._current_width == node._max_width
        assert node._is_grasping == False

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_close_simulation_success(self, mock_action_client):
        """시뮬레이션 모드에서 close 서비스 성공"""
        mock_action_client.return_value.wait_for_server.return_value = False
        mock_action_client.return_value.server_is_ready.return_value = False

        node = GripperNode()

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_close(request, response)

        assert result.success == True
        assert 'closed' in result.message
        assert node._current_width == 0.0

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_grasp_simulation_success(self, mock_action_client):
        """시뮬레이션 모드에서 grasp 서비스 성공"""
        mock_action_client.return_value.wait_for_server.return_value = False
        mock_action_client.return_value.server_is_ready.return_value = False

        node = GripperNode()

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_grasp(request, response)

        assert result.success == True
        assert 'grasped' in result.message
        assert node._is_grasping == True
        assert node._current_state == GripperState.HOLDING

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_open_busy_rejection(self, mock_action_client):
        """busy 상태에서 open 요청 거부"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()
        node._is_busy = True

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_open(request, response)

        assert result.success == False
        assert 'busy' in result.message.lower()
        assert node._last_error == GripperError.BUSY

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_close_busy_rejection(self, mock_action_client):
        """busy 상태에서 close 요청 거부"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()
        node._is_busy = True

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_close(request, response)

        assert result.success == False
        assert 'busy' in result.message.lower()

        node.destroy_node()

    @patch('gripper_node.gripper_node.ActionClient')
    def test_handle_grasp_busy_rejection(self, mock_action_client):
        """busy 상태에서 grasp 요청 거부"""
        mock_action_client.return_value.wait_for_server.return_value = False

        node = GripperNode()
        node._is_busy = True

        request = Trigger.Request()
        response = Trigger.Response()

        result = node._handle_grasp(request, response)

        assert result.success == False
        assert 'busy' in result.message.lower()

        node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
