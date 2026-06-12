"""
conftest.py - Общие фикстуры и конфигурация для тестов rob_box_mcp_tools

Этот файл содержит:
- Mock объекты для ROS 2 компонентов
- Фикстуры для тестовых данных
- Утилиты для тестирования
"""

import json
import os
from typing import Any, Dict, List, Optional
from unittest.mock import MagicMock, Mock

import pytest


# ============================================================
# Mock ROS 2 Components
# ============================================================


class MockLogger:
    """Мок для ROS 2 Logger"""

    def __init__(self):
        self.info_messages = []
        self.warning_messages = []
        self.error_messages = []
        self.debug_messages = []

    def info(self, msg: str):
        self.info_messages.append(msg)

    def warning(self, msg: str):
        self.warning_messages.append(msg)

    def error(self, msg: str):
        self.error_messages.append(msg)

    def debug(self, msg: str):
        self.debug_messages.append(msg)

    def get_child(self, name: str):
        return self


class MockPublisher:
    """Мок для ROS 2 Publisher"""

    def __init__(self, msg_type, topic: str, qos: int = 10):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.published_messages = []

    def publish(self, msg):
        self.published_messages.append(msg)

    def get_messages(self) -> List:
        return self.published_messages

    def clear(self):
        self.published_messages.clear()


class MockSubscription:
    """Мок для ROS 2 Subscription"""

    def __init__(self, msg_type, topic: str, callback, qos: int = 10):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos


class MockServiceClient:
    """Мок для ROS 2 Service Client"""

    def __init__(self, srv_type, srv_name: str):
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.call_count = 0
        self.mock_response = None

    def wait_for_service(self, timeout_sec: float = 1.0) -> bool:
        return True

    def call_async(self, request):
        self.call_count += 1
        future = Mock()
        future.result.return_value = self.mock_response
        return future


class MockNode:
    """Мок для ROS 2 Node с полным API"""

    def __init__(self, node_name: str = "test_node"):
        self.node_name = node_name
        self._logger = MockLogger()
        self._publishers = {}
        self._subscriptions = {}
        self._service_clients = {}
        self._timers = []

    def get_logger(self) -> MockLogger:
        return self._logger

    def create_publisher(self, msg_type, topic: str, qos: int = 10) -> MockPublisher:
        pub = MockPublisher(msg_type, topic, qos)
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic: str, callback, qos: int = 10) -> MockSubscription:
        sub = MockSubscription(msg_type, topic, callback, qos)
        self._subscriptions[topic] = sub
        return sub

    def create_client(self, srv_type, srv_name: str) -> MockServiceClient:
        client = MockServiceClient(srv_type, srv_name)
        self._service_clients[srv_name] = client
        return client

    def create_timer(self, timer_period_sec: float, callback):
        timer = Mock()
        timer.timer_period_nanoseconds = int(timer_period_sec * 1e9)
        timer.callback = callback
        self._timers.append(timer)
        return timer

    def destroy_publisher(self, publisher):
        for topic, pub in list(self._publishers.items()):
            if pub == publisher:
                del self._publishers[topic]

    def destroy_subscription(self, subscription):
        for topic, sub in list(self._subscriptions.items()):
            if sub == subscription:
                del self._subscriptions[topic]

    def destroy_client(self, client):
        for srv_name, cli in list(self._service_clients.items()):
            if cli == client:
                del self._service_clients[srv_name]

    def get_publisher(self, topic: str) -> Optional[MockPublisher]:
        return self._publishers.get(topic)

    def get_subscription(self, topic: str) -> Optional[MockSubscription]:
        return self._subscriptions.get(topic)


# ============================================================
# Fixtures
# ============================================================


@pytest.fixture
def mock_node():
    """Фикстура для мок ROS 2 ноды"""
    return MockNode("test_mcp_node")


@pytest.fixture
def mock_logger():
    """Фикстура для мок логгера"""
    return MockLogger()


@pytest.fixture
def sample_tool_parameters():
    """Фикстура с примерами параметров инструментов"""
    return {
        "string_param": {"name": "test_param", "type": "string", "description": "Test string parameter", "required": True},
        "integer_param": {"name": "count", "type": "integer", "description": "Test integer parameter", "required": False, "default": 5},
        "enum_param": {
            "name": "choice",
            "type": "string",
            "description": "Test enum parameter",
            "required": True,
            "enum": ["option1", "option2", "option3"],
        },
        "number_param": {"name": "value", "type": "number", "description": "Test number parameter", "required": False, "minimum": 0.0, "maximum": 1.0},
    }


@pytest.fixture
def sample_openai_tools():
    """Фикстура с примерами инструментов в OpenAI формате"""
    return [
        {
            "type": "function",
            "function": {
                "name": "test_tool",
                "description": "A test tool for testing",
                "parameters": {
                    "type": "object",
                    "properties": {"param1": {"type": "string", "description": "First parameter"}},
                    "required": ["param1"],
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "another_tool",
                "description": "Another test tool",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "param1": {"type": "string", "description": "String param"},
                        "param2": {"type": "integer", "description": "Integer param"},
                    },
                    "required": ["param1"],
                },
            },
        },
    ]


@pytest.fixture
def sample_tool_calls():
    """Фикстура с примерами tool_calls от LLM"""
    return [
        {"id": "call_123", "type": "function", "function": {"name": "test_tool", "arguments": json.dumps({"param1": "value1"})}},
        {
            "id": "call_456",
            "type": "function",
            "function": {"name": "another_tool", "arguments": json.dumps({"param1": "value1", "param2": 42})},
        },
    ]


@pytest.fixture
def llm_api_key():
    """Фикстура для получения API ключа из переменных окружения"""
    return os.getenv("DEEPSEEK_API_KEY") or os.getenv("MIMO_API_KEY") or os.getenv("LLM_API_KEY")


@pytest.fixture
def llm_api_available(llm_api_key):
    """Фикстура для проверки доступности LLM API"""
    return llm_api_key is not None


@pytest.fixture
def skip_if_no_llm_api(llm_api_available):
    """Фикстура для пропуска тестов если нет LLM API"""
    if not llm_api_available:
        pytest.skip("LLM API key not available")


# ============================================================
# Utilities
# ============================================================


def assert_tool_result_success(result, expected_message: str = None):
    """Утилита для проверки успешного результата инструмента"""
    assert result.success is True, f"Tool execution failed: {result.error}"
    if expected_message:
        assert expected_message in result.message


def assert_tool_result_failure(result, expected_error: str = None):
    """Утилита для проверки неуспешного результата инструмента"""
    assert result.success is False, "Tool execution should have failed"
    if expected_error:
        assert expected_error in result.error


def create_mock_string_msg(data: str):
    """Создать мок std_msgs/String сообщение"""
    msg = Mock()
    msg.data = data
    return msg


def create_mock_service_request(srv_type):
    """Создать мок service request"""
    return Mock(spec=srv_type.Request)


def create_mock_service_response(srv_type):
    """Создать мок service response"""
    return Mock(spec=srv_type.Response)
