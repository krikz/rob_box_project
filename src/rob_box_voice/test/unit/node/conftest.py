"""
conftest.py — Mock всех ROS2/rclpy зависимостей для unit-тестов DialogueNode.

Должен быть загружен ДО импорта любого модуля с rclpy.
pytest автоматически применяет conftest.py к тестам в той же директории.
"""

import sys
import types
from unittest.mock import MagicMock


def _install_ros_mocks():
    """Регистрирует заглушки для всех ROS2 пакетов в sys.modules."""

    # ── rclpy ──────────────────────────────────────────────────────────────
    mock_rclpy = MagicMock()

    # Node — базовый класс DialogueNode
    class FakeNode:
        """Минимальная заглушка rclpy.node.Node без реальных сокетов."""

        def __init__(self, name, **kwargs):
            self._name = name
            self._logger = MagicMock()
            self._logger.info = MagicMock()
            self._logger.warning = MagicMock()
            self._logger.error = MagicMock()
            self._logger.debug = MagicMock()

        def get_logger(self):
            return self._logger

        def declare_parameter(self, name, default=None):
            return MagicMock()

        def get_parameter(self, name):
            p = MagicMock()
            p.value = None
            return p

        def create_publisher(self, *args, **kwargs):
            return MagicMock()

        def create_subscription(self, *args, **kwargs):
            return MagicMock()

        def create_timer(self, *args, **kwargs):
            return MagicMock()

        def create_service(self, *args, **kwargs):
            return MagicMock()

        def create_client(self, *args, **kwargs):
            return MagicMock()

        def get_name(self):
            return self._name

    mock_rclpy_node = MagicMock()
    mock_rclpy_node.Node = FakeNode

    mock_callback_groups = types.SimpleNamespace(
        ReentrantCallbackGroup=type("ReentrantCallbackGroup", (), {}),
    )

    mock_qos = types.SimpleNamespace(
        HistoryPolicy=types.SimpleNamespace(KEEP_LAST="KEEP_LAST"),
        ReliabilityPolicy=types.SimpleNamespace(RELIABLE="RELIABLE"),
        QoSProfile=lambda *args, **kwargs: MagicMock(),
    )

    # ── std_msgs, std_srvs ─────────────────────────────────────────────────
    mock_std_msgs = MagicMock()
    mock_std_msgs_msg = MagicMock()
    mock_std_msgs_msg.String = MagicMock
    mock_std_msgs_msg.Bool = MagicMock

    mock_std_srvs = MagicMock()
    mock_std_srvs_srv = MagicMock()
    mock_std_srvs_srv.Empty = MagicMock

    # ── rob_box_mcp_tools (опциональный) ───────────────────────────────────
    mock_mcp = MagicMock()
    mock_mcp_adapter = MagicMock()

    # ── OpenAI Agents SDK / HTTP clients (опциональные для unit tests) ─────
    def _function_tool(func=None, **kwargs):
        if func is None:
            return lambda wrapped: wrapped
        return func

    class FakeRunner:
        @staticmethod
        async def run(*args, **kwargs):
            return MagicMock(final_output="")

    fake_agents = types.SimpleNamespace(
        Agent=MagicMock,
        Runner=FakeRunner,
        function_tool=_function_tool,
    )
    fake_agents_exceptions = types.SimpleNamespace(
        MaxTurnsExceeded=type("MaxTurnsExceeded", (Exception,), {}),
    )
    fake_agents_items = types.SimpleNamespace(
        ToolCallItem=type("ToolCallItem", (), {}),
    )
    fake_agents_model_settings = types.SimpleNamespace(
        ModelSettings=lambda *args, **kwargs: MagicMock(),
    )
    fake_agents_openai_model = types.SimpleNamespace(
        OpenAIChatCompletionsModel=MagicMock,
    )
    fake_httpx = types.SimpleNamespace(
        Timeout=lambda *args, **kwargs: MagicMock(),
    )
    fake_openai = types.SimpleNamespace(
        APIConnectionError=type("APIConnectionError", (Exception,), {}),
        APIStatusError=type("APIStatusError", (Exception,), {}),
        APITimeoutError=type("APITimeoutError", (Exception,), {}),
        AuthenticationError=type("AuthenticationError", (Exception,), {}),
        AsyncOpenAI=MagicMock,
    )

    mocks = {
        "rclpy": mock_rclpy,
        "rclpy.node": mock_rclpy_node,
        "rclpy.callback_groups": mock_callback_groups,
        "rclpy.qos": mock_qos,
        "std_msgs": mock_std_msgs,
        "std_msgs.msg": mock_std_msgs_msg,
        "std_srvs": mock_std_srvs,
        "std_srvs.srv": mock_std_srvs_srv,
        "rob_box_mcp_tools": mock_mcp,
        "rob_box_mcp_tools.llm_adapter": mock_mcp_adapter,
        "agents": fake_agents,
        "agents.exceptions": fake_agents_exceptions,
        "agents.items": fake_agents_items,
        "agents.model_settings": fake_agents_model_settings,
        "agents.models.openai_chatcompletions": fake_agents_openai_model,
        "httpx": fake_httpx,
        "openai": fake_openai,
    }
    for name, mock in mocks.items():
        sys.modules.setdefault(name, mock)


_install_ros_mocks()
