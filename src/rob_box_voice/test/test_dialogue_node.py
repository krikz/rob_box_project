#!/usr/bin/env python3
"""
test_dialogue_node.py — Реальные тесты DialogueNode (FA-5, issue #833).

⚠️ Историческая справка: раньше этот файл содержал 13+ пустых тест-методов
(тело — только ``pass``), что создавало ложное ощущение покрытия
(задокументировано в COVERAGE_REPORT.md). Все заглушки удалены.

Реальные unit-тесты текущего DialogueNode живут в
``test/unit/node/test_dialogue_node.py`` (53 теста, запускаются в CI).
Этот файл остаётся как legacy-точка входа: он подключает те же моки
rclpy/openai, что и ``test/unit/node/conftest.py``, и ре-экспортирует
реальные тесты, чтобы файл можно было запустить и отдельно:

    python3 -m pytest test/test_dialogue_node.py
"""

import sys
import types
from unittest.mock import MagicMock

# ─────────────────────────────────────────────────────────────────────────────
#  Mocks ROS2/openai — тот же набор, что в test/unit/node/conftest.py.
#  Нужен ДО импорта rob_box_voice.dialogue_node (он тянет rclpy и
#  rob_box_harness.providers → rob_box_llm.providers.deepseek → openai).
# ─────────────────────────────────────────────────────────────────────────────

def _install_ros_mocks():
    """Регистрирует заглушки для всех ROS2 пакетов в sys.modules."""

    # ── rclpy ──────────────────────────────────────────────────────────────
    class FakeNode:
        """Минимальная заглушка rclpy.node.Node без реальных сокетов."""

        def __init__(self, name, **kwargs):
            self._name = name
            self._logger = MagicMock()

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

    mock_std_msgs_msg = MagicMock()
    mock_std_msgs_msg.String = MagicMock
    mock_std_msgs_msg.Bool = MagicMock

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
        "rclpy": MagicMock(),
        "rclpy.node": mock_rclpy_node,
        "rclpy.callback_groups": mock_callback_groups,
        "rclpy.qos": mock_qos,
        "std_msgs": MagicMock(),
        "std_msgs.msg": mock_std_msgs_msg,
        "std_srvs": MagicMock(),
        "std_srvs.srv": MagicMock(),
        "rob_box_mcp_tools": MagicMock(),
        "rob_box_mcp_tools.llm_adapter": MagicMock(),
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

# ─────────────────────────────────────────────────────────────────────────────
#  Ре-экспорт реальных тестов из unit/node (FA-5: убраны пустые pass-заглушки)
# ─────────────────────────────────────────────────────────────────────────────
from test.unit.node.test_dialogue_node import *  # noqa: F401,F403,E402


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
