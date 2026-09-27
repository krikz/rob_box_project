"""Регресс issue #2842: register_speaker через ros_mcp теряет utterance_id.

Прод-путь (``tool_provider: ros_mcp``)::

    dialogue_node ── LLMToolCallAdapter ──/mcp/execute──▶ mcp_server
                                                           └─ RegisterSpeakerTool(node=mcp_server)
                                                              └─/voice/speaker/register──▶ speaker_id_node

Тул исполняется в ДРУГОМ процессе: ``self.node`` у него — mcp_server, а не
dialogue_node, поэтому ``getattr(self.node, "_current_turn_utterance_id")``
всегда ``None`` и speaker_id_node честно отказывает (``no_utterance_context``).
На роботе это ломало регистрацию голоса целиком (#2833 → #2842).

Тест моделирует именно этот путь, а не in-process: настоящий
``LLMToolCallAdapter`` на узле dialogue_node формирует и подписывает запрос,
«mcp_server» проверяет подпись и исполняет настоящий ``RegisterSpeakerTool``
через настоящий ``MCPToolRegistry`` на ОТДЕЛЬНОМ узле без атрибутов хода, и
проверяется JSON, опубликованный в ``/voice/speaker/register``.
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import sys
import types
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import pytest

_PKG_ROOT = Path(__file__).resolve().parents[1] / "rob_box_mcp_tools"
_TOKEN = "test-token-2842-not-real"
_TURN_UTTERANCE_ID = "c9b74f4044de"  # из лога issue #2842


# ---------------------------------------------------------------------------
# rclpy / std_msgs — заглушки, если ROS нет (dev-машина, CI без ROS).
# ---------------------------------------------------------------------------


def _ensure_ros_stubs() -> None:
    try:
        importlib.import_module("rclpy")
        importlib.import_module("std_msgs.msg")
        return
    except ImportError:
        pass

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object  # type: ignore[attr-defined]
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kw: kw  # type: ignore[attr-defined]
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(  # type: ignore[attr-defined]
        RELIABLE="reliable", BEST_EFFORT="best_effort"
    )
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")  # type: ignore[attr-defined]
    rclpy_cb = types.ModuleType("rclpy.callback_groups")
    rclpy_cb.ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})  # type: ignore[attr-defined]

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self, *a, **kw):
            self.data = ""

    std_msgs_msg.String = String  # type: ignore[attr-defined]
    std_msgs.msg = std_msgs_msg  # type: ignore[attr-defined]

    for mod in (rclpy, rclpy_node, rclpy_qos, rclpy_cb, std_msgs, std_msgs_msg):
        sys.modules.setdefault(mod.__name__, mod)


_ensure_ros_stubs()

from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter  # noqa: E402
from rob_box_mcp_tools.mcp_auth import RequestAuthenticator  # noqa: E402
from rob_box_mcp_tools.registry import MCPToolRegistry  # noqa: E402


def _load_register_speaker_tool():
    """Настоящий RegisterSpeakerTool без ``tools/__init__`` (тот тянет nav2/rclpy).

    Модуль грузится под своим именем внутри пакета ``rob_box_mcp_tools.tools``,
    чтобы относительные импорты ``..base`` / ``..voice_state`` резолвились в
    настоящие модули пакета.
    """
    name = "rob_box_mcp_tools.tools._dialogue_isolated_2842"
    spec = importlib.util.spec_from_file_location(
        name, _PKG_ROOT / "tools" / "dialogue.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.RegisterSpeakerTool


RegisterSpeakerTool = _load_register_speaker_tool()


# ---------------------------------------------------------------------------
# Фейковый ROS-граф: два узла в «разных процессах», общий только топик.
# ---------------------------------------------------------------------------


class _Logger:
    def __init__(self) -> None:
        self.lines: List[str] = []

    def info(self, msg: str) -> None:
        self.lines.append(msg)

    warning = warn = error = debug = info


class _Publisher:
    def __init__(self, topic: str, bus: "_Bus") -> None:
        self.topic = topic
        self._bus = bus
        self.sent: List[str] = []

    def publish(self, msg: Any) -> None:
        self.sent.append(msg.data)
        self._bus.deliver(self.topic, msg)


class _Bus:
    def __init__(self) -> None:
        self.subscribers: Dict[str, List[Callable[[Any], None]]] = {}
        self.published: Dict[str, List[str]] = {}

    def deliver(self, topic: str, msg: Any) -> None:
        self.published.setdefault(topic, []).append(msg.data)
        for callback in self.subscribers.get(topic, []):
            callback(msg)


class _Node:
    """Минимальный rclpy.Node. Атрибутов хода у него НЕТ (как у mcp_server)."""

    def __init__(self, name: str, bus: _Bus) -> None:
        self._name = name
        self._bus = bus
        self._logger = _Logger()

    def get_name(self) -> str:
        return self._name

    def get_logger(self) -> _Logger:
        return self._logger

    def create_publisher(self, _msg_type, topic, _qos=10, **_kw):
        return _Publisher(topic, self._bus)

    def create_subscription(self, _msg_type, topic, callback, _qos=10, **_kw):
        self._bus.subscribers.setdefault(topic, []).append(callback)
        return object()


class _DialogueNode(_Node):
    """Узел dialogue_node: знает utterance_id ТЕКУЩЕГО хода."""

    def __init__(self, bus: _Bus, utterance_id: Optional[str]) -> None:
        super().__init__("dialogue_node", bus)
        self._current_turn_utterance_id = utterance_id

    # Зеркало DialogueNode._mcp_turn_context (rob_box_voice/dialogue_node.py).
    def _mcp_turn_context(self) -> dict:
        return {"utterance_id": self._current_turn_utterance_id}


class _MCPServer:
    """Сторона mcp_server: проверка подписи → registry.execute → /mcp/result.

    Повторяет существенное из ``MCPServer.on_execute_request``: HMAC-гард и
    ``self.registry.execute(tool_name, **parameters)``.
    """

    def __init__(self, bus: _Bus) -> None:
        self.node = _Node("mcp_server", bus)
        self.registry = MCPToolRegistry()
        self.registry.register(RegisterSpeakerTool(self.node))
        self.authenticator = RequestAuthenticator(_TOKEN, sender="mcp_server")
        self.requests: List[Dict[str, Any]] = []
        self._result_pub = self.node.create_publisher(None, "/mcp/result")
        self.node.create_subscription(None, "/mcp/execute", self.on_execute_request)

    def on_execute_request(self, msg: Any) -> None:
        request = json.loads(msg.data)
        self.requests.append(request)
        ok, err = self.authenticator.verify(request)
        assert ok, f"mcp_server отклонил бы запрос: {err}"
        result = self.registry.execute(
            request["tool_name"], **request.get("parameters", {})
        )
        out = types.SimpleNamespace(
            data=json.dumps(
                {
                    "tool_name": request["tool_name"],
                    "request_id": request["request_id"],
                    "result": result.to_dict(),
                },
                ensure_ascii=False,
            )
        )
        self._result_pub.publish(out)


def _production_adapter(dialogue: _DialogueNode) -> LLMToolCallAdapter:
    """Адаптер так, как его строит DialogueNode._build_tool_provider.

    До #2842 у ``LLMToolCallAdapter`` не было ``turn_context`` и
    dialogue_node строил его как ``LLMToolCallAdapter(self)``. Тогда
    строим так же, чтобы на старом коде тест показывал сам баг
    (``utterance_id=None`` в /voice/speaker/register), а не TypeError.
    """
    try:
        return LLMToolCallAdapter(dialogue, turn_context=dialogue._mcp_turn_context)
    except TypeError:
        return LLMToolCallAdapter(dialogue)


@pytest.fixture(autouse=True)
def _auth_token(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ROB_BOX_MCP_TOKEN", _TOKEN)


def _register_via_ros_mcp(
    turn_utterance_id: Optional[str], llm_args: Dict[str, Any]
) -> tuple:
    bus = _Bus()
    server = _MCPServer(bus)
    dialogue = _DialogueNode(bus, turn_utterance_id)
    adapter = _production_adapter(dialogue)
    result = adapter.execute_tool_call_sync("register_speaker", llm_args, timeout=1.0)
    registered = [json.loads(d) for d in bus.published.get("/voice/speaker/register", [])]
    return result, registered, server


def test_mcp_server_node_has_no_turn_attribute() -> None:
    """Предпосылка бага: у узла, где исполняется тул, поля хода нет."""
    bus = _Bus()
    server = _MCPServer(bus)
    assert not hasattr(server.node, "_current_turn_utterance_id")


def test_register_via_ros_mcp_carries_turn_utterance_id() -> None:
    result, registered, server = _register_via_ros_mcp(
        _TURN_UTTERANCE_ID, {"name": "Саша"}
    )

    assert result.get("success") is True, result
    assert registered == [{"name": "Саша", "utterance_id": _TURN_UTTERANCE_ID}], (
        "в /voice/speaker/register должен уехать utterance_id ХОДА, "
        f"иначе speaker_id_node ответит no_utterance_context; получено: {registered}"
    )
    # utterance_id едет внутри подписанных parameters (HMAC покрывает их).
    assert server.requests[0]["parameters"]["utterance_id"] == _TURN_UTTERANCE_ID


def test_llm_supplied_utterance_id_is_overwritten_by_turn() -> None:
    """LLM не может подсунуть чужую фразу: значение хода побеждает."""
    _result, registered, _server = _register_via_ros_mcp(
        _TURN_UTTERANCE_ID, {"name": "Саша", "utterance_id": "forged000000"}
    )

    assert registered == [{"name": "Саша", "utterance_id": _TURN_UTTERANCE_ID}]


def test_llm_supplied_utterance_id_is_stripped_without_turn() -> None:
    """Хода нет (utterance_id=None) — подделка LLM вырезается, не уезжает."""
    _result, registered, server = _register_via_ros_mcp(
        None, {"name": "Саша", "utterance_id": "forged000000"}
    )

    assert "utterance_id" not in server.requests[0]["parameters"]
    assert registered == [{"name": "Саша", "utterance_id": None}]


def test_utterance_id_is_hidden_from_llm_schema() -> None:
    """Скрытый аргумент не попадает в LLM-схему тула."""
    tool = RegisterSpeakerTool(_Node("mcp_server", _Bus()))
    schema = tool.to_openai_tool_format()["function"]["parameters"]
    assert "utterance_id" not in schema.get("properties", {})
    assert {p.name for p in tool.parameters} == {"name", "old_name"}


def test_other_tools_parameters_untouched() -> None:
    from rob_box_mcp_tools.llm_adapter import apply_turn_context

    args = {"text": "привет", "utterance_id": "x"}
    assert apply_turn_context(
        "speak_text", args, lambda: {"utterance_id": _TURN_UTTERANCE_ID}
    ) == args
