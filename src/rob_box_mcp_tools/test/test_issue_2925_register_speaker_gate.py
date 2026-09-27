"""Регресс issue #2925: ``register_speaker`` согласуется с тем, что человек
сказал о себе в реплике.

Живой случай (акт 2, run 35933157262)::

    [stt_node]        ✅ ПРИНЯТО: Робот, запомни про меня: Дарья болит за Спартак …
    [mcp_server]      register_speaker с параметрами {'name': 'Дарья', 'utterance_id': 'ed597ace38e7'}
    [speaker_id_node] ⚠️ Speaker 'Дарья' — голос похож на уже известного 'Борис' (score=0.796 …)
    [dialogue_node]   spoken='Сегодня по голосу запомнил троих: Саша, Борис и Дарья.'

Путь тот же, что на роботе (``tool_provider: ros_mcp``): контекст хода
dialogue_node → ``LLMToolCallAdapter`` (скрытые аргументы, подписанный
``/mcp/execute``) → ``MCPToolRegistry`` на узле mcp_server → настоящий
``RegisterSpeakerTool`` → ``/voice/speaker/register``. Харнесс — из
``test_register_speaker_ros_mcp_utterance.py`` (#2842).
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from typing import Any, Dict, Optional

import pytest

_HERE = Path(__file__).resolve().parent


def _load_harness():
    name = "_rsp_2842_harness_for_2925"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(
        name, _HERE / "test_register_speaker_ros_mcp_utterance.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_h = _load_harness()
_TOKEN = _h._TOKEN
UTT = "ed597ace38e7"  # из лога issue #2925


class _DialogueNode2925(_h._Node):
    """dialogue_node с контекстом хода #2925 (зеркало
    ``DialogueNode._mcp_turn_context``)."""

    def __init__(self, bus, context: Dict[str, Any]) -> None:
        super().__init__("dialogue_node", bus)
        self._ctx = context

    def _mcp_turn_context(self) -> dict:
        return dict(self._ctx)


@pytest.fixture(autouse=True)
def _auth_token(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ROB_BOX_MCP_TOKEN", _TOKEN)


def _register(context: Dict[str, Any], llm_args: Dict[str, Any]):
    bus = _h._Bus()
    server = _h._MCPServer(bus)
    dialogue = _DialogueNode2925(bus, context)
    adapter = _h._production_adapter(dialogue)
    result = adapter.execute_tool_call_sync(
        "register_speaker", llm_args, timeout=1.0
    )
    registered = [
        json.loads(d) for d in bus.published.get("/voice/speaker/register", [])
    ]
    return result, registered, server


def _ctx(
    *,
    intro: Optional[str] = None,
    sent: bool = False,
    known: Optional[str] = None,
) -> Dict[str, Any]:
    return {
        "utterance_id": UTT,
        "self_intro_name": intro,
        "intro_registered": sent,
        "known_speaker_name": known,
    }


# ── Случай 3: «запомни про меня: Дарья …», голос уверенно Борис ───────────


def test_darya_on_non_intro_phrase_with_confident_boris_not_registered():
    result, registered, server = _register(_ctx(known="Борис"), {"name": "Дарья"})

    assert registered == [], "третий профиль «Дарья» заводиться не должен"
    assert result.get("success") is False, result
    # Нет слова pending — SchedulerToolExecutor (#2913) не ждёт ack.
    assert "pending" not in json.dumps(result, ensure_ascii=False)
    # Отказ — в лог mcp_server, не озвучка «не расслышал».
    assert any("#2925 ОТКАЗ" in line for line in server.node.get_logger().lines)


def test_llm_cannot_forge_turn_context():
    """LLM не может сама прислать «я представлялся» — ход побеждает."""
    _result, registered, _server = _register(
        _ctx(known="Борис"),
        {"name": "Дарья", "self_intro_name": "Дарья",
         "known_speaker_name": None},
    )
    assert registered == []


# ── Не сломано: нормальная регистрация и ADR-0127 ─────────────────────────


def test_unknown_voice_registers_as_before():
    """Незнакомый голос — регистрация идёт (в т.ч. ответ одним именем)."""
    result, registered, _server = _register(_ctx(), {"name": "Дарья"})
    assert registered == [{"name": "Дарья", "utterance_id": UTT}]
    assert "pending" in json.dumps(result, ensure_ascii=False)


def test_real_intro_with_other_name_still_registers_adr0127():
    """«я Борис» голосом, похожим на Сашу, — настоящее представление:
    регистрация уходит, конфликт разбирает ADR-0127/#2828."""
    _result, registered, _server = _register(
        _ctx(intro="Борис", known="Саша"), {"name": "Борис"}
    )
    assert registered == [{"name": "Борис", "utterance_id": UTT}]


def test_known_speaker_same_name_goes_to_2863_path():
    """Повторный register_speaker узнанного Бориса — дальше «уже знаю»
    решает speaker_id_node (#2863), гейт не мешает."""
    _result, registered, _server = _register(_ctx(known="Борис"), {"name": "борис"})
    assert registered == [{"name": "Борис", "utterance_id": UTT}]


def test_no_turn_context_keeps_old_behaviour():
    """Telegram/синтетика/старый dialogue_node — гейт молчит."""
    _result, registered, _server = _register(
        {"utterance_id": UTT}, {"name": "Дарья"}
    )
    assert registered == [{"name": "Дарья", "utterance_id": UTT}]


# ── Случай 1: робот уже отправил регистрацию по представлению ─────────────


def test_intro_already_sent_by_robot_is_not_duplicated():
    result, registered, _server = _register(
        _ctx(intro="Борис", sent=True), {"name": "Борис"}
    )
    assert registered == []
    assert result.get("success") is True, result
    assert "pending" not in json.dumps(result, ensure_ascii=False)


def test_hidden_args_not_in_llm_schema():
    tool = _h.RegisterSpeakerTool(_h._Node("mcp_server", _h._Bus()))
    props = tool.to_openai_tool_format()["function"]["parameters"].get(
        "properties", {}
    )
    for hidden in ("self_intro_name", "intro_registered", "known_speaker_name"):
        assert hidden not in props
