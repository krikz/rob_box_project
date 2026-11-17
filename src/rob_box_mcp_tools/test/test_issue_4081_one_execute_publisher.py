"""#4081: один издатель ``/mcp/execute`` на ноду, чужие ответы ``/mcp/result`` адаптер не копит.

Рантайм-аудит 27.10 (``duplicate_publishers /mcp/execute /dialog_node``): ``dialog_node`` строит два
``LLMToolCallAdapter`` (Личность и агент чата Telegram), и каждый заводил свой издатель. Решение Шифу 28.10: один на
ноду, как ``speech_client`` в ``architecture/ownership.yml``. Подпись (``sender``), срез тулов и ожидание ответов остаются
за адаптером - издатель их не определяет. Подписку на ``/mcp/result`` слышат все адаптеры ноды; в ответе отправителя нет,
хозяин запроса - тот, кто его выпустил и ждёт: чужой ответ он молча пропускает (раньше он оседал в кэшах навсегда).
"""

from __future__ import annotations

import asyncio
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import pytest

_HERE = Path(__file__).resolve().parent


def _load_harness():
    name = "_rsp_2842_harness_for_4081"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, _HERE / "test_register_speaker_ros_mcp_utterance.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_h = _load_harness()

from rob_box_mcp_tools import llm_adapter  # noqa: E402
from rob_box_mcp_tools.base import ToolExecutionType, shared_publisher  # noqa: E402
from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter  # noqa: E402
from rob_box_mcp_tools.mcp_auth import PERSONALITY_SENDER, TELEGRAM_SENDER  # noqa: E402

EXECUTE_TOPIC = "/mcp/execute"
RESULT_TOPIC = "/mcp/result"


@pytest.fixture(autouse=True)
def _auth_token(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ROB_BOX_MCP_TOKEN", _h._TOKEN)


class _CountingNode(_h._Node):
    """Узел шины с учётом: сколько издателей на какой топик завёл код (``create_publisher`` вызывался)."""

    def __init__(self, bus, name: str = "dialog_node") -> None:
        super().__init__(name, bus)
        self.created: List[str] = []

    def create_publisher(self, msg_type, topic, qos=10, **kw):
        self.created.append(topic)
        return super().create_publisher(msg_type, topic, qos, **kw)


def _two_adapters():
    bus = _h._Bus()
    node = _CountingNode(bus)
    personality = LLMToolCallAdapter(node, sender=PERSONALITY_SENDER)
    chat = LLMToolCallAdapter(node, sender=TELEGRAM_SENDER)
    return bus, node, personality, chat


def _deliver(bus, request_id: str, result: Dict[str, Any] | None = None) -> None:
    """Ответ ``mcp_server`` на ``/mcp/result``: слышат все подписчики шины."""
    msg = _string_msg()
    msg.data = json.dumps({"tool_name": "t", "request_id": request_id, "result": result or {"success": True}})
    bus.deliver(RESULT_TOPIC, msg)


def _string_msg():
    from std_msgs.msg import String

    return String()


def test_two_adapters_of_one_node_share_one_execute_publisher():
    """Тест из карточки: два адаптера на одной ноде -> один ``create_publisher('/mcp/execute')``."""
    _bus, node, personality, chat = _two_adapters()
    assert node.created.count(EXECUTE_TOPIC) == 1
    assert personality.execute_pub is chat.execute_pub
    assert personality.async_executor.execute_pub is chat.async_executor.execute_pub is personality.execute_pub


def test_requests_of_both_adapters_go_through_the_one_publisher_under_their_own_signature():
    bus, _node, personality, chat = _two_adapters()
    personality.execute_tool_call("play_animation", {"animation": "hello"})
    chat.execute_tool_call("get_current_time", {})
    sent = [json.loads(raw) for raw in bus.published[EXECUTE_TOPIC]]
    assert [(r["tool_name"], r["auth"]["sender"]) for r in sent] == [
        ("play_animation", PERSONALITY_SENDER), ("get_current_time", TELEGRAM_SENDER)]
    assert personality.execute_pub.sent == bus.published[EXECUTE_TOPIC], "обе подписанные пачки - в одном издателе"


def test_other_nodes_keep_their_own_publisher():
    """Общий издатель - на ноду, а не на процесс: ``avatar_supervisor`` и ``dialog_node`` не делят его."""
    bus = _h._Bus()
    dialog, supervisor = _CountingNode(bus, "dialog_node"), _CountingNode(bus, "avatar_supervisor")
    a, b = LLMToolCallAdapter(dialog), LLMToolCallAdapter(supervisor)
    assert a.execute_pub is not b.execute_pub
    assert dialog.created.count(EXECUTE_TOPIC) == supervisor.created.count(EXECUTE_TOPIC) == 1


def test_adapter_takes_its_publisher_from_the_one_shared_publisher_helper_and_keeps_no_cache_of_its_own():
    """#4159 п.4 (ревью PR #4147): издатель адаптера - ``base.shared_publisher`` (#3108), на который рассчитан
    ``tools/architecture_audit.py`` (``ROS_HELPER_CALLS``); второго кэша издателя в ``llm_adapter`` нет, и на ноду он
    своих атрибутов не кладёт."""
    bus = _h._Bus()
    node = _CountingNode(bus)
    first = LLMToolCallAdapter(node, sender=PERSONALITY_SENDER)
    second = LLMToolCallAdapter(node, sender=TELEGRAM_SENDER)
    assert first.execute_pub is second.execute_pub
    assert node.created.count(EXECUTE_TOPIC) == 1
    assert not hasattr(node, "_mcp_execute_publisher"), "кэш издателя - у base.shared_publisher, не атрибут ноды"
    assert not hasattr(llm_adapter, "shared_execute_publisher") and not hasattr(llm_adapter, "_EXECUTE_PUB_ATTR")


def test_node_that_cannot_be_cached_gets_its_own_publisher_per_adapter_like_the_helper_says():
    """``base.shared_publisher`` без weak-кэша (узел без ``__weakref__``) честно заводит издателя на каждый вызов."""

    class _Slotted:
        __slots__ = ("created",)

        def __init__(self) -> None:
            self.created = []

        def create_publisher(self, msg_type, topic, qos):
            self.created.append(topic)
            return object()

    slotted = _Slotted()
    shared_publisher(slotted, type(_string_msg()), EXECUTE_TOPIC, 10)
    shared_publisher(slotted, type(_string_msg()), EXECUTE_TOPIC, 10)
    assert slotted.created == [EXECUTE_TOPIC, EXECUTE_TOPIC]


def test_each_adapter_gets_only_its_own_answer_and_the_other_one_caches_nothing():
    """Оба адаптера слышат ``/mcp/result``; ответ на запрос Личности забирает Личность, чат его не кэширует."""
    bus, _node, personality, chat = _two_adapters()
    got: Dict[str, List[Dict[str, Any]]] = {"personality": [], "chat": []}
    rid_p = personality.execute_tool_call("play_animation", {}, callback=got["personality"].append)
    rid_c = chat.execute_tool_call("get_current_time", {}, callback=got["chat"].append)
    _deliver(bus, rid_p, {"success": True, "message": "для личности"})
    assert [r["result"]["message"] for r in got["personality"]] == ["для личности"] and got["chat"] == []
    assert chat.results_cache == {} and chat.async_executor.results_cache == {}
    _deliver(bus, rid_c, {"success": True, "message": "для чата"})
    assert [r["result"]["message"] for r in got["chat"]] == ["для чата"] and len(got["personality"]) == 1
    assert personality.results_cache == {} and personality.async_executor.results_cache == {}


def test_foreign_and_late_answers_are_not_cached_forever():
    """Ответ другой ноды и запоздалый ответ (синхронный вызов уже отпал по таймауту) в кэшах не оседают."""
    bus, _node, personality, chat = _two_adapters()
    _deliver(bus, "ffffffff-0000-4000-8000-00000000dead")  # ответ чужой ноды: слышат оба адаптера
    result = personality.execute_tool_call_sync("get_current_time", {}, timeout=0.05)
    assert result["success"] is False and "Timeout" in result["error"]
    late = json.loads(bus.published[EXECUTE_TOPIC][-1])["request_id"]
    _deliver(bus, late)
    for adapter in (personality, chat):
        assert adapter.results_cache == {} and adapter.async_executor.results_cache == {}


def test_sync_call_still_gets_its_answer_when_another_adapter_hears_it_too():
    bus, _node, personality, chat = _two_adapters()

    def answer(topic: str, msg: Any) -> None:
        if topic == EXECUTE_TOPIC:
            _deliver(bus, json.loads(msg.data)["request_id"], {"success": True, "message": "готово"})

    bus.subscribers.setdefault(EXECUTE_TOPIC, []).append(lambda msg: answer(EXECUTE_TOPIC, msg))
    assert personality.execute_tool_call_sync("get_current_time", {}, timeout=2.0)["message"] == "готово"
    assert personality.results_cache == {} and chat.results_cache == {}


def test_async_executor_waits_only_for_what_it_issued():
    bus, _node, personality, chat = _two_adapters()
    executor = personality.async_executor

    async def scenario() -> Dict[str, Any]:
        def answer(msg: Any) -> None:  # mcp_server отвечает на запрос тула сразу
            _deliver(bus, json.loads(msg.data)["request_id"], {"success": True, "message": "асинхронно"})

        bus.subscribers.setdefault(EXECUTE_TOPIC, []).append(answer)
        return await executor.execute_tool_async("get_current_time", {}, ToolExecutionType.FAST)

    assert asyncio.run(scenario())["message"] == "асинхронно"
    assert executor.results_cache == {} and not executor.expects("any") and chat.async_executor.results_cache == {}
    _deliver(bus, "ffffffff-0000-4000-8000-00000000beef")
    assert executor.results_cache == {}, "чужой ответ в кэш исполнителя не попадает"
