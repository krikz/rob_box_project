"""PR-8 ч.8e2 (#3702): ROS-хост диалога v2 ``dialog_node`` на заглушках rclpy (conftest) и фейковых LLM/тулах."""

from __future__ import annotations

import asyncio
import concurrent.futures
import json
import socket
import sys
import threading
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
try:  # настоящий rosidl-пакет (CI) или заглушка сервиса DialogControl (Windows)
    import rob_box_dialog_msgs.srv  # noqa: F401
except ImportError:
    class _Request:
        ACTION_HOLD, ACTION_RESUME, ACTION_RESET = 0, 1, 2

        def __init__(self, action=0, set_by="", ttl_s=0.0):
            self.action, self.set_by, self.ttl_s = action, set_by, ttl_s

    _srv = types.ModuleType("rob_box_dialog_msgs.srv")
    _srv.DialogControl = type("DialogControl", (), {"Request": _Request,
                                                    "Response": lambda: types.SimpleNamespace()})
    sys.modules["rob_box_dialog_msgs.srv"] = _srv
    sys.modules["rob_box_dialog_msgs"].srv = _srv

from rob_box_dialog_msgs.srv import DialogControl  # noqa: E402
from rob_box_llm.provider import LLMResponse, ToolResult  # noqa: E402
from rob_box_voice import dialog_node as dn  # noqa: E402

TIME = type(M.TurnEvent().stamp)


class _LLM:
    def __init__(self, responses):
        self.responses, self.calls = list(responses), 0
        self.answered_by = "deepseek"

    async def complete(self, messages, tools=(), settings=None):
        self.calls += 1
        return self.responses.pop(0)


class _Tools:
    def __init__(self):
        self.calls = []

    async def execute(self, call):
        self.calls.append(call.name)
        return ToolResult(tool_call_id=call.id, content="ok", data={"status": "done"})


@pytest.fixture()
def node(monkeypatch):
    llm = _LLM([LLMResponse(content="Привет, я робот.")])
    tools = _Tools()
    monkeypatch.setattr(dn, "build_llm", lambda node, counter: (llm, "Ты робот."))
    monkeypatch.setattr(dn, "build_tools", lambda node: tools)
    made = dn.DialogNode()
    stamp = TIME(sec=1, nanosec=0)
    made.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    made.test_llm, made.test_tools = llm, tools
    yield made
    made.shutdown()


def _topics(made, kind):
    return [t for t, _ in getattr(made, kind)]


def _utterance(text):
    msg = M.Utterance()
    msg.text, msg.source, msg.utterance_id = text, "respeaker", "u1"
    return msg


def _published(made, topic):
    pubs = [p for t, p in made._publishers if t == topic]
    return [c.args[0] for p in pubs for c in p.publish.call_args_list]


def test_wiring_v2_topics_and_no_v1_dialogue_topics(node):
    subs = set(_topics(node, "_subscribers"))
    assert {"/voice/stt/utterance", "/voice/speaker/result", "/voice/speech/event", "/voice/sound/event",
            "/voice/music/state", "/voice/music/event", "/voice/providers/state",
            "/dialog/interrupt_request"} <= subs
    pubs = set(_topics(node, "_publishers"))
    assert {"/dialog/session", "/dialog/intent"} <= pubs  # /dialog/plan_event — plan_host в build_tools (подменён)
    for legacy in ("/voice/dialogue/response", "/voice/tts/request", "/dialogue/control", "/dialogue/control_ack"):
        assert legacy not in subs | pubs, legacy
    assert node.engine_flag == "v1"  # параметр есть; какую ноду стартовать, решает launch по dialog_engine.yaml


def test_addressed_utterance_runs_turn_and_speaks(node):
    node._on_utterance(_utterance("Робот, как дела"))
    node.last_turn.result(timeout=10)
    said = _published(node, "/voice/speech/request")
    assert [s.text for s in said] == ["<speak>Привет, я робот.</speak>"]
    assert node.test_llm.calls == 1


def test_unaddressed_utterance_goes_to_backlog_without_turn(node):
    node._on_utterance(_utterance("ну вот покажи анимацию"))
    assert node.last_turn is None and node.test_llm.calls == 0
    assert [e.text for e in node._backlog._entries] == ["ну вот покажи анимацию"]


def test_speech_end_puts_spoken_part_into_window(node):
    node._on_utterance(_utterance("Робот, как дела"))
    node.last_turn.result(timeout=10)
    said = _published(node, "/voice/speech/request")[0]
    event = M.SpeechEvent()
    event.kind, event.speech_id, event.agent, event.spoken_chars = M.SpeechEvent.KIND_FINISHED, said.speech_id, \
        "personality", 7
    node._on_speech_event(event)
    assert [e.content for e in node._engine.log.window()] == ["как дела", "Привет,"]


def test_control_reset_hold_resume(node):
    response = types.SimpleNamespace()
    out = node._on_control(DialogControl.Request(action=DialogControl.Request.ACTION_RESET, set_by="operator"),
                           response)
    assert out.ok and out.epoch == 1
    node._on_control(DialogControl.Request(action=DialogControl.Request.ACTION_HOLD, set_by="operator", ttl_s=60.0),
                     types.SimpleNamespace())
    assert node._session.snapshot.top("OperatorHold") is not None
    node._on_control(DialogControl.Request(action=DialogControl.Request.ACTION_RESUME, set_by="operator"),
                     types.SimpleNamespace())
    assert node._session.snapshot.top("OperatorHold") is None
    assert _published(node, "/dialog/session"), "снимок сессии latched-топиком на каждое изменение"


def test_music_state_goes_to_session_snapshot(node):
    msg = MagicMock()
    msg.data = json.dumps({"state": "playing", "track_id": "t9", "dj": {"enabled": True, "title": "Сет"}})
    node._on_music_state(msg)
    music = node._session.snapshot.music
    assert (music.state, music.track_id) == ("playing", "t9")
    assert node._session.snapshot.top("DJ") is not None


def test_clear_stop_interrupt_cancels_speech_of_epoch(node):
    msg = M.InterruptRequest()
    msg.text_partial, msg.utterance_id = "Робот, стоп", "u2"
    node._on_interrupt(msg)
    assert _published(node, "/voice/speech/cancel")


def test_e2e_db_outside_data_only(node, tmp_path):
    node._parameters["e2e_mode"] = True
    node._declared["e2e_mode"].value = True
    node._declared["e2e_db_path"].value = "/data/x.db"
    assert node._open_events() is None
    node._declared["e2e_db_path"].value = str(tmp_path / "e2e.db")
    events = node._open_events()
    assert events is not None and events.path.endswith("e2e.db")
    events.close()


def test_mcp_tools_sign_as_the_personality_not_as_the_ros_node(monkeypatch):
    """#3853: ``dialog_node`` подписывает MCP-запросы именем личности — иначе mcp_server режет все тулы v2."""
    import rob_box_mcp_tools.llm_adapter as adapter
    from rob_box_mcp_tools.mcp_auth import DEFAULT_ALLOWED_SENDERS

    seen = {}

    class _Stop(Exception):
        pass

    def fake_adapter(node, **kwargs):
        seen.update(kwargs)
        raise _Stop

    monkeypatch.setattr(adapter, "LLMToolCallAdapter", fake_adapter)
    with pytest.raises(_Stop):
        dn.build_tools(types.SimpleNamespace(utterance_id=""))
    assert seen["sender"] in DEFAULT_ALLOWED_SENDERS


# -- перебивание по эпохе (PR-9 #3703, ADR-0150 §6.2) ---------------------------------------------------------------


class _SilentServer:
    """HTTP-сервер, который принимает запрос и молчит (LLM «думает»): видно, закрыл ли клиент соединение."""

    def __init__(self):
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.got_request, self.closed = threading.Event(), threading.Event()
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        conn, _ = self.sock.accept()
        conn.settimeout(30)
        while True:
            data = conn.recv(4096)
            if not data:
                self.closed.set()  # клиент закрыл соединение — запрос к провайдеру оборван
                return
            self.got_request.set()


class _HttpLLM:
    """Провайдер, чей ``complete`` — настоящий HTTP-запрос httpx к молчащему серверу."""

    answered_by = "deepseek"

    def __init__(self, port):
        self.url, self.calls = f"http://127.0.0.1:{port}/v1/chat/completions", 0
        self.cancelled = threading.Event()

    async def complete(self, messages, tools=(), settings=None):
        import httpx

        self.calls += 1
        try:
            async with httpx.AsyncClient(timeout=60) as client:
                await client.post(self.url, json={"messages": []})
        except asyncio.CancelledError:
            self.cancelled.set()
            raise
        raise AssertionError("молчащий сервер не отвечает")


def _hang_node(monkeypatch, llm):
    tools = _Tools()
    monkeypatch.setattr(dn, "build_llm", lambda node, counter: (llm, "Ты робот."))
    monkeypatch.setattr(dn, "build_tools", lambda node: tools)
    made = dn.DialogNode()
    stamp = TIME(sec=1, nanosec=0)
    made.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    return made


def _drain(node):
    """Дать отменённым задачам хода доотмениться в цикле ноды, затем остановить её."""
    asyncio.run_coroutine_threadsafe(asyncio.sleep(0.2), node._loop).result(5)
    node.shutdown()


def _said(text, uid):
    msg = _utterance(text)
    msg.utterance_id = uid
    return msg


@pytest.mark.parametrize("word, boundary", [("стой", 1), ("хватит", 0)])
def test_stop_during_llm_turn_closes_http_request_and_bumps_epoch(monkeypatch, word, boundary):
    """«Робот, расскажи сказку» → LLM думает (HTTP-запрос висит) → «Робот, стой/хватит»: задача хода отменена,
    httpx закрыл соединение с провайдером, речь эпохи 0 отменена по границе слова, эпоха 1, ответа сказки нет."""
    server = _SilentServer()
    llm = _HttpLLM(server.port)
    node = _hang_node(monkeypatch, llm)
    try:
        node._on_utterance(_said("Робот, расскажи сказку", "u1"))
        story = node.last_turn
        assert server.got_request.wait(10), "запрос к провайдеру не ушёл"
        node._on_utterance(_said(f"Робот, {word}", "u2"))
        node.last_turn.result(timeout=10)
        assert story.cancelled() and llm.cancelled.is_set()
        assert server.closed.wait(10), "HTTP-соединение старой эпохи не закрыто"
        cancels = _published(node, "/voice/speech/cancel")
        assert cancels and all(c.epoch == 0 for c in cancels) and cancels[0].boundary == boundary
        assert node._session.snapshot.epoch == 1
        assert _published(node, "/voice/speech/request") == [], "речи старой эпохи нет"
        assert llm.calls == 1
    finally:
        _drain(node)


def test_interrupt_request_clear_stop_cancels_turn_in_flight(monkeypatch):
    server = _SilentServer()
    llm = _HttpLLM(server.port)
    node = _hang_node(monkeypatch, llm)
    try:
        node._on_utterance(_said("Робот, расскажи сказку", "u1"))
        story = node.last_turn
        assert server.got_request.wait(10)
        msg = M.InterruptRequest()
        msg.text_partial, msg.utterance_id = "Робот, стой", "u2"
        node._on_interrupt(msg)
        with pytest.raises(concurrent.futures.CancelledError):
            story.result(timeout=10)
        assert server.closed.wait(10)
        assert _published(node, "/voice/speech/cancel")[0].boundary == 1
    finally:
        _drain(node)


def test_interrupt_request_of_the_stop_phrase_does_not_cancel_its_own_turn(node):
    """Порядок топиков не гарантирован: запрос перебивания пришёл после реплики той же фразы — ход стопа живёт."""
    node._on_utterance(_said("Робот, стой", "u7"))
    own = node.last_turn
    msg = M.InterruptRequest()
    msg.text_partial, msg.utterance_id = "Робот, стой", "u7"
    node._on_interrupt(msg)
    own.result(timeout=10)
    assert not own.cancelled() and node._session.snapshot.epoch == 1


def test_not_a_stop_waits_for_turn_and_does_not_cancel_speech(node):
    msg = M.InterruptRequest()
    msg.text_partial, msg.utterance_id = "Робот, а который час", "u3"
    node._on_interrupt(msg)
    assert _published(node, "/voice/speech/cancel") == []


def test_cancel_goes_through_fallback_chain_without_asking_next_provider(monkeypatch):
    """Отмена хода не считается сбоем провайдера: цепочка ``HealthAwareFallbackLLM`` не зовёт следующего (иначе
    «стой» запустил бы второй HTTP-запрос старой эпохи) и не метит первого больным."""
    from rob_box_harness.health import HealthAwareFallbackLLM, HealthCache

    server = _SilentServer()
    first = _HttpLLM(server.port)
    first.name = "deepseek"
    second = _LLM([LLMResponse(content="Жили-были…")])
    second.name = "minimax"
    cache = HealthCache()
    node = _hang_node(monkeypatch, HealthAwareFallbackLLM([first, second], cache=cache))
    try:
        node._on_utterance(_said("Робот, расскажи сказку", "u1"))
        assert server.got_request.wait(10)
        node._on_utterance(_said("Робот, стой", "u2"))
        node.last_turn.result(timeout=10)
        assert server.closed.wait(10) and first.cancelled.is_set()
        assert second.calls == 0 and not cache.is_unavailable("deepseek")
    finally:
        _drain(node)


def _music_msg(**dj):
    msg = MagicMock()
    msg.data = json.dumps({"state": "playing", "track_id": "t9", "dj": {"enabled": True, **dj}})
    return msg


def _voice_changed(requested, applied, provider):
    event = M.SpeechEvent()
    event.kind, event.voice_requested, event.voice_applied, event.provider = \
        M.SpeechEvent.KIND_VOICE_CHANGED, requested, applied, provider
    return event


def _hold(node, action, set_by="operator", ttl_s=0.0):
    return node._on_control(DialogControl.Request(action=action, set_by=set_by, ttl_s=ttl_s), types.SimpleNamespace())


def test_control_refuses_foreign_set_by_and_unknown_action(node):
    out = _hold(node, DialogControl.Request.ACTION_HOLD, set_by="user")
    assert out.ok is False and node._session.snapshot.top("OperatorHold") is None
    assert _hold(node, 99).ok is False


def test_operator_hold_drops_voice_and_does_not_collect_backlog(node):
    _hold(node, DialogControl.Request.ACTION_HOLD, set_by="tars", ttl_s=60.0)
    node._on_utterance(_utterance("Робот, как дела"))
    node._on_utterance(_utterance("ну вот покажи анимацию"))
    assert node.last_turn is None and node.test_llm.calls == 0
    assert list(node._backlog._entries) == []
    _hold(node, DialogControl.Request.ACTION_RESUME, set_by="tars")
    node._on_utterance(_utterance("Робот, как дела"))
    node.last_turn.result(timeout=10)
    assert node.test_llm.calls == 1


def test_dj_set_pushes_frames_and_idle_returns_voice_changed_during_set(node):
    node._on_speech_event(_voice_changed("Reliable_Man", "Reliable_Man", "minimax"))
    node._on_music_state(_music_msg(set_id="s1", persona="Дядя Стёпа"))
    assert [f.kind for f in node._session.snapshot.scenes] == ["DJ", "Persona"]
    node._on_speech_event(_voice_changed("alena", "alena", "yandex"))  # LLM сета сменила голос
    idle = MagicMock()
    idle.data = json.dumps({"state": "idle"})
    node._on_music_state(idle)
    assert node._session.snapshot.scenes == ()
    asked = [json.loads(m.data) for m in _published(node, "/voice/tts/voice_request")]
    assert [(a["provider"], a["voice"]) for a in asked] == [("minimax", "Reliable_Man")]
    assert asked[0]["request_id"]


def test_new_session_returns_default_voice_once(node):
    node._on_speech_event(_voice_changed("alena", "alena", "yandex"))
    _hold(node, DialogControl.Request.ACTION_RESET)
    asked = [json.loads(m.data) for m in _published(node, "/voice/tts/voice_request")]
    assert [(a["provider"], a["voice"]) for a in asked] == [tuple(dn.robot_default_voice())]
