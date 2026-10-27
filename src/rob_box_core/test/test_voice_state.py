"""Состояние голоса для читателей (шлем Quest, арбитр, кольцо) из контрактов диалога v2 (issue #3911).

Таблицы - одно место: ``rob_box_core.voice_state``. Здесь: переходы по ``TurnEvent``/``SpeechEvent``/``SessionState``,
адаптер v1 (``/voice/dialogue/state``), правило «наибольшая активность, тишина от любого источника», подписки ``watch``.
"""

from __future__ import annotations

import ast
import logging
import sys
import types
from pathlib import Path

import pytest

from rob_box_core import speech_client, voice_state as vs
from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
IDLE = vs.VoiceState("idle")


class _Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def turn(stage: str, status: str = "started", agent: str = "personality"):
    return M.TurnEvent(agent=agent, stage=stage, status=status)


def speech(kind: int, sid: str = "s1", agent: str = "personality"):
    return M.SpeechEvent(agent=agent, kind=kind, speech_id=sid)


def session(*scenes: str, agent: str = "personality"):
    return M.SessionState(agent=agent, scenes=[M.SceneFrame(kind=k) for k in scenes])


# ── v2: ход ────────────────────────────────────────────────────────────────────────────────────────────────


def test_turn_walks_listening_thinking_idle():
    tracker = vs.VoiceStateTracker()
    assert tracker.on_turn_event(turn("addressed")) == vs.VoiceState("listening")
    assert tracker.on_turn_event(turn("thinking")) == vs.VoiceState("thinking")
    assert tracker.on_turn_event(turn("executing")) is None  # «думает» не менялось
    assert tracker.on_turn_event(turn("done", "finished")) == IDLE


@pytest.mark.parametrize("stage", ["done", "failed", "timeout"])
def test_closing_stage_returns_idle_on_any_status_but_skipped(stage):
    tracker = vs.VoiceStateTracker()
    tracker.on_turn_event(turn("thinking"))
    assert tracker.on_turn_event(turn(stage, "skipped")) is None
    assert tracker.current.state == "thinking"
    assert tracker.on_turn_event(turn(stage, "cancelled")) == IDLE


def test_stage_finished_does_not_move_state_only_started_does():
    tracker = vs.VoiceStateTracker()
    tracker.on_turn_event(turn("thinking"))
    assert tracker.on_turn_event(turn("thinking", "finished")) is None
    assert tracker.current.state == "thinking"


def test_other_agent_and_unknown_stage_are_ignored():
    tracker = vs.VoiceStateTracker()
    assert tracker.on_turn_event(turn("thinking", agent="telegram")) is None
    assert tracker.on_turn_event(turn("warp_drive")) is None
    assert tracker.current == IDLE


# ── v2: речь ───────────────────────────────────────────────────────────────────────────────────────────────


def test_speech_started_until_finished_is_speaking_even_after_turn_done():
    tracker = vs.VoiceStateTracker()
    tracker.on_turn_event(turn("thinking"))
    tracker.on_turn_event(turn("done", "finished"))  # ход закрылся сразу после запроса речи
    assert tracker.current == IDLE
    assert tracker.on_speech_event(speech(2)) == vs.VoiceState("speaking")
    assert tracker.on_speech_event(speech(3)) is None  # progress
    assert tracker.on_speech_event(speech(4)) == IDLE


@pytest.mark.parametrize("end", [4, 5, 7])  # finished, cancelled, rejected
def test_every_speech_end_clears_speaking(end):
    tracker = vs.VoiceStateTracker()
    tracker.on_speech_event(speech(2))
    assert tracker.on_speech_event(speech(end)) == IDLE


def test_two_replies_keep_speaking_until_both_end():
    tracker = vs.VoiceStateTracker()
    tracker.on_speech_event(speech(2, "a"))
    tracker.on_speech_event(speech(2, "b"))
    assert tracker.on_speech_event(speech(4, "a")) is None
    assert tracker.on_speech_event(speech(4, "b")) == IDLE


def test_speech_of_other_agent_is_not_the_robot_voice():
    tracker = vs.VoiceStateTracker()
    assert tracker.on_speech_event(speech(2, agent="operator")) is None
    assert tracker.current == IDLE


def test_lost_speech_end_does_not_keep_speaking_forever():
    clock = _Clock()
    tracker = vs.VoiceStateTracker(clock=clock)
    tracker.on_speech_event(speech(2))
    clock.now = 10_000.0
    assert tracker.current == IDLE


# ── v2: тишина ─────────────────────────────────────────────────────────────────────────────────────────────


def test_silence_scene_makes_idle_silenced_and_its_removal_clears_it():
    tracker = vs.VoiceStateTracker()
    assert tracker.on_session(session("Persona", "Silence")) == vs.VoiceState("idle", "silenced")
    assert tracker.on_session(session("Persona", "Silence")) is None
    assert tracker.on_session(session("Persona")) == IDLE


def test_silence_wins_over_activity():
    tracker = vs.VoiceStateTracker()
    tracker.on_turn_event(turn("thinking"))
    assert tracker.on_session(session("Silence")) == vs.VoiceState("idle", "silenced")


def test_session_of_other_agent_is_not_our_silence():
    tracker = vs.VoiceStateTracker()
    assert tracker.on_session(session("Silence", agent="telegram")) is None
    assert tracker.current == IDLE


# ── v1: адаптер строки ────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("raw, expected", [
    ("IDLE", vs.VoiceState("idle")),
    ("LISTENING", vs.VoiceState("listening")),
    ("DIALOGUE", vs.VoiceState("speaking")),
    ("SILENCED", vs.VoiceState("idle", "silenced")),
    ("  IDLE\n", vs.VoiceState("idle")),
])
def test_legacy_string_maps_to_bridge(raw, expected):
    assert vs.legacy_state(raw) == expected
    assert vs.VoiceStateTracker().on_legacy(raw) == expected


def test_legacy_unknown_falls_back_with_warning_and_lowercase_is_unknown(caplog):
    with caplog.at_level(logging.WARNING):
        assert vs.legacy_state("BOGUS_STATE") == IDLE
        assert vs.legacy_state("idle") == IDLE
    assert any("BOGUS_STATE" in r.message and "fallback" in r.message for r in caplog.records)
    assert any("'idle'" in r.message for r in caplog.records)


@pytest.mark.parametrize("raw", ["", "   ", None])
def test_legacy_empty_is_idle_without_warning(raw, caplog):
    with caplog.at_level(logging.DEBUG):
        assert vs.legacy_state(raw) == IDLE
    assert [r for r in caplog.records if r.levelno >= logging.WARNING] == []


def test_legacy_returns_state_every_time_like_v1_did():
    tracker = vs.VoiceStateTracker()
    assert tracker.on_legacy("IDLE") == IDLE
    assert tracker.on_legacy("IDLE") == IDLE


def test_legacy_table_keys_are_uppercase_and_values_are_bridge_states():
    for raw, (state, _detail) in vs.LEGACY_STATE.items():
        assert raw == raw.upper()
        assert state in vs.BRIDGE_STATES


# ── сложение источников ────────────────────────────────────────────────────────────────────────────────────


def test_v1_and_v2_sources_take_the_most_active_and_never_fight():
    tracker = vs.VoiceStateTracker()
    tracker.on_legacy("DIALOGUE")  # v1: весь ответ «говорит»
    tracker.on_turn_event(turn("thinking"))
    assert tracker.current.state == "speaking"
    assert tracker.on_legacy("IDLE") == vs.VoiceState("thinking")  # v1 ушёл в покой, ход v2 ещё думает
    assert tracker.on_turn_event(turn("done", "finished")) == IDLE


def test_silenced_from_either_source_and_cleared_only_when_both_clear():
    tracker = vs.VoiceStateTracker()
    tracker.on_session(session("Silence"))
    assert tracker.on_legacy("IDLE") == vs.VoiceState("idle", "silenced")
    assert tracker.on_session(session()) == IDLE


# ── drift: литералы совпадают со знанием диалога ─────────────────────────────────────────────────────────


def test_literals_match_dialog_knowledge():
    knowledge = pytest.importorskip("rob_box_dialog.knowledge")
    assert vs.VOICE_AGENT == knowledge.AGENTS[0]
    assert vs.SILENCE_SCENE in knowledge.SCENE_KINDS
    assert set(vs.TURN_STAGE_STATE) == set(knowledge.TURN_STAGES)
    assert set(vs.TURN_STAGE_STATE.values()) <= set(vs.BRIDGE_STATES)


def _constant(package: str, relative: str, name: str):
    """Литерал модуля из исходника (без импорта: узлы ROS тянут rclpy)."""
    source = Path(__file__).resolve().parents[2] / package / relative
    if not source.is_file():
        pytest.skip(f"{source} не найден")
    for node in ast.parse(source.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} не найден в {source}")


def test_session_topic_is_the_one_dialog_node_publishes():
    assert vs.SESSION_TOPIC == _constant("rob_box_voice", "rob_box_voice/core/identity_question.py", "SESSION_TOPIC")


def test_perception_literals_match():
    """Образ perception (Main Pi) без rob_box_core держит свои литералы - сверяем здесь."""
    path = "rob_box_perception/core/robot_speech.py"
    assert _constant("rob_box_perception", path, "SPEECH_REQUEST_TOPIC") == speech_client.SPEECH_REQUEST_TOPIC
    assert _constant("rob_box_perception", path, "ROBOT_VOICE_AGENT") == vs.VOICE_AGENT


# ── watch: подписки на три контракта ─────────────────────────────────────────────────────────────────────


class _Node:
    def __init__(self) -> None:
        self.subs, self.log = [], []

    def get_logger(self):
        return types.SimpleNamespace(info=self.log.append)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs.append((msg_type, topic, callback, qos))


def _stub_rclpy_qos(monkeypatch):
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="transient_local")
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable")
    monkeypatch.setitem(sys.modules, "rclpy", types.ModuleType("rclpy"))
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)


def test_watch_subscribes_three_contracts_and_reports_changes(monkeypatch):
    _stub_rclpy_qos(monkeypatch)
    node, seen = _Node(), []
    tracker = vs.VoiceStateTracker()

    vs.watch(node, tracker, seen.append)

    by_topic = {topic: (msg_type, cb, q) for msg_type, topic, cb, q in node.subs}
    assert set(by_topic) == {"/dialog/turn_event", "/voice/speech/event", "/dialog/session"}
    assert by_topic["/dialog/turn_event"][0] is M.TurnEvent
    assert by_topic["/voice/speech/event"][0] is M.SpeechEvent
    assert by_topic["/dialog/session"][0] is M.SessionState
    assert by_topic["/dialog/session"][2].durability == "transient_local"  # latched: состояние после рестарта читателя

    by_topic["/dialog/turn_event"][1](turn("addressed"))
    by_topic["/dialog/turn_event"][1](turn("addressed"))  # повтор без перехода - тишина
    by_topic["/voice/speech/event"][1](speech(2))
    by_topic["/dialog/session"][1](session("Silence"))
    assert seen == [vs.VoiceState("listening"), vs.VoiceState("speaking"), vs.VoiceState("idle", "silenced")]
    assert node.log == ["voice_state -> listening", "voice_state -> speaking", "voice_state -> idle (silenced)"]


def test_bridge_catalog_names_the_topics_watch_subscribes(monkeypatch):
    """Стрим ``voice_state`` (0x1202) в каталоге моста называет те же топики, из которых ``watch`` собирает состояние
    шлема: ``stream_list`` не врёт, а старого ``/voice/dialogue/state`` там нет (#3711, 15-0a')."""
    from rob_box_core.bridge_protocol import get_stream

    _stub_rclpy_qos(monkeypatch)
    node = _Node()
    vs.watch(node, vs.VoiceStateTracker(), lambda state: None)

    source = get_stream("voice_state").source
    assert set(source.split(", ")) == {topic for _msg_type, topic, _cb, _qos in node.subs}
    assert "/voice/dialogue/state" not in source
