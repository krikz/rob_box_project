"""ADR-0150 PR-3 (#3688): единственный клиент речи ``speech_client``."""

from __future__ import annotations

import types

import pytest

from rob_box_core import speech_client
from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_core.utterance import PRIORITY_OPERATOR, Utterance

M = install_dialog_msgs_stub()
#: ``builtin_interfaces/Time`` той сборки сообщений, что под рукой (rosidl в CI, заглушка без ROS).
TIME = type(M.SpeechRequest().stamp)


class _Pub:
    def __init__(self, msg_type, topic):
        self.msg_type, self.topic, self.sent = msg_type, topic, []

    def publish(self, msg):
        self.sent.append(msg)


class _Node:
    def __init__(self):
        self.pubs = []

    def create_publisher(self, msg_type, topic, depth):
        pub = _Pub(msg_type, topic)
        self.pubs.append(pub)
        return pub

    def get_clock(self):
        stamp = TIME(sec=7, nanosec=5)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))


def test_say_builds_request_from_utterance_and_returns_speech_id():
    node = _Node()
    sid = speech_client.say(
        node,
        Utterance(text="x<y & z", voice="ermil", language="ru-RU", priority=PRIORITY_OPERATOR),
        agent="operator",
        epoch=3,
        turn_id="t1",
        at=12.5,
        boundary="sentence",
        commit=False,
    )
    (pub,) = node.pubs
    (req,) = pub.sent
    assert pub.topic == "/voice/speech/request" and pub.msg_type is M.SpeechRequest
    assert req.speech_id == sid and len(sid) == 36
    assert req.text == "<speak>x&lt;y &amp; z</speak>"  # SSML собирает Utterance
    assert (req.agent, req.epoch, req.turn_id) == ("operator", 3, "t1")
    assert (req.voice, req.language, req.sink) == ("ermil", "ru-RU", "speakers")
    assert req.priority == speech_client.PRIORITY_OPERATOR_U8
    assert (req.at, req.boundary, req.commit) == (12.5, M.SpeechRequest.BOUNDARY_SENTENCE, False)
    assert (req.stamp.sec, req.stamp.nanosec) == (7, 5)


def test_one_publisher_per_node_and_kind():
    node = _Node()
    speech_client.say(node, Utterance(text="а", speech_id="s1"), agent="personality")
    speech_client.say(node, Utterance(text="б"), agent="personality")
    speech_client.cancel(node, agent="personality")
    speech_client.cancel(node, agent="personality", epoch=2)
    assert [p.topic for p in node.pubs] == ["/voice/speech/request", "/voice/speech/cancel"]
    assert node.pubs[0].sent[0].speech_id == "s1"


def test_prepare_creates_publishers_up_front_and_reuses_them():
    node = _Node()
    speech_client.prepare(node, "cancel")
    assert [p.topic for p in node.pubs] == ["/voice/speech/cancel"]
    speech_client.cancel(node, agent="personality")
    assert len(node.pubs) == 1 and len(node.pubs[0].sent) == 1


def test_cancel_by_epoch_and_all_epochs():
    node = _Node()
    speech_client.cancel(node, agent="personality", epoch=4, reason="barge_in")
    speech_client.cancel(node, agent=speech_client.ANY_AGENT, reason="ptt")
    speech_client.cancel(node, agent="telegram", speech_id="sid-9")
    by_epoch, any_agent, by_id = node.pubs[0].sent
    assert (by_epoch.agent, by_epoch.epoch, by_epoch.speech_id, by_epoch.reason) == (
        "personality", 4, "", "barge_in")
    assert (any_agent.agent, any_agent.epoch) == ("*", speech_client.EPOCH_ALL)
    assert (by_id.speech_id, by_id.epoch) == ("sid-9", speech_client.EPOCH_ALL)


@pytest.mark.parametrize("agent", ["robot", "tars", "", "*"])
def test_say_rejects_unknown_agent(agent):
    with pytest.raises(ValueError):
        speech_client.say(_Node(), Utterance(text="а"), agent=agent)


def test_cancel_rejects_unknown_agent_and_say_rejects_bad_boundary():
    with pytest.raises(ValueError):
        speech_client.cancel(_Node(), agent="robot")
    with pytest.raises(ValueError):
        speech_client.say(_Node(), Utterance(text="а"), agent="system", boundary="bar")


def test_epoch_watch_tracks_max_epoch_of_its_agent_only():
    watch = speech_client.EpochWatch("personality")
    for agent, epoch in [("personality", 2), ("operator", 9), ("personality", 1), ("personality", 5)]:
        watch.on_event(types.SimpleNamespace(agent=agent, epoch=epoch))
    assert watch.epoch == 5


def test_speaking_watch_follows_started_and_end_events():
    """ADR-0150 PR-6: «громче» во время речи — канал голоса; звучит = started без finished/cancelled/rejected."""
    now = [0.0]
    watch = speech_client.SpeakingWatch("personality", clock=lambda: now[0])
    ev = lambda kind, sid, agent="personality": types.SimpleNamespace(kind=kind, speech_id=sid, agent=agent)  # noqa: E731
    assert not watch.speaking
    watch.on_event(ev(2, "a"))
    watch.on_event(ev(2, "x", agent="operator"))
    assert watch.speaking
    watch.on_event(ev(4, "a"))
    assert not watch.speaking
    for end in (5, 7):
        watch.on_event(ev(2, "b"))
        watch.on_event(ev(end, "b"))
        assert not watch.speaking
    watch.on_event(ev(2, "lost"))
    now[0] = speech_client.SPEAKING_STALE_S + 1.0
    assert not watch.speaking  # событие конца потерялось — не звучит вечно


def test_speaking_watch_of_any_agent_reports_edges_once_for_overlapping_speech():
    """PR-15 15-0b: читатели ``/voice/tts/state`` (лицо, эхо-гейты) слушают речь любого агента и реагируют на фронт:
    начало первой реплики и конец последней; реплика внахлёст и конец незвучавшей фронта не дают."""
    watch = speech_client.SpeakingWatch(speech_client.ANY_AGENT)
    ev = lambda kind, sid, agent="personality": types.SimpleNamespace(kind=kind, speech_id=sid, agent=agent)  # noqa: E731
    assert watch.on_event(ev(0, "a")) is None                       # queued: ещё не звучит
    assert watch.on_event(ev(2, "a")) is True                       # started: зазвучало
    assert watch.on_event(ev(2, "a")) is None                       # started того же куска ещё раз
    assert watch.on_event(ev(2, "op", agent="operator")) is None    # оператор внахлёст: всё ещё звучит
    assert watch.on_event(ev(4, "a")) is None                       # личность закончила, оператор звучит
    assert watch.on_event(ev(5, "never-played")) is None            # отмена незвучавшей реплики
    assert watch.on_event(ev(5, "op", agent="operator")) is False   # последняя замолчала
    assert watch.on_event(ev(2, "s", agent="system")) is True
    assert watch.on_event(ev(7, "s", agent="system")) is False      # rejected тоже конец


def test_watch_speaking_subscribes_to_speech_event_and_reports_only_edges():
    """Эхо-гейты микрофона (audio_node, stt_node): одна подписка на SpeechEvent, реакция — только на фронт."""
    subs, edges = {}, []
    node = types.SimpleNamespace(create_subscription=lambda t, topic, cb, depth: subs.setdefault(topic, (t, cb)))
    watch = speech_client.watch_speaking(node, edges.append)
    msg_type, callback = subs[speech_client.SPEECH_EVENT_TOPIC]
    assert msg_type is M.SpeechEvent
    for kind, sid, agent in [(0, "a", "personality"), (2, "a", "personality"), (2, "b", "operator"),
                             (4, "a", "personality"), (5, "b", "operator")]:
        callback(M.SpeechEvent(kind=kind, speech_id=sid, agent=agent))
    assert edges == [True, False]
    assert not watch.speaking


def test_speaking_watch_of_one_agent_ignores_others_and_unknown_agent_is_refused():
    watch = speech_client.SpeakingWatch("personality")
    assert watch.on_event(types.SimpleNamespace(kind=2, speech_id="x", agent="operator")) is None
    assert not watch.speaking
    with pytest.raises(ValueError):
        speech_client.SpeakingWatch("robot")


def test_speech_agents_cover_dialog_engine_agents():
    """Агенты движка (knowledge.AGENTS) - подмножество авторов речи."""
    knowledge = pytest.importorskip("rob_box_dialog.knowledge")
    assert set(knowledge.AGENTS) <= set(speech_client.SPEECH_AGENTS)


@pytest.mark.parametrize("kind, reason, seconds, expected", [
    (2, "", 0.0, ("pending", None, 0)),
    (4, "", 11.6, ("done", None, 12)),
    (5, "", 3.0, ("cancelled", None, 3)),
    (7, "unsupported_script", 0.0, ("rejected", "unsupported_script", 0)),
    (7, "", 0.0, ("rejected", "error", 0)),
])
def test_event_outcome_maps_speech_event_to_action_status(kind, reason, seconds, expected):
    event = types.SimpleNamespace(kind=kind, reason=reason, seconds=seconds)
    assert speech_client.event_outcome(event) == expected
