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


def test_speech_agents_cover_dialog_engine_agents():
    """Агенты движка (knowledge.AGENTS) - подмножество авторов речи."""
    knowledge = pytest.importorskip("rob_box_dialog.knowledge")
    assert set(knowledge.AGENTS) <= set(speech_client.SPEECH_AGENTS)
