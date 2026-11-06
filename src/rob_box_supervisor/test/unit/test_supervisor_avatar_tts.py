"""ADR-0055 / issue #1993, #4252 — реплики ТАРС в шлем: ``SpeechRequest`` владельцу речи с ``sink=headset``.

До #4252 собственные реплики супервизора шли String JSON в ``/avatar/tts/request`` мимо владельца речи
(``speech_owner`` отвечал ``sink_unsupported``), и конец реплики знал только старый ``/voice/tts/finished``.
Теперь ``headset_speech.say_in_headset`` публикует ``SpeechRequest(agent=operator, sink=headset)``; итог — ``SpeechEvent``
с тем же ``speech_id``. ``/avatar/tts/request`` остался только для образца голоса (``sink=preview``).

Используем mock-rclpy из conftest.py.
"""
from __future__ import annotations

import json
import unittest

from rob_box_core.speech_client import SPEECH_REQUEST_TOPIC
from rob_box_supervisor import headset_speech
from rob_box_supervisor.supervisor_node import (
    AVATAR_TTS_REQUEST_TOPIC,
    AvatarSupervisor,
)


def _say(node: AvatarSupervisor, text: str) -> str:
    return headset_speech.say_in_headset(node, text, node._log)


def _speech_requests(node: AvatarSupervisor) -> list:
    """Все ``SpeechRequest``, опубликованные клиентом речи."""
    return list(node._speech_client_pubs["request"].published)


def _headset(node: AvatarSupervisor) -> list:
    return [r for r in _speech_requests(node) if r.sink == "headset"]


def _speakers(node: AvatarSupervisor) -> list:
    return [r for r in _speech_requests(node) if r.sink != "headset"]


def _avatar_channel(node: AvatarSupervisor) -> list[dict]:
    return [json.loads(m.data) for m in node._avatar_tts_request_pub.published]


class TestPublishersDeclared(unittest.TestCase):
    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_speech_request_publisher_declared(self) -> None:
        self.assertIn(SPEECH_REQUEST_TOPIC, self.node._publishers)

    def test_preview_channel_still_declared(self) -> None:
        self.assertIn(AVATAR_TTS_REQUEST_TOPIC, self.node._publishers)


class TestSayInHeadset(unittest.TestCase):
    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_reply_goes_to_speech_owner_with_headset_sink(self) -> None:
        sid = _say(self.node, "готово")
        (req,) = _headset(self.node)
        self.assertEqual((req.text, req.agent, req.speech_id), ("готово", "operator", sid))
        self.assertTrue(req.commit)
        self.assertEqual(req.at, 0.0)

    def test_speech_ids_are_unique(self) -> None:
        self.assertNotEqual(_say(self.node, "раз"), _say(self.node, "два"))

    def test_headset_reply_is_not_on_the_old_string_channel_nor_speakers(self) -> None:
        _say(self.node, "принял")
        self.assertEqual(_avatar_channel(self.node), [])
        self.assertEqual(_speakers(self.node), [])

    def test_empty_text_is_dropped(self) -> None:
        for text in ("", "   \n\t  "):
            self.assertEqual(_say(self.node, text), "")
        self.assertEqual(_speech_requests(self.node), [])


class TestPublishGripTtsEmptyTextDrop(unittest.TestCase):
    """Issue #2096 + #2137 — грип говорит динамиками робота, не в шлем (инвариант 6b §7.4)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_empty_string_drops_request(self) -> None:
        self.node._publish_grip_tts("")
        self.node._publish_grip_tts("   \n\t  ")
        self.assertEqual(_speech_requests(self.node), [])

    def test_non_empty_text_goes_to_speakers_with_operator_priority(self) -> None:
        self.node._publish_grip_tts("мы начинаем")
        (req,) = _speakers(self.node)
        self.assertEqual((req.text, req.agent), ("мы начинаем", "operator"))
        self.assertGreaterEqual(req.priority, 10)
        self.assertEqual(_headset(self.node), [])
        self.assertEqual(_avatar_channel(self.node), [])


if __name__ == "__main__":
    unittest.main()
