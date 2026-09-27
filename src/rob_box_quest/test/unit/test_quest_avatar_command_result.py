"""Unit-тесты relay /avatar/command_result → WS JSON_EVENT (issue #1988).

QuestNode подписывается на /avatar/command_result и транслирует ответ ТАРС
всем WS-сессиям (type="avatar_command_result"). Handler ``_on_avatar_command_result``
использует только ``self.ws_server`` / ``self.get_logger`` — тестируем
unbound-методом на лёгком стабе, без полного QuestNode.

QuestNode импортирует ROS-msg-пакеты (audio_common_msgs и др.), которых нет
на dev-env — паттерн как в test_quest_bridge.py: в dev тесты skip, в Docker
CI выполняются.
"""

from __future__ import annotations

import json
import unittest
from unittest.mock import MagicMock

from rob_box_core.avatar_command import AVATAR_COMMAND_RESULT_TOPIC

try:  # Docker-only: quest_node тянет rclpy/audio_common_msgs.
    from rob_box_quest.quest_node import QuestNode  # noqa: E402

    _HAVE_QUEST = True
except Exception:  # pragma: no cover — dev-env без ROS-msg
    QuestNode = None  # type: ignore[assignment,misc]
    _HAVE_QUEST = False

_skip = unittest.skipUnless(
    _HAVE_QUEST, "quest_node требует ROS msg-пакеты (только в Docker image)"
)


def _stub_host() -> object:
    """Объект с интерфейсом, который handler требует от QuestNode."""
    host = MagicMock()
    host.ws_server = MagicMock()
    host.get_logger = MagicMock(return_value=MagicMock())
    return host


def _result_msg(payload: dict) -> MagicMock:
    m = MagicMock()
    m.data = json.dumps(payload, ensure_ascii=False)
    return m


@_skip
class TestOnAvatarCommandResult(unittest.TestCase):
    def test_topic_constant(self) -> None:
        self.assertEqual(AVATAR_COMMAND_RESULT_TOPIC, "/avatar/command_result")

    def test_broadcasts_avatar_command_result_event(self) -> None:
        host = _stub_host()
        msg = _result_msg(
            {
                "request_id": "quest:sid1:100",
                "ok": True,
                "summary": "Выполнено",
                "tool_calls": [{"name": "say"}],
            }
        )
        QuestNode._on_avatar_command_result(host, msg)

        host.ws_server.broadcast_json_event.assert_called_once()
        event = host.ws_server.broadcast_json_event.call_args.args[0]
        self.assertEqual(event["type"], "avatar_command_result")
        self.assertEqual(event["request_id"], "quest:sid1:100")
        self.assertTrue(event["ok"])
        self.assertEqual(event["summary"], "Выполнено")
        self.assertEqual(event["tool_calls"], [{"name": "say"}])
        self.assertIn("ts_ms", event)

    def test_bad_json_is_ignored(self) -> None:
        host = _stub_host()
        QuestNode._on_avatar_command_result(host, _result_msg("not-json"))
        host.ws_server.broadcast_json_event.assert_not_called()

    def test_non_dict_payload_is_ignored(self) -> None:
        host = _stub_host()
        QuestNode._on_avatar_command_result(host, _result_msg([1, 2, 3]))
        host.ws_server.broadcast_json_event.assert_not_called()

    def test_broadcast_failure_does_not_raise(self) -> None:
        host = _stub_host()
        host.ws_server.broadcast_json_event.side_effect = RuntimeError("ws closed")
        QuestNode._on_avatar_command_result(
            host, _result_msg({"request_id": "r", "ok": False, "summary": "x"})
        )
        # Не упало — relay best-effort.


if __name__ == "__main__":
    unittest.main()
