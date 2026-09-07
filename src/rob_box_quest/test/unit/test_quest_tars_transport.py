"""Unit-тесты relay-шва TARS1/TARS2 (issue #2113, quest #2112).

PR #2114 построил обе половины фичи изолированно:

* backend публикует `/tars1/text` (tts_node) и `/avatar/tars/panel_url`
  (avatar_supervisor/tars_panel.py) — покрыто своими unit-тестами;
* клиент умеет `tars1Panel.append()` / `tars2Panel.setPanelUrl()` —
  тоже покрыто своими unit-тестами (vitest).

Транспорта между ними не было: `git grep 'tars1/text\\|tars/panel_url'
-- src/` не находил ни одного подписчика в `quest_node.py`. Этот файл
тестирует ровно шов — ``QuestNode._on_tars1_text`` /
``QuestNode._on_tars_panel_url``, добавленные в рамках доделки #2114:
ROS String JSON → ``ws_server.broadcast_json_event`` (тот же
relay-паттерн, что у ``_on_avatar_command_result``, см.
``test_quest_avatar_command_result.py`` — этот файл написан по его
образцу «unbound-метод на лёгком стабе»).

QuestNode импортирует ROS-msg-пакеты (audio_common_msgs и др.), которых
нет на dev-env — паттерн как в test_quest_avatar_command_result.py: в
dev тесты skip, в Docker CI (или colcon-окружении с
ros-humble-audio-common-msgs + msgpack/aiohttp) выполняются реально.
"""

from __future__ import annotations

import json
import unittest
from unittest.mock import MagicMock

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


def _msg(payload) -> MagicMock:
    m = MagicMock()
    if isinstance(payload, str):
        m.data = payload
    else:
        m.data = json.dumps(payload, ensure_ascii=False)
    return m


@_skip
class TestOnTars1Text(unittest.TestCase):
    """Шов: ROS /tars1/text → JSON_EVENT(type=tars1_text) всем WS-сессиям.

    Контракт входа зафиксирован в tts_node._publish_tars1_text (PR #2114,
    src/rob_box_voice/rob_box_voice/tts_node.py):
    {request_id, text, streaming, done}.
    """

    def test_broadcasts_tars1_text_event(self) -> None:
        host = _stub_host()
        msg = _msg(
            {
                "request_id": "abc123",
                "text": "Привет, оператор",
                "streaming": True,
                "done": False,
            }
        )
        QuestNode._on_tars1_text(host, msg)

        host.ws_server.broadcast_json_event.assert_called_once()
        event = host.ws_server.broadcast_json_event.call_args.args[0]
        self.assertEqual(event["type"], "tars1_text")
        self.assertEqual(event["request_id"], "abc123")
        self.assertEqual(event["text"], "Привет, оператор")
        self.assertTrue(event["streaming"])
        self.assertFalse(event["done"])
        self.assertIn("ts_ms", event)

    def test_done_chunk_carries_done_true(self) -> None:
        host = _stub_host()
        QuestNode._on_tars1_text(
            host,
            _msg({"request_id": "r2", "text": "Готово", "streaming": True, "done": True}),
        )
        event = host.ws_server.broadcast_json_event.call_args.args[0]
        self.assertTrue(event["done"])

    def test_bad_json_is_ignored(self) -> None:
        host = _stub_host()
        QuestNode._on_tars1_text(host, _msg("not-json"))
        host.ws_server.broadcast_json_event.assert_not_called()

    def test_non_dict_payload_is_ignored(self) -> None:
        host = _stub_host()
        QuestNode._on_tars1_text(host, _msg([1, 2, 3]))
        host.ws_server.broadcast_json_event.assert_not_called()

    def test_broadcast_failure_does_not_raise(self) -> None:
        host = _stub_host()
        host.ws_server.broadcast_json_event.side_effect = RuntimeError("ws closed")
        QuestNode._on_tars1_text(
            host, _msg({"request_id": "r", "text": "x", "streaming": False, "done": True})
        )
        # Не упало — relay best-effort (зеркало avatar_command_result).


@_skip
class TestOnTarsPanelUrl(unittest.TestCase):
    """Шов: ROS /avatar/tars/panel_url → JSON_EVENT(type=tars_panel_url).

    Контракт входа зафиксирован в tars_panel.py._publish_url (PR #2114,
    src/rob_box_supervisor/rob_box_supervisor/tars_panel.py):
    {request_id, url, status, error}.
    """

    def test_broadcasts_ok_status_with_url(self) -> None:
        host = _stub_host()
        msg = _msg(
            {
                "request_id": "req1",
                "url": "http://prometheus.lan/grafana/d/prometheus-overview?query=cpu",
                "status": "ok",
                "error": "",
            }
        )
        QuestNode._on_tars_panel_url(host, msg)

        host.ws_server.broadcast_json_event.assert_called_once()
        event = host.ws_server.broadcast_json_event.call_args.args[0]
        self.assertEqual(event["type"], "tars_panel_url")
        self.assertEqual(event["request_id"], "req1")
        self.assertEqual(
            event["url"], "http://prometheus.lan/grafana/d/prometheus-overview?query=cpu"
        )
        self.assertEqual(event["status"], "ok")
        self.assertEqual(event["error"], "")
        self.assertIn("ts_ms", event)

    def test_broadcasts_error_status_with_empty_url(self) -> None:
        """dispatcher's error path (empty query / unknown datasource):
        url пуст — relay всё равно шлёт событие, клиент решает honest-state
        сам (см. main.ts: status!=="ok" → tars2Panel.setState("error"))."""
        host = _stub_host()
        msg = _msg(
            {
                "request_id": "req2",
                "url": "",
                "status": "error",
                "error": "empty query",
            }
        )
        QuestNode._on_tars_panel_url(host, msg)

        event = host.ws_server.broadcast_json_event.call_args.args[0]
        self.assertEqual(event["status"], "error")
        self.assertEqual(event["url"], "")
        self.assertEqual(event["error"], "empty query")

    def test_bad_json_is_ignored(self) -> None:
        host = _stub_host()
        QuestNode._on_tars_panel_url(host, _msg("not-json"))
        host.ws_server.broadcast_json_event.assert_not_called()

    def test_non_dict_payload_is_ignored(self) -> None:
        host = _stub_host()
        QuestNode._on_tars_panel_url(host, _msg([1, 2, 3]))
        host.ws_server.broadcast_json_event.assert_not_called()

    def test_broadcast_failure_does_not_raise(self) -> None:
        host = _stub_host()
        host.ws_server.broadcast_json_event.side_effect = RuntimeError("ws closed")
        QuestNode._on_tars_panel_url(
            host, _msg({"request_id": "r", "url": "http://x", "status": "ok", "error": ""})
        )
        # Не упало — relay best-effort.


if __name__ == "__main__":
    unittest.main()
