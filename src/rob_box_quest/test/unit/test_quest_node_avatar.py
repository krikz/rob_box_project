"""Unit-тесты quest_node маршрута /avatar/tts/* (ADR-0055, issue #1993).

Покрывает commit C3 из impl-плана 0055:

* ``_on_avatar_tts_request_meta`` с ``sink="headset"`` → ``register_audio_session
  ("operator_tts", request_id, ws)`` + сохранение request_id/ws в node state.
* ``_on_avatar_tts_request_meta`` с ``sink != "headset"`` → игнор (это tts_node
  канал, не quest).
* ``_on_avatar_tts_audio`` после предрегистрации → ``ws_server.deliver_audio(
  stream="operator_tts", ..., ws=current)``.
* ``_on_avatar_tts_audio`` без предрегистрации (race) → DROP + warning.
* ``_pick_active_operator_ws`` — единственная активная сессия → её ws;
  ноль → None; >1 → последняя (самая свежая).

Запуск:
    PYTHONPATH=src/rob_box_quest:src/rob_box_core:src/rob_box_harness \\
        pytest src/rob_box_quest/test/unit/test_quest_node_avatar.py -v
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


def _make_msg(payload: dict):
    """MagicMock String-msg с JSON-encoded data."""
    m = MagicMock()
    m.data = json.dumps(payload, ensure_ascii=False)
    return m


def _make_audio_msg(pcm_bytes: bytes):
    """AudioData с PCM-данными в ``msg.data`` (bytes/list[int])."""
    m = MagicMock()
    m.data = list(pcm_bytes)  # AudioData.data = list[int]
    return m


def _make_host(*, active_sessions_count: int = 1, ws_for_session: dict | None = None):
    """Минимальный host с интерфейсом, который handler'ы требуют от QuestNode.

    ``ws_server._sessions`` имитируется MagicMock, у которого ``__iter__`` /
    ``.keys()`` отдают нужные ключи; ``get_active_sessions()`` возвращает
    переданное значение.
    """
    host = MagicMock()
    host.ws_server = MagicMock()
    # Имитация dict-семантики _sessions.
    sessions = ws_for_session or {"sess-1": "ws-object-1"}
    host.ws_server._sessions = sessions
    host.ws_server._ws_by_session = {k: MagicMock(name=f"ws:{k}") for k in sessions}
    host.ws_server.get_active_sessions = MagicMock(return_value=active_sessions_count)
    return host


# ── _pick_active_operator_ws ───────────────────────────────────────────


@_skip
class TestPickActiveOperatorWs(unittest.TestCase):
    def test_zero_active_returns_none(self):
        host = _make_host(active_sessions_count=0)
        result = QuestNode._pick_active_operator_ws(host)
        self.assertIsNone(result)

    def test_single_active_returns_first(self):
        host = _make_host(active_sessions_count=1)
        result = QuestNode._pick_active_operator_ws(host)
        # Единственная → берём первый ключ (insertion order).
        self.assertEqual(result, host.ws_server._ws_by_session["sess-1"])

    def test_multi_active_returns_last(self):
        # >1 активных — берём самую свежую (последнюю по insertion order).
        host = _make_host(
            active_sessions_count=2,
            ws_for_session={"sess-old": "ws-old", "sess-new": "ws-new"},
        )
        result = QuestNode._pick_active_operator_ws(host)
        self.assertEqual(result, host.ws_server._ws_by_session["sess-new"])


# ── _on_avatar_tts_request_meta ───────────────────────────────────────


@_skip
class TestOnAvatarTtsRequestMeta(unittest.TestCase):
    def test_sink_headset_registers_session_for_unique_ws(self):
        """sink=="headset" + 1 сессия → register_audio_session("operator_tts", r1, ws)."""
        host = _make_host(active_sessions_count=1)
        msg = _make_msg({
            "request_id": "r-1",
            "ssml": "<speak>готово</speak>",
            "sink": "headset",
        })
        QuestNode._on_avatar_tts_request_meta(host, msg)
        # register_audio_session был вызван с правильными args.
        host.ws_server.register_audio_session.assert_called_once()
        call_args = host.ws_server.register_audio_session.call_args
        self.assertEqual(call_args.args[0], "operator_tts")
        self.assertEqual(call_args.args[1], "r-1")
        # request_id и ws сохранены в node state.
        self.assertEqual(host._current_avatar_request_id, "r-1")
        self.assertIsNotNone(host._current_avatar_ws)

    def test_sink_not_headset_is_ignored(self):
        """sink != "headset" → игнор, register_audio_session НЕ вызван."""
        host = _make_host(active_sessions_count=1)
        msg = _make_msg({
            "request_id": "r-x",
            "ssml": "<speak>...</speak>",
            "sink": "speaker",
        })
        QuestNode._on_avatar_tts_request_meta(host, msg)
        host.ws_server.register_audio_session.assert_not_called()
        # request_id НЕ запомнен.
        self.assertIsNone(host._current_avatar_request_id)

    def test_no_active_session_drops(self):
        """Нет активных сессий → DROP + warning, без регистрации."""
        host = _make_host(active_sessions_count=0)
        msg = _make_msg({
            "request_id": "r-lonely",
            "ssml": "<speak>x</speak>",
            "sink": "headset",
        })
        QuestNode._on_avatar_tts_request_meta(host, msg)
        host.ws_server.register_audio_session.assert_not_called()
        self.assertIsNone(host._current_avatar_request_id)

    def test_bad_json_does_not_crash(self):
        """Битый JSON → warning + DROP."""
        host = _make_host(active_sessions_count=1)
        bad = MagicMock()
        bad.data = "not-json{"
        QuestNode._on_avatar_tts_request_meta(host, bad)
        host.ws_server.register_audio_session.assert_not_called()

    def test_empty_request_id_is_dropped(self):
        """request_id="" или отсутствует → DROP, без регистрации."""
        host = _make_host(active_sessions_count=1)
        msg = _make_msg({
            "request_id": "",
            "ssml": "<speak>x</speak>",
            "sink": "headset",
        })
        QuestNode._on_avatar_tts_request_meta(host, msg)
        host.ws_server.register_audio_session.assert_not_called()


# ── _on_avatar_tts_audio ──────────────────────────────────────────────


@_skip
class TestOnAvatarTtsAudio(unittest.TestCase):
    def test_routes_to_registered_ws(self):
        """После предрегистрации: deliver_audio(stream="operator_tts", ws=current, ...)."""
        host = _make_host(active_sessions_count=1)
        # Симулируем предрегистрацию через _on_avatar_tts_request_meta.
        QuestNode._on_avatar_tts_request_meta(
            host,
            _make_msg({
                "request_id": "r-route",
                "ssml": "<speak>x</speak>",
                "sink": "headset",
            }),
        )
        host.ws_server.deliver_audio.reset_mock()  # сбросим вызов register как side-effect
        # Теперь шлём AudioData.
        QuestNode._on_avatar_tts_audio(host, _make_audio_msg(b"\x01\x02\x03\x04"))
        host.ws_server.deliver_audio.assert_called_once()
        kwargs = host.ws_server.deliver_audio.call_args.kwargs
        self.assertEqual(kwargs["stream"], "operator_tts")
        self.assertEqual(kwargs["request_id"], "r-route")
        self.assertEqual(kwargs["audio_bytes"], b"\x01\x02\x03\x04")
        self.assertEqual(kwargs["audio_format"], "pcm_s16le")
        self.assertEqual(kwargs["content_type"], "audio/pcm")
        self.assertEqual(kwargs["seq"], 0)
        self.assertEqual(kwargs["total"], 0)
        # ws передан явно (ADR-0055 §quest_node).
        self.assertIs(kwargs["ws"], host._current_avatar_ws)

    def test_without_preregistration_drops(self):
        """Race: audio пришёл ДО side-channel → DROP + warning."""
        host = _make_host(active_sessions_count=1)
        # НЕ вызываем _on_avatar_tts_request_meta — request_id/ws == None.
        QuestNode._on_avatar_tts_audio(host, _make_audio_msg(b"\xaa\xbb"))
        host.ws_server.deliver_audio.assert_not_called()

    def test_deliver_audio_exception_does_not_crash_callback(self):
        """Исключение в deliver_audio логируется, не падает в ROS-callback."""
        host = _make_host(active_sessions_count=1)
        QuestNode._on_avatar_tts_request_meta(
            host,
            _make_msg({
                "request_id": "r-boom",
                "ssml": "<speak>x</speak>",
                "sink": "headset",
            }),
        )
        host.ws_server.deliver_audio.side_effect = RuntimeError("ws closed")
        # Не должно бросить исключение (callback от ROS).
        QuestNode._on_avatar_tts_audio(host, _make_audio_msg(b"\x00"))
        host.ws_server.deliver_audio.assert_called_once()


# ── guard: не задеваем /voice/tts/request (старый путь) ──────────────


@_skip
class TestNoRegressionToVoicePath(unittest.TestCase):
    def test_avatar_tts_meta_does_not_publish_to_voice_tts_request(self):
        """Avatar-канал НЕ идёт в /voice/tts/request (он остаётся для say)."""
        host = _make_host(active_sessions_count=1)
        # Имитируем: у ws_server НЕТ метода publish_to_voice_request — это
        # строго avatar-канал. Проверяем, что никаких лишних паблишей не было.
        QuestNode._on_avatar_tts_request_meta(
            host,
            _make_msg({
                "request_id": "r-iso",
                "ssml": "<speak>x</speak>",
                "sink": "headset",
            }),
        )
        # Никаких методов вне register_audio_session не звали.
        called_methods = [c[0] for c in host.ws_server.method_calls]
        self.assertTrue(
            all("publish" not in m.lower() and "say" not in m.lower() for m in called_methods),
            f"avatar-канал не должен трогать /voice/tts/*: {called_methods}",
        )


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
