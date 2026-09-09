"""ADR-0077 / issue #2138.A.3 — тесты supervisor'а на preview-канал.

Покрывают:
1. ``_on_preview_voice`` валидный запрос → публикация в ``/avatar/tts/request``
   с ``sink="preview"`` И ``request_id`` от picker'а (НЕ генерируется новый).
2. Невалидный voice_id (отсутствует) → ``preview_voice_error`` с reason
   ``voice_id_required``.
3. Невалидный text (отсутствует) → ``preview_voice_error`` с reason
   ``text_required``.
4. Voice_id неизвестен реестру (без provider) → ``preview_voice_error`` с
   reason ``voice_unknown``.
5. Voice_id неизвестен для указанного provider → ``preview_voice_error`` с
   reason ``voice_unavailable:<provider>:<voice_id>``.
6. Поле ``provider`` пустое, но голос известен → публикация в
   ``/avatar/tts/request`` с sink="preview".
"""

from __future__ import annotations

import json
import unittest

from rob_box_supervisor.supervisor_node import AvatarSupervisor


def _published_avatar_tts(node: AvatarSupervisor) -> list[dict]:
    """Опубликованные в /avatar/tts/request payloads (parsed JSON)."""
    pub = node._avatar_tts_request_pub
    return [json.loads(m.data) for m in pub.published]


def _published_preview_error(node: AvatarSupervisor) -> list[dict]:
    """Опубликованные в /avatar/preview_voice/error payloads (parsed JSON)."""
    pub = node._preview_error_pub
    return [json.loads(m.data) for m in pub.published]


def _make_msg(data: dict):
    """Простой mock для RosString."""
    msg = type("M", (), {})()
    msg.data = json.dumps(data, ensure_ascii=False)
    return msg


class TestOnPreviewVoiceDelegatesToAvatarTts(unittest.TestCase):
    """Валидный preview → /avatar/tts/request с sink='preview'."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_valid_preview_publishes_avatar_tts_with_sink_preview(self) -> None:
        msg = _make_msg(
            {
                "request_id": "picker-uuid-1234",
                "voice_id": "male-qn-qingse",  # yandex + minimax регистры
                "text": "привет",
            }
        )
        self.node._on_preview_voice(msg)
        pub = _published_avatar_tts(self.node)
        self.assertEqual(len(pub), 1, f"expected 1 avatar_tts publish, got {pub}")
        payload = pub[0]
        self.assertEqual(payload["sink"], "preview")
        self.assertEqual(payload["request_id"], "picker-uuid-1234")
        self.assertIn("привет", payload["ssml"])
        # Не должно быть ошибок.
        self.assertEqual(_published_preview_error(self.node), [])

    def test_provider_hint_preserved_in_publish(self) -> None:
        msg = _make_msg(
            {
                "request_id": "test-rid",
                "voice_id": "alena",  # в yandex-реестре
                "text": "test",
                "provider": "yandex",
            }
        )
        self.node._on_preview_voice(msg)
        pub = _published_avatar_tts(self.node)
        self.assertEqual(len(pub), 1)
        self.assertEqual(pub[0]["sink"], "preview")
        self.assertEqual(pub[0]["voice"], "alena")


class TestOnPreviewVoiceValidation(unittest.TestCase):
    """Валидационные ошибки → preview_voice_error (НЕ avatar_tts publish)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_missing_voice_id_emits_voice_id_required(self) -> None:
        msg = _make_msg({"request_id": "r1", "text": "x"})
        self.node._on_preview_voice(msg)
        err = _published_preview_error(self.node)
        self.assertEqual(len(err), 1)
        self.assertEqual(err[0]["reason"], "voice_id_required")
        self.assertEqual(err[0]["request_id"], "r1")
        self.assertEqual(_published_avatar_tts(self.node), [])

    def test_missing_text_emits_text_required(self) -> None:
        msg = _make_msg({"request_id": "r2", "voice_id": "alena"})
        self.node._on_preview_voice(msg)
        err = _published_preview_error(self.node)
        self.assertEqual(len(err), 1)
        self.assertEqual(err[0]["reason"], "text_required")
        self.assertEqual(_published_avatar_tts(self.node), [])

    def test_unknown_voice_emits_voice_unknown(self) -> None:
        msg = _make_msg(
            {
                "request_id": "r3",
                "voice_id": "totally-bogus-voice-id-xyz",
                "text": "test",
            }
        )
        self.node._on_preview_voice(msg)
        err = _published_preview_error(self.node)
        self.assertEqual(len(err), 1)
        self.assertEqual(err[0]["reason"], "voice_unknown")
        self.assertEqual(_published_avatar_tts(self.node), [])

    def test_voice_unknown_for_provider_emits_voice_unavailable(self) -> None:
        msg = _make_msg(
            {
                "request_id": "r4",
                "voice_id": "alena",  # есть в yandex, нет в minimax
                "text": "test",
                "provider": "minimax",
            }
        )
        self.node._on_preview_voice(msg)
        err = _published_preview_error(self.node)
        self.assertEqual(len(err), 1)
        # Конкретный формат voice_unavailable:<provider>:<voice_id>.
        self.assertTrue(
            err[0]["reason"].startswith("voice_unavailable:minimax:"),
            f"got {err[0]['reason']!r}",
        )
        self.assertEqual(_published_avatar_tts(self.node), [])

    def test_bad_json_does_not_publish_anything(self) -> None:
        msg = type("M", (), {})()
        msg.data = "{not valid json"
        self.node._on_preview_voice(msg)
        self.assertEqual(_published_avatar_tts(self.node), [])
        self.assertEqual(_published_preview_error(self.node), [])

    def test_empty_payload_does_not_publish_anything(self) -> None:
        msg = type("M", (), {})()
        msg.data = ""
        self.node._on_preview_voice(msg)
        self.assertEqual(_published_avatar_tts(self.node), [])
        self.assertEqual(_published_preview_error(self.node), [])


class TestPublishAvatarTtsPreviewSink(unittest.TestCase):
    """``_publish_avatar_tts(sink='preview', request_id=...)`` — прямой
    контракт для других caller'ов (на случай если они хотят опубликовать
    preview-запрос напрямую, без ``_on_preview_voice`` валидации).
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_sink_preview_with_caller_request_id_preserved(self) -> None:
        rid = self.node._publish_avatar_tts(
            text="привет",
            sink="preview",
            request_id="caller-provided-rid",
        )
        self.assertEqual(rid, "caller-provided-rid")
        pub = _published_avatar_tts(self.node)
        self.assertEqual(len(pub), 1)
        self.assertEqual(pub[0]["sink"], "preview")
        self.assertEqual(pub[0]["request_id"], "caller-provided-rid")

    def test_default_sink_remains_headset(self) -> None:
        """Backward-compat: sink='headset' по умолчанию, request_id
        генерируется (не caller-provided)."""
        rid = self.node._publish_avatar_tts(text="привет")
        self.assertEqual(len(rid), 8)  # uuid4().hex[:8]
        pub = _published_avatar_tts(self.node)
        self.assertEqual(pub[0]["sink"], "headset")