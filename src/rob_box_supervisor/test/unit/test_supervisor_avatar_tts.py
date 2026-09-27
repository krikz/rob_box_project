"""ADR-0055 / issue #1993 — тесты supervisor'а на публикацию в /avatar/tts/request.

Шаг 5б: avatar_supervisor должен публиковать собственные реплики ТАРС
(«принял», «не умею», «камера повёрнута», «не понял») в новый обратный
канал шлема, а не в /voice/tts/request (там остался только say).

Используем mock-rclpy из conftest.py. Тесты:

1. ``test_publish_avatar_tts_writes_headset_request`` — прямой вызов
   ``_publish_avatar_tts`` шлёт JSON ``{request_id, ssml, sink:\"headset\"}``
   в /avatar/tts/request.
2. ``test_publish_avatar_tts_returns_request_id`` — возвращается uuid hex8
   (>= 8 chars), caller может его логировать.
3. ``test_publish_avatar_tts_optional_voice_language_forwarded`` —
   язык/голос пробрасываются в payload только когда заданы.
4. ``test_avatar_publisher_declared_on_init`` — паблишер заведён в __init__.
5. ``test_supervisor_does_not_use_voice_tts_request_for_own_lines`` —
   ADR-0055 §C4: для собственных реплик supervisor НЕ идёт в
   /voice/tts/request (это канал для инструмента say).
"""
from __future__ import annotations

import json
import unittest

from rob_box_supervisor.supervisor_node import (
    AVATAR_TTS_REQUEST_TOPIC,
    GRIP_TTS_REQUEST_TOPIC,
    AvatarSupervisor,
)


def _published_avatar_tts(node: AvatarSupervisor) -> list[dict]:
    """Опубликованные в /avatar/tts/request payloads (parsed JSON)."""
    pub = node._avatar_tts_request_pub
    return [json.loads(m.data) for m in pub.published]


def _published_voice_tts(node: AvatarSupervisor) -> list[dict]:
    """Опубликованные в /voice/tts/request payloads (parsed JSON)."""
    pub = node._tts_request_pub
    return [json.loads(m.data) for m in pub.published]


class TestAvatarTtsPublisherDeclared(unittest.TestCase):
    """ADR-0055 #1993: паблишер /avatar/tts/request обязан быть заведён в __init__."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_avatar_tts_request_publisher_declared(self) -> None:
        self.assertIn(AVATAR_TTS_REQUEST_TOPIC, self.node._publishers)
        self.assertEqual(AVATAR_TTS_REQUEST_TOPIC, "/avatar/tts/request")

    def test_old_voice_tts_request_publisher_still_declared_for_say(self) -> None:
        # /voice/tts/request оставлен ТОЛЬКО для инструмента say.
        # Грип больше туда не ходит; supervisor для собственных реплик — тоже.
        self.assertIn(GRIP_TTS_REQUEST_TOPIC, self.node._publishers)
        self.assertEqual(GRIP_TTS_REQUEST_TOPIC, "/voice/tts/request")


class TestPublishAvatarTtsWithDefaults(unittest.TestCase):
    """``_publish_avatar_tts(text)`` — минимальный контракт."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_publish_avatar_tts_writes_headset_request(self) -> None:
        rid = self.node._publish_avatar_tts("готово")
        pub = _published_avatar_tts(self.node)
        self.assertEqual(len(pub), 1)
        self.assertEqual(pub[0]["ssml"], "<speak>готово</speak>")
        self.assertEqual(pub[0]["sink"], "headset")
        self.assertEqual(pub[0]["request_id"], rid)

    def test_publish_avatar_tts_returns_request_id(self) -> None:
        rid = self.node._publish_avatar_tts("не умею")
        # uuid4().hex[:8] — 8 hex chars.
        self.assertEqual(len(rid), 8)
        self.assertTrue(all(c in "0123456789abcdef" for c in rid))

    def test_publish_avatar_tts_unique_request_ids(self) -> None:
        rid1 = self.node._publish_avatar_tts("раз")
        rid2 = self.node._publish_avatar_tts("два")
        self.assertNotEqual(rid1, rid2)


class TestPublishAvatarTtsOptionalArgs(unittest.TestCase):
    """Прокидывание опциональных voice/language в payload."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_optional_language_forwarded(self) -> None:
        self.node._publish_avatar_tts("hello", language="en")
        pub = _published_avatar_tts(self.node)
        self.assertEqual(pub[0]["language"], "en")
        self.assertNotIn("voice", pub[0])

    def test_optional_voice_forwarded(self) -> None:
        self.node._publish_avatar_tts("bonjour", voice="fr-f1")
        pub = _published_avatar_tts(self.node)
        self.assertEqual(pub[0]["voice"], "fr-f1")
        self.assertNotIn("language", pub[0])

    def test_no_optional_args_omits_keys(self) -> None:
        self.node._publish_avatar_tts("чисто")
        pub = _published_avatar_tts(self.node)
        self.assertNotIn("language", pub[0])
        self.assertNotIn("voice", pub[0])


class TestSupervisorOwnLinesUseAvatarChannel(unittest.TestCase):
    """ADR-0055 §C4: собственные реплики supervisor'а идут в /avatar/tts/request,
    не в /voice/tts/request.

    «Собственные реплики» — это прямой вызов ``_publish_avatar_tts``. Тест
    проверяет, что эта функция НЕ трогает ``_tts_request_pub`` (say-канал).
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_avatar_tts_does_not_publish_to_voice_tts_request(self) -> None:
        self.node._publish_avatar_tts("принял")
        self.assertEqual(len(_published_avatar_tts(self.node)), 1)
        # /voice/tts/request НЕ должен был получить ничего (say-канал).
        self.assertEqual(_published_voice_tts(self.node), [])


class TestPublishAvatarTtsEmptyTextDrop(unittest.TestCase):
    """Issue #2096 — guard пустого text в ``_publish_avatar_tts``.

    До фикса ``_publish_avatar_tts("")`` слал ``<speak></speak>`` в
    /avatar/tts/request → tts_node ловил MiniMax bad-request "text is empty"
    → CRITICAL в deploy-логе (run #34144712828). Теперь:
    * пустой/whitespace text → DROP (ничего в /avatar/tts/request)
    * возвращается пустой request_id (caller знает, что синтеза не будет).
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_empty_string_drops_request(self) -> None:
        rid = self.node._publish_avatar_tts("")
        self.assertEqual(rid, "")
        self.assertEqual(_published_avatar_tts(self.node), [])

    def test_whitespace_only_drops_request(self) -> None:
        rid = self.node._publish_avatar_tts("   \n\t  ")
        self.assertEqual(rid, "")
        self.assertEqual(_published_avatar_tts(self.node), [])

    def test_empty_text_does_not_touch_voice_tts_request(self) -> None:
        # Дополнительно: drop не должен «утекать» в /voice/tts/request
        # (say-канал) — ни при каких условиях.
        self.node._publish_avatar_tts("")
        self.assertEqual(_published_voice_tts(self.node), [])

    def test_non_empty_text_still_publishes(self) -> None:
        # Sanity: guard не должен сломать happy-path. Тест идёт после
        # test_empty_*, чтобы при падении guard'а видеть, что контракт
        # для непустого text по-прежнему работает.
        rid = self.node._publish_avatar_tts("готово")
        self.assertNotEqual(rid, "")
        self.assertEqual(len(_published_avatar_tts(self.node)), 1)


class TestPublishGripTtsEmptyTextDrop(unittest.TestCase):
    """Issue #2096 + #2137 — guard пустого text в ``_publish_grip_tts``.

    Зеркальная защита к ``_publish_avatar_tts``: пустой text из грип-пайплайна
    (после LLM-transform мог вернуть ``""``) тоже не должен уходить в
    /voice/tts/request. Также проверяем, что happy-path пишет в
    ``/voice/tts/request`` (динамики робота, §7.5) с ``priority="operator"``
    и без sink=headset (шлем остаётся только для собственных реплик ТАРС,
    инвариант 6b §7.4).
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_empty_string_drops_request(self) -> None:
        self.node._publish_grip_tts("")
        self.assertEqual(_published_voice_tts(self.node), [])

    def test_whitespace_only_drops_request(self) -> None:
        self.node._publish_grip_tts("   \n\t  ")
        self.assertEqual(_published_voice_tts(self.node), [])

    def test_non_empty_text_still_publishes(self) -> None:
        """Happy-path → /voice/tts/request с priority="operator" (§7.5)."""
        self.node._publish_grip_tts("мы начинаем")
        voice_pub = _published_voice_tts(self.node)
        self.assertEqual(len(voice_pub), 1)
        self.assertEqual(voice_pub[0]["ssml"], "<speak>мы начинаем</speak>")
        self.assertEqual(voice_pub[0]["priority"], "operator")
        # Не должен ничего утечь в /avatar/tts/request (шлем только для ТАРС).
        self.assertEqual(_published_avatar_tts(self.node), [])


if __name__ == "__main__":
    unittest.main()
