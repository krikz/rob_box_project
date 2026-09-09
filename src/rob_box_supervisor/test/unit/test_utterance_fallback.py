"""Тесты fallback-модуля ``rob_box_supervisor._utterance_fallback`` (issue #2233).

Контекст: см. docstring :mod:`rob_box_supervisor._utterance_fallback`.
Этот модуль живёт в supervisor только как fallback на случай, когда
base image не содержит ``rob_box_core.utterance`` (round-деплои с
не-SHA-pinned ``voice-assistant-humble-test`` тегом). Когда PR #2218
(utterance) merged в develop — supervisor должен использовать SoT, и
этот fallback остаётся как страховка.

Покрытие:
* ``escape_xml_text`` — XML-экранирование ``&`` / ``<`` / ``>``;
* ``Sink`` enum — строки-значения совпадают с SoT;
* ``Utterance.to_request()`` — контракт payload'а для tts_node
  (``ssml``, ``sink``, ``priority``, ``emotion``, ``voice``, ``language``).
"""
from __future__ import annotations

import unittest

from rob_box_supervisor._utterance_fallback import (
    DEFAULT_PRIORITY,
    Sink,
    Utterance,
    escape_xml_text,
)


class TestEscapeXmlText(unittest.TestCase):
    """XML-экранирование — единая функция для SoT и fallback."""

    def test_ampersand_escaped_first(self) -> None:
        # `&` → `&amp;` ДО `<` / `>` (иначе двойное экранирование).
        self.assertEqual(escape_xml_text("a & b"), "a &amp; b")

    def test_lt_gt_escaped(self) -> None:
        self.assertEqual(escape_xml_text("x < y > z"), "x &lt; y &gt; z")

    def test_combined_escapes(self) -> None:
        # Имитируем пример из docstring SoT.
        self.assertEqual(
            escape_xml_text("Ошибка: x<y & z>0"),
            "Ошибка: x&lt;y &amp; z&gt;0",
        )

    def test_plain_text_unchanged(self) -> None:
        self.assertEqual(escape_xml_text("plain"), "plain")

    def test_empty_string_safe(self) -> None:
        self.assertEqual(escape_xml_text(""), "")

    def test_none_safe(self) -> None:
        # text может прийти None из вызывающего — fallback не должен падать.
        self.assertEqual(escape_xml_text(None), "")  # type: ignore[arg-type]


class TestSinkEnum(unittest.TestCase):
    """Строковые значения Sink должны совпадать с SoT — иначе tts_node
    отфильтрует неизвестный sink (см. tts_node.py:2994).
    """

    def test_speakers_value(self) -> None:
        self.assertEqual(Sink.SPEAKERS.value, "speakers")

    def test_headset_value(self) -> None:
        self.assertEqual(Sink.HEADSET.value, "headset")

    def test_preview_value(self) -> None:
        self.assertEqual(Sink.PREVIEW.value, "preview")

    def test_str_construction_allowed(self) -> None:
        # SoT допускает ``Sink("speakers")`` — fallback тоже должен.
        self.assertEqual(Sink("speakers"), Sink.SPEAKERS)


class TestUtteranceToRequest(unittest.TestCase):
    """``Utterance.to_request()`` — контракт payload'а."""

    def test_minimal_payload(self) -> None:
        u = Utterance(text="привет", sink=Sink.SPEAKERS)
        payload = u.to_request()
        self.assertEqual(payload["ssml"], "<speak>привет</speak>")
        self.assertEqual(payload["sink"], "speakers")
        self.assertEqual(payload["priority"], DEFAULT_PRIORITY)
        self.assertEqual(payload["emotion"], "neutral")
        self.assertNotIn("voice", payload)
        self.assertNotIn("language", payload)
        self.assertNotIn("speech_id", payload)

    def test_priority_operator(self) -> None:
        u = Utterance(text="готово", sink=Sink.HEADSET, priority="operator")
        payload = u.to_request()
        self.assertEqual(payload["priority"], "operator")
        self.assertEqual(payload["sink"], "headset")

    def test_optional_voice_language_forwarded(self) -> None:
        u = Utterance(
            text="hello",
            sink=Sink.SPEAKERS,
            voice="fr-f1",
            language="fr-FR",
        )
        payload = u.to_request()
        self.assertEqual(payload["voice"], "fr-f1")
        self.assertEqual(payload["language"], "fr-FR")

    def test_speech_id_forwarded(self) -> None:
        u = Utterance(text="ok", sink=Sink.HEADSET, speech_id="abc123")
        self.assertEqual(u.to_request()["speech_id"], "abc123")

    def test_xml_escaping_in_ssml(self) -> None:
        # Символы `<`, `>`, `&` в text ДОЛЖНЫ быть экранированы в SSML.
        u = Utterance(text="x < y & z > 0", sink=Sink.SPEAKERS)
        ssml = u.to_request()["ssml"]
        self.assertIn("&lt;", ssml)
        self.assertIn("&amp;", ssml)
        self.assertIn("&gt;", ssml)
        self.assertNotIn("x <", ssml)  # без экранирования сломало бы SSML

    def test_str_sink_normalised(self) -> None:
        # supervisor иногда передаёт sink="headset" строкой (через JSON-литералы).
        # __post_init__ должен нормализовать до enum.
        u = Utterance(text="hi", sink="headset")  # type: ignore[arg-type]
        self.assertEqual(u.sink, Sink.HEADSET)
        self.assertEqual(u.to_request()["sink"], "headset")

    def test_extra_overridable_only_via_utterance(self) -> None:
        # Защита от случайного переопределения ssml/sink через extra.
        u = Utterance(
            text="main",
            sink=Sink.SPEAKERS,
            extra={"ssml": "EVIL", "custom_field": 42},
        )
        payload = u.to_request()
        # Utterance-поля выигрывают.
        self.assertEqual(payload["ssml"], "<speak>main</speak>")
        # Кастомные поля проходят.
        self.assertEqual(payload["custom_field"], 42)


class TestFallbackImportContract(unittest.TestCase):
    """Smoke: импорт из fallback работает и подменяет rob_box_core.utterance.

    Эти тесты НЕ мокают rob_box_core (он может быть или не быть в sys.path).
    Идея: после try/except в supervisor_node символы Sink/Utterance доступны
    независимо от того, есть ли rob_box_core. Здесь мы проверяем сам
    fallback-модуль (как самостоятельный контракт), без supervisor.
    """

    def test_module_importable(self) -> None:
        import importlib

        mod = importlib.import_module("rob_box_supervisor._utterance_fallback")
        self.assertTrue(hasattr(mod, "Sink"))
        self.assertTrue(hasattr(mod, "Utterance"))
        self.assertTrue(hasattr(mod, "escape_xml_text"))
        self.assertTrue(hasattr(mod, "DEFAULT_PRIORITY"))


if __name__ == "__main__":
    unittest.main()
