"""Unit-тесты для rob_box_core.utterance.

Покрывает:
* XML-экранирование ``&``, ``<``, ``>`` в тексте реплики;
* целостность «безопасного» текста (русский, юникод, цифры, пунктуация);
* контракт ``Utterance.to_request()`` — фиксированный набор полей и
  правильный приоритет (``Utterance``-поля выигрывают у ``extra``);
* инвариант «SSML не сыпется при спецсимволах» — сгенерированный
  ``<speak>...</speak>`` остаётся валидным XML.

См. ADR-0080 §1.3 (одна точка сборки), §2.3 инвариант 5/6
(экранирование на стороне сборщика, не TTS).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from rob_box_core.utterance import (
    ALLOWED_PRIORITIES,
    DEFAULT_PRIORITY,
    PRIORITY_BARGE_IN,
    PRIORITY_NORMAL,
    PRIORITY_OPERATOR,
    PRIORITY_REPLACE,
    Sink,
    Utterance,
)


# ---------------------------------------------------------------------------
# SSML escape
# ---------------------------------------------------------------------------


class TestXmlEscape:
    def test_ampersand_escaped(self):
        u = Utterance(text="AT&T и Tom & Jerry", sink=Sink.SPEAKERS)
        # & всегда экранируется первым (иначе повторное экранирование
        # уже-экранированных сущностей).
        assert u.ssml == "<speak>AT&amp;T и Tom &amp; Jerry</speak>"

    def test_less_than_escaped(self):
        u = Utterance(text="x<y", sink=Sink.SPEAKERS)
        assert u.ssml == "<speak>x&lt;y</speak>"

    def test_greater_than_escaped(self):
        u = Utterance(text="x>y", sink=Sink.SPEAKERS)
        assert u.ssml == "<speak>x&gt;y</speak>"

    def test_all_three_combined(self):
        u = Utterance(text="<a href='x'>&</a>", sink=Sink.SPEAKERS)
        assert u.ssml == "<speak>&lt;a href='x'&gt;&amp;&lt;/a&gt;</speak>"

    def test_dangerous_text_produces_valid_xml(self):
        """Главный инвариант DoD: текст с ``&`` и ``<`` даёт валидный XML
        и произносится без потери символов."""
        text = "Ошибка 5 < 10 & нужна проверка"
        u = Utterance(text=text, sink=Sink.SPEAKERS)
        # Парсер не должен падать и не должен «съесть» < или &.
        root = ET.fromstring(u.ssml)
        assert root.tag == "speak"
        assert root.text == text

    def test_safe_text_unchanged(self):
        u = Utterance(text="Привет, мир!", sink=Sink.SPEAKERS)
        assert u.ssml == "<speak>Привет, мир!</speak>"

    def test_unicode_preserved(self):
        text = "ёлка × ÷ ∞ → 👍"
        u = Utterance(text=text, sink=Sink.SPEAKERS)
        assert u.ssml == f"<speak>{text}</speak>"
        # Юникод не должен пострадать и после round-trip через ET.
        assert ET.fromstring(u.ssml).text == text

    def test_existing_ampersand_entity_not_double_escaped(self):
        """Если caller уже прислал ``&amp;`` как часть подготовленного
        текста, мы экранируем амперсанд — на выходе ``&amp;amp;``.
        Это ожидаемо: Utterance не разбирает SSML-вход, только экранирует.
        Документируем поведение явно, чтобы не было сюрпризов."""
        u = Utterance(text="foo &amp; bar", sink=Sink.SPEAKERS)
        # ``&`` в "amp;" экранируется в "amp;amp;".
        assert u.ssml == "<speak>foo &amp;amp; bar</speak>"

    def test_empty_text(self):
        u = Utterance(text="", sink=Sink.SPEAKERS)
        assert u.ssml == "<speak></speak>"

    def test_none_text_becomes_empty(self):
        u = Utterance(text=None, sink=Sink.SPEAKERS)  # type: ignore[arg-type]
        assert u.ssml == "<speak></speak>"


# ---------------------------------------------------------------------------
# to_request
# ---------------------------------------------------------------------------


class TestToRequest:
    def test_minimal_request(self):
        u = Utterance(text="hi", sink=Sink.SPEAKERS)
        req = u.to_request()
        assert req == {
            "ssml": "<speak>hi</speak>",
            "sink": "speakers",
            "priority": DEFAULT_PRIORITY,
            "emotion": "neutral",
        }

    def test_headset_sink(self):
        u = Utterance(text="reply", sink=Sink.HEADSET, priority=PRIORITY_OPERATOR)
        req = u.to_request()
        assert req["sink"] == "headset"
        assert req["priority"] == "operator"

    def test_preview_sink(self):
        u = Utterance(text="sample", sink=Sink.PREVIEW)
        req = u.to_request()
        assert req["sink"] == "preview"

    def test_string_sink_normalized(self):
        """Допускаем ``sink="speakers"`` (из JSON-литералов) — Sink-Enum
        нормализует значение в ``__post_init__``."""
        u = Utterance(text="x", sink="speakers")  # type: ignore[arg-type]
        assert u.sink is Sink.SPEAKERS
        assert u.to_request()["sink"] == "speakers"

    def test_optional_fields_omitted_when_none(self):
        u = Utterance(text="x")
        req = u.to_request()
        assert "voice" not in req
        assert "language" not in req
        assert "speech_id" not in req

    def test_optional_fields_present_when_set(self):
        u = Utterance(
            text="x",
            voice="alena",
            language="ru-RU",
            speech_id="sid-1",
        )
        req = u.to_request()
        assert req["voice"] == "alena"
        assert req["language"] == "ru-RU"
        assert req["speech_id"] == "sid-1"

    def test_extra_merged_into_request(self):
        u = Utterance(
            text="chunk",
            extra={"chunk": 1, "batch_id": "b1", "batch_index": 1, "batch_total": 3},
        )
        req = u.to_request()
        assert req["chunk"] == 1
        assert req["batch_id"] == "b1"
        assert req["batch_index"] == 1
        assert req["batch_total"] == 3

    def test_extra_cannot_override_ssml(self):
        """Utterance-поля выигрывают у extra — иначе caller может
        вернуть «сырой» ``<speak>`` без экранирования."""
        u = Utterance(text="x & y", extra={"ssml": "<speak>RAW</speak>"})
        req = u.to_request()
        assert req["ssml"] == "<speak>x &amp; y</speak>"

    def test_extra_cannot_override_sink(self):
        u = Utterance(text="x", sink=Sink.HEADSET, extra={"sink": "speakers"})
        assert u.to_request()["sink"] == "headset"

    def test_invalid_priority_rejected(self):
        with pytest.raises(ValueError, match="priority"):
            Utterance(text="x", priority="bogus")

    def test_all_allowed_priorities(self):
        for p in (
            PRIORITY_NORMAL,
            PRIORITY_OPERATOR,
            PRIORITY_BARGE_IN,
            PRIORITY_REPLACE,
        ):
            u = Utterance(text="x", priority=p)
            assert u.to_request()["priority"] == p
        # И набор публичный.
        assert PRIORITY_NORMAL in ALLOWED_PRIORITIES
        assert PRIORITY_OPERATOR in ALLOWED_PRIORITIES
        assert PRIORITY_BARGE_IN in ALLOWED_PRIORITIES
        assert PRIORITY_REPLACE in ALLOWED_PRIORITIES

    def test_emotion_defaults_to_neutral(self):
        u = Utterance(text="x", emotion=None)
        assert u.to_request()["emotion"] == "neutral"

    def test_explicit_emotion_kept(self):
        u = Utterance(text="x", emotion="happy")
        assert u.to_request()["emotion"] == "happy"


# ---------------------------------------------------------------------------
# Round-trip: dict → json → Utterance-style shape
# ---------------------------------------------------------------------------


class TestJsonRoundtrip:
    def test_payload_jsonable(self):
        import json

        u = Utterance(
            text="a & b < c",
            sink=Sink.HEADSET,
            priority=PRIORITY_OPERATOR,
            extra={"chunk": 1},
        )
        # asdict-сериализация + ensure_ascii=False — это ровно то, что
        # делают все producer'ы (json.dumps(..., ensure_ascii=False)).
        payload = json.dumps(u.to_request(), ensure_ascii=False)
        assert "<speak>a &amp; b &lt; c</speak>" in payload
        assert '"sink": "headset"' in payload
        assert '"chunk": 1' in payload

    def test_producer_mimic_grip_tts(self):
        """Снимок payload'а, который шлёт ``supervisor._publish_grip_tts``
        (issue #2137) — заменяем ``f\"<speak>{text}</speak>`` на
        ``Utterance(...).to_request()`` и сравниваем ключи."""
        u = Utterance(
            text="готово",
            sink=Sink.SPEAKERS,
            priority=PRIORITY_OPERATOR,
            language="ru-RU",
        )
        req = u.to_request()
        # Обязательные поля grip-пайплайна.
        assert req["ssml"] == "<speak>готово</speak>"
        assert req["priority"] == "operator"
        assert req["language"] == "ru-RU"
        # И никакого голого текста рядом — producer допишет при
        # необходимости.

    def test_producer_mimic_avatar_tts(self):
        """Снимок payload'а ``supervisor._publish_avatar_tts`` (issue #2096,
        #2116) — sink, request_id, voice, language."""
        u = Utterance(
            text="включил",
            sink=Sink.HEADSET,
            voice="alena",
            language="ru-RU",
            speech_id="req-abc",
        )
        req = u.to_request()
        assert req["sink"] == "headset"
        assert req["voice"] == "alena"
        assert req["language"] == "ru-RU"
        assert req["speech_id"] == "req-abc"
