"""Unit-тесты для rob_box_core.utterance.

Покрывает:
* голый текст реплики (ADR-0160 §2.3): ``&``, ``<``, ``>``, юникод и строки
  через ``\n`` уходят как есть, без SSML и экранирования;
* контракт ``Utterance.to_request()`` — фиксированный набор полей и
  правильный приоритет (``Utterance``-поля выигрывают у ``extra``).

Экранирование и конверт ``<speak>`` — у адаптера Silero
(``rob_box_voice/test/unit/tts/test_silero_ssml_envelope_4239.py``).
См. ADR-0080 §1.3 (одна точка сборки), ADR-0160 §2.3.
"""

from __future__ import annotations

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
# Голый текст (ADR-0160 §2.3)
# ---------------------------------------------------------------------------


class TestPlainText:
    @pytest.mark.parametrize(
        "text",
        [
            "AT&T и Tom & Jerry",
            "1 < 2 & 3 > 2",
            "ёлка × ÷ ∞ → 👍",
            "Первая строка куплета\nВторая строка куплета",
            "foo &amp; bar",
        ],
    )
    def test_text_goes_to_request_as_is(self, text):
        assert Utterance(text=text).to_request()["text"] == text

    def test_no_ssml_in_request(self):
        req = Utterance(text="Привет, мир!").to_request()
        assert "ssml" not in req
        assert "<speak" not in req["text"]

    def test_utterance_has_no_ssml_builder(self):
        assert not hasattr(Utterance(text="x"), "ssml")

    def test_none_text_becomes_empty(self):
        u = Utterance(text=None, sink=Sink.SPEAKERS)  # type: ignore[arg-type]
        assert u.to_request()["text"] == ""


# ---------------------------------------------------------------------------
# to_request
# ---------------------------------------------------------------------------


class TestToRequest:
    def test_minimal_request(self):
        u = Utterance(text="hi", sink=Sink.SPEAKERS)
        req = u.to_request()
        assert req == {
            "text": "hi",
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

    def test_extra_cannot_override_text(self):
        """Utterance-поля выигрывают у extra — текст реплики один."""
        u = Utterance(text="x & y", extra={"text": "RAW"})
        req = u.to_request()
        assert req["text"] == "x & y"

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
        assert '"text": "a & b < c"' in payload
        assert '"sink": "headset"' in payload
        assert '"chunk": 1' in payload

    def test_producer_mimic_grip_tts(self):
        """Снимок payload'а, который шлёт ``supervisor._publish_grip_tts``
        (issue #2137) — ``Utterance(...).to_request()``, сравниваем ключи."""
        u = Utterance(
            text="готово",
            sink=Sink.SPEAKERS,
            priority=PRIORITY_OPERATOR,
            language="ru-RU",
        )
        req = u.to_request()
        # Обязательные поля grip-пайплайна.
        assert req["text"] == "готово"
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
