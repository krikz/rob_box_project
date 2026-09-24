"""Issue #2874 — чистая политика «что озвучить из свободного текста хода»."""

from __future__ import annotations

from rob_box_voice.core.speak_helpers import split_into_chunks
from rob_box_voice.core.turn_speech import (
    FIRST_SENTENCE_LIMIT,
    MUSIC_MAX_CHUNKS,
    TurnSpeechHold,
    decide_turn_speech,
    first_sentence,
    wants_lyrics,
)

# Живой монолог 23.09 17:30:32 — 11 TTS-чанков после третьего ретрая.
_MONOLOGUE = (
    "Поехали, зал — Still Dre открывает вечер. "
    + " ".join(
        f"Но чё, gang, я ж DJ на этом железе, фраза номер {i}, "
        "у меня нет mp3-библиотеки хип-хопа и лицензированных битов Dr. Dre."
        for i in range(12)
    )
)


def _decide(text, **kw):
    return decide_turn_speech(text, n_chunks=len(split_into_chunks(text)), **kw)


class TestFirstSentence:
    def test_takes_first_sentence(self):
        assert first_sentence("Раз. Два! Три?") == "Раз."

    def test_long_sentence_cut_on_word_with_ellipsis(self):
        out = first_sentence("слово " * 60)
        assert len(out) <= FIRST_SENTENCE_LIMIT + 1
        assert out.endswith("…")
        assert "слов…" not in out  # не рвём слово пополам

    def test_empty(self):
        assert first_sentence("   ") == ""


class TestDecideTurnSpeech:
    def test_normal_turn_unchanged(self):
        text = "Привет! Как дела? Рад тебя слышать."
        assert _decide(text) == text

    def test_retry_dispatched_is_silent(self):
        assert _decide("Йо, gangsta party!", retry_dispatched=True) is None

    def test_retracted_is_silent(self):
        assert _decide("Йо!", retracted=True) is None

    def test_budget_exhausted_gives_first_sentence(self):
        out = _decide(_MONOLOGUE, budget_exhausted=True)
        assert out == "Поехали, зал — Still Dre открывает вечер."
        assert len(split_into_chunks(_MONOLOGUE)) > 10  # было 11+ чанков
        assert len(split_into_chunks(out)) == 1

    def test_long_music_text_trimmed(self):
        assert len(split_into_chunks(_MONOLOGUE)) > MUSIC_MAX_CHUNKS
        out = _decide(_MONOLOGUE, music_context=True)
        assert out == "Поехали, зал — Still Dre открывает вечер."

    def test_long_text_outside_music_untouched(self):
        assert _decide(_MONOLOGUE) == _MONOLOGUE

    def test_long_lyrics_on_request_untouched(self):
        """«Зачитай рэп» — длинный ответ и есть исполнение (#980)."""
        out = _decide(_MONOLOGUE, music_context=True, lyrics_requested=True)
        assert out == _MONOLOGUE

    def test_short_music_text_untouched(self):
        text = "Вечеринка начинается, трек номер раз — Still Dre."
        assert _decide(text, music_context=True) == text

    def test_empty_text(self):
        assert _decide("") is None


class TestWantsLyrics:
    def test_rap_request(self):
        assert wants_lyrics("зачитай рэп про кота")

    def test_dj_request_is_not_lyrics(self):
        assert not wants_lyrics(
            "Ты диджей Снупдог, у нас гангста-вечеринка на 10 минут. "
            "Играй по очереди: Still Dre, Next Episode"
        )

    def test_none(self):
        assert not wants_lyrics(None)


class TestTurnSpeechHold:
    def test_hold_and_retract(self):
        hold = TurnSpeechHold()
        hold.hold("текст", "юзер")
        hold.retract()
        assert (hold.text, hold.user_input, hold.retracted) == (
            "текст", "юзер", True,
        )
