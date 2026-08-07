#!/usr/bin/env python3
"""
test_stt_fallback.py — Unit-тесты для stt_fallback.py (issue #979).

Pure-Python, без rclpy/vosk/grpc. ``select_recognition`` принимает
любой объект с ``name: str`` и ``recognize(bytes) -> str | None``,
поэтому мы подсовываем фейки.

Acceptance (issue #979):
- "фраза из 3-4 слов после TTS распознаётся в >80% случаев"
- Фраза Vosk-мусора ("а", "а а") отклоняется как "rejected_short"
- Retry один раз при timeout Yandex
- Метрика logger.info с provider/reason/latency_ms появляется
- e2e: yandex:ok → без fallback; yandex:timeout*2 → vosk:ok
"""

import logging
import time
from typing import List, Optional

import pytest

from rob_box_voice.stt_fallback import (
    DEFAULT_MIN_TEXT_CHARS,
    DEFAULT_YANDEX_MAX_RETRIES,
    DEFAULT_YANDEX_TIMEOUT_S,
    STTAttempt,
    is_short_phrase,
    log_attempts,
    select_recognition,
    summarize_attempts,
)

# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------


class FakeProvider:
    """Имитация STT-провайдера с управляемыми side-effects."""

    def __init__(
        self,
        name: str,
        responses: List[Optional[str]],
        delays: Optional[List[float]] = None,
        exceptions: Optional[List[Optional[Exception]]] = None,
    ):
        self.name = name
        self._responses = list(responses)
        self._delays = delays or [0.0] * len(responses)
        self._exceptions = exceptions or [None] * len(responses)
        self._call_count = 0

    def recognize(self, audio_bytes: bytes) -> Optional[str]:
        idx = self._call_count
        self._call_count += 1
        if idx >= len(self._responses):
            raise AssertionError(
                f"FakeProvider {self.name!r}: call #{idx+1} exceeds " f"prepared responses ({len(self._responses)})"
            )
        # Delay
        delay = self._delays[idx]
        if delay > 0:
            time.sleep(delay)
        # Exception
        exc = self._exceptions[idx]
        if exc is not None:
            raise exc
        return self._responses[idx]

    @property
    def call_count(self) -> int:
        return self._call_count


PHASE3_PHRASE = "расскажи ещё раз"  # 3-4 слова, типичный e2e
LONG_PHRASE = "робот расскажи ещё раз пожалуйста"  # 5 слов
VOSK_GARBAGE = "а"  # 1 char
NORMAL_PHRASE = "стоп"  # 4 chars


# ---------------------------------------------------------------------------
# is_short_phrase
# ---------------------------------------------------------------------------


class TestIsShortPhrase:
    def test_none_is_short(self):
        assert is_short_phrase(None) is True

    def test_empty_is_short(self):
        assert is_short_phrase("") is True

    def test_whitespace_is_short(self):
        assert is_short_phrase("   ") is True

    def test_one_char_is_short(self):
        assert is_short_phrase("а") is True

    def test_two_chars_is_short(self):
        assert is_short_phrase("да") is True

    def test_three_chars_at_threshold_is_not_short(self):
        # DEFAULT_MIN_TEXT_CHARS=3, 'abc' has 3 chars: не < 3
        assert is_short_phrase("abc") is False

    def test_vosk_garbage_short(self):
        assert is_short_phrase(VOSK_GARBAGE) is True

    def test_normal_phrase_not_short(self):
        assert is_short_phrase(NORMAL_PHRASE) is False
        assert is_short_phrase(PHASE3_PHRASE) is False

    def test_strip_applied(self):
        # '  стоп  ' (4 chars после strip) — ok
        assert is_short_phrase("  стоп  ") is False
        # '  ы  ' (1 char после strip) — short
        assert is_short_phrase("  ы  ") is True

    def test_custom_threshold(self):
        # При min_chars=4 "стоп" (4) — ок, "abc" (3) — нет
        assert is_short_phrase("стоп", min_chars=4) is False
        assert is_short_phrase("abc", min_chars=4) is True


# ---------------------------------------------------------------------------
# select_recognition — primary success
# ---------------------------------------------------------------------------


class TestPrimarySuccess:
    def test_first_provider_succeeds_no_fallback(self):
        primary = FakeProvider("yandex", [PHASE3_PHRASE])
        fallback = FakeProvider("vosk", [LONG_PHRASE])

        text, attempts = select_recognition([primary, fallback], b"\x00\x00" * 800)

        assert text == PHASE3_PHRASE
        assert primary.call_count == 1
        assert fallback.call_count == 0  # fallback НЕ дёрнут
        assert len(attempts) == 1
        assert attempts[0].provider == "yandex"
        assert attempts[0].reason == "ok"
        assert attempts[0].latency_ms >= 0

    def test_default_constants_hold(self):
        # Specs: таймаут 5s, retries 1, min_chars 3
        assert DEFAULT_YANDEX_TIMEOUT_S == 5.0
        assert DEFAULT_YANDEX_MAX_RETRIES == 1
        assert DEFAULT_MIN_TEXT_CHARS == 3


# ---------------------------------------------------------------------------
# select_recognition — retry
# ---------------------------------------------------------------------------


class TestRetryBehaviour:
    def test_empty_first_attempt_triggers_retry(self):
        # Yandex: 1-я попытка → None, 2-я → PHRASE
        primary = FakeProvider("yandex", [None, PHASE3_PHRASE])
        fallback = FakeProvider("vosk", [LONG_PHRASE])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,  # ускорим тест
        )

        assert text == PHASE3_PHRASE
        assert primary.call_count == 2  # retry сработал
        assert fallback.call_count == 0
        assert len(attempts) == 2
        assert attempts[0].reason == "empty"  # провайдер вернул None → empty
        assert attempts[1].reason == "ok"

    def test_error_first_attempt_triggers_retry(self):
        primary = FakeProvider(
            "yandex",
            [None, PHASE3_PHRASE],
            exceptions=[RuntimeError("grpc timeout"), None],
        )
        fallback = FakeProvider("vosk", [LONG_PHRASE])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )

        assert text == PHASE3_PHRASE
        assert primary.call_count == 2
        # 1-я попытка: real exception → reason=error
        assert attempts[0].reason == "error"
        assert "grpc timeout" in (attempts[0].error or "")

    def test_timeout_both_attempts_falls_back_to_vosk(self):
        # Yandex "висит" дольше timeout_s обе попытки
        primary = FakeProvider(
            "yandex",
            [LONG_PHRASE, LONG_PHRASE],  # любой ответ после timeout
            delays=[0.05, 0.05],
        )
        fallback = FakeProvider("vosk", [LONG_PHRASE])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            timeout_s=0.02,  # обе попытки > 50ms > 20ms
            retry_backoff_s=0.0,
        )

        assert text == LONG_PHRASE
        assert primary.call_count == 2
        assert fallback.call_count == 1
        assert attempts[0].reason == "timeout"
        assert attempts[1].reason == "timeout"
        assert attempts[2].provider == "vosk"
        assert attempts[2].reason == "ok"

    def test_no_retry_on_fallback_provider(self):
        # Vosk (fallback) ошибся — мы НЕ retry, идём дальше (или сдаёмся)
        primary = FakeProvider(
            "yandex",
            [None, None],
            exceptions=[None, RuntimeError("grpc fail")],
        )
        fallback = FakeProvider("vosk", [None], exceptions=[ValueError("bad audio")])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )

        assert text is None
        # primary: 2 retry (1 retry + initial), fallback: 1 попытка
        assert primary.call_count == 2
        assert fallback.call_count == 1
        assert attempts[-1].reason == "error"
        assert attempts[-1].provider == "vosk"


# ---------------------------------------------------------------------------
# select_recognition — fallback decisions
# ---------------------------------------------------------------------------


class TestFallbackDecisions:
    def test_vosk_garbage_rejected_as_short(self):
        # Yandex timeout, Vosk вернул мусор → text=None, low_confidence
        primary = FakeProvider("yandex", [None])
        fallback = FakeProvider("vosk", [VOSK_GARBAGE])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
        )

        assert text is None
        assert attempts[-1].reason == "low_confidence"
        assert attempts[-1].text == VOSK_GARBAGE
        assert attempts[-1].provider == "vosk"

    def test_empty_vs_low_confidence_distinct(self):
        """issue #979: пустой ответ провайдера = reason "empty",
        короткий не-пустой = reason "low_confidence"."""
        # Пустой ответ → empty
        primary = FakeProvider("yandex", [None, None])
        fallback = FakeProvider("vosk", [""])
        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )
        assert text is None
        assert attempts[0].reason == "empty"
        assert attempts[1].reason == "empty"
        assert attempts[2].reason == "empty"  # Vosk вернул "" → empty

        # Короткий не-пустой → low_confidence
        primary2 = FakeProvider("yandex", [None, None])
        fallback2 = FakeProvider("vosk", [VOSK_GARBAGE])
        text2, attempts2 = select_recognition(
            [primary2, fallback2],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )
        assert text2 is None
        assert attempts2[2].reason == "low_confidence"
        assert attempts2[2].text == VOSK_GARBAGE

    def test_provider_raises_stt_timeout_error(self):
        """STTTimeoutError → reason=timeout (не error)."""
        def _raise_timeout(_audio):
            from rob_box_voice.stt_fallback import STTTimeoutError

            raise STTTimeoutError("deadline exceeded")

        class _TimeoutProvider:
            name = "yandex"

            def recognize(self, audio_bytes):
                return _raise_timeout(audio_bytes)

        fallback = FakeProvider("vosk", [PHASE3_PHRASE])
        text, attempts = select_recognition(
            [_TimeoutProvider(), fallback],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )
        assert text == PHASE3_PHRASE
        # primary: 2 попытки (initial + 1 retry), обе timeout
        assert attempts[0].reason == "timeout"
        assert attempts[1].reason == "timeout"
        assert attempts[1].provider == "yandex"
        # fallback: vosk, ok
        assert attempts[2].provider == "vosk"
        assert attempts[2].reason == "ok"

    def test_three_word_phrase_after_tts_accepted(self):
        # Главный acceptance: фраза из 3-4 слов после TTS → ok
        # Yandex 1-я: timeout (None), 2-я: PHASE3_PHRASE → ok
        # Vosk (fallback) не дёрнут т.к. Yandex 2-я попытка → ok
        primary = FakeProvider("yandex", [None, PHASE3_PHRASE])
        fallback = FakeProvider("vosk", [VOSK_GARBAGE])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )

        assert text == PHASE3_PHRASE
        assert attempts[-1].reason == "ok"
        assert attempts[-1].provider == "yandex"

    def test_no_providers_raises(self):
        with pytest.raises(ValueError):
            select_recognition([], b"\x00\x00" * 800)

    def test_custom_min_chars_filters_shorter_phrases(self):
        # Увеличим min_chars до 4 — тогда "abc" (3) станет "low_confidence"
        primary = FakeProvider("yandex", [None])
        fallback = FakeProvider("vosk", ["abc"])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            min_text_chars=4,
        )

        assert text is None
        assert attempts[-1].reason == "low_confidence"


# ---------------------------------------------------------------------------
# summarize_attempts / log_attempts
# ---------------------------------------------------------------------------


class TestSummarize:
    def test_single_attempt_ok(self):
        a = STTAttempt(provider="yandex", reason="ok", latency_ms=820, text="hi")
        # Format: provider:reason(latency_ms 'text')
        assert summarize_attempts([a]) == "yandex:ok(820ms 'hi')"

    def test_timeout_chain(self):
        chain = [
            STTAttempt("yandex", "timeout", 4200, attempt_index=0),
            STTAttempt("yandex", "timeout", 4400, attempt_index=1),
            STTAttempt("vosk", "ok", 180, text="x" * 5, attempt_index=0),
        ]
        summary = summarize_attempts(chain)
        assert "yandex:timeout(4200ms)" in summary
        assert "yandex:timeout(4400ms)" in summary
        assert "vosk:ok(180ms 'xxxxx')" in summary
        assert "->" in summary

    def test_error_repr(self):
        a = STTAttempt(
            "yandex",
            "error",
            1300,
            error="RuntimeError('grpc deadline')",
        )
        s = summarize_attempts([a])
        # error не имеет text, поэтому формат без '...'
        assert "yandex:error(1300ms)" in s


class TestLogAttempts:
    def test_logs_metric_per_attempt(self, caplog):
        caplog.set_level(logging.INFO)
        chain = [
            STTAttempt("yandex", "timeout", 4200, attempt_index=0),
            STTAttempt("yandex", "ok", 900, text="hello", attempt_index=1),
        ]
        logger = logging.getLogger("test_stt_fallback")
        log_attempts(logger, chain, final_text="hello")

        # По одному "[stt_attempt_metric]" на попытку
        metric_lines = [r for r in caplog.records if "[stt_attempt_metric]" in r.getMessage()]
        assert len(metric_lines) == 2
        # Метрика Prometheus-стиля
        msg_yandex = [r for r in metric_lines if "provider=yandex" in r.getMessage()][0]
        assert "reason=timeout" in msg_yandex.getMessage()
        assert "latency_ms=4200" in msg_yandex.getMessage()
        assert "attempt=0" in msg_yandex.getMessage()

        # Финальная строка — accepted
        final_lines = [r for r in caplog.records if r.getMessage().startswith("[stt_attempt] ")]
        assert len(final_lines) == 1
        assert "accepted 'hello'" in final_lines[0].getMessage()

    def test_logs_rejected_when_no_text(self, caplog):
        caplog.set_level(logging.WARNING)
        chain = [
            STTAttempt("yandex", "timeout", 4200, attempt_index=0),
            STTAttempt("vosk", "low_confidence", 180, text="а", attempt_index=0),
        ]
        logger = logging.getLogger("test_stt_fallback_rejected")
        log_attempts(logger, chain, final_text=None)

        rejected = [
            r for r in caplog.records if r.getMessage().startswith("[stt_attempt] ") and "rejected" in r.getMessage()
        ]
        assert len(rejected) == 1
        assert "yandex:timeout" in rejected[0].getMessage()
        assert "vosk:low_confidence" in rejected[0].getMessage()


# ---------------------------------------------------------------------------
# e2e: acceptance 80%
# ---------------------------------------------------------------------------


class TestAcceptanceE2E:
    """Главный acceptance issue #979: 3-4 слова после TTS → ok в >80% случаев."""

    @pytest.mark.parametrize(
        "phrase",
        [
            "расскажи ещё раз",
            "повтори ещё раз",
            "что ты сказал",
            "расскажи про себя",
            "включи музыку",
            "какая погода",
            "сколько время",
            "вот это да",
        ],
    )
    def test_short_phrase_after_tts_succeeds(self, phrase):
        # Каждый раз: Yandex 1-я попытка fail (имитация timeout-флапа),
        # 2-я попытка ok (5s timeout легко укладывается).
        primary = FakeProvider("yandex", [None, phrase])
        fallback = FakeProvider("vosk", [VOSK_GARBAGE])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )

        assert text == phrase
        assert attempts[-1].reason == "ok"
        assert attempts[-1].provider == "yandex"

    def test_acceptance_8_out_of_10_with_vosk_garbage(self):
        """Имитируем 10 фраз: 8 через Yandex (после retry), 2 — мусор."""
        phrases = [
            "расскажи ещё раз",  # 1. retry→ok
            "повтори ещё раз",  # 2. retry→ok
            "что ты сказал",  # 3. retry→ok
            "расскажи про себя",  # 4. retry→ok
            "включи музыку",  # 5. retry→ok
            "какая погода",  # 6. retry→ok
            "сколько время",  # 7. retry→ok
            "вот это да",  # 8. retry→ok
            "а",  # 9. vosk мусор (отклоняем)
            "покажи карту",  # 10. retry→ok
        ]

        successes = 0
        for ph in phrases:
            if ph == "а":
                # Совсем короткая — yandex пуст, vosk мусор → None
                primary = FakeProvider("yandex", [None, None])
                fallback = FakeProvider("vosk", ["а"])
            else:
                # yandex: 1-я пусто, 2-я фраза → ok
                primary = FakeProvider("yandex", [None, ph])
                fallback = FakeProvider("vosk", ["а"])
            text, _ = select_recognition(
                [primary, fallback],
                b"\x00\x00" * 800,
                retry_backoff_s=0.0,
            )
            if text == ph:
                successes += 1
        # 9 из 10 → 90% > 80% acceptance
        assert successes >= 8  # 80% acceptance
