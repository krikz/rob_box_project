#!/usr/bin/env python3
"""
test_stt_fallback.py — Unit-тесты для stt_fallback.py (issue #979).

Pure-Python, без rclpy/vosk/grpc. ``select_recognition`` принимает
любой объект с ``name: str`` и ``recognize(bytes) -> str | None``,
поэтому мы подсовываем фейки.

Acceptance (issue #979):
- "фраза из 3-4 слов после TTS распознаётся в >80% случаев"
- Фраза Vosk-мусора ("а", "а а") отклоняется как "rejected_short"
- Retry один раз при сетевой ошибке Yandex (timeout — без повтора, issue #2924)
- Метрика logger.info с provider/reason/latency_ms появляется
- e2e: yandex:ok → без fallback; yandex:timeout*2 → vosk:ok
"""

import logging
import time
from typing import List, Optional

import pytest

from rob_box_voice.stt_fallback import (
    DEFAULT_MAX_TOTAL_BUDGET_S,
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
        # Specs (issue #979): таймаут 5s, retries 1, min_chars 3
        # Specs (issue #1477): таймаут 12s (фразы 4-6с + FULL_DATA fallback)
        assert DEFAULT_YANDEX_TIMEOUT_S == 12.0
        assert DEFAULT_YANDEX_MAX_RETRIES == 1
        assert DEFAULT_MIN_TEXT_CHARS == 3


# ---------------------------------------------------------------------------
# select_recognition — retry
# ---------------------------------------------------------------------------


class TestRetryBehaviour:
    def test_empty_does_not_retry_moves_to_next_provider(self):
        """Issue #2767: ``empty`` — НЕ транзиентный сбой, повтор того же
        провайдера на тех же байтах бесполезен (гарантированно даст тот
        же ``empty``). Живой лог 23.09:
        ``minimax:empty(3641ms)->minimax:empty(3866ms)`` — retry на empty
        тратил лишние ~3.6с на КАЖДУЮ фразу без единого шанса на успех.
        Теперь primary пробуется РОВНО один раз и цепочка сразу идёт
        дальше, даже если ``max_retries`` в политике > 0.
        """
        # только 1 ответ — retry на empty упал бы (issue #2767)
        primary = FakeProvider("yandex", [None])
        fallback = FakeProvider("vosk", [PHASE3_PHRASE])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )

        assert text == PHASE3_PHRASE
        assert primary.call_count == 1  # НЕТ повтора на empty
        assert fallback.call_count == 1
        assert len(attempts) == 2
        assert attempts[0].reason == "empty"
        assert attempts[0].attempt_index == 0
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

    def test_timeout_is_not_retried_falls_back_to_vosk(self):
        # Issue #2924: Yandex "висит" дольше timeout_s — второй заход в той же
        # фразе НЕ делается (до фикса было yandex:timeout->yandex:timeout).
        primary = FakeProvider(
            "yandex",
            [LONG_PHRASE, LONG_PHRASE],  # любой ответ после timeout
            delays=[0.05, 0.05],
        )
        fallback = FakeProvider("vosk", [LONG_PHRASE])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            timeout_s=0.02,  # попытка > 50ms > 20ms
            retry_backoff_s=0.0,
        )

        assert text == LONG_PHRASE
        assert primary.call_count == 1
        assert fallback.call_count == 1
        assert attempts[0].reason == "timeout"
        assert attempts[1].provider == "vosk"
        assert attempts[1].reason == "ok"

    def test_no_retry_on_fallback_provider(self):
        # Vosk (fallback) ошибся — мы НЕ retry, идём дальше (или сдаёмся).
        # Оба вызова primary — реальные транзиентные ошибки (не empty,
        # issue #2767: empty не ретраится вовсе, здесь мы теста ради
        # проверяем, что retry primary'я — это именно про index==0, а не
        # про fallback).
        primary = FakeProvider(
            "yandex",
            [None, None],
            exceptions=[
                RuntimeError("grpc fail 1"),
                RuntimeError("grpc fail 2"),
            ],
        )
        fallback = FakeProvider(
            "vosk", [None], exceptions=[ValueError("bad audio")]
        )

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
        # Yandex timeout, Vosk вернул мусор → text=VOSK_GARBAGE (не None!),
        # reason=low_confidence. caller (speech_audio_callback) по непустому
        # тексту отличает rejected(short) от rejected(empty) и говорит
        # «не расслышал» вместо молчания (issue #979 acceptance).
        primary = FakeProvider("yandex", [None])
        fallback = FakeProvider("vosk", [VOSK_GARBAGE])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
        )

        assert text == VOSK_GARBAGE
        assert attempts[-1].reason == "low_confidence"
        assert attempts[-1].text == VOSK_GARBAGE
        assert attempts[-1].provider == "vosk"

    def test_empty_vs_low_confidence_distinct(self):
        """issue #979: пустой ответ провайдера = reason "empty",
        короткий не-пустой = reason "low_confidence". Issue #2767: ни
        один из них не ретраится — по одной попытке на провайдера."""
        # Пустой ответ → empty (без retry — issue #2767)
        primary = FakeProvider("yandex", [None])
        fallback = FakeProvider("vosk", [""])
        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )
        assert text is None
        assert len(attempts) == 2
        assert attempts[0].reason == "empty"
        assert attempts[1].reason == "empty"  # Vosk вернул "" → empty

        # Короткий не-пустой → low_confidence, но текст возвращается
        # (rejected(short), не rejected(empty)) — caller переспросит.
        primary2 = FakeProvider("yandex", [None])
        fallback2 = FakeProvider("vosk", [VOSK_GARBAGE])
        text2, attempts2 = select_recognition(
            [primary2, fallback2],
            b"\x00\x00" * 800,
            retry_backoff_s=0.0,
        )
        assert text2 == VOSK_GARBAGE
        assert attempts2[1].reason == "low_confidence"
        assert attempts2[1].text == VOSK_GARBAGE

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
        # primary: 1 попытка, timeout НЕ ретраится (issue #2924)
        assert attempts[0].reason == "timeout"
        assert attempts[0].provider == "yandex"
        # fallback: vosk, ok
        assert attempts[1].provider == "vosk"
        assert attempts[1].reason == "ok"

    def test_three_word_phrase_after_tts_accepted(self):
        # Главный acceptance: фраза из 3-4 слов после TTS → ok
        # Yandex 1-я: моргнула сеть (транзиентный error — ретраится; issue
        # #2767: ``empty`` НЕ ретраится, #2924: ``timeout`` тоже НЕ
        # ретраится), 2-я: PHASE3_PHRASE → ok. Vosk (fallback) не дёрнут.
        primary = FakeProvider(
            "yandex",
            [None, PHASE3_PHRASE],
            exceptions=[RuntimeError("UNAVAILABLE: network flap"), None],
        )
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
        # Увеличим min_chars до 4 — тогда "abc" (3) станет "low_confidence",
        # но текст вернётся (rejected(short)) — caller переспросит.
        primary = FakeProvider("yandex", [None])
        fallback = FakeProvider("vosk", ["abc"])

        text, attempts = select_recognition(
            [primary, fallback],
            b"\x00\x00" * 800,
            min_text_chars=4,
        )

        assert text == "abc"
        assert attempts[-1].reason == "low_confidence"


# ---------------------------------------------------------------------------
# select_recognition — phrase budget (issue #2767 п.3)
# ---------------------------------------------------------------------------


class _FakeClock:
    """Управляемые монотонные часы — бюджет тестируем без реального sleep.

    На Windows monotonic-тик ~15.6мс (issue #2799/PR #2800) — реальный
    time.sleep() в тесте бюджета был бы одновременно медленным и хрупким.
    Вместо этого fake-провайдер сам двигает часы внутри recognize().
    """

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class _ClockAdvancingProvider:
    """Провайдер, который сам продвигает fake-часы на время своего вызова
    (имитирует долгий сетевой звонок без реального sleep)."""

    def __init__(self, name, response, clock, advance_by=0.0):
        self.name = name
        self._response = response
        self._clock = clock
        self._advance_by = advance_by
        self.calls = 0

    def recognize(self, audio_bytes):
        self.calls += 1
        self._clock.advance(self._advance_by)
        return self._response


class TestPhraseBudget:
    def test_default_budget_constant(self):
        # issue #2767: 20с — худший однопроходный сценарий с запасом.
        assert DEFAULT_MAX_TOTAL_BUDGET_S == 20.0

    def test_first_provider_always_tried_even_over_budget(self):
        """Бюджет не может оставить фразу вовсе без единой попытки."""
        clock = _FakeClock()
        primary = _ClockAdvancingProvider(
            "minimax", "привет робот", clock, advance_by=0.0
        )

        text, attempts = select_recognition(
            [primary],
            b"\x00\x00" * 800,
            max_total_s=0.0,  # бюджет исчерпан с самого начала
            clock=clock,
        )

        assert primary.calls == 1
        assert text == "привет робот"
        assert attempts[0].reason == "ok"

    def test_last_provider_always_tried_even_over_budget(self):
        """Живой сценарий issue #2767: minimax+yandex сожрали весь бюджет —
        Vosk (дешёвый офлайн-фолбэк, последний в цепочке) ВСЁ РАВНО
        получает шанс, иначе фраза осталась бы вовсе без текста."""
        clock = _FakeClock()
        # minimax «висит» 15с (имитация холодного cloud timeout).
        minimax = _ClockAdvancingProvider(
            "minimax", None, clock, advance_by=15.0
        )
        vosk = _ClockAdvancingProvider(
            "vosk", "расскажи ещё раз", clock, advance_by=1.5
        )

        text, attempts = select_recognition(
            [minimax, vosk],
            b"\x00\x00" * 800,
            max_total_s=10.0,  # уже меньше, чем потратил один minimax
            clock=clock,
            retry_backoff_s=0.0,
        )

        assert vosk.calls == 1  # последний провайдер — бюджет его не режет
        assert text == "расскажи ещё раз"
        assert attempts[-1].reason == "ok"

    def test_middle_provider_skipped_once_budget_exhausted(self):
        """3 провайдера: 1-й сжигает весь бюджет, 2-й (средний) пропускается
        с reason=budget_exceeded, 3-й (последний, Vosk) всё равно пробуется."""
        clock = _FakeClock()
        minimax = _ClockAdvancingProvider(
            "minimax", None, clock, advance_by=15.0
        )
        yandex = _ClockAdvancingProvider(
            "yandex", "не должно вызваться", clock, advance_by=12.0
        )
        vosk = _ClockAdvancingProvider(
            "vosk", "расскажи ещё раз", clock, advance_by=1.5
        )

        text, attempts = select_recognition(
            [minimax, yandex, vosk],
            b"\x00\x00" * 800,
            max_total_s=10.0,
            clock=clock,
            retry_backoff_s=0.0,
        )

        assert minimax.calls == 1
        assert yandex.calls == 0  # пропущен по бюджету
        assert vosk.calls == 1  # последний — не режем
        assert text == "расскажи ещё раз"

        budget_attempts = [
            a for a in attempts if a.reason == "budget_exceeded"
        ]
        assert len(budget_attempts) == 1
        assert budget_attempts[0].provider == "yandex"

    def test_budget_none_disables_cap(self):
        """Легаси-режим: max_total_s=None — бюджет никого не режет."""
        clock = _FakeClock()
        minimax = _ClockAdvancingProvider(
            "minimax", None, clock, advance_by=50.0
        )
        yandex = _ClockAdvancingProvider(
            "yandex", None, clock, advance_by=50.0
        )
        vosk = _ClockAdvancingProvider(
            "vosk", "расскажи ещё раз", clock, advance_by=1.0
        )

        text, attempts = select_recognition(
            [minimax, yandex, vosk],
            b"\x00\x00" * 800,
            max_total_s=None,
            clock=clock,
            retry_backoff_s=0.0,
        )

        assert minimax.calls == 1
        assert yandex.calls == 1  # НЕ пропущен, бюджет отключён
        assert vosk.calls == 1
        assert text == "расскажи ещё раз"
        assert all(a.reason != "budget_exceeded" for a in attempts)

    def test_budget_does_not_abort_running_call(self):
        """Бюджет НЕ прерывает уже начавшийся вызов — только решает,
        стоит ли НАЧИНАТЬ следующий (см. docstring _run_provider:
        gRPC/HTTP не отменяем)."""
        clock = _FakeClock()
        # Один провайдер, который сам «съедает» весь бюджет ВНУТРИ вызова —
        # вызов должен всё равно завершиться штатно (не оборваться).
        primary = _ClockAdvancingProvider(
            "minimax", "привет робот", clock, advance_by=100.0
        )

        text, attempts = select_recognition(
            [primary],
            b"\x00\x00" * 800,
            max_total_s=5.0,
            clock=clock,
        )

        assert text == "привет робот"
        assert attempts[0].reason == "ok"


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
        # Каждый раз: Yandex 1-я попытка — сетевой флап (транзиентный error,
        # ретраится), 2-я попытка ok. ``empty`` НЕ ретраится (issue #2767),
        # ``timeout`` тоже (issue #2924) — поэтому флап здесь error.
        primary = FakeProvider(
            "yandex",
            [None, phrase],
            exceptions=[RuntimeError("UNAVAILABLE: network flap"), None],
        )
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
        """Имитируем 10 фраз: 8 через Yandex (после retry сетевого флапа),
        2 — мусор."""
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
                # Совсем короткая — yandex пуст (без retry, issue #2767),
                # vosk мусор → None
                primary = FakeProvider("yandex", [None])
                fallback = FakeProvider("vosk", ["а"])
            else:
                # yandex: 1-я сетевой флап (error, ретраится), 2-я
                # фраза → ok
                primary = FakeProvider(
                    "yandex",
                    [None, ph],
                    exceptions=[RuntimeError("UNAVAILABLE: network flap"), None],
                )
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


class _PreparingProvider(FakeProvider):
    """Fallback с медленной подготовкой (ленивый Vosk, issue #2609)."""

    def __init__(self, *a, prepare_delay: float = 0.0, prepare_exc=None, **kw):
        super().__init__(*a, **kw)
        self.prepare_calls = 0
        self._prepare_delay = prepare_delay
        self._prepare_exc = prepare_exc

    def prepare(self) -> None:
        self.prepare_calls += 1
        time.sleep(self._prepare_delay)
        if self._prepare_exc is not None:
            raise self._prepare_exc


class TestProviderPrepare:
    def test_prepare_time_not_counted_in_timeout(self):
        primary = FakeProvider("yandex", [None], exceptions=[RuntimeError("PERMISSION_DENIED")])
        vosk = _PreparingProvider("vosk", ["привет робот"], prepare_delay=0.3)
        text, attempts = select_recognition(
            [primary, vosk], b"\x00", timeout_s=0.2, max_retries=0, retry_backoff_s=0
        )
        assert text == "привет робот"
        assert attempts[-1].provider == "vosk"
        assert attempts[-1].reason == "ok"

    def test_prepare_not_called_when_primary_succeeds(self):
        primary = FakeProvider("yandex", ["привет робот"])
        vosk = _PreparingProvider("vosk", [])
        text, _ = select_recognition([primary, vosk], b"\x00", max_retries=0)
        assert text == "привет робот"
        assert vosk.prepare_calls == 0

    def test_prepare_failure_still_calls_recognize(self):
        primary = FakeProvider("yandex", [None])
        vosk = _PreparingProvider("vosk", [None], prepare_exc=RuntimeError("broken model"))
        text, attempts = select_recognition([primary, vosk], b"\x00", max_retries=0, retry_backoff_s=0)
        assert text is None
        assert vosk.prepare_calls == 1
        assert [a.provider for a in attempts] == ["yandex", "vosk"]
