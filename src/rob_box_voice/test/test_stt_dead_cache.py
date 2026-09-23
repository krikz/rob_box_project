#!/usr/bin/env python3
"""test_stt_dead_cache.py — кэш «мёртвых» STT-провайдеров и per-provider бюджет.

Issue #2365 Phase 2 (ADR-0124). Тестируем чистую логику из
``rob_box_voice.stt_fallback`` — без rclpy, vosk и сети, как и весь
модуль (он stdlib-only именно ради этого).

Что покрываем:

* :class:`ProviderDeadCache` — TTL, длинный vs короткий, снятие отметки,
  round-trip персистентности в wall-clock.
* :class:`ProviderPolicy` — у каждого провайдера свой таймаут и число
  повторов (до Phase 2 бюджет был один на всю цепочку).
* :func:`select_recognition` с кэшем — пропуск лежащих облаков,
  ``reason="dead"`` в логе попыток, деградация «все мёртвые → всё равно
  пробуем».
* Сценарий 21.09.2026: MiniMax и Yandex без денег (квота) → первая
  фраза платит за оба, вторая уходит в Vosk сразу.
"""

from __future__ import annotations

import time

import pytest

from rob_box_voice.stt_fallback import (
    DEFAULT_DEAD_TTL_PERMANENT_LADDER,
    DEFAULT_DEAD_TTL_S,
    DEFAULT_DEAD_TTL_TRANSIENT_S,
    ProviderDeadCache,
    ProviderPolicy,
    STTAuthError,
    STTQuotaError,
    STTTimeoutError,
    is_permanent_failure,
    select_recognition,
    summarize_attempts,
)


class _FakeClock:
    """Ручные монотонные часы — TTL тестируем без sleep."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class _StubProvider:
    """Провайдер с заранее заданным поведением на каждый вызов."""

    def __init__(self, name: str, results):
        self.name = name
        self._results = list(results)
        self.calls = 0

    def recognize(self, audio_bytes: bytes):
        self.calls += 1
        result = self._results[min(self.calls - 1, len(self._results) - 1)]
        if isinstance(result, BaseException):
            raise result
        return result


AUDIO = b"\x00\x00" * 8000  # 1 секунда тишины 16 кГц


# ---------------------------------------------------------------------------
# ProviderDeadCache
# ---------------------------------------------------------------------------


class TestProviderDeadCache:
    def test_fresh_cache_has_nobody_dead(self):
        cache = ProviderDeadCache()
        assert cache.is_dead("minimax") is False
        assert cache.remaining_s("minimax") == 0.0
        assert cache.reason("minimax") is None

    def test_transient_uses_short_ttl(self):
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)

        ttl = cache.mark_dead("yandex", "connection reset", transient=True)

        assert ttl == DEFAULT_DEAD_TTL_TRANSIENT_S
        assert cache.is_dead("yandex") is True
        clock.advance(DEFAULT_DEAD_TTL_TRANSIENT_S - 1)
        assert cache.is_dead("yandex") is True
        clock.advance(2)
        assert cache.is_dead("yandex") is False

    def test_permanent_uses_long_ttl(self):
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)

        ttl = cache.mark_dead("minimax", "quota (2056)", transient=False)

        assert ttl == DEFAULT_DEAD_TTL_S
        clock.advance(DEFAULT_DEAD_TTL_TRANSIENT_S + 1)
        # Короткий TTL уже истёк бы — длинный ещё держит.
        assert cache.is_dead("minimax") is True
        clock.advance(DEFAULT_DEAD_TTL_S)
        assert cache.is_dead("minimax") is False

    def test_mark_alive_clears_entry(self):
        cache = ProviderDeadCache()
        cache.mark_dead("minimax", "quota", transient=False)
        assert cache.is_dead("minimax") is True

        cache.mark_alive("minimax")

        assert cache.is_dead("minimax") is False
        assert cache.reason("minimax") is None

    def test_reason_is_kept_for_operator(self):
        cache = ProviderDeadCache()
        cache.mark_dead("minimax", "HTTP 429 rate-limited", transient=False)
        assert "429" in (cache.reason("minimax") or "")

    def test_reason_is_truncated(self):
        cache = ProviderDeadCache()
        cache.mark_dead("yandex", "x" * 1000, transient=True)
        assert len(cache.reason("yandex") or "") == 300

    def test_explicit_ttl_overrides_classification(self):
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)
        assert cache.mark_dead("yandex", "manual", ttl_s=7.0) == 7.0
        clock.advance(8)
        assert cache.is_dead("yandex") is False


class TestPermanentFailureEscalation:
    """Issue #2767 — лестница эскалации TTL для повторных permanent-отказов.

    Живой инцидент 23.09: Yandex STT падает с PERMISSION_DENIED (нет прав
    на папку в IAM) КАЖДЫЙ раз — 300с плоского TTL не «лечит» проблему с
    правами, нода просто простукивает мёртвого провайдера заново каждые
    5 минут бесконечно. Эскалация (аналог tts_node._transient_ttl_for_streak
    из PR #2712/#2721, но по другой оси — здесь эскалируем PERMANENT, а не
    транзиентные отказы) снижает частоту повторных проверок.
    """

    def test_first_permanent_failure_uses_base_ttl(self):
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)

        ttl = cache.mark_dead("yandex", "PERMISSION_DENIED", transient=False)

        assert ttl == DEFAULT_DEAD_TTL_S

    def test_second_consecutive_permanent_failure_escalates(self):
        """TTL истёк, попробовали снова, снова PERMISSION_DENIED —
        2-й подряд отказ получает удлинённый TTL (x3 base), а не те же
        300с по кругу."""
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)

        ttl1 = cache.mark_dead(
            "yandex", "PERMISSION_DENIED", transient=False
        )
        clock.advance(ttl1 + 1)
        assert cache.is_dead("yandex") is False  # TTL истёк, воскрес

        ttl2 = cache.mark_dead(
            "yandex", "PERMISSION_DENIED", transient=False
        )

        assert ttl1 == DEFAULT_DEAD_TTL_S
        assert ttl2 == (
            DEFAULT_DEAD_TTL_S * DEFAULT_DEAD_TTL_PERMANENT_LADDER[1]
        )
        assert ttl2 > ttl1

    def test_third_and_further_permanent_failures_cap_at_top_rung(self):
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)
        top_rung = DEFAULT_DEAD_TTL_S * DEFAULT_DEAD_TTL_PERMANENT_LADDER[-1]

        for _ in range(5):
            ttl = cache.mark_dead(
                "yandex", "PERMISSION_DENIED", transient=False
            )
            clock.advance(ttl + 1)

        # Последний (5-й подряд) отказ уже на верхней ступени лестницы.
        assert ttl == top_rung

    def test_success_resets_permanent_escalation_streak(self):
        """Баланс/права починили — следующий отказ снова с базового TTL,
        а не продолжает эскалацию с прошлой серии."""
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)

        ttl1 = cache.mark_dead("yandex", "PERMISSION_DENIED", transient=False)
        clock.advance(ttl1 + 1)
        ttl2 = cache.mark_dead("yandex", "PERMISSION_DENIED", transient=False)
        assert ttl2 > ttl1  # эскалация подтверждена

        cache.mark_alive("yandex")  # права починили

        ttl3 = cache.mark_dead("yandex", "PERMISSION_DENIED", transient=False)
        assert ttl3 == DEFAULT_DEAD_TTL_S  # сброшено к базовому

    def test_explicit_ttl_bypasses_escalation(self):
        """Ручной override (диагностика/тест) не трогает streak-счётчик."""
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)

        cache.mark_dead("yandex", "manual", transient=False, ttl_s=1.0)
        clock.advance(2)
        ttl = cache.mark_dead("yandex", "PERMISSION_DENIED", transient=False)

        # explicit override не считался streak'ом
        assert ttl == DEFAULT_DEAD_TTL_S

    def test_escalation_is_per_provider(self):
        """Эскалация MiniMax не влияет на Yandex — независимые счётчики."""
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)

        ttl_yandex_1 = cache.mark_dead(
            "yandex", "PERMISSION_DENIED", transient=False
        )
        clock.advance(ttl_yandex_1 + 1)
        cache.mark_dead("yandex", "PERMISSION_DENIED", transient=False)

        ttl_minimax_1 = cache.mark_dead("minimax", "quota", transient=False)

        assert ttl_minimax_1 == DEFAULT_DEAD_TTL_S  # 1-й отказ minimax — база


class TestDeadCachePersistence:
    """Рестарт ноды не должен снова платить таймаут мёртвому облаку (#2676)."""

    def test_snapshot_restore_round_trip(self):
        clock_a = _FakeClock()
        source = ProviderDeadCache(clock=clock_a)
        source.mark_dead("minimax", "quota", transient=False)

        now_wall = 1_700_000_000.0
        snapshot = source.snapshot_wall(now_wall=now_wall)
        assert snapshot["minimax"] == pytest.approx(now_wall + DEFAULT_DEAD_TTL_S)

        clock_b = _FakeClock(start=5.0)  # другой процесс — другие monotonic
        restored = ProviderDeadCache(clock=clock_b)
        names = restored.restore_wall(snapshot, now_wall=now_wall + 10)

        assert names == ["minimax"]
        assert restored.is_dead("minimax") is True
        # Потраченные 10 секунд списались с остатка TTL.
        assert restored.remaining_s("minimax") == pytest.approx(
            DEFAULT_DEAD_TTL_S - 10
        )

    def test_expired_entries_are_dropped_on_restore(self):
        cache = ProviderDeadCache()
        names = cache.restore_wall({"minimax": 1_000.0}, now_wall=2_000.0)
        assert names == []
        assert cache.is_dead("minimax") is False

    def test_garbage_entries_are_ignored(self):
        cache = ProviderDeadCache()
        names = cache.restore_wall(
            {"minimax": "не число", "yandex": None}, now_wall=time.time()
        )
        assert names == []

    def test_snapshot_skips_expired(self):
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)
        cache.mark_dead("yandex", "net", transient=True)
        clock.advance(DEFAULT_DEAD_TTL_TRANSIENT_S + 1)
        assert cache.snapshot_wall() == {}


class TestPermanentFailureClassification:
    @pytest.mark.parametrize(
        "exc,expected",
        [
            (STTAuthError("401"), True),
            (STTQuotaError("429"), True),
            (STTTimeoutError("deadline"), False),
            (RuntimeError("boom"), False),
            (None, False),
        ],
    )
    def test_is_permanent_failure(self, exc, expected):
        assert is_permanent_failure(exc) is expected


# ---------------------------------------------------------------------------
# ProviderPolicy
# ---------------------------------------------------------------------------


class TestProviderPolicy:
    def test_policy_gives_each_provider_its_own_retries(self):
        """MiniMax с 1 retry, Yandex без retry — раньше так было нельзя.

        Первый ответ MiniMax — реальный транзиентный сбой (не ``empty``,
        issue #2767: ``empty`` больше не ретраится вовсе), иначе retry
        не сработал бы и тест перестал бы проверять то, что заявлено.
        """
        minimax = _StubProvider("minimax", [STTTimeoutError("deadline"), None])
        yandex = _StubProvider("yandex", [None])
        vosk = _StubProvider("vosk", ["расскажи анекдот"])

        text, attempts = select_recognition(
            [minimax, yandex, vosk],
            AUDIO,
            retry_backoff_s=0.0,
            policies={
                "minimax": ProviderPolicy(timeout_s=5.0, max_retries=1),
                "yandex": ProviderPolicy(timeout_s=12.0, max_retries=0),
                "vosk": ProviderPolicy(timeout_s=10.0, max_retries=0),
            },
        )

        assert text == "расскажи анекдот"
        assert minimax.calls == 2  # 1 попытка + 1 retry
        assert yandex.calls == 1  # без retry
        assert vosk.calls == 1

    def test_missing_policy_falls_back_to_legacy_rule(self):
        """Провайдер без явной политики живёт по старому правилу.

        Первый ответ — транзиентный сбой (не ``empty``, issue #2767),
        иначе retry не сработал бы вовсе.
        """
        first = _StubProvider("minimax", [STTTimeoutError("deadline"), None])
        second = _StubProvider("vosk", ["длинная фраза"])

        text, _ = select_recognition(
            [first, second],
            AUDIO,
            max_retries=1,
            retry_backoff_s=0.0,
            policies={"vosk": ProviderPolicy(timeout_s=10.0, max_retries=0)},
        )

        assert text == "длинная фраза"
        assert first.calls == 2  # legacy: primary получает retry

    def test_policy_timeout_is_per_provider(self):
        """Медленный ответ MiniMax ловится его собственным таймаутом (5с),
        а не общим яндексовым (12с)."""

        class _Slow:
            name = "minimax"

            def recognize(self, audio_bytes):
                time.sleep(0.05)
                return "нормальная фраза"

        _, attempts = select_recognition(
            [_Slow()],
            AUDIO,
            timeout_s=12.0,
            policies={"minimax": ProviderPolicy(timeout_s=0.01, max_retries=0)},
        )

        assert attempts[0].reason == "timeout"


# ---------------------------------------------------------------------------
# select_recognition + dead cache
# ---------------------------------------------------------------------------


class TestSelectRecognitionWithDeadCache:
    def test_failure_marks_provider_dead(self):
        minimax = _StubProvider("minimax", [STTQuotaError("limit reached (2056)")])
        vosk = _StubProvider("vosk", ["расскажи анекдот"])
        cache = ProviderDeadCache()

        select_recognition(
            [minimax, vosk], AUDIO, retry_backoff_s=0.0, dead_cache=cache
        )

        assert cache.is_dead("minimax") is True
        assert cache.remaining_s("minimax") == pytest.approx(
            DEFAULT_DEAD_TTL_S, abs=1.0
        )

    def test_transient_failure_gets_short_ttl(self):
        yandex = _StubProvider("yandex", [STTTimeoutError("deadline")])
        vosk = _StubProvider("vosk", ["расскажи анекдот"])
        cache = ProviderDeadCache()

        select_recognition(
            [yandex, vosk], AUDIO, retry_backoff_s=0.0, dead_cache=cache
        )

        assert cache.remaining_s("yandex") == pytest.approx(
            DEFAULT_DEAD_TTL_TRANSIENT_S, abs=1.0
        )

    def test_quota_error_skips_retry(self):
        """Квота не «рассосётся» между попытками — второй заход не делаем."""
        minimax = _StubProvider("minimax", [STTQuotaError("2056")])
        vosk = _StubProvider("vosk", ["расскажи анекдот"])

        select_recognition(
            [minimax, vosk],
            AUDIO,
            retry_backoff_s=0.0,
            policies={"minimax": ProviderPolicy(timeout_s=5.0, max_retries=3)},
            dead_cache=ProviderDeadCache(),
        )

        assert minimax.calls == 1

    def test_dead_provider_is_skipped_next_call(self):
        minimax = _StubProvider("minimax", [STTQuotaError("2056")])
        vosk = _StubProvider("vosk", ["расскажи анекдот"])
        cache = ProviderDeadCache()

        select_recognition(
            [minimax, vosk], AUDIO, retry_backoff_s=0.0, dead_cache=cache
        )
        calls_after_first = minimax.calls

        text, attempts = select_recognition(
            [minimax, vosk], AUDIO, retry_backoff_s=0.0, dead_cache=cache
        )

        assert minimax.calls == calls_after_first  # в облако больше не ходили
        assert text == "расскажи анекдот"
        assert attempts[0].provider == "minimax"
        assert attempts[0].reason == "dead"
        assert attempts[0].latency_ms == 0

    def test_dead_skip_is_visible_in_operator_log(self):
        cache = ProviderDeadCache()
        cache.mark_dead("minimax", "HTTP 429", transient=False)
        vosk = _StubProvider("vosk", ["расскажи анекдот"])

        _, attempts = select_recognition(
            [_StubProvider("minimax", [None]), vosk],
            AUDIO,
            retry_backoff_s=0.0,
            dead_cache=cache,
        )

        summary = summarize_attempts(attempts)
        assert summary.startswith("minimax:dead")
        assert "vosk:ok" in summary

    def test_success_revives_provider(self):
        """Баланс пополнили — первый же успешный ответ снимает отметку."""
        cache = ProviderDeadCache()
        cache.mark_dead("yandex", "quota", transient=True)
        # Только yandex в цепочке → кэш игнорируется (все мёртвые).
        yandex = _StubProvider("yandex", ["робот расскажи анекдот"])

        text, _ = select_recognition(
            [yandex], AUDIO, retry_backoff_s=0.0, dead_cache=cache
        )

        assert text == "робот расскажи анекдот"
        assert cache.is_dead("yandex") is False

    def test_all_dead_still_tries_everyone(self):
        """Глухой робот хуже медленного: если мертвы все — идём по цепочке."""
        cache = ProviderDeadCache()
        cache.mark_dead("minimax", "quota", transient=False)
        cache.mark_dead("vosk", "странно, но пусть", transient=False)
        minimax = _StubProvider("minimax", [None])
        vosk = _StubProvider("vosk", ["расскажи анекдот"])

        text, attempts = select_recognition(
            [minimax, vosk],
            AUDIO,
            max_retries=0,
            retry_backoff_s=0.0,
            dead_cache=cache,
        )

        assert text == "расскажи анекдот"
        assert vosk.calls == 1
        assert [a.reason for a in attempts] == ["empty", "ok"]

    def test_empty_result_does_not_mark_dead(self):
        """Тишина — не отказ провайдера: облако ответило, просто нечего сказать."""
        minimax = _StubProvider("minimax", [""])
        vosk = _StubProvider("vosk", ["расскажи анекдот"])
        cache = ProviderDeadCache()

        select_recognition(
            [minimax, vosk], AUDIO, retry_backoff_s=0.0, dead_cache=cache
        )

        assert cache.is_dead("minimax") is False

    def test_low_confidence_does_not_mark_dead(self):
        minimax = _StubProvider("minimax", ["а"])
        vosk = _StubProvider("vosk", ["расскажи анекдот"])
        cache = ProviderDeadCache()

        select_recognition(
            [minimax, vosk], AUDIO, retry_backoff_s=0.0, dead_cache=cache
        )

        assert cache.is_dead("minimax") is False


class TestBothCloudsOutOfCredit:
    """Сценарий 21.09.2026 — на счетах MiniMax и Yandex кончились деньги."""

    def _chain(self):
        return (
            _StubProvider("minimax", [STTQuotaError("limit reached (2056)")]),
            _StubProvider("yandex", [STTQuotaError("RESOURCE_EXHAUSTED")]),
            _StubProvider("vosk", ["робот расскажи анекдот"]),
        )

    def test_first_phrase_pays_for_both_clouds_then_lands_on_vosk(self):
        minimax, yandex, vosk = self._chain()
        cache = ProviderDeadCache()

        text, attempts = select_recognition(
            [minimax, yandex, vosk],
            AUDIO,
            retry_backoff_s=0.0,
            dead_cache=cache,
        )

        assert text == "робот расскажи анекдот"
        assert [a.provider for a in attempts] == ["minimax", "yandex", "vosk"]
        assert [a.reason for a in attempts] == ["error", "error", "ok"]

    def test_second_phrase_goes_straight_to_vosk(self):
        minimax, yandex, vosk = self._chain()
        cache = ProviderDeadCache()
        chain = [minimax, yandex, vosk]

        select_recognition(chain, AUDIO, retry_backoff_s=0.0, dead_cache=cache)
        select_recognition(chain, AUDIO, retry_backoff_s=0.0, dead_cache=cache)
        text, attempts = select_recognition(
            chain, AUDIO, retry_backoff_s=0.0, dead_cache=cache
        )

        # За три фразы в каждое облако сходили РОВНО один раз.
        assert minimax.calls == 1
        assert yandex.calls == 1
        assert vosk.calls == 3
        assert text == "робот расскажи анекдот"
        assert [a.reason for a in attempts] == ["dead", "dead", "ok"]

    def test_clouds_are_retried_after_ttl(self):
        """Баланс пополнили — по истечении TTL снова пробуем облако."""
        clock = _FakeClock()
        cache = ProviderDeadCache(clock=clock)
        minimax = _StubProvider(
            "minimax", [STTQuotaError("2056"), "робот расскажи анекдот"]
        )
        vosk = _StubProvider("vosk", ["мусор мусор"])
        chain = [minimax, vosk]

        select_recognition(chain, AUDIO, retry_backoff_s=0.0, dead_cache=cache)
        assert minimax.calls == 1

        clock.advance(DEFAULT_DEAD_TTL_S + 1)
        text, _ = select_recognition(
            chain, AUDIO, retry_backoff_s=0.0, dead_cache=cache
        )

        assert minimax.calls == 2
        assert text == "робот расскажи анекдот"
        assert cache.is_dead("minimax") is False
