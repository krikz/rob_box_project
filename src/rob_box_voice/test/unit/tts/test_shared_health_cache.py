"""Unit tests for issue #2702 — «при мёртвых облаках ответ идёт 1-2 мин».

Прод-лог 17.09.2026 (voice-assistant): пока у MiniMax кончился Token Plan
(quota) и Yandex отдаёт ``PERMISSION_DENIED`` (ADR-0124, до ≈21.09), TTS
честно ждал СВОЙ таймаут на каждой фразе, хотя LLM-сторона уже знала про
дохлый MiniMax (общий ``rob_box_harness.health.HealthCache``), а Yandex
auth-отказ помечался транзиентным 30с вместо длинного TTL.

Покрываем (acceptance-чеклист issue #2702):

* ``TTSNode._provider_is_dead`` дополнительно читает ОБЩИЙ health-кэш для
  ``minimax`` (Token Plan общий для LLM и T2A) — HTTP-запрос не делается,
  если LLM-сторона уже пометила провайдер unavailable;
* ``TTSNode._mark_provider_dead`` зеркалит падение MiniMax в общий кэш
  (запись — обратное направление);
* Yandex ``PERMISSION_DENIED``/``UNAUTHENTICATED`` → длинный TTL
  (``provider_dead_ttl_s``), а не транзиентные 30с;
* эскалация TTL транзиентных отказов ПОДРЯД: 30 → 120 → 300с, сброс на
  первом успехе (и на классификации отказа как auth/quota);
* ``cloud_tts_budget_s`` — суммарный бюджет облачной части цепочки:
  исчерпан → сразу Silero, без дальнейших retry/переходов;
* мягкая деградация без ``rob_box_harness`` (тот же паттерн, что уже есть
  в tts_node для отсутствующего ``rob_box_llm``/MiniMax).

Стиль и стабы — как в ``test_provider_chain.py`` (issue #1083) и
``test_stt_dead_cache.py`` (issue #2365 Phase 2): чистые bare-стабы вместо
``TTSNode()`` (rclpy не поднимаем), реальный ``HealthCache`` из
``rob_box_harness.health`` вместо мока (это ровно тот класс, который делят
dialogue_node/supervisor — если его контракт изменится, эти тесты должны
это заметить).
"""
from __future__ import annotations

import asyncio
import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_harness.health import HealthCache  # noqa: E402
from rob_box_llm.errors import TTSTimeoutError  # noqa: E402
from rob_box_voice import tts_node  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402

MiniMaxTTSAuthError = tts_node.MiniMaxTTSAuthError


class _FakeClock:
    """Ручные монотонные часы — TTL тестируем без sleep (как в test_stt_dead_cache.py)."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class _Bare:
    """Minimal node stub: attributes only, no methods (contract for stubs)."""


def _bare_node() -> _Bare:
    return _Bare()


def _bind_dead_cache(node: _Bare) -> _Bare:
    """Полный набор dead-cache методов, включая issue #2702 (эскалация +
    общий кэш). Отдельная от ``test_provider_chain._bind_dead_cache`` копия
    — та НЕ знает про новые методы, и намеренно: старые тесты остаются
    регрессией для «а что если новые методы не забиндены» (getattr-fallback
    в проде), а эти — регрессией для «а что если забиндены».
    """
    node.provider_dead_ttl_s = 300.0
    node.provider_dead_ttl_transient_s = 30.0
    node._provider_dead_until = {}
    node._provider_dead_reason = {}
    node._provider_transient_streak = {}
    node._shared_health_cache = None
    node.get_logger = MagicMock()
    node._provider_is_dead = TTSNode._provider_is_dead.__get__(node, type(node))  # type: ignore[attr-defined]
    node._mark_provider_dead = TTSNode._mark_provider_dead.__get__(node, type(node))  # type: ignore[attr-defined]
    node._provider_dead_until_s = TTSNode._provider_dead_until_s.__get__(node, type(node))  # type: ignore[attr-defined]
    node._transient_ttl_for_streak = TTSNode._transient_ttl_for_streak.__get__(node, type(node))  # type: ignore[attr-defined]
    node._reset_provider_fail_streak = TTSNode._reset_provider_fail_streak.__get__(node, type(node))  # type: ignore[attr-defined]
    return node


def _playback_node() -> _Bare:
    """Node stub with everything ``_synthesize_and_play`` touches after synth.

    Копия ``test_provider_chain._playback_node`` (issue #1083) — держим
    отдельную, чтобы этот файл был самодостаточным (инструкция issue
    #2702: «тесты клади в src/rob_box_voice/test/, стиль бери с
    существующих tts-тестов», а не «импортируй их внутренности»).
    """
    node = _bare_node()
    node.provider = "minimax"
    node.normalize_text = False
    node.stop_requested = False
    node.processing_dialogue_id = None
    node.current_dialogue_id = None
    node.minimax_streaming = False
    node.minimax_model = "speech-02-hd"
    node.minimax_voice = "male-qn-qingse"
    node._synthesize_minimax = MagicMock(
        return_value={"audio_np": np.zeros(800, dtype=np.float32), "sample_rate": 32000}
    )
    node.publish_state = MagicMock()
    node._publish_audio = MagicMock()
    node.logger = MagicMock()
    node.get_logger = lambda: node.logger
    node.chipmunk_mode = False
    node.pitch_shift = 1.0
    node.volume_gain = 1.0
    node.device_index = None
    node.current_stream = None
    node.playback_manager = MagicMock()
    node.playback_manager.play_audio.return_value = True
    node.cleanup_playback_noise = MagicMock()
    node.finished_pub = MagicMock()
    node.audio_output_sample_rate = 16000
    node._prepare_audio_for_topic = TTSNode._prepare_audio_for_topic.__get__(node, type(node))  # type: ignore[attr-defined]

    node._silero_loaded = threading.Event()
    node._silero_loaded.set()
    node.silero_model = MagicMock(name="FakeSileroModel")
    node.silero_sample_rate = 48000
    node.silero_put_stress_homo = True
    node._synthesize_silero = MagicMock(return_value=np.zeros(4800, dtype=np.float32))
    return node


def _run_and_play(node) -> None:
    TTSNode._synthesize_and_play(
        node,
        "<speak>hello</speak>",
        "hello",
        None,
        {},
        None,
    )


# ---------------------------------------------------------------------------
# Общий health-кэш — чтение (issue #2702 п.1, acceptance test #1)
# ---------------------------------------------------------------------------


class TestSharedHealthCacheRead:
    def test_minimax_unavailable_in_shared_cache_is_seen_without_local_mark(self) -> None:
        """LLM-сторона узнала про quota первой — TTS видит то же самое,
        БЕЗ своего собственного отказа/таймаута."""
        node = _bind_dead_cache(_bare_node())
        shared = HealthCache()
        shared.mark_unavailable("minimax", reason="LLM-side quota (2056)", ttl_s=300.0)
        node._shared_health_cache = shared

        assert TTSNode._provider_is_dead(node, "minimax") is True
        # Локальный кэш никто не трогал — правда пришла только из общего.
        assert node._provider_dead_until == {}

    def test_yandex_key_in_shared_cache_is_ignored(self) -> None:
        """Общий кэш читается ТОЛЬКО для minimax (issue #2702 scope) — у
        Yandex своя, не общая с LLM, квота (аудио, а не Token Plan)."""
        node = _bind_dead_cache(_bare_node())
        shared = HealthCache()
        shared.mark_unavailable("yandex", reason="unrelated", ttl_s=300.0)
        node._shared_health_cache = shared

        assert TTSNode._provider_is_dead(node, "yandex") is False

    def test_expired_shared_entry_does_not_block(self) -> None:
        clock = _FakeClock()
        node = _bind_dead_cache(_bare_node())
        shared = HealthCache(clock=clock)
        shared.mark_unavailable("minimax", reason="quota", ttl_s=10.0)
        node._shared_health_cache = shared
        clock.advance(11.0)

        assert TTSNode._provider_is_dead(node, "minimax") is False

    def test_broken_shared_cache_does_not_crash(self) -> None:
        """Общий кэш — best-effort: сломанный объект не должен ронять TTS."""
        node = _bind_dead_cache(_bare_node())
        broken = MagicMock()
        broken.is_unavailable.side_effect = RuntimeError("disk full")
        node._shared_health_cache = broken

        assert TTSNode._provider_is_dead(node, "minimax") is False

    def test_local_dead_cache_still_works_without_shared_cache(self) -> None:
        """Регрессия: узел без общего кэша (harness недоступен) ведёт себя
        ровно как до issue #2702."""
        node = _bind_dead_cache(_bare_node())
        node._provider_dead_until["minimax"] = time.monotonic() + 60.0

        assert TTSNode._provider_is_dead(node, "minimax") is True


class TestSharedHealthCacheChainIntegration:
    """Acceptance test #1 — сквозной прогон через ``_synthesize_and_play``."""

    def test_minimax_unavailable_in_shared_cache_skips_http_call(self) -> None:
        node = _playback_node()
        _bind_dead_cache(node)
        shared = HealthCache()
        shared.mark_unavailable("minimax", reason="LLM-side quota (2056)", ttl_s=300.0)
        node._shared_health_cache = shared
        node.yandex_stub = object()
        node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
        # Canary: если регрессия — HTTP-запрос к MiniMax падает громко,
        # а не тихо тратит 30-90с на честный таймаут.
        node._synthesize_minimax = MagicMock(
            side_effect=AssertionError(
                "minimax MUST NOT be called when the SHARED health cache "
                "already says unavailable (#2702)"
            )
        )

        _run_and_play(node)

        node._synthesize_minimax.assert_not_called()
        node._synthesize_yandex.assert_called_once()
        node._synthesize_silero.assert_not_called()
        node._publish_audio.assert_called_once()

    def test_minimax_local_failure_mirrors_into_shared_cache(self) -> None:
        """Issue #2702 п.1 (запись, обратное направление) — падение MiniMax
        здесь зеркалится в общий кэш, чтобы LLM-сторона тоже узнала без
        своего отдельного честного таймаута."""
        node = _playback_node()
        _bind_dead_cache(node)
        shared = HealthCache()
        node._shared_health_cache = shared
        node.yandex_stub = object()
        node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
        node._synthesize_minimax = MagicMock(
            side_effect=MiniMaxTTSAuthError(
                "2056 Token Plan usage limit reached", provider="minimax"
            )
        )

        _run_and_play(node)

        assert shared.is_unavailable("minimax") is True


# ---------------------------------------------------------------------------
# Yandex PERMISSION_DENIED/UNAUTHENTICATED → длинный TTL (issue #2702 п.2)
# ---------------------------------------------------------------------------


class TestYandexAuthFailureClassification:
    def test_permission_denied_gets_long_ttl(self) -> None:
        node = _bind_dead_cache(_bare_node())
        TTSNode._mark_provider_dead(
            node,
            "yandex",
            Exception(
                "Yandex gRPC error: StatusCode.PERMISSION_DENIED - "
                "account has no role on the folder"
            ),
        )
        assert node._provider_dead_until["yandex"] > time.monotonic() + 250

    def test_unauthenticated_gets_long_ttl(self) -> None:
        node = _bind_dead_cache(_bare_node())
        TTSNode._mark_provider_dead(
            node,
            "yandex",
            Exception("Yandex gRPC error: StatusCode.UNAUTHENTICATED - bad IAM token"),
        )
        assert node._provider_dead_until["yandex"] > time.monotonic() + 250

    def test_transient_yandex_error_still_gets_short_ttl(self) -> None:
        """Не любой gRPC-код Yandex — auth; UNAVAILABLE остаётся транзиентным."""
        node = _bind_dead_cache(_bare_node())
        TTSNode._mark_provider_dead(
            node,
            "yandex",
            Exception("Yandex gRPC error: StatusCode.UNAVAILABLE - connection reset"),
        )
        remaining = node._provider_dead_until["yandex"] - time.monotonic()
        assert 25.0 < remaining <= 30.0

    def test_minimax_permission_denied_text_does_not_trigger_yandex_rule(self) -> None:
        """``_yandex_is_permanent_failure`` игнорирует провайдеров, кроме
        yandex — MiniMax-классификация идёт только через isinstance."""
        node = _bind_dead_cache(_bare_node())
        TTSNode._mark_provider_dead(
            node, "minimax", Exception("some PERMISSION_DENIED-like text")
        )
        remaining = node._provider_dead_until["minimax"] - time.monotonic()
        assert 25.0 < remaining <= 30.0  # транзиентная лестура, не длинный TTL


class TestYandexPermanentFailureHelper:
    """Module-level classifier — тестируем напрямую (issue #2702 п.2)."""

    def test_non_yandex_provider_is_never_permanent(self) -> None:
        assert (
            tts_node._yandex_is_permanent_failure(
                "minimax", Exception("PERMISSION_DENIED")
            )
            is False
        )

    def test_permission_denied_text_is_permanent(self) -> None:
        assert (
            tts_node._yandex_is_permanent_failure(
                "yandex", Exception("StatusCode.PERMISSION_DENIED")
            )
            is True
        )

    def test_classifier_never_raises_on_odd_input(self) -> None:
        # __str__ шатается — классификатор не должен ронять _mark_provider_dead.
        class _Weird(Exception):
            def __str__(self) -> str:
                raise RuntimeError("boom")

        assert tts_node._yandex_is_permanent_failure("yandex", _Weird()) is False


# ---------------------------------------------------------------------------
# Эскалация TTL при повторяющихся транзиентных отказах (issue #2702 п.3)
# ---------------------------------------------------------------------------


class TestTransientTtlEscalation:
    def test_three_consecutive_timeouts_escalate_ttl(self) -> None:
        node = _bind_dead_cache(_bare_node())

        TTSNode._mark_provider_dead(node, "minimax", TimeoutError("net down"))
        ttl_1 = node._provider_dead_until["minimax"] - time.monotonic()

        TTSNode._mark_provider_dead(node, "minimax", TimeoutError("net down"))
        ttl_2 = node._provider_dead_until["minimax"] - time.monotonic()

        TTSNode._mark_provider_dead(node, "minimax", TimeoutError("net down"))
        ttl_3 = node._provider_dead_until["minimax"] - time.monotonic()

        assert ttl_1 == pytest.approx(30.0, abs=1.0)
        assert ttl_2 == pytest.approx(120.0, abs=1.0)
        assert ttl_3 == pytest.approx(300.0, abs=1.0)

    def test_streak_caps_at_long_ttl_for_further_failures(self) -> None:
        node = _bind_dead_cache(_bare_node())
        for _ in range(5):
            TTSNode._mark_provider_dead(node, "minimax", TimeoutError("net down"))
        remaining = node._provider_dead_until["minimax"] - time.monotonic()
        assert remaining == pytest.approx(300.0, abs=1.0)

    def test_streak_is_tracked_per_provider(self) -> None:
        node = _bind_dead_cache(_bare_node())
        TTSNode._mark_provider_dead(node, "minimax", TimeoutError("net down"))
        TTSNode._mark_provider_dead(node, "minimax", TimeoutError("net down"))
        # yandex ещё ни разу не падал транзиентно — своя лестница с нуля.
        TTSNode._mark_provider_dead(node, "yandex", Exception("network blip"))
        remaining_yandex = node._provider_dead_until["yandex"] - time.monotonic()
        assert remaining_yandex == pytest.approx(30.0, abs=1.0)

    def test_explicit_ttl_bypasses_escalation(self) -> None:
        """Явный ``ttl_s=`` (manual override) не трогает streak-счётчик."""
        node = _bind_dead_cache(_bare_node())
        TTSNode._mark_provider_dead(node, "minimax", RuntimeError("boom"), ttl_s=5.0)
        assert node._provider_transient_streak.get("minimax", 0) == 0

    def test_success_resets_the_streak(self) -> None:
        node = _playback_node()
        _bind_dead_cache(node)
        node.yandex_stub = object()
        node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
        node._synthesize_minimax = MagicMock(side_effect=TimeoutError("net down"))

        _run_and_play(node)  # 1st failure → streak=1, ttl≈30
        ttl_after_first = node._provider_dead_until["minimax"] - time.monotonic()
        assert ttl_after_first == pytest.approx(30.0, abs=1.0)

        # Провайдер «ожил» (TTL истёк вручную) и на этот раз отвечает успешно.
        node._provider_dead_until["minimax"] = time.monotonic() - 1.0
        node._synthesize_minimax = MagicMock(
            return_value={"audio_np": np.zeros(800, dtype=np.float32), "sample_rate": 32000}
        )
        _run_and_play(node)
        assert node._provider_transient_streak.get("minimax", 0) == 0

        # Следующий отказ снова начинает лестницу с 30с, а не с 120/300.
        node._provider_dead_until.pop("minimax", None)
        node._synthesize_minimax = MagicMock(side_effect=TimeoutError("net down again"))
        _run_and_play(node)
        ttl_after_reset = node._provider_dead_until["minimax"] - time.monotonic()
        assert ttl_after_reset == pytest.approx(30.0, abs=1.0)

    def test_quota_failure_resets_the_streak_too(self) -> None:
        """auth/quota — отдельный класс отказа, не наследует эскалацию
        транзиентных таймаутов (и не наследуется ею)."""
        node = _bind_dead_cache(_bare_node())
        TTSNode._mark_provider_dead(node, "minimax", TimeoutError("net down"))  # streak=1
        assert node._provider_transient_streak.get("minimax", 0) == 1

        TTSNode._mark_provider_dead(
            node,
            "minimax",
            MiniMaxTTSAuthError("2056 Token Plan usage limit reached", provider="minimax"),
        )
        assert node._provider_transient_streak.get("minimax", 0) == 0


# ---------------------------------------------------------------------------
# Облачный TTS-бюджет (issue #2702 п.4)
# ---------------------------------------------------------------------------


class TestCloudTtsBudgetChainLevel:
    def test_budget_exhausted_skips_remaining_cloud_providers(self) -> None:
        """Acceptance #4 — бюджет исчерпан → сразу Silero, Yandex не трогаем."""
        node = _playback_node()
        node.cloud_tts_budget_s = 0.02

        def _slow_minimax_failure(*args, **kwargs):
            time.sleep(0.15)  # съедает весь 20мс бюджет с большим запасом
            raise RuntimeError("MiniMax timeout (simulated)")

        node._synthesize_minimax = MagicMock(side_effect=_slow_minimax_failure)
        node.yandex_stub = object()
        node._synthesize_yandex = MagicMock(
            side_effect=AssertionError(
                "yandex MUST NOT be called once the cloud TTS budget is "
                "exhausted (#2702)"
            )
        )

        _run_and_play(node)

        node._synthesize_minimax.assert_called_once()
        node._synthesize_yandex.assert_not_called()
        node._synthesize_silero.assert_called_once()
        node._publish_audio.assert_called_once()

    def test_ample_budget_does_not_skip_yandex(self) -> None:
        """Sanity — обычный (не исчерпанный) бюджет ничего не меняет."""
        node = _playback_node()
        node.cloud_tts_budget_s = 8.0
        node.yandex_stub = object()
        node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
        node._synthesize_minimax = MagicMock(side_effect=RuntimeError("boom"))

        _run_and_play(node)

        node._synthesize_yandex.assert_called_once()
        node._synthesize_silero.assert_not_called()

    def test_default_budget_is_used_when_attribute_missing(self) -> None:
        """getattr-fallback (default 8.0s) — стаб без cloud_tts_budget_s не падает."""
        node = _playback_node()
        assert not hasattr(node, "cloud_tts_budget_s")
        node.yandex_stub = object()
        node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
        node._synthesize_minimax = MagicMock(side_effect=RuntimeError("boom"))

        _run_and_play(node)  # не должно кинуть AttributeError

        node._synthesize_yandex.assert_called_once()


class TestCloudTtsBudgetRetryLoopLevel:
    """Дедлайн внутри retry-loop MiniMax (``_synthesize_minimax_with_retry``).

    Прод-инцидент 17.09.2026: единственная причина 57с на одном MiniMax —
    3 честных попытки ПОДРЯД (30с + ~16с + ~11с). Между-провайдерная
    проверка (``TestCloudTtsBudgetChainLevel``) не спасает, если весь
    бюджет сжирает ОДИН провайдер своими retries — нужен дедлайн внутри
    самого retry-loop.
    """

    def _run(self, coro):
        return asyncio.run(coro)

    def test_budget_exhausted_before_first_retry_skips_remaining_attempts(self) -> None:
        """Первая попытка происходит безусловно (цепочка уже решила звать
        MiniMax) — но раз дедлайн уже в прошлом, retry ОТМЕНЯЮТСЯ."""
        call_count = [0]

        async def fake_async(text, ssml_attributes, voice=None, language=None):
            call_count[0] += 1
            raise TTSTimeoutError("net down")

        node = _Bare()
        node.minimax_max_retries = 2
        node.minimax_retry_backoff_ms = 1
        node.get_logger = MagicMock()
        node._synthesize_minimax_async = fake_async

        expired_deadline = time.monotonic() - 1.0
        with pytest.raises(TTSTimeoutError):
            self._run(
                TTSNode._synthesize_minimax_with_retry(
                    node, "hi", {}, budget_deadline=expired_deadline
                )
            )

        assert call_count[0] == 1  # ни одного retry — бюджет уже истёк

    def test_budget_not_exhausted_still_retries_normally(self) -> None:
        """Sanity — дедлайн в будущем не мешает обычным retry (regression
        guard: budget_deadline=None остаётся поведением по умолчанию для
        существующих вызовов ``_synthesize_minimax_with_retry``)."""
        call_count = [0]

        async def fake_async(text, ssml_attributes, voice=None, language=None):
            call_count[0] += 1
            if call_count[0] < 2:
                raise TTSTimeoutError("net down")
            return {"audio_np": np.zeros(8, dtype=np.float32), "sample_rate": 32000}

        node = _Bare()
        node.minimax_max_retries = 2
        node.minimax_retry_backoff_ms = 1
        node.get_logger = MagicMock()
        node._synthesize_minimax_async = fake_async

        far_future_deadline = time.monotonic() + 100.0
        result = self._run(
            TTSNode._synthesize_minimax_with_retry(
                node, "hi", {}, budget_deadline=far_future_deadline
            )
        )
        assert call_count[0] == 2
        assert result["sample_rate"] == 32000

    def test_none_deadline_is_unlimited_by_budget(self) -> None:
        """Regression guard — старые вызовы без ``budget_deadline=`` не
        ограничиваются бюджетом (только своим ``minimax_max_retries``)."""
        call_count = [0]

        async def fake_async(text, ssml_attributes, voice=None, language=None):
            call_count[0] += 1
            raise TTSTimeoutError("net down")

        node = _Bare()
        node.minimax_max_retries = 2
        node.minimax_retry_backoff_ms = 1
        node.get_logger = MagicMock()
        node._synthesize_minimax_async = fake_async

        with pytest.raises(TTSTimeoutError):
            self._run(TTSNode._synthesize_minimax_with_retry(node, "hi", {}))

        assert call_count[0] == 3  # 1 + 2 retries, как до issue #2702


# ---------------------------------------------------------------------------
# Мягкая деградация без rob_box_harness (issue #2702, "ImportError-friendly")
# ---------------------------------------------------------------------------


class TestBuildSharedHealthCache:
    def test_returns_real_cache_when_harness_available_and_path_set(self, tmp_path) -> None:
        path = tmp_path / "llm_health.json"
        cache = TTSNode._build_shared_health_cache(str(path), 300.0)
        assert cache is not None
        cache.mark_unavailable("minimax", reason="test", ttl_s=300.0)
        assert cache.is_unavailable("minimax") is True

    def test_empty_path_disables_shared_cache(self) -> None:
        assert TTSNode._build_shared_health_cache("", 300.0) is None

    def test_import_error_degrades_to_none(self, monkeypatch) -> None:
        """Узел без rob_box_harness (минимальная сборка) не должен падать —
        общий кэш просто отсутствует, тот же паттерн, что у MiniMax-
        провайдера при отсутствующем rob_box_llm (см. модульный docstring
        над ``HARNESS_HEALTH_AVAILABLE`` в tts_node.py)."""
        monkeypatch.setattr(tts_node, "HARNESS_HEALTH_AVAILABLE", False)
        assert (
            TTSNode._build_shared_health_cache("~/.rob_box/llm_health.json", 300.0)
            is None
        )

    def test_broken_cache_construction_logs_warning_and_returns_none(
        self, monkeypatch, tmp_path
    ) -> None:
        """Файл недоступен (права/диск, как у Yandex в ADR-0124) — не
        критично для TTS: warning в лог, локальный кэш продолжает работать."""

        class _ExplodingHealthCache:
            def __init__(self, *a, **kw):
                raise OSError("disk full")

        monkeypatch.setattr(tts_node, "_SharedHealthCache", _ExplodingHealthCache)
        logger = MagicMock()

        result = TTSNode._build_shared_health_cache(
            str(tmp_path / "llm_health.json"), 300.0, logger=logger
        )

        assert result is None
        logger.warn.assert_called_once()

    def test_harness_is_actually_importable_in_this_env(self) -> None:
        """Canary — если этот тест падает, вся деградация в файле выше
        тестирует не ту ветку (окружение изменилось, harness пропал)."""
        assert tts_node.HARNESS_HEALTH_AVAILABLE is True
