"""Unit-тесты для :class:`BacklogPressure` (issue #1668).

Чистый модуль: тесты используют подменённый ``clock`` для
детерминированной проверки sliding window / publish interval.

Что покрываем:

1. ``record`` инкрементирует счётчик и возвращает текущий уровень.
2. ``events_per_minute`` нормализуется по ``window_sec``.
3. ``level_from_rate`` корректно классифицирует LOW / ELEVATED / HIGH.
4. Sliding window: события старше ``window_sec`` удаляются из rate.
5. ``should_suppress_publish`` = True только при HIGH.
6. ``should_publish`` ограничивает частоту публикации (``publish_min_interval_sec``).
7. ``mark_published`` обновляет ``_last_publish``.
8. ``snapshot.to_json()`` — стабильная сериализация для ROS2 topic.
9. ``is_quiet`` — True только при очень низком давлении (≤1 событие / окно).
10. Validation: невалидные параметры → ``ValueError``.
11. Fairness gate при 100% заполнении backlog: HIGH подавляет, wake-word
    не теряется (подготовка к integration-тесту в test_wake_gate_fairness.py).
"""

from __future__ import annotations

import json

import pytest

from rob_box_voice.core.backlog_pressure import (
    DEFAULT_HIGH_ABOVE_PER_MIN,
    DEFAULT_LOW_ABOVE_PER_MIN,
    DEFAULT_PUBLISH_MIN_INTERVAL_SEC,
    DEFAULT_WINDOW_SEC,
    BacklogPressure,
    BacklogPressureLevel,
    level_from_rate,
)


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


class TestValidation:
    def test_window_sec_must_be_positive(self) -> None:
        with pytest.raises(ValueError, match="window_sec"):
            BacklogPressure(window_sec=0)
        with pytest.raises(ValueError, match="window_sec"):
            BacklogPressure(window_sec=-1.0)

    def test_thresholds_must_be_non_negative(self) -> None:
        with pytest.raises(ValueError, match="thresholds"):
            BacklogPressure(low_above=-1.0)
        with pytest.raises(ValueError, match="thresholds"):
            BacklogPressure(high_above=-1.0)

    def test_low_above_must_be_le_high_above(self) -> None:
        with pytest.raises(ValueError, match="low_above"):
            BacklogPressure(low_above=10.0, high_above=5.0)

    def test_publish_interval_must_be_non_negative(self) -> None:
        with pytest.raises(ValueError, match="publish_min_interval_sec"):
            BacklogPressure(publish_min_interval_sec=-0.5)

    def test_defaults_match_module_constants(self) -> None:
        bp = BacklogPressure()
        assert bp.window_sec == DEFAULT_WINDOW_SEC
        assert bp.low_above == DEFAULT_LOW_ABOVE_PER_MIN
        assert bp.high_above == DEFAULT_HIGH_ABOVE_PER_MIN
        assert bp.publish_min_interval_sec == DEFAULT_PUBLISH_MIN_INTERVAL_SEC


# ---------------------------------------------------------------------------
# level_from_rate — чистая функция
# ---------------------------------------------------------------------------


class TestLevelFromRate:
    def test_below_low_is_low(self) -> None:
        assert (
            level_from_rate(0.0, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.LOW
        )
        assert (
            level_from_rate(4.99, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.LOW
        )

    def test_between_is_elevated(self) -> None:
        assert (
            level_from_rate(5.0, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.ELEVATED
        )
        assert (
            level_from_rate(7.5, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.ELEVATED
        )
        assert (
            level_from_rate(9.99, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.ELEVATED
        )

    def test_at_or_above_high_is_high(self) -> None:
        assert (
            level_from_rate(10.0, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.HIGH
        )
        assert (
            level_from_rate(16.3, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.HIGH
        )
        assert (
            level_from_rate(100.0, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.HIGH
        )

    def test_boundary_inclusive(self) -> None:
        """Граница ``low_above`` и ``high_above`` inclusive (>=)."""
        # low_above=5 → rate=5.0 это ELEVATED, не LOW
        assert (
            level_from_rate(5.0, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.ELEVATED
        )
        # high_above=10 → rate=10.0 это HIGH
        assert (
            level_from_rate(10.0, low_above=5.0, high_above=10.0)
            is BacklogPressureLevel.HIGH
        )

    def test_invalid_low_above_greater_than_high_behaves_like_normal(self) -> None:
        """Защита от невалидного конфига: конструктор BacklogPressure уже
        отвергает ``low_above > high_above`` (см. test_validation). Чистая
        функция ``level_from_rate`` принимает любые числа и сравнивает
        ``rate >= high_above`` → HIGH, иначе ``rate >= low_above`` →
        ELEVATED, иначе LOW. Для rate=4.0 с инвертированными порогами
        ``low=10 > high=5`` функция возвращает LOW (rate < high=5 неверно,
        rate=4 < high=5 → LOW path)."""
        # rate=4.0 < high=5.0 → LOW (даже если low_above > high_above).
        assert (
            level_from_rate(4.0, low_above=10.0, high_above=5.0)
            is BacklogPressureLevel.LOW
        )
        # rate=15.0 >= high=5.0 → HIGH.
        assert (
            level_from_rate(15.0, low_above=10.0, high_above=5.0)
            is BacklogPressureLevel.HIGH
        )


# ---------------------------------------------------------------------------
# record / events_per_minute
# ---------------------------------------------------------------------------


class FakeClock:
    """Подменённый monotonic clock для детерминированных тестов."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class TestRecordAndRate:
    def test_empty_backlog_is_zero_rate(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(window_sec=60.0, clock=clock)
        assert bp.events_per_minute() == 0.0
        assert bp.level() is BacklogPressureLevel.LOW

    def test_single_event_rate_is_one_per_minute(self) -> None:
        """window_sec=60, 1 событие → 1 event / 1 min = 1/мин."""
        clock = FakeClock()
        bp = BacklogPressure(window_sec=60.0, clock=clock)
        bp.record()
        assert bp.events_per_minute() == 1.0

    def test_rate_normalized_by_window_sec(self) -> None:
        """window_sec=30, 16 событий за 30с → 32/мин."""
        clock = FakeClock()
        bp = BacklogPressure(window_sec=30.0, clock=clock)
        for _ in range(16):
            bp.record()
        assert bp.events_per_minute() == pytest.approx(32.0)

    def test_record_returns_current_level(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(
            window_sec=60.0, low_above=5.0, high_above=10.0, clock=clock
        )
        # До записи — LOW.
        assert bp.record() is BacklogPressureLevel.LOW
        # 5 событий → 5/мин = граница ELEVATED.
        for _ in range(4):
            level = bp.record()
        assert level is BacklogPressureLevel.ELEVATED
        # До 10 — ELEVATED.
        for _ in range(5):
            level = bp.record()
        assert level is BacklogPressureLevel.HIGH

    def test_total_counter_is_cumulative(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(window_sec=60.0, clock=clock)
        for _ in range(7):
            bp.record()
        snap = bp.snapshot()
        assert snap.total_events == 7


# ---------------------------------------------------------------------------
# Sliding window
# ---------------------------------------------------------------------------


class TestSlidingWindow:
    def test_events_older_than_window_are_dropped(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(window_sec=60.0, clock=clock)
        for _ in range(10):
            bp.record()
        # 10 событий в момент t=1000 → 10/мин = HIGH.
        assert bp.events_per_minute() == pytest.approx(10.0)
        assert bp.level() is BacklogPressureLevel.HIGH
        # Через 61 секунду все события старше окна.
        clock.advance(61.0)
        assert bp.events_per_minute() == 0.0
        assert bp.level() is BacklogPressureLevel.LOW

    def test_partial_window_keeps_recent_events(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(window_sec=60.0, clock=clock)
        # 5 событий в t=1000.
        for _ in range(5):
            bp.record()
        # 30с спустя — 5 событий в окне 60с → 5/мин.
        clock.advance(30.0)
        assert bp.events_per_minute() == pytest.approx(5.0)
        # + ещё 30с (t=1030) — старые события ещё в окне (< 60с).
        clock.advance(30.0)
        assert bp.events_per_minute() == pytest.approx(5.0)
        # + ещё 1с — окно сдвинулось, события старше 60с отброшены.
        clock.advance(1.0)
        assert bp.events_per_minute() == 0.0

    def test_burst_during_window(self) -> None:
        """100 событий за 30с при window_sec=60 → 100/мин (но >= high_above)."""
        clock = FakeClock()
        bp = BacklogPressure(
            window_sec=60.0, low_above=5.0, high_above=10.0, clock=clock
        )
        for _ in range(100):
            bp.record()
        assert bp.events_per_minute() == pytest.approx(100.0)
        assert bp.level() is BacklogPressureLevel.HIGH


# ---------------------------------------------------------------------------
# Fairness gate
# ---------------------------------------------------------------------------


class TestShouldSuppressPublish:
    def test_low_does_not_suppress(self) -> None:
        bp = BacklogPressure()
        assert bp.should_suppress_publish(BacklogPressureLevel.LOW) is False

    def test_elevated_does_not_suppress(self) -> None:
        bp = BacklogPressure()
        assert bp.should_suppress_publish(BacklogPressureLevel.ELEVATED) is False

    def test_high_suppresses(self) -> None:
        bp = BacklogPressure()
        assert bp.should_suppress_publish(BacklogPressureLevel.HIGH) is True

    def test_mark_suppressed_increments_counter(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(clock=clock)
        bp.mark_suppressed()
        bp.mark_suppressed()
        assert bp.snapshot().suppressed_events == 2
        # total_events не растёт от mark_suppressed (только от record).
        assert bp.snapshot().total_events == 0


# ---------------------------------------------------------------------------
# Publish interval
# ---------------------------------------------------------------------------


class TestPublishInterval:
    def test_first_call_should_publish(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(publish_min_interval_sec=1.0, clock=clock)
        assert bp.should_publish() is True

    def test_second_call_within_window_should_not_publish(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(publish_min_interval_sec=1.0, clock=clock)
        bp.mark_published()
        clock.advance(0.5)
        assert bp.should_publish() is False

    def test_call_after_window_should_publish(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(publish_min_interval_sec=1.0, clock=clock)
        bp.mark_published()
        clock.advance(1.5)
        assert bp.should_publish() is True

    def test_zero_interval_always_publishes(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(publish_min_interval_sec=0.0, clock=clock)
        bp.mark_published()
        assert bp.should_publish() is True


# ---------------------------------------------------------------------------
# Snapshot serialization
# ---------------------------------------------------------------------------


class TestSnapshot:
    def test_to_json_has_stable_fields(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(window_sec=60.0, clock=clock)
        for _ in range(12):
            bp.record()
        bp.mark_suppressed()
        snap = bp.snapshot()
        data = json.loads(snap.to_json())
        assert set(data.keys()) == {
            "level",
            "events_per_minute",
            "window_events",
            "total_events",
            "suppressed_events",
            "window_sec",
            "low_above",
            "high_above",
        }
        assert data["level"] == "high"
        assert data["events_per_minute"] == pytest.approx(12.0)
        assert data["window_events"] == 12
        assert data["total_events"] == 12
        assert data["suppressed_events"] == 1
        assert data["window_sec"] == 60.0

    def test_to_json_keys_sorted(self) -> None:
        """Stable ordering для читаемого лога (sort_keys=True)."""
        bp = BacklogPressure()
        snap = bp.snapshot()
        keys = list(json.loads(snap.to_json()).keys())
        assert keys == sorted(keys)


# ---------------------------------------------------------------------------
# is_quiet — self-test
# ---------------------------------------------------------------------------


class TestIsQuiet:
    def test_empty_backlog_is_quiet(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(clock=clock)
        assert bp.is_quiet() is True

    def test_single_event_below_low_is_quiet(self) -> None:
        """Один event при window=60 → rate=1/мин < low_above=5 → quiet."""
        clock = FakeClock()
        bp = BacklogPressure(
            window_sec=60.0, low_above=5.0, high_above=10.0, clock=clock
        )
        bp.record()
        assert bp.is_quiet() is True

    def test_two_events_is_not_quiet(self) -> None:
        """Два events → rate=2/мин < low, но >1 событие → НЕ quiet."""
        clock = FakeClock()
        bp = BacklogPressure(
            window_sec=60.0, low_above=5.0, high_above=10.0, clock=clock
        )
        bp.record()
        bp.record()
        assert bp.is_quiet() is False

    def test_high_pressure_is_not_quiet(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(
            window_sec=60.0, low_above=5.0, high_above=10.0, clock=clock
        )
        for _ in range(20):
            bp.record()
        assert bp.is_quiet() is False

    def test_quiet_after_window_expires(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(window_sec=60.0, clock=clock)
        # 1 событие — почти тишина (< low_above). e2e pre-flight считает
        # это «можно проигрывать».
        bp.record()
        assert bp.is_quiet() is True
        # Через 60.001с событие вне окна → точно тишина.
        clock.advance(60.001)
        assert bp.is_quiet() is True


# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------


class TestReset:
    def test_reset_clears_all_counters(self) -> None:
        clock = FakeClock()
        bp = BacklogPressure(clock=clock)
        for _ in range(5):
            bp.record()
        bp.mark_suppressed()
        bp.mark_published()
        bp.reset()
        snap = bp.snapshot()
        assert snap.window_events == 0
        assert snap.total_events == 0
        assert snap.suppressed_events == 0
        assert snap.events_per_minute == 0.0


# ---------------------------------------------------------------------------
# Fairness scenario — подготовка к integration-тесту
# ---------------------------------------------------------------------------


class TestFairnessScenario:
    """Сценарий из issue #1668 acceptance: 100% заполнение backlog.

    При HIGH давлении wake-gate **подавляет** публикацию, но wake-word
    команда всё равно проходит на следующем шаге (проверяется на уровне
    fairness gate — это чистый unit, без ROS2).
    """

    def test_high_pressure_suppresses_all_events(self) -> None:
        """После выхода на HIGH — все последующие события подавляются."""
        clock = FakeClock()
        bp = BacklogPressure(
            window_sec=60.0, low_above=5.0, high_above=10.0, clock=clock
        )
        # При low_above=5 и high_above=10 (inclusive):
        # 1..4 событий → LOW (rate 1..4 < low);
        # 5..9 событий → ELEVATED (rate 5..9);
        # 10..20 событий → HIGH (rate 10..20).
        # То есть первые 9 — пропускаются, последние 11 — подавляются.
        level: BacklogPressureLevel = BacklogPressureLevel.LOW
        for _ in range(20):
            level = bp.record()
            if bp.should_suppress_publish(level):
                bp.mark_suppressed()
        # Все 20 событий в окне, 20/мин ≥ high_above.
        assert bp.events_per_minute() == pytest.approx(20.0)
        assert bp.level() is BacklogPressureLevel.HIGH
        # Уровень последней записи = HIGH.
        assert level is BacklogPressureLevel.HIGH
        snap = bp.snapshot()
        # Все 20 событий зарегистрированы (для телеметрии).
        assert snap.total_events == 20
        # suppressed = только HIGH-события. Из 20: HIGH с 10-го по 20-й = 11.
        assert snap.suppressed_events == 11

    def test_after_silence_window_drains(self) -> None:
        """После 60с+ тишины окно очищено и backlog снова LOW."""
        clock = FakeClock()
        bp = BacklogPressure(window_sec=60.0, clock=clock)
        # Фаза 1: 16 событий/мин, HIGH.
        for _ in range(16):
            bp.record()
        assert bp.level() is BacklogPressureLevel.HIGH
        # Фаза 2: +60.001с без событий (cutoff = now - 60.0 < events_ts).
        clock.advance(60.001)
        # Старые события вне окна → rate=0, LOW.
        assert bp.events_per_minute() == 0.0
        assert bp.level() is BacklogPressureLevel.LOW
        assert bp.is_quiet() is True
        # Фаза 3: новая wake-word команда (НЕ no_wake_word → не record),
        # backlog пуст, fairness gate не срабатывает.
        # Здесь мы только проверяем, что трекер «остыл» и готов к работе.

    def test_publish_interval_one_hz(self) -> None:
        """Acceptance: backlog_overflow публикуется не чаще 1 Hz."""
        clock = FakeClock()
        bp = BacklogPressure(publish_min_interval_sec=1.0, clock=clock)
        for _ in range(20):
            bp.record()
        # Первый snapshot — публикуем.
        assert bp.should_publish() is True
        bp.mark_published()
        # Сразу второй — не публикуем.
        assert bp.should_publish() is False
        # +0.5с — не публикуем.
        clock.advance(0.5)
        assert bp.should_publish() is False
        # +0.5с (итого 1.0с) — публикуем.
        clock.advance(0.5)
        assert bp.should_publish() is True
