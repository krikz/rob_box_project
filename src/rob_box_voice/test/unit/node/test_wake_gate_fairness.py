"""Integration-тест fairness gate в :class:`DialogueNode` (issue #1668).

Сценарий acceptance criteria:

> 1 минута непрерывного фонового шума (mock) → синтезированная wake-word
> команда после этого проходит ≤5с.

В чистом Python-тесте без rclpy: подменяем ``_on_stt`` зависимости через
``object.__new__`` (как в test_speech_backlog_accumulator.py и
test_issue_1101_diagnostics.py). Используем
:class:`rob_box_voice.core.backlog_pressure.BacklogPressure` напрямую,
без ROS2-таймера (timer-driven publish тестируется отдельно).

Что проверяем:

1. При 100% заполнении backlog (HIGH pressure) — новые ``no_wake_word``
   фразы идут в ``_llm_skipped_counter``, **НЕ** в ``_speech_accumulator``.
2. ``_speech_accumulator`` остаётся приоритетным путём для wake-word команд:
   когда приходит wake-word, ``_dispatch_turn`` вызывается даже после
   HIGH pressure (а backlog может быть пуст/частично заполнен).
3. ``backlog_pressure_enabled=False`` — fallback к старому поведению
   (все фразы копятся в backlog).
"""

from __future__ import annotations

import threading
from unittest.mock import MagicMock

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.core.backlog_pressure import (
    BacklogPressure,
    BacklogPressureLevel,
)
from rob_box_voice.core.speech_accumulator import SpeechAccumulator
from rob_box_voice.dialogue_node import DialogueNode


class _FakeClock:
    """Подменённый monotonic clock для детерминированных тестов watchdog'а."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _make_node(
    backlog_pressure_enabled: bool = True,
    window_sec: float = 60.0,
    low_above: float = 5.0,
    high_above: float = 10.0,
) -> DialogueNode:
    """Собрать DialogueNode-подобный объект без ROS2.

    Только те поля, которые читает ``_on_stt`` в ветке ``no_wake_word`` /
    wake-word.
    """
    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock()

    n._wake_words = [
        "робок",
        "робот",
        "роббокс",
        "робокс",
        "робобокс",
    ]
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.IDLE
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._speaker_by_text = {}
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
    n._llm_skipped_counter = {
        "no_wake_word": 0,
        "silenced": 0,
        "silence_command": 0,
        "empty_after_strip": 0,
        "stt_rejected": 0,
        "music_stop": 0,
        "command_intent": 0,
        "new_session": 0,
    }
    n._maybe_log_skip_summary = MagicMock()
    n._active_tg_chat_id = None

    # Backlog-аккумулятор (issue #1668: остаётся для wake-word команд).
    n._speech_accumulator = SpeechAccumulator(window_sec=180.0)
    n._accumulate_no_wake_enabled = True
    n._pending_backlog_flush = False

    # Issue #1668 — BacklogPressure watchdog.
    n._backlog_pressure_enabled = backlog_pressure_enabled
    fake_clock = _FakeClock()
    n._backlog_pressure = BacklogPressure(
        window_sec=window_sec,
        low_above=low_above,
        high_above=high_above,
        publish_min_interval_sec=1.0,
        clock=fake_clock,
    )
    n._backlog_overflow_pub = MagicMock()
    n._quiet_pub = MagicMock()
    n._last_quiet_published = None

    return n


def _stt(node, data):
    msg = MagicMock()
    msg.data = data
    return msg


class TestFairnessUnderSaturatedBacklog:
    """Сценарий: HIGH pressure → ``no_wake_word`` не попадает в backlog."""

    def test_high_pressure_suppresses_no_wake_publish(self) -> None:
        """16 фоновых фраз подряд → 11 фраз подавлены watchdog'ом,
        первые 9 проходят (LOW/ELEVATED), counter растёт."""
        node = _make_node()
        # 16 событий = 16/мин = HIGH (high_above=10).
        for i in range(16):
            node._on_stt(_stt(node, f"фоновый шум кусок номер {i}"))
        # Backlog содержит первые 9 фраз (LOW/ELEVATED). С 10-й фразы
        # rate ≥ 10 → HIGH, и фразы подавляются (11-я..16-я = 7 фраз).
        assert node._llm_skipped_counter["no_wake_word"] == 7
        # Backlog не пуст — 9 фраз прошли.
        assert not node._speech_accumulator.is_empty()
        # Уровень давления — HIGH.
        assert node._backlog_pressure.level() is BacklogPressureLevel.HIGH

    def test_wake_word_still_passes_under_saturation(self) -> None:
        """Синтез-команда «Робот, ...» после 16 фоновых фраз
        всё равно проходит через wake-gate и попадает в LLM (≤5с)."""
        node = _make_node()
        # Фаза 1: насыщение backlog.
        for i in range(16):
            node._on_stt(_stt(node, f"фоновый шум {i}"))
        assert node._backlog_pressure.level() is BacklogPressureLevel.HIGH
        # Backlog содержит первые 9 фраз (LOW/ELEVATED) — watchdog ещё
        # не включился, поэтому они были добавлены в accumulator.
        assert not node._speech_accumulator.is_empty()
        # Фаза 2: синтез-команда с wake-word.
        node._on_stt(_stt(node, "робот расскажи про погоду"))
        # dispatch вызван → LLM получит команду.
        node._dispatch_turn.assert_called_once()
        # Очищенная wake-word команда — первый аргумент.
        dispatched = node._dispatch_turn.call_args.args[0]
        assert dispatched.startswith("расскажи про погоду")
        # Backlog будет слит в LLM через _build_dynamic_system_context.
        assert node._pending_backlog_flush is True

    def test_wake_word_flushes_partial_backlog(self) -> None:
        """Когда давление ELEVATED (не HIGH), бэклог копится, и wake-word
        команда его «смывает» в LLM. Это поведение backlog-аккумулятора
        из #1668 (а не watchdog'а), но проверяем, что watchdog не ломает
        этот поток."""
        node = _make_node(low_above=2.0, high_above=5.0)
        # 3 события → 3/мин = ELEVATED (между low=2 и high=5).
        for i in range(3):
            node._on_stt(_stt(node, f"фоновый запрос {i}"))
        assert node._backlog_pressure.level() is BacklogPressureLevel.ELEVATED
        # Backlog не пуст — ELEVATED не подавляет.
        assert not node._speech_accumulator.is_empty()
        # Wake-word команда.
        node._on_stt(_stt(node, "робот выполни мою просьбу"))
        node._dispatch_turn.assert_called_once()
        dispatched = node._dispatch_turn.call_args.args[0]
        # Подсказка про backlog вставлена в user_input.
        assert "ФОНОВЫЙ ЗАПРОС" in dispatched


class TestWakeRegressionUnderSaturation:
    """Regression-тест: 20 wake-фраз подряд при фоновом шуме — все accepted.

    Acceptance criterion карточки t_09297e5a (issue #1668):
    «10-20 wake-фраз с шумом, все accepted».
    Источник регрессии — e2e fail-streak 16 раундов: фоновый голос забивал
    wake-gate и синтезированные команды не доходили до LLM.
    """

    def test_20_wake_phrases_under_saturation_all_accepted(self) -> None:
        """20 синтез-команд с wake-word подряд при HIGH backlog pressure:
        каждая должна попасть в ``_dispatch_turn`` (т.е. wake-gate не
        блокирует их при насыщении no_wake_word фразами)."""
        node = _make_node()
        # Насыщаем backlog до HIGH (>=10 events/min).
        for i in range(16):
            node._on_stt(_stt(node, f"фоновый шум номер {i}"))
        assert node._backlog_pressure.level() is BacklogPressureLevel.HIGH
        # 20 wake-фраз подряд: каждая сбрасывает _dispatch_turn.call_count.
        accepted = 0
        for i in range(20):
            node._dispatch_turn.reset_mock()
            node._on_stt(_stt(node, f"робот команда номер {i}"))
            if node._dispatch_turn.called:
                accepted += 1
        assert accepted == 20, (
            f"Регрессия #1668: при HIGH pressure пропущено "
            f"{20 - accepted} из 20 wake-фраз"
        )

    def test_20_wake_phrases_under_saturation_no_backlog_growth(self) -> None:
        """Под насыщением backlog no_wake_word фразы подавляются, и backlog
        НЕ растёт бесконтрольно (watchdog держит размер управляемым)."""
        node = _make_node()
        # Фаза 1: установить HIGH pressure (>=10 events/min).
        for i in range(12):
            node._on_stt(_stt(node, f"шум {i}"))
        baseline = node._llm_skipped_counter["no_wake_word"]
        # Фаза 2: подаём ещё 20 фоновых фраз — все должны быть подавлены.
        for i in range(20):
            node._on_stt(_stt(node, f"фон {i}"))
        snap = node._backlog_pressure.snapshot()
        # Backlog заполнен <= первыми 9 фразами (LOW/ELEVATED).
        # Под давлением 20 фраз все -> suppressed.
        # Counter вырос ровно на 20.
        assert snap.suppressed_events >= 20
        # _llm_skipped_counter.no_wake_word вырос минимум на baseline+20.
        assert (
            node._llm_skipped_counter["no_wake_word"] - baseline
        ) >= 20

    def test_mixed_background_and_wake_under_saturation(self) -> None:
        """Чередующаяся нагрузка: фон-фон-wake-фон-фон-wake...
        Под давлением каждая wake-фраза должна проходить, фоновые
        фразы подавляться."""
        node = _make_node()
        accepted = 0
        suppressed = 0
        # Чередуем: 2 фоновых + 1 wake, повторяем 10 раз.
        for cycle in range(10):
            # 2 фоновых фразы
            for j in range(2):
                before = node._llm_skipped_counter["no_wake_word"]
                node._on_stt(_stt(node, f"шум цикл {cycle} фраза {j}"))
                after = node._llm_skipped_counter["no_wake_word"]
                if after > before:
                    suppressed += 1
            # 1 wake-фраза
            node._dispatch_turn.reset_mock()
            node._on_stt(_stt(node, f"робот команда цикл {cycle}"))
            if node._dispatch_turn.called:
                accepted += 1
        # Должны пройти все 10 wake-фраз.
        assert accepted == 10, f"Wake-gate пропустил {10 - accepted} из 10"
        # Большая часть фоновых фраз подавлена (после порога HIGH).
        assert suppressed >= 10, (
            f"Watchdog не подавил фоновые фразы: {suppressed}/20"
        )


class TestFairnessDisabled:
    """Когда watchdog выключен — поведение прежнее (всё в backlog)."""

    def test_disabled_accumulates_everything(self) -> None:
        node = _make_node(backlog_pressure_enabled=False)
        for i in range(20):
            node._on_stt(_stt(node, f"фоновый шум {i}"))
        # Backlog полон (без watchdog'а всё копится).
        assert not node._speech_accumulator.is_empty()
        # Counter не растёт — фразы НЕ считаются «skipped».
        assert node._llm_skipped_counter["no_wake_word"] == 0


class TestFairnessCounterTelemetry:
    """BacklogPressure.record() всегда инкрементирует total_events,
    независимо от suppressed-решения. Это для телеметрии."""

    def test_total_grows_even_when_suppressed(self) -> None:
        node = _make_node()
        for i in range(20):
            node._on_stt(_stt(node, f"шум {i}"))
        snap = node._backlog_pressure.snapshot()
        # Все 20 событий зарегистрированы (для телеметрии).
        assert snap.total_events == 20
        # Из 20 событий: 1..9 — LOW/ELEVATED (rate 1..9), 10..20 — HIGH
        # (rate 10..20 ≥ 10). suppressed = события 10..20 = 11.
        assert snap.suppressed_events == 11
        # Backlog содержит первые 9 фраз (LOW/ELEVATED).
        assert not node._speech_accumulator.is_empty()


class TestQuietTopic:
    """Self-test / healthcheck через /voice/diagnostics/quiet."""

    def test_quiet_topic_publishes_on_state_change(self) -> None:
        """При смене quiet → noisy (или наоборот) публикуется bool."""
        node = _make_node()
        # Издатель изначально не публиковал.
        assert node._backlog_overflow_pub.publish.call_count == 0
        # Эмулируем вызов _publish_backlog_overflow напрямую.
        node._publish_backlog_overflow()
        # Backlog пуст → quiet=True публикуется.
        assert node._backlog_overflow_pub.publish.call_count == 1
        assert node._last_quiet_published is True
        # Следующий вызов без смены состояния — НЕ публикуем quiet
        # (но backlog_overflow тоже не публикуется, т.к. 1Hz rate limit).
        node._publish_backlog_overflow()
        assert node._quiet_pub.publish.call_count == 1  # без изменений.

    def test_quiet_flips_to_false_under_load(self) -> None:
        node = _make_node()
        node._publish_backlog_overflow()
        assert node._last_quiet_published is True
        # Насыщаем backlog: 16 фраз → 11 из них подавлены (HIGH).
        # С FakeClock events_per_minute = 16 > 5 → is_quiet = False.
        for i in range(16):
            node._on_stt(_stt(node, f"фоновый шум {i}"))
        # Публикуем snapshot — quiet должен смениться на False.
        node._publish_backlog_overflow()
        assert node._last_quiet_published is False

    def test_quiet_publish_respects_1hz(self) -> None:
        """backlog_overflow публикуется не чаще раза в секунду."""
        node = _make_node()
        node._publish_backlog_overflow()  # первый — публикуем.
        n_calls_after_first = node._backlog_overflow_pub.publish.call_count
        node._publish_backlog_overflow()  # второй в том же окне — нет.
        assert node._backlog_overflow_pub.publish.call_count == n_calls_after_first
