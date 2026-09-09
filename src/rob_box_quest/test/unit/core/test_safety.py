"""Unit-тесты safety Watchdog — edge-triggered + ARMED/DISARMED FSM.

Семантика (см. core/safety.py):
  - DISARMED по умолчанию: trip/consume_trip = False, никакого спама
    до первого feed() (шлем не подключался → нода молчит).
  - ARMED (после feed()): trip по timeout, consume_trip() = True один раз.
  - reset() → DISARMED (клиент ушёл, не триггерим до нового feed()).
"""

from rob_box_quest.core.safety import WATCHDOG_TIMEOUT_S, Watchdog


def test_disarmed_default_does_not_trip():
    """Без feed() никаких trip'ов — убивает спам EMERGENCY в первые 100мс
    после старта ноды (шлем ещё не подключался)."""
    w = Watchdog()
    assert w.armed is False
    assert w.tripped(now_monotonic=100.0) is False
    assert w.consume_trip(now_monotonic=100.0) is False
    # Даже через час — DISARMED не триггерится.
    assert w.tripped(now_monotonic=100.0 + 3600.0) is False


def test_feed_arms_watchdog():
    """После feed() → ARMED, в окне timeout — не tripped."""
    w = Watchdog()
    w.feed(now_monotonic=100.0)
    assert w.armed is True
    assert w.tripped(now_monotonic=100.0 + 0.1) is False


def test_tripped_after_timeout_when_armed():
    """ARMED + прошло > TIMEOUT_S → tripped."""
    w = Watchdog()
    w.feed(now_monotonic=100.0)
    # Свежий — нет.
    assert w.tripped(now_monotonic=100.0 + 0.1) is False
    # Ровно на границе — нет (строгое >).
    assert w.tripped(now_monotonic=100.0 + WATCHDOG_TIMEOUT_S) is False
    # Через 1 мс после границы — да.
    assert w.tripped(now_monotonic=100.0 + WATCHDOG_TIMEOUT_S + 0.001) is True


def test_consume_trip_is_edge_triggered():
    """consume_trip() возвращает True только ОДИН раз на trip,
    затем False до следующего feed() — это и есть анти-спам."""
    w = Watchdog()
    w.feed(now_monotonic=100.0)
    # Прошло много времени → ARMED+timed out.
    later = 100.0 + WATCHDOG_TIMEOUT_S + 1.0
    # Первый consume — True (edge).
    assert w.consume_trip(later) is True
    # Повторные consumes — False (дедуп, не спамим).
    assert w.consume_trip(later + 0.1) is False
    assert w.consume_trip(later + 0.2) is False
    assert w.consume_trip(later + 1.0) is False


def test_feed_after_trip_clears_edge_flag():
    """После нового feed() — edge-флаг снят, можно снова словить trip."""
    w = Watchdog()
    w.feed(now_monotonic=100.0)
    later = 100.0 + WATCHDOG_TIMEOUT_S + 1.0
    assert w.consume_trip(later) is True
    # Клиент вернулся → feed.
    w.feed(now_monotonic=later + 0.5)
    # Свежий — нет.
    assert w.consume_trip(later + 0.6) is False
    # Клиент снова пропал → опять один раз.
    later2 = later + 0.5 + WATCHDOG_TIMEOUT_S + 1.0
    assert w.consume_trip(later2) is True
    assert w.consume_trip(later2 + 0.1) is False


def test_reset_disarms_watchdog():
    """reset() → DISARMED: никаких trip'ов до следующего feed()."""
    w = Watchdog()
    w.feed(now_monotonic=100.0)
    w.reset()
    assert w.armed is False
    assert w.tripped(now_monotonic=100.0) is False
    assert w.consume_trip(now_monotonic=100.0) is False
    # Чтобы снова триггериться — нужен новый feed().
    w.feed(now_monotonic=200.0)
    assert w.armed is True


def test_custom_timeout():
    w = Watchdog(timeout_s=1.0)
    w.feed(now_monotonic=100.0)
    # 0.5 с — нет.
    assert w.tripped(now_monotonic=100.5) is False
    # 1.5 с — да.
    assert w.tripped(now_monotonic=101.5) is True


def test_last_feed_monotonic_property():
    w = Watchdog()
    assert w.last_feed_monotonic is None
    w.feed(now_monotonic=42.5)
    assert w.last_feed_monotonic == 42.5


def test_fast_timer_loop_does_not_spam():
    """Симуляция timer 10 Гц (каждые 100мс) на DISARMED watchdog —
    ноль ложных trip'ов. Это та регрессия которую лечим."""
    w = Watchdog()
    t = 1000.0
    trips = 0
    for _ in range(100):  # 10 секунд симуляции
        if w.consume_trip(t):
            trips += 1
        t += 0.1
    assert trips == 0, f"DISARMED должен молчать, поймали {trips} ложных trip'ов"

    # Подключился клиент, поиграл, пропал — ОДИН trip, не 100.
    w.feed(now_monotonic=t)
    for _ in range(100):
        if w.consume_trip(t):
            trips += 1
        t += 0.1
    assert trips == 1, f"После link-loss должен быть ровно 1 trip, получили {trips}"

    # Клиент вернулся, поиграл, снова пропал — ещё один.
    w.feed(now_monotonic=t)
    for _ in range(100):
        if w.consume_trip(t):
            trips += 1
        t += 0.1
    assert trips == 2, f"После второго link-loss должно быть 2 trip'а всего, получили {trips}"
