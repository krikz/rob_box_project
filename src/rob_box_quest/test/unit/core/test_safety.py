"""Unit-тесты safety Watchdog."""

from rob_box_quest.core.safety import WATCHDOG_TIMEOUT_S, Watchdog


def test_watchdog_starts_tripped():
    w = Watchdog()
    # Ни разу не feed → tripped.
    assert w.tripped(now_monotonic=100.0) is True


def test_feed_resets_tripped():
    w = Watchdog()
    w.feed(now_monotonic=100.0)
    assert w.tripped(now_monotonic=100.0 + 0.1) is False


def test_tripped_after_timeout():
    w = Watchdog()
    w.feed(now_monotonic=100.0)
    # Свежий — нет.
    assert w.tripped(now_monotonic=100.0 + 0.1) is False
    # Ровно на границе — нет (строгое >).
    assert w.tripped(now_monotonic=100.0 + WATCHDOG_TIMEOUT_S) is False
    # Через 1 мс после границы — да.
    assert w.tripped(now_monotonic=100.0 + WATCHDOG_TIMEOUT_S + 0.001) is True


def test_feed_resets_after_trip():
    w = Watchdog()
    w.feed(now_monotonic=100.0)
    # Прошло много времени → tripped.
    assert w.tripped(now_monotonic=200.0) is True
    # Свежий feed снова оживил.
    w.feed(now_monotonic=300.0)
    assert w.tripped(now_monotonic=300.0 + 0.1) is False


def test_reset_clears_state():
    w = Watchdog()
    w.feed(now_monotonic=100.0)
    w.reset()
    # После reset — снова tripped (как новый).
    assert w.tripped(now_monotonic=100.0) is True


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
