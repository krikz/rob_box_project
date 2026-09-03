"""Unit-тесты ClientSession (чистая логика, без aiohttp)."""

import time

import pytest

from rob_box_quest.server.session import (
    ClientSession,
    HEARTBEAT_INTERVAL_S,
    SessionState,
    WATCHDOG_TIMEOUT_S,
    generate_pin,
)


class TestPinGeneration:
    def test_pin_is_six_digits(self):
        pin = generate_pin()
        assert len(pin) == 6
        assert pin.isdigit()

    def test_pin_format_includes_leading_zeros(self):
        # randbelow(1_000_000) может выдать 0 → "000000"
        # Проверяем что формат не падает на любом значении.
        for _ in range(50):
            pin = generate_pin()
            assert len(pin) == 6


class TestAuthFlow:
    def test_initial_state_is_awaiting_hello(self):
        s = ClientSession()
        assert s.state == SessionState.AWAITING_HELLO

    def test_mark_authenticated_moves_state(self):
        s = ClientSession()
        s.mark_authenticated("0.1.0", ["webxr"])
        assert s.state == SessionState.AUTHENTICATED
        assert s.client_version == "0.1.0"
        assert s.capabilities == ["webxr"]

    def test_cannot_authenticate_twice(self):
        s = ClientSession()
        s.mark_authenticated("0.1.0", [])
        with pytest.raises(RuntimeError):
            s.mark_authenticated("0.2.0", [])


class TestWatchdog:
    def test_watchdog_disabled_before_auth(self):
        s = ClientSession()
        # Без auth watchdog не считается.
        assert s.watchdog_tripped() is False
        # Даже через час.
        assert s.watchdog_tripped(now_monotonic=time.monotonic() + 3600.0) is False

    def test_watchdog_trips_after_timeout(self):
        s = ClientSession()
        s.mark_authenticated("0.1.0", [])
        t0 = time.monotonic()
        s.feed_ping(now_monotonic=t0)
        # Свежий ping — нет trip.
        assert s.watchdog_tripped(now_monotonic=t0 + 0.1) is False
        # Ровно на границе — нет trip (строгое >).
        assert s.watchdog_tripped(now_monotonic=t0 + WATCHDOG_TIMEOUT_S) is False
        # Через 1 мс после границы — trip.
        assert s.watchdog_tripped(now_monotonic=t0 + WATCHDOG_TIMEOUT_S + 0.001) is True

    def test_feed_ping_resets_watchdog(self):
        s = ClientSession()
        s.mark_authenticated("0.1.0", [])
        t0 = time.monotonic()
        s.feed_ping(now_monotonic=t0)
        # Подошли к границе, но не перешли.
        boundary = t0 + WATCHDOG_TIMEOUT_S - 0.01
        assert s.watchdog_tripped(now_monotonic=boundary) is False
        # Пришёл свежий ping прямо на границе.
        s.feed_ping(now_monotonic=boundary)
        # Проверяем в пределах нового окна — НЕ trip.
        still_safe = boundary + WATCHDOG_TIMEOUT_S - 0.05
        assert s.watchdog_tripped(now_monotonic=still_safe) is False
        # За пределами нового окна — trip.
        assert s.watchdog_tripped(now_monotonic=still_safe + 0.1) is True


class TestStreamIdAllocation:
    def test_first_id_is_in_server_pool(self):
        s = ClientSession()
        sid = s.allocate_stream_id(set())
        assert 0x1000 <= sid < 0x10000

    def test_avoids_already_used(self):
        s = ClientSession()
        used = {0x1000, 0x1001, 0x1002}
        sid = s.allocate_stream_id(used)
        assert sid == 0x1003

    def test_pool_exhaustion_raises(self):
        s = ClientSession()
        # Заполним весь пул 0x1000..0xFFFF.
        used = set(range(0x1000, 0x10000))
        with pytest.raises(RuntimeError):
            s.allocate_stream_id(used)


class TestClose:
    def test_close_sets_state(self):
        s = ClientSession()
        s.mark_authenticated("0.1.0", [])
        s.close()
        assert s.state == SessionState.CLOSED
        assert s.is_open() is False


def test_heartbeat_constant_is_200ms():
    assert HEARTBEAT_INTERVAL_S == 0.2


class TestSubprotocolVersion:
    """AV-16: ClientSession хранит ``protocol_version`` (1/2)."""

    def test_initial_protocol_version_is_none(self):
        s = ClientSession()
        assert s.protocol_version is None

    def test_apply_subprotocol_v2(self):
        s = ClientSession()
        v = s.apply_subprotocol("robbox-quest-v2")
        assert v == 2
        assert s.protocol_version == 2

    def test_apply_subprotocol_v1(self):
        s = ClientSession()
        v = s.apply_subprotocol("robbox-quest-v1")
        assert v == 1
        assert s.protocol_version == 1

    def test_apply_subprotocol_none_falls_back_to_v1(self):
        # Без Sec-WebSocket-Protocol (старые клиенты) — считаем v1.
        s = ClientSession()
        v = s.apply_subprotocol(None)
        assert v == 1

    def test_apply_subprotocol_unknown_falls_back_to_v1(self):
        # Будущая v3 → сервер не знает, default = v1 (надёжно/консервативно).
        s = ClientSession()
        v = s.apply_subprotocol("robbox-quest-v99")
        assert v == 1


class TestServerClientId:
    """AV-16/§11: сервер формирует client_id сам, не доверяя payload."""

    def test_server_client_id_set_after_auth(self):
        s = ClientSession()
        assert s.server_client_id is None
        s.mark_authenticated("0.1.0", [])
        assert s.server_client_id is not None
        # Формат «quest:<uuid>» — префикс для отличия от telegram-клиентов.
        assert s.server_client_id.startswith("quest:")
        assert s.server_client_id.endswith(s.session_id)

    def test_two_sessions_have_distinct_server_client_id(self):
        s1 = ClientSession()
        s2 = ClientSession()
        s1.mark_authenticated("0.1.0", [])
        s2.mark_authenticated("0.1.0", [])
        assert s1.server_client_id != s2.server_client_id
        assert s1.session_id != s2.session_id
