"""Unit-тесты для local_test/quest_smoke_lib.py (Phase 1.7, issue #1643).

Не требуют aiohttp / rclpy / websockets-сервера — проверяют чистую логику:
- frame codec (LEB128, header)
- FrameType значения (синхронизированы с rob_box_quest/protocol/frame.py)
- exit-code matrix (битовая маска по секциям)

Запуск: pytest src/rob_box_quest/test/unit/test_quest_smoke_lib.py -v
(или: python -m pytest path/to/this.py)
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

# quest_smoke_lib лежит рядом с тестом (в local_test/) — добавляем в sys.path.
_LIB = Path(__file__).resolve().parent / "quest_smoke_lib.py"
sys.path.insert(0, str(_LIB.parent))
import quest_smoke_lib as L  # noqa: E402


# --- Frame codec -------------------------------------------------------------
class TestFrameCodec:
    def test_roundtrip_basic(self):
        raw = L.encode_frame(0x03, 0x1001, b'{"topic":"camera_rear"}')
        ftype, sid, payload = L.decode_frame(raw)
        assert ftype == 0x03
        assert sid == 0x1001
        assert payload == b'{"topic":"camera_rear"}'

    def test_roundtrip_empty_payload(self):
        raw = L.encode_frame(0x20, 0, b"")
        ftype, sid, payload = L.decode_frame(raw)
        assert ftype == 0x20 and sid == 0 and payload == b""

    def test_leb128_zero(self):
        assert L._encode_leb128(0) == b"\x00"

    def test_leb127_single_byte(self):
        assert L._encode_leb128(127) == b"\x7f"

    def test_leb128_two_bytes(self):
        assert L._encode_leb128(128) == b"\x80\x01"

    def test_leb128_two_bytes_max(self):
        assert L._encode_leb128(16383) == b"\xff\x7f"

    def test_leb128_three_bytes(self):
        assert L._encode_leb128(16384) == b"\x80\x80\x01"

    def test_leb128_decode_roundtrip(self):
        for n in (0, 1, 127, 128, 255, 16383, 16384, 0xFFFF, 0x100000):
            encoded = L._encode_leb128(n)
            value, off = L._decode_leb128(encoded, 0)
            assert value == n
            assert off == len(encoded)

    def test_leb128_negative_raises(self):
        with pytest.raises(ValueError, match="unsigned only"):
            L._encode_leb128(-1)


# --- FrameType ID contract (должен совпадать с rob_box_quest/protocol/frame.py)
#
# Этот тест валидирует, что локальные константы (HELLO=0x01 и т.д.) совпадают
# с реальным FrameType из src/rob_box_quest/protocol/frame.py. Если бы они
# расходились — клиент/сервер обменивались бы мусором.
#
# Skip'ается если rob_box_quest не установлен в pytest-env (Phase 1.4/1.5
# код ещё не в develop — он придёт через #1652). Контракт будет проверяться
# автоматически в полном CI-прогоне где rob_box_quest установлен.
def _try_import_upstream():
    try:
        from rob_box_quest.protocol.frame import FrameType as UpstreamFT
        return UpstreamFT
    except ImportError:
        return None


@pytest.mark.skipif(
    _try_import_upstream() is None,
    reason="rob_box_quest not installed in this test env (Phase 1.5 lands via #1652)",
)
def test_frame_type_ids_match_upstream():
    UpstreamFT = _try_import_upstream()
    assert L.HELLO == UpstreamFT.HELLO == 0x01
    assert L.WELCOME == UpstreamFT.WELCOME == 0x02
    assert L.SUBSCRIBE == UpstreamFT.SUBSCRIBE == 0x03
    assert L.UNSUBSCRIBE == UpstreamFT.UNSUBSCRIBE == 0x04
    assert L.BINARY_FRAME == UpstreamFT.BINARY_FRAME == 0x10
    assert L.JSON_CMD == UpstreamFT.JSON_CMD == 0x11
    assert L.JSON_EVENT == UpstreamFT.JSON_EVENT == 0x12
    assert L.GOODBYE == UpstreamFT.GOODBYE == 0x20
    assert L.ERROR == UpstreamFT.ERROR == 0xFF


# --- Exit-code matrix --------------------------------------------------------
class TestExitCode:
    """Smoke: exit code = bitwise-OR fail-bits; partial (skip без fail) = 99."""

    def test_all_pass_is_zero(self):
        sr = L.SuiteResult()
        for n in ("dns", "tls", "healthz", "wss_handshake", "heartbeat", "subscribe_jpeg", "goodbye"):
            sr.add(L.SectionResult(name=n, status="pass"))
        assert L._exit_code(sr) == 0

    def test_dns_fail_is_one(self):
        sr = L.SuiteResult()
        sr.add(L.SectionResult(name="dns", status="fail"))
        sr.add(L.SectionResult(name="tls", status="pass"))
        assert L._exit_code(sr) == 1

    def test_tls_fail_is_two(self):
        sr = L.SuiteResult()
        sr.add(L.SectionResult(name="dns", status="pass"))
        sr.add(L.SectionResult(name="tls", status="fail"))
        assert L._exit_code(sr) == 2

    def test_multiple_fails_or_bits(self):
        sr = L.SuiteResult()
        sr.add(L.SectionResult(name="dns", status="fail"))           # bit 1
        sr.add(L.SectionResult(name="heartbeat", status="fail"))     # bit 5
        sr.add(L.SectionResult(name="goodbye", status="fail"))       # bit 7
        assert L._exit_code(sr) == (1 | 5 | 7)

    def test_skip_with_pass_returns_99(self):
        """Partial state: есть SKIP, но нет FAIL → 99 (CI понимает)."""
        sr = L.SuiteResult()
        sr.add(L.SectionResult(name="dns", status="pass"))
        sr.add(L.SectionResult(name="tls", status="pass"))
        sr.add(L.SectionResult(name="healthz", status="skip"))
        sr.add(L.SectionResult(name="wss_handshake", status="skip"))
        assert L._exit_code(sr) == 99

    def test_skip_with_fail_returns_fail_bits(self):
        sr = L.SuiteResult()
        sr.add(L.SectionResult(name="dns", status="pass"))
        sr.add(L.SectionResult(name="tls", status="fail"))
        sr.add(L.SectionResult(name="healthz", status="skip"))
        assert L._exit_code(sr) == 2  # tls bit wins over skip


# --- to_dict shape ----------------------------------------------------------
class TestSuiteResultShape:
    def test_to_dict_keys(self):
        d = L.SuiteResult().to_dict()
        assert set(d.keys()) == {"pass", "fail", "skip", "sections"}

    def test_to_dict_counts(self):
        sr = L.SuiteResult()
        sr.add(L.SectionResult(name="dns", status="pass"))
        sr.add(L.SectionResult(name="tls", status="pass"))
        sr.add(L.SectionResult(name="healthz", status="fail"))
        sr.add(L.SectionResult(name="wss_handshake", status="skip"))
        d = sr.to_dict()
        assert d["pass"] == 2
        assert d["fail"] == 1
        assert d["skip"] == 1
        assert len(d["sections"]) == 4
