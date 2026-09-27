"""Unit-тесты streams.wifi (RSSI из /proc/net/wireless)."""

from rob_box_quest.streams.wifi import parse_wireless, read_wifi_rssi

# Реальный дамп с Raspberry Pi (2 строки шапки + интерфейс).
PROC_SAMPLE = """Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
 face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
 wlan0: 0000   62.  -48.  -256        0      0      0      0     34        0
"""

PROC_TWO_IFACES = PROC_SAMPLE + " wlan1: 0000   40.  -72.  -256        0      0      0      0      0        0\n"


class TestParseWireless:
    def test_reads_level_as_dbm(self):
        assert parse_wireless(PROC_SAMPLE) == -48

    def test_picks_first_iface_when_unspecified(self):
        assert parse_wireless(PROC_TWO_IFACES) == -48

    def test_picks_requested_iface(self):
        assert parse_wireless(PROC_TWO_IFACES, iface="wlan1") == -72

    def test_unknown_iface_returns_none(self):
        assert parse_wireless(PROC_TWO_IFACES, iface="wlan9") is None

    def test_header_only_returns_none(self):
        header = "\n".join(PROC_SAMPLE.splitlines()[:2]) + "\n"
        assert parse_wireless(header) is None

    def test_empty_text_returns_none(self):
        assert parse_wireless("") is None

    def test_garbage_level_is_skipped(self):
        broken = " wlan0: 0000   62.  ---.  -256        0      0      0      0      0        0\n"
        assert parse_wireless(broken) is None


class TestReadWifiRssi:
    def test_reads_from_file(self, tmp_path):
        path = tmp_path / "wireless"
        path.write_text(PROC_SAMPLE, encoding="utf-8")
        assert read_wifi_rssi(str(path)) == -48

    def test_missing_file_returns_none(self, tmp_path):
        # Dev-машина под Windows — /proc/net/wireless не существует.
        assert read_wifi_rssi(str(tmp_path / "nope")) is None
