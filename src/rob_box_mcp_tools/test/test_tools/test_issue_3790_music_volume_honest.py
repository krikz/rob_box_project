"""Issue #3790 - ``set_music_volume`` не врёт об успехе: уровень подтверждает scsynth (``/s_get``), а не «отправили».

Исходы (``engine.master_gain``): ``applied`` / ``rejected(no_master_node | osc_error | server_fail | mismatch)`` /
``timeout``. Проверяем чистые функции на готовых OSC-пакетах, тул - с фейковым транспортом, а сам сокет
``RenardoRuntime._osc_exchange`` - против настоящего UDP-сервера-«scsynth» на loopback (без SuperCollider).
"""

from __future__ import annotations

import socket
import struct
import threading
from typing import Callable, List, Optional

import pytest

from .._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.engine import master_gain as mg
    from rob_box_mcp_tools.engine.renardo_runtime import RenardoRuntime, osc_message
    from rob_box_mcp_tools.tools.music import SetMusicVolumeTool
_ros_stubs = _ros.fixture()

NODE = RenardoRuntime.MASTER_NODE


def _gain_reply(value: float, node: int = NODE) -> bytes:
    return osc_message("/n_set", node, "gain", value)


def _fail_reply(text: str = "Node 999 not found") -> bytes:
    return osc_message("/fail", "/n_set", text)


class TestPureOutcomes:
    def test_gain_reply_equal_to_requested_is_applied(self) -> None:
        out = mg.outcome_from_replies([_gain_reply(0.7)], NODE, 0.7)
        assert out.status == mg.APPLIED and out.applied
        assert out.confirmed == pytest.approx(0.7)

    def test_float32_noise_is_within_tolerance(self) -> None:
        assert mg.outcome_from_replies([_gain_reply(0.706)], NODE, 0.70632).applied

    def test_gain_reply_with_other_level_is_rejected_mismatch(self) -> None:
        out = mg.outcome_from_replies([_gain_reply(0.5)], NODE, 0.7)
        assert (out.status, out.reason) == (mg.REJECTED, mg.MISMATCH)
        assert out.confirmed == pytest.approx(0.5)

    def test_fail_node_not_found_is_no_master_node(self) -> None:
        out = mg.outcome_from_replies([_fail_reply()], NODE, 0.7)
        assert (out.status, out.reason) == (mg.REJECTED, mg.NO_MASTER_NODE)
        assert "999" in out.detail and out.confirmed is None

    def test_other_fail_is_server_fail(self) -> None:
        out = mg.outcome_from_replies([_fail_reply("Control 'gain' wrong type")], NODE, 0.7)
        assert (out.status, out.reason) == (mg.REJECTED, mg.SERVER_FAIL)

    def test_no_replies_is_timeout(self) -> None:
        out = mg.outcome_from_replies([], NODE, 0.7)
        assert (out.status, out.reason) == (mg.TIMEOUT, mg.TIMEOUT)
        assert not out.applied

    def test_foreign_packets_are_ignored(self) -> None:
        noise = [osc_message("/done", "/g_new"), _gain_reply(0.7, node=1000), b"garbage", b""]
        assert mg.outcome_from_replies(noise, NODE, 0.7).status == mg.TIMEOUT
        assert mg.outcome_from_replies([*noise, _gain_reply(0.7)], NODE, 0.7).applied

    def test_first_final_reply_decides(self) -> None:
        assert mg.outcome_from_replies([_fail_reply(), _gain_reply(0.7)], NODE, 0.7).reason == mg.NO_MASTER_NODE


class TestConfirmGainTransport:
    def test_sends_n_set_then_s_get_and_reports_applied(self) -> None:
        seen: list = []

        def transport(calls, timeout, is_final):
            seen.append((list(calls), timeout, is_final(_gain_reply(0.7)), is_final(osc_message("/done", "/x"))))
            return [_gain_reply(0.7)]

        out = mg.confirm_gain(transport, NODE, 0.7)
        assert out.applied
        calls, timeout, final_gain, final_done = seen[0]
        assert calls == [("/n_set", NODE, "gain", 0.7), ("/s_get", NODE, "gain")]
        assert timeout == mg.CONFIRM_TIMEOUT_S and final_gain and not final_done

    def test_oserror_is_rejected_osc_error_not_applied(self) -> None:
        def transport(calls, timeout, is_final):
            raise ConnectionRefusedError("scsynth is down")

        out = mg.confirm_gain(transport, NODE, 0.7)
        assert (out.status, out.reason) == (mg.REJECTED, mg.OSC_ERROR)
        assert "ConnectionRefusedError" in out.detail

    def test_silent_transport_is_timeout(self) -> None:
        assert mg.confirm_gain(lambda calls, timeout, is_final: [], NODE, 0.7).status == mg.TIMEOUT

    def test_fail_from_transport_is_no_master_node(self) -> None:
        out = mg.confirm_gain(lambda calls, timeout, is_final: [_fail_reply()], NODE, 0.7)
        assert out.reason == mg.NO_MASTER_NODE


def _tool(gain: float = 0.5, reply: Optional[Callable] = None):
    runtime = RenardoRuntime(master_gain=gain, boot=False)
    runtime._osc_exchange = reply or (lambda calls, timeout, is_final: [_gain_reply(calls[0][3])])  # type: ignore
    return SetMusicVolumeTool(None, runtime), runtime


class TestToolBuildsAnswerFromOutcome:
    def test_applied_says_old_and_new_percent_from_confirmed_level(self) -> None:
        tool, runtime = _tool(0.5, lambda calls, timeout, is_final: [_gain_reply(0.7)])
        res = tool.execute(action="set", level=70)
        assert res.success and res.message == "Громкость музыки: 50% → 70%"
        assert res.data["outcome"] == "applied" and res.data["percent"] == 70
        assert runtime.master_gain == pytest.approx(0.7)

    def test_server_level_is_truth_when_it_differs(self) -> None:
        tool, runtime = _tool(0.5, lambda calls, timeout, is_final: [_gain_reply(0.5)])
        res = tool.execute(action="louder")
        assert not res.success and res.data["reason"] == mg.MISMATCH
        assert runtime.master_gain == pytest.approx(0.5)

    def test_no_master_node_is_failure_without_percent(self) -> None:
        tool, runtime = _tool(0.5, lambda calls, timeout, is_final: [_fail_reply()])
        res = tool.execute(action="louder")
        assert not res.success
        assert res.error == res.message == "Не получилось поменять громкость музыки (в звуковом сервере нет мастер-фейдера)."
        assert "%" not in res.message and "percent" not in res.data and "new_gain" not in res.data
        assert res.data["outcome"] == "rejected" and res.data["reason"] == mg.NO_MASTER_NODE
        assert runtime.master_gain == 0.5  # фейдер не сдвинулся - состояние не врёт

    def test_oserror_is_failure(self) -> None:
        def down(calls, timeout, is_final):
            raise OSError("network unreachable")

        tool, runtime = _tool(0.5, down)
        res = tool.execute(action="max")
        assert not res.success and res.data["reason"] == mg.OSC_ERROR
        assert "не отвечает" in res.message and "100%" not in res.message
        assert runtime.master_gain == 0.5

    def test_timeout_is_failure_not_louder(self) -> None:
        tool, runtime = _tool(0.5, lambda calls, timeout, is_final: [])
        res = tool.execute(action="louder")
        assert not res.success and res.data["outcome"] == mg.TIMEOUT
        assert "не подтвердил" in res.message
        assert runtime.master_gain == 0.5

    def test_unchanged_level_still_confirmed_and_reported_as_such(self) -> None:
        tool, _ = _tool(1.0)
        res = tool.execute(action="louder")
        assert res.success and "не изменилась" in res.message and res.data["old_gain"] == res.data["new_gain"] == 1.0

    def test_event_source_declared_for_adr0150_guard(self) -> None:
        assert SetMusicVolumeTool.event_source.startswith("inproc:")


class _FakeScsynthServer:
    """Настоящий UDP-сокет на loopback: отвечает отправителю как scsynth на ``/n_set`` + ``/s_get``."""

    def __init__(self, mode: str) -> None:
        self.mode = mode
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(2.0)
        self.port = self.sock.getsockname()[1]
        self.got: List[bytes] = []
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self) -> None:
        level: Optional[float] = None
        try:
            while True:
                data, addr = self.sock.recvfrom(4096)
                self.got.append(data)
                if data.startswith(b"/n_set") and self.mode == "ok":
                    level = float(struct.unpack(">f", data[-4:])[0])
                elif data.startswith(b"/n_set") and self.mode == "fail":
                    self.sock.sendto(_fail_reply(), addr)
                elif data.startswith(b"/s_get") and self.mode == "ok":
                    self.sock.sendto(_gain_reply(level or 0.0), addr)
        except OSError:
            return

    def close(self) -> None:
        self.sock.close()
        self.thread.join(timeout=3)


@pytest.fixture
def scsynth_server():
    servers: List[_FakeScsynthServer] = []

    def make(mode: str) -> _FakeScsynthServer:
        servers.append(_FakeScsynthServer(mode))
        return servers[-1]

    yield make
    for server in servers:
        server.close()


def _runtime_on(port: int, gain: float = 0.5) -> RenardoRuntime:
    runtime = RenardoRuntime(master_gain=gain, boot=False)
    runtime.SC_PORT = port  # type: ignore[misc]
    return runtime


class TestRealSocketExchange:
    def test_live_node_confirms_level_over_udp(self, scsynth_server) -> None:
        server = scsynth_server("ok")
        runtime = _runtime_on(server.port)
        res = SetMusicVolumeTool(None, runtime).execute(action="set", level=80)
        assert res.success, res.error
        assert runtime.master_gain == pytest.approx(0.8)
        assert [m.split(b"\x00")[0] for m in server.got] == [b"/n_set", b"/s_get"]

    def test_missing_node_fail_reply_over_udp(self, scsynth_server) -> None:
        server = scsynth_server("fail")
        runtime = _runtime_on(server.port)
        res = SetMusicVolumeTool(None, runtime).execute(action="louder")
        assert not res.success and res.data["reason"] == mg.NO_MASTER_NODE
        assert runtime.master_gain == 0.5

    def test_silent_server_times_out_over_udp(self, scsynth_server) -> None:
        server = scsynth_server("silent")
        runtime = _runtime_on(server.port)
        res = SetMusicVolumeTool(None, runtime).execute(action="louder")
        assert not res.success and res.data["outcome"] == mg.TIMEOUT
        assert runtime.master_gain == 0.5

    def test_dead_port_is_failure_not_louder(self) -> None:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()  # порт свободен: никто не слушает
        runtime = _runtime_on(port)
        res = SetMusicVolumeTool(None, runtime).execute(action="louder")
        assert not res.success and res.data["reason"] in (mg.OSC_ERROR, mg.TIMEOUT)
        assert runtime.master_gain == 0.5
