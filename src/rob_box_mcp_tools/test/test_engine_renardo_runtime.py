"""Звуковой сервер движка v2 — ``engine.renardo_runtime.RenardoRuntime`` (#3614).

Перенесено из тестов ``MusicManager`` (удалён в #3614) то, что живёт дальше: формат OSC (#778, #1444),
мастер-фейдер (#986, #3125), слушатель ``/fail`` (#1808), подъём Renardo (795d5447), проба синтов
(13.08, 19.08), палитра, подтверждённая сервером (#2838, #2841). Без Renardo, scsynth и ROS.
"""

import re
import socket
import struct
import sys
import types
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from rob_box_mcp_tools.engine import renardo_runtime as rr
from rob_box_mcp_tools.engine.renardo_runtime import CRITICAL_SYNTHS, RenardoRuntime, osc_message

pytestmark = pytest.mark.unit


def _runtime(**kwargs):
    runtime = RenardoRuntime(boot=False, **kwargs)
    warnings: list = []
    runtime.attach_logger(SimpleNamespace(warning=warnings.append, info=lambda m: None, error=warnings.append))
    return runtime, warnings


def _osc_string(s: str) -> bytes:
    b = s.encode() + b"\x00"
    while len(b) % 4:
        b += b"\x00"
    return b


def _fail(*strings: str) -> bytes:
    return _osc_string("/fail") + _osc_string("," + "s" * len(strings)) + b"".join(_osc_string(s) for s in strings)


class _Sock:
    """UDP-сокет: записывает отправленное, ``recvfrom`` — по очереди ответов (исключение — бросить)."""

    def __init__(self, replies=()):
        self.sent: list = []
        self.replies = list(replies)
        self.timeouts: list = []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def settimeout(self, t):
        self.timeouts.append(t)

    def sendto(self, data, addr):
        self.sent.append((bytes(data), addr))

    def recvfrom(self, n):
        reply = self.replies.pop(0) if self.replies else socket.timeout()
        if isinstance(reply, BaseException) or (isinstance(reply, type) and issubclass(reply, BaseException)):
            raise reply
        return reply, ("127.0.0.1", 57110)


# --------------------------------------------------------------------------- формат OSC (#778, #1444)


def test_osc_address_only_is_aligned():
    assert osc_message("/status") == b"/status\x00,\x00\x00\x00"


def test_osc_int_args_match_the_legacy_g_new_packet():
    """``/g_new 1 0 0`` байт в байт как раньше — поведение scsynth не меняется."""
    assert osc_message("/g_freeAll", 1) == b"/g_freeAll\x00\x00,i\x00\x00\x00\x00\x00\x01"
    assert osc_message("/g_new", 1, 0, 0) == (
        b"/g_new\x00\x00" b",iii\x00\x00\x00\x00" b"\x00\x00\x00\x01" b"\x00\x00\x00\x00" b"\x00\x00\x00\x00")


def test_control_name_travels_as_string_not_int32():
    """#1444: имя контрола как int32 scsynth молча игнорирует (пробные ноды свистели с amp=1)."""
    data = osc_message("/n_set", 999, "gain", 0.5)
    assert b",isf" in data and b"gain\x00\x00\x00\x00" in data
    assert data.endswith(struct.pack(">f", 0.5)) and len(data) % 4 == 0


def test_probe_s_new_has_the_siiisf_tag():
    data = osc_message("/s_new", "pads", 9000, 0, 1, "amp", 0.0)
    assert data.startswith(b"/s_new\x00\x00") and b",siiisf" in data and len(data) % 4 == 0


# --------------------------------------------------------------------------- send_osc и /fail (#1808)


def test_send_osc_goes_to_scsynth_and_waits_briefly_for_fail():
    runtime, warnings = _runtime()
    sock = _Sock([_fail("/g_new", "Group 1 not found")])
    got: list = []
    runtime.osc_fail_listener = got.append
    with patch.object(rr.socket, "socket", lambda *a, **k: sock):
        runtime.send_osc("/g_new", 1, 0, 0)
    assert sock.sent == [(osc_message("/g_new", 1, 0, 0), ("127.0.0.1", 57110))]
    assert sock.timeouts == [RenardoRuntime.OSC_REPLY_TIMEOUT_SECONDS]
    assert got == ["/g_new Group 1 not found"] and "Group 1 not found" in warnings[0]


def test_send_osc_timeout_is_the_normal_silent_case():
    runtime, warnings = _runtime()
    sock = _Sock([socket.timeout])
    with patch.object(rr.socket, "socket", lambda *a, **k: sock):
        runtime.send_osc("/n_set", 999, "gain", 0.5)
    assert warnings == []


def test_send_osc_propagates_transport_errors():
    """Транспорт без скрытого «проглотить»: best-effort решают вызывающие (``set_master_gain``, адаптер)."""
    runtime, _ = _runtime()
    with patch.object(rr.socket, "socket", side_effect=OSError("network unreachable")):
        with pytest.raises(OSError):
            runtime.send_osc("/g_new", 1, 0, 0)


def test_done_and_garbage_are_not_failures():
    runtime, warnings = _runtime()
    runtime._on_reply(_osc_string("/done") + _osc_string(",ss") + _osc_string("/foxdot") + _osc_string("ok"))
    runtime._on_reply(b"\x00\x00\x00\x00")
    assert warnings == []


def test_reply_loop_logs_fail_survives_garbage_and_exits_on_close():
    runtime, warnings = _runtime()
    sock = _Sock([b"\xff\xff garbage", _fail("/s_new", "SynthDef blip not found"), OSError("closed")])
    runtime._reply_loop(sock)
    assert len(warnings) == 1 and "SynthDef blip not found" in warnings[0]


def test_reply_listener_attaches_once_per_real_socket():
    runtime, _ = _runtime()
    real = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rt = SimpleNamespace(Server=SimpleNamespace(client=SimpleNamespace(socket=real)))
        with patch.object(rr.threading, "Thread") as thread_cls:
            thread_cls.return_value = Mock()
            runtime._attach_reply_listener(rt)
            runtime._attach_reply_listener(rt)
        assert thread_cls.call_count == 1
        kwargs = thread_cls.call_args.kwargs
        assert kwargs["target"] == runtime._reply_loop and kwargs["args"] == (real,) and kwargs["daemon"] is True
    finally:
        real.close()


@pytest.mark.parametrize("rt", [
    None, SimpleNamespace(), SimpleNamespace(Server=SimpleNamespace(client=SimpleNamespace())),
    SimpleNamespace(Server=SimpleNamespace(client=SimpleNamespace(socket="not-a-socket")))])
def test_reply_listener_ignores_other_renardo_object_graphs(rt):
    runtime, _ = _runtime()
    with patch.object(rr.threading, "Thread") as thread_cls:
        runtime._attach_reply_listener(rt)
    thread_cls.assert_not_called()


# --------------------------------------------------------------------------- мастер-фейдер (#986, #3125)


def _echo_scsynth(calls_log):
    """Живой scsynth с узлом 999: на ``/s_get`` отвечает уровнем, выставленным ``/n_set`` (#3790)."""
    def exchange(calls, timeout, is_final):
        calls_log.append(list(calls))
        return [rr.osc_message("/n_set", 999, "gain", calls[0][3])]
    return exchange


def test_master_gain_targets_node_999_and_clamps():
    runtime, _ = _runtime(master_gain=0.5)
    log: list = []
    with patch.object(RenardoRuntime, "_osc_exchange", new=staticmethod(_echo_scsynth(log))):
        assert runtime.set_master_gain(0.35) == pytest.approx(0.35)
        assert log[0][0] == ("/n_set", 999, "gain", pytest.approx(0.35))
        assert log[0][1] == ("/s_get", 999, "gain")  # уровень проверяется у сервера, а не «отправлено»
        assert runtime.set_master_gain(3.0) == 1.0 and runtime.set_master_gain(-1.0) == 0.0
    assert RenardoRuntime.MASTER_NODE < 1000  # renardo раздаёт id с 1001: узел мастер-шины без коллизий


def test_master_gain_survives_a_dead_socket():
    """Сокет мёртв: уровень НЕ считается применённым - возвращается прежний, в лог - предупреждение (#3790)."""
    runtime, warnings = _runtime(master_gain=0.5)
    with patch.object(RenardoRuntime, "_osc_exchange", side_effect=OSError("no route")):
        assert runtime.set_master_gain(0.4) == 0.5
        outcome = runtime.apply_master_gain(0.4)
    assert (outcome.status, outcome.reason) == ("rejected", "osc_error")
    assert runtime.master_gain == 0.5
    assert "master gain" in warnings[0] and "osc_error" in warnings[0]


# --------------------------------------------------------------------------- подъём Renardo (795d5447)


def _fake_renardo(monkeypatch, tmp_path, synthdefs):
    rt = SimpleNamespace(Server=SimpleNamespace(booted=True), effect_manager=SimpleNamespace(reload=lambda: None),
                         SynthDefs=synthdefs, SynthDef=lambda name: SimpleNamespace(name=name, add=Mock()))
    module = types.ModuleType("renardo_lib")
    module.runtime = rt
    monkeypatch.setitem(sys.modules, "renardo_lib", module)
    monkeypatch.setitem(sys.modules, "renardo_lib.runtime", rt)
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("USERPROFILE", str(tmp_path))
    monkeypatch.setattr(rr.time, "sleep", lambda _s: None)
    monkeypatch.setattr(rr.socket, "socket", lambda *a, **k: _Sock())  # проба: таймаут — синт на месте
    return rt


def test_boot_brings_renardo_up_and_is_idempotent(monkeypatch, tmp_path):
    """Повторный подъём не зовёт ``sdef.add()`` снова: каждый add мутирует UGen-граф (live 20.08)."""
    rt = _fake_renardo(monkeypatch, tmp_path, {f"s{i}": SimpleNamespace(add=Mock()) for i in range(12)})
    runtime, _ = _runtime(sclang_log_path=str(tmp_path / "absent.log"))
    assert runtime.boot() is True and runtime.available and runtime.last_error is None
    assert runtime.context()["SynthDefs"] is rt.SynthDefs
    runtime._available = False  # как после сбоя: context() поднимает снова
    runtime.context()
    assert all(sdef.add.call_count == 1 for sdef in rt.SynthDefs.values())


def test_failed_boot_is_reported_and_retried_on_next_use(monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, "renardo_lib", None)  # импорт падает
    monkeypatch.setitem(sys.modules, "renardo_lib.runtime", None)
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("USERPROFILE", str(tmp_path))
    runtime, warnings = _runtime()
    assert runtime.boot() is False and not runtime.available
    assert runtime.context() == {} and "Renardo не поднялся" in warnings[-1]
    rt = _fake_renardo(monkeypatch, tmp_path, {"pluck": SimpleNamespace(add=Mock())})
    assert runtime.context()["SynthDefs"] is rt.SynthDefs  # следующий заказ музыки поднимает снова


# --------------------------------------------------------------------------- проба синтов (13.08, 19.08)


def test_command_not_found_is_not_a_missing_synthdef(monkeypatch):
    """13.08: невыровненный ``/s_new`` давал «Command not found» — его нельзя считать пропажей синта."""
    sock = _Sock([b"/fail\x00\x00\x00,ss\x00/s_new\x00\x00Command not found\x00"] * len(CRITICAL_SYNTHS))
    monkeypatch.setattr(rr.socket, "socket", lambda *a, **k: sock)
    runtime, _ = _runtime()
    rt = SimpleNamespace(SynthDefs={"pads": SimpleNamespace(add=Mock())})
    runtime._resend_missing(rt)
    assert rt.SynthDefs["pads"].add.call_count == 0
    frees = [d for d, _ in sock.sent if d.startswith(b"/n_free")]
    assert frees and all(len(d) % 4 == 0 for d in frees)  # пробные ноды освобождаются (#1444)


def test_really_missing_synth_is_resent_and_already_added_goes_through_load(monkeypatch):
    monkeypatch.setattr(rr.time, "sleep", lambda _s: None)
    missing = b"/fail\x00\x00\x00,ss\x00/s_new\x00\x00SynthDef pads not found\x00"
    monkeypatch.setattr(rr.socket, "socket", lambda *a, **k: _Sock([missing] * 200))
    runtime, warnings = _runtime()
    runtime._synthdefs_added.add("pads")
    rt = SimpleNamespace(SynthDefs={"pads": SimpleNamespace(add=Mock(), load=Mock()),
                                    "dub": SimpleNamespace(add=Mock(), load=Mock())})
    runtime._resend_missing(rt)
    assert rt.SynthDefs["pads"].load.call_count >= 1 and rt.SynthDefs["pads"].add.call_count == 0
    assert rt.SynthDefs["dub"].add.call_count == 1  # не добавлялся — add() один раз, дальше load()
    assert "still missing" in warnings[-1]


# --------------------------------------------------------------------------- палитра (#2838, #2841)


def _foxdot_init_preload() -> list:
    root = next(p for p in Path(__file__).resolve().parents if (p / "docker").is_dir() and (p / "src").is_dir())
    content = (root / "docker" / "sound" / "voice_assistant" / "foxdot_init.sc").read_text(encoding="utf-8")
    names: list = []
    for var in ("startupSynths", "customSynths"):
        match = re.search(r"var " + var + r" = \[(.*?)\];", content, re.S)
        assert match, var
        names += re.findall(r'"([^"]+)"', match.group(1))
    return names


def _sclang_log_for(names) -> str:
    lines = ["FoxDot OSCdef registered. Ready to compile SynthDefs.", "Server running: true"]
    lines += [f"SynthDef in scsynth: {name}" for name in names]
    lines.append(f"SynthDef preload finished: {len(names)} defs")
    return "\n".join(lines) + "\n"


def _runtime_with_log(tmp_path, log_text, added):
    log = tmp_path / "sclang.log"
    log.write_text(log_text, encoding="utf-8")
    runtime, warnings = _runtime(sclang_log_path=str(log))
    runtime._server_confirmed = rr.load_confirmed_synths(str(log))
    runtime._synthdefs_added = set(added)
    return runtime, warnings


def test_palette_is_unknown_before_boot():
    runtime, _ = _runtime()
    assert runtime.known_synth_names() is None  # «неизвестна», а не «ничего нельзя»


def test_palette_is_what_sclang_confirmed_in_scsynth(tmp_path):
    """#2838: 'sine' отправлен ``sdef.add()``, но до сервера не доехал — его в палитре нет."""
    preload = _foxdot_init_preload()
    runtime, warnings = _runtime_with_log(tmp_path, _sclang_log_for(preload),
                                          (set(preload) - {"masterlimiter", "masterfilter"}) | {"sine", "siren"})
    known = runtime.known_synth_names()
    assert not ({"sine", "siren"} & known) and known <= set(preload)
    assert "masterfilter" not in known and "masterlimiter" not in known
    assert {"epiano", "sinepad", "supersawlead"} <= known
    runtime._log_synth_truth()
    assert "#2838" in warnings[0] and "'sine'" in warnings[0]


def test_unverified_fallback_when_sclang_log_is_missing(tmp_path):
    runtime, warnings = _runtime(sclang_log_path=str(tmp_path / "absent.log"))
    runtime._server_confirmed = rr.load_confirmed_synths(str(tmp_path / "absent.log"))
    runtime._synthdefs_added = {"pluck", "sine"}
    assert runtime._server_confirmed is None and {"pluck", "sine", "supersawlead"} <= runtime.known_synth_names()
    runtime._log_synth_truth()
    assert "сервером не проверена" in warnings[0]


def test_groove_loop_is_in_the_palette_only_with_the_repo_preload(tmp_path):
    """#2841: ``loop`` подтверждает только прелоад ``foxdot_init.sc`` — в логе робота 23.09 его нет."""
    robot_log = (Path(__file__).resolve().parent / "fixtures" / "sclang_robot_2026-09-23.log").read_text(
        encoding="utf-8")
    added = (set(_foxdot_init_preload()) - {"masterlimiter", "masterfilter"}) | {"loop"}
    runtime, _ = _runtime_with_log(tmp_path, robot_log, added)
    assert "loop" not in runtime.known_synth_names()
    assert "loop" in _foxdot_init_preload()
    runtime, _ = _runtime_with_log(tmp_path, _sclang_log_for(_foxdot_init_preload()), added)
    assert "loop" in runtime.known_synth_names()


def test_runtime_feeds_the_v2_adapter(monkeypatch):
    """Проводка: адаптер владельца получает контекст, палитру и ``send_osc`` рантайма (не приватные поля)."""
    from rob_box_mcp_tools.engine.renardo_adapter import RenardoAdapter

    runtime, _ = _runtime()
    sent = []
    monkeypatch.setattr(runtime, "send_osc", lambda *a: sent.append(a))
    monkeypatch.setattr(runtime, "boot", lambda: False)
    adapter = RenardoAdapter(runtime.context, runtime.known_synth_names, runtime.send_osc)
    assert sent and sent[0][:2] == ("/n_set", 999)  # сброс мастер-шины при старте адаптера
    assert adapter.check(SimpleNamespace(synths=frozenset(), samples=frozenset(), sample_files=frozenset())) == (
        "renardo_unavailable", "нет Clock/Samples в контексте Renardo")
