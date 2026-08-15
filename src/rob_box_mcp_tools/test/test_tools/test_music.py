"""
test_music.py - Unit тесты для инструментов управления музыкой

Тестирует:
- MusicManager: фильтрация кода, проверка SC, execute_code, stop_pattern, stop_all,
  set_vibe_preset, get_state
- ExecuteMusicCodeTool, StopMusicTool, SetVibePresetTool, GetMusicStateTool
"""

import sys
import time
import json
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import pytest

# Mock ROS 2 modules before importing anything from rob_box_mcp_tools
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "std_msgs",
    "std_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.srv",
    "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.music import (  # noqa: E402
    MusicManager,
    ExecuteMusicCodeTool,
    StopMusicTool,
    SetVibePresetTool,
    GetMusicStateTool,
    SaveTrackTool,
    ListTracksTool,
    LoadTrackTool,
    DeleteTrackTool,
    SearchSamplesTool,
    SetDjModeTool,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_manager(*, sc_running: bool = False, renardo_available: bool = False) -> MusicManager:
    """Create a MusicManager with patched infrastructure."""
    from rob_box_voice.core.music_stack_validation import MusicStackStatus

    mgr = MusicManager.__new__(MusicManager)
    mgr._max_amp = 0.7
    mgr._pattern_history = {}
    mgr._active_patterns = set()
    mgr._current_preset = None
    mgr._renardo_available = renardo_available
    mgr._renardo_last_error = None
    mgr._renardo_context = {}
    # issue G-MUSIC — music-stack health (must match __init__ defaults).
    # Tests that need a degraded manager should overwrite _music_stack_status
    # and/or _require_healthy directly.
    mgr._music_stack_status = MusicStackStatus(
        is_healthy=True,
        oscdef_registered=True,
        missing_synths=(),
        fatal_errors=(),
    )
    mgr._require_healthy = True
    mgr._critical_synths = MusicManager.DEFAULT_CRITICAL_SYNTHS
    # issue #935 — music session lifecycle (must match __init__ defaults
    # to keep tests faithful; see ``MusicManager.__init__``)
    mgr._auto_stop_ttl_seconds = 300
    mgr._music_session_active_since = None
    mgr._last_music_activity_at = None
    mgr._last_stop_at = None
    mgr._auto_stop_count = 0
    # issue #990 — segments safety-net deadline
    mgr._music_deadline_at = None
    mgr._music_deadline_segments = None
    # issue #1000 — DJ mode flag (default off; tests can call mgr.set_dj_mode(True))
    mgr._dj_mode_enabled = False
    mgr._check_supercollider = Mock(return_value=sc_running)
    return mgr


# ---------------------------------------------------------------------------
# MusicManager — code safety filter
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerFilter:
    """Тесты фильтра безопасности кода."""

    def setup_method(self):
        self.mgr = _make_manager()

    def test_safe_renardo_code_passes(self):
        ok, err = self.mgr._filter_code("p1 >> pluck([0, 2, 4], dur=0.5)")
        assert ok is True
        assert err == ""

    def test_import_is_blocked(self):
        ok, err = self.mgr._filter_code("import os")
        assert ok is False
        assert "import" in err

    def test_os_is_blocked(self):
        ok, err = self.mgr._filter_code("os.system('rm -rf /')")
        assert ok is False
        assert "os" in err

    def test_subprocess_is_blocked(self):
        ok, err = self.mgr._filter_code("subprocess.run(['ls'])")
        assert ok is False
        assert "subprocess" in err

    def test_open_is_blocked(self):
        ok, err = self.mgr._filter_code("open('/etc/passwd').read()")
        assert ok is False
        assert "open" in err

    def test_eval_is_blocked(self):
        ok, err = self.mgr._filter_code("eval('1+1')")
        assert ok is False
        assert "eval" in err

    def test_exec_is_blocked(self):
        ok, err = self.mgr._filter_code("exec('print(1)')")
        assert ok is False
        assert "exec" in err

    def test_dunder_import_is_blocked(self):
        ok, err = self.mgr._filter_code("__import__('os')")
        assert ok is False
        assert "__import__" in err

    def test_builtins_is_blocked(self):
        ok, err = self.mgr._filter_code("__builtins__['eval']('1')")
        assert ok is False

    def test_setattr_is_allowed(self):
        """setattr must pass — needed for Clock.future() BPM/Scale changes."""
        ok, err = self.mgr._filter_code("Clock.future(8, lambda: setattr(Clock, 'bpm', 170))")
        assert ok is True
        assert err == ""

    def test_getattr_is_allowed(self):
        """getattr must pass — needed for pattern introspection."""
        ok, err = self.mgr._filter_code("getattr(p1, 'degree')")
        assert ok is True
        assert err == ""

    def test_clock_future_setattr_scale_passes(self):
        """Clock.future with setattr for Scale/Root changes must pass."""
        ok, err = self.mgr._filter_code(
            "Clock.future(16, lambda: setattr(Scale, 'default', 'minor'))\n"
            "Clock.future(16, lambda: setattr(Root, 'default', 4))"
        )
        assert ok is True
        assert err == ""

    def test_scale_code_passes(self):
        ok, err = self.mgr._filter_code("Scale.default = Scale.major\nClock.bpm = 120")
        assert ok is True

    def test_clock_clear_passes(self):
        ok, err = self.mgr._filter_code("Clock.clear()")
        assert ok is True


# ---------------------------------------------------------------------------
# MusicManager — issue #1000 anti-click caps (oct, amplify, ramp-down)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerCaps:
    """``_cap_amp`` ограничивает amp / amplify / oct — фазa-3.2 anti-click.

    Issue #1000: ``pianovel/piano → rhpiano`` (MdaPiano цокает) уже
    автозаменяется в ``execute_code``. Капы ``oct <= 4`` и ``amplify`` ≤
    ``max_amp`` — защита от громких/резких звуков.
    """

    def setup_method(self):
        self.mgr = _make_manager()

    def test_oct_is_capped_at_4(self):
        # oct=5 (резкий диапазон) → oct=4
        code = "p1 >> pluck([0,2,4], oct=5)"
        out = self.mgr._cap_amp(code)
        assert "oct=4" in out
        assert "oct=5" not in out

    def test_oct_unchanged_when_leq_4(self):
        code = "p1 >> pluck([0,2,4], oct=3)"
        out = self.mgr._cap_amp(code)
        assert "oct=3" in out

    def test_amplify_var_is_capped(self):
        # amplify=var([1, 0.3]) → amplify=var([0.7, 0.3])
        code = "d1 >> play('X', amplify=var([1,0.3]))"
        out = self.mgr._cap_amp(code)
        assert "0.7" in out
        # внутри var() 1.0 должно быть заменено на 0.7, 0.3 остаётся
        assert "amplify=var([0.7,0.3])" in out.replace(" ", "")

    def test_amplify_simple_is_capped(self):
        # amplify=0.8 → amplify=0.7
        code = "d1 >> play('X', amplify=0.8)"
        out = self.mgr._cap_amp(code)
        assert "amplify=0.7" in out

    def test_amp_simple_is_capped(self):
        # amp=0.9 → amp=0.7 (default max_amp=0.7)
        code = "p1 >> pluck([0], amp=0.9)"
        out = self.mgr._cap_amp(code)
        assert "amp=0.7" in out

    def test_dj_mode_flag_default_off(self):
        # Issue #1000 — DJ mode flag should default to False
        assert self.mgr.dj_mode_enabled is False

    def test_set_dj_mode_toggles_flag(self):
        # Issue #1000 — set_dj_mode(True) → dj_mode_enabled True
        self.mgr.set_dj_mode(True)
        assert self.mgr.dj_mode_enabled is True
        self.mgr.set_dj_mode(False)
        assert self.mgr.dj_mode_enabled is False


# ---------------------------------------------------------------------------
# MusicManager — SuperCollider check
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerSCCheck:
    """Тесты проверки SuperCollider."""

    def _make_raw_manager(self):
        mgr = MusicManager.__new__(MusicManager)
        mgr._pattern_history = {}
        mgr._active_patterns = set()
        mgr._current_preset = None
        mgr._renardo_available = False
        mgr._renardo_context = {}
        # issue #935 — music session lifecycle defaults
        mgr._auto_stop_ttl_seconds = 300
        mgr._dj_mode_enabled = False
        mgr._music_session_active_since = None
        mgr._last_music_activity_at = None
        mgr._last_stop_at = None
        mgr._auto_stop_count = 0
        # issue #990 — segments safety-net deadline
        mgr._music_deadline_at = None
        mgr._music_deadline_segments = None
        return mgr

    def test_sc_running_returns_true(self):
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mock_sock.recvfrom.return_value = (b"\x00" * 16, ("127.0.0.1", 57110))
            result = mgr._check_supercollider()
        assert result is True

    def test_sc_not_running_returns_false(self):
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mock_sock.recvfrom.side_effect = OSError("timeout")
            result = mgr._check_supercollider()
        assert result is False

    def test_sc_check_sends_osc_status_message(self):
        """Verify the correct OSC /status message is sent to scsynth."""
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mock_sock.recvfrom.return_value = (b"\x00" * 16, ("127.0.0.1", 57110))
            mgr._check_supercollider()
        # Проверяем что отправлен правильный OSC /status
        call_args = mock_sock.sendto.call_args
        assert call_args is not None
        sent_data, (host, port) = call_args[0]
        assert sent_data == b"/status\x00,\x00\x00\x00"
        assert port == 57110

    def test_sc_check_uses_udp_socket(self):
        """Verify UDP socket (SOCK_DGRAM) is used, not TCP."""
        import socket as socket_module

        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mock_sock.recvfrom.side_effect = OSError
            mgr._check_supercollider()
        mock_sock_class.assert_called_once_with(socket_module.AF_INET, socket_module.SOCK_DGRAM)

    # ----- _send_osc_raw (issue #778) --------------------------------------

    def test_send_osc_raw_address_only(self):
        """/status style: address only, no args. Packet must be 4-byte aligned."""
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mgr._send_osc_raw("/status")
        sent_data, (host, port) = mock_sock.sendto.call_args[0]
        # /status = 7 chars + 1 NUL = 8 bytes (already aligned) + ","
        # + 3 NUL padding = 12 bytes total
        assert sent_data == b"/status\x00,\x00\x00\x00"
        assert len(sent_data) % 4 == 0
        assert port == 57110

    def test_send_osc_raw_int_args(self):
        """/g_freeAll 1: type tag ',i' + 4-byte big-endian int."""
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mgr._send_osc_raw("/g_freeAll", 1)
        sent_data, _ = mock_sock.sendto.call_args[0]
        # /g_freeAll\x00\x00,i\x00\x00\x00\x00\x00\x01
        #   ^addr 12B    ^typ 8B    ^arg 4B = 24 bytes total
        assert sent_data == b"/g_freeAll\x00\x00,i\x00\x00\x00\x00\x00\x01"
        assert len(sent_data) % 4 == 0  # 4-byte aligned

    def test_send_osc_raw_three_int_args(self):
        """/g_new 1 0 0: 3 int args, big-endian, 4-byte aligned.

        Must byte-match the legacy hand-built packet (execute_music_code
        used to construct it as: /g_new\x00\x00 + ,iii\x00\x00\x00\x00 +
        >i 1 + >i 0 + >i 0), so scsynth behaviour is unchanged.
        """
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mgr._send_osc_raw("/g_new", 1, 0, 0)
        sent_data, _ = mock_sock.sendto.call_args[0]
        expected = (
            b"/g_new\x00\x00"  # 6 chars + 2 NUL = 8 bytes (aligned)
            b",iii\x00\x00\x00\x00"  # 5 chars + 3 NUL = 8 bytes
            b"\x00\x00\x00\x01"  # >i 1
            b"\x00\x00\x00\x00"  # >i 0
            b"\x00\x00\x00\x00"  # >i 0
        )
        assert sent_data == expected
        assert len(sent_data) % 4 == 0  # 32 bytes

    def test_send_osc_raw_float_arg(self):
        """Float args are big-endian 4-byte float."""
        import struct

        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mgr._send_osc_raw("/n_set", 1, 0.5)
        sent_data, _ = mock_sock.sendto.call_args[0]
        # Last 4 bytes = 0.5 in big-endian float
        assert sent_data.endswith(struct.pack(">f", 0.5))
        # Type tag should contain ',if'
        assert b",if" in sent_data

    def test_send_osc_raw_propagates_socket_errors(self):
        """_send_osc_raw is a raw transport — it propagates OSError.

        Callers that need best-effort semantics (execute_music_code,
        stop_all) wrap it in try/except themselves, exactly like the
        legacy inline bytearray code did. This test pins the contract:
        no hidden swallow inside the method.
        """
        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock_class.return_value.__enter__ = Mock(
                side_effect=OSError("network unreachable")
            )
            with pytest.raises(OSError):
                mgr._send_osc_raw("/g_new", 1, 0, 0)


# ---------------------------------------------------------------------------
# MusicManager — Renardo initialization (regression 795d5447)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerRenardoInitialize:
    """Regression: commit 795d5447 broke `_initialize_renardo`.

    The commit added `_time.sleep(0.1)` between `sdef.add()` calls but the
    `import time as _time` lived only in `_verify_and_retry_synthdefs()`,
    so any run with >=5 SynthDefs died with ``NameError: name '_time' is
    not defined`` → ``_renardo_available = False`` → every musical e2e red.

    Fixing only `_time` would unmask a second NameError: the verification
    call passed bare ``_send_osc_raw`` which is never defined in this
    method (only ``self._send_osc_raw`` exists). Both must be covered.
    """

    @staticmethod
    def _make_rt() -> SimpleNamespace:
        """Fake renardo_lib.runtime with >5 SynthDefs (hits ``idx % 5 == 4``)."""
        return SimpleNamespace(
            Server=SimpleNamespace(booted=True),
            effect_manager=SimpleNamespace(reload=lambda: None),
            SynthDefs={
                f"s{i}": SimpleNamespace(add=lambda: None)
                for i in range(12)
            },
            # Factory used by register_sc_only_custom_synthdefs
            SynthDef=lambda name: SimpleNamespace(name=name),
        )

    def test_initialize_renardo_completes_without_nameerror(self, tmp_path, monkeypatch):
        import socket
        import types

        from rob_box_mcp_tools.tools import music as music_mod

        mgr = _make_manager()
        mgr._logger = None
        mgr._renardo_last_error = "sentinel"
        mgr._renardo_available = False

        rt = self._make_rt()
        renardo_lib = types.ModuleType("renardo_lib")
        renardo_lib.runtime = rt
        monkeypatch.setitem(sys.modules, "renardo_lib", renardo_lib)
        monkeypatch.setitem(sys.modules, "renardo_lib.runtime", rt)
        # Sample-dir bootstrap writes under $HOME — keep it out of the real home.
        monkeypatch.setenv("HOME", str(tmp_path))
        # Don't actually sleep; the NameError would still fire on _time before sleep.
        monkeypatch.setattr(music_mod.time, "sleep", lambda _seconds: None)
        # Probe socket: recvfrom timeout == "SynthDef exists" → nothing missing.
        fake_sock = MagicMock()
        fake_sock.recvfrom.side_effect = socket.timeout
        monkeypatch.setattr(music_mod.socket, "socket", lambda *a, **k: fake_sock)

        mgr._initialize_renardo()

        assert mgr._renardo_available is True
        assert mgr._renardo_last_error is None


# ---------------------------------------------------------------------------
# MusicManager — SynthDef verification probe (regression 13.08.2026)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestVerifyAndRetrySynthdefsProbe:
    """Regression: probe sent ``/s_new`` without 4-byte OSC alignment.

    ``b"/s_new\x00"`` is 7 bytes — the address pattern must be padded to
    a multiple of 4 (8). scsynth answered ``FAILURE IN SERVER: /s_new
    Command not found`` and the old check ``b"not found" in data``
    counted every one of the 31 critical SynthDefs as missing → 3 full
    re-send rounds per init → repeated ``sdef.add()`` mutations
    compounded the generated SynthDef graphs into 3+ MB ``.scd`` files.
    """

    @staticmethod
    def _fake_rt():
        return SimpleNamespace(
            SynthDefs={
                name: SimpleNamespace(add=Mock())
                for name in ("pads", "noise")
            }
        )

    def test_probe_padding_and_command_not_found_not_missing(
        self, monkeypatch, capsys
    ):
        from rob_box_mcp_tools.tools import music as music_mod

        sent: list[bytes] = []

        class _FakeSock:
            def __init__(self, *a, **k):
                pass

            def settimeout(self, t):
                pass

            def sendto(self, data, addr):
                sent.append(bytes(data))

            def recvfrom(self, n):
                # scsynth's exact reply for a malformed command —
                # must NOT be treated as "SynthDef missing".
                return (
                    b"/fail\x00\x00\x00,ss\x00/s_new\x00\x00Command not found\x00",
                    ("127.0.0.1", 57110),
                )

            def close(self):
                pass

        monkeypatch.setattr(
            music_mod.socket, "socket", lambda *a, **k: _FakeSock(*a, **k)
        )
        mgr = _make_manager()
        rt = self._fake_rt()

        mgr._verify_and_retry_synthdefs(rt, mgr._send_osc_raw)

        # Every /s_new must be 4-byte aligned (address + type string).
        assert sent, "probe должен отправлять OSC-сообщения"
        for data in sent:
            assert len(data) % 4 == 0, f"OSC not aligned: {len(data)}"
            assert data.startswith(b"/s_new\x00\x00"), f"bad address pad: {data[:16]!r}"
        # No re-send round may be triggered by "Command not found".
        assert rt.SynthDefs["pads"].add.call_count == 0
        assert rt.SynthDefs["noise"].add.call_count == 0
        assert "round 1" not in capsys.readouterr().err

    def test_probe_resends_when_synthdef_really_missing(self, monkeypatch):
        from rob_box_mcp_tools.tools import music as music_mod

        class _FakeSock:
            def __init__(self, *a, **k):
                pass

            def settimeout(self, t):
                pass

            def sendto(self, data, addr):
                pass

            def recvfrom(self, n):
                return (
                    b"/fail\x00\x00\x00,ss\x00/s_new\x00\x00SynthDef pads not found\x00",
                    ("127.0.0.1", 57110),
                )

            def close(self):
                pass

        monkeypatch.setattr(
            music_mod.socket, "socket", lambda *a, **k: _FakeSock(*a, **k)
        )
        mgr = _make_manager()
        rt = self._fake_rt()

        mgr._verify_and_retry_synthdefs(rt, mgr._send_osc_raw)

        # "SynthDef pads not found" IS a real miss → re-send per round.
        assert rt.SynthDefs["pads"].add.call_count >= 1


# ---------------------------------------------------------------------------
# MusicManager — sample buffer prewarm (regression 13.08.2026)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSampleBufferPrewarm:
    """Regression: play("x-o-") стартовал раньше, чем scsynth дочитал
    сэмпл в буфер → "Buffer UGen: no buffer data" + резкий свист на
    старте музыки. Предзагрузка должна дёргать Samples.getBufferFromSymbol
    для каждого символа (кроме пробелов/пауз) ДО exec."""

    def test_prewarm_loads_symbols_and_skips_rests(self):
        from rob_box_mcp_tools.tools import music as music_mod

        calls: list[str] = []

        class _FakeSamples:
            def getBufferFromSymbol(self, symbol, spack, index=0):
                calls.append((symbol, spack))
                return f"buf-{symbol}"

        mgr = _make_manager()
        mgr._renardo_context = {"Samples": _FakeSamples()}

        mgr._prewarm_sample_buffers('d1 >> play("x-o-", dur=0.5, sample=1)')

        assert [c[0] for c in calls] == ["x", "o"], (
            f"должны грузиться только символы x и o, получено: {calls!r}"
        )
        assert all(spack == 0 for _, spack in calls)

    def test_prewarm_ignores_missing_samples_manager(self):
        mgr = _make_manager()
        mgr._renardo_context = {}
        # Не должно падать при отсутствии Samples в контексте.
        mgr._prewarm_sample_buffers('d1 >> play("x-o-")')

    def test_prewarm_survives_broken_samples_manager(self):
        class _Broken:
            def getBufferFromSymbol(self, symbol, spack, index=0):
                raise RuntimeError("no such sample")

        mgr = _make_manager()
        mgr._renardo_context = {"Samples": _Broken()}
        # Ошибки менеджера не должны ронять предзагрузку.
        mgr._prewarm_sample_buffers('d1 >> play("x-o-")')


# ---------------------------------------------------------------------------
# MusicManager — execute_code
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerExecuteCode:
    """Тесты выполнения кода."""

    def test_execute_fails_if_sc_not_running(self):
        mgr = _make_manager(sc_running=False, renardo_available=True)
        result = mgr.execute_code("p1 >> pluck([0])")
        assert result["success"] is False
        assert "SuperCollider" in result["error"]

    def test_execute_fails_if_renardo_not_available(self):
        mgr = _make_manager(sc_running=True, renardo_available=False)
        # Must mock _ensure_renardo_available — the real method calls
        # _initialize_renardo() which may succeed on machines where
        # Renardo is installed, overriding the manual False setting.
        mgr._ensure_renardo_available = Mock(return_value=False)
        result = mgr.execute_code("p1 >> pluck([0])")
        assert result["success"] is False
        assert "Renardo" in result["error"]

    def test_execute_retries_renardo_initialization_before_failing(self):
        mgr = _make_manager(sc_running=True, renardo_available=False)

        def _recover() -> None:
            mgr._renardo_available = True
            mgr._renardo_context = {}

        mgr._initialize_renardo = Mock(side_effect=_recover)

        with patch("builtins.exec"):
            result = mgr.execute_code("Clock.bpm = 83")

        assert result["success"] is True
        mgr._initialize_renardo.assert_called_once()

    def test_execute_fails_if_code_is_dangerous(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        result = mgr.execute_code("import os; os.system('ls')")
        assert result["success"] is False
        assert "Запрещённый" in result["error"]

    def test_execute_success_stores_pattern_in_history(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            result = mgr.execute_code("p1 >> pluck([0, 2])", pattern_name="p1")
        assert result["success"] is True
        assert "p1" in mgr._pattern_history
        assert "p1" in mgr._active_patterns

    def test_execute_success_without_pattern_name(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            result = mgr.execute_code("Clock.bpm = 120")
        assert result["success"] is True
        assert len(mgr._pattern_history) == 0

    def test_execute_returns_code_in_result(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        code = "p2 >> bass([0, -2])"
        with patch("builtins.exec"):
            result = mgr.execute_code(code, pattern_name="p2")
        assert result["code"] == code

    def test_execute_runtime_error_returns_failure(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec", side_effect=RuntimeError("boom")):
            result = mgr.execute_code("p1 >> bad()")
        assert result["success"] is False
        assert "boom" in result["error"]

    # ----- Issue #990 — segments contract ---------------------------------

    def test_execute_with_segments_injects_total_beats_and_schedules_deadline(self):
        """``segments`` (bars) → __total_beats = segments*4 + deadline set."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._renardo_context["Clock"] = SimpleNamespace(bpm=110)
        with patch("builtins.exec"):
            result = mgr.execute_code("p1 >> pluck([0])", segments=16)
        assert result["success"] is True
        assert mgr._renardo_context["__total_segments"] == 16
        assert mgr._renardo_context["__total_beats"] == 64  # 16 bars * 4 beats
        assert mgr._renardo_context["__bpm"] == 110
        assert mgr._renardo_context["__bar_duration"] == pytest.approx(4 * 60.0 / 110.0)
        # 16 bars @110bpm = 34.9s → deadline in the future
        assert mgr._music_deadline_at is not None
        assert mgr._music_deadline_at > time.monotonic()
        assert mgr._music_deadline_segments == 16

    def test_execute_with_small_segments_respects_deadline_floor(self):
        """A tiny segments guess must not cut a song off after ~1s (#990)."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", segments=1)
        # 1 bar @120bpm = 2s, but the floor is 15s
        remaining = mgr._music_deadline_at - time.monotonic()
        assert remaining >= MusicManager.MIN_SEGMENTS_DEADLINE_SECONDS - 0.5

    def test_execute_with_segments_clamps_absurd_values(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", segments=10_000)
        assert mgr._renardo_context["__total_segments"] == MusicManager.MAX_SEGMENTS

    def test_execute_duration_sec_is_deprecated_and_does_not_schedule_stop(self):
        """Backward compat: duration_sec is clamped, never schedules a stop."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._renardo_context["Clock"] = SimpleNamespace(bpm=110)
        with patch("builtins.exec"):
            result = mgr.execute_code("p1 >> pluck([0])", duration_sec=6.0)
        assert result["success"] is True
        # clamped up so legacy Clock.future(__total_beats) cannot cut early
        assert mgr._renardo_context["__duration_sec"] == pytest.approx(
            MusicManager.DEPRECATED_DURATION_SEC_CLAMP
        )
        # __total_beats derives from the CLAMPED duration, not the LLM guess
        expected_beats = (MusicManager.DEPRECATED_DURATION_SEC_CLAMP * 110.0) / 60.0
        assert mgr._renardo_context["__total_beats"] == pytest.approx(expected_beats)
        # No safety-net deadline from duration_sec
        assert mgr._music_deadline_at is None
        assert mgr._music_deadline_segments is None

    def test_execute_without_segments_keeps_previous_deadline(self):
        """A follow-up pattern change without segments keeps the backstop."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", segments=16)
        deadline_before = mgr._music_deadline_at
        with patch("builtins.exec"):
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        assert mgr._music_deadline_at == deadline_before

    # ----- Clock.clear() OSC race-fix (issue #778) -----------------------

    def test_execute_with_clock_clear_sends_g_freeAll_then_g_new(self):
        """When code contains Clock.clear(), /g_freeAll MUST be sent
        BEFORE /g_new, so Group 1 is freed before being recreated.

        Issue #778: scsynth logged ``FAILURE IN SERVER /g_new negative
        node IDs are reserved`` because the new /g_new arrived before
        the previous Group 1 was actually freed by /g_freeAll.
        """
        mgr = _make_manager(sc_running=True, renardo_available=True)
        osc_calls = []

        def _capture_send(address, *args):
            osc_calls.append(address)

        with patch("builtins.exec"), patch.object(mgr, "_send_osc_raw", side_effect=_capture_send):
            mgr.execute_code("Clock.clear()\np1 >> pluck([0])", pattern_name="p1")

        # Both messages must be sent
        assert "/g_freeAll" in osc_calls
        assert "/g_new" in osc_calls
        # Order is critical: free first, recreate second
        assert osc_calls.index("/g_freeAll") < osc_calls.index("/g_new")

    def test_execute_with_clock_clear_sleeps_between_free_and_new(self):
        """A 50ms pause MUST sit between /g_freeAll and /g_new so that
        scsynth (UDP fire-and-forget) has time to actually free Group 1
        before we ask it to recreate the group with the same ID.
        """
        mgr = _make_manager(sc_running=True, renardo_available=True)
        call_log = []

        # Mock _send_osc_raw to record the order
        def _fake_send(address, *args):
            call_log.append(("osc", address))

        # Mock time.sleep to record sleep calls AND ensure they happen
        # between the two OSC sends (in the actual code path).
        def _fake_sleep(seconds):
            call_log.append(("sleep", seconds))

        with patch("builtins.exec"), patch.object(
            mgr, "_send_osc_raw", side_effect=_fake_send
        ), patch("rob_box_mcp_tools.tools.music.time.sleep", side_effect=_fake_sleep):
            mgr.execute_code("Clock.clear()\np1 >> pluck([0])", pattern_name="p1")

        # Find indices of the two OSC calls and the sleep between them
        try:
            idx_free = next(i for i, c in enumerate(call_log) if c == ("osc", "/g_freeAll"))
            idx_new = next(i for i, c in enumerate(call_log) if c == ("osc", "/g_new"))
        except StopIteration:
            pytest.fail("Expected /g_freeAll and /g_new OSC sends, got: " + str(call_log))

        # At least one sleep must occur between free and new, with positive duration
        sleeps_between = [
            s for s in call_log[idx_free + 1:idx_new] if s[0] == "sleep"
        ]
        assert sleeps_between, (
            "Race condition: no sleep between /g_freeAll and /g_new. "
            "Issue #778 will recur. Calls: " + str(call_log)
        )
        # And the sleep must be at least 0.05s (50ms is what we picked)
        assert any(s[1] >= 0.05 for s in sleeps_between), (
            "Sleep too short to let scsynth free Group 1: " + str(sleeps_between)
        )

    def test_execute_without_clock_clear_does_not_send_g_freeAll(self):
        """The OSC dance is gated on ``Clock.clear()`` in the code — no
        Clock.clear() → no /g_freeAll + /g_new sequence (would kill live
        music for a pattern-update that doesn't ask for it).
        """
        mgr = _make_manager(sc_running=True, renardo_available=True)
        osc_calls = []

        def _capture_send(address, *args):
            osc_calls.append(address)

        with patch("builtins.exec"), patch.object(mgr, "_send_osc_raw", side_effect=_capture_send):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")

        assert "/g_freeAll" not in osc_calls
        assert "/g_new" not in osc_calls


# ---------------------------------------------------------------------------
# MusicManager — stop_pattern / stop_all
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerStop:
    """Тесты остановки паттернов."""

    def test_stop_unknown_pattern_succeeds_without_sc(self):
        """stop_pattern always succeeds (discards from active set) even for unknown patterns."""
        mgr = _make_manager()
        result = mgr.stop_pattern("unknown")
        assert result["success"] is True

    def test_stop_known_pattern_removes_from_active(self):
        mgr = _make_manager(sc_running=False, renardo_available=False)
        mgr._pattern_history["p1"] = "p1 >> pluck([])"
        mgr._active_patterns.add("p1")
        result = mgr.stop_pattern("p1")
        assert result["success"] is True
        assert "p1" not in mgr._active_patterns

    def test_stop_pattern_with_sc_and_renardo(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._pattern_history["p1"] = "p1 >> pluck([])"
        mgr._active_patterns.add("p1")
        with patch("builtins.exec"):
            result = mgr.stop_pattern("p1")
        assert result["success"] is True
        assert "p1" not in mgr._active_patterns

    def test_stop_all_clears_active_patterns(self):
        mgr = _make_manager(sc_running=False, renardo_available=False)
        mgr._active_patterns = {"p1", "p2", "p3"}
        result = mgr.stop_all()
        assert result["success"] is True
        assert len(mgr._active_patterns) == 0

    def test_stop_all_with_sc_and_renardo(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._active_patterns = {"p1", "p2"}
        with patch("builtins.exec"):
            result = mgr.stop_all()
        assert result["success"] is True
        assert len(mgr._active_patterns) == 0


# ---------------------------------------------------------------------------
# MusicManager — set_vibe_preset
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerPreset:
    """Тесты вайб-пресетов."""

    @pytest.mark.parametrize("preset", list(MusicManager.VIBE_PRESETS.keys()))
    def test_set_valid_preset_without_sc(self, preset):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset(preset)
        assert result["success"] is True
        assert mgr._current_preset == preset
        assert result["preset"] == MusicManager.VIBE_PRESETS[preset]

    def test_set_invalid_preset_fails(self):
        mgr = _make_manager()
        result = mgr.set_vibe_preset("nonexistent")
        assert result["success"] is False
        assert "nonexistent" in result["error"]

    def test_set_preset_with_sc_applies_renardo_code(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec") as mock_exec:
            result = mgr.set_vibe_preset("chill")
        assert result["success"] is True
        mock_exec.assert_called_once()
        executed_code = mock_exec.call_args[0][0]
        assert "Scale.default" in executed_code
        assert "Clock.bpm" in executed_code

    def test_set_preset_without_sc_remembers_name(self):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset("energetic")
        assert result["success"] is True
        assert mgr._current_preset == "energetic"

    def test_chill_preset_values(self):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset("chill")
        assert result["preset"]["scale"] == "major"
        assert result["preset"]["bpm"] == 85
        assert result["preset"]["root"] == 0  # C = 0 semitones from C

    def test_dark_preset_values(self):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset("dark")
        assert result["preset"]["scale"] == "phrygian"
        assert result["preset"]["bpm"] == 100
        assert result["preset"]["root"] == 4  # E = 4 semitones from C

    @pytest.mark.parametrize("preset", ["rock", "latin", "electronic", "cinematic", "funk", "reggae", "classical"])
    def test_new_presets_execute_successfully(self, preset):
        mgr = _make_manager(sc_running=False)
        result = mgr.set_vibe_preset(preset)
        assert result["success"] is True
        assert mgr._current_preset == preset
        assert "scale" in result["preset"]
        assert "bpm" in result["preset"]
        assert "root" in result["preset"]


# ---------------------------------------------------------------------------
# MusicManager — get_state
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerGetState:
    """Тесты получения состояния."""

    def test_get_state_returns_all_fields(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._pattern_history = {"p1": "code1"}
        mgr._active_patterns = {"p1"}
        mgr._current_preset = "chill"
        state = mgr.get_state()
        assert state["renardo_available"] is True
        assert state["supercollider_running"] is True
        assert state["current_preset"] == "chill"
        assert "p1" in state["pattern_history"]
        assert "p1" in state["active_patterns"]

    def test_initial_state_is_empty(self):
        mgr = _make_manager()
        state = mgr.get_state()
        assert state["current_preset"] is None
        assert state["pattern_history"] == {}
        assert state["active_patterns"] == []

    def test_get_state_retries_renardo_initialization_when_sc_is_running(self):
        mgr = _make_manager(sc_running=True, renardo_available=False)

        def _recover() -> None:
            mgr._renardo_available = True
            mgr._renardo_context = {}

        mgr._initialize_renardo = Mock(side_effect=_recover)

        state = mgr.get_state()

        assert state["renardo_available"] is True
        mgr._initialize_renardo.assert_called_once()


# ---------------------------------------------------------------------------
# MusicManager — degraded sclang runtime (issue G-MUSIC)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerDegradedStack:
    """Tests for the G-MUSIC fail-fast behaviour when sclang reports
    syntax errors during startup. The acceptance criterion is: mcp_server
    must answer "music unavailable" instead of letting the LLM fight a
    broken Renardo stack.
    """

    @staticmethod
    def _mark_degraded(mgr, *, require_healthy: bool = True) -> None:
        """Simulate a degraded sclang startup log snapshot.

        Note: we leave ``_renardo_available`` alone unless the caller
        explicitly overrides it — that lets us test the degraded-guard
        independently from the renardo-init guard.
        """
        from rob_box_voice.core.music_stack_validation import MusicStackStatus

        mgr._music_stack_status = MusicStackStatus(
            is_healthy=False,
            oscdef_registered=True,
            missing_synths=("strings",),
            fatal_errors=(
                "ERROR: syntax error, unexpected '.', expecting '}'",
            ),
        )
        mgr._require_healthy = require_healthy

    def test_execute_code_is_blocked_when_stack_is_degraded(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        self._mark_degraded(mgr)

        with patch("builtins.exec") as exec_mock:
            result = mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")

        assert result["success"] is False
        assert "Музыка недоступна" in result["error"]
        assert "syntax error" in result["error"]
        # exec must not be called — we never want to talk to Renardo
        # while sclang itself is in degraded mode.
        exec_mock.assert_not_called()

    def test_set_vibe_preset_is_blocked_when_stack_is_degraded(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        self._mark_degraded(mgr)

        with patch("builtins.exec") as exec_mock:
            result = mgr.set_vibe_preset("chill")

        assert result["success"] is False
        assert "Музыка недоступна" in result["error"]
        assert "warning" in result
        exec_mock.assert_not_called()

    def test_stop_pattern_returns_unavailable_but_clears_active_set(self):
        """In degraded mode we still drop the pattern from active_patterns
        so the safety-net watchdog (issue #935) sees an empty session."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._active_patterns = {"p1"}
        self._mark_degraded(mgr)

        result = mgr.stop_pattern("p1")

        assert "p1" not in mgr._active_patterns
        assert result["success"] is False
        assert "degraded" in result["error"].lower() or "Музыка недоступна" in result["error"]

    def test_stop_all_returns_unavailable_but_clears_active_set(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._active_patterns = {"p1", "p2"}
        self._mark_degraded(mgr)

        result = mgr.stop_all()

        assert mgr._active_patterns == set()
        assert result["success"] is False
        assert "Музыка недоступна" in result["error"]

    def test_require_healthy_false_lets_degraded_stack_execute(self):
        """Operator opt-out: ROB_BOX_MUSIC_REQUIRE_HEALTHY=0 → degraded
        mode does NOT block the LLM from calling execute_code. This is
        useful for debugging a known-broken stack or running with a
        patched Renardo.
        """
        mgr = _make_manager(sc_running=True, renardo_available=True)
        self._mark_degraded(mgr, require_healthy=False)

        with patch("builtins.exec") as exec_mock:
            result = mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")

        # Guard explicitly disabled → degraded-guard does NOT short-circuit.
        # exec runs (mocked); we only care that the error message is NOT
        # the G-MUSIC one.
        assert result["success"] is True
        assert "Музыка недоступна" not in result.get("error", "")
        exec_mock.assert_called_once()

    def test_get_state_surfaces_music_stack_health(self):
        """The LLM can read get_music_state and see the failure reason."""
        mgr = _make_manager(sc_running=False, renardo_available=False)
        self._mark_degraded(mgr)

        state = mgr.get_state()

        assert state["music_stack_healthy"] is False
        assert state["music_stack_require_healthy"] is True
        assert any("syntax error" in err for err in state["music_stack_fatal_errors"])
        assert "strings" in state["music_stack_missing_synths"]

    def test_evaluate_music_stack_health_clears_renardo_when_degraded(self, tmp_path, monkeypatch):
        """Integration: end-to-end degraded snapshot from a real log file."""
        log_path = tmp_path / "sclang.log"
        log_path.write_text(
            "\n".join([
                "FoxDot OSCdef registered. Ready to compile SynthDefs.",
                "ERROR: syntax error, unexpected '.', expecting '}'",
                "ERROR: Command line parse failed",
            ]),
            encoding="utf-8",
        )
        monkeypatch.setenv("SCLANG_LOG_PATH", str(log_path))

        mgr = _make_manager(sc_running=True, renardo_available=True)
        status = mgr._evaluate_music_stack_health()

        assert status.is_healthy is False
        assert mgr._renardo_available is False
        assert not mgr.is_music_stack_healthy()

    def test_evaluate_music_stack_health_keeps_renardo_when_healthy(self, tmp_path, monkeypatch):
        # Narrow the critical-synths list so a 2-synth log counts as healthy.
        log_path = tmp_path / "sclang.log"
        log_path.write_text(
            "\n".join([
                "FoxDot OSCdef registered. Ready to compile SynthDefs.",
                "SynthDef preload ok: strings",
                "SynthDef preload ok: wobblebass",
            ]),
            encoding="utf-8",
        )
        monkeypatch.setenv("SCLANG_LOG_PATH", str(log_path))

        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._critical_synths = ("strings", "wobblebass")
        status = mgr._evaluate_music_stack_health()

        assert status.is_healthy is True
        # We don't promise renardo_available stays True — that flag is
        # the result of the heavier _initialize_renardo() flow — but
        # the degraded-mode path must not flip it to False.
        assert mgr._renardo_available is True


# ---------------------------------------------------------------------------
# ExecuteMusicCodeTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestExecuteMusicCodeTool:
    """Тесты MCPTool для выполнения кода."""

    def _make_tool(self, mock_node, **kwargs):
        mgr = _make_manager(**kwargs)
        return ExecuteMusicCodeTool(mock_node, mgr), mgr

    def test_tool_name(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.name == "execute_music_code"

    def test_tool_description_mentions_renardo(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert "Renardo" in tool.description

    def test_tool_has_code_parameter(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        param_names = [p.name for p in tool.parameters]
        assert "code" in param_names

    def test_tool_has_pattern_name_parameter(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        param_names = [p.name for p in tool.parameters]
        assert "pattern_name" in param_names

    def test_tool_has_segments_parameter(self, mock_node):
        """Issue #990: execute_music_code schema exposes ``segments``."""
        tool, _ = self._make_tool(mock_node)
        param_names = [p.name for p in tool.parameters]
        assert "segments" in param_names
        seg_param = next(p for p in tool.parameters if p.name == "segments")
        assert seg_param.type == "integer"
        assert seg_param.required is False

    def test_tool_has_duration_sec_parameter_deprecated(self, mock_node):
        """Issue #990: duration_sec stays in schema for backward compat only."""
        tool, _ = self._make_tool(mock_node)
        param_names = [p.name for p in tool.parameters]
        assert "duration_sec" in param_names
        dur_param = next(p for p in tool.parameters if p.name == "duration_sec")
        assert "DEPRECATED" in dur_param.description

    def test_execute_forwards_segments_to_manager(self, mock_node):
        """Issue #990: tool passes ``segments`` through to MusicManager."""
        tool, mgr = self._make_tool(mock_node, sc_running=True, renardo_available=True)
        mgr._renardo_context["Clock"] = SimpleNamespace(bpm=110)
        with patch("builtins.exec"), patch.object(mgr, "execute_code", wraps=mgr.execute_code) as spy:
            result = tool.execute(code="p1 >> pluck([0])", segments=16)
        assert result.success is True
        spy.assert_called_once()
        kwargs = spy.call_args
        # execute_code(code, pattern_name, *, segments, duration_sec)
        assert kwargs.args[0] == "p1 >> pluck([0])"
        assert kwargs.kwargs.get("segments") == 16
        assert mgr._renardo_context["__total_segments"] == 16

    def test_execute_success(self, mock_node):
        tool, mgr = self._make_tool(mock_node, sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            result = tool.execute(code="p1 >> pluck([0])", pattern_name="p1")
        assert result.success is True

    def test_execute_failure_propagates(self, mock_node):
        tool, _ = self._make_tool(mock_node, sc_running=False)
        result = tool.execute(code="p1 >> pluck([0])")
        assert result.success is False
        assert result.error is not None

    def test_execute_blocked_code(self, mock_node):
        tool, _ = self._make_tool(mock_node, sc_running=True, renardo_available=True)
        result = tool.execute(code="import os")
        assert result.success is False


# ---------------------------------------------------------------------------
# StopMusicTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestStopMusicTool:
    """Тесты MCPTool для остановки музыки."""

    def _make_tool(self, mock_node, **kwargs):
        mgr = _make_manager(**kwargs)
        return StopMusicTool(mock_node, mgr), mgr

    def test_tool_name(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.name == "stop_music"

    def test_stop_all_when_no_pattern_name(self, mock_node):
        tool, mgr = self._make_tool(mock_node)
        mgr._active_patterns = {"p1", "p2"}
        result = tool.execute()
        assert result.success is True
        assert len(mgr._active_patterns) == 0

    def test_stop_all_when_pattern_name_is_all(self, mock_node):
        tool, mgr = self._make_tool(mock_node)
        mgr._active_patterns = {"p1"}
        result = tool.execute(pattern_name="all")
        assert result.success is True
        assert len(mgr._active_patterns) == 0

    def test_stop_specific_pattern(self, mock_node):
        tool, mgr = self._make_tool(mock_node, sc_running=False, renardo_available=False)
        mgr._pattern_history["p1"] = "code"
        mgr._active_patterns.add("p1")
        result = tool.execute(pattern_name="p1")
        assert result.success is True
        assert "p1" not in mgr._active_patterns

    def test_stop_unknown_pattern_succeeds(self, mock_node):
        """stop_pattern always succeeds — LLM can stop any player name."""
        tool, _ = self._make_tool(mock_node)
        result = tool.execute(pattern_name="nonexistent")
        assert result.success is True


# ---------------------------------------------------------------------------
# SetVibePresetTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSetVibePresetTool:
    """Тесты MCPTool для вайб-пресетов."""

    def _make_tool(self, mock_node, **kwargs):
        mgr = _make_manager(**kwargs)
        return SetVibePresetTool(mock_node, mgr), mgr

    def test_tool_name(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.name == "set_vibe_preset"

    def test_preset_name_enum_contains_all_presets(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        param = next(p for p in tool.parameters if p.name == "preset_name")
        for preset_key in MusicManager.VIBE_PRESETS:
            assert preset_key in param.enum

    def test_execute_valid_preset(self, mock_node):
        tool, mgr = self._make_tool(mock_node, sc_running=False)
        result = tool.execute(preset_name="chill")
        assert result.success is True

    def test_execute_invalid_preset(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        result = tool.execute(preset_name="nonexistent")
        assert result.success is False

    @pytest.mark.parametrize("preset", list(MusicManager.VIBE_PRESETS.keys()))
    def test_all_presets_execute_successfully(self, mock_node, preset):
        tool, _ = self._make_tool(mock_node, sc_running=False)
        result = tool.execute(preset_name=preset)
        assert result.success is True


# ---------------------------------------------------------------------------
# GetMusicStateTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestGetMusicStateTool:
    """Тесты MCPTool для получения состояния музыки."""

    def _make_tool(self, mock_node, **kwargs):
        mgr = _make_manager(**kwargs)
        return GetMusicStateTool(mock_node, mgr), mgr

    def test_tool_name(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.name == "get_music_state"

    def test_tool_has_no_required_parameters(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert len(tool.parameters) == 0

    def test_tool_is_read_only(self, mock_node):
        tool, _ = self._make_tool(mock_node)
        assert tool.read_only is True

    def test_execute_returns_state(self, mock_node):
        tool, mgr = self._make_tool(mock_node, sc_running=False, renardo_available=False)
        mgr._pattern_history = {"bass": "code"}
        mgr._active_patterns = {"bass"}
        mgr._current_preset = "jazz"
        result = tool.execute()
        assert result.success is True
        assert result.data["current_preset"] == "jazz"
        assert "bass" in result.data["pattern_history"]

    def test_execute_message_contains_status(self, mock_node):
        tool, _ = self._make_tool(mock_node, sc_running=False)
        result = tool.execute()
        assert result.success is True
        assert "SuperCollider" in result.message
        assert "Renardo" in result.message


# ---------------------------------------------------------------------------
# MusicManager — music session lifecycle / safety-net (issue #935)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicSessionLifecycle:
    """Тесты session lifecycle + safety-net (issue #935):

    * ``get_state`` exposes lifecycle fields (timestamps, ttl, counter)
    * ``execute_code`` stamps ``_last_music_activity_at`` (and starts a session
      on the first pattern)
    * ``stop_pattern`` / ``stop_all`` reset / close the session correctly
    * ``auto_stop_idle_music`` no-ops when nothing is active, no-ops when the
      idle interval is below the configured TTL, and **does** call
      ``stop_all`` (incrementing the counter) once the TTL is exceeded
    * ``stop_music_on_session_end`` is idempotent and reports what was active
    """

    # ----- get_state lifecycle fields ---------------------------------------

    def test_get_state_exposes_lifecycle_fields(self):
        mgr = _make_manager(sc_running=True, renardo_available=False)
        state = mgr.get_state()
        # default-from-__init__ values
        assert state["music_session_active_since"] is None
        assert state["last_music_activity_at"] is None
        assert state["last_stop_at"] is None
        assert state["auto_stop_ttl_seconds"] == 300
        assert state["auto_stop_count"] == 0
        # nothing is active yet → no idle window
        assert state["idle_seconds"] is None
        assert state["active_patterns"] == []

    # ----- execute_code stamps lifecycle ------------------------------------

    def test_execute_code_stamps_last_activity(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        # No session yet → after execute_code, both timestamps populated.
        assert mgr._music_session_active_since is None
        assert mgr._last_music_activity_at is None
        with patch("builtins.exec"):
            result = mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        assert result["success"] is True
        assert mgr._music_session_active_since is not None
        assert mgr._last_music_activity_at is not None
        # session start == first activity (monotonic)
        assert (
            mgr._music_session_active_since <= mgr._last_music_activity_at
        )

    def test_execute_code_without_pattern_opens_session(self):
        """Any successful code execution (even without pattern_name) stamps
        lifecycle timestamps so safety nets can stop unnamed music (issue #935 fix)."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            result = mgr.execute_code("Clock.bpm = 120")
        assert result["success"] is True
        # Session IS opened — safety nets need the timestamps even for unnamed code.
        assert mgr._music_session_active_since is not None
        assert mgr._last_music_activity_at is not None
        # But _active_patterns stays empty (no pattern_name given).
        assert len(mgr._active_patterns) == 0

    def test_execute_code_continues_existing_session(self):
        """Second execute keeps session_open but bumps last_activity."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
            first_session = mgr._music_session_active_since
            first_activity = mgr._last_music_activity_at
            # Execute again — same session, but activity bumped.
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        assert mgr._music_session_active_since == first_session
        assert mgr._last_music_activity_at >= first_activity

    # ----- stop_all resets session ------------------------------------------

    def test_stop_all_resets_session(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        assert mgr._music_session_active_since is not None
        mgr.stop_all()
        assert mgr._music_session_active_since is None
        assert mgr._last_music_activity_at is None
        assert mgr._last_stop_at is not None

    def test_stop_pattern_partial_does_not_reset_session(self):
        """stop_pattern leaves lifecycle intact until ALL patterns are gone."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        session_before = mgr._music_session_active_since
        activity_before = mgr._last_music_activity_at
        mgr.stop_pattern("p1")
        # session & activity should still be set — p2 is still active
        assert mgr._music_session_active_since == session_before
        assert mgr._last_music_activity_at == activity_before
        assert "p1" not in mgr._active_patterns
        assert "p2" in mgr._active_patterns

    # ----- auto_stop_idle_music --------------------------------------------

    def test_auto_stop_is_noop_when_inactive(self):
        mgr = _make_manager()
        # Nothing was ever executed → _last_music_activity_at is None → no-op.
        result = mgr.auto_stop_idle_music()
        assert result["stopped"] is False
        assert result["active_patterns"] == []
        assert result["auto_stop_count"] == 0
        assert result["idle_seconds"] is None

    def test_auto_stop_works_with_unnamed_patterns(self):
        """Issue #935 regression: auto_stop must work even when the LLM
        executed music code without a pattern_name (active_patterns empty)."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        # Simulate: music was executed, but without pattern_name.
        mgr._last_music_activity_at = 100.0
        mgr._music_session_active_since = 100.0
        # _active_patterns is empty — the exact bug scenario.
        assert len(mgr._active_patterns) == 0
        # Idle for 400 s, TTL=300 s → should auto-stop.
        result = mgr.auto_stop_idle_music(now=500.0)
        assert result["stopped"] is True
        assert result["auto_stop_count"] == 1

    def test_stop_music_on_session_end_always_calls_stop_all(self):
        """Issue #935 regression: stop_music_on_session_end must call stop_all
        even when _active_patterns is empty (unnamed patterns)."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        # Simulate: music was executed without pattern_name.
        mgr._last_music_activity_at = 100.0
        mgr._music_session_active_since = 100.0
        assert len(mgr._active_patterns) == 0
        # Should still report was_active=True and call stop_all().
        result = mgr.stop_music_on_session_end()
        assert result["was_active"] is True
        # After stop_all, session is reset.
        assert mgr._music_session_active_since is None
        assert mgr._last_music_activity_at is None
        assert mgr._last_stop_at is not None

    def test_auto_stop_is_noop_when_within_ttl(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        # 10 s idle, ttl 300 s → no auto-stop.
        result = mgr.auto_stop_idle_music(ttl_seconds=300, now=time.monotonic() + 10)
        assert result["stopped"] is False
        assert mgr._auto_stop_count == 0

    def test_auto_stop_stops_when_ttl_exceeded(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        assert "p1" in mgr._active_patterns
        # Inject a future "now" so idle = ttl+10.
        result = mgr.auto_stop_idle_music(
            ttl_seconds=300, now=time.monotonic() + 310
        )
        assert result["stopped"] is True
        assert mgr._auto_stop_count == 1
        assert mgr._active_patterns == set()
        assert mgr._music_session_active_since is None

    def test_auto_stop_count_increments_on_repeat(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        # Round 1: trigger an auto-stop, then re-arm by adding a pattern.
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        now_offset = 0.0
        t0 = time.monotonic() + now_offset
        mgr._last_music_activity_at = t0
        result1 = mgr.auto_stop_idle_music(ttl_seconds=1, now=t0 + 10)
        assert result1["stopped"] is True
        assert mgr._auto_stop_count == 1
        # Round 2: re-create a session and trigger again.
        with patch("builtins.exec"):
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        t1 = time.monotonic()
        result2 = mgr.auto_stop_idle_music(ttl_seconds=1, now=t1 + 10)
        assert result2["stopped"] is True
        assert mgr._auto_stop_count == 2

    def test_auto_stop_default_ttl_matches_env_or_300(self):
        """The default TTL constant must be 300 seconds when env unset."""
        mgr = _make_manager()
        mgr._auto_stop_ttl_seconds = 300
        assert mgr._auto_stop_ttl_seconds == 300

    def test_auto_stop_returns_diagnostic_fields(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        result = mgr.auto_stop_idle_music(
            ttl_seconds=300, now=time.monotonic() + 999
        )
        # Returning all keys makes it easy for DialogueCore to log without
        # touching internals.
        for key in (
            "stopped", "idle_seconds", "ttl_seconds",
            "active_patterns", "auto_stop_count", "stop_result",
        ):
            assert key in result, f"missing key: {key}"
        assert result["stopped"] is True
        assert isinstance(result["idle_seconds"], float)
        assert result["idle_seconds"] >= 300.0

    # ----- Issue #990 — segments safety-net deadline -----------------------

    def test_auto_stop_fires_at_segments_deadline(self):
        """If the TTS batch never completes, music stops at the deadline."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1", segments=16)
        assert mgr._music_deadline_at is not None
        # Simulate the watchdog firing after the deadline (idle < TTL).
        result = mgr.auto_stop_idle_music(
            ttl_seconds=300, now=mgr._music_deadline_at + 1
        )
        assert result["stopped"] is True
        assert result.get("stop_reason") == "segments_deadline"
        assert mgr._auto_stop_count == 1
        # stop_all cleared the deadline.
        assert mgr._music_deadline_at is None
        assert mgr._music_deadline_segments is None

    def test_auto_stop_noop_before_segments_deadline(self):
        """Before the deadline, the segments backstop must NOT fire."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1", segments=16)
        deadline = mgr._music_deadline_at
        # now is just BEFORE the deadline and idle is within the high TTL.
        result = mgr.auto_stop_idle_music(ttl_seconds=999, now=deadline - 1)
        assert result["stopped"] is False
        assert result.get("stop_reason") is None

    def test_auto_stop_uses_deadline_before_idle_ttl(self):
        """The deadline takes priority over the idle TTL (#990)."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1", segments=16)
        now = mgr._music_deadline_at + 0.5
        result = mgr.auto_stop_idle_music(ttl_seconds=999, now=now)
        assert result["stopped"] is True
        assert result.get("stop_reason") == "segments_deadline"

    def test_stop_all_clears_segments_deadline(self):
        """tts_batch_complete → stop_all must cancel the backstop."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1", segments=16)
        assert mgr._music_deadline_at is not None
        mgr.stop_all()
        assert mgr._music_deadline_at is None
        assert mgr._music_deadline_segments is None

    def test_get_state_exposes_segments_deadline(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1", segments=8)
        state = mgr.get_state()
        assert state["music_deadline_segments"] == 8
        assert state["music_deadline_at"] == mgr._music_deadline_at

    # ----- stop_music_on_session_end ---------------------------------------

    def test_session_end_hook_noop_when_silent(self):
        mgr = _make_manager()
        result = mgr.stop_music_on_session_end()
        assert result["was_active"] is False
        assert result["stopped_patterns"] == []
        # Always calls stop_all (prophylactic), message reflects that.
        assert "профилактически" in result["message"]

    def test_session_end_hook_stops_active_music(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        result = mgr.stop_music_on_session_end()
        assert result["was_active"] is True
        assert set(result["stopped_patterns"]) == {"p1", "p2"}
        # Message describes auto-stop (mixed RU/EN keywords).
        msg_lower = result["message"].lower()
        assert "stop_music" in msg_lower or "стоп" in msg_lower
        # Lifecycle is fully closed.
        assert mgr._active_patterns == set()
        assert mgr._music_session_active_since is None
        assert mgr._last_stop_at is not None

    def test_session_end_hook_is_idempotent(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        first = mgr.stop_music_on_session_end()
        second = mgr.stop_music_on_session_end()
        assert first["was_active"] is True
        # Second call: session already closed, but stop_all still called.
        assert second["was_active"] is False
        assert second["stopped_patterns"] == []
        assert "профилактически" in second["message"]


# ---------------------------------------------------------------------------
# Issue #1016 — music-quality guardrail (dramaturgy validator)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicQualityValidator:
    """Валидатор музыкального качества (issue #1016).

    Отделен от ``_filter_code`` (безопасность): этот валидатор ловит
    *музыкальные* ошибки LLM — абсолютные частоты, отсутствие ``dur=``
    и статичные лупы без развития. errors блокируют выполнение,
    warnings только добавляются в message.
    """

    def setup_method(self):
        self.mgr = _make_manager()

    # ----- absolute frequencies (hard errors) ------------------------------

    def test_freq_kwarg_is_rejected(self):
        errors, warnings = self.mgr._validate_music_code(
            "p1 >> pluck(freq=440, dur=1)"
        )
        assert errors, "freq=440 must be a hard error"
        assert "частот" in errors[0]

    def test_hz_kwarg_is_rejected(self):
        errors, warnings = self.mgr._validate_music_code(
            "p1 >> pluck(hz=220, dur=1)"
        )
        assert errors

    def test_midinote_kwarg_is_rejected(self):
        errors, warnings = self.mgr._validate_music_code(
            "p1 >> pluck(midinote=69, dur=1)"
        )
        assert errors

    def test_scale_degrees_pass(self):
        errors, warnings = self.mgr._validate_music_code(
            "p1 >> pluck([0,4,7], dur=0.5)"
        )
        assert errors == []

    def test_execute_code_blocks_absolute_frequency(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec") as exec_mock:
            result = mgr.execute_code("p1 >> pluck(freq=440, dur=1)")
        assert result["success"] is False
        assert "валидатор" in result["error"]
        exec_mock.assert_not_called()  # код не ушёл в Renardo

    # ----- dur= warnings ---------------------------------------------------

    def test_missing_dur_warns_but_passes(self):
        errors, warnings = self.mgr._validate_music_code(
            "p1 >> pluck([0,2,4])"
        )
        assert errors == []
        assert any("dur" in w for w in warnings)

    def test_play_without_dur_is_ok(self):
        # play() имеет собственный dur из строки паттерна — не нудим.
        errors, warnings = self.mgr._validate_music_code(
            'd1 >> play("x-o-", sample=1, amp=0.2)'
        )
        assert errors == []
        assert warnings == []

    def test_missing_dur_warning_surfaces_in_execute_result(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            result = mgr.execute_code("p1 >> pluck([0,2,4])")
        assert result["success"] is True
        assert "dur" in result["message"]

    # ----- static-loop warning (developing patterns) -----------------------

    def test_static_loop_without_dev_pattern_warns(self):
        errors, warnings = self.mgr._validate_music_code(
            "p1 >> pluck([0,2,4], dur=0.5)\n"
            "p2 >> bass([0,-2], dur=1)"
        )
        assert errors == []
        assert any("статичн" in w for w in warnings)

    def test_dev_pattern_suppresses_static_loop_warning(self):
        errors, warnings = self.mgr._validate_music_code(
            "p1 >> pluck([0,2,4], dur=0.5).every(4, 'stutter')\n"
            "p2 >> bass([0,-2], dur=1)"
        )
        assert errors == []
        assert not any("статичн" in w for w in warnings)

    def test_pvar_suppresses_static_loop_warning(self):
        errors, warnings = self.mgr._validate_music_code(
            "current_chord = Pvar([[0,2,4],[3,5,7]], 8)\n"
            "p1 >> pad(current_chord, dur=4)\n"
            "p2 >> dub([0,-2], dur=2)"
        )
        assert errors == []
        assert not any("статичн" in w for w in warnings)

    # ----- Clock.future structures from the library ------------------------

    def test_bootstrap_track_passes_validator(self):
        """csm_132_full_track (Intro→Verse→Chorus→Outro через Clock.future)
        не должен блокироваться валидатором и не должен нудить про
        статичный луп — это эталон драматургии (issue #1016)."""
        code = (
            "Clock.bpm = 132\n"
            "Scale.default = 'minor'\n"
            "Root.default = 'C#'\n"
            "def intro():\n"
            "    d1 >> play('X...', sample=1, amp=0.2)\n"
            "    p1 >> pads([0,4,5,3], dur=8, amp=0.15)\n"
            "    Clock.future(16, verse)\n"
            "def verse():\n"
            "    p2 >> dub([0,-2,0,-3], dur=2, oct=3, amp=0.3)\n"
            "    p3 >> blip([0,2,4,7,4,2,0,-2], dur=0.5, amp=0.5)\n"
            "    Clock.future(32, chorus)\n"
            "def chorus():\n"
            "    d2 >> play('--.-', sample=3, amp=0.15)\n"
            "    p1 >> pads([0,4,7,4], dur=2, amp=0.25)\n"
            "    Clock.future(32, bridge)\n"
            "def bridge():\n"
            "    d2.stop()\n"
            "    p3.stop()\n"
            "    Clock.future(16, outro)\n"
            "def outro():\n"
            "    p2.stop()\n"
            "    Clock.future(16, lambda: Clock.clear())\n"
            "intro()"
        )
        errors, warnings = self.mgr._validate_music_code(code)
        assert errors == []
        # Play-плееры (d1/d2) без dur= — это ок; synth-плееры с dur=.
        assert not any("статичн" in w for w in warnings)

    def test_filter_code_does_not_block_clock_future(self):
        """_filter_code (безопасность) не должен ломать Clock.future —
        это требование acceptance (bootstrap-трек использует
        Clock.future(16, verse) для смены секций)."""
        ok, err = self.mgr._filter_code(
            "def verse():\n"
            "    p1 >> pads([0,4,5,3], dur=8)\n"
            "    Clock.future(16, verse)"
        )
        assert ok is True
        assert err == ""


# ---------------------------------------------------------------------------
# SaveTrackTool / ListTracksTool / LoadTrackTool / DeleteTrackTool
# ---------------------------------------------------------------------------


class _FakeLibrary:
    """Минимальный фейк TrackLibrary для tool-тестов (без SQLite)."""

    def __init__(self, tracks=None):
        self.tracks = tracks or {}
        self.save_calls = []
        self.delete_calls = []

    def save_track(self, name, code, title="", description="", tags=None, rating=0, notes=""):
        self.save_calls.append((name, code))
        if name == "dup":
            return {"success": False, "error": "Трек с таким именем уже существует"}
        self.tracks[name] = {
            "name": name, "code": code, "title": title, "description": description,
            "tags": tags or [], "rating": rating, "notes": notes, "play_count": 0,
        }
        return {"success": True, "message": f"Трек '{name}' сохранён", "track": self.tracks[name]}

    def list_tracks(self, tag=None, min_rating=0):
        tracks = list(self.tracks.values())
        if tag:
            tracks = [t for t in tracks if tag in t["tags"]]
        if min_rating:
            tracks = [t for t in tracks if t["rating"] >= min_rating]
        return {"tracks": tracks, "total": len(tracks)}

    def load_track(self, name):
        if name not in self.tracks:
            return {"success": False, "error": f"Трек '{name}' не найден"}
        track = dict(self.tracks[name])
        track["play_count"] = track.get("play_count", 0) + 1
        self.tracks[name] = track
        return {"success": True, "code": track["code"], "track": track}

    def delete_track(self, name):
        self.delete_calls.append(name)
        if name not in self.tracks:
            return {"success": False, "error": f"Трек '{name}' не найден"}
        del self.tracks[name]
        return {"success": True, "message": f"Трек '{name}' удалён"}


@pytest.mark.unit
class TestSaveTrackTool:
    def _make_tool(self, mock_node, library=None, mgr=None):
        from rob_box_mcp_tools.tools.music import SaveTrackTool as _SaveTrackTool
        lib = library or _FakeLibrary()
        manager = mgr or _make_manager()
        return _SaveTrackTool(mock_node, lib, manager), lib, manager

    def test_tool_name(self, mock_node):
        tool, _, _ = self._make_tool(mock_node)
        assert tool.name == "save_track"
        assert tool.execution_type.value == "fast"
        assert tool.destructive is False

    def test_execute_with_code(self, mock_node):
        tool, lib, _ = self._make_tool(mock_node)
        result = tool.execute(name="my_track", code="p1 >> pluck([0])")
        assert result.success is True
        assert lib.save_calls == [("my_track", "p1 >> pluck([0])")]

    def test_execute_without_code_uses_history(self, mock_node):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._pattern_history["full_track"] = "p1 >> pluck([0, 2])"
        tool, lib, _ = self._make_tool(mock_node, mgr=mgr)
        result = tool.execute(name="my_track")
        assert result.success is True
        assert lib.save_calls == [("my_track", "p1 >> pluck([0, 2])")]

    def test_execute_without_code_and_empty_history(self, mock_node):
        tool, _, _ = self._make_tool(mock_node)
        result = tool.execute(name="my_track")
        assert result.success is False
        assert "история паттернов пуста" in result.error

    def test_execute_library_error(self, mock_node):
        lib = _FakeLibrary()
        tool, _, _ = self._make_tool(mock_node, library=lib)
        result = tool.execute(name="dup", code="p1 >> pluck([0])")
        assert result.success is False
        assert "уже существует" in result.error


@pytest.mark.unit
class TestListTracksTool:
    def _make_tool(self, mock_node, library=None):
        from rob_box_mcp_tools.tools.music import ListTracksTool as _ListTracksTool
        return _ListTracksTool(mock_node, library or _FakeLibrary())

    def test_tool_name(self, mock_node):
        tool = self._make_tool(mock_node)
        assert tool.name == "list_tracks"
        assert tool.read_only is True

    def test_execute_empty(self, mock_node):
        tool = self._make_tool(mock_node)
        result = tool.execute()
        assert result.success is True
        assert "Медиатека пуста" in result.message

    def test_execute_with_tracks(self, mock_node):
        lib = _FakeLibrary()
        lib.save_track("a1", "code1", title="Alpha", tags=["chill"], rating=4)
        lib.save_track("b2", "code2", title="Beta", tags=["rock"], rating=2)
        tool = self._make_tool(mock_node, library=lib)

        result = tool.execute()

        assert result.success is True
        assert result.data["total"] == 2
        assert "Alpha" in result.message
        assert "Beta" in result.message

    def test_execute_filter_by_tag(self, mock_node):
        lib = _FakeLibrary()
        lib.save_track("a1", "code1", title="Alpha", tags=["chill"], rating=4)
        lib.save_track("b2", "code2", title="Beta", tags=["rock"], rating=2)
        tool = self._make_tool(mock_node, library=lib)

        result = tool.execute(tag="chill")

        assert result.data["total"] == 1
        assert "Alpha" in result.message
        assert "Beta" not in result.message

    def test_execute_filter_by_rating(self, mock_node):
        lib = _FakeLibrary()
        lib.save_track("a1", "code1", title="Alpha", rating=4)
        lib.save_track("b2", "code2", title="Beta", rating=2)
        tool = self._make_tool(mock_node, library=lib)

        result = tool.execute(min_rating=3)

        assert result.data["total"] == 1
        assert "Alpha" in result.message


@pytest.mark.unit
class TestLoadTrackTool:
    def _make_tool(self, mock_node, library=None, mgr=None):
        from rob_box_mcp_tools.tools.music import LoadTrackTool as _LoadTrackTool
        return _LoadTrackTool(mock_node, library or _FakeLibrary(), mgr or _make_manager())

    def test_tool_name(self, mock_node):
        tool = self._make_tool(mock_node)
        assert tool.name == "load_track"

    def test_execute_track_not_found(self, mock_node):
        tool = self._make_tool(mock_node)
        result = tool.execute(name="missing")
        assert result.success is False
        assert "не найден" in result.error

    def test_execute_success(self, mock_node):
        lib = _FakeLibrary()
        lib.save_track("a1", "p1 >> pluck([0])", title="Alpha")
        mgr = _make_manager(sc_running=True, renardo_available=True)
        tool = self._make_tool(mock_node, library=lib, mgr=mgr)

        with patch("builtins.exec"):
            result = tool.execute(name="a1")

        assert result.success is True
        assert "Alpha" in result.message
        assert "сыграно" in result.message

    def test_execute_play_failure(self, mock_node):
        lib = _FakeLibrary()
        lib.save_track("a1", "p1 >> pluck([0])", title="Alpha")
        mgr = _make_manager(sc_running=False, renardo_available=False)  # SC не запущен
        tool = self._make_tool(mock_node, library=lib, mgr=mgr)

        result = tool.execute(name="a1")

        assert result.success is False
        assert "не запущен" in result.error


@pytest.mark.unit
class TestDeleteTrackTool:
    def _make_tool(self, mock_node, library=None):
        from rob_box_mcp_tools.tools.music import DeleteTrackTool as _DeleteTrackTool
        return _DeleteTrackTool(mock_node, library or _FakeLibrary())

    def test_tool_name(self, mock_node):
        tool = self._make_tool(mock_node)
        assert tool.name == "delete_track"
        assert tool.destructive is True

    def test_execute_success(self, mock_node):
        lib = _FakeLibrary()
        lib.save_track("a1", "code1")
        tool = self._make_tool(mock_node, library=lib)

        result = tool.execute(name="a1")

        assert result.success is True
        assert lib.delete_calls == ["a1"]
        assert "удалён" in result.message

    def test_execute_missing(self, mock_node):
        tool = self._make_tool(mock_node)
        result = tool.execute(name="nope")
        assert result.success is False
        assert "не найден" in result.error


# ---------------------------------------------------------------------------
# SearchSamplesTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSearchSamplesTool:
    def test_tool_name(self, mock_node):
        tool = SearchSamplesTool(mock_node)
        assert tool.name == "search_samples"
        assert tool.read_only is True
        assert tool.destructive is False

    @patch("rob_box_voice.core.sample_search.search_renardo_samples")
    def test_execute_error(self, mock_search, mock_node):
        mock_search.return_value = {"error": "pack not found", "hint": "hint", "available_packs": ["0_foxdot_default"]}
        tool = SearchSamplesTool(mock_node)

        result = tool.execute(query="kick")

        assert result.success is False
        assert "pack not found" in result.error
        assert "0_foxdot_default" in result.error

    @patch("rob_box_voice.core.sample_search.search_renardo_samples")
    def test_execute_letters_overview(self, mock_search, mock_node):
        mock_search.return_value = {"letters": {"a": 10, "b": 5}, "total_samples": 15}
        tool = SearchSamplesTool(mock_node)

        result = tool.execute(query="*")

        assert result.success is True
        assert result.data["letters"]["a"] == 10
        assert "2 букв" in result.message

    @patch("rob_box_voice.core.sample_search.search_renardo_samples")
    def test_execute_no_results(self, mock_search, mock_node):
        mock_search.return_value = {"found": 0, "results": []}
        tool = SearchSamplesTool(mock_node)

        result = tool.execute(query="xyz")

        assert result.success is True
        assert "ничего не найдено" in result.message

    @patch("rob_box_voice.core.sample_search.search_renardo_samples")
    def test_execute_with_results(self, mock_search, mock_node):
        mock_search.return_value = {
            "found": 2,
            "results": [
                {"play_code": "d1 >> play('k', sample=1)"},
                {"play_code": "d1 >> play('k', sample=2)"},
            ],
        }
        tool = SearchSamplesTool(mock_node)

        result = tool.execute(query="kick")

        assert result.success is True
        assert "Найдено 2" in result.message
        assert "sample=1" in result.message


# ---------------------------------------------------------------------------
# SetDjModeTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSetDjModeTool:
    def test_tool_name(self, mock_node):
        tool = SetDjModeTool(mock_node)
        assert tool.name == "set_dj_mode"
        assert tool.execution_type.value == "instant"
        assert tool.destructive is False

    def test_execute_enable(self, mock_node):
        tool = SetDjModeTool(mock_node)
        result = tool.execute(enabled=True, next_transition_sec=45, theme="8 марта", persona="диджей Пёс")
        assert result.success is True
        assert "включён" in result.message
        assert "45" in result.message
        pub = mock_node.get_publisher("/voice/dj_mode")
        assert pub is not None
        assert pub.published_messages
        payload = json.loads(pub.published_messages[-1].data)
        assert payload["enabled"] is True
        assert payload["next_transition_sec"] == 45
        assert payload["theme"] == "8 марта"
        assert payload["persona"] == "диджей Пёс"

    def test_execute_disable(self, mock_node):
        tool = SetDjModeTool(mock_node)
        result = tool.execute(enabled=False)
        assert result.success is True
        assert "выключен" in result.message
        pub = mock_node.get_publisher("/voice/dj_mode")
        payload = json.loads(pub.published_messages[-1].data)
        assert payload["enabled"] is False

    def test_execute_transition_seconds_alias(self, mock_node):
        """LLM иногда шлёт transition_seconds вместо next_transition_sec."""
        tool = SetDjModeTool(mock_node)
        result = tool.execute(enabled=True, transition_seconds=60)
        assert result.success is True
        assert "60" in result.message
        pub = mock_node.get_publisher("/voice/dj_mode")
        payload = json.loads(pub.published_messages[-1].data)
        assert payload["next_transition_sec"] == 60

    def test_execute_clamps_transition(self, mock_node):
        tool = SetDjModeTool(mock_node)
        tool.execute(enabled=True, next_transition_sec=5)
        pub = mock_node.get_publisher("/voice/dj_mode")
        payload = json.loads(pub.published_messages[-1].data)
        assert payload["next_transition_sec"] == 15  # max(15, ...)

        tool.execute(enabled=True, next_transition_sec=500)
        pub = mock_node.get_publisher("/voice/dj_mode")
        payload = json.loads(pub.published_messages[-1].data)
        assert payload["next_transition_sec"] == 300  # min(300, ...)

    def test_execute_with_plan(self, mock_node):
        tool = SetDjModeTool(mock_node)
        result = tool.execute(enabled=True, plan="Трек 1: старт\nТрек 2: диско")
        assert result.success is True
        assert "2 треков" in result.message
