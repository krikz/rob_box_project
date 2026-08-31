"""
test_music.py - Unit тесты для инструментов управления музыкой

Тестирует:
- MusicManager: фильтрация кода, проверка SC, execute_code, stop_pattern, stop_all,
  set_vibe_preset, get_state
- ExecuteMusicCodeTool, StopMusicTool, SetVibePresetTool, GetMusicStateTool
"""

import re
import socket
import struct
import sys
import time
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
    ComposeMusicTool,
    ExecuteMusicCodeTool,
    StopMusicTool,
    SetVibePresetTool,
    GetMusicStateTool,
    TrackLibrary,
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
    mgr._synthdefs_added = set()
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
    # issue #1812 — non-repeating compose_music() form-end deadline
    mgr._music_form_deadline_at = None
    # issue #1000 — DJ mode flag (default off; tests can call mgr.set_dj_mode(True))
    mgr._dj_mode_enabled = False
    mgr._check_supercollider = Mock(return_value=sc_running)
    return mgr


def _osc_string(s: str) -> bytes:
    """Build one OSC string field: NUL-terminated, padded to a multiple of 4.

    Mirrors the correct algorithm from issue #1808 (a naive
    ``(4 - len(b) % 4) % 4`` gives ZERO padding for strings whose encoded
    length is already a multiple of 4 — the exact bug this test helper must
    NOT reproduce, since it is used to build the "ground truth" packets the
    parser is tested against).
    """
    b = s.encode() + b"\x00"
    while len(b) % 4:
        b += b"\x00"
    return b


def _build_osc_message(address: str, tags: str = "", *args: object) -> bytes:
    """Build a raw OSC message: address + (optional) type-tag string + args.

    ``tags`` is the tag string WITHOUT the leading comma (e.g. ``"ss"``,
    ``"sif"``); pass ``""`` for an address-only message (no type tag at
    all — legal OSC, used by some scsynth notifications).
    """
    msg = bytearray(_osc_string(address))
    if not tags:
        return bytes(msg)
    msg.extend(_osc_string("," + tags))
    for tag, arg in zip(tags, args):
        if tag == "s":
            msg.extend(_osc_string(str(arg)))
        elif tag == "i":
            msg.extend(struct.pack(">i", int(arg)))
        elif tag == "f":
            msg.extend(struct.pack(">f", float(arg)))
        else:  # pragma: no cover — test helper only supports s/i/f
            raise ValueError(f"unsupported tag {tag!r} in test helper")
    return bytes(msg)


def _manager_with_captured_warnings():
    """A ``_make_manager()`` instance whose ``_log_warning`` calls are
    captured into a list instead of hitting stderr — used by the #1808
    OSC-reply tests below."""
    mgr = _make_manager()
    mgr._logger = None
    logged: list = []
    mgr._log_warning = lambda message: logged.append(message)
    return mgr, logged


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

    # -- AST layer: bypasses of the plain-text blocklist --------------------

    def test_dunder_attribute_chain_is_blocked(self):
        """().__class__.__subclasses__() never appears as a blocked *word*."""
        ok, err = self.mgr._filter_code("x = ().__class__")
        assert ok is False
        assert "__class__" in err

    def test_dunder_globals_is_blocked(self):
        ok, err = self.mgr._filter_code("f = (lambda: 0).__globals__")
        assert ok is False

    def test_computed_getattr_name_is_blocked(self):
        """String-built identifiers are the standard blocklist bypass."""
        ok, err = self.mgr._filter_code("getattr(p1, 'deg' + 'ree')")
        assert ok is False
        assert "getattr" in err

    def test_computed_setattr_name_is_blocked(self):
        ok, err = self.mgr._filter_code("setattr(Clock, chr(98) + 'pm', 170)")
        assert ok is False

    def test_getattr_with_dunder_literal_is_blocked(self):
        ok, err = self.mgr._filter_code("getattr(p1, '__class__')")
        assert ok is False
        assert "__class__" in err

    def test_syntax_error_is_rejected(self):
        ok, err = self.mgr._filter_code("p1 >> pluck([0, 2")
        assert ok is False
        assert "интаксическая" in err


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

    def test_oct_is_capped_at_5(self):
        """Санитарный потолок: выше oct=5 на 16 kHz только писк.

        🔴 Live 31.08: потолок стоял на 6 в расчёте на анти-алиасинговый
        LPF внутри ``masterlimiter``. Лимитер снят (выдавал NaN и глушил
        выход), и модель тут же засвистела: ``bell(..., oct=7)`` обрезался
        до 6 и всё равно зеркалил обертоны из-за Найквиста в 8 kHz.
        Регистры аранжировщика (бас 3, пэд 4, мелодия 5) потолок 5 не
        задевает — режется только рукописный код выше них.
        """
        code = "p1 >> pluck([0,2,4], oct=9)"
        out = self.mgr._cap_amp(code)
        assert "oct=5" in out
        assert "oct=9" not in out

    def test_bell_from_the_live_whistle_is_brought_down(self):
        """Ровно та строка, на которой робот засвистел 31.08."""
        code = "p3 >> bell([7, 4, 2, 0], dur=1, oct=7, amp=0.2)"
        out = self.mgr._cap_amp(code)
        assert "oct=5" in out
        assert "oct=7" not in out

    def test_oct_5_survives_for_register_separation(self):
        # RC2 в docs/analysis/2026-08-30-music-quality-audit.md: старый кап
        # oct<=4 схлопывал бас (oct=3) и лид в соседние октавы — микс без
        # регистрового разделения слышится как «одна повторяющаяся мелодия».
        code = "p1 >> pluck([0,2,4], oct=5)"
        out = self.mgr._cap_amp(code)
        assert "oct=5" in out

    def test_oct_unchanged_when_leq_5(self):
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
# MusicManager — issue #1803: рисунок play(...) должен делить такт
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerPatternLength:
    """``_fix_pattern_length`` достраивает рисунок до степени двойки.

    Живые прогоны 30-31.08, четыре трека подряд — барабаны «плыли», потому
    что модель писала рисунки, чья длина не делит такт (см. issue #1803).
    """

    def setup_method(self):
        self.mgr = _make_manager()

    def test_nine_step_pattern_trimmed_to_eight(self):
        """🔴 live 01.09: раньше добивали до 16, и грув бил вдвое реже.

        Хвостовые паузы снимаются: 9 шагов с паузой на конце дают ровно
        такт, а не два такта с тишиной во второй половине.
        """
        code = 'd1 >> play("X..X.o...")'  # джаз-трек 30.08
        out = self.mgr._fix_pattern_length(code)
        assert 'play("X..X.o..")' in out

    def test_pad_character_is_a_true_rest_not_a_sample(self):
        """Живой инцидент: добивка ``-`` — это звучащий сэмпл "hyphen"
        (renardo_gatherer/collections.py, каталог
        samples/0_foxdot_default/_/hyphen существует на роботе), а не
        пауза. У ``.`` сэмпл-каталога нет ни в одном паке — это и есть
        настоящая тишина. Проверяем инвариант напрямую: нормализация не
        должна добавлять НИ ОДНОГО звучащего символа, только точки.
        """
        code = 'd1 >> play("X..o.X.o.")'  # 9 шагов, диско трек 6, live 31.08
        out = self.mgr._fix_pattern_length(code)
        pattern = re.search(r'play\("([^"]*)"\)', out).group(1)
        sounding = [c for c in pattern if c != "."]
        assert sounding == [c for c in "X..o.X.o." if c != "."], (
            "звучащие символы обязаны сохраниться один в один"
        )
        assert "-" not in pattern

    def test_power_of_two_pattern_is_untouched(self):
        # "....o..." — 8 шагов, уже степень двойки: трогать нечего.
        code = 'd3 >> play("....o...")'
        assert self.mgr._fix_pattern_length(code) == code

    def test_nine_step_disco_pattern_trimmed_to_eight(self):
        code = 'd1 >> play("X..o.X.o.")'  # 9 шагов, диско трек 6, live 31.08
        out = self.mgr._fix_pattern_length(code)
        assert len(re.search(r'play\("([^"]*)"\)', out).group(1)) == 8

    def test_single_quoted_pattern_is_handled(self):
        code = "d1 >> play('X..o.X.o.')"  # 9 шагов, диско трек 6
        out = self.mgr._fix_pattern_length(code)
        assert "play('X..o.X.o')" in out

    def test_eighteen_step_pattern_trimmed_to_sixteen(self):
        # Финал диджей-сета: d2 >> play("V..o.....V..o.....") — 18 шагов.
        code = 'd2 >> play("V..o.....V..o.....")'
        out = self.mgr._fix_pattern_length(code)
        match = re.search(r'play\("([^"]*)"\)', out)
        assert len(match.group(1)) == 16
        assert match.group(1) == "V..o.....V..o..."

    def test_trailing_rests_are_not_stripped_past_a_power_of_two(self):
        """'X.....' — «бочка раз в шесть шагов», не повод бить на каждом."""
        code = 'd1 >> play("X.....")'
        out = self.mgr._fix_pattern_length(code)
        assert 'play("X...")' in out

    def test_pattern_without_trailing_rests_is_padded_as_before(self):
        """Резать нечего — добиваем, звучащие символы терять нельзя."""
        code = 'd2 >> play("-.---")'
        out = self.mgr._fix_pattern_length(code)
        assert 'play("-.---...")' in out

    def test_short_pattern_left_alone(self):
        code = 'd1 >> play("X")'
        assert self.mgr._fix_pattern_length(code) == code

    def test_multiple_layers_each_normalized_independently(self):
        # Дэт-метал трек: три рассинхронизированных рисунка в одном коде.
        code = (
            'd1 >> play("X...X...X...X...")\n'
            'd2 >> play("..........o.......")\n'
            'd3 >> play("---.-.-.-.-.-.-")\n'
        )
        out = self.mgr._fix_pattern_length(code)
        lengths = [len(m.group(1)) for m in re.finditer(r'play\("([^"]*)"\)', out)]
        # d1 уже 16; у d2 семь хвостовых пауз снимаются до 16 (а не добиваются
        # до 32); d3 звучит до последнего символа — режем нечего, добиваем.
        assert lengths == [16, 16, 16], (
            "после нормализации все три слоя обязаны делить такт ОДИНАКОВО, "
            "иначе они продолжат расходиться по фазе"
        )

    def test_execute_code_applies_pattern_fix_before_sending_to_renardo(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec") as mock_exec:
            result = mgr.execute_code('d1 >> play("X..X.o...")')
        assert result["success"] is True
        assert "-" not in result["code"]
        assert 'play("X..X.o..")' in result["code"]
        executed_code = mock_exec.call_args[0][0]
        assert 'play("X..X.o..")' in executed_code


# ---------------------------------------------------------------------------
# MusicManager — issue #1804: только d1-d3/p1-p3 звучат на роботе
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestMusicManagerSlotRemap:
    """``_remap_illegal_slots`` спасает слои из d4+/p4+ (issue #1804).

    Живой прогон 31.08, «в траве сидел кузнечик»: ``p4 >> play(...)`` не
    звучал — на роботе физически подключены только d1-d3/p1-p3, а тул
    рапортовал success. Правило было только в промпте, кода-стража не было.
    """

    def setup_method(self):
        self.mgr = _make_manager()

    def test_illegal_play_slot_moves_to_free_d_slot(self):
        code = 'p4 >> play("..o...o.", amp=0.2)'
        out, error = self.mgr._remap_illegal_slots(code)
        assert error is None
        assert 'd1 >> play("..o...o.", amp=0.2)' in out

    def test_illegal_synth_slot_moves_to_free_p_slot(self):
        code = 'd4 >> pluck([0, 2, 4])'
        out, error = self.mgr._remap_illegal_slots(code)
        assert error is None
        assert "p1 >> pluck([0, 2, 4])" in out

    def test_preferred_category_full_falls_back_to_the_other(self):
        # d1-d3 уже заняты — play() из d4 должен уйти в p-слот.
        code = (
            'd1 >> play("x")\n'
            'd2 >> play("x")\n'
            'd3 >> play("x")\n'
            'd4 >> play("o")\n'
        )
        out, error = self.mgr._remap_illegal_slots(code)
        assert error is None
        assert 'p1 >> play("o")' in out

    def test_no_free_slots_returns_honest_error_instead_of_silence(self):
        code = (
            'd1 >> play("x")\n'
            'd2 >> play("x")\n'
            'd3 >> play("x")\n'
            'p1 >> pluck([0])\n'
            'p2 >> pluck([2])\n'
            'p3 >> pluck([4])\n'
            'p4 >> play("o")\n'
        )
        out, error = self.mgr._remap_illegal_slots(code)
        assert error is not None
        assert "p4" in error
        assert out == code  # исходный код не подменяется на невалидный

    def test_repeated_illegal_name_reuses_the_same_new_slot(self):
        # Один и тот же p4 упомянут дважды — не должен расщепиться на два
        # разных плеера.
        code = 'p4 >> play("x")\np4.amp = 0.3\n'
        # `.amp =` не матчится ассайн-регексом (нет `>>`), поэтому
        # проверяем именно случай двух `>>`-строк на одно илегальное имя:
        code2 = 'p4 >> play("x")\np4 >> play("o")\n'
        out, error = self.mgr._remap_illegal_slots(code2)
        assert error is None
        assert out.count("d1 >>") == 2

    def test_the_live_grasshopper_incident_is_fixed(self):
        # Ровно тот код из живого прогона 31.08.
        code = (
            "p1 >> blip([0,2,4,7,9,7,4,2], dur=0.25, amp=0.4)\n"
            "p2 >> dub([0,0,0,-2], dur=0.5, oct=3, amp=0.35)\n"
            'p3 >> play("X..X..X.", amp=0.25)\n'
            'p4 >> play("..o...o.", amp=0.2)\n'
        )
        out, error = self.mgr._remap_illegal_slots(code)
        assert error is None
        assert "p4" not in out
        assert 'd1 >> play("..o...o.", amp=0.2)' in out

    def test_allowed_slots_are_never_touched(self):
        code = "p1 >> pluck([0])\nd2 >> play('x-o-')\n"
        out, error = self.mgr._remap_illegal_slots(code)
        assert error is None
        assert out == code

    def test_execute_code_rejects_when_all_slots_are_taken(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        code = (
            'd1 >> play("x")\n'
            'd2 >> play("x")\n'
            'd3 >> play("x")\n'
            'p1 >> pluck([0])\n'
            'p2 >> pluck([2])\n'
            'p3 >> pluck([4])\n'
            'p4 >> play("o")\n'
        )
        with patch("builtins.exec"):
            result = mgr.execute_code(code)
        assert result["success"] is False
        assert "слот" in result["error"].lower()

    def test_execute_code_remaps_before_sending_to_renardo(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec") as mock_exec:
            result = mgr.execute_code('p4 >> play("..o...o.")')
        assert result["success"] is True
        executed_code = mock_exec.call_args[0][0]
        assert "p4" not in executed_code
        assert "d1 >>" in executed_code


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

    def test_set_master_gain_targets_limiter_node_and_clamps(self):
        """The fader is the only music-vs-speech level knob (issue #986).

        It must land on the fixed ``masterlimiter`` node, and clamp — a
        gain above 1.0 would push the mix back into the limiter and undo
        the headroom the limiter exists to provide.
        """
        mgr = self._make_raw_manager()
        with patch.object(MusicManager, "_send_osc_raw") as send:
            assert mgr.set_master_gain(0.35) == 0.35
            send.assert_called_once_with(
                "/n_set", MusicManager.MASTER_LIMITER_NODE, "gain", 0.35
            )
        assert MusicManager.MASTER_LIMITER_NODE < 1000, (
            "renardo hands out node IDs from 1001 upwards "
            "(ServerManager.nextnodeID) — staying below 1000 is what keeps "
            "the limiter node collision-free"
        )
        with patch.object(MusicManager, "_send_osc_raw"):
            assert mgr.set_master_gain(3.0) == 1.0
            assert mgr.set_master_gain(-1.0) == 0.0

    def test_set_master_gain_survives_missing_socket(self):
        """A dead OSC socket must not take the music tools down with it."""
        mgr = self._make_raw_manager()
        with patch.object(
            MusicManager, "_send_osc_raw", side_effect=OSError("no route")
        ):
            assert mgr.set_master_gain(0.4) == 0.4

    def test_send_osc_raw_string_arg_is_padded_and_tagged(self):
        """Control names must travel as OSC ``s``, not as int32.

        ``/n_set <node> <control-name> <value>`` is how the master limiter
        fader is driven. If the name went out as ``i``, scsynth would read
        the first 4 bytes of ``gain`` as an int32 and silently ignore the
        set — the exact trap documented in ``_verify_and_retry_synthdefs``
        for ``"amp"``.
        """
        import struct

        mgr = self._make_raw_manager()
        with patch("rob_box_mcp_tools.tools.music.socket.socket") as mock_sock_class:
            mock_sock = MagicMock()
            mock_sock_class.return_value.__enter__ = Mock(return_value=mock_sock)
            mock_sock_class.return_value.__exit__ = Mock(return_value=False)
            mgr._send_osc_raw("/n_set", 999, "gain", 0.5)
        sent_data, _ = mock_sock.sendto.call_args[0]
        assert b",isf" in sent_data
        # "gain" is 4 chars -> needs a full 4-byte pad block for the NUL.
        assert b"gain\x00\x00\x00\x00" in sent_data
        assert sent_data.endswith(struct.pack(">f", 0.5))
        assert len(sent_data) % 4 == 0

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
# MusicManager — OSC reply parsing (issue #1808)
# ---------------------------------------------------------------------------
#
# scsynth's /fail replies used to be sent into the void (Renardo never reads
# its own OSC socket). These are the pure byte-level parsing functions that
# make the new listener trustworthy: if THEY silently swallow a malformed
# packet or misparse a real /fail, the whole feature degrades back to
# silent failure with an added false sense of safety — worse than before.


@pytest.mark.unit
class TestSplitOscAddress:
    """``MusicManager._split_osc_address`` — byte-level, no socket needed."""

    def test_fail_with_ss_args(self):
        """Typical scsynth error: /fail ,ss <failed-command> <description>."""
        data = _build_osc_message("/fail", "ss", "/g_new", "Group 1 not found")
        address, rest = MusicManager._split_osc_address(data)
        assert address == "/fail"
        # Byte-exact: rest must start with the type-tag block, unchanged.
        assert rest == data[8:]  # "/fail\0\0\0" is 8 bytes (5 chars + 3 pad)

    def test_done_address_only_no_type_tag(self):
        """Some scsynth notifications carry no type tag at all (legal OSC)."""
        data = _build_osc_message("/done")
        address, rest = MusicManager._split_osc_address(data)
        assert address == "/done"
        assert rest == b""

    def test_address_length_multiple_of_4_still_gets_full_padding(self):
        """Regression for the naive-padding trap named in issue #1808.

        A naive ``(4 - len(b) % 4) % 4`` gives ZERO padding when the
        encoded string length is already a multiple of 4 — the address
        then has no NUL terminator at all and everything after it is
        misaligned. "/abc" is 4 bytes before any terminator; a correct
        packer still adds a full 4-byte pad block (1 terminator + 3 more
        NULs), landing the payload at offset 8, not 4.
        """
        # Correctly built: "/abc" (4) + 4 pad bytes (terminator + 3 more) = 8.
        data = b"/abc" + b"\x00" * 4 + b"MARKER!!"
        address, rest = MusicManager._split_osc_address(data)
        assert address == "/abc"
        assert rest == b"MARKER!!"

    def test_empty_bytes_returns_none_safely(self):
        assert MusicManager._split_osc_address(b"") == (None, b"")

    def test_missing_leading_slash_returns_none_safely(self):
        """Garbage on the wire (not an OSC message at all) must not raise."""
        assert MusicManager._split_osc_address(b"garbage\x00") == (None, b"")

    def test_truncated_message_with_no_terminator_returns_none_safely(self):
        """A cut-off UDP packet (no NUL anywhere) must not raise or hang."""
        assert MusicManager._split_osc_address(b"/fail") == (None, b"")


@pytest.mark.unit
class TestDecodeOscArgs:
    """``MusicManager._decode_osc_args`` — byte-level, no socket needed."""

    def test_ss_args(self):
        """The common /fail shape: failed command + human-readable reason."""
        data = _build_osc_message("/fail", "ss", "/g_new", "Group 1 not found")
        _address, rest = MusicManager._split_osc_address(data)
        args = MusicManager._decode_osc_args(rest)
        assert args == ["/g_new", "Group 1 not found"]

    def test_sif_mixed_types_regression_1444(self):
        """Regression for #1444: a control name typed as int32 by mistake.

        ``/n_set`` with tag ``,isf`` sent a float 1.0 through an int32
        slot and it came back as ``Node 1065353216 not found`` — a value
        only explicable once you know 1065353216 is IEEE-754 1.0f reread
        as int32. The decoder must keep ``i`` and ``f`` distinct instead
        of collapsing everything to one numeric type.
        """
        data = _build_osc_message("/fail", "sif", "/n_set", 1065353216, 1.0)
        _address, rest = MusicManager._split_osc_address(data)
        args = MusicManager._decode_osc_args(rest)
        assert args == ["/n_set", 1065353216, 1.0]
        assert isinstance(args[1], int)
        assert isinstance(args[2], float)

    def test_done_style_ss_args_alignment(self):
        """``,ss`` is itself exactly 4 bytes (",ss" + NUL) — zero EXTRA
        padding needed after the terminator. Pins the other half of the
        alignment math: the loop must add nothing when already aligned,
        not just something when it isn't (see TestSplitOscAddress for the
        opposite case)."""
        data = _build_osc_message("/done", "ss", "/foxdot", "ok")
        _address, rest = MusicManager._split_osc_address(data)
        # ",ss\0" is 4 bytes: tag block ends exactly on a boundary.
        assert rest[:4] == b",ss\x00"
        args = MusicManager._decode_osc_args(rest)
        assert args == ["/foxdot", "ok"]

    def test_no_type_tag_returns_empty_list(self):
        assert MusicManager._decode_osc_args(b"") == []
        assert MusicManager._decode_osc_args(b"not-a-tag-block") == []

    def test_truncated_after_type_tag_returns_partial_without_raising(self):
        """Type tag promises an int, but the packet is cut short."""
        rest = _osc_string(",i")  # tag block present, no int payload follows
        assert MusicManager._decode_osc_args(rest) == []

    def test_truncated_string_arg_returns_partial_without_raising(self):
        """String tag with no NUL terminator anywhere in the remaining bytes."""
        rest = _osc_string(",s") + b"nonulhere"
        assert MusicManager._decode_osc_args(rest) == []

    def test_unknown_type_tag_stops_without_raising(self):
        """A blob ('b') or other unsupported tag must stop parsing cleanly,
        keeping whatever was already decoded instead of raising."""
        msg = bytearray(_osc_string(",sb"))
        msg.extend(_osc_string("/s_new"))
        # No attempt to encode a real blob — decoder must bail out at 'b'
        # without needing valid blob bytes to follow.
        args = MusicManager._decode_osc_args(bytes(msg))
        assert args == ["/s_new"]


@pytest.mark.unit
class TestLogOscReply:
    """``MusicManager._log_osc_reply`` — formatting into the mcp_server log."""

    def test_fail_is_logged_with_command_and_description(self):
        mgr, logged = _manager_with_captured_warnings()
        data = _build_osc_message("/fail", "ss", "/g_new", "Group 1 not found")
        mgr._log_osc_reply(data)
        assert len(logged) == 1
        assert "/g_new" in logged[0]
        assert "Group 1 not found" in logged[0]

    def test_fail_without_type_tag_falls_back_to_raw_bytes(self):
        """Some scsynth versions/edge cases may send /fail with no args at
        all — must still produce a readable (non-crashing) log line."""
        mgr, logged = _manager_with_captured_warnings()
        data = _build_osc_message("/fail")
        mgr._log_osc_reply(data)
        assert len(logged) == 1
        assert "scsynth" in logged[0].lower() or "FAILURE" in logged[0]

    def test_non_fail_address_is_not_logged(self):
        """/done and friends are routine acks — logging them would drown
        out the actual failures this feature exists to surface."""
        mgr, logged = _manager_with_captured_warnings()
        data = _build_osc_message("/done", "ss", "/foxdot", "ok")
        mgr._log_osc_reply(data)
        assert logged == []

    def test_garbage_bytes_do_not_raise_or_log(self):
        mgr, logged = _manager_with_captured_warnings()
        mgr._log_osc_reply(b"\x00\x00\x00\x00")
        assert logged == []


# ---------------------------------------------------------------------------
# MusicManager — reply-listener plumbing (issue #1808)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestLogScsynthReplyIfAny:
    """``_send_osc_raw``'s post-sendto reply check — must never block long
    or raise; a timeout (the common, successful case) is normal, not an
    error."""

    def test_timeout_is_swallowed_silently(self):
        mgr, logged = _manager_with_captured_warnings()
        sock = MagicMock()
        sock.recvfrom.side_effect = socket.timeout
        mgr._log_scsynth_reply_if_any(sock)
        assert logged == []
        sock.settimeout.assert_called_once_with(MusicManager.OSC_REPLY_TIMEOUT_SECONDS)

    def test_fail_reply_is_logged(self):
        mgr, logged = _manager_with_captured_warnings()
        sock = MagicMock()
        sock.recvfrom.return_value = (
            _build_osc_message("/fail", "ss", "/g_new", "Group 1 not found"),
            ("127.0.0.1", 57110),
        )
        mgr._log_scsynth_reply_if_any(sock)
        assert len(logged) == 1
        assert "Group 1 not found" in logged[0]

    def test_arbitrary_socket_error_is_swallowed(self):
        """Any transport hiccup here must never propagate into the admin
        OSC send path it's attached to (``_send_osc_raw``)."""
        mgr, logged = _manager_with_captured_warnings()
        sock = MagicMock()
        sock.recvfrom.side_effect = OSError("network unreachable")
        mgr._log_scsynth_reply_if_any(sock)  # must not raise
        assert logged == []


@pytest.mark.unit
class TestRenardoReplyListenerLoop:
    """``_renardo_reply_listener_loop`` — the background thread body."""

    def test_logs_fail_then_exits_cleanly_when_socket_closes(self):
        mgr, logged = _manager_with_captured_warnings()
        sock = MagicMock()
        fail_packet = _build_osc_message("/fail", "ss", "/s_new", "SynthDef blip not found")
        sock.recvfrom.side_effect = [
            (fail_packet, ("127.0.0.1", 57110)),
            OSError("socket closed"),
        ]
        # Must return (not hang) once the socket goes away.
        mgr._renardo_reply_listener_loop(sock)
        assert len(logged) == 1
        assert "SynthDef blip not found" in logged[0]

    def test_bad_packet_does_not_kill_the_loop(self):
        """One malformed datagram must not stop the listener from seeing
        the next (real) one."""
        mgr, logged = _manager_with_captured_warnings()
        sock = MagicMock()
        fail_packet = _build_osc_message("/fail", "ss", "/g_new", "Group 1 not found")
        sock.recvfrom.side_effect = [
            (b"\xff\xff garbage", ("127.0.0.1", 57110)),
            (fail_packet, ("127.0.0.1", 57110)),
            OSError("socket closed"),
        ]
        mgr._renardo_reply_listener_loop(sock)
        assert len(logged) == 1
        assert "Group 1 not found" in logged[0]


@pytest.mark.unit
class TestAttachRenardoReplyListener:
    """``_attach_renardo_reply_listener`` — best-effort tap onto Renardo's
    own long-lived scsynth socket. Must never raise, regardless of what
    ``_rt``'s object graph looks like, and must never spawn more than one
    listener thread per underlying socket."""

    def _mgr(self):
        mgr = _make_manager()
        mgr._logger = None
        mgr._renardo_reply_sock = None
        return mgr

    def test_real_socket_gets_a_daemon_listener_thread(self):
        mgr = self._mgr()
        real_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            fake_rt = SimpleNamespace(
                Server=SimpleNamespace(client=SimpleNamespace(socket=real_sock))
            )
            with patch("rob_box_mcp_tools.tools.music.threading.Thread") as mock_thread_cls:
                mock_thread = Mock()
                mock_thread_cls.return_value = mock_thread

                mgr._attach_renardo_reply_listener(fake_rt)

                assert mgr._renardo_reply_sock is real_sock
                _args, kwargs = mock_thread_cls.call_args
                assert kwargs["target"] == mgr._renardo_reply_listener_loop
                assert kwargs["args"] == (real_sock,)
                assert kwargs["daemon"] is True
                mock_thread.start.assert_called_once()
        finally:
            real_sock.close()

    def test_missing_server_attribute_does_not_raise_or_attach(self):
        mgr = self._mgr()
        with patch("rob_box_mcp_tools.tools.music.threading.Thread") as mock_thread_cls:
            mgr._attach_renardo_reply_listener(SimpleNamespace())  # no .Server at all
        assert mgr._renardo_reply_sock is None
        mock_thread_cls.assert_not_called()

    def test_none_rt_does_not_raise(self):
        mgr = self._mgr()
        mgr._attach_renardo_reply_listener(None)
        assert mgr._renardo_reply_sock is None

    def test_client_socket_attribute_missing_does_not_attach(self):
        mgr = self._mgr()
        fake_rt = SimpleNamespace(Server=SimpleNamespace(client=SimpleNamespace()))
        with patch("rob_box_mcp_tools.tools.music.threading.Thread") as mock_thread_cls:
            mgr._attach_renardo_reply_listener(fake_rt)
        assert mgr._renardo_reply_sock is None
        mock_thread_cls.assert_not_called()

    def test_socket_attribute_is_not_a_real_socket_does_not_attach(self):
        """A different Renardo version (or a mock in some other test) might
        expose a ``.socket`` that isn't a ``socket.socket`` — must be
        ignored rather than handed to a thread expecting real recv()."""
        mgr = self._mgr()
        fake_rt = SimpleNamespace(
            Server=SimpleNamespace(client=SimpleNamespace(socket="not-a-socket"))
        )
        with patch("rob_box_mcp_tools.tools.music.threading.Thread") as mock_thread_cls:
            mgr._attach_renardo_reply_listener(fake_rt)
        assert mgr._renardo_reply_sock is None
        mock_thread_cls.assert_not_called()

    def test_same_socket_is_attached_only_once(self):
        """A retried ``_ensure_renardo_available`` must not stack up a new
        listener thread per retry as long as Renardo kept the same socket."""
        mgr = self._mgr()
        real_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            fake_rt = SimpleNamespace(
                Server=SimpleNamespace(client=SimpleNamespace(socket=real_sock))
            )
            with patch("rob_box_mcp_tools.tools.music.threading.Thread") as mock_thread_cls:
                mock_thread_cls.return_value = Mock()
                mgr._attach_renardo_reply_listener(fake_rt)
                mgr._attach_renardo_reply_listener(fake_rt)
                assert mock_thread_cls.call_count == 1
        finally:
            real_sock.close()

    def test_attaching_raises_internally_is_swallowed(self):
        """Even a genuinely broken object graph (attribute access itself
        raises) must not take down Renardo initialization."""
        mgr = self._mgr()

        class _Explodes:
            @property
            def Server(self):
                raise RuntimeError("boom")

        mgr._attach_renardo_reply_listener(_Explodes())  # must not raise
        assert mgr._renardo_reply_sock is None


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

    def test_initialize_renardo_is_idempotent(self, tmp_path, monkeypatch):
        """Повторная инициализация НЕ должна повторно звать sdef.add() —
        каждый add() мутирует UGen-граф (osc*env) → компаундинг
        ("too big for sending" → scsynth "late")."""
        import socket
        import types

        from rob_box_mcp_tools.tools import music as music_mod

        mgr = _make_manager()
        mgr._logger = None

        rt = SimpleNamespace(
            Server=SimpleNamespace(booted=True),
            effect_manager=SimpleNamespace(reload=lambda: None),
            SynthDefs={
                f"s{i}": SimpleNamespace(add=Mock())
                for i in range(12)
            },
            # Кастомные synthdef (warmpad и пр.) имеют add() в проде —
            # верно отражаем это, чтобы повторная инициализация не падала.
            SynthDef=lambda name: SimpleNamespace(name=name, add=Mock()),
        )
        renardo_lib = types.ModuleType("renardo_lib")
        renardo_lib.runtime = rt
        monkeypatch.setitem(sys.modules, "renardo_lib", renardo_lib)
        monkeypatch.setitem(sys.modules, "renardo_lib.runtime", rt)
        monkeypatch.setenv("HOME", str(tmp_path))
        monkeypatch.setattr(music_mod.time, "sleep", lambda _seconds: None)
        fake_sock = MagicMock()
        fake_sock.recvfrom.side_effect = socket.timeout
        monkeypatch.setattr(music_mod.socket, "socket", lambda *a, **k: fake_sock)

        mgr._initialize_renardo()
        mgr._initialize_renardo()

        for i in range(12):
            name = f"s{i}"
            sdef = rt.SynthDefs[name]
            assert sdef.add.call_count == 1, (
                f"{name}.add() вызван {sdef.add.call_count} раз, ожидался 1"
            )


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
        s_new_msgs = [d for d in sent if d.startswith(b"/s_new")]
        n_free_msgs = [d for d in sent if d.startswith(b"/n_free")]
        assert s_new_msgs, "probe должен отправлять /s_new"
        for data in s_new_msgs:
            assert len(data) % 4 == 0, f"OSC not aligned: {len(data)}"
            assert data.startswith(b"/s_new\x00\x00"), f"bad address pad: {data[:16]!r}"
            # Fix (19.08, свист #1444): type tag обязан быть ",siiisf"
            # (control "amp" — string, а не int), иначе amp=0 не применяется
            # и пробные ноды играют с дефолтным amp=1 / sus=1.
            assert b",siiisf" in data, f"bad type tag: {data!r}"
        # Fix (19.08, свист #1444): созданные пробные ноды должны
        # освобождаться через /n_free (иначе sustained тон живёт вечно).
        assert n_free_msgs, "пробные ноды должны освобождаться через /n_free"
        for data in n_free_msgs:
            assert len(data) % 4 == 0, f"/n_free not aligned: {len(data)}"
            assert data.startswith(b"/n_free"), f"bad /n_free: {data[:16]!r}"
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

    def test_resend_uses_load_when_already_added(self, monkeypatch):
        """Уже добавленный SynthDef досылается через load(), а НЕ add() —
        add() повторно мутирует UGen-граф (компаундинг)."""
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
        mgr._synthdefs_added.add("pads")
        rt = SimpleNamespace(
            SynthDefs={
                "pads": SimpleNamespace(add=Mock(), load=Mock()),
                "noise": SimpleNamespace(add=Mock(), load=Mock()),
            }
        )

        mgr._verify_and_retry_synthdefs(rt, mgr._send_osc_raw)

        assert rt.SynthDefs["pads"].load.call_count >= 1, (
            "уже добавленный синт должен досылаться через load()"
        )
        assert rt.SynthDefs["pads"].add.call_count == 0, (
            "повторный add() мутирует UGen-граф — запрещён"
        )


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

        # Issue #1815: "-" — звучащий хэт ("hyphen"), а не пауза; настоящая
        # пауза — ".". Прогреваться должны x, "-" (дважды) и o — всё, кроме
        # точки. Обе "-" объединены в проверке ниже, а не отброшены.
        mgr._prewarm_sample_buffers('d1 >> play("x-o-.", dur=0.5, sample=1)')

        assert [c[0] for c in calls] == ["x", "-", "o", "-"], (
            f"должны грузиться все звучащие символы (в т.ч. '-'), кроме "
            f"паузы '.', получено: {calls!r}"
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

    def test_live_3008_short_beat_survives_longer_than_the_tts_reply(self):
        """🔴 Регрессия 30.08 (vision-pi 12:30:22 → 12:30:43).

        «сыграй короткий бит» → LLM отдала ``segments=8`` при
        ``Clock.bpm=90``. Старая формула давала дедлайн 8*2.667 = 21.3 s,
        и watchdog убил бит через 20 s: TTS-ответ («Бит играет и сохранён
        как «тисбит».», 6.8 s) закончился на 11-й секунде, музыка играла
        одна ещё 7 секунд и оборвалась. Следующая реплика юзера —
        «продолжай развивать этот бит» — пришла в тишину.

        Дедлайн обязан оставаться ПРЕДОХРАНИТЕЛЕМ: заметно длиннее того,
        что напросила LLM, и не короче минуты.
        """
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._renardo_context["Clock"] = SimpleNamespace(bpm=90)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> blip([0,2,4,7])", segments=8)
        remaining = mgr._music_deadline_at - time.monotonic()
        assert remaining >= 60.0 - 0.5, (
            "8 баров на 90 bpm давали 21 s — бит умирал раньше, чем "
            "юзер успевал попросить продолжение"
        )

    def test_segments_deadline_applies_the_safety_factor(self):
        """Дедлайн = музыкальная длина × запас, а не ровно она."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._renardo_context["Clock"] = SimpleNamespace(bpm=120)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", segments=64)
        bar = 4 * 60.0 / 120.0  # 2.0 s
        expected = 64 * bar * MusicManager.SEGMENTS_DEADLINE_SAFETY_FACTOR
        remaining = mgr._music_deadline_at - time.monotonic()
        assert remaining == pytest.approx(expected, abs=1.0)
        assert expected > MusicManager.MIN_SEGMENTS_DEADLINE_SECONDS

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

    def test_stop_unknown_pattern_is_rejected(self):
        """Unknown names are rejected — pattern_name is whitelisted (RCE fix)."""
        mgr = _make_manager()
        result = mgr.stop_pattern("unknown")
        assert result["success"] is False
        assert "unknown" in result["error"]

    def test_stop_builtin_player_name_succeeds_without_sc(self):
        """Renardo's own players (d1-d9/p1-p9/s1-s9/l1-l9) are always allowed."""
        mgr = _make_manager()
        result = mgr.stop_pattern("p1")
        assert result["success"] is True

    def test_stop_pattern_rejects_code_injection(self):
        """The RCE payload from the audit must not reach exec()."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec") as mock_exec:
            result = mgr.stop_pattern("__import__('os').system('id') #")
        assert result["success"] is False
        mock_exec.assert_not_called()

    def test_stop_pattern_rejects_dotted_name(self):
        """Anything that is not a bare identifier is refused."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        mgr._active_patterns.add("p1")
        with patch("builtins.exec") as mock_exec:
            result = mgr.stop_pattern("p1.__class__")
        assert result["success"] is False
        mock_exec.assert_not_called()
        assert "p1" in mgr._active_patterns

    def test_stop_pattern_calls_player_stop_without_exec(self):
        """The player object is resolved from the namespace, never exec'd."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        player = MagicMock()
        mgr._renardo_context = {"bass": player}
        mgr._active_patterns.add("bass")
        with patch("builtins.exec") as mock_exec:
            result = mgr.stop_pattern("bass")
        assert result["success"] is True
        player.stop.assert_called_once_with()
        mock_exec.assert_not_called()

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
# ComposeMusicTool — form-end watchdog protection (issue #1812)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestComposeMusicToolFormDeadline:
    """A ``compose_music(repeat=False)`` track has a computable finite
    length. The tool must arm the manager's form-end deadline so the idle
    watchdog doesn't cut it off before it actually finishes (issue #1812).
    A looping (``repeat=True``) track has no such end, so the deadline must
    stay cleared and idle-TTL alone governs it — same as before #1812.
    """

    def _make_tool(self, mock_node, **kwargs):
        mgr = _make_manager(sc_running=True, renardo_available=True, **kwargs)
        return ComposeMusicTool(mock_node, mgr), mgr

    _COMMON_KWARGS = dict(
        bpm=100,
        root="C",
        scale="minor",
        form="arc",
        drums="X..o.X.o",
        bass_synth="dub",
        bass_notes="0, 0, 3, -2",
        lead_synth="blip",
        lead_notes="0, 2, 4, 7",
    )

    def test_repeat_false_arms_the_form_deadline(self, mock_node):
        tool, mgr = self._make_tool(mock_node)
        with patch("builtins.exec"):
            result = tool.execute(repeat=False, **self._COMMON_KWARGS)
        assert result.success is True
        assert mgr._music_form_deadline_at is not None
        assert mgr._music_form_deadline_at > time.monotonic()

    def test_repeat_false_deadline_matches_the_form_length(self, mock_node):
        from rob_box_mcp_tools.core.arranger import FORMS, BEATS_PER_BAR

        tool, mgr = self._make_tool(mock_node)
        with patch("builtins.exec"):
            tool.execute(repeat=False, **self._COMMON_KWARGS)
        total_beats = sum(bars for _n, bars, _i in FORMS["arc"]) * BEATS_PER_BAR
        expected_duration = total_beats * 60.0 / 100.0
        remaining = mgr._music_form_deadline_at - time.monotonic()
        assert remaining == pytest.approx(expected_duration, abs=1.0)

    def test_repeat_true_leaves_the_form_deadline_cleared(self, mock_node):
        tool, mgr = self._make_tool(mock_node)
        with patch("builtins.exec"):
            result = tool.execute(repeat=True, **self._COMMON_KWARGS)
        assert result.success is True
        assert mgr._music_form_deadline_at is None

    def test_repeat_true_clears_a_stale_deadline_from_a_previous_track(self, mock_node):
        """LLM plays a fixed-length track, then starts a DJ loop — the old
        track's form-end protection must not leak into the new session."""
        tool, mgr = self._make_tool(mock_node)
        with patch("builtins.exec"):
            tool.execute(repeat=False, **self._COMMON_KWARGS)
        assert mgr._music_form_deadline_at is not None
        with patch("builtins.exec"):
            tool.execute(repeat=True, **self._COMMON_KWARGS)
        assert mgr._music_form_deadline_at is None

    def test_repeat_false_track_survives_idle_ttl_via_the_real_watchdog_call(self, mock_node):
        """End-to-end: compose a finite track, then run the exact watchdog
        query (auto_stop_idle_music) that mcp_server's timer uses — it must
        NOT stop the track while the form is still playing, even though
        dialogue has been silent well past the (short, here) idle TTL."""
        tool, mgr = self._make_tool(mock_node)
        with patch("builtins.exec"):
            tool.execute(repeat=False, **self._COMMON_KWARGS)
        # Idle far past a short TTL, but still inside the form's own runtime.
        result = mgr.auto_stop_idle_music(
            ttl_seconds=1, now=mgr._music_form_deadline_at - 1.0
        )
        assert result["stopped"] is False
        assert result.get("held_reason") == "form_not_finished"


# ---------------------------------------------------------------------------
# StopMusicTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestTrackLibrarySlug:
    """🔴 Регрессия 30.08: русское имя трека превращалось в частокол «_».

    Старый ``_slug`` был ``re.sub(r"[^a-z0-9_]", "_", name.lower())``: КАЖДЫЙ
    кириллический символ заменялся на «_», поэтому slug кодировал только
    длину имени. В живой медиатеке робота лежит запись
    ``('________________', 'комната_мудрости')``, а «сохрани как трек
    тисбит» породило четыре записи одного трека — ``tisbeat``, ``tisbit``,
    ``thisbit``, ``tinbit``: LLM каждый раз транслитерировала имя сама и
    каждый раз по-своему. «Удали трек тисбит» после этого не находил ничего.
    """

    def test_cyrillic_is_transliterated(self):
        assert TrackLibrary._slug("Тисбит") == "tisbit"

    def test_slug_is_case_insensitive(self):
        assert TrackLibrary._slug("ТисБит") == TrackLibrary._slug("тисбит")

    def test_different_names_do_not_collide(self):
        """Раньше «мурка» и «пляска» (по 5 букв) давали один и тот же slug."""
        assert TrackLibrary._slug("мурка") != TrackLibrary._slug("пляск")

    def test_spaces_do_not_become_a_picket_fence(self):
        assert TrackLibrary._slug("комната мудрости") == "komnata_mudrosti"

    def test_latin_slugs_are_unchanged(self):
        """Существующие записи медиатеки не должны переехать."""
        assert TrackLibrary._slug("csm_chill_v2") == "csm_chill_v2"
        assert TrackLibrary._slug("club_energy_128bpm") == "club_energy_128bpm"

    def test_empty_name_stays_empty(self):
        """``save_track`` отвергает пустой slug — контракт не меняем."""
        assert TrackLibrary._slug("   ") == ""

    def test_save_and_delete_round_trip_by_russian_name(self, tmp_path):
        lib = TrackLibrary(db_path=str(tmp_path / "tracks.db"))
        saved = lib.save_track(name="Тисбит", code="p1 >> blip([0])")
        assert saved["success"] is True
        assert saved["name"] == "tisbit"
        # То, как юзер произнёс имя во второй раз, значения не имеет.
        deleted = lib.delete_track("тисбит")
        assert deleted["success"] is True


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

    def test_stop_unknown_pattern_is_rejected(self, mock_node):
        """Names outside the whitelist are refused instead of exec'd (RCE fix)."""
        tool, _ = self._make_tool(mock_node)
        result = tool.execute(pattern_name="nonexistent")
        assert result.success is False

    def test_stop_builtin_player_name_succeeds(self, mock_node):
        """The LLM can still stop any of Renardo's built-in players."""
        tool, _ = self._make_tool(mock_node)
        result = tool.execute(pattern_name="d1")
        assert result.success is True

    def test_sound_stop_is_delegated_to_the_node(self, mock_node):
        """🔴 Регрессия 30.08: mp3 из ``gen_play_from_library`` играет в
        ``sound_node``, и остановить его умел ТОЛЬКО этот тул. Те же два
        топика нужны ``music_cleanup`` и watchdog'у, поэтому публикация
        переехала в ``McpServerNode.stop_generated_track_playback`` — тул
        обязан звать её, а не дублировать паблишеры.
        """
        calls = []
        mock_node.stop_generated_track_playback = lambda: calls.append(1)
        tool, _ = self._make_tool(mock_node)
        result = tool.execute()
        assert result.success is True
        assert calls == [1], "stop_music не позвал stop_generated_track_playback"

    def test_sound_stop_degrades_when_node_has_no_helper(self, mock_node):
        """Нода без хелпера (юнит-тесты / minimal install) — не падаем."""
        assert not hasattr(mock_node, "stop_generated_track_playback")
        tool, _ = self._make_tool(mock_node)
        assert tool.execute().success is True


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

    def test_real_init_default_ttl_is_1800_when_env_unset(self, monkeypatch):
        """Issue #1812: 300s was too short for "listening in silence" —
        the default TTL is now 1800s (30 min) when nobody overrides it.

        Exercises the REAL ``MusicManager.__init__`` (not the ``_make_manager``
        test double, which sets ``_auto_stop_ttl_seconds`` by hand) so the
        assertion catches a regression in the actual constructor logic.
        """
        monkeypatch.delenv("MUSIC_AUTO_STOP_TTL_SECONDS", raising=False)
        with patch.object(MusicManager, "_evaluate_music_stack_health", lambda self, **k: None), \
                patch.object(MusicManager, "_initialize_renardo", lambda self: None):
            mgr = MusicManager()
        assert mgr._auto_stop_ttl_seconds == 1800

    def test_real_init_honors_ttl_env_override(self, monkeypatch):
        """``MUSIC_AUTO_STOP_TTL_SECONDS`` still overrides the 1800s default."""
        monkeypatch.setenv("MUSIC_AUTO_STOP_TTL_SECONDS", "42")
        with patch.object(MusicManager, "_evaluate_music_stack_health", lambda self, **k: None), \
                patch.object(MusicManager, "_initialize_renardo", lambda self: None):
            mgr = MusicManager()
        assert mgr._auto_stop_ttl_seconds == 42

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

    def test_idle_ttl_stop_reports_its_reason(self):
        """Строка watchdog'а в логе врала: писала «после 20.1s (ttl=300s)»,
        хотя убил музыку segments-дедлайн. Обе ветки теперь называют себя.
        """
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        # Без segments дедлайна нет — сработает только idle-TTL.
        assert mgr._music_deadline_at is None
        result = mgr.auto_stop_idle_music(ttl_seconds=1, now=time.monotonic() + 10)
        assert result["stopped"] is True
        assert result.get("stop_reason") == "idle_ttl"

    def test_segments_stop_reports_the_segments_value(self):
        """``deadline_segments`` в результате — чтобы из лога было видно,
        какую цифру напросила LLM."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1", segments=16)
        result = mgr.auto_stop_idle_music(
            ttl_seconds=300, now=mgr._music_deadline_at + 1
        )
        assert result.get("stop_reason") == "segments_deadline"
        assert result.get("deadline_segments") == 16

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

    # ----- Issue #1812 — compose_music() form-end deadline -----------------

    def test_set_form_deadline_arms_a_future_wall_clock_time(self):
        mgr = _make_manager()
        mgr.set_form_deadline(120.0)
        assert mgr._music_form_deadline_at is not None
        assert mgr._music_form_deadline_at > time.monotonic()

    def test_clear_form_deadline_resets_to_none(self):
        mgr = _make_manager()
        mgr.set_form_deadline(120.0)
        mgr.clear_form_deadline()
        assert mgr._music_form_deadline_at is None

    def test_form_not_finished_survives_idle_ttl(self):
        """A repeat=False track must NOT be cut off by idle-TTL before its
        one pass of the form has actually finished playing — listening to
        music in silence is the expected use, not an abandoned session."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        # Form is 120s long; the idle-TTL (1s) has long been exceeded, but
        # the form itself has 60s left to play.
        mgr.set_form_deadline(120.0)
        now = mgr._music_form_deadline_at - 60.0
        result = mgr.auto_stop_idle_music(ttl_seconds=1, now=now)
        assert result["stopped"] is False
        assert result.get("held_reason") == "form_not_finished"
        assert result["form_deadline_remaining_s"] == pytest.approx(60.0, abs=0.5)
        # Music is still active — nothing was torn down.
        assert "p1" in mgr._active_patterns

    def test_form_deadline_passed_lets_idle_ttl_stop_it(self):
        """Once the form has actually finished, idle-TTL governs normally."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        mgr.set_form_deadline(120.0)
        # 1s past the form's natural end, and idle (ttl=1) is also exceeded.
        now = mgr._music_form_deadline_at + 1.0
        result = mgr.auto_stop_idle_music(ttl_seconds=1, now=now)
        assert result["stopped"] is True
        assert result.get("stop_reason") == "idle_ttl"

    def test_looping_track_has_no_form_deadline_and_obeys_ttl(self):
        """repeat=True music has no natural end — idle-TTL alone governs it,
        exactly like before #1812 (form deadline stays None)."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        assert mgr._music_form_deadline_at is None
        result = mgr.auto_stop_idle_music(ttl_seconds=1, now=time.monotonic() + 10)
        assert result["stopped"] is True
        assert result.get("stop_reason") == "idle_ttl"

    def test_stop_all_clears_form_deadline(self):
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        mgr.set_form_deadline(120.0)
        mgr.stop_all()
        assert mgr._music_form_deadline_at is None

    def test_execute_code_clears_a_stale_form_deadline(self):
        """A fresh code push (e.g. plain execute_music_code after a
        compose_music track) must not stay protected by the OLD track's
        form-end deadline — that would block idle-TTL for unrelated code."""
        mgr = _make_manager(sc_running=True, renardo_available=True)
        with patch("builtins.exec"):
            mgr.execute_code("p1 >> pluck([0])", pattern_name="p1")
        mgr.set_form_deadline(120.0)
        with patch("builtins.exec"):
            mgr.execute_code("p2 >> pluck([2])", pattern_name="p2")
        assert mgr._music_form_deadline_at is None

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

    # ----- chop / spack hard blocks (live 20.08) ---------------------------

    def test_chop_is_rejected(self):
        errors, _ = self.mgr._validate_music_code(
            "p1 >> pluck([0,2,4], dur=0.5, chop=[8,16])"
        )
        assert errors
        assert any("chop" in e for e in errors)

    def test_chop_zero_is_allowed(self):
        errors, _ = self.mgr._validate_music_code(
            "p1 >> pluck([0,2,4], dur=0.5, chop=0)"
        )
        assert errors == []

    def test_spack_nonzero_is_rejected(self):
        errors, _ = self.mgr._validate_music_code(
            'd1 >> play("a", sample=11, spack=1)'
        )
        assert errors
        assert any("spack" in e for e in errors)

    def test_spack_zero_is_allowed(self):
        errors, _ = self.mgr._validate_music_code(
            'd1 >> play("x-o-", sample=1, spack=0)'
        )
        assert errors == []

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

