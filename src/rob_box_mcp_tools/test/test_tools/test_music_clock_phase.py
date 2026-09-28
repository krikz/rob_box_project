"""Issue #3112 — фаза клока Renardo относительно формы трека.

Диагностика (всегда вкл.): снимок фазы в результате ``compose_music`` и в
INFO-логе. Кандидат фикса (``ROB_BOX_MUSIC_ALIGN_CLOCK=1``, по умолчанию
выкл.): строка ``Clock.set_time(...)`` сразу после ``Clock.clear()``.

Renardo здесь нет — клок подделан, арифметика ``set_time``/``next_bar``/
``_now`` повторяет ``renardo_lib/TempoClock.py`` 0.9.13 (строки 420, 463,
648).
"""

import sys
import time
from unittest.mock import MagicMock, patch

import pytest

for _mod in [
    "rclpy", "rclpy.node", "rclpy.action", "rclpy.qos", "std_msgs", "std_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg", "nav2_msgs", "nav2_msgs.action",
    "action_msgs", "action_msgs.srv", "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.core import renardo_sanitizer  # noqa: E402
from rob_box_mcp_tools.core.arranger import form_total_beats, render, spec_from_flat  # noqa: E402
from rob_box_mcp_tools.core.clock_phase import (  # noqa: E402
    ALIGN_LEAD_BEATS,
    clock_align_prelude,
    clock_phase_snapshot,
)
from rob_box_mcp_tools.core.club_arranger import club_form_beats, render_club  # noqa: E402
from rob_box_mcp_tools.tools.music import ComposeMusicTool, music_align_clock_enabled  # noqa: E402

from .test_music import _make_manager  # noqa: E402


class FakeClock:
    """Минимальный TempoClock: float-bpm, ``_now`` от времени, как в renardo."""

    def __init__(self, beat: float = 0.0, bpm: float = 120.0, nudge_s: float = 0.0):
        self.meter = (4, 4)
        self.bpm = bpm
        self.nudge_s = nudge_s
        self.bpm_start_beat = beat
        self.bpm_start_time = time.time()
        self.queue = ["stale-event"]

    def now(self) -> float:  # TempoClock._now (float bpm), время с nudge
        elapsed = time.time() + self.nudge_s - self.bpm_start_time
        return self.bpm_start_beat + elapsed * self.bpm / 60.0

    def set_time(self, beat: float) -> None:  # TempoClock.py:420 (без nudge)
        self.queue.clear()
        self.bpm_start_beat = beat
        self.bpm_start_time = time.time()

    def next_bar(self) -> float:  # TempoClock.py:648
        beat = self.now()
        return beat + (self.meter[0] - (beat % self.meter[0]))

    def clear(self) -> None:
        self.queue.clear()


def _spec(**over):
    flat = dict(
        bpm=100, root="C", scale="minor", form="arc", drums="X..o.X.o",
        bass_synth="dub", bass_notes="0, 0, 3, -2", lead_synth="blip",
        lead_notes="0, 2, 4, 7", repeat=False,
    )
    flat.update(over)
    return spec_from_flat(**flat)


# --------------------------------------------------------------------- core


def test_prelude_exact_text():
    assert clock_align_prelude(128) == "Clock.set_time(((Clock.now() + 2) // 128 + 1) * 128 - 2)"


@pytest.mark.parametrize("bad", [0, -4, 130])
def test_prelude_rejects_form_not_multiple_of_bar(bad):
    with pytest.raises(ValueError):
        clock_align_prelude(bad)


@pytest.mark.parametrize("start", [0.0, 3.99, 37.3, 125.9, 126.0, 127.99, 500.9, 1021.0, 99999.5])
@pytest.mark.parametrize("nudge_s", [0.0, 0.45, -0.45])
def test_prelude_lands_next_bar_on_form_start_and_only_jumps_forward(start, nudge_s):
    clock = FakeClock(beat=start, nudge_s=nudge_s)
    before = clock.now()
    exec(clock_align_prelude(128), {"Clock": clock})  # noqa: S102
    assert clock.bpm_start_beat > before  # вперёд: персистентные TimeVar не замерзают
    assert clock.next_bar() % 128 == 0
    assert clock.queue == []  # set_time чистит очередь — поэтому строка ДО Clock.bpm


def test_snapshot_reports_offset_inside_form():
    snap = clock_phase_snapshot(FakeClock(beat=37.3, bpm=1e-9), 128)
    assert snap == {
        "clock_beat": 37.3, "start_beat": 40.0, "form_total_beats": 128.0,
        "phase_offset_beats": 40.0,
    }


def test_snapshot_after_prelude_is_zero():
    clock = FakeClock(beat=500.9)
    exec(clock_align_prelude(128), {"Clock": clock})  # noqa: S102
    assert clock_phase_snapshot(clock, 128)["phase_offset_beats"] == 0.0


class _BrokenClock:
    meter = (4, 4)

    def now(self):
        raise RuntimeError("clock thread died")


@pytest.mark.parametrize("clock,beats", [(None, 128), (_BrokenClock(), 128), (FakeClock(), 0)])
def test_snapshot_never_raises(clock, beats):
    assert clock_phase_snapshot(clock, beats) is None


# ---------------------------------------------------------------- arrangers


def test_render_default_has_no_set_time():
    code = render(_spec())
    assert "set_time" not in code
    assert code.splitlines()[:2] == ["Clock.clear()", "Clock.bpm = 100"]


def test_render_aligned_puts_prelude_between_clear_and_bpm():
    spec = _spec()
    total = form_total_beats(spec.form, getattr(spec, "theme_bars", 0))
    lines = render(spec, align_clock=True).splitlines()
    assert lines[:3] == ["Clock.clear()", clock_align_prelude(total), "Clock.bpm = 100"]
    assert lines[-1] == f"Clock.future({total + ALIGN_LEAD_BEATS}, Clock.clear)"
    # всё остальное — байт-в-байт как без флага
    plain = render(spec).splitlines()
    assert lines[3:-1] == plain[2:-1]


def test_render_club_aligned():
    plain = render_club(seed=0).splitlines()
    lines = render_club(seed=0, align_clock=True).splitlines()
    i = lines.index("Clock.clear()")
    assert lines[i + 1] == clock_align_prelude(club_form_beats())
    assert lines[i + 2].startswith("Clock.bpm = ")
    assert lines[-1] == f"Clock.future({club_form_beats() + ALIGN_LEAD_BEATS}, Clock.clear)"
    assert [ln for ln in lines if "set_time" not in ln][:-1] == plain[:-1]


@pytest.mark.parametrize(
    "code", [render(_spec(), align_clock=True), render_club(align_clock=True)], ids=["classic", "club"],
)
def test_sanitizer_keeps_prelude_unchanged(code):
    result = renardo_sanitizer.sanitize_renando(code, 0.7)
    assert result.security_error is None
    assert not result.quality_errors
    prelude = next(ln for ln in code.splitlines() if "set_time" in ln)
    assert prelude in result.code.splitlines()


# ------------------------------------------------------------ music.py wiring


@pytest.mark.parametrize("value,expected", [("1", True), ("true", True), ("0", False), ("", False)])
def test_env_flag(monkeypatch, value, expected):
    monkeypatch.setenv("ROB_BOX_MUSIC_ALIGN_CLOCK", value)
    assert music_align_clock_enabled() is expected


def test_env_flag_default_off(monkeypatch):
    monkeypatch.delenv("ROB_BOX_MUSIC_ALIGN_CLOCK", raising=False)
    assert music_align_clock_enabled() is False


def _club_run(mock_node, clock):
    mgr = _make_manager(sc_running=True, renardo_available=True)
    mgr._renardo_context = {"Clock": clock} if clock is not None else {}
    tool = ComposeMusicTool(mock_node, mgr)
    with patch("builtins.exec") as fake_exec:
        result = tool.execute(style="club", seed=0)
    return result, fake_exec.call_args[0][0]


def test_club_default_off_reports_phase(mock_node, monkeypatch):
    monkeypatch.delenv("ROB_BOX_MUSIC_ALIGN_CLOCK", raising=False)
    result, executed = _club_run(mock_node, FakeClock(beat=37.3, bpm=1e-9))
    assert result.success, result.error
    assert "set_time" not in executed
    assert result.data["clock_phase_offset_beats"] == 40.0
    assert result.data["clock_phase"]["align_clock"] is False
    assert result.data["clock_phase"]["before_exec"]["start_beat"] == 40.0
    info = mock_node.get_logger().info_messages
    assert any("[#3112]" in m and "смещение в форме 40.0 из 128 долей" in m for m in info), info


def test_club_flag_on_executes_prelude(mock_node, monkeypatch):
    monkeypatch.setenv("ROB_BOX_MUSIC_ALIGN_CLOCK", "1")
    result, executed = _club_run(mock_node, FakeClock(beat=37.3))
    assert result.success, result.error
    assert clock_align_prelude(128) in executed.splitlines()
    assert result.data["clock_phase"]["align_clock"] is True


def test_no_clock_does_not_break_music(mock_node, monkeypatch):
    monkeypatch.delenv("ROB_BOX_MUSIC_ALIGN_CLOCK", raising=False)
    result, _ = _club_run(mock_node, None)
    assert result.success, result.error
    assert result.data["clock_phase_offset_beats"] is None


def test_broken_clock_does_not_break_music(mock_node, monkeypatch):
    monkeypatch.delenv("ROB_BOX_MUSIC_ALIGN_CLOCK", raising=False)
    result, _ = _club_run(mock_node, _BrokenClock())
    assert result.success, result.error
    assert result.data["clock_phase_offset_beats"] is None


def test_classic_compose_reports_phase_and_honours_flag(mock_node, monkeypatch):
    monkeypatch.setenv("ROB_BOX_MUSIC_ALIGN_CLOCK", "1")
    mgr = _make_manager(sc_running=True, renardo_available=True)
    mgr._renardo_context = {"Clock": FakeClock(beat=37.3)}
    tool = ComposeMusicTool(mock_node, mgr)
    with patch("builtins.exec") as fake_exec:
        result = tool.execute(
            bpm=100, root="C", scale="minor", form="arc", drums="X..o.X.o",
            bass_synth="dub", bass_notes="0, 0, 3, -2", lead_synth="blip",
            lead_notes="0, 2, 4, 7",
        )
    assert result.success, result.error
    executed = fake_exec.call_args[0][0]
    assert executed.splitlines()[1].startswith("Clock.set_time(")
    assert result.data["clock_phase"]["align_clock"] is True
    assert isinstance(result.data["clock_phase_offset_beats"], float)
