"""Issue #3154 — модель громкости classic и калибровка секций формы.

Числа — модель и офлайн-рендер (``scripts/music/club_loudness_nrt.py``),
НЕ замер на роботе. ``fixtures/classic_loudness_nrt.json`` — NRT-рендер
пяти тем golden-фикстуры (путь ``render_case``, форма ``arc``, синты как
у LLM) до и после калибровки, по секциям формы, dB RMS. Тесты держат:

* модель по коду сходится с рендером (до и после калибровки);
* после калибровки основной блок (секции с ударными) — на уровне club,
  тихие секции не ниже него больше чем на 8 dB, если слои не упёрлись в
  потолок;
* ручка ``levels`` работает поверх калибровки как раньше (×0.5 = вдвое);
* синт с обрывом громкости (ambi за hpf) не поднимается выше порога.
"""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

import pytest

from rob_box_mcp_tools.core import arranger as A
from rob_box_mcp_tools.core import classic_loudness as C
from rob_box_mcp_tools.core import _classic_loudness_table as T
from rob_box_mcp_tools.core.club_loudness import TARGET_MAIN_DB
from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params, rtttl_to_melody

_FIXTURES = Path(__file__).parent / "fixtures"
NRT = json.loads((_FIXTURES / "classic_loudness_nrt.json").read_text(encoding="utf-8"))
GOLDEN = {c["key"]: c["rtttl"] for c in json.loads(
    (_FIXTURES / "arranger_golden.json").read_text(encoding="utf-8"))["cases"]}
#: ambi (пэд) и marimba (лид) — синты со случайным возбуждением: модель
#: по сетке нот ошибается на их тихих секциях до ~7 dB (см. PR).
ERRATIC = {"hallofth"}
AMP = re.compile(r"\bamp=(var\(\[([^\]]*)\], \[[^\]]*\]\)|([0-9.]+))")


def _spec(key: str, **overrides):
    synths = dict(NRT.get(key, {}).get("synths", {}), **overrides)
    params = melody_to_compose_params(rtttl_to_melody(GOLDEN[key]), drum_style="auto")
    return A.spec_from_flat(
        harmony=params["harmony"], bpm=float(params["bpm"]), root=str(params["root"]),
        scale=str(params["scale"]), lead_midi=str(params["lead_midi"]),
        lead_dur=str(params["lead_dur"]), form="arc", repeat=False, **synths,
    )


def _bounds(spec):
    plan = A.resolve_form(spec.form, spec.theme_bars)
    bounds = [0.0]
    for _n, bars, _i in plan:
        bounds.append(bounds[-1] + bars * A.BEATS_PER_BAR)
    main = [i for i, (_n, _b, it) in enumerate(plan) if A._section_intensity("drums", it) > 0]
    return plan, bounds, main


def _model(key: str, calibrate: bool):
    spec = _spec(key)
    _plan, bounds, main = _bounds(spec)
    bpm = max(A.BPM_RANGE[0], min(A.BPM_RANGE[1], float(spec.bpm)))
    code = A.render(spec, calibrate=calibrate)
    model = C.form_model(C.robot_code(code), bounds, bpm)
    return [s.actual_db() for s in model.sections], main, model


def _mean(levels, sections, weights):
    power = sum(10 ** (levels[i] / 10) * weights[i] for i in sections)
    return 10 * math.log10(power / sum(weights[i] for i in sections))


# ── таблица ────────────────────────────────────────────────────────────


def test_table_covers_the_synth_palette():
    from unittest.mock import MagicMock

    for mod in ("rclpy", "rclpy.node", "rclpy.action", "rclpy.qos", "std_msgs", "std_msgs.msg",
                "geometry_msgs", "geometry_msgs.msg", "nav2_msgs", "nav2_msgs.action",
                "action_msgs", "action_msgs.srv", "action_msgs.msg"):
        sys.modules.setdefault(mod, MagicMock())
    from rob_box_mcp_tools.tools.music import CRITICAL_SYNTHS

    for synth in CRITICAL_SYNTHS:
        name = C.SYNTH_ALIASES.get(synth, synth)
        if name in ("loop", "piano", "play1", "play2"):  # сэмплеры, не тембры
            continue
        assert name in T.NOTE_DB, synth
        assert set(T.NOTE_DB[name]) == set(T.FILTERS)
        assert T.EXPONENT[name] in (1, 2)
    for symbol in ("X", "x", "o", "O", "-", "=", "*"):
        assert len(T.DRUM_DB[symbol]) == 4


def test_ambi_behind_hpf_is_capped_below_its_collapse():
    """Рендер: ambi за hpf 261 при amp ≥ 0.15 обрывается до −110 dB."""
    assert C.safe_amp("ambi", {"hpf": 261.6}) <= 0.1
    assert C.safe_amp("ambi", {}) == C.MAX_AMP
    assert C.safe_amp("pluck", {"hpf": 523.3, "lpf": 2000.0, "lpf_sweep": True}) == C.MAX_AMP


def test_note_energy_scales_with_amp_by_exponent():
    base = C.note_db("pluck", 60, 0.5, 0.1, {})
    assert C.note_db("pluck", 60, 0.5, 0.2, {}) - base == pytest.approx(20 * math.log10(2), abs=1e-6)
    sub = C.note_db("subbass", 60, 0.5, 0.1, {})
    assert C.note_db("subbass", 60, 0.5, 0.2, {}) - sub == pytest.approx(40 * math.log10(2), abs=1e-6)
    assert C.note_db("pianovel", 60, 0.5, 0.1, {}) == C.note_db("rhpiano", 60, 0.5, 0.1, {})
    assert C.note_db("no_such_synth", 60, 0.5, 0.1, {}) is None


def test_held_synth_sounds_up_to_eight_sus():
    """makeSound renardo держит узел sus·8: moogbass звучит ~7.5·sus, pluck — меньше sus."""
    assert C.sounding_s("moogbass", 0.5) == pytest.approx(3.8, abs=0.3)
    assert C.sounding_s("pluck", 0.5) < 0.5


# ── модель против офлайн-рендера ───────────────────────────────────────


@pytest.mark.parametrize("key", sorted(NRT))
@pytest.mark.parametrize("stage", ["before", "after"])
def test_model_matches_offline_render(key, stage):
    model, _main, _m = _model(key, calibrate=stage == "after")
    rendered = NRT[key][f"nrt_{stage}"]
    tolerance = 7.0 if key in ERRATIC else 2.1
    assert max(abs(a - b) for a, b in zip(model, rendered)) <= tolerance, (model, rendered)


@pytest.mark.parametrize("key", sorted(NRT))
def test_rendered_main_block_lands_on_club_level(key):
    """Офлайн-рендер после калибровки: основной блок classic = уровень club ±1.5 dB."""
    spec = _spec(key)
    plan, _bounds_, main = _bounds(spec)
    weights = [bars for _n, bars, _i in plan]
    assert _mean(NRT[key]["nrt_after"], main, weights) == pytest.approx(TARGET_MAIN_DB, abs=1.5)
    assert _mean(NRT[key]["nrt_before"], main, weights) < TARGET_MAIN_DB + 1.0


@pytest.mark.parametrize("key", sorted(set(NRT) - ERRATIC))
def test_rendered_quiet_sections_within_10db(key):
    rendered = NRT[key]["nrt_after"]
    assert TARGET_MAIN_DB - min(rendered) <= 10.0
    assert TARGET_MAIN_DB - min(NRT[key]["nrt_before"]) > 13.0  # до калибровки было хуже


# ── калибровка ─────────────────────────────────────────────────────────


@pytest.mark.parametrize("key", sorted(GOLDEN))
def test_calibration_puts_main_on_target_and_lifts_quiet_sections(key):
    spec = _spec(key, lead_synth="blip", bass_synth="moogbass", pad_synth="strings")
    plan, bounds, main = _bounds(spec)
    bpm = max(A.BPM_RANGE[0], min(A.BPM_RANGE[1], float(spec.bpm)))
    cal = C.section_gains(A.render(spec, calibrate=False), bounds, bpm, main)
    assert cal is not None and cal.unmodeled_events == 0
    assert cal.main_after_db == pytest.approx(TARGET_MAIN_DB, abs=0.5)
    assert min(cal.after_db) >= TARGET_MAIN_DB - C.SECTION_FLOOR_DB - 0.2
    code = A.render(spec)
    for match in AMP.finditer(code):
        values = (match.group(2) or match.group(3)).split(",")
        assert max(float(v) for v in values) <= C.MAX_AMP + 1e-9


def test_levels_knob_still_scales_calibrated_amp_exactly():
    spec = _spec("furelise")
    half = _spec("furelise")
    half.levels = {"bass": 0.5}
    full_amps = [float(v) for v in AMP.search(_line(A.render(spec), "p1")).group(2).split(",")]
    half_amps = [float(v) for v in AMP.search(_line(A.render(half), "p1")).group(2).split(",")]
    assert half_amps == [pytest.approx(a / 2, abs=1e-4) for a in full_amps]
    assert _line(A.render(half), "p2") == _line(A.render(spec), "p2")


def test_calibration_changes_only_amp_values():
    spec = _spec("tetris")
    raw, calibrated = A.render(spec, calibrate=False), A.render(spec)
    assert raw != calibrated
    assert AMP.sub("amp=_", raw) == AMP.sub("amp=_", calibrated)


def test_unmodelable_program_keeps_code_uncalibrated():
    assert C.section_gains("p1 >> pluck([0], dur=1, amp=P[1, 2])", [0.0, 4.0], 120, [0]) is None
    assert C.section_gains("p1 >> nosuch([60], dur=1, amp=0.3)", [0.0, 4.0], 120, [0]) is None


def _line(code: str, player: str) -> str:
    return next(line for line in code.splitlines() if line.startswith(f"{player} >>"))


if __name__ == "__main__":  # pragma: no cover
    sys.exit(pytest.main([__file__, "-v"]))
