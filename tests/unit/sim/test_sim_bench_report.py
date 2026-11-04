"""rob_box_sim.bench_report: сводка повторов стенда по конфигурациям, без ROS."""

from __future__ import annotations

import json

from rob_box_sim.bench_report import aggregate, config_of, load_rows, markdown, totals


def _goal(goal, status, time_s=20.0, clear=0.2, osc=10.0, osc_raw=None, rec=0, coll=False, tilt=1.0, rtf=None):
    ran = status != "SKIPPED"
    return {
        "world": "polygon", "goal": goal, "status": status, "success": status == "SUCCEEDED",
        "time_s": time_s if ran else None, "min_clearance_m": clear if ran else None,
        "osc_per_m": osc if ran else None, "osc_raw_per_m": osc_raw, "recoveries": rec if ran else None,
        "collision": coll if ran else None, "tilt_max_deg": tilt if ran else None, "rtf": rtf,
    }


def _write(tmp_path, name, label, goals):
    d = tmp_path / name
    d.mkdir()
    (d / "polygon.json").write_text(json.dumps({"world": "polygon", "label": label, "goals": goals}))
    return d


def test_config_strips_repeat_index():
    assert config_of("dwb-navfn@3") == "dwb-navfn"
    assert config_of("baseline") == "baseline"


def test_repeats_fold_into_one_row_and_skipped_is_not_an_attempt(tmp_path):
    d1 = _write(tmp_path, "r1", "rpp@1", [_goal("gap_1.2", "SUCCEEDED", 20.0, 0.30, 4.0, 6.0),
                                          _goal("gap_0.9", "TIMEOUT", 150.0, 0.05, 8.0, rec=3)])
    d2 = _write(tmp_path, "r2", "rpp@2", [_goal("gap_1.2", "SUCCEEDED", 30.0, 0.20, 2.0, 2.0),
                                          _goal("gap_0.9", "SKIPPED")])
    d3 = _write(tmp_path, "r3", "rpp@3", [_goal("gap_1.2", "FAILED", 90.0, 0.10, 6.0, coll=True, tilt=9.5),
                                          _goal("gap_0.9", "SKIPPED")])
    agg = aggregate(load_rows([d1, d2, d3]))
    by_goal = {a["goal"]: a for a in agg}
    a = by_goal["gap_1.2"]
    assert (a["config"], a["ok"], a["ran"], a["skipped"]) == ("rpp", 2, 3, 0)
    assert a["time_med_s"] == 25.0  # только успешные
    assert a["clear_min_m"] == 0.10 and a["clear_med_m"] == 0.20
    assert a["osc_med"] == 4.0 and a["osc_raw_med"] == 4.0
    assert a["collisions"] == 1 and a["tilt_max_deg"] == 9.5
    b = by_goal["gap_0.9"]
    assert (b["ok"], b["ran"], b["skipped"]) == (0, 1, 2)
    assert b["time_med_s"] is None and b["rec_max"] == 3
    assert totals(agg) == {"rpp": (2, 4)}
    md = markdown(agg)
    assert "| rpp | polygon | gap_1.2 | 2/3 |" in md
    assert "| rpp | polygon | gap_0.9 | 0/1 (+2 SKIPPED) | — |" in md
    assert "| rpp | 2/4 |" in md


def test_rtf_marks_loaded_and_unmeasured_attempts(tmp_path):
    d1 = _write(tmp_path, "r1", "dwb@1", [_goal("gap_1.2", "SUCCEEDED", rtf=0.95)])
    d2 = _write(tmp_path, "r2", "dwb@2", [_goal("gap_1.2", "SUCCEEDED", rtf=0.4)])
    d3 = _write(tmp_path, "r3", "dwb@3", [_goal("gap_1.2", "SUCCEEDED")])
    old = _write(tmp_path, "r4", "old@1", [_goal("gap_1.2", "SUCCEEDED")])
    agg = {a["config"]: a for a in aggregate(load_rows([d1, d2, d3, old]))}
    assert (agg["dwb"]["rtf_min"], agg["dwb"]["rtf_low"], agg["dwb"]["rtf_unknown"]) == (0.4, 1, 1)
    md = markdown(list(agg.values()))
    assert "| 0.40 ⚠ 1<0.8 (1 не мерян) |" in md
    assert "| не мерян |" in md
