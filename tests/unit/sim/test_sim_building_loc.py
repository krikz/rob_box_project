"""Прогон здания (#4468): вердикты локализации — нашёл себя, ложный скачок, дрейф, голос — на синтетике.

Стоят на rob_box_sim.loc_metrics стенда #4346 (PR #4568). Пока его нет в develop, модуль пропускается
(pytest -rs покажет причину); после влития #4568 тесты идут в CI как обычные.
"""

from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path

import pytest

lm = pytest.importorskip("rob_box_sim.loc_metrics", reason="нужен rob_box_sim.loc_metrics из PR #4568 (#4346)")

from rob_box_sim import building_run as br  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
S = lm.Sample


def line_samples(t0, t1, x0, x1, y=0.0, est_offset=(0.0, 0.0), dt=0.1, est=True):
    """Робот едет по прямой y из x0 в x1; оценка = истина + сдвиг (или нет оценки)."""
    n = int(round((t1 - t0) / dt))
    out = []
    for i in range(n + 1):
        t = t0 + i * dt
        x = x0 + (x1 - x0) * i / n
        truth = (x, y, 0.0)
        e = (x + est_offset[0], y + est_offset[1], 0.0) if est else None
        out.append(S(round(t, 3), truth, e))
    return out


def test_fix_found_late_wrong_none():
    crit = br.Criteria(found_budget_s=20.0)
    # оценка 3 м мимо 10 с, потом совпадает
    found = line_samples(0, 10, 0, 1, est_offset=(3.0, 0.0)) + line_samples(10.1, 40, 1, 4)
    fix = br.fix_result(found, 0.0, 40.0, crit)
    assert fix.status == br.FOUND
    assert fix.err0_m == pytest.approx(3.0) and fix.found_s == pytest.approx(10.1, abs=0.15)
    assert fix.found_path_m == pytest.approx(1.0, abs=0.05) and fix.err_final_m == pytest.approx(0.0)
    late = line_samples(0, 30, 0, 1, est_offset=(3.0, 0.0)) + line_samples(30.1, 60, 1, 2)
    assert br.fix_result(late, 0.0, 60.0, crit).status == br.LATE
    # «узнал» соседний одинаковый кабинет: 4 м мимо до конца
    assert br.fix_result(line_samples(0, 30, 30, 31, est_offset=(4.0, 0.0)), 0.0, 30.0, crit).status == br.WRONG
    # в допуске не держится, но и не уверенно мимо
    wobble = [S(i * 0.1, (0.0, 0.0, 0.0), (0.5 if i % 20 < 10 else 0.0, 0.0, 0.0)) for i in range(300)]
    assert br.fix_result(wobble, 0.0, 30.0, crit).status == br.NOT_FOUND
    assert br.fix_result(line_samples(0, 10, 0, 1, est=False), 0.0, 10.0, crit).status == br.NO_ESTIMATE


def test_jumps_false_fix_and_not_jumps():
    crit = br.Criteria()
    base = line_samples(0, 5, 30, 30.5)  # точно
    false = [S(5.1, (30.5, 0.0, 0.0), (34.5, 0.0, 0.0))] + line_samples(5.2, 8, 30.5, 30.8, est_offset=(4.0, 0.0))
    back = [S(8.1, (30.8, 0.0, 0.0), (30.8, 0.0, 0.0))] + line_samples(8.2, 10, 30.8, 31.0)
    jumps = br.jump_events(base + false + back, 0.0, 10.0, crit)
    assert [(j.kind, j.similar) for j in jumps] == [("false", True), ("fix", True)]
    assert jumps[0].err_before == pytest.approx(0.0) and jumps[0].err_after == pytest.approx(4.0)
    # телепорт истины — не скачок оценки
    tp = line_samples(0, 2, 0, 0.2) + [S(2.1, (8.0, 0.0, 0.0), (0.2, 0.0, 0.0))]
    assert br.jump_events(tp, 0.0, 3.0, crit) == []
    # первая оценка после разрыва (рестарт стека) — не скачок, её судит fix_result
    gap = line_samples(0, 2, 0, 0.2) + [S(2.1, (0.2, 0.0, 0.0), None), S(5.0, (0.2, 0.0, 0.0), (6.0, 0.0, 0.0))]
    assert br.jump_events(gap, 0.0, 6.0, crit) == []


def test_drift_stats():
    s = line_samples(0, 10, 0, 10, est_offset=(0.0, 0.2)) + line_samples(10.1, 11, 10, 10.1, est_offset=(0.0, 0.9))
    st = br.drift_stats(s, 0.0, 11.0)
    assert st["max_m"] == pytest.approx(0.9) and st["mean_m"] == pytest.approx(0.26, abs=0.02)
    assert st["path_m"] == pytest.approx(10.1, abs=0.05)
    assert br.drift_stats(line_samples(0, 1, 0, 1, est=False), 0, 1) == {"samples": 0}


def ev(event, sim, **kv):
    return br.event_line(event, sim, **kv)


GOAL_END = (
    "12:00:01.000 GOAL_END {name} result=STATUS_SUCCEEDED sim={sim} frame=-1 dur_sim=30.0 err={err} em_episodes=0 "
    "recoveries=0 gt=(1, 2, 3)"
)


def synthetic_run():
    """Карта, рестарт, маршрут, старт из случайной точки (нашёл себя), похищение (ложное узнавание), голос."""
    lines = [
        ev("SEG_START", 0.0, name="map", kind="map"),
        GOAL_END.format(name="aud_101", sim=20.0, err=0.2),
        ev("SEG_END", 30.0, name="map"),
        ev("SEG_START", 30.0, name="restart_after_map", kind="restart"),
        ev("MARK", 35.0, name="restart_end"),
        ev("SEG_END", 60.0, name="restart_after_map"),
        ev("SEG_START", 60.0, name="route", kind="route"),
        GOAL_END.format(name="dean", sim=90.0, err=0.3),
        GOAL_END.format(name="room_201", sim=110.0, err=1.8),
        ev("SEG_END", 120.0, name="route"),
        ev("SEG_START", 120.0, name="start_1", kind="start", x=40.0, y=0.0, yaw=0.0),
        ev("TELEPORT", 121.0, ok=1, x=40.0, y=0.0),
        ev("MARK", 130.0, name="restart_end"),
        ev("SEG_END", 170.0, name="start_1"),
        ev("SEG_START", 170.0, name="kidnap_similar", kind="kidnap"),
        ev("TELEPORT", 175.0, ok=1, x=45.0, y=0.0),
        ev("SEG_END", 210.0, name="kidnap_similar"),
        ev("SEG_START", 210.0, name="voice_1", kind="voice", target="dean"),
        ev("SEG_END", 260.0, name="voice_1"),
    ]
    samples = (
        line_samples(0, 30, -1, 5.6)
        + line_samples(30.1, 60, 5.6, 5.6)
        + line_samples(60.1, 120, 5.6, 30.0, est_offset=(0.0, 0.3))
        + line_samples(120.1, 140, 40.0, 40.0, est_offset=(-8.0, 0.0))  # после старта 8 м мимо…
        + line_samples(140.1, 170, 40.0, 41.0)  # …нашёл себя через 10 с после рестарта
        + line_samples(170.1, 175, 37.0, 37.5)
        + line_samples(175.1, 210, 45.0, 46.0, est_offset=(-8.0, 0.0))  # похищен: оценка осталась на 37
        + line_samples(210.1, 260, 20.0, 15.1, y=-2.6)  # голос: доехал до деканата
    )
    return [x + "\n" for x in lines], samples


def test_evaluate_blocks():
    lines, samples = synthetic_run()
    events, goals = br.parse_events(lines), br.parse_goal_ends(lines)
    v = {x.block: x for x in br.evaluate(events, goals, samples)}
    assert list(v) == ["map", "registry", "route", "drift", "restart", "global_loc", "similar", "kidnap", "voice"]
    assert v["map"].status == br.PASS
    assert v["registry"].status == br.NOT_TRIGGERED
    assert v["route"].status == br.FAIL and "FALSE_SUCCESS: room_201" in v["route"].line()
    assert v["drift"].status == br.PASS and "макс 0.3 м" in v["drift"].line()
    assert v["restart"].status == br.PASS
    assert v["global_loc"].status == br.PASS and "start_1 @(40.000,0.000) похожее: FOUND" in v["global_loc"].line()
    assert v["kidnap"].status == br.FAIL and "kidnap_similar: WRONG" in v["kidnap"].line()
    assert v["voice"].status == br.PASS
    assert v["similar"].status == br.PASS  # телепорт и рестарт не скачки


def write_run(tmp_path: Path):
    lines, samples = synthetic_run()
    (tmp_path / "steps").mkdir()
    (tmp_path / "steps" / "001_all.log").write_text("".join(lines), encoding="utf-8")
    (tmp_path / "maprec_loc").mkdir()
    with open(tmp_path / "maprec_loc" / "poses.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "tx", "ty", "tyaw", "ex", "ey", "eyaw"])
        for s in samples:
            e = [f"{v:.4f}" for v in s.estimate] if s.estimate else ["", "", ""]
            w.writerow([f"{s.t:.3f}", *[f"{v:.4f}" for v in s.truth], *e])
    (tmp_path / "verdicts.txt").write_text("VERDICT - preflight PASS: stand.lock: shidi\n", encoding="utf-8")
    return tmp_path


def test_cli_verdicts_reads_run_dir(tmp_path, capsys):
    out = write_run(tmp_path)
    assert br.main(["verdicts", str(out)]) == 0
    printed = capsys.readouterr().out
    assert "VERDICT #4468 kidnap FAIL" in printed and "VERDICT #4468 global_loc PASS" in printed


def test_report_script_writes_verdicts_md_and_gif(tmp_path):
    pytest.importorskip("PIL")
    out = write_run(tmp_path)
    spec = importlib.util.spec_from_file_location("building_report", REPO / "scripts/hil/building/building_report.py")
    rep = importlib.util.module_from_spec(spec)
    sys.modules["building_report"] = rep
    spec.loader.exec_module(rep)
    assert rep.main([str(out), "--frames", "3"]) == 0
    verdicts = (out / "verdicts.txt").read_text(encoding="utf-8").splitlines()
    assert verdicts[0].startswith("VERDICT - preflight PASS")  # предусловия сохранены
    assert sum(x.startswith("VERDICT #4468 ") for x in verdicts) == 9
    md = (out / "report.md").read_text(encoding="utf-8")
    assert "| start_1 | start |" in md
    gifs = sorted(p.name for p in (out / "anim").glob("*.gif"))
    assert gifs == sorted(
        f"{n}.gif" for n in ("map", "restart_after_map", "route", "start_1", "kidnap_similar", "voice_1")
    )
    assert all((out / "anim" / g).stat().st_size > 0 for g in gifs)
