"""Предельная скорость в HIL (scripts/hil/speed/speed_verdict.py): метрики, вердикт и дихотомию решает код.

Без ROS и стенда: задачи — в формате ``result.json`` от ``speed_bench.py``.
"""

import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "hil_speed_verdict", ROOT / "scripts" / "hil" / "speed" / "speed_verdict.py"
)
sv = importlib.util.module_from_spec(_spec)
sys.modules["hil_speed_verdict"] = sv
_spec.loader.exec_module(sv)


def good_goal(**kw):
    t = {
        "name": "E090",
        "kind": "goal",
        "status": "SUCCEEDED",
        "err_xy_m": 0.2,
        "min_clearance_m": 0.12,
        "pose_err": {"n": 100, "p50": 0.01, "p95": 0.03, "max": 0.05},
        "icp": {"frames": 300, "lost": 0, "longest_lost_run": 0},
        "cmd_interval": {"n": 400, "p50": 0.05, "p95": 0.07, "max": 0.2, "gaps": 0, "gaps_per_min": 0.0},
    }
    t.update(kw)
    return t


MAIN_OK = {"cpu": {"nav2": {"n": 50, "p50": 110.0, "p95": 130.0, "max": 150.0}}, "nav2_missed": 12}


def test_criteria_numbers_are_pinned():
    c = sv.CRITERIA
    assert (c.pose_p95_max_m, c.icp_max_lost_run, c.cmd_p95_max_s, c.cmd_gap_s) == (0.10, 1, 0.10, 0.25)
    assert (c.nav2_cpu_p95_max, c.obstacle_gap_min_m, c.goal_tol_m, c.min_runs) == (200.0, 0.20, 0.35, 2)


def test_percentile_matches_linear_interpolation():
    assert sv.percentile([1, 2, 3, 4], 50) == pytest.approx(2.5)
    assert sv.percentile([0, 10], 95) == pytest.approx(9.5)
    assert sv.percentile([], 50) is None
    assert sv.spread_stats([0.1, float("nan"), 0.3]) == {"n": 2, "p50": 0.2, "p95": 0.29, "max": 0.3}


def test_pose_error_uses_spawn_as_map_origin():
    spawn = sv.Pose2D(-2.5, 1.3, 0.0)
    assert sv.pose_error((-2.5, 1.3), spawn, (0.0, 0.0, 0.0)) == pytest.approx(0.0)
    turned = sv.Pose2D(0.0, 0.0, math.pi)
    # map смотрит назад: 1 м вперёд в map — это x = −1 в мире
    assert sv.pose_error((-1.0, 0.0), turned, (1.0, 0.0, 0.0)) == pytest.approx(0.0, abs=1e-9)
    assert sv.pose_error((-1.0, 0.1), turned, (1.0, 0.0, 0.0)) == pytest.approx(0.1)


def test_lost_runs_counts_longest_series():
    assert sv.lost_runs([False, True, False, True, True, False]) == (3, 2)
    assert sv.lost_runs([]) == (0, 0)


def test_cmd_intervals_only_while_driving_forward():
    cmds = [(0.0, 0.5), (0.05, 0.5), (0.10, 0.5), (0.45, 0.5), (0.50, 0.0), (1.50, 0.0), (1.55, 0.5)]
    st = sv.cmd_interval_stats(cmds)
    assert st["n"] == 3  # 0→0.05, 0.05→0.10, 0.10→0.45; развороты (v=0) не в счёт
    assert st["gaps"] == 1 and st["max"] == pytest.approx(0.35)


def test_speeds_finite_difference():
    track = [(i * 0.05, i * 0.05 * 0.8, 0.0) for i in range(40)]
    vs = [v for _, v in sv.speeds(track)]
    assert vs and all(v == pytest.approx(0.8) for v in vs)


def _stop_track(v0=1.0, decel=1.0, gap0=1.5, still_after=2.0):
    """Едем по +X с v0, в t=0 появилась стенка в gap0 от носа, тормозим с decel, стоим still_after с."""
    out, t, x, v = [], 0.0, 0.0, v0
    dt = 0.02
    while t < v0 / decel + still_after:
        out.append((t, x, 0.0, 0.0))
        x += v * dt
        v = max(0.0, v - decel * dt)
        t += dt
    return out


def test_obstacle_stop_path_and_gap():
    wall = sv.wall_ahead(sv.Pose2D(0, 0, 0), 1.5)
    ob = sv.obstacle_metrics(_stop_track(), wall, 0.0)
    assert ob["stopped"] is True
    assert ob["gap_at_spawn_m"] == pytest.approx(1.5, abs=1e-3)
    assert ob["stop_path_m"] == pytest.approx(0.5, abs=0.03)  # v²/2a
    assert ob["stop_gap_m"] == pytest.approx(1.0, abs=0.03)
    assert ob["min_gap_m"] == pytest.approx(ob["stop_gap_m"], abs=0.01)


def test_obstacle_not_stopped_when_moving_through():
    wall = sv.wall_ahead(sv.Pose2D(0, 0, 0), 1.5)
    track = [(i * 0.05, i * 0.05, 0.0, 0.0) for i in range(100)]
    ob = sv.obstacle_metrics(track, wall, 0.0)
    assert ob["stopped"] is False and ob["min_gap_m"] < 0


def test_good_goal_passes():
    assert sv.task_failures(good_goal()) == []
    assert sv.run_verdict([good_goal()], MAIN_OK) == (sv.PASS, [])


@pytest.mark.parametrize(
    "kw, needle",
    [
        ({"status": "TIMEOUT"}, "TIMEOUT"),
        ({"err_xy_m": 0.5}, "ошибка по правде"),
        ({"pose_err": {"p95": 0.12}}, "ошибка позы p95"),
        ({"icp": {"longest_lost_run": 2}}, "срыв ICP"),
        ({"cmd_interval": {"p95": 0.15, "gaps": 0, "gaps_per_min": 0}}, "такт контроллера"),
        ({"min_clearance_m": -0.002}, "касание"),
    ],
)
def test_goal_failures(kw, needle):
    reasons = sv.task_failures(good_goal(**kw))
    assert any(needle in r for r in reasons), reasons


def test_cmd_gap_series_counted_over_the_whole_run():
    short = good_goal(cmd_interval={"p95": 0.06, "gaps": 1, "gaps_per_min": 9.3, "moving_s": 6.5})
    long_ = good_goal(name="DE", cmd_interval={"p95": 0.06, "gaps": 0, "gaps_per_min": 0.0, "moving_s": 120.0})
    # одна серия на 126.5 с движения — 0.47 в минуту: не провал, хотя короткая задача дала бы «9.3 в минуту»
    assert sv.run_verdict([short, long_], MAIN_OK) == (sv.PASS, [])
    many = good_goal(cmd_interval={"p95": 0.06, "gaps": 3, "gaps_per_min": 3.0, "moving_s": 60.0})
    v, reasons = sv.run_verdict([many], MAIN_OK)
    assert v == sv.FAIL and any("серии пропусков" in r for r in reasons)


def test_single_icp_blip_with_reset_is_not_a_failure():
    assert sv.task_failures(good_goal(icp={"frames": 300, "lost": 1, "longest_lost_run": 1})) == []


def test_skipped_goal_is_not_judged_alone():
    assert sv.task_failures({"name": "W090", "kind": "goal", "status": "SKIPPED"}) == []


def test_obstacle_verdict():
    base = good_goal(kind="obstacle", status="CANCELED_BY_BENCH", err_xy_m=9.0, name="OBST")
    ob = {
        "spawned": True,
        "stopped": True,
        "min_gap_m": 0.30,
        "guard_margin_m": 0.10,
        "guard_gap_m": 0.30,
        "guard_limit_m": 0.20,
    }
    assert sv.task_failures(dict(base, obstacle=ob)) == []
    # (б) в коридоре хода ближе 0.20 — провал
    close = dict(ob, min_gap_m=0.12, guard_margin_m=-0.08, guard_gap_m=0.12, guard_limit_m=0.20)
    assert any("по правилу защиты: зазор 0.120" in r for r in sv.task_failures(dict(base, obstacle=close)))
    # (а) 0.16 м до торца сбоку, (б) запас против 0.08 положительный — не провал (#4631)
    side = dict(ob, min_gap_m=0.16, guard_margin_m=0.08, guard_gap_m=0.16, guard_limit_m=0.08)
    assert sv.task_failures(dict(base, obstacle=side)) == []
    through = dict(ob, stopped=False, min_gap_m=-0.01)
    assert any("касание стенки" in r for r in sv.task_failures(dict(base, obstacle=through)))
    no_track = {"spawned": True, "min_gap_m": 0.3}
    assert any("правилу защиты" in r for r in sv.task_failures(dict(base, obstacle=no_track)))
    assert sv.task_failures(dict(base, obstacle={"spawned": False})) == ["стенка не появилась"]


def _approach(lateral_m, stop_x, v=0.25):
    """Едем по +X к x=stop_x и стоим; стенка поперёк — по guard-модулю репо."""
    out, t, x = [], 0.0, 0.0
    while t < 12.0:
        out.append((round(t, 2), x, lateral_m, 0.0))
        x = min(stop_x, x + v * 0.1)
        t += 0.1
    return out


def test_guard_rule_gap_corridor_vs_side():
    guard = sv.load_guard()
    _, body, th = guard
    wall = sv.wall_ahead(sv.Pose2D(0, 0, 0), 2.0)  # ближняя грань на x = 0.3064 + 2.0, ширина 3 м по y
    face = wall.cx - wall.sx / 2
    # прямо на стенку, встал в 0.12 от неё: в коридоре хода, порог stop_m
    r = sv.guard_rule_gap(_approach(0.0, face - body.x_max - 0.12), wall, 0.0, 99.0, guard)
    assert r["guard_limit_m"] == pytest.approx(th.stop_m)
    assert r["guard_gap_m"] == pytest.approx(0.12, abs=0.01) and r["guard_margin_m"] < 0
    # проехал мимо торца стенки в 0.15 сбоку: порог side_stop_m, запас положительный
    lateral = wall.cy + wall.sy / 2 + 0.15 - body.y_min
    r = sv.guard_rule_gap(_approach(lateral, face + 1.0), wall, 0.0, 99.0, guard)
    assert r["guard_limit_m"] == pytest.approx(th.side_stop_m)
    assert r["guard_gap_m"] == pytest.approx(0.15, abs=0.01) and r["guard_margin_m"] > 0


def test_run_verdict_cpu_and_missing_data():
    hot = {"cpu": {"nav2": {"p95": 250.0}}}
    v, reasons = sv.run_verdict([good_goal()], hot)
    assert v == sv.FAIL and any("CPU nav2" in r for r in reasons)
    v, reasons = sv.run_verdict([], MAIN_OK)
    assert v == sv.FAIL
    v, reasons = sv.run_verdict([good_goal()], {})
    assert v == sv.FAIL and "нет замера CPU Main Pi" in reasons


def row(speed, scene, verdict, rep=1):
    return {"speed": speed, "scene": scene, "verdict": verdict, "rep": rep}


def test_point_verdicts_need_two_runs_of_each_scene():
    led = [row(0.5, "polygon", "PASS"), row(0.5, "turn90", "PASS"), row(0.5, "polygon", "PASS", 2)]
    assert sv.point_verdicts(led, "tight") == {0.5: "PENDING"}
    led.append(row(0.5, "turn90", "PASS", 2))
    assert sv.point_verdicts(led, "tight") == {0.5: "PASS"}
    led.append(row(1.0, "turn90", "FAIL"))
    assert sv.point_verdicts(led, "tight")[1.0] == "FAIL"
    assert sv.point_verdicts(led, "open") == {}


def test_next_speed_halves_then_bisects_on_grid():
    assert sv.next_speed({}) == (1.0, "старт")
    assert sv.next_speed({1.0: "FAIL"})[0] == 0.5
    assert sv.next_speed({1.0: "FAIL", 0.5: "FAIL"})[0] == 0.25
    assert sv.next_speed({1.0: "FAIL", 0.5: "PASS"})[0] == 0.75
    assert sv.next_speed({1.0: "FAIL", 0.5: "PASS", 0.75: "FAIL"})[0] == 0.6
    assert sv.next_speed({1.0: "FAIL", 0.6: "PASS", 0.75: "FAIL"})[0] == 0.65
    assert sv.next_speed({1.0: "FAIL", 0.65: "PASS", 0.75: "FAIL"})[0] == 0.7
    v, why = sv.next_speed({1.0: "FAIL", 0.65: "PASS", 0.7: "FAIL"})
    assert v is None and "0.65 проходит" in why
    v, why = sv.next_speed({1.0: "PASS"})
    assert v is None and "проходит 1.0" in why
    assert sv.next_speed({0.5: "PENDING", 1.0: "FAIL"}) == (0.5, "точка не досчитана")
    v, why = sv.next_speed({0.05: "FAIL"})
    assert v is None and "не проходит ни одна" in why


def test_next_run_repeats_only_the_failed_scene():
    assert sv.next_run([], "tight", 1.0) == ("turn90", 1)
    led = [row(1.0, "turn90", "FAIL")]
    assert sv.next_run(led, "tight", 1.0) == ("turn90", 2)
    led.append(row(1.0, "turn90", "PASS", 2))
    assert sv.next_run(led, "tight", 1.0) is None  # провал есть, воспроизведение сделано
    led = [row(0.5, "turn90", "PASS"), row(0.5, "turn90", "PASS", 2)]
    assert sv.next_run(led, "tight", 0.5) == ("polygon", 1)


def test_step_walks_the_dichotomy():
    led = []
    assert sv.step(led, "open")[:3] == (1.0, "depot", 1)
    led += [row(1.0, "depot", "FAIL")]
    assert sv.step(led, "open")[:3] == (1.0, "depot", 2)
    led += [row(1.0, "depot", "FAIL", 2)]
    assert sv.step(led, "open")[:3] == (0.5, "depot", 1)
    led += [row(0.5, "depot", "PASS"), row(0.5, "depot", "PASS", 2)]
    assert sv.step(led, "open")[:3] == (0.75, "depot", 1)


def test_parse_cpu_log():
    text = "T 1\nL 3.10\nS nav2 120.5%\nS rtabmap 40%\nT 3\nL 5,20\nS nav2 140.5%\nS rtabmap --\n"
    d = sv.parse_cpu_log(text)
    assert d["load1"]["max"] == pytest.approx(5.2)
    assert d["cpu"]["nav2"]["n"] == 2 and d["cpu"]["nav2"]["max"] == pytest.approx(140.5)
    assert d["cpu"]["rtabmap"]["n"] == 1


def test_cli_run_writes_ledger(tmp_path):
    res = tmp_path / "result.json"
    res.write_text(json.dumps([good_goal()]), encoding="utf-8")
    main = tmp_path / "main.json"
    main.write_text(json.dumps(MAIN_OK), encoding="utf-8")
    led = tmp_path / "ledger.jsonl"
    rc = sv.main(["run", str(res), str(main), "--speed", "0.5", "--scene", "turn90", "--ledger", str(led)])
    assert rc == 0
    rows = [json.loads(x) for x in led.read_text(encoding="utf-8").splitlines()]
    assert rows == [{"speed": 0.5, "scene": "turn90", "rep": 1, "label": "", "verdict": "PASS", "reasons": []}]
