"""Прогон здания (#4468, шаг 2): план прогона, разбор лога шагов и вердикт «доехал» — без ROS и стенда.

Локализационные вердикты (нашёл себя, скачки, дрейф) — в test_sim_building_loc.py: им нужен loc_metrics (#4568).
"""

from __future__ import annotations

import builtins
import json
import math
import py_compile
from pathlib import Path

import pytest
from rob_box_sim.geometry2d import Pose2D, clearance

from rob_box_sim import building_floor as bf
from rob_box_sim import building_run as br

REPO = Path(__file__).resolve().parents[3]


# ── план ─────────────────────────────────────────────────────────────────────────────────────


def test_explore_tour_visits_every_point_once_and_returns_home():
    tour = br.explore_tour()
    names = [p.name for p in bf.named_points()]
    assert tour[-1] == br.HOME
    assert sorted(tour[:-1]) == sorted(n for n in names if n != br.HOME)
    assert len(set(tour[:-1])) == len(tour) - 1


def test_explore_tour_goes_east_without_doubling_back():
    """Ближайший сосед по коридору: x целей растёт (кроме холла в начале и возврата) — объезд, а не зигзаг."""
    pts = br.points_by_name()
    xs = [pts[n].x for n in br.explore_tour()[:-1] if pts[n].x >= bf.CORRIDOR_X0]
    back = sum(max(0.0, a - b) for a, b in zip(xs, xs[1:]))
    assert back <= 2.0, xs  # отступ назад не больше одного шага (конец коридора между 205/211 и 206/212)


def test_tour_files_names_match_goals_yaml():
    """Имена тура — те же, что route.py прочтёт из building.goals.yaml."""
    import yaml

    doc = yaml.safe_load((REPO / "src/rob_box_sim/worlds/building.goals.yaml").read_text(encoding="utf-8"))
    assert set(br.explore_tour()) <= {g["name"] for g in doc["goals"]}


def test_corridor_cost_rooms_and_hall():
    pts = br.points_by_name()
    a, b = pts["room_201"], pts["room_207"]  # напротив через коридор
    assert br.corridor_cost(a, b) == pytest.approx(abs(a.y) + abs(b.y))
    h, e = pts["hall"], pts["entrance"]  # оба в холле — по прямой
    assert br.corridor_cost(h, e) == pytest.approx(math.hypot(h.x - e.x, h.y - e.y))
    assert br.corridor_cost(h, a) == pytest.approx(br.corridor_cost(a, h))


def test_nearest_point():
    assert br.nearest_point(15.0, -3.0) == "dean"
    assert br.nearest_point(50.0, 0.2) == "corridor_end"


def test_similar_span_is_east_wing():
    assert br.similar_span() == (28.0, 52.0)
    assert br.in_similar(40.0) and not br.in_similar(10.0)


def test_random_starts_free_spread_and_deterministic():
    starts = br.random_starts(8, seed=7)
    assert starts == br.random_starts(8, seed=7)
    assert starts != br.random_starts(8, seed=8)
    boxes = bf.floor_boxes()
    lo, _ = br.similar_span()
    for i, p in enumerate(starts):
        assert clearance(p, boxes) >= 0.35
        assert (p.x >= lo) == (i % 2 == 1), (i, p)  # чётные — запад, нечётные — одинаковое крыло
        assert bf.X_WEST < p.x < bf.X_EAST and abs(p.y) < bf.Y_OUT
        assert math.hypot(p.x, p.y) >= 6.0  # не у начала map: там «нашёл себя» даром
    for i, p in enumerate(starts):
        for q in starts[i + 1 :]:  # noqa: E203
            assert p.distance_to(q) >= 6.0


def test_random_starts_impossible_raises():
    with pytest.raises(ValueError):
        br.random_starts(3, min_clear=50.0)


def test_similar_target_shifts_two_rooms_inside_wing():
    p = br.similar_target(37.0, 0.0, 0.0, 2)
    assert (p.x, p.y) == (45.0, 0.0)
    q = br.similar_target(49.0, 0.0, 0.0, 2)  # на восток некуда — на запад
    assert q.x == 41.0
    with pytest.raises(ValueError):
        br.similar_target(10.0, 0.0, 0.0, 2)  # не в одинаковом крыле


def test_kidnaps_reference_known_points():
    pts = br.points_by_name()
    for k in br.KIDNAPS:
        assert k.approach in pts and k.goal in pts
        assert (k.to is None) != (k.similar == 0)
        if k.similar:
            assert br.in_similar(pts[k.approach].x)
        else:
            assert clearance(Pose2D(k.to[0], k.to[1], math.radians(k.to[2])), bf.floor_boxes()) > 0.3


def test_set_pose_request():
    req = br.set_pose_request("rob_box", 1.0, -2.0, math.pi / 2)
    assert req.startswith('name: "rob_box", position: {x: 1.0000, y: -2.0000, z: 0.050}')
    assert "orientation: {z: 0.707107, w: 0.707107}" in req


def test_plan_estimate_adds_up():
    est = br.plan_estimate(6, voice=3)
    assert est["goals"] == len(br.explore_tour())
    assert est["tour_m"] == pytest.approx(br.tour_length(br.explore_tour()), abs=0.1)
    assert est["total"] == pytest.approx(est["map"] + est["route"] + est["loc"] + est["voice"], abs=0.3)
    assert est["voice"] == pytest.approx(15.0)
    assert br.plan_estimate(10)["loc"] > br.plan_estimate(5)["loc"]


def test_floor_grid_marks_walls_and_leaves_corridor_free():
    g = br.floor_grid(res=0.1)

    def cell(x, y):
        return g["data"][int((y - g["origin_y"]) / g["resolution"])][int((x - g["origin_x"]) / g["resolution"])]

    assert cell(20.0, 0.0) == 0  # коридор
    assert cell(20.0, bf.CORRIDOR_AXIS) == 100  # стена коридора
    assert cell(bf.X_EAST, 3.0) == 100  # внешняя стена


# ── лог шагов ────────────────────────────────────────────────────────────────────────────────

GOAL_END = (
    "12:00:01.000 GOAL_END {name} result={res} sim={sim} frame=-1 dur_sim={dur} err={err} em_episodes={em} "
    "recoveries={rec} gt=(1.0, 2.0, 90.0)"
)


def goal_end(name, res="STATUS_SUCCEEDED", sim=10.0, dur=30.0, err=0.2, em=0, rec=0):
    return GOAL_END.format(name=name, res=res, sim=sim, dur=dur, err=err, em=em, rec=rec)


def test_event_line_roundtrip():
    line = "12:00:00.000 " + br.event_line("SEG_START", 12.5, name="start_1", kind="start", x=1.25, label="a b")
    ev = br.parse_events([line, "мусор", "EVENT BROKEN без sim"])
    assert len(ev) == 1
    e = ev[0]
    assert (e.kind, e.sim, e.get("name"), e.num("x"), e.get("label")) == ("SEG_START", 12.5, "start_1", 1.25, "a_b")


def test_parse_goal_end_normalizes_status():
    g = br.parse_goal_ends([goal_end("dean", sim=99.5, err=0.31, em=2, rec=1), "GOAL_START dean"])[0]
    assert (g.name, g.outcome.result, g.sim, g.err_m, g.outcome.em_episodes, g.recoveries) == (
        "dean", "SUCCEEDED", 99.5, 0.31, 2, 1
    )  # fmt: skip
    assert br.parse_goal_ends([goal_end("x", res="TIMEOUT")])[0].outcome.result == "TIMEOUT"


def test_arrival_kinds():
    ok, far, failed = br.parse_goal_ends(
        [goal_end("a", err=0.3), goal_end("b", err=2.4), goal_end("c", res="STATUS_ABORTED", err=0.1)]
    )
    assert br.arrival(ok) == br.ARRIVED
    assert br.arrival(far) == br.FALSE_SUCCESS  # Nav2 «дошёл», а истина в 2.4 м
    assert br.arrival(failed) == br.FAILED


def test_segments_attach_goals_and_events_by_sim_time():
    lines = [
        br.event_line("SEG_START", 0.0, name="map", kind="map"),
        goal_end("reception", sim=40.0),
        br.event_line("SEG_END", 100.0, name="map"),
        br.event_line("SEG_START", 110.0, name="restart_after_map", kind="restart"),
        br.event_line("MARK", 130.0, name="restart_end"),
        goal_end("stray", sim=150.0),  # вне отрезка map
    ]
    segs = br.segments(br.parse_events(lines), br.parse_goal_ends(lines))
    assert [(s.name, s.kind, s.t0, s.t1) for s in segs] == [
        ("map", "map", 0.0, 100.0),
        ("restart_after_map", "restart", 110.0, 150.0),  # не закрыт — до последней записи
    ]
    assert [g.name for g in segs[0].goals] == ["reception"]
    assert segs[1].first("MARK", "restart_end").sim == 130.0


def test_route_verdict_pass_fail_and_facts():
    goals = br.parse_goal_ends([goal_end("a", dur=20.0, em=1), goal_end("b", dur=40.0, rec=2)])
    v = br.route_verdict("route", goals)
    assert v.status == br.PASS
    assert v.line().startswith("VERDICT #4468 route PASS: доехал 2/2 (100 %)")
    assert "стопов защиты 1, восстановлений 2" in v.line()
    bad = br.route_verdict("route", goals + br.parse_goal_ends([goal_end("c", err=3.0), goal_end("d", res="TIMEOUT")]))
    assert bad.status == br.FAIL
    assert "FALSE_SUCCESS: c(SUCCEEDED,3.00м)" in bad.line() and "FAILED: d(TIMEOUT" in bad.line()
    assert br.route_verdict("route", []).status == br.NOT_TRIGGERED


TOOLS_LOG = """2026-11-10T09:00:00Z TOOL start_mapping {"map_name": "здание"}
[live_check] /mcp/execute subscribers=1
{"tool_name": "start_mapping", "request_id": "r1", "result": {"success": true, "message": "Начинаю"}}
2026-11-10T09:40:00Z TOOL save_waypoint {"name": "деканат"}
{"tool_name": "save_waypoint", "request_id": "r2", "result": {"success": false, "error": "Позиция неизвестна"}}
2026-11-10T09:41:00Z TOOL save_waypoint {"name": "столовая"}
{"error": "таймаут 90s: нет /mcp/result для r3"}
TOOL_FAILED save_waypoint rc=2
2026-11-10T09:42:00Z TOOL list_waypoints {}
"""


def test_registry_verdict_from_tool_answers():
    res = br.parse_tool_results(TOOLS_LOG.splitlines())
    assert res == [
        ("start_mapping", True, "Начинаю"),
        ("save_waypoint", False, "Позиция неизвестна"),
        ("save_waypoint", False, "таймаут 90s: нет /mcp/result для r3"),
        ("list_waypoints", False, "ответа нет"),
    ]
    v = br.registry_verdict(res)
    assert v.status == br.FAIL and "save_waypoint 0/2" in v.line()
    assert br.registry_verdict(res[:1]).status == br.PASS
    assert br.registry_verdict([]).status == br.NOT_TRIGGERED


def test_loc_part_refuses_without_loc_metrics(monkeypatch):
    """Без loc_metrics (#4568) — понятный отказ, а не «посчитали как получится»."""
    real = builtins.__import__

    def fake(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "rob_box_sim" and fromlist and "loc_metrics" in fromlist:
            raise ImportError("нет loc_metrics")
        return real(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", fake)
    with pytest.raises(RuntimeError, match="#4568"):
        br.read_poses_csv(["t,tx,ty,tyaw,ex,ey,eyaw"])


def test_cli_plan_prints_tour_starts_estimate(capsys):
    assert br.main(["plan", "--starts", "4", "--seed", "3"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0] == "TOUR " + " ".join(br.explore_tour())
    assert sum(line.startswith("START ") for line in out) == 4
    est = json.loads(next(line for line in out if line.startswith("ESTIMATE "))[len("ESTIMATE ") :])  # noqa: E203
    assert est["goals"] == len(br.explore_tour())


def test_cli_starts_and_nearest(capsys):
    br.main(["starts", "--starts", "2", "--seed", "5"])
    rows = [line.split() for line in capsys.readouterr().out.splitlines()]
    want = br.random_starts(2, 5)
    assert [(float(x), float(y)) for x, y, _ in rows] == [(p.x, p.y) for p in want]
    br.main(["nearest", "15", "-3"])
    assert capsys.readouterr().out.strip() == "dean"
    br.main(["label", "dean"])
    assert capsys.readouterr().out.strip() == "деканат"


def test_step_and_report_scripts_compile():
    for rel in ("scripts/hil/video/building_step.py", "scripts/hil/building/building_report.py"):
        py_compile.compile(str(REPO / rel), doraise=True)


def test_step_script_uses_module_not_own_rules():
    """Шаг только исполняет: пороги, цели телепорта и запрос set_pose — из building_run (ADR-0148)."""
    text = (REPO / "scripts/hil/video/building_step.py").read_text(encoding="utf-8")
    for needle in ("br.set_pose_request", "br.similar_target", "br.KIDNAPS", "br.Criteria().arrive_m", "br.event_line"):
        assert needle in text
