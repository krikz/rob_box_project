"""Зона безопасности вокруг корпуса (#4365): геометрия, «в движении», гистерезис, мигалки и звук."""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from rob_box_teleop import proximity as px

REPO = Path(__file__).resolve().parents[3]
NAV2_PARAMS = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"
TWIST_MUX = REPO / "docker" / "main" / "config" / "twist_mux" / "twist_mux.yaml"
DATA = Path(__file__).resolve().parent / "data" / "hil_2026-11-05_stops.csv"

# Габарит РОББОКСа (замер 04.11, nav2_params.yaml #4361).
BODY = px.Body(-0.3036, 0.3064, -0.3025, 0.3025)
TH = px.Thresholds()


def ahead(gap):
    """Точка прямо по курсу на ``gap`` от переднего края."""
    return (BODY.x_max + gap, 0.0)


def straight(gap):
    """Угроза по коридору хода: точка ``gap`` от корпуса с порогом ``stop_m``."""
    return px.Threat(gap=gap, slack=gap - TH.stop_m, stop_gap=gap)


# -- геометрия ----------------------------------------------------------------------------------


def test_body_is_read_from_the_same_nav2_footprint_as_the_costmap():
    body = px.body_from_nav2_params(yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8")))
    assert body == BODY


def test_gap_is_measured_from_the_hull_not_from_the_center():
    assert BODY.gap(*ahead(0.30)) == pytest.approx(0.30)
    assert BODY.gap(0.0, BODY.y_max + 0.10) == pytest.approx(0.10)
    # Угол: по диагонали от угла корпуса.
    assert BODY.gap(BODY.x_max + 0.03, BODY.y_max + 0.04) == pytest.approx(0.05)


def test_points_inside_the_hull_have_zero_gap():
    assert BODY.gap(0.1, 0.1) == 0.0


def test_scan_points_apply_mount_offset_and_yaw_and_drop_bad_ranges():
    mount = px.Mount(0.1, 0.0, math.pi)  # лидар повёрнут назад
    pts = px.scan_points([1.0, float("inf"), 0.05, 20.0], 0.0, math.pi / 2, 0.15, 12.0, mount)
    assert len(pts) == 1
    assert tuple(pts[0]) == pytest.approx((-0.9, 0.0))


def test_yaw_from_quaternion():
    half = math.pi / 4
    assert px.yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(math.pi / 2)


def test_closing_speed_forward_back_and_alongside():
    x, y = ahead(0.3)
    assert px.closing_speed(BODY, x, y, 0.3, 0.0) == pytest.approx(0.3)
    assert px.closing_speed(BODY, x, y, -0.3, 0.0) == pytest.approx(-0.3)
    # Стена сбоку, робот едет вдоль — не приближается.
    assert px.closing_speed(BODY, 0.0, BODY.y_max + 0.1, 0.3, 0.0) == pytest.approx(0.0)


def test_rotation_closes_on_a_point_off_the_corner():
    # Точка справа у переднего края: поворот вправо (w < 0) ведёт правый борт на неё, влево — от неё.
    x, y = BODY.x_max - 0.05, BODY.y_min - 0.05
    assert px.closing_speed(BODY, x, y, 0.0, -1.0) > 0.0
    assert px.closing_speed(BODY, x, y, 0.0, 1.0) < 0.0


# -- угроза: только в движении и только по ходу -----------------------------------------------


def test_standing_next_to_an_obstacle_is_no_threat():
    assert px.nearest_threat(BODY, [ahead(0.05)], 0.0, 0.0, TH) == px.NO_THREAT


def test_driving_at_an_obstacle_is_a_threat():
    wall = [ahead(0.5), ahead(0.15), (BODY.x_max + 0.16, 0.01)]
    threat = px.nearest_threat(BODY, wall, 0.2, 0.0, TH)
    assert threat.gap == pytest.approx(0.16)
    assert threat.point == (BODY.x_max + 0.16, 0.01)


def test_a_single_ghost_ray_is_no_threat():
    """Сим, прогон run5: один луч давал точку в 0.20 м от корпуса в пустом мире — тормозить по ней нельзя."""
    assert px.nearest_threat(BODY, [ahead(0.15), ahead(0.9)], 0.2, 0.0, TH).gap == pytest.approx(0.9)
    assert px.nearest_threat(BODY, [ahead(0.15)], 0.2, 0.0, TH) == px.NO_THREAT
    assert px.nearest_threat(BODY, [ahead(0.15)], 0.2, 0.0, px.Thresholds(min_points=1)).gap == pytest.approx(0.15)


def test_backing_away_or_passing_a_doorway_is_no_threat():
    assert px.nearest_threat(BODY, [ahead(0.10)], -0.2, 0.0, TH) == px.NO_THREAT
    jamb = [(0.0, BODY.y_max + 0.12), (0.0, BODY.y_min - 0.12)]  # проём 0.85 м
    assert px.nearest_threat(BODY, jamb, 0.2, 0.0, TH) == px.NO_THREAT


def test_points_inside_the_hull_are_ignored():
    assert px.nearest_threat(BODY, [(0.2, 0.0)], 0.3, 0.0, TH) == px.NO_THREAT


def test_thresholds_reject_stop_beyond_warn():
    with pytest.raises(ValueError):
        px.Thresholds(warn_m=0.2, stop_m=0.4)
    with pytest.raises(ValueError):
        px.Thresholds(min_points=0)


# -- зона и гистерезис ----------------------------------------------------------------------


def test_classify_default_thresholds():
    assert px.classify(straight(0.39), TH) == px.ZONE_WARN
    assert px.classify(straight(0.19), TH) == px.ZONE_STOP
    assert px.classify(straight(0.41), TH) == px.ZONE_CLEAR
    assert px.classify(px.NO_THREAT, TH) == px.ZONE_CLEAR


def test_enter_is_immediate_and_escalates_through_warn_to_stop():
    tr = px.ZoneTracker(TH)
    assert tr.update(0.0, straight(0.35)) == px.Transition(px.ZONE_CLEAR, px.ZONE_WARN, 0.35)
    assert tr.update(0.1, straight(0.18)).zone == px.ZONE_STOP
    assert tr.stopping


def test_warn_releases_only_after_margin_held_for_release_s():
    tr = px.ZoneTracker(TH)
    tr.update(0.0, straight(0.35))
    assert tr.update(0.1, straight(0.42)) is None          # дальше порога, но в запасе 5 см — держим
    assert tr.update(0.2, straight(0.50)) is None          # с запасом — начали отсчёт
    assert tr.update(0.6, straight(0.50)) is None          # 0.4 с < 0.5 с
    assert tr.update(0.75, straight(0.50)).zone == px.ZONE_CLEAR


def test_flicker_resets_the_release_timer():
    tr = px.ZoneTracker(TH)
    tr.update(0.0, straight(0.35))
    tr.update(0.1, straight(0.50))
    tr.update(0.4, straight(0.38))                         # снова близко — отсчёт сначала
    assert tr.update(0.7, straight(0.50)) is None
    assert tr.update(1.25, straight(0.50)).zone == px.ZONE_CLEAR


def test_stop_is_held_at_least_stop_hold_s_even_when_the_robot_stood_still():
    tr = px.ZoneTracker(TH)
    tr.update(0.0, straight(0.15))
    # Встал — угрозы нет (inf), но стоп держится stop_hold_s.
    for t in (0.1, 0.6, 1.0, 1.4):
        assert tr.update(t, px.NO_THREAT) is None and tr.stopping
    assert tr.update(1.5, px.NO_THREAT).zone == px.ZONE_CLEAR
    assert not tr.stopping


def test_stop_releases_to_warn_when_still_close():
    tr = px.ZoneTracker(TH)
    tr.update(0.0, straight(0.15))
    tr.update(1.0, straight(0.30))
    assert tr.update(1.6, straight(0.30)).zone == px.ZONE_WARN


# -- тревога: мигалка, писк, голос — одним событием ------------------------------------------

LIGHTS = px.Effect(px.EFFECT_LIGHTS, "roadworks_beacon", px.LIGHTS_TTL_MS)
WARN_BEEP = px.Effect(px.EFFECT_SOUND, "robot_backup_beep")
STOP_BEEP = px.Effect(px.EFFECT_SOUND, "robot_backup_beep_fast")


def test_alarm_is_the_roadworks_beacon_and_the_truck_backup_beeper_not_the_police():
    """Решение Шифу 05.11: маячок «дорожные работы» вместо полицейской мигалки, бипер заднего хода вместо писков;
    «внимание» бипает мерно (бип 0.45 с / пауза 0.45 с), «стоп» — чаще."""
    warn, stop = px.ALARMS[px.ZONE_WARN], px.ALARMS[px.ZONE_STOP]
    assert warn.animation == stop.animation == "roadworks_beacon"
    assert (warn.sound, warn.sound_every_s) == ("robot_backup_beep", 0.9)
    assert (stop.sound, stop.sound_every_s) == ("robot_backup_beep_fast", 0.4)
    assert stop.sound_every_s < warn.sound_every_s
    assert all(a.animation != "police_lights" for a in px.ALARMS.values())


def test_warn_is_lights_and_a_steady_beeper_without_voice():
    s = px.AlarmScheduler()
    assert s.tick(0.0, px.ZONE_WARN) == [LIGHTS, WARN_BEEP]
    assert s.tick(0.3, px.ZONE_WARN) == []
    assert s.tick(0.5, px.ZONE_WARN) == [LIGHTS]          # продление мигалки, бип ещё не пора
    assert s.tick(0.9, px.ZONE_WARN) == [WARN_BEEP]       # бип 0.45 с + пауза 0.45 с — следующий
    assert s.tick(1.0, px.ZONE_WARN) == [LIGHTS]
    assert s.tick(1.8, px.ZONE_WARN) == [LIGHTS, WARN_BEEP]
    assert all(e.kind != px.EFFECT_SPEECH for t in (2.7, 3.6, 4.5) for e in s.tick(t, px.ZONE_WARN))


def test_stop_starts_lights_beep_and_voice_in_one_tick():
    s = px.AlarmScheduler()
    s.tick(0.0, px.ZONE_WARN)
    assert s.tick(0.1, px.ZONE_STOP) == [LIGHTS, STOP_BEEP, px.Effect(px.EFFECT_SPEECH, "Осторожно!")]


def test_stop_beeps_fast_and_keeps_lights_while_it_holds_voice_only_on_entry():
    s = px.AlarmScheduler()
    s.tick(0.0, px.ZONE_STOP)
    assert s.tick(0.2, px.ZONE_STOP) == []
    assert s.tick(0.4, px.ZONE_STOP) == [STOP_BEEP]
    assert s.tick(0.5, px.ZONE_STOP) == [LIGHTS]
    assert s.tick(0.8, px.ZONE_STOP) == [STOP_BEEP]
    beeps = sum(e == STOP_BEEP for k in range(1, 41) for e in s.tick(0.8 + k * 0.05, px.ZONE_STOP))
    assert beeps == 5                                     # 2 с при 20 Гц — бип каждые 0.4 с
    assert [e.kind for e in s.tick(3.6, px.ZONE_STOP)] == [px.EFFECT_LIGHTS, px.EFFECT_SOUND]


def test_alarm_ends_with_the_zone_lights_expire_by_ttl():
    s = px.AlarmScheduler()
    s.tick(0.0, px.ZONE_STOP)
    assert s.tick(1.0, px.ZONE_CLEAR) == []
    assert s.tick(5.0, px.ZONE_CLEAR) == []
    assert px.LIGHTS_TTL_MS / 1000.0 > px.LIGHTS_EVERY_S   # продление успевает до того, как мигалка погаснет


def test_voice_is_not_repeated_more_often_than_the_cooldown():
    s = px.AlarmScheduler()
    s.tick(0.0, px.ZONE_STOP)
    s.tick(1.5, px.ZONE_CLEAR)
    again = s.tick(1.6, px.ZONE_STOP)                     # стоп сразу снова — бип и мигалка, голоса нет
    assert [e.kind for e in again] == [px.EFFECT_LIGHTS, px.EFFECT_SOUND]
    s.tick(3.0, px.ZONE_CLEAR)
    later = s.tick(px.PHRASE_COOLDOWN_S + 0.1, px.ZONE_STOP)
    assert px.Effect(px.EFFECT_SPEECH, "Осторожно!") in later


def test_clear_is_silent():
    s = px.AlarmScheduler()
    assert s.tick(0.0, px.ZONE_CLEAR) == []
    s.tick(1.0, px.ZONE_WARN)
    assert s.tick(2.0, px.ZONE_CLEAR) == []


# Имена мигалки и звука сверяются с каталогами в tests/unit/docker/test_proximity_guard_wiring.py:
# в test_ws пакета нет sound_pack/.


def test_describe_transition():
    assert px.describe(px.Transition(px.ZONE_CLEAR, px.ZONE_STOP, 0.123)) == "зона clear → stop (до корпуса 0.12 м)"
    assert "—" in px.describe(px.Transition(px.ZONE_STOP, px.ZONE_CLEAR, math.inf))


# -- коридор хода, бока и разворот (HIL 05.11, #4365) ------------------------------------------------------------


def wall(x0, x1, y0, y1, step=0.01):
    """Контур прямоугольника стены в мире точками через ``step``."""
    xs = np.arange(x0, x1 + 1e-9, step)
    ys = np.arange(y0, y1 + 1e-9, step)
    edges = [np.column_stack((xs, np.full_like(xs, y))) for y in (y0, y1)]
    edges += [np.column_stack((np.full_like(ys, x), ys)) for x in (x0, x1)]
    return np.vstack(edges)


def doorway(x0, width, thickness=0.10, reach=2.0):
    """Перегородка поперёк хода в ``x0`` с проёмом ``width`` по центру (как в мире polygon)."""
    half = width / 2.0
    return np.vstack((wall(x0, x0 + thickness, half, half + reach), wall(x0, x0 + thickness, -half - reach, -half)))


def in_robot_frame(world, x, y, yaw):
    """Точки мира в ``base_footprint`` робота в позе ``(x, y, yaw)``."""
    c, s = math.cos(yaw), math.sin(yaw)
    d = np.asarray(world) - (x, y)
    return np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))


def zone_at(world, pose, v, w, th=TH):
    return px.classify(px.nearest_threat(BODY, in_robot_frame(world, *pose), v, w, th), th)


@pytest.mark.parametrize("width", [1.0, 0.9])
@pytest.mark.parametrize("offset,yaw_deg", [(0.0, 0.0), (0.05, 0.0), (-0.03, 3.0)])
def test_driving_straight_through_a_narrow_doorway_never_stops(width, offset, yaw_deg):
    """HIL 05.11: проём 1.0 м давал 65/34/86 стопов (косяк впереди-сбоку под 0.20 м), без защиты Nav2 проезжал за
    20 с. Косяк вне коридора хода — порог ``side_stop_m``; в проёме 1.0 м до него ~0.2 м на сторону."""
    door = doorway(1.0, width)
    zones = {zone_at(door, (x, offset, math.radians(yaw_deg)), 0.15, 0.0) for x in np.arange(-0.5, 2.0, 0.02)}
    assert px.ZONE_STOP not in zones
    assert px.ZONE_WARN in zones  # «внимание» по косякам слышно — тесно


def test_head_on_wall_stops_at_stop_m():
    front = wall(1.0, 1.1, -1.0, 1.0)
    assert zone_at(front, (1.0 - BODY.x_max - 0.21, 0.0, 0.0), 0.10, 0.0) == px.ZONE_WARN
    assert zone_at(front, (1.0 - BODY.x_max - 0.19, 0.0, 0.0), 0.10, 0.0) == px.ZONE_STOP


def test_wall_behind_stops_when_reversing_and_not_when_driving_away():
    back = wall(-1.1, -1.0, -1.0, 1.0)
    pose = (-1.0 - BODY.x_min + 0.19, 0.0, 0.0)
    assert zone_at(back, pose, -0.10, 0.0) == px.ZONE_STOP
    assert zone_at(back, pose, 0.10, 0.0) == px.ZONE_CLEAR


def test_jamb_beside_the_corridor_stops_only_at_side_stop_m():
    y = BODY.y_max + TH.corridor_margin_m + 0.02  # вне коридора хода
    far, near = (BODY.x_max + 0.10, y), (BODY.x_max + 0.01, y)
    assert BODY.gap(*far) > TH.side_stop_m > BODY.gap(*near)
    assert px.classify(px.nearest_threat(BODY, [far] * 2, 0.15, 0.0, TH), TH) == px.ZONE_WARN
    assert px.classify(px.nearest_threat(BODY, [near] * 2, 0.15, 0.0, TH), TH) == px.ZONE_STOP


def test_corridor_edge_is_body_width_plus_margin():
    inside = (BODY.x_max + 0.15, BODY.y_min - TH.corridor_margin_m + 0.01)
    outside = (BODY.x_max + 0.15, BODY.y_min - TH.corridor_margin_m - 0.01)
    assert px.classify(px.nearest_threat(BODY, [inside] * 2, 0.15, 0.0, TH), TH) == px.ZONE_STOP
    assert px.classify(px.nearest_threat(BODY, [outside] * 2, 0.15, 0.0, TH), TH) == px.ZONE_WARN


@pytest.mark.parametrize("corner_gap,zone", [(0.08, px.ZONE_STOP), (0.13, px.ZONE_WARN), (0.17, px.ZONE_WARN)])
def test_turning_in_place_by_a_wall_stops_only_near_the_corner(corner_gap, zone):
    """HIL 05.11 (4b): разворот w=0.5 в 0.61 м от стены — угол проходил в 0.16–0.19 м и вставал 8 раз из 8.
    Стена вдоль борта в ``corner_gap``: на развороте в любую сторону на неё идёт один из углов."""
    side = wall(-2.0, 2.0, BODY.y_max + corner_gap, BODY.y_max + corner_gap + 0.1)
    assert [zone_at(side, (0.0, 0.0, 0.0), 0.0, w) for w in (0.5, -0.5)] == [zone, zone]


def test_logged_hil_stops_replayed_through_the_corridor_rule():
    """Все 466 входов в stop из HIL 05.11 (точка, v, w — как напечатала нода): в проёмах 1.0/0.9/0.8 м, в углу и на
    развороте почти ничего не остаётся, лоб в стену и задний ход в стену — стопят как раньше. Точку лога подаём
    дважды (``min_points`` = 2), поэтому это пересчёт по логам, не прогон на скане."""
    rows = [line.split(",") for line in DATA.read_text(encoding="utf-8").splitlines()[3:]]
    kept = {}
    for run, _, _, x, y, v, w in rows:
        point = (float(x), float(y))
        threat = px.nearest_threat(BODY, [point, point], float(v), float(w), TH)
        before, after = kept.get(run, (0, 0))
        kept[run] = (before + 1, after + (px.classify(threat, TH) == px.ZONE_STOP))
    assert len(rows) == 466
    doors = ("s1b_door10", "s1f_door10_repeat", "s1k_door10_r3", "s1g_door09", "s1h_door09_repeat", "s1i_door08")
    assert sum(kept[r][0] for r in doors) == 297
    assert sum(kept[r][1] for r in doors) <= 1
    assert kept["s2a_corner"] == (62, 0)
    assert kept["s4b_teleop_spin"] == (8, 0)
    assert kept["s5a_push_wall"][1] >= 4  # лоб в стену: 4 из 5
    assert kept["s5c_reverse_into_wall"] == (2, 2)


# -- стенка у угла корпуса (#4631, HIL 09.11 «внезапная стенка», Depot, 0.25 м/с) -----------------------------------


def box(cx, cy, sx, sy, yaw):
    """Контур бокса Gazebo (центр, размеры по своим осям, поворот) точками мира через 1 см."""
    c, s = math.cos(yaw), math.sin(yaw)
    local = wall(-sx / 2.0, sx / 2.0, -sy / 2.0, sy / 2.0)
    return np.column_stack((cx + c * local[:, 0] - s * local[:, 1], cy + s * local[:, 0] + c * local[:, 1]))


#: Стенка из прогона ``depot_v0.25_r1_213959`` (``speed_bench.wall_ahead``: 0.2 × 3.0 м поперёк хода).
HIL_4631_WALL = box(2.217, 2.526, 0.2, 3.0, 0.145)


@pytest.mark.parametrize(
    "pose,v,w",
    [
        ((1.69942, 1.23458, -0.98744), 0.034, 0.043),  # t = 199.5 с, зазор 0.195 м
        ((1.72278, 1.20590, -0.96424), 0.042, 0.038),  # t = 200.5 с, зазор 0.173 м
    ],
)
def test_hil_4631_wall_end_beside_the_path_is_a_jamb_not_a_stop(pose, v, w):
    """#4631: правда Gazebo, поза и скорость из ``track.csv`` того прогона. Ближе 0.20 м к корпусу был ТОРЕЦ стенки:
    впереди носа на 7–9 см и на 15–19 см левее борта — вне коридора хода. Робот объезжал стенку и проходил мимо торца,
    как мимо косяка проёма 0.90 м (там до косяка 0.1475 м на сторону). Порог такой точки — ``side_stop_m``,
    не ``stop_m`` (#4438): стоп по ней вернул бы стопы у косяков в проёмах (решение Шифу 05.11 — минимальный
    проём 0.90 м)."""
    pts = in_robot_frame(HIL_4631_WALL, *pose)
    gaps = np.array([BODY.gap(x, y) for x, y in pts])
    nearest = pts[int(np.argmin(gaps))]
    assert TH.side_stop_m < gaps.min() < TH.stop_m
    assert nearest[0] > BODY.x_max and nearest[1] > BODY.y_max + TH.corridor_margin_m  # впереди-слева, вне коридора
    assert zone_at(HIL_4631_WALL, pose, v, w) == px.ZONE_WARN


@pytest.mark.parametrize("w", [0.0, 0.3, -0.3])
def test_wall_in_front_of_one_corner_stops_at_stop_m_straight_and_on_an_arc(w):
    """Стенка перекрывает только левый угол корпуса (от y = 0.20 м влево): корпус на неё наедет — стоп с 0.20 м.
    На дуге коридор прямого хода не отменяется: защита не слабее, чем при прямом ходе."""
    for gap, zone in ((0.21, px.ZONE_WARN), (0.19, px.ZONE_STOP)):
        corner_wall = wall(BODY.x_max + gap, BODY.x_max + gap + 0.1, 0.20, 2.0)
        assert zone_at(corner_wall, (0.0, 0.0, 0.0), 0.15, w) == zone


@pytest.mark.parametrize("w", [0.0, 0.05, -0.05])
@pytest.mark.parametrize("yaw_deg", [0.0, 2.0, -2.0])
def test_wall_alongside_at_the_coverage_perimeter_offset_never_stops(w, yaw_deg):
    """Проход пылесоса по периметру (#4607, PR #4623): центр в 0.4125 м от стены (борт в 0.11 м), 0.25 м/с.
    Стена сбоку вне коридора хода — стопа нет ни на прямой, ни при подруливании DWB."""
    side = wall(-3.0, 3.0, 0.4125, 0.5125)
    zones = {zone_at(side, (x, 0.0, math.radians(yaw_deg)), 0.25, w) for x in np.arange(-1.0, 1.0, 0.05)}
    assert px.ZONE_STOP not in zones
    assert px.ZONE_WARN in zones


@pytest.mark.parametrize("w", [0.0, 0.15, -0.15])
def test_doorway_090_centered_with_dwb_steering_never_stops(w):
    """Проём 0.90 м по центру (минимальный проём, решение Шифу 05.11): косяк в 0.1475 м от борта. DWB подруливает
    (HIL 05.11: w до ±0.15 в проёме) — стопа всё равно нет: косяк вне коридора хода и дальше ``side_stop_m``."""
    door = doorway(1.0, 0.90)
    zones = {zone_at(door, (x, 0.0, 0.0), 0.15, w) for x in np.arange(-0.5, 2.0, 0.02)}
    assert px.ZONE_STOP not in zones


def test_turning_in_place_at_the_hil_4b_distance_from_a_wall_never_stops():
    """HIL 05.11 (4b): разворот w=±0.5 с центром в 0.61 м от стены — углы (радиус 0.43 м) до неё не достают."""
    side = wall(-2.0, 2.0, 0.61, 0.71)
    assert {zone_at(side, (0.0, 0.0, 0.0), 0.0, w) for w in (0.5, -0.5, 1.0)} <= {px.ZONE_CLEAR, px.ZONE_WARN}


def test_thresholds_reject_side_or_spin_stop_beyond_stop():
    with pytest.raises(ValueError):
        px.Thresholds(side_stop_m=0.25)
    with pytest.raises(ValueError):
        px.Thresholds(spin_stop_m=0.0)
    with pytest.raises(ValueError):
        px.Thresholds(corridor_margin_m=-0.01)


def test_scan_points_are_an_array_for_the_whole_scan():
    pts = px.scan_points(np.full(720, 2.0), -math.pi, 2 * math.pi / 720, 0.15, 12.0, px.Mount(-0.171))
    assert pts.shape == (720, 2)
    assert np.hypot(pts[:, 0] + 0.171, pts[:, 1]) == pytest.approx(np.full(720, 2.0))


# -- удержание стопа, пока команда толкает (решение Шифу 05.11) ---------------------------------------------------

WALL_AHEAD = [ahead(0.18), (BODY.x_max + 0.185, 0.02)]
TICK = 0.05


def run_hold(cmds):
    """Робот встал у стены в 0.18 м (одометрия — ноль), дальше в mux приходят команды ``cmds`` по одной на такт.
    Вернуть зоны по тактам."""
    tracker = px.ZoneTracker(TH)
    tracker.update(0.0, px.assess(BODY, WALL_AHEAD, (0.10, 0.0), None, False, TH))
    assert tracker.stopping
    zones = []
    for i, cmd in enumerate(cmds, start=1):
        threat = px.assess(BODY, WALL_AHEAD, (0.0, 0.0), cmd, tracker.stopping, TH)
        tracker.update(i * TICK, threat)
        zones.append(tracker.zone)
    return zones


def test_stop_holds_while_the_command_pushes_into_the_wall():
    """HIL 05.11 (5a): телеоп 0.1 м/с в стену — стоп 1.5 с, отпускание, 0.1 с хода, снова стоп (период 1.65 с)."""
    zones = run_hold([(0.10, 0.0)] * 200)  # 10 с толкаем
    assert set(zones) == {px.ZONE_STOP}


@pytest.mark.parametrize("cmd", [(-0.10, 0.0), (0.0, 0.0), None])
def test_stop_releases_after_hold_when_the_command_reverses_or_stops(cmd):
    zones = run_hold([(0.10, 0.0)] * 60 + [cmd] * 60)  # 3 с толкаем, потом назад / ноль / нет команды
    assert set(zones[:60]) == {px.ZONE_STOP}
    assert zones[-1] == px.ZONE_CLEAR  # одометрия — ноль: стоящий робот не тревожит
    assert zones.index(px.ZONE_CLEAR) - 60 <= round((TH.release_s + TICK) / TICK)


def test_hold_ignores_the_command_until_a_stop_has_happened():
    assert px.assess(BODY, WALL_AHEAD, (0.0, 0.0), (0.10, 0.0), False, TH) == px.NO_THREAT


def test_command_mux_picks_the_freshest_highest_priority_source_like_twist_mux():
    doc = yaml.safe_load(TWIST_MUX.read_text(encoding="utf-8"))
    sources = px.mux_sources(doc, "cmd_vel_emergency")
    assert "cmd_vel_emergency" not in {s.topic for s in sources}  # свой выход не слушаем — петли нет
    assert {"cmd_vel", "cmd_vel_joy", "cmd_vel_web"} <= {s.topic for s in sources}
    mux = px.CommandMux(sources)
    assert mux.current(0.0) is None
    mux.update("cmd_vel", 0.0, 0.15, 0.0)  # Nav2
    assert mux.current(0.1) == (0.15, 0.0)
    mux.update("cmd_vel_joy", 0.2, -0.1, 0.0)  # пульт перебивает Nav2
    assert mux.current(0.3) == (-0.1, 0.0)
    mux.update("cmd_vel", 0.5, 0.15, 0.0)
    assert mux.current(0.6) == (-0.1, 0.0)  # пульт свеж (timeout 0.5) — он
    assert mux.current(0.8) == (0.15, 0.0)  # пульт протух — снова Nav2
    assert mux.current(1.1) is None  # и Nav2 протух
