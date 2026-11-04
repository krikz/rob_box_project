"""Зона безопасности вокруг корпуса (#4365): геометрия, «в движении», гистерезис, мигалки и звук."""

import math
from pathlib import Path

import pytest
import yaml

from rob_box_teleop import proximity as px

REPO = Path(__file__).resolve().parents[3]
NAV2_PARAMS = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"

# Габарит РОББОКСа (замер 04.11, nav2_params.yaml #4361).
BODY = px.Body(-0.3036, 0.3064, -0.3025, 0.3025)
TH = px.Thresholds()


def ahead(gap):
    """Точка прямо по курсу на ``gap`` от переднего края."""
    return (BODY.x_max + gap, 0.0)


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
    assert pts[0] == pytest.approx((-0.9, 0.0))


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
    assert px.classify(0.39, TH) == px.ZONE_WARN
    assert px.classify(0.19, TH) == px.ZONE_STOP
    assert px.classify(0.41, TH) == px.ZONE_CLEAR
    assert px.classify(math.inf, TH) == px.ZONE_CLEAR


def test_enter_is_immediate_and_escalates_through_warn_to_stop():
    tr = px.ZoneTracker(TH)
    assert tr.update(0.0, 0.35) == px.Transition(px.ZONE_CLEAR, px.ZONE_WARN, 0.35)
    assert tr.update(0.1, 0.18).zone == px.ZONE_STOP
    assert tr.stopping


def test_warn_releases_only_after_margin_held_for_release_s():
    tr = px.ZoneTracker(TH)
    tr.update(0.0, 0.35)
    assert tr.update(0.1, 0.42) is None          # дальше порога, но в запасе 5 см — держим
    assert tr.update(0.2, 0.50) is None          # с запасом — начали отсчёт
    assert tr.update(0.6, 0.50) is None          # 0.4 с < 0.5 с
    assert tr.update(0.75, 0.50).zone == px.ZONE_CLEAR


def test_flicker_resets_the_release_timer():
    tr = px.ZoneTracker(TH)
    tr.update(0.0, 0.35)
    tr.update(0.1, 0.50)
    tr.update(0.4, 0.38)                         # снова близко — отсчёт сначала
    assert tr.update(0.7, 0.50) is None
    assert tr.update(1.25, 0.50).zone == px.ZONE_CLEAR


def test_stop_is_held_at_least_stop_hold_s_even_when_the_robot_stood_still():
    tr = px.ZoneTracker(TH)
    tr.update(0.0, 0.15)
    # Встал — угрозы нет (inf), но стоп держится stop_hold_s.
    for t in (0.1, 0.6, 1.0, 1.4):
        assert tr.update(t, math.inf) is None and tr.stopping
    assert tr.update(1.5, math.inf).zone == px.ZONE_CLEAR
    assert not tr.stopping


def test_stop_releases_to_warn_when_still_close():
    tr = px.ZoneTracker(TH)
    tr.update(0.0, 0.15)
    tr.update(1.0, 0.30)
    assert tr.update(1.6, 0.30).zone == px.ZONE_WARN


# -- тревога: мигалка, писк, голос — одним событием ------------------------------------------

LIGHTS = px.Effect(px.EFFECT_LIGHTS, "police_lights", px.LIGHTS_TTL_MS)


def test_warn_is_lights_and_one_short_beep_without_voice():
    s = px.AlarmScheduler()
    assert s.tick(0.0, px.ZONE_WARN) == [LIGHTS, px.Effect(px.EFFECT_SOUND, "ui_notification")]
    assert s.tick(0.3, px.ZONE_WARN) == []
    assert s.tick(0.5, px.ZONE_WARN) == [LIGHTS]          # продление мигалки, писк не повторяется
    assert s.tick(2.0, px.ZONE_WARN) == [LIGHTS]


def test_stop_starts_lights_beep_and_voice_in_one_tick():
    s = px.AlarmScheduler()
    s.tick(0.0, px.ZONE_WARN)
    assert s.tick(0.1, px.ZONE_STOP) == [LIGHTS, px.Effect(px.EFFECT_SOUND, "robot_error"),
                                         px.Effect(px.EFFECT_SPEECH, "Осторожно!")]


def test_stop_keeps_beeping_and_lights_while_it_holds_voice_only_on_entry():
    s = px.AlarmScheduler()
    s.tick(0.0, px.ZONE_STOP)
    assert s.tick(0.5, px.ZONE_STOP) == [LIGHTS]
    assert s.tick(0.8, px.ZONE_STOP) == [px.Effect(px.EFFECT_SOUND, "robot_error")]
    assert [e.kind for e in s.tick(1.6, px.ZONE_STOP)] == [px.EFFECT_LIGHTS, px.EFFECT_SOUND]


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
    again = s.tick(1.6, px.ZONE_STOP)                     # стоп сразу снова — писк и мигалка, голоса нет
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
