"""Face outside a dialogue: turn signals / brake by /cmd_vel, battery, holiday calendar (ADR-0150 §5.5.6)."""

from __future__ import annotations

import datetime as dt

import pytest
from rob_box_dialog.knowledge import CALENDAR

from rob_box_animations.face import rear as R
from rob_box_animations.face.states import SCENES
from rob_box_animations.face_director import FaceDirector
from rob_box_animations.outside import (BatteryWatch, DriveTracker, battery_percent, calendar_rear, calendar_scene)

T0 = 1_800_000_000.0
MSK = 3.0


def feed(tr: DriveTracker, samples, t0=T0, dt_s=0.1):
    """(vx, wz) every ``dt_s``; returns the scenes seen after each sample."""
    out = []
    for i, (vx, wz) in enumerate(samples):
        now = t0 + i * dt_s
        tr.on_cmd(vx, wz, now)
        out.append(tr.scene(now))
    return out


def test_turn_needs_300ms_of_yaw_above_threshold():
    seen = feed(DriveTracker(), [(0.3, 0.5)] * 6)
    assert seen[:3] == [None, None, None]
    assert seen[3:] == ['turn_left'] * 3


def test_turn_right_for_negative_yaw():
    assert feed(DriveTracker(), [(0.3, -0.5)] * 6)[-1] == 'turn_right'


def test_short_yaw_blip_is_joystick_noise():
    assert set(feed(DriveTracker(), [(0.3, 0.0), (0.3, 0.6), (0.3, 0.0)] * 5)) == {None}


def test_yaw_below_threshold_never_signals():
    assert set(feed(DriveTracker(), [(0.3, 0.19)] * 20)) == {None}


def test_turn_signal_holds_through_dip_then_releases():
    seen = feed(DriveTracker(), [(0.3, 0.5)] * 5 + [(0.3, 0.15)] * 3 + [(0.3, 0.0)] * 10)
    assert seen[4] == 'turn_left'
    assert seen[5:8] == ['turn_left'] * 3          # 0.1..0.2 rad/s: hysteresis band
    assert seen[-1] is None


def test_opposite_turn_does_not_revive_the_old_signal():
    """Found on the robot: left, 1 s pause, right -> 'turn_left' flashed again on the first right samples."""
    seen = feed(DriveTracker(), [(0.2, 0.6)] * 15 + [(0.2, 0.0)] * 10 + [(0.2, -0.6)] * 15)
    assert 'turn_left' not in seen[16 + 5:]
    assert seen[25:28] == [None, None, None]
    assert seen[-1] == 'turn_right'


def test_turn_signal_dies_when_cmd_vel_goes_stale():
    tr = DriveTracker()
    feed(tr, [(0.3, 0.5)] * 6)
    assert tr.scene(T0 + 0.5) == 'turn_left'
    assert tr.scene(T0 + 1.5) is None


def test_hard_deceleration_gives_one_second_brake():
    tr = DriveTracker()
    seen = feed(tr, [(0.8, 0.0)] * 5 + [(0.8 - 0.2 * k, 0.0) for k in range(1, 5)] + [(0.0, 0.0)] * 20)
    assert seen[0] is None
    first = seen.index('brake')
    assert first <= 8
    braking = [i for i, s in enumerate(seen) if s == 'brake']
    assert braking == list(range(first, braking[-1] + 1))       # one continuous stretch
    assert 10 <= len(braking) <= 18          # 1 s hold + the 0.5 s window that still sees the drop
    assert seen[-1] is None


def test_gentle_slowdown_and_noise_do_not_brake():
    gentle = [(0.8 - 0.02 * k, 0.0) for k in range(20)]      # 0.2 m/s^2
    noise = [(0.5 + (0.03 if k % 2 else -0.03), 0.0) for k in range(20)]
    assert set(feed(DriveTracker(), gentle)) == {None}
    assert set(feed(DriveTracker(), noise)) == {None}


def test_standing_robot_cannot_brake():
    assert set(feed(DriveTracker(), [(0.08, 0.0)] * 5 + [(0.0, 0.0)] * 5)) == {None}


def test_battery_low_shows_4s_once_a_minute():
    b = BatteryWatch()
    shown = []
    for i in range(0, 130):
        b.on_battery(15.0, 0, T0 + i)
        shown.append(b.scene(T0 + i))
    assert shown[0:4] == ['low_battery'] * 4 and shown[4] is None
    assert shown[60:64] == ['low_battery'] * 4 and shown[64] is None
    assert shown[120] == 'low_battery'


def test_battery_hysteresis_20_to_25():
    b = BatteryWatch()
    b.on_battery(19.0, 0, T0)
    assert b.scene(T0) == 'low_battery'
    b.on_battery(22.0, 0, T0 + 61)
    assert b.scene(T0 + 61) == 'low_battery'     # still low until 25 %
    b.on_battery(26.0, 0, T0 + 122)
    assert b.scene(T0 + 122) is None


def test_battery_ok_unknown_and_stale_show_nothing():
    b = BatteryWatch()
    assert b.scene(T0) is None
    b.on_battery(80.0, 0, T0)
    assert b.scene(T0) is None
    b.on_battery(None, 0, T0 + 1)
    assert b.scene(T0 + 1) is None
    b.on_battery(10.0, 0, T0 + 2)
    assert b.scene(T0 + 40) is None              # no message for 30 s: not our fact any more


def test_charging_status_shows_charging():
    b = BatteryWatch()
    b.on_battery(50.0, 1, T0)
    assert b.scene(T0) == 'charging'
    b.on_battery(50.0, 3, T0 + 1)
    assert b.scene(T0 + 1) is None


def test_battery_percent_scale_and_unknown():
    assert battery_percent(0.15, 100.0) == pytest.approx(15.0)
    assert battery_percent(-1.0, 100.0) is None
    assert battery_percent(float('nan'), 100.0) is None


# -- calendar -----------------------------------------------------------------
def epoch(y, m, d, hh=12, mm=0, ss=0):
    return dt.datetime(y, m, d, hh, mm, ss, tzinfo=dt.timezone(dt.timedelta(hours=MSK))).timestamp()


def test_calendar_has_no_overlapping_days():
    days = [(c.month, c.day) for c in CALENDAR]
    assert len(days) == len(set(days))


def test_calendar_names_are_real_themes_and_scenes():
    for c in CALENDAR:
        assert c.rear in R.THEMES
        assert c.scene is None or c.scene in SCENES


@pytest.mark.parametrize('c', CALENDAR, ids=lambda c: f'{c.day:02d}.{c.month:02d}')
def test_calendar_theme_covers_whole_day_only(c):
    assert calendar_rear(epoch(2027, c.month, c.day, 0, 0, 1), MSK) == c.rear
    assert calendar_rear(epoch(2027, c.month, c.day, 23, 59, 59), MSK) == c.rear
    ordinary = dt.date(2027, c.month, c.day) + dt.timedelta(days=-3)
    if (ordinary.month, ordinary.day) not in {(x.month, x.day) for x in CALENDAR}:
        assert calendar_rear(epoch(2027, ordinary.month, ordinary.day), MSK) is None


def test_midnight_scene_is_last_minute_of_31_dec_only():
    assert calendar_scene(epoch(2026, 12, 31, 23, 58, 59), MSK) is None
    assert calendar_scene(epoch(2026, 12, 31, 23, 59, 1), MSK) == 'new_year_eve'
    assert calendar_scene(epoch(2027, 1, 1, 0, 0, 5), MSK) is None
    assert calendar_scene(epoch(2026, 10, 31, 23, 59, 30), MSK) is None


def test_timezone_decides_the_day():
    utc_late = epoch(2026, 10, 31, 22, 30)                   # 31.10 22:30 MSK = 19:30 UTC
    assert calendar_rear(utc_late, 0.0) == 'halloween'
    assert calendar_rear(utc_late, 5.0) is None              # already 01.11 at UTC+5


# -- director -----------------------------------------------------------------
def awake(now=T0, **kw) -> FaceDirector:
    d = FaceDirector(now, **kw)
    d.voice_seen(now + 7.0)
    return d


def test_director_shows_turn_then_idle_and_talking_beats_it():
    d = awake(tz_offset_h=MSK)
    t = T0 + 7.0
    for i in range(6):
        d.on_cmd_vel(0.3, 0.5, t + i * 0.1)
    assert d.choose(t + 0.5)[0] == 'turn_left'
    d.on_tts_state('playing', t + 0.5)
    assert d.choose(t + 0.5)[0].startswith('talk_')
    d.on_tts_state('stopped', t + 0.6)
    d.voice_seen(t + 3)
    assert d.choose(t + 3)[0].startswith('idle_')


def test_director_requested_scene_beats_driving():
    d = awake(tz_offset_h=MSK)
    t = T0 + 7.0
    d.request('happy', 5.0, t)
    for i in range(6):
        d.on_cmd_vel(0.3, 0.5, t + i * 0.1)
    assert d.choose(t + 0.5)[0] == 'happy'


def test_director_driving_beats_battery():
    d = awake(tz_offset_h=MSK)
    t = T0 + 7.0
    d.on_battery(10.0, 0, t)
    assert d.choose(t)[0] == 'low_battery'
    for i in range(6):
        d.on_cmd_vel(0.3, -0.5, t + i * 0.1)
    assert d.choose(t + 0.5)[0] == 'turn_right'


def test_director_rear_theme_priority_manual_over_param_over_calendar():
    halloween = epoch(2026, 10, 31, 12)
    d = FaceDirector(halloween, tz_offset_h=MSK)
    assert d.rear_theme_at(halloween) == 'halloween'
    assert d.set_rear('valentine', 10.0, halloween)
    assert d.rear_theme_at(halloween + 5) == 'valentine'
    assert d.rear_theme_at(halloween + 11) == 'halloween'
    assert not d.set_rear('no_such_theme', 10.0, halloween)
    assert FaceDirector(halloween, tz_offset_h=MSK, rear_theme='birthday').rear_theme_at(halloween) == 'birthday'


def test_director_midnight_scene_on_new_year_eve():
    start = epoch(2026, 12, 31, 23, 59, 10)
    d = FaceDirector(start - 100, tz_offset_h=MSK)
    d.voice_seen(start)
    assert d.choose(start)[0] == 'new_year_eve'
    d.voice_seen(start - 60)
    assert d.choose(start - 60)[0] != 'new_year_eve'
