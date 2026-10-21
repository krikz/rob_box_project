"""The face outside a dialogue (ADR-0150 §5.5.3, §5.5.6): turn signals / brake light, battery, holiday themes.

Pure Python, no ROS: ``face_node`` feeds ``DriveTracker`` with ``/cmd_vel`` and ``BatteryWatch`` with
``/battery_state``; ``FaceDirector`` asks them for a scene name. Holiday themes come from the single
table ``rob_box_dialog.knowledge.CALENDAR`` — no second copy here.
"""
from __future__ import annotations

import datetime as dt
from collections import deque
from typing import Deque, Optional, Tuple

from rob_box_dialog.knowledge import CALENDAR, CalendarTheme

#: yaw rate (rad/s) that counts as «turning» and how long it has to last (joystick noise filter)
TURN_ON_RADPS, TURN_HOLD_S = 0.2, 0.3
#: turn signal goes off when |yaw| stays below this for ``TURN_RELEASE_S`` (hysteresis)
TURN_OFF_RADPS, TURN_RELEASE_S = 0.1, 0.4
#: braking: |linear.x| falls faster than this (m/s^2), by at least BRAKE_MIN_DROP over the window
BRAKE_DECEL_MPS2, BRAKE_MIN_DROP_MPS, BRAKE_WINDOW_S, BRAKE_SHOW_S = 0.3, 0.1, 0.5, 1.0
#: moving at all: below this the robot «stands» and cannot brake
MOVING_MPS = 0.1
#: no /cmd_vel for this long -> no turn signal (a dead publisher must not leave it blinking)
CMD_STALE_S = 0.5

BATTERY_LOW_PCT, BATTERY_OK_PCT = 20.0, 25.0
BATTERY_SHOW_S, BATTERY_EVERY_S, BATTERY_STALE_S = 4.0, 60.0, 30.0
#: sensor_msgs/BatteryState.POWER_SUPPLY_STATUS_CHARGING
STATUS_CHARGING = 1


class DriveTracker:
    """``/cmd_vel`` -> ``turn_left`` | ``turn_right`` | ``brake`` | None, with hysteresis."""

    def __init__(self) -> None:
        self._hist: Deque[Tuple[float, float]] = deque()
        self._cand: Optional[str] = None
        self._cand_since = 0.0
        self._turn: Optional[str] = None
        self._turn_seen = 0.0
        self._brake_until = -1.0
        self._last_msg = -1e9

    def on_cmd(self, linear_x: float, angular_z: float, now: float) -> None:
        self._last_msg = now
        self._track_turn(angular_z, now)
        self._track_brake(abs(linear_x), now)

    def _track_turn(self, wz: float, now: float) -> None:
        cand = 'left' if wz > TURN_ON_RADPS else 'right' if wz < -TURN_ON_RADPS else None
        if cand != self._cand:
            self._cand, self._cand_since = cand, now
        if cand and now - self._cand_since >= TURN_HOLD_S - 1e-6:
            self._turn, self._turn_seen = cand, now
        elif self._turn and abs(wz) >= TURN_OFF_RADPS and (wz > 0) == (self._turn == 'left'):
            self._turn_seen = now        # the hysteresis band holds only the signal of its own side

    def _track_brake(self, speed: float, now: float) -> None:
        self._hist.append((now, speed))
        while self._hist and now - self._hist[0][0] > BRAKE_WINDOW_S:
            self._hist.popleft()
        t_old, v_old = self._hist[0]
        span = now - t_old
        if span < 0.2 or v_old < MOVING_MPS:
            return
        drop = v_old - speed
        if drop >= BRAKE_MIN_DROP_MPS and drop / span > BRAKE_DECEL_MPS2:
            self._brake_until = now + BRAKE_SHOW_S

    def scene(self, now: float) -> Optional[str]:
        if now < self._brake_until:
            return 'brake'
        if self._turn and now - self._last_msg < CMD_STALE_S and now - self._turn_seen < TURN_RELEASE_S:
            return f'turn_{self._turn}'
        return None


class BatteryWatch:
    """``/battery_state`` -> ``charging`` | ``low_battery`` (4 s once a minute while < 20 %) | None."""

    def __init__(self) -> None:
        self.percent: Optional[float] = None
        self._charging = False
        self._seen = -1e9
        self._low = False
        self._shown_at = -1e9

    def on_battery(self, percent: Optional[float], status: int, now: float) -> None:
        self._seen, self._charging = now, status == STATUS_CHARGING
        self.percent = percent
        if percent is None:
            self._low = False
        elif percent < BATTERY_LOW_PCT:
            self._low = True
        elif percent >= BATTERY_OK_PCT:
            self._low = False

    def scene(self, now: float) -> Optional[str]:
        if now - self._seen > BATTERY_STALE_S:
            return None
        if self._charging:
            return 'charging'
        if not self._low:
            return None
        if now - self._shown_at >= BATTERY_EVERY_S:
            self._shown_at = now
        return 'low_battery' if now - self._shown_at < BATTERY_SHOW_S else None


def battery_percent(raw: float, scale: float) -> Optional[float]:
    """``BatteryState.percentage`` * ``scale`` -> percent; negative / NaN = unknown."""
    if raw != raw or raw < 0.0:
        return None
    return raw * scale


def theme_on(day: dt.date) -> Optional[CalendarTheme]:
    """The holiday on ``day`` (CALENDAR windows are one day long) or None."""
    return next((c for c in CALENDAR if (c.month, c.day) == (day.month, day.day)), None)


def local_datetime(now: float, tz_offset_h: float) -> dt.datetime:
    return dt.datetime.fromtimestamp(now, dt.timezone(dt.timedelta(hours=tz_offset_h))).replace(tzinfo=None)


def calendar_rear(now: float, tz_offset_h: float) -> Optional[str]:
    theme = theme_on(local_datetime(now, tz_offset_h).date())
    return theme.rear if theme else None


def calendar_scene(now: float, tz_offset_h: float) -> Optional[str]:
    """Full-face midnight scene: the last ``scene_s`` seconds of the day that has one (31.12)."""
    when = local_datetime(now, tz_offset_h)
    theme = theme_on(when.date())
    if not theme or not theme.scene or not theme.scene_s:
        return None
    end = dt.datetime.combine(when.date(), dt.time()) + dt.timedelta(days=1)
    return theme.scene if (end - when).total_seconds() <= theme.scene_s else None
