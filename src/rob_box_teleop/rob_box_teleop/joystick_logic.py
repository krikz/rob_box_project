"""Pure joystick/SBUS logic for joystick_control_node (no ROS dependencies).

Extracted so the channel→axis mapping, deadzone and ARM-gating logic can be
unit-tested without a ROS2 runtime (mirrors the pattern used by
rob_box_voice unit tests: mock rclpy, test pure methods).
"""

from typing import Optional

# SBUS channel value range (ExpressLRS)
SBUS_MIN = 172
SBUS_CENTER = 992
SBUS_MAX = 1811
SBUS_HALF_RANGE = (SBUS_MAX - SBUS_MIN) / 2.0  # 819.5

# ARM switch is considered HIGH above this raw channel value
ARM_THRESHOLD = 1500.0

# ARM announcement debounce (issue #4003). The ARM channel of the ExpressLRS
# receiver chatters LOW->HIGH within 45..100 ms (robot log of 2026-10-24); a
# person flips a switch no faster than a few times per second and a flip stays
# put for far longer than 300 ms. 300 ms is therefore ~3x the longest observed
# glitch and still short enough to feel instant after a real flip.
ARM_ANNOUNCE_HOLD_S = 0.3
# One phrase is ~2-3 s of speech plus 1-2 s of synthesis (~4 s in the tts_node
# queue). Announcing no more often than every 5 s keeps the queue empty even
# when the operator really flips the switch back and forth.
ARM_ANNOUNCE_MIN_INTERVAL_S = 5.0


def normalize_sbus_channel(ch: float) -> float:
    """Map an SBUS channel value (172..1811) to [-1.0, 1.0].

    SBUS center (992) maps to 0.0, min (172) to -1.0, max (1811) to ~0.9994
    (the range is asymmetric: 819 below center, 820 above). Values outside
    [172, 1811] are clamped.
    """
    normalized = (ch - SBUS_CENTER) / SBUS_HALF_RANGE
    return max(-1.0, min(1.0, normalized))


def compute_axes(channels) -> list:
    """Convert SBUS channel values to normalized Joy axes [-1.0, 1.0]."""
    return [normalize_sbus_channel(ch) for ch in channels]


def compute_buttons(channels, threshold: float = ARM_THRESHOLD) -> list:
    """Convert SBUS channel values to digital button array (1 if > threshold)."""
    return [1 if ch > threshold else 0 for ch in channels]


def is_armed(buttons, ch_arm: int) -> bool:
    """True if the ARM channel button is high (channel value > threshold)."""
    if ch_arm < 0 or len(buttons) <= ch_arm:
        return False
    return buttons[ch_arm] == 1


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply deadzone: values with |value| < deadzone become 0.0.

    Values outside the deadzone are re-scaled to the full [-1.0, 1.0] range
    so the usable stick travel still reaches max speed.
    """
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return sign * scaled


def compute_twist(
    axes,
    ch_pitch: int,
    ch_yaw: int,
    max_linear: float,
    max_angular: float,
    deadzone: float,
):
    """Map joy axes to (linear_x, angular_z) using the configured channels."""
    linear_x = 0.0
    angular_z = 0.0
    if len(axes) > ch_pitch:
        linear_x = apply_deadzone(axes[ch_pitch], deadzone) * max_linear
    if len(axes) > ch_yaw:
        angular_z = apply_deadzone(axes[ch_yaw], deadzone) * max_angular
    return linear_x, angular_z


class ArmAnnouncer:
    """Decides WHEN a change of the ARM state may be announced by voice (issue #4003).

    Speech only. Driving (``/joystick_lock``, the brake message, ``cmd_vel_joy``)
    keeps following the raw channel with no delay: the node does not ask this
    class whether it may stop.

    Rules (``now`` is injected, so tests need no clock):

    * the state is *stable* only after the new raw value has been held for
      ``hold_s``; a glitch shorter than that never changes it;
    * only a stable state that differs from the last *announced* one is
      announced, so ON-OFF-ON that settles back where it started is silent;
    * announcements are at least ``min_interval_s`` apart; a change that is not
      yet allowed to speak waits and is announced by a later :meth:`update`
      with its *current* stable value, never with a stale one.

    The initial state is "disarmed" and counts as announced: a robot that boots
    with the switch down says nothing.
    """

    def __init__(
        self,
        hold_s: float = ARM_ANNOUNCE_HOLD_S,
        min_interval_s: float = ARM_ANNOUNCE_MIN_INTERVAL_S,
    ) -> None:
        self.hold_s = max(0.0, float(hold_s))
        self.min_interval_s = max(0.0, float(min_interval_s))
        self.stable = False
        self._announced = False
        self._candidate = False
        self._candidate_since: Optional[float] = None
        self._last_announce: Optional[float] = None

    def update(self, armed: bool, now: float) -> Optional[bool]:
        """Feed one raw sample; return the state to announce now, or ``None``."""
        if self._candidate_since is None or armed != self._candidate:
            self._candidate = armed
            self._candidate_since = now
        if self._candidate != self.stable and now - self._candidate_since >= self.hold_s:
            self.stable = self._candidate
        if self.stable == self._announced:
            return None
        if self._last_announce is not None and now - self._last_announce < self.min_interval_s:
            return None
        self._announced = self.stable
        self._last_announce = now
        return self.stable
