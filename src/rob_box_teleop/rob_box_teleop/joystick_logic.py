"""Pure joystick/SBUS logic for joystick_control_node (no ROS dependencies).

Extracted so the channel→axis mapping, deadzone and ARM-gating logic can be
unit-tested without a ROS2 runtime (mirrors the pattern used by
rob_box_voice unit tests: mock rclpy, test pure methods).
"""

import json

# SBUS channel value range (ExpressLRS)
SBUS_MIN = 172
SBUS_CENTER = 992
SBUS_MAX = 1811
SBUS_HALF_RANGE = (SBUS_MAX - SBUS_MIN) / 2.0  # 819.5

# ARM switch is considered HIGH above this raw channel value
ARM_THRESHOLD = 1500.0


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


# tts_node's single synthesis input (ADR-0080 §2.3). The legacy /tts/speak
# topic this node used to publish on has no subscriber anywhere (issue #3107).
TTS_REQUEST_TOPIC = "/voice/tts/request"


def build_tts_request(text: str) -> str:
    """Build the ``/voice/tts/request`` JSON payload for a spoken ``text``.

    tts_node drops plain strings (``json.loads`` fails) and chunks without an
    ``ssml`` field, so the payload mirrors ``rob_box_core.utterance.Utterance
    .to_request()``: escaped ``<speak>`` SSML, speakers sink, default priority
    and emotion. The teleop image builds only rob_box_teleop, so the contract
    is reproduced here instead of importing rob_box_core.
    """
    escaped = (text or "").replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    return json.dumps(
        {
            "ssml": f"<speak>{escaped}</speak>",
            "sink": "speakers",
            "priority": "normal",
            "emotion": "neutral",
        },
        ensure_ascii=False,
    )
