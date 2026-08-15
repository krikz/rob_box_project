"""Config-consistency guards for the VESC drive train (BG-6, issue #817).

BG-6: wheel jitter at start / hard braking. Root causes addressed at config
level (rob_box_project, runtime source of truth = URDF hardware params):

1. ``min_duty`` deadzone compensation makes ANY nonzero velocity map to a duty
   step >= min_duty (see ``VescHandler::sendSpeed`` in the vesc_nexus
   submodule). A smaller min_duty => smaller start step and smaller
   zero-crossing reversal (2*min_duty) at braking.
2. ``max_acceleration`` in the diff_drive_controller ramps the velocity
   command; 0.5 m/s^2 gives a gentler start/stop than 1.0.
3. ``gear_ratio`` / ``control_mode`` / ``max_rps`` in the standalone
   ``vesc_config.yaml`` had drifted from the calibrated URDF values (5.0 vs
   2.17, rpm vs duty, 6.5 vs 12.2). This test keeps the two files in sync so a
   standalone vesc_nexus run does not behave differently from the deployed
   ros2-control stack.

Run explicitly (the root pytest.ini is scoped to rob_box_harness):

    python3 -m pytest tests/unit/test_control_config_consistency.py -v

The test is offline / no ROS required.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]

URDF_XACRO = REPO_ROOT / "src" / "rob_box_description" / "urdf" / "rob_box_ros2_control.xacro"
VESC_CONFIG = REPO_ROOT / "docker" / "main" / "config" / "vesc_nexus" / "vesc_config.yaml"
RUNTIME_CTRL = REPO_ROOT / "docker" / "main" / "config" / "controllers" / "controller_manager.yaml"
MIRROR_CTRL = REPO_ROOT / "docker" / "main" / "config" / "vesc_nexus" / "robot_controller.yaml"

# Sanity bounds for the tuned values (BG-6 tuning, 15.08).
# Tune deliberately, then update here — the test guards against accidental drift.
EXPECTED_GEAR_RATIO = 2.17
EXPECTED_MIN_DUTY = 0.04
EXPECTED_MAX_RPS = 12.2
EXPECTED_CONTROL_MODE = "duty"
EXPECTED_MAX_ACCEL = 0.5  # m/s^2 — smooth start/stop
EXPECTED_WHEEL_RADIUS = 0.1143  # m (9" wheels with gearboxes)
EXPECTED_WHEEL_SEPARATION = 1.11  # m (4WD skid-steer calibration)
EXPECTED_CMD_VEL_TIMEOUT = 0.5  # s

# --- helpers ----------------------------------------------------------------


def load_urdf_params(path: Path) -> dict:
    """Extract hardware + joint params from rob_box_ros2_control.xacro."""
    tree = ET.parse(str(path))
    root = tree.getroot()
    ns = {"xacro": "http://www.ros.org/wiki/xacro"}
    hw = {}
    for el in root.iter("param"):
        name = el.attrib.get("name")
        if name:
            hw[name] = el.text.strip() if el.text else ""
    return hw


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def param_value(node: dict, dotted_key: str):
    """Read a param from a ros2_control YAML that may use EITHER flat dotted
    keys (``linear.x.max_acceleration: 0.5``) or nested maps
    (``linear: {x: {max_acceleration: 0.5}}``)."""
    parts = dotted_key.split(".")
    cur = node
    for p in parts:
        if isinstance(cur, dict) and p in cur:
            cur = cur[p]
        else:
            # flat dotted key (e.g. "linear.x.max_acceleration")
            if dotted_key in node:
                return node[dotted_key]
            return None
    return cur


def duty_at(linear_speed: float, min_duty: float, max_speed_mps: float) -> float:
    """Mirror of VescHandler::sendSpeed() duty mapping (vesc_nexus submodule).

    Returns the duty cycle sent to the VESC for a given linear speed in duty
    control mode (open loop, with deadzone compensation).
    """
    if max_speed_mps <= 0.0:
        return 0.0
    target_duty = max(-1.0, min(1.0, linear_speed / max_speed_mps))
    if min_duty > 0.0 and target_duty != 0.0:
        abs_duty = abs(target_duty)
        scaled = min_duty + abs_duty * (1.0 - min_duty)
        return scaled if target_duty > 0 else -scaled
    return target_duty


# --- fixtures ---------------------------------------------------------------


@pytest.fixture(scope="module")
def urdf_params():
    assert URDF_XACRO.exists(), f"URDF not found: {URDF_XACRO}"
    return load_urdf_params(URDF_XACRO)


@pytest.fixture(scope="module")
def vesc_cfg():
    assert VESC_CONFIG.exists(), f"vesc_config.yaml not found: {VESC_CONFIG}"
    return load_yaml(VESC_CONFIG)


@pytest.fixture(scope="module")
def runtime_ctrl():
    assert RUNTIME_CTRL.exists(), f"runtime controller_manager.yaml not found: {RUNTIME_CTRL}"
    return load_yaml(RUNTIME_CTRL)


@pytest.fixture(scope="module")
def mirror_ctrl():
    assert MIRROR_CTRL.exists(), f"mirror robot_controller.yaml not found: {MIRROR_CTRL}"
    return load_yaml(MIRROR_CTRL)


# --- URDF vs standalone vesc_config.yaml ------------------------------------


class TestUrdfVescConfigConsistency:
    """Runtime source of truth (URDF hardware params) must match the
    standalone vesc_config.yaml — otherwise a standalone vesc_nexus run
    behaves differently from the deployed ros2-control stack."""

    def test_gear_ratio_matches(self, urdf_params, vesc_cfg):
        assert float(urdf_params["gear_ratio"]) == pytest.approx(EXPECTED_GEAR_RATIO)
        params = vesc_cfg["/**"]["ros__parameters"]
        assert float(params["gear_ratio"]) == pytest.approx(float(urdf_params["gear_ratio"])), (
            "gear_ratio drift: vesc_config.yaml != rob_box_ros2_control.xacro"
        )

    def test_min_duty_matches(self, urdf_params, vesc_cfg):
        assert float(urdf_params["min_duty"]) == pytest.approx(EXPECTED_MIN_DUTY)
        params = vesc_cfg["/**"]["ros__parameters"]
        assert float(params["min_duty"]) == pytest.approx(float(urdf_params["min_duty"])), (
            "min_duty drift: vesc_config.yaml != rob_box_ros2_control.xacro"
        )

    def test_control_mode_matches(self, urdf_params, vesc_cfg):
        assert urdf_params["control_mode"].strip().lower() == EXPECTED_CONTROL_MODE
        params = vesc_cfg["/**"]["ros__parameters"]
        assert str(params["control_mode"]).strip().lower() == urdf_params["control_mode"].strip().lower(), (
            "control_mode drift: vesc_config.yaml != rob_box_ros2_control.xacro"
        )

    def test_max_rps_matches(self, urdf_params, vesc_cfg):
        # URDF sets per-joint max_rps; vesc_config uses wheel_max_rps list.
        joint_rps = [float(v) for k, v in urdf_params.items() if k == "max_rps"]
        assert joint_rps, "no max_rps params found in URDF joints"
        assert all(r == pytest.approx(EXPECTED_MAX_RPS) for r in joint_rps)
        params = vesc_cfg["/**"]["ros__parameters"]
        assert params["wheel_max_rps"] == pytest.approx([EXPECTED_MAX_RPS] * 4), (
            "wheel_max_rps drift: vesc_config.yaml != rob_box_ros2_control.xacro"
        )

    def test_wheel_radius_matches(self, urdf_params, vesc_cfg):
        assert float(urdf_params["wheel_radius"]) == pytest.approx(EXPECTED_WHEEL_RADIUS)
        params = vesc_cfg["/**"]["ros__parameters"]
        assert params["wheel_radii"] == pytest.approx([EXPECTED_WHEEL_RADIUS] * 4)


# --- controller configs: runtime vs mirror ----------------------------------


class TestControllerConfigConsistency:
    """docker/main/config/controllers/controller_manager.yaml is the deployed
    file; docker/main/config/vesc_nexus/robot_controller.yaml is a mirror used
    by calibration tooling. Keep them in sync."""

    def test_linear_acceleration_limits_match(self, runtime_ctrl, mirror_ctrl):
        r = runtime_ctrl["diff_drive_controller"]["ros__parameters"]
        m = mirror_ctrl["diff_drive_controller"]["ros__parameters"]
        assert param_value(r, "linear.x.max_acceleration") == pytest.approx(EXPECTED_MAX_ACCEL)
        assert param_value(m, "linear.x.max_acceleration") == pytest.approx(
            param_value(r, "linear.x.max_acceleration")
        )
        assert param_value(m, "linear.x.min_acceleration") == pytest.approx(
            param_value(r, "linear.x.min_acceleration")
        )

    def test_wheel_separation_matches(self, runtime_ctrl, mirror_ctrl):
        r = runtime_ctrl["diff_drive_controller"]["ros__parameters"]
        m = mirror_ctrl["diff_drive_controller"]["ros__parameters"]
        assert param_value(r, "wheel_separation") == pytest.approx(EXPECTED_WHEEL_SEPARATION)
        assert param_value(m, "wheel_separation") == pytest.approx(param_value(r, "wheel_separation"))

    def test_cmd_vel_timeout(self, runtime_ctrl, mirror_ctrl):
        r = runtime_ctrl["diff_drive_controller"]["ros__parameters"]
        m = mirror_ctrl["diff_drive_controller"]["ros__parameters"]
        assert param_value(r, "cmd_vel_timeout") == pytest.approx(EXPECTED_CMD_VEL_TIMEOUT)
        assert param_value(m, "cmd_vel_timeout") == pytest.approx(param_value(r, "cmd_vel_timeout"))

    def test_open_loop_matches(self, runtime_ctrl, mirror_ctrl):
        r = runtime_ctrl["diff_drive_controller"]["ros__parameters"]
        m = mirror_ctrl["diff_drive_controller"]["ros__parameters"]
        assert param_value(m, "open_loop") is True
        assert param_value(r, "open_loop") is True

    def test_base_frame_matches(self, runtime_ctrl, mirror_ctrl):
        r = runtime_ctrl["diff_drive_controller"]["ros__parameters"]
        m = mirror_ctrl["diff_drive_controller"]["ros__parameters"]
        assert param_value(r, "base_frame_id") == "base_footprint"
        assert param_value(m, "base_frame_id") == "base_footprint"


# --- duty mapping sanity (mirror of vesc_nexus sendSpeed) -------------------


class TestDutyMappingSmoothness:
    """With the tuned min_duty, the deadzone-compensated duty mapping must keep
    the start step and the braking zero-crossing reversal small enough to avoid
    visible jitter in the 0.1–0.5 m/s band (BG-6 acceptance)."""

    @pytest.fixture(scope="module")
    def mapping(self, urdf_params):
        gear_ratio = float(urdf_params["gear_ratio"])
        max_rps = float(urdf_params["max_rps"])
        wheel_radius = float(urdf_params["wheel_radius"])
        min_duty = float(urdf_params["min_duty"])
        # max_speed = 2π × (max_rps / gear_ratio) × wheel_radius
        max_speed = 2.0 * math.pi * (max_rps / gear_ratio) * wheel_radius
        return {"min_duty": min_duty, "max_speed": max_speed}

    def test_start_step_at_0_1_mps_is_small(self, mapping):
        """duty(0.1 m/s) must stay well below 10% — a bigger step at start is
        the classic jitter/inrush trigger."""
        d = duty_at(0.1, mapping["min_duty"], mapping["max_speed"])
        assert 0.0 < d < 0.10, f"start step too large: duty(0.1 m/s) = {d:.3f}"

    def test_zero_crossing_reversal_is_bounded(self, mapping):
        """Braking through zero flips duty by 2×min_duty in one 20ms tick.
        With min_duty=0.04 the swing is 8%; keep it < 12%."""
        swing = 2.0 * mapping["min_duty"]
        assert swing < 0.12, f"zero-crossing duty swing too large: {swing:.3f}"

    def test_low_speed_band_is_monotonic(self, mapping):
        speeds = [0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5]
        duties = [duty_at(s, mapping["min_duty"], mapping["max_speed"]) for s in speeds]
        assert duties == sorted(duties), "duty mapping must be monotonic in 0–0.5 m/s"

    def test_duty_at_0_is_zero(self, mapping):
        assert duty_at(0.0, mapping["min_duty"], mapping["max_speed"]) == 0.0
