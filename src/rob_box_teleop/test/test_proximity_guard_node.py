"""proximity_guard_node (#4365, HIL 05.11): стоп держится, пока команда в twist_mux толкает в стену; SIGTERM — выход.

Нода собирается на заглушках rclpy (conftest); скан и одометрия — ``SimpleNamespace`` с полями сообщений.
"""

import math
import signal
import types
from pathlib import Path

import pytest

from rob_box_teleop import proximity as px
from rob_box_teleop import proximity_guard_node as guard

REPO = Path(__file__).resolve().parents[3]
CONFIG = REPO / "docker" / "main" / "config"
LIDAR = px.Mount(-0.171, 0.0, 0.0)  # крепление lslidar_n10 на РОББОКСе (HIL 05.11)
N = 720


@pytest.fixture
def node(node_params):
    node_params["footprint_file"] = str(CONFIG / "nav2" / "nav2_params.yaml")
    node_params["twist_mux_file"] = str(CONFIG / "twist_mux" / "twist_mux.yaml")
    n = guard.ProximityGuardNode()
    n._mount = LIDAR
    n.clock = 0.0
    n._now = lambda: n.clock
    return n


def wall_scan(x):
    """Скан lslidar: стена поперёк хода в ``x`` (base_footprint), шириной 2 м."""
    ranges = []
    for i in range(N):
        a = -math.pi + i * 2 * math.pi / N
        r = (x - LIDAR.x) / math.cos(a) if math.cos(a) > 1e-6 else math.inf
        ranges.append(r if abs(r * math.sin(a)) <= 1.0 else math.inf)
    return types.SimpleNamespace(header=types.SimpleNamespace(frame_id="laser"), ranges=ranges, angle_min=-math.pi,
                                 angle_increment=2 * math.pi / N, range_min=0.15, range_max=12.0)


def odom(v):
    t = types.SimpleNamespace(linear=types.SimpleNamespace(x=v), angular=types.SimpleNamespace(z=0.0))
    return types.SimpleNamespace(twist=types.SimpleNamespace(twist=t))


def twist(v):
    return types.SimpleNamespace(linear=types.SimpleNamespace(x=v), angular=types.SimpleNamespace(z=0.0))


def subscription(n, topic):
    return dict(n.subscriptions)[topic]


def stops(n):
    return [pub for topic, pub in n.publishers if topic == guard.EMERGENCY_TOPIC][0].messages


def drive(n, seconds, v_odom, cmd, gap=0.18):
    """``seconds`` тактов по 0.05 с: скан со стеной в ``gap`` от носа, одометрия ``v_odom``, команда Nav2 ``cmd``."""
    for _ in range(round(seconds / 0.05)):
        n.clock += 0.05
        n._on_scan(wall_scan(n.body.x_max + gap))
        n._on_odom(odom(v_odom))
        if cmd is not None:
            subscription(n, "cmd_vel")(twist(cmd))
        n._on_tick()


def test_node_listens_to_every_twist_mux_input_but_its_own_emergency_output(node):
    topics = {t for t, _ in node.subscriptions}
    assert {"cmd_vel", "cmd_vel_joy", "cmd_vel_quest", "cmd_vel_web", "cmd_vel_voice"} <= topics
    assert guard.EMERGENCY_TOPIC not in topics


def test_stop_is_held_while_nav2_keeps_pushing_and_released_when_it_backs_off(node):
    drive(node, 0.1, 0.10, 0.10)                          # въехал в 0.18 м — стоп
    assert node.tracker.stopping
    drive(node, 5.0, 0.0, 0.10)                           # встал, Nav2 толкает 5 с
    assert node.tracker.stopping
    sent = len(stops(node))
    assert sent >= 100                                    # нулевой Twist шёл все 5 с на 20 Гц
    drive(node, 1.0, 0.0, -0.05)                          # Nav2 сдаёт назад
    assert not node.tracker.stopping
    assert len(stops(node)) - sent <= 12                  # отпустил за release_s (+ такт)


def test_without_a_command_the_stop_releases_after_stop_hold_like_before(node):
    drive(node, 0.1, 0.10, None)
    drive(node, 2.1, 0.0, None)
    assert not node.tracker.stopping


def test_new_thresholds_are_node_parameters(node_params):
    node_params.update(footprint_file=str(CONFIG / "nav2" / "nav2_params.yaml"), side_stop_m=0.05,
                       spin_stop_m=0.07, corridor_margin_m=0.03)
    n = guard.ProximityGuardNode()
    assert (n.th.side_stop_m, n.th.spin_stop_m, n.th.corridor_margin_m) == (0.05, 0.07, 0.03)
    assert guard._DEFAULTS["side_stop_m"] == px.Thresholds.side_stop_m  # умолчания — из одного места


def test_missing_twist_mux_file_disables_only_the_hold(node_params, tmp_path):
    node_params.update(footprint_file=str(CONFIG / "nav2" / "nav2_params.yaml"),
                       twist_mux_file=str(tmp_path / "nope.yaml"))
    n = guard.ProximityGuardNode()
    assert list(n.mux.sources) == []
    assert "twist_mux" in str(n.get_logger().error.call_args)


def test_sigterm_interrupts_spin_like_ctrl_c():
    """HIL 05.11: ``docker stop proximity-guard`` → Exited (137) через 10 с — нода не выходила по SIGTERM."""
    old = signal.getsignal(signal.SIGTERM)
    try:
        guard.exit_on_sigterm()
        with pytest.raises(KeyboardInterrupt):
            signal.raise_signal(signal.SIGTERM)
    finally:
        signal.signal(signal.SIGTERM, old)


def test_start_script_runs_the_node_as_pid_1_not_through_ros2_run():
    script = (REPO / "docker" / "main" / "scripts" / "teleop" / "start_proximity_guard.sh").read_text(encoding="utf-8")
    assert "exec python3 -m rob_box_teleop.proximity_guard_node" in script
    assert "ros2 run" not in script.split("exec ", 1)[1]
