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


# -- HIL: звук и голос защиты не уходят на настоящего робота (#4470) -----------------------------------
def _sound_msgs(n):
    return [pub for topic, pub in n.publishers if topic == guard.SOUND_TRIGGER_TOPIC][0].messages


def _hil_node(node_params, hil):
    node_params.update(footprint_file=str(CONFIG / "nav2" / "nav2_params.yaml"),
                       twist_mux_file=str(CONFIG / "twist_mux" / "twist_mux.yaml"), hil_mute_alarm=hil)
    n = guard.ProximityGuardNode()
    n._mount = LIDAR
    n.clock = 0.0
    n._now = lambda: n.clock
    return n


@pytest.fixture
def spoken(monkeypatch):
    said = []
    monkeypatch.setattr(guard.speech_client, "say", lambda node, utt, agent=None: said.append(utt.text))
    return said


def test_hil_mutes_sound_and_speech_but_keeps_lights_and_emergency_stop(node_params, spoken, monkeypatch):
    n = _hil_node(node_params, True)
    lights = []
    monkeypatch.setattr(guard.expression_client, "express", lambda *a, **kw: lights.append(kw["gesture"]))
    drive(n, 3.0, 0.10, 0.10)
    assert n.tracker.stopping and len(stops(n)) > 0       # безопасность не тронута
    assert lights                                          # мигалка осталась
    assert _sound_msgs(n) == [] and spoken == []           # писка и «Осторожно!» нет
    muted = [c for c in n.get_logger().info.call_args_list if "HIL" in str(c)]
    assert len(muted) == 1                                 # одна строка на вход в зону, без спама


def test_live_mode_still_beeps_and_speaks(node_params, spoken, monkeypatch):
    n = _hil_node(node_params, False)
    monkeypatch.setattr(guard.expression_client, "express", lambda *a, **kw: None)
    drive(n, 3.0, 0.10, 0.10)
    assert len(_sound_msgs(n)) >= 2 and spoken
    assert not [c for c in n.get_logger().info.call_args_list if "HIL" in str(c)]


def _node_from_env(node_params):
    node_params.update(footprint_file=str(CONFIG / "nav2" / "nav2_params.yaml"))
    return guard.ProximityGuardNode()


@pytest.mark.parametrize("sim, guard_sound, muted", [
    ("true", "off", True),        # HIL + флаг off -> глушим
    ("true", "on", False),        # HIL, звук включён
    ("true", None, False),        # HIL, флага нет -> умолчание: звучит
    ("false", "off", False),      # боевой режим: флаг не действует
    (None, "off", False),
    (None, None, False),
])
def test_mute_only_in_hil_with_guard_sound_off(node_params, monkeypatch, sim, guard_sound, muted):
    for name, val in ((px.SIM_TIME_ENV, sim), (px.GUARD_SOUND_ENV, guard_sound)):
        monkeypatch.delenv(name, raising=False)
        if val is not None:
            monkeypatch.setenv(name, val)
    assert _node_from_env(node_params).hil_mute is muted


def test_hil_default_keeps_sound_and_speech(node_params, spoken, monkeypatch):
    n = _hil_node(node_params, False)  # HIL с HIL_GUARD_SOUND=on: hil_mute_alarm=False
    monkeypatch.setattr(guard.expression_client, "express", lambda *a, **kw: None)
    drive(n, 3.0, 0.10, 0.10)
    assert _sound_msgs(n) and spoken


def test_split_hil_effects_filters_only_in_hil():
    effects = [px.Effect(px.EFFECT_LIGHTS, "a", 1), px.Effect(px.EFFECT_SOUND, "b"), px.Effect(px.EFFECT_SPEECH, "c")]
    kept, muted = px.split_hil_effects(effects, True)
    assert [e.kind for e in kept] == [px.EFFECT_LIGHTS]
    assert [e.kind for e in muted] == [px.EFFECT_SOUND, px.EFFECT_SPEECH]
    assert px.split_hil_effects(effects, False) == (effects, [])


def test_hil_overlay_and_script_pass_the_same_flag_to_proximity_guard():
    overlay = (REPO / "docker" / "main" / "compose.hil.yaml").read_text(encoding="utf-8")
    assert "proximity-guard:\n    environment:\n      - USE_SIM_TIME=true" in overlay
    assert "HIL_GUARD_SOUND=${HIL_GUARD_SOUND:-on}" in overlay
    assert "proximity-guard" in (REPO / "scripts" / "hil" / "hil.sh").read_text(encoding="utf-8")
