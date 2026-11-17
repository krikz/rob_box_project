"""Guard: оверлеи HIL (эпик #4371) — сим под ID робота, Main Pi на sim time, боевые данные не тронуты.

План docs/research/2026-11-04-hil-robot-on-gazebo-data.md §1.2, §3.3, §5. Проверяется,
что запустит docker compose с оверлеем (слияние как у compose: env и тома по ключу/target),
а не тексты. Docker не нужен. Боевой режим сима (compose.yaml без оверлея) проверяет
test_sim_compose.py — его тесты не меняются.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
SIM = REPO_ROOT / "docker" / "sim"
MAIN = REPO_ROOT / "docker" / "main"

NAV_SERVICES = ("robot-state-publisher", "twist-mux", "rtabmap", "imu-filter", "nav2")


class _Loader(yaml.SafeLoader):
    """SafeLoader + теги слияния compose (``!override`` / ``!reset``)."""


def _passthrough(loader, node):
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node)
    if isinstance(node, yaml.MappingNode):
        return loader.construct_mapping(node)
    return loader.construct_scalar(node)


_Loader.add_constructor("!override", _passthrough)
_Loader.add_constructor("!reset", _passthrough)


def _load(path: Path) -> dict:
    return yaml.load(path.read_text(encoding="utf-8"), Loader=_Loader)  # noqa: S506 — SafeLoader


SIM_BASE = _load(SIM / "compose.yaml")["services"]
SIM_HIL = _load(SIM / "compose.hil.yaml")["services"]
MAIN_BASE = _load(MAIN / "docker-compose.yaml")["services"]
MAIN_HIL = _load(MAIN / "compose.hil.yaml")["services"]


def _env(service: dict) -> dict[str, str]:
    env = service.get("environment") or {}
    if isinstance(env, list):
        return dict(item.partition("=")[::2] for item in env)
    return {k: str(v) for k, v in env.items()}


def _merged_env(base: dict, overlay: dict) -> dict[str, str]:
    return {**_env(base), **_env(overlay)}


def _merged_volumes(base: dict, overlay: dict) -> dict[str, str]:
    """target → source, как compose сливает тома оверлея (по target)."""
    out = {}
    for v in [*(base.get("volumes") or []), *(overlay.get("volumes") or [])]:
        parts = v.split(":")
        src, target = (parts[0], parts[1]) if not v.startswith("${") else _split_var_volume(v)
        out[target] = src
    return out


def _split_var_volume(v: str) -> tuple[str, str]:
    m = re.match(r"^(\$\{[^}]*\}):([^:]+)", v)
    assert m, v
    return m.group(1), m.group(2)


# ── Песочница ────────────────────────────────────────────────────────────────


def test_hil_runs_only_router_and_gazebo_on_sandbox():
    active = {
        name for name in {*SIM_BASE, *SIM_HIL}
        if not (SIM_HIL.get(name, {}).get("profiles") or SIM_BASE.get(name, {}).get("profiles"))
    }
    assert active == {"zenoh-router", "gazebo"}
    for name in NAV_SERVICES:
        assert SIM_HIL[name]["profiles"] == ["sim-only"], name
    assert SIM_HIL["gazebo"]["depends_on"] == ["zenoh-router"]


def test_hil_publishes_under_robot_id_not_sim():
    for name in ("zenoh-router", "gazebo"):
        env = _merged_env(SIM_BASE[name], SIM_HIL[name])
        assert env["ROBOT_ID"].startswith("${HIL_ROBOT_ID:?"), name   # без ID — отказ compose
        assert env["ZENOH_LISTEN_IP"].startswith("${HIL_SANDBOX_IP:?"), name
        assert SIM_HIL[name]["network_mode"] == "host", name
    assert _env(SIM_HIL["zenoh-router"])["ZENOH_CONNECT_ENDPOINTS"].startswith("${HIL_ZENOH_CONNECT:?")


def test_base_sim_mode_stays_isolated():
    # Боевой режим сима не меняется: ID sim, сети нет.
    assert _env(SIM_BASE["zenoh-router"])["ROBOT_ID"] == "sim"
    assert SIM_BASE["zenoh-router"]["network_mode"] == "none"
    assert "hil_run.sh" not in " ".join(SIM_BASE["gazebo"]["command"])


def test_hil_gazebo_runs_under_guard():
    cmd = SIM_HIL["gazebo"]["command"]
    assert cmd[:2] == ["/ros_scripts/ros_with_namespace.sh", "/hil_run.sh"]
    assert "./hil_run.sh:/hil_run.sh:ro" in SIM_HIL["gazebo"]["volumes"]
    script = (SIM / "hil_run.sh").read_text(encoding="utf-8")
    # Порядок: precheck до launch, launch с hil:=true, watch рядом.
    assert script.index("python3 -m rob_box_sim.hil precheck") < script.index(
        'ros2 launch rob_box_sim sim.launch.py hil:=true "$@" &'
    )
    assert "python3 -m rob_box_sim.hil watch" in script
    assert _env(SIM_HIL["gazebo"])["HIL_GUARD_NO_LINK"] == "${HIL_GUARD_NO_LINK:-0}"


# ── Main Pi ──────────────────────────────────────────────────────────────────


def test_main_overlay_touches_only_nav_loop():
    # + proximity-guard: звук защиты в HIL гасится флагом HIL_GUARD_SOUND (#4470)
    # + followpath-watchdog (#4458): сторож сирот FollowPath считает таймаут по sim time
    assert set(MAIN_HIL) == set(NAV_SERVICES) | {"proximity-guard", "followpath-watchdog"}
    assert "ros2-control" not in MAIN_HIL and "lslidar" not in MAIN_HIL
    for name, svc in MAIN_HIL.items():
        assert set(svc) <= {"environment", "volumes"}, name
        assert _env(svc)["USE_SIM_TIME"] == "true", name
        allowed = {"USE_SIM_TIME"} | ({"RTABMAP_START_MODE"} if name == "rtabmap" else set())
        allowed |= {"HIL_GUARD_SOUND"} if name == "proximity-guard" else set()
        if name == "proximity-guard":
            assert _env(svc)["HIL_GUARD_SOUND"] == "${HIL_GUARD_SOUND:-on}"
        assert set(_env(svc)) == allowed, name


def test_main_overlay_moves_maps_off_live_map():
    for name in ("rtabmap", "nav2"):
        base_maps = _merged_volumes(MAIN_BASE[name], {})["/maps"]
        assert base_maps == "./maps", name   # боевая карта мастерской
        merged = _merged_volumes(MAIN_BASE[name], MAIN_HIL[name])
        assert merged["/maps"].startswith("${HIL_MAPS_DIR:?"), (name, merged["/maps"])
    text = (MAIN / "compose.hil.yaml").read_text(encoding="utf-8")
    assert "/data" not in text.split("services:", 1)[1]


def test_main_overlay_is_not_picked_up_by_deploy_or_audit():
    # Деплой и architecture_audit читают docker/*/docker-compose*.y*ml — оверлей HIL туда не попадает.
    assert not re.fullmatch(r"docker-compose.*\.ya?ml", "compose.hil.yaml")


def test_main_rtabmap_mapping_only_with_sim_time():
    env = _merged_env(MAIN_BASE["rtabmap"], MAIN_HIL["rtabmap"])
    assert env["RTABMAP_START_MODE"] == "${HIL_RTABMAP_MODE:-mapping}"
    assert env["USE_SIM_TIME"] == "true"
