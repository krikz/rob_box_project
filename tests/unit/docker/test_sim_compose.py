"""Guard: docker/sim — профиль Gazebo поднимает БОЕВЫЕ узлы навигации Main Pi.

План docs/research/2026-11-04-gazebo-sim-navigation.md §6, шаг 2. Смысл сима —
крутить те же скрипты и конфиги, что на роботе, поэтому:

* robot-state-publisher, twist-mux, rtabmap, nav2, zenoh-router наследуются
  (``extends``) из docker/main/docker-compose.yaml и команду не переопределяют;
* сим отличается переменными USE_SIM_TIME / URDF_SIM, образом и сетью;
* сеть изолирована: ни host-сети, ни подключения роутера к облаку;
* боевая карта ``docker/main/maps`` в сим не монтируется.

Проверяется поведение compose-файлов (что запустит docker compose), а не тексты.
Docker не нужен.
"""

from __future__ import annotations

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
SIM = REPO_ROOT / "docker" / "sim"
MAIN = REPO_ROOT / "docker" / "main"

NAV_SERVICES = ("robot-state-publisher", "twist-mux", "rtabmap", "nav2")
EXTENDED = ("zenoh-router", *NAV_SERVICES)
# Железо робота, которое в симе заменяет сервис gazebo.
REPLACED_BY_GAZEBO = {"ros2-control", "lslidar"}


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


SIM_COMPOSE = _load(SIM / "compose.yaml")["services"]
MAIN_COMPOSE = _load(MAIN / "docker-compose.yaml")["services"]


def _env(service: dict) -> dict[str, str]:
    env = service.get("environment") or {}
    if isinstance(env, list):
        return dict(item.partition("=")[::2] for item in env)
    return {k: str(v) for k, v in env.items()}


def _depends(service: dict) -> set[str]:
    dep = service.get("depends_on") or []
    return set(dep if isinstance(dep, list) else dep.keys())


def test_nav_services_extend_robot_compose_without_own_command():
    for name in EXTENDED:
        svc = SIM_COMPOSE[name]
        assert svc["extends"] == {"file": "../main/docker-compose.yaml", "service": name}, name
        assert name in MAIN_COMPOSE, name
        # Своя команда = вторая реализация запуска: запрещено.
        assert "command" not in svc and "entrypoint" not in svc, name


def test_sim_time_switch_on_every_nav_node():
    for name in NAV_SERVICES:
        assert _env(SIM_COMPOSE[name]).get("USE_SIM_TIME") == "true", name
    assert _env(SIM_COMPOSE["gazebo"]).get("USE_SIM_TIME") == "true"
    assert _env(SIM_COMPOSE["robot-state-publisher"]).get("URDF_SIM") == "true"


def test_robot_compose_has_no_sim_switches():
    for name in NAV_SERVICES:
        env = _env(MAIN_COMPOSE[name])
        assert "USE_SIM_TIME" not in env and "URDF_SIM" not in env, name


def test_network_is_isolated_from_host_and_cloud():
    router = SIM_COMPOSE["zenoh-router"]
    assert router["network_mode"] == "none"
    endpoints = _env(router)["ZENOH_CONNECT_ENDPOINTS"]
    assert endpoints.startswith("tcp/127.0.0.1:"), endpoints
    assert _env(router)["ZENOH_LISTEN_IP"] == "127.0.0.1"
    for name, svc in SIM_COMPOSE.items():
        if name == "zenoh-router":
            continue
        assert svc["network_mode"] == "service:zenoh-router", name
        assert _env(svc)["ZENOH_LISTEN_IP"] == "127.0.0.1", name


def test_robot_map_is_not_mounted():
    for name in ("rtabmap", "nav2"):
        maps = [v for v in SIM_COMPOSE[name]["volumes"] if v.split(":")[1] == "/maps"]
        assert maps and all(v.startswith("sim-maps:") for v in maps), (name, maps)


def test_depends_on_only_sim_services_and_tracks_robot():
    for name, svc in SIM_COMPOSE.items():
        assert _depends(svc) <= set(SIM_COMPOSE), name
    robot_nav2 = _depends(MAIN_COMPOSE["nav2"])
    # Новая зависимость nav2 на роботе должна появиться и в симе (или быть заменена gazebo).
    assert robot_nav2 - REPLACED_BY_GAZEBO <= _depends(SIM_COMPOSE["nav2"])
    assert "gazebo" in _depends(SIM_COMPOSE["nav2"])


def test_gazebo_reads_robot_controller_config():
    vols = SIM_COMPOSE["gazebo"]["volumes"]
    assert "../main/config:/config:ro" in vols
    assert (MAIN / "config" / "controllers" / "controller_manager.yaml").is_file()


def test_video_overlay_changes_only_world_and_scenario():
    """compose.video.yaml (видео из Gazebo): навигация, сеть, железо — как у стенда."""
    video = _load(SIM / "compose.video.yaml")["services"]
    assert set(video) == {"gazebo", "bench"}
    for name, svc in video.items():
        assert set(svc) <= {"volumes", "command"}, name
    gz_cmd = " ".join(video["gazebo"]["command"])
    assert "rob_box_sim.video_overlay" in gz_cmd and "world:=/sim_video/world.sdf" in gz_cmd
    assert (REPO_ROOT / "src" / "rob_box_sim" / "scenarios" / "video.yaml").is_file()
    assert "/ws/install/rob_box_sim/share/rob_box_sim/scenarios/video.yaml" in video["bench"]["command"]


# ── Шаг 6: оверлеи Nav2 для сравнения контроллеров ───────────────────────────

SIM_NAV2 = REPO_ROOT / "src" / "rob_box_sim" / "config" / "nav2"
# Оверлей сима трогает только то, что сравнивается; остальное — боевой nav2_params.yaml.
OVERLAY_SECTIONS = {"controller_server", "planner_server", "global_costmap", "local_costmap"}


def _overlay_paths(controller: str, planner: str, footprint: str) -> list[Path]:
    raw = _env(SIM_COMPOSE["nav2"])["NAV2_PARAMS_OVERLAYS"]
    raw = raw.replace("${SIM_CONTROLLER:-dwb}", controller).replace("${SIM_PLANNER:-navfn}", planner)
    raw = raw.replace("${SIM_FOOTPRINT:-radius}", footprint)
    assert "$" not in raw, raw
    out = []
    for p in raw.split():
        assert p.startswith("/sim_nav2/"), p
        out.append(SIM_NAV2 / p[len("/sim_nav2/"):])
    return out


def test_nav2_overlays_mounted_and_exist_for_every_choice():
    assert "../../src/rob_box_sim/config/nav2:/sim_nav2:ro" in SIM_COMPOSE["nav2"]["volumes"]
    for controller in ("dwb", "rpp", "mppi"):
        for planner in ("navfn", "smac"):
            for footprint in ("radius", "urdf"):
                paths = _overlay_paths(controller, planner, footprint)
                assert paths[0].name == "sim.yaml" and len(paths) == 4
                for p in paths:
                    assert p.is_file(), p


def test_footprint_radius_overlay_is_robot_value():
    robot = yaml.safe_load((MAIN / "config" / "nav2" / "nav2_params.yaml").read_text(encoding="utf-8"))
    doc = yaml.safe_load((SIM_NAV2 / "footprint_radius.yaml").read_text(encoding="utf-8"))
    for cm in ("local_costmap", "global_costmap"):
        assert doc[cm][cm]["ros__parameters"]["robot_radius"] == robot[cm][cm]["ros__parameters"]["robot_radius"]


def test_nav2_overlays_touch_only_compared_sections_and_name_plugin():
    plugins = {}
    for p in sorted(SIM_NAV2.glob("*.yaml")):
        doc = yaml.safe_load(p.read_text(encoding="utf-8"))
        assert set(doc) <= OVERLAY_SECTIONS, (p.name, set(doc))
        if p.name.startswith("controller_"):
            plugins[p.stem] = doc["controller_server"]["ros__parameters"]["FollowPath"]["plugin"]
        if p.name.startswith("planner_"):
            plugins[p.stem] = doc["planner_server"]["ros__parameters"]["GridBased"]["plugin"]
    assert plugins == {
        "controller_dwb": "dwb_core::DWBLocalPlanner",
        "controller_mppi": "nav2_mppi_controller::MPPIController",
        "controller_rpp": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        "planner_navfn": "nav2_navfn_planner/NavfnPlanner",
        "planner_smac": "nav2_smac_planner/SmacPlanner2D",
    }


def test_sim_global_costmap_takes_rtabmap_map_and_robot_does_not():
    doc = yaml.safe_load((SIM_NAV2 / "sim.yaml").read_text(encoding="utf-8"))
    gc = doc["global_costmap"]["global_costmap"]["ros__parameters"]
    assert gc["plugins"][0] == "static_layer"
    assert gc["static_layer"]["map_topic"] == "/rtabmap/map"
    robot = yaml.safe_load((MAIN / "config" / "nav2" / "nav2_params.yaml").read_text(encoding="utf-8"))
    assert "static_layer" not in robot["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]


def test_sim_smoother_chain_on_by_default_robot_has_no_switch():
    assert _env(SIM_COMPOSE["nav2"])["NAV2_SMOOTH_CMD_VEL"] == "${SIM_SMOOTH_CMD_VEL:-true}"
    env = _env(MAIN_COMPOSE["nav2"])
    assert "NAV2_SMOOTH_CMD_VEL" not in env and "NAV2_PARAMS_OVERLAYS" not in env
