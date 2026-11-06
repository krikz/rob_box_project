"""Параметры RTAB-Map на Main Pi — один источник: patch_rtabmap_launch.py (#4495).

rtabmap.ini не применялся (rtabmap читает из ini только секцию [Core], а у нас были [Icp], [Odom], …),
rtabmap.yaml не подключался ни одним сервисом. Оба удалены; cfg:= в compose нет.
Решение по каждому параметру — docs/research/2026-11-05-nav2-hil-tuning.md §9.6.
"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
RTABMAP_CONFIG_DIR = REPO_ROOT / "docker/main/config/rtabmap"
MAIN_COMPOSE = REPO_ROOT / "docker/main/docker-compose.yaml"
PATCH = REPO_ROOT / "docker/main/scripts/rtabmap/patch_rtabmap_launch.py"


def load_module():
    spec = spec_from_file_location("patch_rtabmap_launch", PATCH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_no_second_source_of_rtabmap_params():
    assert not (RTABMAP_CONFIG_DIR / "rtabmap.ini").exists()
    assert not (RTABMAP_CONFIG_DIR / "rtabmap.yaml").exists()
    command = yaml.safe_load(MAIN_COMPOSE.read_text(encoding="utf-8"))["services"]["rtabmap"]["command"]
    assert not any(arg.startswith(("cfg:=", "args:=", "rtabmap_args:=", "odom_args:=")) for arg in command)


def test_rtabmap_and_odometry_params_do_not_overlap():
    """Odom/* — только у icp_odometry: узел rtabmap их стирает (rtabmap_ros CoreWrapper)."""
    module = load_module()
    assert not set(module.RTABMAP_PARAMS) & set(module.ICP_ODOMETRY_PARAMS)
    assert not [k for k in module.RTABMAP_PARAMS if k.startswith("Odom/")]
    assert all(isinstance(v, str) for v in {**module.RTABMAP_PARAMS, **module.ICP_ODOMETRY_PARAMS}.values())


def test_launch_sets_incremental_memory_itself():
    """Mem/IncrementalMemory задаёт launch из localization:=; явный дубль был бы мёртв или опасен."""
    assert "Mem/IncrementalMemory" not in load_module().RTABMAP_PARAMS


def test_map_is_not_reanchored_to_graph_end():
    """RGBD/OptimizeFromGraphEnd=false (по умолчанию rtabmap): карта не перецепляется к последнему узлу."""
    assert load_module().RTABMAP_PARAMS.get("RGBD/OptimizeFromGraphEnd", "false") == "false"
