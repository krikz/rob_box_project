"""#5151: дроп скана costmap сидит на таймауте MessageFilter, мёртвый ключ его не лечит.

Дефект не в тексте лога. Текст «earlier than all the data in the transform
cache» в humble — общий ярлык любого исключения future.get()
(message_filter.hpp:549-554 и :105-107). Ловится другое:

  1. лаг дропа (log_stamp − scan_stamp) равен transform_tolerance — это
     buffer_timeout фильтра (obstacle_layer.cpp:230-234), не возраст кэша;
  2. tf_buffer_duration в боевом yaml узел пина a0970867 не объявляет и в
     конструктор буфера не передаёт (costmap_2d_ros.cpp:98,150,181);
  3. живой минимум 1.00027 с при значении 1.0 попадает в окно таймаута,
     а не в «скан старше кэша» (скан отстаёт на миллисекунды).

Исходник пина качается из того же коммита, что собирает образ
(docker/main/nav2/Dockerfile). Сети нет — тест честно падает, не пропускается.
"""

from __future__ import annotations

import importlib.util
import urllib.request
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
NAV2_PARAMS = REPO / "docker/main/config/nav2/nav2_params.yaml"
DOCKERFILE = REPO / "docker/main/nav2/Dockerfile"
MODULE = REPO / "scripts/nav2/scan_drop_lag.py"
PINNED_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
NAV2_RAW = f"https://raw.githubusercontent.com/ros-navigation/navigation2/{PINNED_COMMIT}"
GEOMETRY2_RAW = "https://raw.githubusercontent.com/ros2/geometry2/humble"


def _load():
    spec = importlib.util.spec_from_file_location("scan_drop_lag", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fetch(url: str) -> str:
    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            return resp.read().decode("utf-8")
    except Exception as exc:  # noqa: BLE001 — честный отказ, если апстрим недоступен
        pytest.fail(f"could not fetch pinned source {url}: {exc}")


def _costmaps() -> dict:
    doc = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
    return {name: doc[name][name]["ros__parameters"] for name in ("local_costmap", "global_costmap")}


def test_measured_drop_lag_sits_on_filter_timeout_not_on_scan_age():
    """Живой min 1.00027 с — это таймаут фильтра при 1.0, не «скан старше кэша»."""
    lag = _load()
    tolerance = _costmaps()["local_costmap"]["transform_tolerance"]
    assert tolerance == 1.0
    assert _costmaps()["global_costmap"]["transform_tolerance"] == tolerance
    assert lag.filter_timeout_s(tolerance) == tolerance
    assert lag.drop_lag_is_filter_timeout(lag.MEASURED_DROP_LAG_MIN_S, tolerance)
    # Скан, который просто старше кэша, дал бы лаг около cache_time (10 с), не 1.0.
    assert not lag.drop_lag_is_filter_timeout(lag.TF2_DEFAULT_CACHE_S, tolerance)
    # Таймер не стреляет раньше таймаута: лаг 0.83 с из старого комментария — не этот дефект.
    assert not lag.drop_lag_is_filter_timeout(0.83, tolerance)


def test_dead_yaml_key_does_not_change_the_default_cache():
    """30.0 в yaml не равен кэшу: пин ключ не объявляет, буфер остаётся на 10 с."""
    lag = _load()
    for params in _costmaps().values():
        assert params["tf_buffer_duration"] == 30.0
        assert lag.dead_yaml_key_does_not_set_cache(params["tf_buffer_duration"], lag.TF2_DEFAULT_CACHE_S)
    assert not lag.dead_yaml_key_does_not_set_cache(30.0, 30.0)


def test_pinned_costmap_declares_tolerance_and_builds_buffer_without_cache_time():
    """Пин образа: параметр читается, мёртвый ключ не объявлен, буфер без cache_time."""
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert PINNED_COMMIT in df
    src = _fetch(f"{NAV2_RAW}/nav2_costmap_2d/src/costmap_2d_ros.cpp")
    assert src.count('declare_parameter("transform_tolerance"') == 2
    assert "tf_buffer_duration" not in src
    built = src.index("tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());")
    # Конструктор без второго аргумента: cache_time остаётся дефолтом tf2.
    assert "cache_time" not in src[built : built + 80]


def test_pinned_obstacle_layer_passes_tolerance_as_filter_timeout():
    """7-й аргумент MessageFilter — transform_tolerance, не отдельный таймаут."""
    src = _fetch(f"{NAV2_RAW}/nav2_costmap_2d/plugins/obstacle_layer.cpp")
    call = src.index("std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(")
    window = src[call : call + 400]
    assert "tf2::durationFromSec(transform_tolerance)" in window
    assert src.index("setTolerance(rclcpp::Duration::from_seconds(0.05))") > call


def test_humble_maps_every_wait_exception_to_the_same_reason_string():
    """Текст причины один на все исключения: по нему возраст кэша не отличить."""
    header = _fetch(f"{GEOMETRY2_RAW}/tf2_ros/include/tf2_ros/message_filter.hpp")
    catch = header.index("future.get();")
    mapped = header[catch : catch + 180]
    assert "catch (...)" in mapped
    assert "filter_failure_reasons::OutTheBack" in mapped
    reason = header[
        header.index("case filter_failure_reasons::OutTheBack:") : header.index(
            "case filter_failure_reasons::EmptyFrameID:"
        )
    ]
    assert reason.count("earlier than all the data in the transform cache") == 1
    core = _fetch(f"{GEOMETRY2_RAW}/tf2/include/tf2/buffer_core.hpp")
    assert "BUFFER_CORE_DEFAULT_CACHE_TIME = std::chrono::seconds(10)" in core
