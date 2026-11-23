"""#5003: BackUp не решает столкновение по устаревшей копии costmap_raw.

HIL 22.11 (стенд dead_end): «Running backup», и через 1–20 мс behavior_server
сам отменяет DriveOnHeading строкой «Collision Ahead», колёса не крутятся.
costmap_raw публикуется на 2 Гц, очистка отвечает сразу, так что тик в десятки
миллисекунд после clear читает копию ещё со стеной. Патч: копия старше допуска
из конфига пропускает тик (до лимита), а не абортит; за лимитом проверка
закрывается отказом. Решение — C++ в behavior_server (ADR-0148).
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
NAV2_DIR = REPO / "docker" / "main" / "nav2"
PATCH = NAV2_DIR / "behavior_costmap_freshness.patch"
DOCKERFILE = NAV2_DIR / "Dockerfile"
PARAMS = NAV2_DIR.parent / "config" / "nav2" / "nav2_params.yaml"
PINNED_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
UPSTREAM = f"https://raw.githubusercontent.com/ros-navigation/navigation2/{PINNED_COMMIT}"

PINNED_FILES = {
    "nav2_costmap_2d/include/nav2_costmap_2d/costmap_subscriber.hpp": (
        f"{UPSTREAM}/nav2_costmap_2d/include/nav2_costmap_2d/costmap_subscriber.hpp"
    ),
    "nav2_costmap_2d/src/costmap_subscriber.cpp": f"{UPSTREAM}/nav2_costmap_2d/src/costmap_subscriber.cpp",
    "nav2_behaviors/include/nav2_behaviors/behavior_server.hpp": (
        f"{UPSTREAM}/nav2_behaviors/include/nav2_behaviors/behavior_server.hpp"
    ),
    "nav2_behaviors/src/behavior_server.cpp": f"{UPSTREAM}/nav2_behaviors/src/behavior_server.cpp",
    "nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp": (
        f"{UPSTREAM}/nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp"
    ),
}


def _added(text: str) -> str:
    return "\n".join(line[1:] for line in text.splitlines() if line.startswith("+") and not line.startswith("+++"))


def _fetch(url: str) -> bytes:
    import urllib.request

    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            return resp.read()
    except Exception as exc:  # noqa: BLE001 — честный отказ, если апстрим недоступен
        pytest.fail(f"could not fetch pinned Nav2 source: {exc}")


def test_stale_copy_skips_the_tick_instead_of_aborting():
    added = _added(PATCH.read_text(encoding="utf-8"))
    # Устаревшая копия — не «Collision Ahead»: тик пропускается, статус RUNNING.
    assert 'return "skip"' in added
    assert "skipping DriveOnHeading tick" in added
    assert "return Status::RUNNING;" in added
    # За лимитом пропусков проверка закрывается отказом, а не едет вслепую.
    assert 'return "fail"' in added
    assert "Exiting DriveOnHeading" in added
    assert "return Status::FAILED;" in added
    assert "stopRobot();" in added
    # Существующая проверка столкновения патчем не подменяется.
    assert "Collision Ahead" not in added


def test_tolerance_comes_from_config_and_is_declared_once():
    added = _added(PATCH.read_text(encoding="utf-8"))
    # Допуск читается из параметра, а не зашит числом в решение.
    assert 'kCostmapFreshnessParam[] = "costmap_freshness_tolerance"' in added
    assert "get_parameter(kCostmapFreshnessParam, tolerance);" in added
    assert added.count("declare_parameter(") == 1
    # Число допуска живёт в конфиге behavior_server, не в патче.
    assert "costmap_freshness_tolerance:" not in added
    params = PARAMS.read_text(encoding="utf-8")
    assert "costmap_freshness_tolerance: 0.6" in params


def test_dockerfile_builds_behaviors_and_ships_the_patched_libs():
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert "behavior_costmap_freshness.patch" in df
    assert PINNED_COMMIT in df
    # Патч применяется до сборки и до подмены либ, которые везёт образ.
    assert df.index("behavior_costmap_freshness.patch") < df.index("libbehavior_server_core.so")
    select = df[df.index("--packages-select") : df.index("--cmake-args")]
    assert "nav2_behaviors" in select
    # BackUp наследует DriveOnHeading, решение тика — в сервере: нужны обе либы.
    assert "libbehavior_server_core.so" in df
    assert "libnav2_drive_on_heading_behavior.so" in df
    assert df.index("libbehavior_server_core.so") < df.index("rm -rf /tmp/nav2-fix")


def test_patch_applies_on_pinned_behaviors(tmp_path: Path):
    """Патч ложится на pinned-исходник (нужна сеть)."""
    src = tmp_path / "navigation2"
    for rel, url in PINNED_FILES.items():
        dest = src / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(_fetch(url))
    result = subprocess.run(
        ["patch", "-p1", "--forward", "--batch"],
        cwd=src,
        input=PATCH.read_text(encoding="utf-8"),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}{result.stderr}"
    server = (src / "nav2_behaviors/src/behavior_server.cpp").read_text(encoding="utf-8")
    # Параметр допуска объявлен ровно один раз (issue #976).
    assert server.count("declare_parameter(kCostmapFreshnessParam") == 1
    assert "declare_parameter(kCostmapFreshnessParam, rclcpp::ParameterValue(0.6));" in server
    heading = (src / "nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp").read_text(encoding="utf-8")
    skip = heading.index("skipping DriveOnHeading tick")
    collision = heading.index("Collision Ahead - Exiting DriveOnHeading")
    assert skip < collision
