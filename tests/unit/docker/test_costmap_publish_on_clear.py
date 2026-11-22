"""#5101: очистка карты публикует её сразу, до ответа сервиса.

BackUp отменялся «Collision Ahead» через 1–20 мс после «Running backup»:
behavior_server (10 Гц) читает local_costmap/costmap_raw, а очистка
clearEntirely сбрасывает карту в память и ответ отдаёт сразу. costmap_raw
публикуется только в цикле 5 Гц, так что проверка столкновения читала копию
до 0.2 с старую — ещё со стеной, в которую робот упёрся. Патч публикует
сброшенную карту в обработчике очистки, до возврата ответа.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
NAV2_DIR = REPO / "docker" / "main" / "nav2"
PATCH = NAV2_DIR / "costmap_publish_on_clear.patch"
DOCKERFILE = NAV2_DIR / "Dockerfile"
PINNED_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
UPSTREAM = f"https://raw.githubusercontent.com/ros-navigation/navigation2/{PINNED_COMMIT}/nav2_costmap_2d"


def _fetch(url: str) -> bytes:
    import urllib.request

    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            return resp.read()
    except Exception as exc:  # noqa: BLE001 — честный отказ, если апстрим недоступен
        pytest.fail(f"could not fetch pinned Nav2 source: {exc}")


def test_dockerfile_builds_costmap_and_applies_clear_patch():
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert "costmap_publish_on_clear.patch" in df
    assert PINNED_COMMIT in df
    # Патч применяется до сборки и до подмены либы, которую везёт образ.
    assert df.index("costmap_publish_on_clear.patch") < df.index("libnav2_costmap_2d_core.so")
    assert "nav2_costmap_2d" in df
    select = df[df.index("--packages-select") : df.index("--cmake-args")]
    assert "nav2_costmap_2d" in select


def test_patch_publishes_before_the_service_returns():
    text = PATCH.read_text(encoding="utf-8")
    assert "void publishCostmapNow();" in text
    assert "costmap_.publishCostmapNow();" in text
    # Публикация — в обработчике полной очистки, после сброса и до конца функции.
    cb = text.index("clearEntirely();")
    pub = text.index("costmap_.publishCostmapNow();")
    assert cb < pub


def test_patch_applies_on_pinned_costmap(tmp_path: Path):
    """Патч ложится на pinned-исходник nav2_costmap_2d (нужна сеть)."""
    src = tmp_path / "navigation2"
    files = {
        "include/nav2_costmap_2d/costmap_2d_ros.hpp": f"{UPSTREAM}/include/nav2_costmap_2d/costmap_2d_ros.hpp",
        "src/costmap_2d_ros.cpp": f"{UPSTREAM}/src/costmap_2d_ros.cpp",
        "src/clear_costmap_service.cpp": f"{UPSTREAM}/src/clear_costmap_service.cpp",
    }
    for rel, url in files.items():
        dest = src / "nav2_costmap_2d" / rel
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
    cleared = (src / "nav2_costmap_2d/src/clear_costmap_service.cpp").read_text(encoding="utf-8")
    assert "costmap_.publishCostmapNow();" in cleared
    ros = (src / "nav2_costmap_2d/src/costmap_2d_ros.cpp").read_text(encoding="utf-8")
    assert "Costmap2DROS::publishCostmapNow()" in ros
    assert "costmap_publisher_->publishCostmap();" in ros
