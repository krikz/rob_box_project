"""#5003: BackUp не абортит по устаревшей копии costmap_raw.

HIL 22.11 (``/tmp/hil_5003_1122/nav2.log``): четыре ``Running backup`` кончились
``Collision Ahead`` через 1–20 мс, смещение 0.000 м. Пятый попал на свежую
копию и отъехал (``backup completed successfully``, 6.602 с). Копия публикуется
на 2 Гц, так что тик сразу после clear читает стену, которую clear уже стёр.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from rob_box_teleop.backup_costmap_tick import (
    CHECK,
    FAIL,
    MAX_STALE_COSTMAP_SKIPS,
    SKIP,
    CostmapTickGate,
    backup_tick,
)

REPO = Path(__file__).resolve().parents[3]
PATCH = REPO / "docker" / "main" / "nav2" / "behavior_costmap_freshness.patch"
DOCKERFILE = REPO / "docker" / "main" / "nav2" / "Dockerfile"

# HIL 22.11: зазор Running backup → Collision Ahead, секунды.
HIL_ABORT_GAPS_S = (0.020, 0.001, 0.001, 0.002)
# Период публикации costmap_raw (publish_frequency 2.0).
PUBLISH_PERIOD_S = 0.5


def test_stale_copy_within_the_publish_period_does_not_abort():
    """Команда назад есть, копия старше допуска, в ней стена — это не отказ.

    1–20 мс из лога короче периода публикации 0.5 с, так что копия ещё старая.
    """
    for gap in HIL_ABORT_GAPS_S:
        assert gap < PUBLISH_PERIOD_S
        # Каждый эпизод — свой BackUp: счётчик пропусков начинается заново.
        gate = CostmapTickGate()
        assert backup_tick(gate.action(copy_fresh=False), collision_ahead=True) == "running"


def test_stale_skips_stop_at_the_measured_limit():
    """Три пропуска покрывают 0.5 с при 10 Гц. Четвёртый тик без копии — отказ."""
    assert MAX_STALE_COSTMAP_SKIPS == 3
    gate = CostmapTickGate()
    outcomes = [backup_tick(gate.action(copy_fresh=False), collision_ahead=False) for _ in range(4)]
    assert outcomes == ["running", "running", "running", "failed"]


def test_fresh_copy_still_aborts_on_a_real_wall():
    """Свежая копия со стеной позади — прежний Collision Ahead, не пропуск."""
    gate = CostmapTickGate()
    assert gate.action(copy_fresh=True) == CHECK
    assert backup_tick(CHECK, collision_ahead=True) == "collision_ahead"
    assert backup_tick(CHECK, collision_ahead=False) == "running"


def test_fresh_copy_resets_the_skip_count():
    gate = CostmapTickGate()
    gate.action(copy_fresh=False)
    gate.action(copy_fresh=False)
    assert gate.action(copy_fresh=True) == CHECK
    assert gate.stale_skips == 0
    assert gate.action(copy_fresh=False) == SKIP


def test_fail_closed_does_not_drive_on_an_empty_stale_copy():
    assert backup_tick(FAIL, collision_ahead=False) == "failed"


def _added_by_file(text: str) -> dict[str, str]:
    """Добавленные строки патча, разобранные по файлу (``diff -ruN a/... b/...``)."""
    files: dict[str, list[str]] = {}
    current: str | None = None
    for line in text.splitlines():
        if line.startswith("diff -ruN "):
            current = line.split()[-1]
            if current.startswith("b/"):
                current = current[2:]
            files.setdefault(current, [])
            continue
        if current is None or line.startswith("+++") or line.startswith("---") or line.startswith("@@"):
            continue
        if line.startswith("+"):
            files[current].append(line[1:])
    return {path: "\n".join(lines) for path, lines in files.items()}


def test_patch_puts_the_decision_in_the_server_both_plugins_call():
    """Решение в behavior_server. Тело шаблона — в заголовке, чтобы TU BackUp его эмитил.

    ``plugins/drive_on_heading.cpp`` инстанцирует только
    ``DriveOnHeading<DriveOnHeading>``. Тело там оставляет
    ``DriveOnHeading<BackUp>::costmapTickStatus`` неопределённым, и dlopen падает.
    Оба плагина зовут behavior_server через ``costmapTickStatusFor``.
    """
    by_file = _added_by_file(PATCH.read_text(encoding="utf-8"))
    server_hpp = by_file["nav2_behaviors/include/nav2_behaviors/behavior_server.hpp"]
    server_cpp = by_file["nav2_behaviors/src/behavior_server.cpp"]
    loop = by_file["nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp"]
    heading = by_file["nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp"]
    backup_cpp = by_file["nav2_behaviors/plugins/back_up.cpp"]
    # Решение — в сервере, не в плагине.
    assert "bool costmapCopyFresh() const;" in server_hpp
    assert "costmapTickStatusFor" in server_hpp
    assert "costmapCopyFresh" in server_cpp
    assert "costmapTickStatusFor" in server_cpp
    assert "Costmap copy stale" in loop
    assert "dynamic_cast<BehaviorServer" in server_cpp
    # Каст живёт в сервере. Плагин только спрашивает costmapTickStatusFor.
    for path, added in by_file.items():
        if path == "nav2_behaviors/src/behavior_server.cpp":
            continue
        assert "dynamic_cast<" not in added, path
    # В заголовке плагина — объявление override и слабое тело шаблона, не решение.
    assert "Status costmapTickStatus() const override;" in heading
    assert "template<typename ActionT>" in heading
    assert "Status DriveOnHeading<ActionT>::costmapTickStatus() const" in heading
    assert "costmapCopyFresh" not in heading
    assert "Costmap copy stale" not in heading
    # Оба плагина ходят в сервер.
    assert "costmapTickStatusFor" in heading
    assert "costmapTickStatusFor" in backup_cpp
    assert "nav2_behaviors::BackUp" in by_file["nav2_behaviors/include/nav2_behaviors/plugins/back_up.hpp"]
    assert "nav2_behaviors::DriveOnHeading" in heading


def test_image_ships_the_backup_library_not_only_drive_on_heading():
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert "libnav2_back_up_behavior.so" in df
    assert df.index("behavior_costmap_freshness.patch") < df.index("libnav2_back_up_behavior.so")
    assert df.index("libnav2_back_up_behavior.so") < df.index("rm -rf /tmp/nav2-fix")


def test_patch_applies_and_backup_sees_the_server_hook(tmp_path: Path):
    """Патч ложится на pinned-исходник, и BackUp компилирует вызов сервера (нужна сеть)."""
    src = tmp_path / "navigation2"
    base = "https://raw.githubusercontent.com/ros-navigation/navigation2/a097086719c88f781aa59788eca29ac6ca5e56db"
    files = (
        "nav2_costmap_2d/include/nav2_costmap_2d/costmap_subscriber.hpp",
        "nav2_costmap_2d/src/costmap_subscriber.cpp",
        "nav2_behaviors/include/nav2_behaviors/behavior_server.hpp",
        "nav2_behaviors/src/behavior_server.cpp",
        "nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp",
        "nav2_behaviors/include/nav2_behaviors/plugins/back_up.hpp",
        "nav2_behaviors/plugins/back_up.cpp",
        "nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp",
        "nav2_behaviors/plugins/drive_on_heading.cpp",
    )
    for rel in files:
        dest = src / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        try:
            with __import__("urllib.request", fromlist=["urlopen"]).urlopen(f"{base}/{rel}", timeout=30) as resp:
                dest.write_bytes(resp.read())
        except Exception as exc:  # noqa: BLE001 — честный отказ, если апстрим недоступен
            pytest.fail(f"could not fetch {rel}: {exc}")
    result = subprocess.run(
        ["patch", "-p1", "--forward", "--batch"],
        cwd=src,
        input=PATCH.read_text(encoding="utf-8"),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}{result.stderr}"
    backup = (src / "nav2_behaviors/plugins/back_up.cpp").read_text(encoding="utf-8")
    # Тело шаблона — в заголовке. В .cpp его нет: тот файл инстанцирует только
    # DriveOnHeading<DriveOnHeading>, и тело там снова оставит BackUp без символа.
    heading_cpp = (src / "nav2_behaviors/plugins/drive_on_heading.cpp").read_text(encoding="utf-8")
    assert "costmapTickStatusFor" not in heading_cpp
    header = (src / "nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp").read_text(encoding="utf-8")
    assert "costmapTickStatus() const override;" in header
    assert "template<typename ActionT>" in header
    assert "Status DriveOnHeading<ActionT>::costmapTickStatus() const" in header
    assert "costmapTickStatusFor" in header
    assert '#include "nav2_behaviors/behavior_server.hpp"' in header
    assert "BackUp::costmapTickStatus" in backup
    assert "costmapTickStatusFor" in backup
