"""Патч #5003: упор в препятствие — свой отказ прогресса, без CPython (#5021).

Humble SimpleProgressChecker абортит FollowPath только после 30 с без
смещения 0.08 м. У стены DWB смещается вдоль стены, и NavigateRecovery
(ClearThenBackUp, #5000) не тикается. Патч решает в самом controller_server:
команда держится, центр стоит, по курсу клетка препятствия —
``Failed to make progress: stuck against obstacle``.

#5021: встроенный CPython в конструкторе ронял процесс (Py_Initialize до spin,
2 из 4 стартов образа dev-b3de9ff). Решение — код на C++, не вызов
rob_box_teleop.stuck_against из процесса контроллера (ADR-0148).
"""

from __future__ import annotations

import re
import subprocess
import urllib.request
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
NAV2_DIR = REPO / "docker" / "main" / "nav2"
PATCH = NAV2_DIR / "controller_stuck_against.patch"
DOCKERFILE = NAV2_DIR / "Dockerfile"
PINNED_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
UPSTREAM_CPP = (
    f"https://raw.githubusercontent.com/ros-navigation/navigation2/"
    f"{PINNED_COMMIT}/nav2_controller/src/controller_server.cpp"
)
UPSTREAM_HPP = (
    f"https://raw.githubusercontent.com/ros-navigation/navigation2/"
    f"{PINNED_COMMIT}/nav2_controller/include/nav2_controller/controller_server.hpp"
)
UPSTREAM_CMAKE = (
    f"https://raw.githubusercontent.com/ros-navigation/navigation2/" f"{PINNED_COMMIT}/nav2_controller/CMakeLists.txt"
)
UPSTREAM_MAIN = (
    f"https://raw.githubusercontent.com/ros-navigation/navigation2/" f"{PINNED_COMMIT}/nav2_controller/src/main.cpp"
)

THROW = 'throw nav2_core::PlannerException("Failed to make progress: stuck against obstacle")'
# #5021: ни одного вызова CPython в том, что линкуется в controller_server.
CPYTHON = re.compile(r"Py_Initialize|Py_IsInitialized|PyGILState_Ensure|PyRun_SimpleString")
# Вызов проверки из конструктора или on_configure. Тик computeAndPublishVelocity
# — единственное место, где проверка имеет право тикать.
CTOR_OR_CONFIGURE_CALL = re.compile(r"stuckAgainst\w*\s*\(")


def _added(text: str) -> str:
    return "\n".join(line[1:] for line in text.splitlines() if line.startswith("+") and not line.startswith("+++"))


def _fetch(url: str) -> bytes:
    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            return resp.read()
    except Exception as exc:  # noqa: BLE001 — honest fail if upstream unreachable
        pytest.fail(f"could not fetch pinned Nav2 source: {exc}")


def test_patch_names_the_stuck_failure_and_does_not_touch_thresholds():
    added = _added(PATCH.read_text(encoding="utf-8"))
    assert THROW in added
    assert "stuckAgainstObstacle" in added
    # Решение в патче, не в импорте Python-модуля.
    assert "rob_box_teleop.stuck_against" not in added
    assert "reset_controller_state" not in added
    # Чужие пороги патч не двигает: радиус и окно остаются в nav2_params.yaml.
    assert "required_movement_radius:" not in added
    assert "movement_time_allowance" not in added
    assert "xy_goal_tolerance" not in added
    assert "terminate_all" not in added


def test_stuck_patch_embeds_no_cpython_and_does_not_link_libpython():
    """#5021: патч и Dockerfile контроллера не зовут CPython и не линкуют libpython."""
    patch_text = PATCH.read_text(encoding="utf-8")
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert CPYTHON.search(patch_text) is None, CPYTHON.findall(patch_text)
    assert "Python.h" not in patch_text
    assert "find_package(Python3" not in patch_text
    assert "Python3::Python" not in patch_text
    assert "-lpython" not in patch_text
    # Dockerfile по-прежнему ставит python3-dev: его линкует узел WouldSpinOnStall,
    # не controller_server. Линковка контроллера с libpython — только в патче.
    assert "target_link_libraries(${library_name} Python3::Python)" not in df
    assert "libcontroller_server_core.so" in df


def test_stuck_check_is_not_called_from_ctor_or_on_configure(tmp_path: Path):
    """#5021: ctor и on_configure не зовут stuckAgainst*. Образ везёт библиотеку,
    и конструктор тикает в `ros2 run` до spin — оттуда и был segfault.
    """
    src_root = _patched_tree(tmp_path)
    cpp = (src_root / "nav2_controller" / "src" / "controller_server.cpp").read_text(encoding="utf-8")
    header = (src_root / "nav2_controller" / "include" / "nav2_controller" / "controller_server.hpp").read_text(
        encoding="utf-8"
    )
    assert CPYTHON.search(cpp) is None
    assert CPYTHON.search(header) is None
    ctor = cpp.index("ControllerServer::ControllerServer(")
    ctor_end = cpp.index("ControllerServer::~ControllerServer()")
    configure = cpp.index("ControllerServer::on_configure(")
    configure_end = cpp.index("ControllerServer::on_activate(")
    tick = cpp.index("void ControllerServer::computeAndPublishVelocity()")
    assert CTOR_OR_CONFIGURE_CALL.search(cpp[ctor:ctor_end]) is None
    assert CTOR_OR_CONFIGURE_CALL.search(cpp[configure:configure_end]) is None
    assert "stuckAgainstObstacle(pose)" in cpp[tick:]


def test_dockerfile_applies_stuck_patch_after_progress_abort_on_pinned_controller():
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert "controller_stuck_against.patch" in df
    assert PINNED_COMMIT in df
    assert df.index("controller_progress_abort.patch") < df.index("controller_stuck_against.patch")
    # Образ подменяет libcontroller_server_core.so — патч должен попасть в эту сборку.
    assert df.index("controller_stuck_against.patch") < df.index("libcontroller_server_core.so")
    patch_text = PATCH.read_text(encoding="utf-8")
    assert "stuckAgainstObstacle" in patch_text
    assert "stuckAgainstReset" in patch_text
    assert "kProgressRadiusM = 0.08" in patch_text
    assert "kWindowS = 2.0" in patch_text


def _patched_tree(tmp_path: Path) -> Path:
    src_root = tmp_path / "navigation2"
    cpp = src_root / "nav2_controller" / "src" / "controller_server.cpp"
    hpp = src_root / "nav2_controller" / "include" / "nav2_controller" / "controller_server.hpp"
    cmake = src_root / "nav2_controller" / "CMakeLists.txt"
    main = src_root / "nav2_controller" / "src" / "main.cpp"
    cpp.parent.mkdir(parents=True)
    hpp.parent.mkdir(parents=True)
    cpp.write_bytes(_fetch(UPSTREAM_CPP))
    hpp.write_bytes(_fetch(UPSTREAM_HPP))
    cmake.write_bytes(_fetch(UPSTREAM_CMAKE))
    main.write_bytes(_fetch(UPSTREAM_MAIN))
    order = (
        "controller_goal_tf_fail.patch",
        "controller_progress_abort.patch",
        "controller_stuck_against.patch",
    )
    for name in order:
        result = subprocess.run(
            ["patch", "-p1", "--forward", "--batch"],
            cwd=src_root,
            input=(NAV2_DIR / name).read_text(encoding="utf-8"),
            text=True,
            capture_output=True,
            check=False,
        )
        assert result.returncode == 0, f"{name}: {result.stdout}{result.stderr}"
    return src_root


def test_patch_applies_after_existing_controller_patches(tmp_path: Path):
    """Все три патча контроллера ложатся на pinned-исходник по порядку (нужна сеть)."""
    src_root = _patched_tree(tmp_path)
    cpp = src_root / "nav2_controller" / "src" / "controller_server.cpp"
    hpp = src_root / "nav2_controller" / "include" / "nav2_controller" / "controller_server.hpp"
    cmake = src_root / "nav2_controller" / "CMakeLists.txt"

    after = cpp.read_text(encoding="utf-8")
    assert THROW in after
    assert "stuckAgainstReset();" in after
    assert "have_last_cmd_ = true;" in after
    # Прежний отказ по радиусу патч не убрал.
    assert 'throw nav2_core::PlannerException("Failed to make progress");' in after
    # Проверка упора — после записи команды этого тика, не до compute:
    # ноль из catch (DWB NoLegalTrajectoriesException) иначе не виден.
    assert after.index("have_last_cmd_ = true;") < after.index("Failed to make progress: stuck against obstacle")
    header = hpp.read_text(encoding="utf-8")
    assert "bool stuckAgainstObstacle(" in header
    assert "bool have_last_cmd_{false};" in header
    assert "std::vector<StuckSample> stuck_samples_;" in header
    cmake_text = cmake.read_text(encoding="utf-8")
    assert "Python3::Python" not in cmake_text
    assert "find_package(Python3" not in cmake_text
    # #5021: конструктор не инициализирует проверку. Образ везёт только
    # libcontroller_server_core.so, а `ros2 run` запускает штатный бинарь.
    ctor = after.index('RCLCPP_INFO(get_logger(), "Creating controller server");')
    assert "stuckAgainstInit" not in after
    assert after.index('declare_parameter("controller_frequency", 20.0);', ctor) < after.index(
        "void ControllerServer::computeAndPublishVelocity()"
    )
    main = (src_root / "nav2_controller" / "src" / "main.cpp").read_text(encoding="utf-8")
    assert "stuckAgainst" not in main
