"""NavFn must not hand the controller a path through a lidar-seen wall (#4694).

Bag depot4694_20261117T1235Z: M1/M4/R3 plans ended at the goal (160/160) with
the tail in cost 255 beyond a wall the lidar already marked (253/254).
Humble NavfnPlanner ``allow_unknown: false`` maps unknown to lethal and will
not reach a goal that still sits in unscanned space, even when the path to the
known frontier is free. That flag stays true. The refusal is in the planner:
a path that crosses lethal, or unknown after a seen wall (cost >= 253), is not
returned. planner_server then ABORTs ComputePathToPose and does not publish
/plan (Humble validatePath + publishPlan order).

This module does not build NavFn and does not drive the robot.
"""

from __future__ import annotations

import subprocess
import urllib.request
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
NAV2_DIR = REPO / "docker" / "main" / "nav2"
PATCH = NAV2_DIR / "navfn_refuse_unknown_tail.patch"
DOCKERFILE = NAV2_DIR / "Dockerfile"
PARAMS = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"
PINNED_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
UPSTREAM = (
    "https://raw.githubusercontent.com/ros-navigation/navigation2/"
    f"{PINNED_COMMIT}/nav2_navfn_planner/"
)

# nav2_costmap_2d cost_values.hpp (Humble 1.1.20), same bytes NavFn reads.
NO_INFORMATION = 255
LETHAL_OBSTACLE = 254
INSCRIBED = 253  # COST_OBS_ROS: lidar-inflated wall, not yet lethal at the centre
FREE = 0

REFUSAL = "refusing path whose tail is unknown or lethal beyond a seen wall"


def tail_blocked(costs: list[int]) -> bool:
    """Same rule as NavfnPlanner::pathTailBlocked.

    Lethal anywhere is a refusal. Unknown is a refusal only after a seen wall
    (cost >= 253 and not NO_INFORMATION). Unknown before that wall is still
    explorable, so a goal in unscanned space can start.
    """
    seen_wall = False
    unknown_after_wall = False
    for cost in costs:
        if cost == LETHAL_OBSTACLE:
            return True
        if cost >= INSCRIBED and cost != NO_INFORMATION:
            seen_wall = True
        elif seen_wall and cost == NO_INFORMATION:
            unknown_after_wall = True
    return unknown_after_wall


def planner_outcome(costs: list[int]) -> str:
    """What ComputePathToPose does with a non-empty NavFn path of these costs."""
    if tail_blocked(costs):
        return "ABORT"
    return "SUCCESS"


def test_depot_m1_tail_through_seen_wall_is_refused():
    """Bag M1 t=151.6: free approach, then 253, then cost 255 toward the goal."""
    costs = [FREE, FREE, 152, INSCRIBED, NO_INFORMATION, NO_INFORMATION]
    assert tail_blocked(costs)
    assert planner_outcome(costs) == "ABORT"


def test_lethal_cell_on_the_path_is_refused():
    assert tail_blocked([FREE, 100, LETHAL_OBSTACLE, FREE])
    assert planner_outcome([FREE, LETHAL_OBSTACLE]) == "ABORT"


def test_unknown_before_any_seen_wall_still_starts():
    """Goal in unscanned space, path to the known frontier is free — not a refusal."""
    costs = [FREE, 20, 40, NO_INFORMATION, NO_INFORMATION]
    assert not tail_blocked(costs)
    assert planner_outcome(costs) == "SUCCESS"


def test_inscribed_without_unknown_tail_is_not_a_refusal():
    """Inflation beside a wall the path only grazes is not a through-wall plan."""
    assert not tail_blocked([FREE, 180, INSCRIBED, 180, FREE])


def test_unknown_then_wall_then_unknown_is_refused():
    assert tail_blocked([NO_INFORMATION, FREE, INSCRIBED, NO_INFORMATION])


def test_gridbased_requires_the_requested_endpoint():
    planner = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))["planner_server"]["ros__parameters"]
    assert planner["planner_plugins"] == ["GridBased"]
    assert planner["GridBased"]["tolerance"] == 0.0


def test_allow_unknown_stays_true_and_costmap_still_tracks_unknown():
    doc = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    grid = doc["planner_server"]["ros__parameters"]["GridBased"]
    assert grid["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    assert grid["allow_unknown"] is True
    global_cm = doc["global_costmap"]["global_costmap"]["ros__parameters"]
    assert global_cm["track_unknown_space"] is True


def test_patch_refuses_tail_and_does_not_flip_allow_unknown():
    text = PATCH.read_text(encoding="utf-8")
    assert REFUSAL in text
    assert "pathTailBlocked" in text
    assert "nav2_navfn_planner/src/navfn_planner.cpp" in text
    # Snapshot hunk may cite setCostmap(..., allow_unknown_) as context only —
    # must not assign or flip the parameter (#4694 stays allow_unknown: true).
    allow_lines = [ln for ln in text.splitlines() if "allow_unknown" in ln]
    assert allow_lines, "expected setCostmap context mentioning allow_unknown_"
    assert all("setCostmap" in ln for ln in allow_lines)
    assert "allow_unknown_ =" not in text
    assert "kSeenWall = 253" in text
    assert "LETHAL_OBSTACLE" in text
    # Bag recovery needs the first blocking cell in the throw text (#4682).
    assert ' + " world_x=" + std::to_string(' in text
    assert ' + " world_y=" + std::to_string(' in text
    assert ' + " mx=" + std::to_string(' in text
    assert ' + " my=" + std::to_string(' in text
    assert ' + " cost=" + std::to_string(static_cast<int>(' in text
    # #4771: branch + cost_navfn/cost_live + path samples so M1 proves one fix.
    assert "branch=lethal_on_path" in text or '"lethal_on_path"' in text
    assert "branch=unknown_after_seen_wall" in text or '"unknown_after_seen_wall"' in text
    assert "lethal_on_path" in text
    assert "unknown_after_seen_wall" in text
    assert "cost_navfn=" in text
    assert "cost_live=" in text
    assert "path_n=" in text
    assert "head_xy=" in text
    assert "navfn_cost_snap_" in text
    assert "std::memcpy" in text


def test_dockerfile_builds_pinned_navfn_with_both_patches():
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert "navfn_refuse_unknown_tail.patch" in df
    assert "controller_goal_tf_fail.patch" in df
    assert PINNED_COMMIT in df
    assert "libnav2_navfn_planner.so" in df
    assert "--packages-select nav2_controller nav2_navfn_planner" in df


def test_patch_applies_to_pinned_upstream_navfn(tmp_path: Path):
    """Fetch pinned NavFn sources and prove the patch applies (needs network)."""
    src_root = tmp_path / "navigation2"
    files = (
        "src/navfn_planner.cpp",
        "include/nav2_navfn_planner/navfn_planner.hpp",
    )
    for rel in files:
        target = src_root / "nav2_navfn_planner" / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        try:
            with urllib.request.urlopen(UPSTREAM + rel, timeout=30) as resp:
                body = resp.read()
        except Exception as exc:  # noqa: BLE001 — honest fail if upstream unreachable
            pytest.fail(f"could not fetch pinned Nav2 source {rel}: {exc}")
        target.write_bytes(body)

    before = (src_root / "nav2_navfn_planner" / "src" / "navfn_planner.cpp").read_text(encoding="utf-8")
    assert "pathTailBlocked" not in before
    assert "return path;" in before

    result = subprocess.run(
        ["patch", "-p1", "--forward", "--batch"],
        cwd=src_root,
        input=PATCH.read_text(encoding="utf-8"),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    after = (src_root / "nav2_navfn_planner" / "src" / "navfn_planner.cpp").read_text(encoding="utf-8")
    header = (src_root / "nav2_navfn_planner" / "include" / "nav2_navfn_planner" / "navfn_planner.hpp").read_text(
        encoding="utf-8"
    )
    assert "bool pathTailBlocked" in header
    assert REFUSAL in after
    assert "world_x=" in after
    assert "world_y=" in after
    assert "mx=" in after
    assert "my=" in after
    assert "cost=" in after
    assert "lethal_on_path" in after
    assert "unknown_after_seen_wall" in after
    assert "cost_navfn=" in after
    assert "cost_live=" in after
    assert "path_n=" in after
    assert "head_xy=" in after
    assert "navfn_cost_snap_" in after
    # Snapshot is taken under the same lock window as setCostmap, then unlocked.
    assert "std::memcpy(navfn_cost_snap_.get(), costmap_->getCharMap()" in after
    assert "lock.unlock();" in after
    # Refusal sits in createPlan, before the path is returned to planner_server.
    create = after.split("NavfnPlanner::isPlannerOutOfDate", 1)[0]
    assert "pathTailBlocked(path" in create
    assert create.rfind("pathTailBlocked(path") < create.rfind("return path;")
    assert "throw std::runtime_error(tail_refusal)" in create
