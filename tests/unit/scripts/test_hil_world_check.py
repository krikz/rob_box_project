"""Проверка мира Gazebo до первой цели Depot (scripts/hil/checks/world_check.py).

PASS/FAIL строит код по raw (args, mounts, SDF). Без ROS и стенда.
"""

import importlib.util
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CHECKS = ROOT / "scripts" / "hil" / "checks"
_spec = importlib.util.spec_from_file_location("hil_checks_world_check", CHECKS / "world_check.py")
wc = importlib.util.module_from_spec(_spec)
sys.modules["hil_checks_world_check"] = wc
_spec.loader.exec_module(wc)

DEPOT_ARGS = "ros2 launch rob_box_sim sim.launch.py world:=/sim_video/world.sdf x:=0.0 y:=0.0 yaw:=0.0"
DEPOT_MOUNTS = ["/mkdepot.py", "/fuel", "/sim_video", "/tmp/speed"]
DEPOT_SDF = """<?xml version='1.0'?>
<sdf version='1.8'>
  <world name='polygon'>
    <include>
      <name>depot</name>
      <uri>file:///fuel/Depot_vid</uri>
    </include>
  </world>
</sdf>
"""
POLYGON_ARGS = "exec /hil_run.sh world:=/sim_worlds/polygon.sdf x:=0 y:=0 yaw:=0"
POLYGON_SDF = """<?xml version='1.0'?>
<sdf version='1.8'>
  <world name='polygon'>
    <model name='polygon_walls'/>
  </world>
</sdf>
"""
# Ложный PASS на подстроку world.sdf / комментарий Depot_vid — дыры ревью #4877.
WORLD_SDF_OLD_ARGS = (
    "ros2 launch rob_box_sim sim.launch.py world:=/sim_video/world.sdf.old x:=0.0 y:=0.0 yaw:=0.0"
)
DEPOT_VID_COMMENT_ONLY_SDF = """<?xml version='1.0'?>
<sdf version='1.8'>
  <world name='polygon'>
    <!-- Depot_vid -->
    <model name='polygon_walls'/>
  </world>
</sdf>
"""


def test_depot_fail_args_without_sim_video_world():
    """Красный: args указывают polygon.sdf, не /sim_video/world.sdf."""
    status, detail = wc.check_world("depot", POLYGON_ARGS, DEPOT_MOUNTS, DEPOT_SDF)
    assert status == wc.FAIL
    assert wc.WORLD_MISMATCH in detail
    assert "args missing" in detail
    assert wc.DEPOT_WORLD_ARG in detail


def test_depot_fail_mounts_without_sim_video():
    """Красный: тома без /sim_video (мир polygon с чужим SDF не спасёт)."""
    status, detail = wc.check_world("depot", DEPOT_ARGS, ["/sim_worlds", "/tmp"], DEPOT_SDF)
    assert status == wc.FAIL
    assert wc.WORLD_MISMATCH in detail
    assert "mounts missing" in detail
    assert "/sim_video" in detail


def test_depot_fail_sdf_without_depot_vid():
    """Красный: SDF без Depot_vid (имя polygon.sdf само по себе ни PASS, ни FAIL)."""
    status, detail = wc.check_world("depot", DEPOT_ARGS, DEPOT_MOUNTS, POLYGON_SDF)
    assert status == wc.FAIL
    assert wc.WORLD_MISMATCH in detail
    assert "sdf missing" in detail
    assert "Depot_vid" in detail


def test_depot_pass_three_facts():
    """Зелёный: args world.sdf, mount /sim_video, SDF с Depot_vid."""
    status, detail = wc.check_world("depot", DEPOT_ARGS, DEPOT_MOUNTS, DEPOT_SDF)
    assert status == wc.PASS, detail
    assert "Depot_vid" in detail


def test_polygon_sdf_name_alone_is_neither_pass_nor_fail():
    """Имя мира polygon в SDF не решает: без трёх фактов — FAIL, с ними — PASS."""
    named = POLYGON_SDF  # <world name='polygon'>
    st_bad, _ = wc.check_world("depot", POLYGON_ARGS, ["/sim_worlds"], named)
    assert st_bad == wc.FAIL
    st_ok, _ = wc.check_world(
        "depot",
        DEPOT_ARGS,
        DEPOT_MOUNTS,
        named.replace("</world>", "  <uri>file:///fuel/Depot_vid</uri>\n  </world>"),
    )
    assert st_ok == wc.PASS


def test_cli_exit_codes(tmp_path):
    sdf = tmp_path / "world.sdf"
    sdf.write_text(DEPOT_SDF, encoding="utf-8")
    ok = subprocess.run(
        [
            sys.executable,
            "-X",
            "utf8",
            str(CHECKS / "world_check.py"),
            "depot",
            "--args",
            DEPOT_ARGS,
            "--mount",
            "/sim_video",
            "--sdf-file",
            str(sdf),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert ok.returncode == 0, ok.stdout + ok.stderr
    assert ok.stdout.startswith("PASS:")

    bad = subprocess.run(
        [
            sys.executable,
            "-X",
            "utf8",
            str(CHECKS / "world_check.py"),
            "depot",
            "--args",
            POLYGON_ARGS,
            "--mount",
            "/sim_worlds",
            "--sdf",
            POLYGON_SDF,
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert bad.returncode == 1
    assert "WORLD_MISMATCH" in bad.stdout


def test_depot_fail_world_sdf_old_substring():
    """Красный: world:=/sim_video/world.sdf.old — не точный токен world.sdf."""
    status, detail = wc.check_world("depot", WORLD_SDF_OLD_ARGS, DEPOT_MOUNTS, DEPOT_SDF)
    assert status == wc.FAIL
    assert wc.WORLD_MISMATCH in detail
    assert "args missing" in detail
    assert wc.DEPOT_WORLD_ARG in detail


def test_depot_fail_depot_vid_only_in_xml_comment():
    """Красный: Depot_vid только в <!-- … --> — без <uri>…Depot_vid…</uri>."""
    status, detail = wc.check_world("depot", DEPOT_ARGS, DEPOT_MOUNTS, DEPOT_VID_COMMENT_ONLY_SDF)
    assert status == wc.FAIL
    assert wc.WORLD_MISMATCH in detail
    assert "sdf missing" in detail
    assert "Depot_vid" in detail


def test_depot_pass_exact_token_uri_outside_comment_and_mount():
    """Зелёный: точный world.sdf + mount + uri Depot_vid вне комментария."""
    status, detail = wc.check_world("depot", DEPOT_ARGS, DEPOT_MOUNTS, DEPOT_SDF)
    assert status == wc.PASS, detail
    assert "Depot_vid" in detail


# Копия compose.hil.yaml после patch_compose.py — штатный Depot.
DEPOT_COMPOSE = """
    volumes:
      - ./hil_run.sh:/hil_run.sh:ro
      - /home/ros2/rob_box_hilvideo/mkdepot.py:/mkdepot.py:ro
      - /home/ros2/rob_box_hilvideo/fuel:/fuel:ro
      - /home/ros2/rob_box_hilvideo/out:/sim_video
    command:
      - /ros_scripts/ros_with_namespace.sh
      - bash
      - -c
      - python3 /mkdepot.py && exec /hil_run.sh world:=/sim_video/world.sdf x:=0.0 y:=0.0 yaw:=0.0
"""
# Обычный Compose: мир из SIM_WORLD, тома /sim_video нет. Depot_vid в комментарии.
POLYGON_COMPOSE = """
    # Depot_vid в комментарии — не маркер Depot-копии.
    volumes:
      - ./hil_run.sh:/hil_run.sh:ro
    command:
      - /ros_scripts/ros_with_namespace.sh
      - /hil_run.sh
      - world:=/sim_worlds/${SIM_WORLD:-polygon}.sdf
"""
WORLD_SDF_OLD_COMPOSE = DEPOT_COMPOSE.replace(
    "world:=/sim_video/world.sdf", "world:=/sim_video/world.sdf.old", 1
)


def test_depot_scenario_sets_expect_world_without_env():
    """Штатный Depot без env сам получает HIL_EXPECT_WORLD=depot."""
    assert wc.expect_world_for_scenario("depot") == "depot"
    assert wc.scenario_expects_depot("depot") is True
    # HIL_EXPECT_WORLD не передан: значение печатает код.
    got = subprocess.run(
        [sys.executable, "-X", "utf8", str(CHECKS / "world_check.py"), "--scenario", "depot"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert got.returncode == 0, got.stderr
    assert got.stdout.strip() == "depot"


def test_polygon_scenario_does_not_set_expect_world():
    """Обычный Polygon без env гейт не включает: код не печатает depot."""
    assert wc.expect_world_for_scenario("polygon") == ""
    assert wc.expect_world_for_scenario("building") == ""
    assert wc.expect_world_for_scenario("") == ""
    assert wc.gate_required("", POLYGON_ARGS) is False
    got = subprocess.run(
        [sys.executable, "-X", "utf8", str(CHECKS / "world_check.py"), "--scenario", "polygon"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert got.returncode == 0, got.stderr
    assert got.stdout.strip() == ""


def test_gate_required_only_for_depot_expect_or_exact_world_path():
    """Гейт: depot-env или точный /sim_video/world.sdf. Polygon и world.sdf.old — skip."""
    assert wc.gate_required("depot", POLYGON_ARGS) is True
    assert wc.gate_required("depot", "") is True
    assert wc.gate_required("", DEPOT_ARGS) is True
    assert wc.gate_required("", POLYGON_ARGS) is False
    assert wc.gate_required("", WORLD_SDF_OLD_ARGS) is False
    assert wc.gate_required("polygon", POLYGON_ARGS) is False


def test_compose_world_sdf_old_is_not_depot():
    """world.sdf.old в копии Compose — не штатный Depot."""
    assert wc.compose_is_depot(DEPOT_COMPOSE) is True
    assert wc.compose_is_depot(WORLD_SDF_OLD_COMPOSE) is False
    assert wc.compose_is_depot(POLYGON_COMPOSE) is False


def test_compose_depot_vid_only_in_comment_is_not_depot():
    """Depot_vid только в комментарии Compose — не Depot."""
    assert "Depot_vid" in POLYGON_COMPOSE
    assert wc.compose_is_depot(POLYGON_COMPOSE) is False
    assert wc.gate_required("", "world:=/sim_worlds/polygon.sdf") is False
