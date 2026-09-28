"""Tests for tools/architecture_runtime_graph.py (readable Mermaid runtime views).

The run 36339397150 diagram was unreadable because:

* 39 of 52 runtime nodes had no container ("physical ownership unresolved");
* /tf, /tf_static, /diagnostics and tf2 ``transform_listener_impl_*`` helper
  nodes dominated the "overview" (it picked the 8 busiest topics);
* nested Pi/container/class subgraphs forced every edge around the frames.

These tests pin the fixes: evidence-based placement, noise filtering,
1:1 topic folding and the per-container Markdown report.
"""

from __future__ import annotations

import importlib.util
import json
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
TOOL = REPO / "tools" / "architecture_runtime_graph.py"

spec = importlib.util.spec_from_file_location("architecture_runtime_graph", TOOL)
graph = importlib.util.module_from_spec(spec)
sys.modules["architecture_runtime_graph"] = graph
spec.loader.exec_module(graph)

# Node list captured by run 36339397150 (`ros2 node list`, Main Pi / nav2).
RUN_36339397150_NODES = """
/apriltag /audio_node /avatar_arbiter /avatar_supervisor /behavior_server /bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node /bt_navigator_navigate_to_pose_rclcpp_node
/camera/camera /ceiling_camera/usb_cam /command_node /context_aggregator /controller_manager
/controller_server /dialogue_node /diff_drive_controller /global_costmap/global_costmap
/health_monitor /joint_state_broadcaster /joystick_control_node /launch_ros_1
/led_matrix_compositor /led_matrix_driver /led_node /lifecycle_manager_navigation
/local_costmap/local_costmap /lslidar_driver_node /mcp_server /perception_bridge /planner_server
/quest_node /robot_state_publisher /rtabmap/icp_odometry /rtabmap/rtabmap
/rtabmap/transform_listener_impl_aaaad8a52090 /rtabmap/transform_listener_impl_aaaafae457a0
/smoother_server /sound_node /speaker_id_node /stt_node /telegram_node
/transform_listener_impl_aaaace948a40 /transform_listener_impl_aaaae6ecd230
/transform_listener_impl_aaaaf2418df0 /transform_listener_impl_aaab0362d7c0 /tts_node
/twist_mux /velocity_smoother /vision_face /vision_hailo /voice_animation_player /waypoint_follower
""".split()


def endpoint(node):
    ns, _, name = node.strip("/").rpartition("/")
    return {"node": name, "namespace": "/" + ns if ns else "/"}


def runtime(nodes, edges):
    return {
        "nodes": nodes,
        "topics": sorted(edges),
        "topic_info": {
            t: {"publishers": [endpoint(p) for p in pubs], "subscribers": [endpoint(s) for s in subs]}
            for t, (pubs, subs) in edges.items()
        },
        "captures": [{"pi": "Main Pi", "host": "10.1.1.20"}, {"pi": "Vision Pi", "host": "10.1.1.21"}],
    }


@pytest.fixture()
def mini_repo(tmp_path):
    """Two Pis, three containers, start scripts naming their nodes."""
    main_scripts = tmp_path / "docker/main/scripts"
    vision_scripts = tmp_path / "docker/vision/scripts"
    (main_scripts / "lidar").mkdir(parents=True)
    (main_scripts / "mux").mkdir(parents=True)
    (vision_scripts / "voice").mkdir(parents=True)
    (tmp_path / "docker/vision/config/voice").mkdir(parents=True)
    (main_scripts / "lidar/start_lidar.sh").write_text("exec ros2 run lidar_pkg lidar_driver_node\n")
    (main_scripts / "mux/start_mux.sh").write_text("exec ros2 run twist_mux twist_mux -r __node:=mux_main\n")
    (vision_scripts / "voice/start_voice.sh").write_text("exec ros2 launch /config/voice/voice.launch.py\n")
    (tmp_path / "docker/vision/config/voice/voice.launch.py").write_text(
        "Node(package='v', executable='stt', name='stt_node')\nNode(package='v', executable='tts_node')\n"
    )
    inventory = {
        "containers": [
            {
                "file": "docker/main/docker-compose.yaml",
                "services": [
                    {"name": "lidar", "container_name": "lidar", "command": ["/scripts/start_lidar.sh"]},
                    {"name": "mux", "container_name": "mux", "command": ["/scripts/start_mux.sh"]},
                ],
            },
            {
                "file": "docker/vision/docker-compose.yaml",
                "services": [{"name": "voice", "container_name": "voice", "entrypoint": ["/scripts/start_voice.sh"]}],
            },
        ],
        "classes": [],
    }
    return tmp_path, inventory


def test_placement_follows_start_scripts_and_launch_files(mini_repo):
    root, inventory = mini_repo
    placement = graph.resolve_placement(
        ["lidar_driver_node", "mux_main", "stt_node", "tts_node", "ghost_node"], inventory, root
    )
    where = {n: (m["service"]["host"], graph.service_label(m["service"]), m["method"]) for n, m in placement.items()}
    assert where["lidar_driver_node"] == ("Main Pi", "lidar", "start-script")
    assert where["mux_main"] == ("Main Pi", "mux", "start-script")
    assert where["stt_node"] == ("Vision Pi", "voice", "start-script")
    assert where["tts_node"] == ("Vision Pi", "voice", "start-script")
    assert "ghost_node" not in placement, "no evidence -> must stay unplaced, never guessed"
    assert "voice.launch.py" in placement["stt_node"]["evidence"]


def test_noise_is_hidden_and_reported(mini_repo):
    root, inventory = mini_repo
    nodes = ["/lidar_driver_node", "/stt_node", "/transform_listener_impl_aaaa", "/launch_ros_1"]
    rt = runtime(
        nodes,
        {
            "/scan": (["/lidar_driver_node"], ["/stt_node"]),
            "/tf": (["/lidar_driver_node"], ["/transform_listener_impl_aaaa"]),
            "/lidar/transition_event": (["/lidar_driver_node"], ["/stt_node"]),
        },
    )
    model = graph.Model(rt, inventory, root)
    assert model.nodes == ["lidar_driver_node", "stt_node"]
    assert set(model.helper_nodes) == {"transform_listener_impl_aaaa", "launch_ros_1"}
    assert set(model.topics) == {"/scan"}
    assert set(model.infra_topics) == {"/tf", "/lidar/transition_event"}
    full = graph.render_full(model)
    assert "transform_listener" not in full and "/tf" not in full


def test_overview_aggregates_per_container_pair(mini_repo):
    root, inventory = mini_repo
    rt = runtime(
        ["/lidar_driver_node", "/mux_main", "/stt_node", "/tts_node"],
        {
            "/scan": (["/lidar_driver_node"], ["/mux_main", "/stt_node"]),
            "/a": (["/stt_node"], ["/mux_main"]),
            "/b": (["/tts_node"], ["/mux_main"]),
            "/internal": (["/stt_node"], ["/tts_node"]),
        },
    )
    model = graph.Model(rt, inventory, root)
    edges = model.container_edges()
    assert edges[(("Vision Pi", "voice"), ("Main Pi", "mux"))] == ["/a", "/b"]
    assert (("Vision Pi", "voice"), ("Vision Pi", "voice")) not in edges, "intra-container traffic is not a map edge"
    overview = graph.render_overview(model)
    assert re.search(r'C_Vision_Pi_voice -->\|"/a<br/>/b"\| C_Main_Pi_mux', overview)
    assert "subgraph H_" not in overview, "hosts are colours, not frames (frames tangle cross-Pi edges)"


def test_one_to_one_topics_fold_into_edge_labels(mini_repo):
    root, inventory = mini_repo
    rt = runtime(
        ["/lidar_driver_node", "/mux_main", "/stt_node"],
        {
            "/one": (["/lidar_driver_node"], ["/mux_main"]),
            "/fan": (["/lidar_driver_node"], ["/mux_main", "/stt_node"]),
        },
    )
    full = graph.render_full(graph.Model(rt, inventory, root))
    assert 'N_lidar_driver_node -->|"/one"| N_mux_main' in full
    assert 'T_fan(["/fan"]):::topic' in full
    assert "T_one" not in full


def test_markdown_report_has_map_details_and_evidence(mini_repo, tmp_path):
    root, inventory = mini_repo
    rt = runtime(
        ["/lidar_driver_node", "/stt_node", "/ghost_node"],
        {"/scan": (["/lidar_driver_node"], ["/stt_node", "/ghost_node"])},
    )
    md = graph.render_markdown(graph.Model(rt, inventory, root))
    assert md.count("```mermaid") == 1 + 3  # map + lidar, voice, unplaced
    assert "### Node placement evidence" in md
    assert "| ghost_node | **unplaced** |" in md
    assert "<details><summary><b>🐳 lidar — Main Pi</b>" in md


def test_labels_escape_mermaid_metacharacters():
    assert graph.safe('a|b"<c>') == "a&#124;b&quot;&lt;c&gt;"
    assert graph.mid("N", "rtabmap/rtabmap") == "N_rtabmap_rtabmap"


def test_real_repo_places_every_node_from_run_36339397150(tmp_path):
    """Every non-helper node seen on the robot maps to a container via repo evidence."""
    pytest.importorskip("yaml")
    inv_path = tmp_path / "inventory.json"
    subprocess.run(
        [
            sys.executable,
            str(REPO / "tools/architecture_audit.py"),
            "--output",
            str(inv_path),
            "--markdown",
            str(tmp_path / "inventory.md"),
        ],
        cwd=REPO,
        check=True,
        capture_output=True,
    )
    inventory = json.loads(inv_path.read_text())
    nodes = [graph.canonical_node(n) for n in RUN_36339397150_NODES]
    visible = [n for n in nodes if not graph.is_helper_node(n)]
    assert len(nodes) == 52 and len(visible) == 43
    placement = graph.resolve_placement(visible, inventory, REPO)
    unplaced = [n for n in visible if n not in placement]
    assert unplaced == []
    where = {n: graph.service_label(placement[n]["service"]) for n in visible}
    assert where["bt_navigator"] == "nav2"
    assert where["global_costmap/global_costmap"] == "nav2"
    assert where["diff_drive_controller"] == "ros2-control"
    assert where["rtabmap/rtabmap"] == "rtabmap"
    assert where["tts_node"] == "voice-assistant"
    assert where["avatar_supervisor"] == "avatar-supervisor"
    assert where["ceiling_camera/usb_cam"] == "ceiling-camera"
    assert placement["lslidar_driver_node"]["service"]["host"] == "Main Pi"
    assert placement["apriltag"]["service"]["host"] == "Vision Pi"
