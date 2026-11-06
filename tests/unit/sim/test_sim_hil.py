"""Сторож графа HIL (rob_box_sim.hil): «ровно один /controller_manager, и он в Gazebo».

Эпик #4371, план docs/research/2026-11-04-hil-robot-on-gazebo-data.md §5.1–§5.2.
Проверяется решение по списку узлов; rclpy не нужен.
"""

from __future__ import annotations

from rob_box_sim import hil

ROBOT_GRAPH = ["/robot_state_publisher", "/twist_mux", "/rtabmap/rtabmap", "/bt_navigator"]
GAZEBO_GRAPH = ["/controller_manager", "/diff_drive_controller", "/gz_ros2_control",
                "/sim/robot_state_publisher", "/sim/hil_guard"]


def test_full_names_like_ros2_node_list():
    pairs = [("controller_manager", "/"), ("robot_state_publisher", "/sim"), ("rtabmap", "/rtabmap/")]
    assert hil.full_names(pairs) == ["/controller_manager", "/rtabmap/rtabmap", "/sim/robot_state_publisher"]


def test_precheck_passes_when_robot_visible_and_no_controller():
    v = hil.precheck(ROBOT_GRAPH)
    assert v.ok, v.reason


def test_precheck_refuses_when_robot_controller_alive():
    v = hil.precheck(ROBOT_GRAPH + ["/controller_manager"])
    assert not v.ok and "ros2-control" in v.reason


def test_precheck_refuses_without_link_to_robot():
    # «Контроллеров нет» без видимого робота ничего не доказывает.
    v = hil.precheck(["/some_node"])
    assert not v.ok and hil.ROBOT_LINK_PROBE in v.reason


def test_precheck_without_link_check_is_only_for_sandbox():
    assert hil.precheck([], require_link=False).ok
    assert not hil.precheck(["/controller_manager"], require_link=False).ok


def test_sim_rsp_is_not_robot_rsp():
    # Сим-RSP не должен совпасть с RSP Main Pi — иначе precheck видел бы «робота» в самом себе,
    # а gz_ros2_control брал бы боевой URDF.
    assert hil.SIM_RSP_NODE != hil.ROBOT_LINK_PROBE
    assert hil.SIM_RSP_NODE.startswith(f"/{hil.SIM_NAMESPACE}/")
    assert hil.SIM_ROBOT_DESCRIPTION_TOPIC.startswith(f"/{hil.SIM_NAMESPACE}/")
    assert dict(hil.SIM_TF_REMAPS) == {"/tf": "/sim/tf", "/tf_static": "/sim/tf_static"}


def test_watch_ok_with_one_gazebo_controller():
    v = hil.watch_verdict(ROBOT_GRAPH + GAZEBO_GRAPH)
    assert v.ok, v.reason


def test_watch_ok_while_gazebo_controller_not_up_yet():
    assert hil.watch_verdict(ROBOT_GRAPH).ok


def test_watch_violation_when_robot_controller_returns():
    v = hil.watch_verdict(ROBOT_GRAPH + GAZEBO_GRAPH + ["/controller_manager"])
    assert not v.ok and "2 ×" in v.reason


def test_namespaced_controller_manager_is_not_counted():
    assert hil.count_controller_managers(["/sim/controller_manager", "/controller_manager_x"]) == 0
