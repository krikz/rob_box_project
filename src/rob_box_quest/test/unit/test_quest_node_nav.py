"""QuestBridge ↔ Nav2 (issue #3151): emergency-гейт, отмена по E-stop, nav_path."""

from __future__ import annotations

import struct
from types import SimpleNamespace
from unittest.mock import MagicMock

import msgpack

# Фикстура ``quest_node_mod`` (ROS-заглушки) приходит из test/unit/conftest.py.
from rob_box_quest.core.nav_goal import NavGoalRequest

REQ = NavGoalRequest(seq=1, x=1.0, y=2.0, yaw=0.0)


class _WS:
    def __init__(self):
        self.frames = []

    def broadcast_frame(self, ui_name, payload):
        self.frames.append((ui_name, payload))
        return 1

    def get_active_sessions(self):
        return 1


def _bridge(mod):
    ws = _WS()
    bridge = mod.QuestBridge(
        node=MagicMock(),
        cmd_vel_quest_pub=MagicMock(),
        cmd_vel_emergency_pub=MagicMock(),
        ws_server=ws,
    )
    return bridge, ws


def _nav2():
    nav2 = MagicMock()
    nav2.send_goal.return_value = None
    nav2.cancel.return_value = True
    return nav2


def test_nav_goal_without_nav2_is_unavailable(quest_node_mod):
    bridge, _ = _bridge(quest_node_mod)
    assert bridge.nav_goal("quest:x", REQ) == "nav2_unavailable"
    assert bridge.nav_cancel("quest:x") is False


def test_nav_goal_forwarded_to_nav2(quest_node_mod):
    bridge, _ = _bridge(quest_node_mod)
    nav2 = _nav2()
    bridge.attach_nav2(nav2)
    assert bridge.nav_goal("quest:x", REQ) is None
    nav2.send_goal.assert_called_once_with(REQ)


def test_emergency_lock_blocks_nav_goal_and_cancels_active(quest_node_mod):
    bridge, _ = _bridge(quest_node_mod)
    nav2 = _nav2()
    bridge.attach_nav2(nav2)
    bridge.emergency_stop()
    nav2.cancel.assert_called_once_with()
    assert bridge.nav_goal("quest:x", REQ) == "emergency_active"
    nav2.send_goal.assert_not_called()
    bridge.reset()  # новый HELLO снимает lock
    assert bridge.nav_goal("quest:x", REQ) is None


def _path(frame, pts):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.0))) for x, y in pts],
    )


def test_on_nav_path_publishes_decimated_map_frame(quest_node_mod):
    bridge, ws = _bridge(quest_node_mod)
    bridge.on_nav_path(_path("map", [(float(i), 0.0) for i in range(500)]))
    assert len(ws.frames) == 1
    name, payload = ws.frames[0]
    body = msgpack.unpackb(payload, raw=False)
    assert name == "nav_path"
    assert body["frame"] == "map" and body["n"] == 200
    xy = struct.unpack(f"<{2 * body['n']}f", body["xy"])
    assert xy[0] == 0.0 and xy[-2] == 499.0
    # Второй план сразу же — дроссель ≤ 2 Гц.
    bridge.on_nav_path(_path("map", [(0.0, 0.0), (1.0, 1.0)]))
    assert len(ws.frames) == 1


def test_on_nav_path_drops_non_map_frame(quest_node_mod):
    bridge, ws = _bridge(quest_node_mod)
    bridge.on_nav_path(_path("odom", [(0.0, 0.0), (1.0, 0.0)]))
    assert ws.frames == []


def test_publish_nav_path_clear_bypasses_throttle(quest_node_mod):
    bridge, ws = _bridge(quest_node_mod)
    bridge.on_nav_path(_path("map", [(0.0, 0.0), (1.0, 0.0)]))
    bridge.publish_nav_path_clear()
    assert len(ws.frames) == 2
    assert msgpack.unpackb(ws.frames[1][1], raw=False)["n"] == 0
