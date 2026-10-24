"""Фейковый rclpy для fake_robot_ssh.sh (issue #3871): ``docker exec voice-assistant python3 -c ...``.

Харнесс гоняет внутри контейнера настоящий python (подписанный /mcp/execute, снимок /voice/music/state).
Здесь вместо ROS -- файловый «плеер» в $FAKE_ROBOT_STATE:

* ``music_state.json`` -- последний latched-снимок /voice/music/state (нет файла -- плеер ничего не публиковал);
* ``mcp_calls`` -- по JSON-строке на каждый /mcp/execute (с признаком подписи);
* стоп-тул (``stop_music`` / ``dj_set`` action=stop) переводит плеер в idle через FAKE_MUSIC_STOP_DELAY секунд;
  FAKE_MUSIC_STUCK=1 -- плеер стоп игнорирует (тул отвечает ok, а музыка играет).
"""

import json
import os
import time

_STATE = os.environ.get("FAKE_ROBOT_STATE", "")
_STATE_FILE = os.path.join(_STATE, "music_state.json")
_CALLS_FILE = os.path.join(_STATE, "mcp_calls")

_pending_stops = []  # (due, tool)
_queued_results = []  # (due, json_text)
_nodes = []


def _read_state():
    try:
        with open(_STATE_FILE, encoding="utf-8") as fh:
            return fh.read()
    except OSError:
        return None


def _write_idle():
    with open(_STATE_FILE, "w", encoding="utf-8") as fh:
        json.dump({"state": "idle", "track_id": None, "form_ends_at": None, "stops_at": None,
                   "dj": {"enabled": False}, "finished_track_id": None, "ts": time.time()}, fh)


class _Publisher:
    def __init__(self, topic):
        self.topic = topic

    def get_subscription_count(self):
        return 1

    def publish(self, msg):
        request = json.loads(msg.data)
        signed = bool(request.get("signature")) and request.get("sender") == "harness"
        with open(_CALLS_FILE, "a", encoding="utf-8") as fh:
            fh.write(json.dumps({"tool": request.get("tool_name"), "parameters": request.get("parameters"),
                                 "signed": signed}, ensure_ascii=False) + "\n")
        now = time.time()
        if not signed:
            result = {"success": False, "error": "unsigned_request"}
        else:
            result = {"success": True, "data": {}}
            tool, params = request.get("tool_name"), request.get("parameters") or {}
            if tool == "stop_music" or (tool == "dj_set" and params.get("action") == "stop"):
                if os.environ.get("FAKE_MUSIC_STUCK") != "1":
                    _pending_stops.append((now + float(os.environ.get("FAKE_MUSIC_STOP_DELAY", "0")), tool))
        _queued_results.append((now, json.dumps({"tool_name": request.get("tool_name"),
                                                 "request_id": request.get("request_id"), "result": result})))


class _Node:
    def __init__(self, name):
        self.name = name
        self.subs = []
        self._last_state = None

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs.append((topic, callback))

    def create_publisher(self, msg_type, topic, qos):
        return _Publisher(topic)

    def destroy_node(self):
        pass


def init(*args, **kwargs):
    pass


def shutdown(*args, **kwargs):
    pass


def create_node(name):
    node = _Node(name)
    _nodes.append(node)
    return node


def spin_once(node, timeout_sec=0.0):
    from std_msgs.msg import String

    time.sleep(min(float(timeout_sec or 0), 0.05))
    now = time.time()
    for due, tool in list(_pending_stops):
        if now >= due:
            _pending_stops.remove((due, tool))
            _write_idle()
    for item in list(_queued_results):
        if now >= item[0]:
            _queued_results.remove(item)
            for topic, callback in node.subs:
                if topic == "/mcp/result":
                    callback(String(item[1]))
    state = _read_state()
    if state is not None and state != node._last_state:  # latched: подписчик получает последний снимок
        node._last_state = state
        for topic, callback in node.subs:
            if topic == "/voice/music/state":
                callback(String(state))
