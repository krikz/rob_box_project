"""nav2_goal.Nav2GoalBridge (issue #3151) на фейковом action-клиенте.

Фейк повторяет ровно ту часть rclpy ActionClient API, которой пользуется
мост: ``server_is_ready`` / ``send_goal_async(goal, feedback_callback=)``
→ future с ``add_done_callback``; handle с ``accepted`` /
``get_result_async`` / ``cancel_goal_async``. Колбэки дёргаем руками — так
же, как их дёрнул бы ROS executor.
"""

from __future__ import annotations

from types import SimpleNamespace

from rob_box_quest.core.nav_goal import NavGoalRequest
from rob_box_quest.nav2_goal import (
    NAV2_TIMEOUT_REASON,
    Nav2GoalBridge,
    build_navigate_to_pose_goal,
)


class _Future:
    def __init__(self):
        self._cbs = []
        self._result = None
        self._exc = None

    def add_done_callback(self, cb):
        self._cbs.append(cb)

    def result(self):
        if self._exc is not None:
            raise self._exc
        return self._result

    def complete(self, result=None, exc=None):
        self._result, self._exc = result, exc
        for cb in self._cbs:
            cb(self)


class _Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = _Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _Future()


class _Client:
    def __init__(self, ready=True):
        self.ready = ready
        self.sent = []  # (goal, feedback_cb, future)

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        fut = _Future()
        self.sent.append((goal, feedback_callback, fut))
        return fut


class _Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _make(ready=True, result_timeout_s=15.0):
    client = _Client(ready)
    events = []
    terminals = []
    clock = _Clock()
    bridge = Nav2GoalBridge(
        client,
        make_goal=lambda req: ("goal", req),
        emit=events.append,
        on_terminal=lambda: terminals.append(True),
        clock=clock,
        wall_ms=lambda: 123,
        result_timeout_s=result_timeout_s,
    )
    return bridge, client, events, terminals, clock


REQ = NavGoalRequest(seq=1, x=2.0, y=3.0, yaw=0.5)


def _states(events):
    return [e["state"] for e in events]


def test_nav2_not_ready_is_nack():
    bridge, client, events, _, _ = _make(ready=False)
    assert bridge.send_goal(REQ) == "nav2_unavailable"
    assert client.sent == []
    assert events == []


def test_happy_path_accepted_feedback_succeeded():
    bridge, client, events, terminals, clock = _make()
    assert bridge.send_goal(REQ) is None
    goal, fb_cb, fut = client.sent[0]
    assert goal == ("goal", REQ)
    handle = _Handle(accepted=True)
    fut.complete(handle)
    assert _states(events) == ["accepted"]
    fb_cb(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=4.5)))
    fb_cb(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=4.4)))  # дроссель
    clock.t += 0.6
    fb_cb(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=3.0)))
    assert _states(events) == ["accepted", "active", "active"]
    assert events[1]["distance_remaining"] == 4.5
    assert events[2]["distance_remaining"] == 3.0
    handle.result_future.complete(SimpleNamespace(status=4))
    assert _states(events)[-1] == "succeeded"
    assert events[-1]["seq"] == 1 and events[-1]["x"] == 2.0
    assert terminals == [True]
    assert not bridge.has_active_goal()


def test_rejected_by_nav2():
    bridge, client, events, terminals, _ = _make()
    bridge.send_goal(REQ)
    client.sent[0][2].complete(_Handle(accepted=False))
    assert _states(events) == ["rejected"]
    assert events[0]["reason"] == "nav2_rejected"
    assert terminals == [True]


def test_send_failure_is_rejected_with_reason():
    bridge, client, events, _, _ = _make()
    bridge.send_goal(REQ)
    client.sent[0][2].complete(exc=RuntimeError("boom"))
    assert _states(events) == ["rejected"]
    assert "boom" in events[0]["reason"]


def test_non_terminal_result_is_not_reported_as_success():
    bridge, client, events, _, _ = _make()
    bridge.send_goal(REQ)
    handle = _Handle()
    client.sent[0][2].complete(handle)
    handle.result_future.complete(SimpleNamespace(status=0))
    assert _states(events)[-1] == "aborted"


def test_cancel_with_handle_calls_cancel_and_reports_canceled():
    bridge, client, events, terminals, _ = _make()
    bridge.send_goal(REQ)
    handle = _Handle()
    client.sent[0][2].complete(handle)
    assert bridge.cancel() is True
    assert handle.cancel_calls == 1
    handle.result_future.complete(SimpleNamespace(status=5))
    assert _states(events)[-1] == "canceled"
    assert terminals == [True]


def test_cancel_before_goal_response_cancels_on_accept():
    bridge, client, events, _, _ = _make()
    bridge.send_goal(REQ)
    assert bridge.cancel() is True
    handle = _Handle()
    client.sent[0][2].complete(handle)
    assert handle.cancel_calls == 1


def test_cancel_without_goal():
    bridge, _, _, _, _ = _make()
    assert bridge.cancel() is False


def test_preempted_goal_events_are_dropped():
    bridge, client, events, terminals, _ = _make()
    bridge.send_goal(REQ)
    old_handle = _Handle()
    client.sent[0][2].complete(old_handle)
    req2 = NavGoalRequest(seq=2, x=5.0, y=5.0, yaw=0.0)
    bridge.send_goal(req2)
    # Nav2 вытеснил старую цель: её ABORTED — не наш провал.
    old_handle.result_future.complete(SimpleNamespace(status=6))
    client.sent[0][1](SimpleNamespace(feedback=SimpleNamespace(distance_remaining=1.0)))
    assert [(e["seq"], e["state"]) for e in events] == [(1, "accepted")]
    assert terminals == []
    new_handle = _Handle()
    client.sent[1][2].complete(new_handle)
    assert [(e["seq"], e["state"]) for e in events][-1] == (2, "accepted")


def test_emit_failure_does_not_raise():
    client = _Client()
    bridge = Nav2GoalBridge(client, make_goal=lambda r: r, emit=lambda e: 1 / 0)
    bridge.send_goal(REQ)
    client.sent[0][2].complete(_Handle())  # не должно бросить


def test_check_timeout_noop_without_active_goal():
    bridge, _, events, terminals, clock = _make(result_timeout_s=10.0)
    clock.t += 999.0
    bridge.check_timeout()
    assert events == []
    assert terminals == []


def test_check_timeout_noop_before_deadline():
    bridge, client, events, _, clock = _make(result_timeout_s=10.0)
    bridge.send_goal(REQ)
    client.sent[0][2].complete(_Handle())
    clock.t += 9.9
    bridge.check_timeout()
    assert _states(events) == ["accepted"]


def test_check_timeout_aborts_when_accepted_without_feedback():
    """Nav2 принял цель и умолк — таймер стартует уже с accept."""
    bridge, client, events, terminals, clock = _make(result_timeout_s=10.0)
    bridge.send_goal(REQ)
    handle = _Handle()
    client.sent[0][2].complete(handle)
    clock.t += 10.1
    bridge.check_timeout()
    assert _states(events) == ["accepted", "aborted"]
    assert events[-1]["reason"] == NAV2_TIMEOUT_REASON
    assert handle.cancel_calls == 1
    assert terminals == [True]
    assert not bridge.has_active_goal()


def test_check_timeout_aborts_when_no_response_at_all():
    """Nav2 даже не ответил на send_goal — таймер стартует с send."""
    bridge, client, events, terminals, clock = _make(result_timeout_s=10.0)
    bridge.send_goal(REQ)
    clock.t += 10.1
    bridge.check_timeout()
    assert _states(events) == ["aborted"]
    assert events[-1]["reason"] == NAV2_TIMEOUT_REASON
    assert terminals == [True]


def test_feedback_resets_timeout_clock():
    bridge, client, events, _, clock = _make(result_timeout_s=10.0)
    bridge.send_goal(REQ)
    handle = _Handle()
    fb_cb = client.sent[0][1]
    client.sent[0][2].complete(handle)
    clock.t += 9.0
    fb_cb(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=5.0)))
    clock.t += 9.0
    bridge.check_timeout()
    # 18с с accept, но только 9с с последнего feedback — ещё не пора.
    assert "aborted" not in _states(events)


def test_check_timeout_is_idempotent_after_finish():
    bridge, client, events, terminals, clock = _make(result_timeout_s=10.0)
    bridge.send_goal(REQ)
    clock.t += 10.1
    bridge.check_timeout()
    bridge.check_timeout()
    assert _states(events) == ["aborted"]
    assert terminals == [True]


def test_build_goal_sets_map_frame_and_quaternion():
    class Goal:
        def __init__(self):
            self.pose = SimpleNamespace(
                header=SimpleNamespace(frame_id="", stamp=None),
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )

    req = NavGoalRequest(seq=1, x=1.0, y=2.0, yaw=3.141592653589793)
    goal = build_navigate_to_pose_goal(Goal, req, "stamp")
    assert goal.pose.header.frame_id == "map"
    assert goal.pose.header.stamp == "stamp"
    assert (goal.pose.pose.position.x, goal.pose.pose.position.y) == (1.0, 2.0)
    assert abs(goal.pose.pose.orientation.z - 1.0) < 1e-9
    assert abs(goal.pose.pose.orientation.w) < 1e-9
