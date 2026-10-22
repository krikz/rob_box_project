"""PR-10 (#3704): ``pause``/``resume`` супервизора при ``dialog_engine=v2`` — ``DialogControl`` с ack владельца."""

from __future__ import annotations

import json
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_dialog_msgs.srv import DialogControl
from rob_box_supervisor.supervisor_node import (
    DIALOGUE_CONTROL_PAUSE,
    DIALOGUE_CONTROL_RESUME,
    _DialogControlClient,
)


class _Future:
    def __init__(self, ok: bool, answers: bool = True) -> None:
        self._ok, self._answers = ok, answers

    def add_done_callback(self, callback) -> None:
        if self._answers:
            callback(self)

    def result(self):
        return SimpleNamespace(ok=self._ok)


def _client(future: _Future, ready: bool = True):
    ros = MagicMock()
    ros.wait_for_service.return_value = ready
    ros.call_async.side_effect = lambda _request: future
    return ros, _DialogControlClient(ros, DialogControl, timeout_s=0.05)


def _msg(action: str, **extra):
    return SimpleNamespace(data=json.dumps({"action": action, "reason": "t", **extra}))


def test_pause_is_hold_by_operator_with_ttl_and_waits_for_ack():
    ros, client = _client(_Future(ok=True))
    client.publish(_msg(DIALOGUE_CONTROL_PAUSE, ttl_s=300.0))
    request = ros.call_async.call_args.args[0]
    assert (request.action, request.set_by, request.ttl_s) == (DialogControl.Request.ACTION_HOLD, "operator", 300.0)


def test_resume_is_resume_without_ttl():
    ros, client = _client(_Future(ok=True))
    client.publish(_msg(DIALOGUE_CONTROL_RESUME))
    request = ros.call_async.call_args.args[0]
    assert (request.action, request.ttl_s) == (DialogControl.Request.ACTION_RESUME, 0.0)


def test_no_service_is_a_loud_refusal():
    ros, client = _client(_Future(ok=True), ready=False)
    with pytest.raises(RuntimeError, match="сервиса нет"):
        client.publish(_msg(DIALOGUE_CONTROL_PAUSE))
    ros.call_async.assert_not_called()


def test_owner_refusal_is_not_silent_success():
    _ros, client = _client(_Future(ok=False))
    with pytest.raises(RuntimeError, match="отказал"):
        client.publish(_msg(DIALOGUE_CONTROL_PAUSE))


def test_no_ack_in_time_is_timeout():
    _ros, client = _client(_Future(ok=True, answers=False))
    with pytest.raises(TimeoutError):
        client.publish(_msg(DIALOGUE_CONTROL_PAUSE))
