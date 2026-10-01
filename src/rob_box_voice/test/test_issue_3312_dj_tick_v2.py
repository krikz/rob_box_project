"""ADR-0149 PR-5 (эпик #3312): при ``music_engine: v2`` тик DJ старого пути не заводится.

Переходы сета v2 ведёт ``SetSession`` в ``mcp_server`` по ``nearly_finished``; ход LLM на
переходе (``DJ_AUTO``) при v2 не должен случиться ни разу — поэтому таймера ``tick`` нет вовсе.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

import test_dialogue_shell  # noqa: F401 — заглушки rclpy/std_msgs до импорта dialogue_node
from rob_box_voice.core.dj_mode import DJModeController
from rob_box_voice.dialogue_node import start_dj_tick

pytestmark = pytest.mark.unit


class _Node:
    def __init__(self, engine):
        self.engine, self.timers, self.info = engine, [], []

    def get_parameter(self, name):
        assert name == "music_engine"
        return SimpleNamespace(value=self.engine)

    def get_logger(self):
        return SimpleNamespace(info=self.info.append)

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return object()


def _tick():
    raise AssertionError("tick вызван")


@pytest.mark.parametrize("engine", ["v1", "", None])
def test_v1_starts_the_dj_tick_as_before(engine):
    node = _Node(engine)
    assert start_dj_tick(node, _tick) is not None
    assert node.timers == [(DJModeController.DJ_TICK_INTERVAL_S, _tick)]


def test_v2_never_starts_the_dj_tick():
    node = _Node("v2")
    assert start_dj_tick(node, _tick) is None
    assert node.timers == []
    assert any("tick не запускается" in m for m in node.info)
