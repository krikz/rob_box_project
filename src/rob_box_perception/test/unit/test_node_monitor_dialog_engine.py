"""Тесты ожидаемой ноды диалога в ``NodeAvailabilityMonitor`` (issue #3865, ADR-0150 PR-8).

При ``dialog_engine: v2`` стартует ``/dialog_node``, ``/dialogue_node`` не запускается по замыслу; монитор не должен
считать её упавшей (иначе ``get_perception_context`` вечно ``DEGRADED``). Без ROS: ``rclpy`` монитору нужен только для
аннотаций.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_node_monitor_dialog_engine.py -v -o addopts=""
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

_PKG_ROOT = Path(__file__).resolve().parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

node_monitor = importlib.import_module('rob_box_perception.utils.node_monitor')

_OTHERS = [
    '/audio_node', '/stt_node', '/tts_node', '/context_aggregator', '/camera/camera',
    '/lslidar_driver_node', '/rtabmap/rtabmap',
]


def _monitor():
    node = MagicMock()
    return node_monitor.NodeAvailabilityMonitor(node)


def _check(monitor, active):
    out = SimpleNamespace(returncode=0, stdout='\n'.join(active), stderr='')
    with patch.object(node_monitor.subprocess, 'run', return_value=out):
        monitor.check_nodes()


@pytest.mark.parametrize('engine_node, other', [
    ('/dialog_node', '/dialogue_node'),   # dialog_engine: v2
    ('/dialogue_node', '/dialog_node'),   # dialog_engine: v1
])
def test_running_engine_node_is_the_expected_one(engine_node, other):
    monitor = _monitor()
    _check(monitor, _OTHERS + [engine_node])

    summary = monitor.get_status_summary()
    assert summary['failed_list'] == []
    assert summary['missing_list'] == []
    assert engine_node in summary['active_list']
    assert other not in monitor.node_status
    assert other not in monitor.expected_nodes
    assert summary['total'] == len(_OTHERS) + 1


@pytest.mark.parametrize('engine_node', ['/dialog_node', '/dialogue_node'])
def test_crash_of_running_engine_node_is_failed(engine_node):
    monitor = _monitor()
    _check(monitor, _OTHERS + [engine_node])
    _check(monitor, _OTHERS)

    assert monitor.get_failed_nodes() == [engine_node]


def test_dialog_node_never_seen_is_missing_under_combined_label():
    monitor = _monitor()
    _check(monitor, _OTHERS)

    assert monitor.get_missing_nodes() == [node_monitor.DIALOG_NODES_LABEL]
    assert monitor.get_failed_nodes() == []


def test_engine_switch_drops_stale_entry_of_old_engine():
    monitor = _monitor()
    _check(monitor, _OTHERS + ['/dialogue_node'])
    _check(monitor, _OTHERS + ['/dialog_node'])

    assert monitor.get_failed_nodes() == []
    assert '/dialogue_node' not in monitor.node_status
    assert monitor.get_active_nodes().count('/dialog_node') == 1


def test_explicit_expected_nodes_are_honoured():
    monitor = node_monitor.NodeAvailabilityMonitor(MagicMock(), ['/audio_node', '/dialogue_node'])
    _check(monitor, ['/audio_node', '/dialog_node'])

    assert monitor.expected_nodes == ['/audio_node', '/dialog_node']
    assert monitor.get_failed_nodes() == []
