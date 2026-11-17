"""Тесты ожидаемой ноды диалога в ``NodeAvailabilityMonitor`` (issue #3865, ADR-0150 PR-8).

Нода диалога одна, ``/dialog_node`` (v1 ``/dialogue_node`` удалён, #4354 A5). Без ROS: ``rclpy`` монитору нужен только для
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


def test_dialog_node_is_the_expected_one():
    monitor = _monitor()
    _check(monitor, _OTHERS + ['/dialog_node'])

    summary = monitor.get_status_summary()
    assert summary['failed_list'] == []
    assert summary['missing_list'] == []
    assert '/dialog_node' in summary['active_list']
    assert summary['total'] == len(_OTHERS) + 1


def test_dialogue_node_is_not_expected_anywhere():
    """#4354 A5: ``/dialogue_node`` (v1) удалён, монитор его не ждёт и не помнит."""
    monitor = _monitor()
    _check(monitor, _OTHERS + ['/dialog_node'])

    assert '/dialogue_node' not in monitor.expected_nodes
    assert '/dialogue_node' not in monitor.node_status


def test_crash_of_dialog_node_is_failed():
    monitor = _monitor()
    _check(monitor, _OTHERS + ['/dialog_node'])
    _check(monitor, _OTHERS)

    assert monitor.get_failed_nodes() == ['/dialog_node']


def test_dialog_node_never_seen_is_missing():
    monitor = _monitor()
    _check(monitor, _OTHERS)

    assert monitor.get_missing_nodes() == ['/dialog_node']
    assert monitor.get_failed_nodes() == []


def test_explicit_expected_nodes_are_honoured():
    monitor = node_monitor.NodeAvailabilityMonitor(MagicMock(), ['/audio_node', '/dialog_node'])
    _check(monitor, ['/audio_node', '/dialog_node'])

    assert monitor.expected_nodes == ['/audio_node', '/dialog_node']
    assert monitor.get_failed_nodes() == []
