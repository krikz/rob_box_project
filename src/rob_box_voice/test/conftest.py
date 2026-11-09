#!/usr/bin/env python3
# Copyright 2026 krikz (Rob Box Project)
#
# Licensed under the MIT License. See LICENSE for details.
"""Pytest configuration for the top-level ``test/`` directory.

This directory holds two layers of tests:

* ``test/unit/**`` — pure unit tests, CI-friendly (max duration ~minutes).
* ``test/*.py`` (root) — integration / smoke tests that import the real ROS2
  nodes (``rclpy``, ``std_msgs``, ``nav2_msgs``), require optional hardware
  (``sounddevice``, ``grpc``) and external services (Yandex Cloud, OpenAI).

The root-level tests are intentionally NOT collected by default; they
require a fully provisioned workstation and must be run on the robot.

This conftest makes the intent explicit at the dir level so pytest 9.x
(automatic-discovery mode) does not try to import them.
"""

import sys
from pathlib import Path

# ``from ros_stubs import ...`` должно работать из любого каталога тестов.
sys.path.insert(0, str(Path(__file__).resolve().parent))

# ADR-0150 PR-2: ``rob_box_dialog_msgs`` (rosidl) без ROS не собирается;
# заглушка строится из настоящих ``.msg`` и ставится до импорта любых нод.
from rob_box_core.msg_stub import install_dialog_msgs_stub  # noqa: E402

install_dialog_msgs_stub()

# ADR-0167 Ш1б: хосты робота (dialog_node, stt_node) грузят профиль продукта строго (``robot_profile.path()``,
# в контейнере /config/assistant.yaml). В тестах - тот же профиль из чекаута; заданный снаружи env не трогаем.
import os as _os  # noqa: E402

_ROBOT_PROFILE = Path(__file__).resolve().parents[3] / "docker" / "sound" / "config" / "assistant.yaml"
_os.environ.setdefault("ROB_BOX_PROFILE", str(_ROBOT_PROFILE))

import pytest  # noqa: E402

from rob_box_dialog import knowledge as _kn  # noqa: E402
from voice_robot_wake import ROBOT_WAKE as _ROBOT_WAKE  # noqa: E402


@pytest.fixture(autouse=True)
def _robot_wake_table(monkeypatch):
    """Правила ядра без таблицы - на вейках робота, как у хоста робота, который всегда передаёт таблицу профиля
    (ядро без профиля нейтрально, ADR-0167 §2.4)."""
    monkeypatch.setattr(_kn, "WAKE_WORDS_WITHOUT_PROFILE", _ROBOT_WAKE)


# This file lives at test/conftest.py — ``test`` IS a Python package thanks
# to test/unit/__init__.py, so pytest will pick this up for any subdirectory.
# We use __file__ to resolve the root ``test/`` directory at runtime.
_THIS_DIR = Path(__file__).resolve().parent

# Globs are absolute paths relative to this conftest and are matched as
# patterns; pytest tests each path against these.
collect_ignore_glob = [
    str(p) for p in _THIS_DIR.glob("test_*.py")
]

# Also ensure the obsolete conftest at ``test/`` itself never gets re-collected
# as a test module (it isn't, but defensive).
collect_ignore = []
