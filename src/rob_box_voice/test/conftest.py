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

from pathlib import Path

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
