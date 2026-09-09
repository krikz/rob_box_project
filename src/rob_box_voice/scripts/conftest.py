#!/usr/bin/env python3
# Copyright 2026 krikz (Rob Box Project)
#
# Licensed under the MIT License. See LICENSE for details.
"""Pytest configuration for ``scripts/``.

Files in this directory have the ``test_*.py`` prefix so humans can run them as
manual CLI smoke tests (e.g. ``python scripts/test_yandex_v3.py``), but they
are **not** unit tests. They depend on optional hardware and services — ReSpeaker
USB arrays, ``sounddevice``, ``grpc``, Yandex Cloud, OpenAI/MiniMax TTS — that
are unavailable in CI.

``colcon test`` runs ``pytest`` from the installed package directory, and
without this conftest it would collect every ``test_*.py`` here and fail the
build on the missing imports.

This conftest tells pytest "scripts/ is not a test directory". It is dynamic:
any future ``test_*.py`` added to ``scripts/`` is automatically excluded.
"""

from pathlib import Path

# Absolute glob patterns relative to this conftest's directory.
# Using __file__ makes the ignore work regardless of pytest's rootdir/cwd —
# this matters because colcon chdir's into the install site-packages dir.
_THIS_DIR = Path(__file__).resolve().parent
collect_ignore_glob = sorted(
    str(p) for p in _THIS_DIR.glob("test_*.py")
) + sorted(
    str(p) for p in _THIS_DIR.glob("*_test.py") if str(p) not in {str(x) for x in _THIS_DIR.glob("test_*.py")}
)
