#!/usr/bin/env python3
# Copyright 2026 krikz (Rob Box Project)
#
# Licensed under the MIT License. See LICENSE for details.
"""Test directory marker for pytest 9.x.

pytest 9.x auto-discovery resolves dotted module names from the layout of
``test_*.py`` files. When collecting ``test/unit/tts/test_*.py`` it tries to
import ``test.unit`` as a Python package. Without this ``__init__.py`` the
import fails with ``ModuleNotFoundError: No module named 'test.unit'``.

The file is intentionally empty — tests are not part of the runtime package;
this marker only tells pytest 9.x that the namespace exists.
"""
