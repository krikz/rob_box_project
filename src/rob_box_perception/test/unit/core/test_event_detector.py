"""Guard: EventDetector is gone — zero runtime importers (issue #3869).

ADR-0103 marks ``core.event_detector`` dead: the only importer on
origin/develop was this unit test. OccasionGate keeps its own cooldown
state; the extracted module was never wired after W10.
"""

import importlib
import importlib.util

import pytest


def test_event_detector_module_is_removed():
    """Package path must not resolve after deletion."""
    spec = importlib.util.find_spec('rob_box_perception.core.event_detector')
    assert spec is None


def test_event_detector_import_raises():
    """Direct import must fail honestly (no silent stub)."""
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module('rob_box_perception.core.event_detector')
