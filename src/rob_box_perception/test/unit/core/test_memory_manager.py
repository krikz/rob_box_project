"""Guard: MemoryManager is gone — zero runtime importers (issue #3869).

context_aggregator keeps its own short-term event buffer; the extracted
``core.memory_manager`` module was never wired after W10 and had no
production importer on origin/develop.
"""

import importlib
import importlib.util

import pytest


def test_memory_manager_module_is_removed():
    """Package path must not resolve after deletion."""
    spec = importlib.util.find_spec('rob_box_perception.core.memory_manager')
    assert spec is None


def test_memory_manager_import_raises():
    """Direct import must fail honestly (no silent stub)."""
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module('rob_box_perception.core.memory_manager')
