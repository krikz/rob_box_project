"""architecture audit 2026-09-29, ADR-0145: единый набор мусорных имён спикера."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from rob_box_core.speaker_names import INVALID_SPEAKER_NAMES

# Члены, которые раньше были только в одной из трёх разошедшихся копий.
DIVERGED_MEMBERS = [
    "null",
    "none",
    "undefined",
    "unknown",
    "",
    "гость",
    "user",
    "зовут",
    "имя",
    "-",
    "?",
]


def test_is_frozenset_lowercase():
    assert isinstance(INVALID_SPEAKER_NAMES, frozenset)
    assert all(n == n.lower() for n in INVALID_SPEAKER_NAMES)


@pytest.mark.parametrize("name", DIVERGED_MEMBERS)
def test_contains_previously_divergent_members(name):
    assert name in INVALID_SPEAKER_NAMES


@pytest.mark.parametrize("name", ["юзеф", "гостомысл", "денис"])
def test_real_names_not_filtered(name):
    assert name not in INVALID_SPEAKER_NAMES


def _load_by_path(name: str, rel: str):
    """Загрузить модуль rob_box_voice по пути, минуя тяжёлый ``__init__`` пакета."""
    path = Path(__file__).resolve().parents[2] / "rob_box_voice" / "rob_box_voice" / rel
    if not path.exists():
        pytest.skip(f"{path} not found in this checkout")
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod  # dataclass + `from __future__ import annotations`
    spec.loader.exec_module(mod)
    return mod


def test_dialogue_helpers_alias_is_same_object():
    mod = _load_by_path("_dh_under_test", "core/dialogue_helpers.py")
    assert mod.INVALID_SPEAKER_NAMES is INVALID_SPEAKER_NAMES


def test_speaker_embeddings_alias_is_same_object():
    pytest.importorskip("numpy")
    mod = _load_by_path("_se_under_test", "utils/speaker_embeddings.py")
    assert mod._INVALID_SPEAKER_NAMES is INVALID_SPEAKER_NAMES


def test_mcp_register_speaker_alias_is_same_object():
    try:
        from rob_box_mcp_tools.tools.dialogue import RegisterSpeakerTool
    except ImportError as exc:  # rclpy / rob_box_voice deps недоступны
        pytest.skip(f"rob_box_mcp_tools not importable: {exc}")
    assert RegisterSpeakerTool._NOISE_NAMES is INVALID_SPEAKER_NAMES
