"""Unit tests for SC-only custom synthdef registration helpers."""

import sys
from types import SimpleNamespace

from rob_box_voice.core.sc_only_custom_synthdefs import (
    CUSTOM_SC_ONLY_SYNTH_NAMES,
    register_sc_only_custom_synthdefs,
)


class FakeSynthDef:
    def __init__(self, name: str) -> None:
        self.name = name


def test_register_sc_only_custom_synthdefs_exposes_names_in_runtime_context() -> None:
    runtime_context = {"SynthDef": FakeSynthDef}
    runtime_module = SimpleNamespace(SynthDefs={})

    registered = register_sc_only_custom_synthdefs(runtime_module, runtime_context)

    assert registered == CUSTOM_SC_ONLY_SYNTH_NAMES
    assert set(runtime_module.SynthDefs) == set(CUSTOM_SC_ONLY_SYNTH_NAMES)
    for name in CUSTOM_SC_ONLY_SYNTH_NAMES:
        assert runtime_context[name].name == name


def test_register_sc_only_custom_synthdefs_includes_imperial_march_palette() -> None:
    assert "imperialbrass" in CUSTOM_SC_ONLY_SYNTH_NAMES
    assert "marchstrings" in CUSTOM_SC_ONLY_SYNTH_NAMES


def test_register_sc_only_custom_synthdefs_reuses_existing_registry_entries() -> None:
    existing = FakeSynthDef("warmpad")
    runtime_context = {"SynthDef": FakeSynthDef}
    runtime_module = SimpleNamespace(SynthDefs={"warmpad": existing})

    register_sc_only_custom_synthdefs(runtime_module, runtime_context)

    assert runtime_context["warmpad"] is existing
    assert runtime_module.SynthDefs["warmpad"] is existing


def test_register_sc_only_custom_synthdefs_falls_back_to_file_synthdef_factory() -> None:
    runtime_context: dict[str, object] = {}
    runtime_module = SimpleNamespace(SynthDefs={})
    fake_module = SimpleNamespace(FileSynthDef=FakeSynthDef)

    original_module = sys.modules.get("renardo_lib.SynthDefManagement.SimpleSynthDefs")
    sys.modules["renardo_lib.SynthDefManagement.SimpleSynthDefs"] = fake_module
    try:
        registered = register_sc_only_custom_synthdefs(runtime_module, runtime_context)
    finally:
        if original_module is None:
            del sys.modules["renardo_lib.SynthDefManagement.SimpleSynthDefs"]
        else:
            sys.modules["renardo_lib.SynthDefManagement.SimpleSynthDefs"] = original_module

    assert registered == CUSTOM_SC_ONLY_SYNTH_NAMES
    for name in CUSTOM_SC_ONLY_SYNTH_NAMES:
        assert runtime_context[name].name == name


def test_register_sc_only_custom_synthdefs_requires_synthdef_factory() -> None:
    runtime_context: dict[str, object] = {}
    runtime_module = SimpleNamespace(SynthDefs={})

    try:
        register_sc_only_custom_synthdefs(runtime_module, runtime_context)
    except ValueError as exc:
        assert "SynthDef factory" in str(exc)
    else:
        raise AssertionError("Expected ValueError when SynthDef factory is missing")