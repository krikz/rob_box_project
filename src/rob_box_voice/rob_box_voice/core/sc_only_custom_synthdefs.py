"""Helpers for registering repository-owned SC-only custom synthdefs."""

from __future__ import annotations

from importlib import import_module
from typing import Any


CUSTOM_SC_ONLY_SYNTH_NAMES = (
    "warmpad",
    "retrobass",
    "supersawlead",
    "imperialbrass",
    "marchstrings",
    "strangerpulsepad",
    "strangerarp",
    "strangerbrass",
)


def _resolve_synthdef_factory(runtime_module: Any, runtime_context: dict[str, Any]) -> Any:
    """Resolve the Python-side synth wrapper factory exposed by renardo_lib.

    Different renardo_lib builds expose different factory entry points.
    The runtime module used in production does not export `SynthDef`, but built-in
    file-backed synths are created with `FileSynthDef(...)`.
    """

    synthdef_factory = runtime_context.get("SynthDef") or getattr(runtime_module, "SynthDef", None)
    if synthdef_factory is not None:
        return synthdef_factory

    try:
        simple_synthdefs = import_module("renardo_lib.SynthDefManagement.SimpleSynthDefs")
    except ImportError as exc:
        raise ValueError("Renardo runtime does not expose SynthDef factory") from exc

    file_synthdef = getattr(simple_synthdefs, "FileSynthDef", None)
    if file_synthdef is None:
        raise ValueError("Renardo runtime does not expose SynthDef factory")

    return file_synthdef


def register_sc_only_custom_synthdefs(runtime_module: Any, runtime_context: dict[str, Any]) -> tuple[str, ...]:
    """Expose repository-owned custom synthdefs in the Renardo runtime context.

    The SC `.scd` sources are preloaded separately by `sclang` startup. This helper
    only creates matching Python-side `SynthDef(name)` wrappers so generated FoxDot
    code can use direct syntax like `p1 >> warmpad(...)`.
    """

    synthdef_factory = _resolve_synthdef_factory(runtime_module, runtime_context)

    registry = getattr(runtime_module, "SynthDefs", None)
    registered: list[str] = []

    for name in CUSTOM_SC_ONLY_SYNTH_NAMES:
        synth = None
        if isinstance(registry, dict):
            synth = registry.get(name)

        if synth is None:
            synth = synthdef_factory(name)
            if isinstance(registry, dict):
                registry[name] = synth

        runtime_context[name] = synth
        registered.append(name)

    return tuple(registered)