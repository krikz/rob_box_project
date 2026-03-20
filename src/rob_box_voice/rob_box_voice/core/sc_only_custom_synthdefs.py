"""Helpers for registering repository-owned SC-only custom synthdefs."""

from __future__ import annotations

from typing import Any


CUSTOM_SC_ONLY_SYNTH_NAMES = ("warmpad", "retrobass", "supersawlead", "imperialbrass", "marchstrings")


def register_sc_only_custom_synthdefs(runtime_module: Any, runtime_context: dict[str, Any]) -> tuple[str, ...]:
    """Expose repository-owned custom synthdefs in the Renardo runtime context.

    The SC `.scd` sources are preloaded separately by `sclang` startup. This helper
    only creates matching Python-side `SynthDef(name)` wrappers so generated FoxDot
    code can use direct syntax like `p1 >> warmpad(...)`.
    """

    synthdef_factory = runtime_context.get("SynthDef") or getattr(runtime_module, "SynthDef", None)
    if synthdef_factory is None:
        raise ValueError("Renardo runtime does not expose SynthDef factory")

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