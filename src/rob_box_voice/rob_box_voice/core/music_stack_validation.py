"""Helpers for validating the FoxDot / SuperCollider music runtime."""

from __future__ import annotations

from dataclasses import dataclass
import re


_PLUGIN_UGENS = {
    "MoogVCF",
    "DFM1",
    "Squiz",
    "Decimator",
    "CrossoverDistortion",
    "RLPFD",
    "Henon2DN",
    "Latoocarfian2DN",
    "Gendy4",
    "MoogLadder",
}
_PLUGIN_UGEN_RE = re.compile(
    r"\b(" + "|".join(sorted(_PLUGIN_UGENS)) + r")\s*\.\s*(?:ar|kr)\b",
    re.IGNORECASE,
)
_MERGE_CONFLICT_START_RE = re.compile(r"^<{7}(?:\s|$)")
_MERGE_CONFLICT_MID_RE = re.compile(r"^={7}$")
_MERGE_CONFLICT_END_RE = re.compile(r"^>{7}(?:\s|$)")
_MISSING_SYNTH_RE = re.compile(r"SynthDef\s+([A-Za-z0-9_]+)\s+not found", re.IGNORECASE)
_LOADED_SYNTH_RE = re.compile(r"SynthDef\s+preload\s+ok:\s*([A-Za-z0-9_]+)", re.IGNORECASE)
_FATAL_LOG_PATTERNS = (
    "syntax error",
    "Class not defined",
    "Command line parse failed",
)


@dataclass(frozen=True)
class MusicStackStatus:
    """Result of validating the sclang startup/runtime log."""

    is_healthy: bool
    oscdef_registered: bool
    missing_synths: tuple[str, ...]
    fatal_errors: tuple[str, ...]


def format_music_stack_report(status: MusicStackStatus) -> str:
    """Render a concise human-readable startup health report."""

    missing = ", ".join(status.missing_synths) if status.missing_synths else "none"
    lines = [
        "Music stack healthy" if status.is_healthy else "Music stack degraded",
        f"OSCdef ready: {'yes' if status.oscdef_registered else 'no'}",
        f"Missing critical SynthDefs: {missing}",
    ]
    if status.fatal_errors:
        lines.append("Fatal errors:")
        lines.extend(f"- {error}" for error in status.fatal_errors)
    return "\n".join(lines)


def contains_merge_conflict_markers(content: str) -> bool:
    """Return True when .scd content still contains git conflict markers."""

    lines = (line.strip() for line in content.splitlines())
    saw_start = False
    saw_mid = False
    for line in lines:
        if _MERGE_CONFLICT_START_RE.match(line):
            saw_start = True
            continue
        if saw_start and _MERGE_CONFLICT_MID_RE.match(line):
            saw_mid = True
            continue
        if saw_start and saw_mid and _MERGE_CONFLICT_END_RE.match(line):
            return True
    return False


def is_plugin_dependent_synthdef(synthdef_source: str) -> bool:
    """Return True when SynthDef source depends on known SC3 plugin UGens."""

    return _PLUGIN_UGEN_RE.search(synthdef_source) is not None


def classify_sclang_log(log_text: str, critical_synths: list[str]) -> MusicStackStatus:
    """Classify sclang log output into healthy or degraded runtime state."""

    lines = [line.strip() for line in log_text.splitlines() if line.strip()]
    normalized_patterns = tuple(pattern.lower() for pattern in _FATAL_LOG_PATTERNS)
    critical_lookup = {name.lower(): name for name in critical_synths}
    fatal_errors = [line for line in lines if any(pattern in line.lower() for pattern in normalized_patterns)]
    loaded_synths = set()
    for line in lines:
        loaded_match = _LOADED_SYNTH_RE.search(line)
        if loaded_match:
            loaded_synths.add(loaded_match.group(1).lower())

        match = _MISSING_SYNTH_RE.search(line)
        if not match:
            continue
        synth_name = match.group(1).lower()
        loaded_synths.discard(synth_name)

    missing_synths = []
    for synth_name in critical_synths:
        if synth_name.lower() not in loaded_synths:
            missing_synths.append(synth_name)

    oscdef_registered = any(
        "foxdot" in line.lower()
        and "oscdef" in line.lower()
        and ("registered" in line.lower() or "ready" in line.lower())
        for line in lines
    )
    is_healthy = oscdef_registered and not fatal_errors and not missing_synths
    return MusicStackStatus(
        is_healthy=is_healthy,
        oscdef_registered=oscdef_registered,
        missing_synths=tuple(missing_synths),
        fatal_errors=tuple(fatal_errors),
    )