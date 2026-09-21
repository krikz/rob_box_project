"""Helpers for validating the FoxDot / SuperCollider music runtime."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
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
# 🔴 FIX (живой инцидент 21.09.2026, Vision Pi): foxdot_init.sc поменял
# формат подтверждения ещё в live-фиксе 30.08 ("SynthDef preload ok: X" →
# "SynthDef in scsynth: X", см. docker/vision/voice_assistant/foxdot_init.sc
# :159-179 — старая строка печаталась сразу после path.load(), то есть
# подтверждала лишь то, что sclang СКОМПИЛИРОВАЛ .scd, а не что scsynth его
# принял; Server.sync дал правдивое "ok", но заодно сменил текст лога).
# Этот regex остался на старом тексте — 21.09.2026 валидатор рапортовал
# "Missing critical SynthDefs: ..." на ВСЕ 11 критичных синтов, хотя
# /tmp/sclang.log содержал ровно "SynthDef in scsynth: <name>" для каждого
# из них (подтверждено grep -x на роботе). Тесты были зелёными, потому что
# гоняли старую строку, которую sclang больше не печатает — см.
# test_classify_sclang_log_matches_actual_foxdot_init_log_format ниже.
# Оставляем ОБА варианта в одном regex — старый формат дёшево сохранить
# для обратной совместимости (например, для логов, снятых до 30.08).
_LOADED_SYNTH_RE = re.compile(
    r"SynthDef\s+(?:preload\s+ok|in\s+scsynth):\s*([A-Za-z0-9_]+)",
    re.IGNORECASE,
)
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


def missing_log_hint(status: MusicStackStatus, log_path: str | Path) -> str | None:
    """Return the "Log file not found" operator hint, or ``None``.

    Issue #2716 (RAW from the Vision Pi, 22.09.2026): the sclang runtime was
    healthy — ``/tmp/sclang.log`` existed and was read fine, 11 critical
    SynthDefs simply had not confirmed into scsynth yet (or were reported
    missing) — yet ``validate_music_stack.py`` printed ``Log file not
    found: /tmp/sclang.log`` right below the report. The old condition in
    that script's ``main()`` was ``not status.is_healthy and not
    status.fatal_errors``: true for ANY degraded reason that isn't a fatal
    sclang error, not just a missing file, so it fired even when the log
    was right there with useful content. The operator followed the hint,
    found the file present and non-empty, and had no idea the message was
    simply wrong.

    This helper makes the check honest: the hint only fires when the log
    file is actually absent from disk. ``format_music_stack_report`` already
    surfaces the "Missing sclang log file: ..." fatal error in that case
    (see :func:`load_sclang_health`); this is the short, operator-facing
    echo of the same fact, not an independent diagnosis.
    """

    if status.is_healthy:
        return None
    if Path(log_path).exists():
        return None
    return f"Log file not found: {log_path}"


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


# ---------------------------------------------------------------------------
# Filesystem-backed helper for downstream consumers (MusicManager, mcp_server)
# ---------------------------------------------------------------------------


def load_sclang_health(
    log_path: str | Path | None = None,
    critical_synths: list[str] | None = None,
) -> MusicStackStatus:
    """Read the sclang startup log from disk and classify it.

    Used by :class:`MusicManager` (rob_box_mcp_tools) at boot time to detect
    upstream-renardo syntax errors and downgrade the music subsystem into a
    hard-fail mode where ``execute_music_code`` / ``set_vibe_preset`` return
    a clear "music unavailable" error rather than letting the LLM fight a
    broken SuperCollider.

    Resolution order for the log path:

    1. ``log_path`` argument if provided.
    2. ``SCLANG_LOG_PATH`` environment variable (set by
       ``start_voice_assistant.sh`` when sclang starts).
    3. ``/tmp/sclang.log`` — the conventional location.

    Args:
        log_path: Override for the log file location.
        critical_synths: Critical SynthDefs that must appear in the log
            (e.g. ``["strings", "wobblebass", ...]``). Empty list means
            don't require any specific SynthDef by name.

    Returns:
        ``MusicStackStatus`` with ``is_healthy=False`` when the log is
        missing, unreadable, or contains fatal sclang errors.
    """

    resolved = (
        Path(log_path)
        if log_path is not None
        else Path(os.environ.get("SCLANG_LOG_PATH", "/tmp/sclang.log"))
    )

    if not resolved.exists():
        return MusicStackStatus(
            is_healthy=False,
            oscdef_registered=False,
            missing_synths=tuple(critical_synths or ()),
            fatal_errors=(f"Missing sclang log file: {resolved}",),
        )

    try:
        log_text = resolved.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return MusicStackStatus(
            is_healthy=False,
            oscdef_registered=False,
            missing_synths=tuple(critical_synths or ()),
            fatal_errors=(f"Cannot read sclang log {resolved}: {exc}",),
        )

    return classify_sclang_log(log_text, critical_synths=list(critical_synths or []))