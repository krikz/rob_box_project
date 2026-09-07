"""test_voice_db_path_consistency.py — ADR-0055 Phase 2 blocker regression guard.

Issue #2000 Phase 2 asked to point every SQLite-backed store MCP tools own
(``WaypointStore``, the legacy ``VoiceMemory`` fallback, ``FAQStore``,
``TrackLibrary``) at the same file the dialogue node uses
(``/data/harness_voice.db``). That merge was investigated for this card and
rejected: ``SQLiteVoiceMemory`` (harness, ``rob_box_harness/memory/sqlite_voice.py``)
already defines its own ``waypoints`` (``name`` TEXT PRIMARY KEY) and
``faq_items`` (``created_at``, no FTS5) tables in ``harness_voice.db`` — see
the ``sqlite_db_path`` comment in
``docker/vision/config/voice_assistant/dialogue_node.yaml`` and
``docs/adr/0055-voice-memory-db-unify-with-harness.md`` (§1.3, §3). Those
schemas are incompatible with ``WaypointStore``'s ``waypoints`` (``id`` PK,
``map_id`` NOT NULL FK -> ``maps``) and the legacy ``FAQStore``'s
``faq_items`` (``indexed_at``, FTS5 triggers). ``CREATE TABLE IF NOT EXISTS``
would silently keep whichever schema got created first, and every
INSERT/SELECT issued by the other store would then fail with "no such
column".

Until that reconciliation happens (a real schema-mapping adapter, not a
path swap), ``WaypointStore`` / ``FAQStore`` / ``TrackLibrary`` / the legacy
``VoiceMemory`` fallback in ``mcp_server.py`` must all keep pointing at the
SAME default file — today that is still ``/data/voice_memory.db`` — so a
future well-intentioned edit does not quietly fork the "one file" invariant
into three.

This test does not instantiate the stores: ``WaypointStore``/``TrackLibrary``
need real filesystem access under ``/data``, and ``mcp_server.py``'s inits
need the optional ``rob_box_voice`` package which may not be installed in
CI. Instead it reads the literal fallback argument passed to
``os.getenv("VOICE_MEMORY_DB_PATH", <default>)`` at each call site and
asserts they agree — a plain, source-level characterization test that fails
loudly the moment one call site drifts from the others.
"""
from __future__ import annotations

import re
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1] / "rob_box_mcp_tools"
_MCP_SERVER = _PKG_ROOT / "mcp_server.py"
_WAYPOINT_STORE = _PKG_ROOT / "waypoint_store.py"
_MUSIC = _PKG_ROOT / "tools" / "music.py"

_PATTERN = re.compile(r'os\.getenv\(\s*"VOICE_MEMORY_DB_PATH"\s*,\s*"([^"]+)"\s*\)')


def _defaults(path: Path) -> list[str]:
    assert path.exists(), f"expected file not found: {path}"
    return _PATTERN.findall(path.read_text(encoding="utf-8"))


def test_all_voice_memory_db_path_call_sites_agree_on_one_default():
    """WaypointStore, TrackLibrary and mcp_server.py's legacy call sites
    must all fall back to the exact same literal path.

    mcp_server.py has three ``VOICE_MEMORY_DB_PATH`` reads (the legacy
    VoiceMemory fact-store fallback, and FAQStore); waypoint_store.py and
    tools/music.py each have one. All five must resolve to the same string.
    """
    mcp_server_defaults = _defaults(_MCP_SERVER)
    waypoint_defaults = _defaults(_WAYPOINT_STORE)
    music_defaults = _defaults(_MUSIC)

    assert len(mcp_server_defaults) >= 2, (
        "expected at least 2 VOICE_MEMORY_DB_PATH call sites in mcp_server.py "
        f"(legacy VoiceMemory fallback + FAQStore), found {mcp_server_defaults!r} "
        "— did a call site get renamed or removed?"
    )
    assert waypoint_defaults, "waypoint_store.py: no VOICE_MEMORY_DB_PATH default found"
    assert music_defaults, "tools/music.py: no VOICE_MEMORY_DB_PATH default found"

    all_defaults = set(mcp_server_defaults) | set(waypoint_defaults) | set(music_defaults)
    assert len(all_defaults) == 1, (
        "VOICE_MEMORY_DB_PATH default literal has forked across call sites: "
        f"{sorted(all_defaults)!r}. WaypointStore / FAQStore / TrackLibrary / "
        "the legacy VoiceMemory fallback in mcp_server.py must keep sharing "
        "one default file until the waypoints/faq_items schema collision "
        "with harness_voice.db (ADR-0055) is resolved by an actual adapter, "
        "not by editing one os.getenv default in isolation."
    )


def test_default_is_not_silently_repointed_at_harness_voice_db():
    """Guard against a well-intentioned but unsafe drive-by path swap.

    Repointing any of these defaults at ``/data/harness_voice.db`` without
    first landing a schema-reconciling adapter would make WaypointStore's
    or FAQStore's inserts fail at runtime against SQLiteVoiceMemory's
    pre-existing ``waypoints``/``faq_items`` tables there.
    """
    for path in (_MCP_SERVER, _WAYPOINT_STORE, _MUSIC):
        for default in _defaults(path):
            assert "harness_voice.db" not in default, (
                f"{path.name} now defaults VOICE_MEMORY_DB_PATH to "
                f"{default!r} — this collides with SQLiteVoiceMemory's "
                "waypoints/faq_items schema in harness_voice.db (see "
                "ADR-0055 and docker/vision/config/voice_assistant/"
                "dialogue_node.yaml). Do not flip this default without a "
                "real reconciliation adapter for waypoints/faq_items."
            )
