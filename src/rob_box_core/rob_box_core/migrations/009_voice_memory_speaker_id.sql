-- ============================================================================
-- Migration: 009_voice_memory_speaker_id.sql
-- Purpose:   Issue #1770 — per-speaker personal memory.
--            Add ``speaker_id`` column to ``voice_facts`` and ``voice_turns``
--            so memory tools can scope a result to a single voice user.
--            Without it, two registered users (e.g. Денчик and Саша) share
--            the same fact pool → "что ты знаешь обо мне" / "о чём говорили
--            неделю назад" returns the *other* user's facts.
--
-- Scope:
--   - voice_facts.speaker_id   — TEXT, NULL for global / pre-migration facts
--   - voice_turns.speaker_id   — TEXT, NULL for global / pre-migration turns
--
-- Why TEXT (not INTEGER FK):
--   The ``speakers`` table lives in a different database
--   (``/data/speakers.db``) managed by ``speaker_id_node``. Cross-database
--   foreign keys are not supported by SQLite, so we store the textual
--   ``speaker_id`` and validate it in the application layer
--   (VoiceMemory.save_fact / save_turn / search) when one is provided.
--
-- Backfill strategy:
--   Old rows (inserted before this migration) have no reliable link to a
--   speaker — ``voice_turns.session_id`` is a free-form per-boot tag, not
--   tied to ``speakers.speaker_id``. We keep them with ``speaker_id = NULL``
--   so they are visible to everyone (treated as "global"). New facts/turns
--   are always saved with a concrete ``speaker_id`` from the live session.
--
-- Index strategy:
--   We filter by ``speaker_id`` on every memory_context / memory_search
--   call, so a covering index speeds it up. Composite (speaker_id, …)
--   indexes serve the dominant queries:
--     - facts: WHERE speaker_id = ? ORDER BY updated_at DESC
--     - turns: WHERE speaker_id = ? ORDER BY timestamp DESC
--
-- Version: 9  (follows 008_downgrade_github_presets.sql)
-- ============================================================================

-- ----------------------------------------------------------------------------
-- voice_facts — long-lived user preferences and facts
-- ----------------------------------------------------------------------------
ALTER TABLE voice_facts ADD COLUMN speaker_id TEXT;

-- Dominant query: filter + sort by updated_at DESC for get_facts(speaker_id=…).
CREATE INDEX IF NOT EXISTS idx_vf_speaker_updated
    ON voice_facts(speaker_id, updated_at DESC);

-- Backfill is intentionally a no-op: pre-migration facts have no reliable
-- link to a speaker (the old schema stored facts globally). New facts will
-- always be saved with the current speaker_id.

-- ----------------------------------------------------------------------------
-- voice_turns — per-session conversation history
-- ----------------------------------------------------------------------------
ALTER TABLE voice_turns ADD COLUMN speaker_id TEXT;

-- Dominant query: filter + sort by timestamp DESC for get_context() and
-- load_recent_turns(speaker_id=…) excluding current session.
CREATE INDEX IF NOT EXISTS idx_vt_speaker_timestamp
    ON voice_turns(speaker_id, timestamp DESC);

-- FTS5 index (``voice_turns_fts``) keeps working without changes — it only
-- mirrors the ``content`` column. Filter by ``speaker_id`` happens after
-- the FTS match by joining on ``voice_turns.id``.
