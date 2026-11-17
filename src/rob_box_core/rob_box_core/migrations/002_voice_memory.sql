-- ============================================================================
-- Migration: 002_voice_memory.sql
-- Purpose:   Cross-session persistent memory for the voice assistant.
--            Creates voice_turns (dialogue history with FTS5 search) and
--            voice_facts (user preferences / important facts).
--
-- Inspired by: github.com/mraza007/echovault (local-first memory for agents)
-- Adapted for: embedded ROS 2 use — no MCP, no markdown, lighter footprint.
--
-- Tables:
--   voice_turns      — every user/assistant exchange, indexed by session & time
--   voice_turns_fts  — FTS5 virtual table for keyword search over content
--   voice_facts      — long-lived user preferences and important facts
--
-- Triggers:
--   voice_turns_ai / voice_turns_au / voice_turns_ad  — keep FTS in sync
--
-- Version: 2  (follows 001_init.sql which owns schema version 1)
-- ============================================================================

-- ----------------------------------------------------------------------------
-- Conversation turns
-- ----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS voice_turns (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT    NOT NULL,          -- unique per robot boot, e.g. "20260219_143022"
    role       TEXT    NOT NULL,          -- 'user' | 'assistant'
    content    TEXT    NOT NULL,
    timestamp  REAL    NOT NULL           -- Unix epoch (float)
);

CREATE INDEX IF NOT EXISTS idx_vt_session   ON voice_turns(session_id);
CREATE INDEX IF NOT EXISTS idx_vt_timestamp ON voice_turns(timestamp);

-- FTS5 index for keyword search over content
CREATE VIRTUAL TABLE IF NOT EXISTS voice_turns_fts USING fts5(
    content,
    content       = voice_turns,
    content_rowid = id,
    tokenize      = 'unicode61'
);

-- Keep FTS in sync: INSERT
CREATE TRIGGER IF NOT EXISTS voice_turns_ai
AFTER INSERT ON voice_turns BEGIN
    INSERT INTO voice_turns_fts(rowid, content) VALUES (new.id, new.content);
END;

-- Keep FTS in sync: UPDATE
CREATE TRIGGER IF NOT EXISTS voice_turns_au
AFTER UPDATE OF content ON voice_turns BEGIN
    INSERT INTO voice_turns_fts(voice_turns_fts, rowid, content)
        VALUES ('delete', old.id, old.content);
    INSERT INTO voice_turns_fts(rowid, content) VALUES (new.id, new.content);
END;

-- Keep FTS in sync: DELETE
CREATE TRIGGER IF NOT EXISTS voice_turns_ad
AFTER DELETE ON voice_turns BEGIN
    INSERT INTO voice_turns_fts(voice_turns_fts, rowid, content)
        VALUES ('delete', old.id, old.content);
END;

-- ----------------------------------------------------------------------------
-- User facts / preferences
-- ----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS voice_facts (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    fact       TEXT    NOT NULL,
    category   TEXT    NOT NULL DEFAULT 'general',   -- 'preference'|'habit'|'name'|'general'
    created_at REAL    NOT NULL,
    updated_at REAL    NOT NULL
);

-- ----------------------------------------------------------------------------
-- Metadata (embedding dimension, config)
-- vec table (voice_turns_vec) is created lazily by VoiceMemory when the
-- embedding dimension is first known. See: voice_memory.py _ensure_vec_table()
-- ----------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS voice_memory_meta (
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);
