-- ============================================================================
-- Migration: 007_generated_music_library.sql
-- Purpose:   Persistent library for AI-generated MP3 tracks (MiniMax Music
--            API). Separate from music_tracks (v4) which stores Renardo
--            code, and from music_github_presets (v6) which stores curated
--            examples.
--
-- Tables:
--   generated_tracks      — one row per generated MP3 file on /data volume
--   generated_tracks_fts  — FTS5 virtual table over prompt/lyrics/name/tags
--
-- Triggers keep FTS5 in sync with the main table.
--
-- Version: 7  (follows 006_music_github_presets.sql)
-- ============================================================================

CREATE TABLE IF NOT EXISTS generated_tracks (
    track_id    TEXT    PRIMARY KEY,        -- UUID4 hex
    name        TEXT    NOT NULL DEFAULT '',-- human-readable title (user-set)
    prompt      TEXT    NOT NULL DEFAULT '',-- MiniMax "prompt" field (style/mood)
    lyrics      TEXT    NOT NULL DEFAULT '',-- MiniMax "lyrics" field (or "[Instrumental]")
    model       TEXT    NOT NULL DEFAULT '',-- e.g. "music-3.0"
    file_path   TEXT    NOT NULL DEFAULT '',-- absolute path to track.mp3 on /data
    duration_s  REAL    NOT NULL DEFAULT 0, -- decoded duration in seconds
    file_size   INTEGER NOT NULL DEFAULT 0, -- bytes on disk
    mood        TEXT    NOT NULL DEFAULT '',-- e.g. "romantic", "energetic"
    genre       TEXT    NOT NULL DEFAULT '',-- e.g. "jazz", "rock", "ambient"
    lang        TEXT    NOT NULL DEFAULT '',-- e.g. "ru", "en", "zh"
    tags        TEXT    NOT NULL DEFAULT '[]', -- JSON array of strings
    play_count  INTEGER NOT NULL DEFAULT 0, -- bumped on each play_from_library call
    generated_at REAL   NOT NULL,           -- unix epoch (set on first save)
    updated_at   REAL   NOT NULL            -- unix epoch (set on every UPDATE)
);

CREATE INDEX IF NOT EXISTS idx_gt_generated_at ON generated_tracks(generated_at DESC);
CREATE INDEX IF NOT EXISTS idx_gt_play_count   ON generated_tracks(play_count DESC);
CREATE INDEX IF NOT EXISTS idx_gt_mood         ON generated_tracks(mood);
CREATE INDEX IF NOT EXISTS idx_gt_genre        ON generated_tracks(genre);

-- FTS5 over the searchable text fields.  ``unicode61`` gives us decent
-- Russian / Chinese tokenization without external dependencies.
CREATE VIRTUAL TABLE IF NOT EXISTS generated_tracks_fts USING fts5(
    prompt, lyrics, name, mood, genre, tags,
    content = 'generated_tracks',
    content_rowid = 'rowid',
    tokenize = 'unicode61'
);

-- Keep FTS5 in sync with the main table
CREATE TRIGGER IF NOT EXISTS generated_tracks_ai AFTER INSERT ON generated_tracks BEGIN
    INSERT INTO generated_tracks_fts(rowid, prompt, lyrics, name, mood, genre, tags)
    VALUES (new.rowid, new.prompt, new.lyrics, new.name, new.mood, new.genre, new.tags);
END;

CREATE TRIGGER IF NOT EXISTS generated_tracks_au AFTER UPDATE ON generated_tracks BEGIN
    INSERT INTO generated_tracks_fts(generated_tracks_fts, rowid, prompt, lyrics, name, mood, genre, tags)
    VALUES ('delete', old.rowid, old.prompt, old.lyrics, old.name, old.mood, old.genre, old.tags);
    INSERT INTO generated_tracks_fts(rowid, prompt, lyrics, name, mood, genre, tags)
    VALUES (new.rowid, new.prompt, new.lyrics, new.name, new.mood, new.genre, new.tags);
END;

CREATE TRIGGER IF NOT EXISTS generated_tracks_ad AFTER DELETE ON generated_tracks BEGIN
    INSERT INTO generated_tracks_fts(generated_tracks_fts, rowid, prompt, lyrics, name, mood, genre, tags)
    VALUES ('delete', old.rowid, old.prompt, old.lyrics, old.name, old.mood, old.genre, old.tags);
END;
