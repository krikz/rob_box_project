-- ============================================================================
-- Migration: 004_music_library.sql
-- Purpose:   Persistent music track library for Rob Box robot.
--            Stores Renardo/FoxDot track code, metadata, ratings and play counts.
--
-- Tables:
--   music_tracks  — each saved track (code + metadata)
--
-- Stored in the same SQLite file as voice_memory (VOICE_MEMORY_DB_PATH).
--
-- Version: 4  (follows 003_waypoints.sql)
-- ============================================================================

CREATE TABLE IF NOT EXISTS music_tracks (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    name        TEXT    NOT NULL UNIQUE,            -- slug identifier (e.g. "csm_132_full_track")
    title       TEXT    NOT NULL DEFAULT '',        -- human-readable title
    code        TEXT    NOT NULL,                   -- Renardo/FoxDot Python code
    description TEXT    NOT NULL DEFAULT '',        -- description of mood, structure, features
    tags        TEXT    NOT NULL DEFAULT '[]',      -- JSON array of string tags
    rating      INTEGER NOT NULL DEFAULT 0          -- 0-5 stars
                        CHECK (rating BETWEEN 0 AND 5),
    notes       TEXT    NOT NULL DEFAULT '',        -- personal notes
    play_count  INTEGER NOT NULL DEFAULT 0,
    created_at  TEXT    NOT NULL,                   -- ISO 8601
    updated_at  TEXT    NOT NULL                    -- ISO 8601
);

CREATE INDEX IF NOT EXISTS idx_music_tracks_rating ON music_tracks(rating);
CREATE INDEX IF NOT EXISTS idx_music_tracks_name   ON music_tracks(name);

-- ----------------------------------------------------------------------------
-- Bootstrap: first robot-authored track (2026-03-06)
-- Rob Box composed this autonomously when asked what it wanted to do.
-- C# minor, 132 BPM. Intro → Verse → Chorus → Bridge → Chorus2 → Outro.
-- ----------------------------------------------------------------------------
INSERT OR IGNORE INTO music_tracks
    (name, title, code, description, tags, rating, notes, play_count, created_at, updated_at)
VALUES (
    'csm_132_full_track',
    'C# Minor Full Track (первый авторский)',
    'Clock.bpm = 132
Scale.default = "minor"
Root.default = "C#"

# Intro - атмосферное начало
def intro():
    d1 >> play("X...", sample=1, amp=0.2, room=0.3)
    p1 >> pads([0,4,5,3], dur=8, amp=0.15, room=0.6, sus=8)
    p2 >> space([0], dur=16, amp=0.1, room=0.8, sus=16)
    Clock.future(16, verse)

# Verse - появляется бас и мелодия
def verse():
    d1 >> play("X..X.o..", sample=1, amp=0.25, room=0.2)
    p1 >> pads([0,4,5,3], dur=4, amp=0.2, room=0.5)
    p2 >> dub([0,-2,0,-3], dur=2, oct=3, amp=0.3, room=0.3)
    p3 >> blip([0,2,4,7,4,2,0,-2], dur=0.5, amp=0.5, room=0.2)
    Clock.future(32, chorus)

# Chorus - максимальная энергия
def chorus():
    d1 >> play("X.o.X.o.", sample=1, amp=0.3, room=0.2)
    d2 >> play("--.-", sample=3, amp=0.15, room=0.2)
    p1 >> pads([0,4,7,4], dur=2, amp=0.25, room=0.4)
    p2 >> dub([0,-2,3,-1], dur=1, oct=3, amp=0.35, room=0.3)
    p3 >> blip([0,4,7,4,0,2,5,3], dur=0.25, amp=0.6, room=0.2)
    p4 >> strings((0,2,4), dur=8, amp=0.15, room=0.5, sus=8)
    Clock.future(32, bridge)

# Bridge - переломный момент
def bridge():
    d1 >> play("X...X...", sample=1, amp=0.2, room=0.3)
    d2.stop()
    p1 >> pads([0,4,5,3], dur=4, amp=0.3, room=0.6)
    p2 >> dub([0,-2], dur=4, oct=3, amp=0.25, room=0.4)
    p3.stop()
    p4.stop()
    Clock.future(16, chorus2)

# Final chorus - с усилением
def chorus2():
    d1 >> play("X.o.X.o.", sample=1, amp=0.35, room=0.2)
    d2 >> play("--.-", sample=3, amp=0.2, room=0.2)
    p1 >> pads([0,4,7,4], dur=2, amp=0.3, room=0.4)
    p2 >> dub([0,-2,3,-1], dur=1, oct=3, amp=0.4, room=0.3)
    p3 >> blip([0,4,7,4,0,2,5,3], dur=0.25, amp=0.65, room=0.2)
    p4 >> strings((0,2,4), dur=8, amp=0.2, room=0.5, sus=8)
    Clock.future(32, outro)

# Outro - плавное завершение
def outro():
    d1 >> play("X...", sample=1, amp=0.15, room=0.3)
    d2.stop()
    p1 >> pads([0,4,5,3], dur=8, amp=0.2, room=0.6, sus=8)
    p2.stop()
    p3.stop()
    p4.stop()
    Clock.future(16, lambda: Clock.clear())

intro()',
    'Полноструктурная вещь: Intro → Verse → Chorus → Bridge → Chorus2 → Outro. C# minor, 132 BPM.',
    '["full_track","minor","132bpm","C#","atmospheric","robot_authored"]',
    5,
    'Робот сам захотел сделать что-то творческое и сочинил этот трек когда его спросили, что он хочет сделать.',
    0,
    '2026-03-06T00:00:00+00:00',
    '2026-03-06T00:00:00+00:00'
);
