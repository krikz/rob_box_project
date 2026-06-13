-- ============================================================================
-- Migration: 006_music_presets.sql
-- Purpose:   Add type column (preset/user) and seed diverse genre presets.
--            Presets are curated read-only tracks that LLM can reference via list_tracks.
--
-- Version: 6  (follows 005_*.sql)
-- ============================================================================

-- Add type column to distinguish presets from user-saved tracks
ALTER TABLE music_tracks ADD COLUMN type TEXT NOT NULL DEFAULT 'user';

-- Create index for type filtering
CREATE INDEX IF NOT EXISTS idx_music_tracks_type ON music_tracks(type);

-- ----------------------------------------------------------------------------
-- Seed presets: 6+ diverse genres, each with unique BPM/root/scale/instruments
-- These are curated arrangements the LLM can load via list_tracks(tag="preset")
-- ----------------------------------------------------------------------------
INSERT OR IGNORE INTO music_tracks
    (name, title, code, description, tags, rating, type, created_at, updated_at)
VALUES

-- 1. Star Wars Imperial March (G minor, 108 BPM, brass+strings)
('star_wars_march', 'Imperial March (Star Wars)', 'Clock.clear(); Clock.bpm = 108
Root.default = "G"; Scale.default = "minor"
d1 >> play("X..X..o.", sample=2, amp=0.18)
d2 >> play("--.-", dur=1, sample=4, amp=0.1, delay=PWhite(-0.08, 0.08))
p1 >> moogbass([0,0,0,-1,-2,0,-1,-2,0], dur=[1,0.5,0.5,1,1,0.5,0.5,1,2], oct=3, amp=0.32)
p2 >> brass([0,0,0,-1,-2,0,-1,-2,0], dur=[1,0.5,0.5,1,1,0.5,0.5,1,2], oct=4, amp=0.38, sus=1.2)
p3 >> strings(dur=4, amp=0.14, sus=4, room=0.5).follow(p1) + (0,2,4)',
'G minor march, brass+strings, 108 BPM, Star Wars theme', '["preset","march","cinematic","star-wars"]', 5, 'preset', datetime('now'), datetime('now')),

-- 2. Jazz Groove (D dorian, 96 BPM, pianovel+strings)
('jazz_groove_96bpm', 'Jazz Groove', 'Clock.clear(); Clock.bpm = 96
Root.default = "D"; Scale.default = "dorian"
d1 >> play("X..X..o.", sample=2, amp=0.18)
d2 >> play("-.--", dur=1, sample=4, amp=0.1, delay=PWhite(-0.1, 0.1))
p1 >> wobblebass([0,0,-2,3], dur=4, oct=3, amp=0.28)
p2 >> pianovel([0,2,4,3,2,0], dur=0.75, oct=5, amp=0.45, room=0.35)
p3 >> strings(dur=6, amp=0.14, sus=6, room=0.6).follow(p1) + (0,2,4)',
'Dorian jazz, pianovel lead, wobblebass, 96 BPM', '["preset","jazz","dorian","smooth"]', 5, 'preset', datetime('now'), datetime('now')),

-- 3. Acid Techno (F# phrygian, 136 BPM, saw+karp)
('acid_techno_136bpm', 'Acid Techno', 'Clock.clear(); Clock.bpm = 136
Root.default = "F#"; Scale.default = "phrygian"
d1 >> play("X...X...", sample=4, amp=0.2)
d2 >> play("--.-", dur=1, sample=PRand([3,4,5]), amp=0.1, delay=PWhite(-0.08, 0.08))
p1 >> saw(PRand([0,1,2,3,4]), dur=0.5, oct=3, amp=0.3, attack=0.02, decay=0.1, sus=0.05, room=0.2)
p2 >> karp(PRand([0,1,2,3]), dur=PRand([0.5,1,2]), oct=2, amp=0.28)',
'Acid techno, PRand pentatonic, saw+karp, 136 BPM', '["preset","techno","acid","dark"]', 5, 'preset', datetime('now'), datetime('now')),

-- 4. Ambient Dreams (A lydian, 88 BPM, sitar+fuzz)
('ambient_dreamy_88bpm', 'Ambient Dreams', 'Clock.clear(); Clock.bpm = 88
Root.default = "A"; Scale.default = "lydian"
d1 >> play("x...x...", sample=1, amp=0.15)
p1 >> fuzz([0,4,7,5], dur=6, oct=2, amp=0.25, room=0.5)
p2 >> sitar([0,4,2,7], dur=1.5, oct=5, amp=0.4, room=0.55)
p3 >> faim(dur=8, amp=0.12, sus=8, room=0.85).follow(p1) + (0,2,4)',
'Lydian ambient, sitar lead, fuzz bass, 88 BPM', '["preset","ambient","dreamy","chill"]', 5, 'preset', datetime('now'), datetime('now')),

-- 5. Hip-Hop Boom Bap (D minorPentatonic, 90 BPM, moogbass+karp)
('hiphop_boom_bap', 'Hip-Hop Boom Bap', 'Clock.clear(); Clock.bpm = 90
Root.default = "D"; Scale.default = "minorPentatonic"
d1 >> play("X..X.o..", sample=2, amp=0.2)
d2 >> play("--.-", dur=1, sample=3, amp=0.1, delay=PRand([0, PWhite(-0.1, 0.1)]))
p1 >> moogbass([0,0,-2,3], dur=2, oct=2, amp=0.35)
p2 >> karp([0,2,4,2,3,1], dur=0.25, oct=6, amp=0.5, room=0.1)',
'Boom bap hip-hop, karp lead, moogbass, 90 BPM', '["preset","hiphop","boom-bap","groove"]', 5, 'preset', datetime('now'), datetime('now')),

-- 6. Folk Fiddle Reel (G mixolydian, 120 BPM, karp+dub)
('folk_fiddle_reel', 'Folk Fiddle Reel', 'Clock.clear(); Clock.bpm = 120
Root.default = "G"; Scale.default = "mixolydian"
d1 >> play("X.oX.o..", sample=1, amp=0.15)
p1 >> karp([0,2,4,2,3,1,3,1], dur=0.25, oct=6, amp=0.5, room=0.1)
p2 >> dub([0,0,4,5], dur=2, oct=3, amp=0.3)
p3 >> strings(dur=4, amp=0.12, sus=4, room=0.4).follow(p1) + (0,2,4)',
'Folk reel, karp fiddle, dub bass, mixolydian, 120 BPM', '["preset","folk","fiddle","upbeat"]', 5, 'preset', datetime('now'), datetime('now')),

-- 7. Electro Funk (E minor, 115 BPM, retrobass+varsaw)
('electro_funk_115bpm', 'Electro Funk', 'Clock.clear(); Clock.bpm = 115
Root.default = "E"; Scale.default = "minor"
d1 >> play("X..X.o..", sample=3, amp=0.2)
d2 >> play("-.--", dur=1, sample=5, amp=0.12, delay=PWhite(-0.06, 0.06))
p1 >> moogbass([0,3,5,7,5,3], dur=0.5, oct=3, amp=0.3)
p2 >> varsaw([0,2,4,7,4,2], dur=0.5, oct=5, amp=0.42, room=0.25)
p3 >> pads(dur=6, amp=0.1, sus=6, room=0.5).follow(p1) + (0,2,4)',
'Electro funk, retrobass+varsaw, minor, 115 BPM', '["preset","funk","electro","groove"]', 5, 'preset', datetime('now'), datetime('now'));
