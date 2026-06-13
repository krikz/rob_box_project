-- ============================================================================
-- Migration: 007_known_melodies.sql
-- Purpose:   Move all known melodies from inline prompt code to TrackLibrary.
--            Includes melodies previously hardcoded in music_skill_prompt.txt
--            plus new presets converted from Strudel live coding examples.
--
-- Version: 7  (follows 006_music_presets.sql)
-- ============================================================================

INSERT OR IGNORE INTO music_tracks
    (name, title, code, description, tags, rating, type, created_at, updated_at)
VALUES

-- === MELODIES FROM PROMPT (moved from inline code) ===

-- 8. Happy Birthday (C major, 100 BPM, pianovel)
('happy_birthday', 'Happy Birthday', 'Clock.clear(); Clock.bpm = 100
Root.default = "C"; Scale.default = "major"
p1 >> pianovel(midinote=[60,60,62,60,65,64,60,60,62,60,67,65,60,60,72,69,65,64,62,70,70,69,65,67,65],
               dur=[0.75,0.25,1,1,1,2,0.75,0.25,1,1,1,2,0.75,0.25,1,1,1,1,1,0.75,0.25,1,1,1,2],
               amp=0.6, room=0.3)',
'Happy Birthday melody, pianovel, C major, 100 BPM', '["preset","melody","classic","birthday"]', 5, 'preset', datetime('now'), datetime('now')),

-- 9. Jingle Bells (C major, 120 BPM, bell)
('jingle_bells', 'Jingle Bells', 'Clock.clear(); Clock.bpm = 120
Root.default = "C"; Scale.default = "major"
p1 >> bell(midinote=[64,64,64,64,64,64,64,67,60,62,64,65,65,65,65,65,64,64,64,64,62,62,64,62,67],
           dur=[0.5,0.5,1,0.5,0.5,1,0.5,0.5,1,0.5,2,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.25,0.25,0.5,0.5,0.5,0.5,2],
           amp=0.6, room=0.3)',
'Jingle Bells chorus, bell synth, C major, 120 BPM', '["preset","melody","classic","christmas"]', 5, 'preset', datetime('now'), datetime('now')),

-- 10. Smoke on the Water (D minor, 112 BPM, rave)
('smoke_on_water', 'Smoke on the Water', 'Clock.clear(); Clock.bpm = 112
Root.default = "D"; Scale.default = "minor"
p1 >> rave(midinote=[55,58,60,55,58,61,60,55,58,60,58,55],
           dur=[1,1,1.5,1,1,0.5,1.5,1,1,1.5,1,2],
           amp=0.6, sus=0.9)',
'Smoke on the Water riff, rave synth, D minor, 112 BPM', '["preset","melody","rock","riff"]', 5, 'preset', datetime('now'), datetime('now')),

-- 11. Fur Elise (A minor, 80 BPM, pianovel)
('fur_elise', 'Fur Elise', 'Clock.clear(); Clock.bpm = 80
Root.default = "A"; Scale.default = "minor"
p1 >> pianovel(midinote=[76,75,76,75,76,71,74,72,69,60,64,69,71,64,68,71,72,64,76,75,76,75,76,71,74,72,69],
               dur=[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,1,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,1],
               amp=0.5, room=0.4, sus=0.8)',
'Fur Elise intro, pianovel, A minor, 80 BPM', '["preset","melody","classical","piano"]', 5, 'preset', datetime('now'), datetime('now')),

-- 12. Katyusha (G major, 116 BPM, brass)
('katyusha', 'Katyusha', 'Clock.clear(); Clock.bpm = 116
Root.default = "G"; Scale.default = "major"
p1 >> brass(midinote=[67,71,74,72,71,67,71,74,72,71,69,67,66,67,69,71,72,71,69,67,66,62,67],
            dur=[1,0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5,1,1,2],
            amp=0.6, room=0.2)',
'Katyusha verse, brass synth, G major, 116 BPM', '["preset","melody","folk","russian"]', 5, 'preset', datetime('now'), datetime('now')),

-- 13. Nokia Tune (A major, 120 BPM, blip)
('nokia_tune', 'Nokia Tune', 'Clock.clear(); Clock.bpm = 120
Root.default = "A"; Scale.default = "major"
p1 >> blip(midinote=[76,74,66,68,73,71,64,66,71,69,62,64,69],
           dur=[0.5,0.5,1,1,0.5,0.5,1,1,0.5,0.5,1,1,2],
           amp=0.55, room=0.2)',
'Nokia ringtone, blip synth, A major, 120 BPM', '["preset","melody","classic","ringtone"]', 5, 'preset', datetime('now'), datetime('now')),

-- 14. Kalinka (C major, 130 BPM, brass)
('kalinka', 'Kalinka', 'Clock.clear(); Clock.bpm = 130
Root.default = "C"; Scale.default = "major"
p1 >> brass(midinote=[64,64,65,67,69,67,65,64,64,65,67,69,67,65,64,62,64,65,67,69,71,69,67,65,64],
            dur=[0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5,0.5,0.5,1,0.5,0.5,1,0.5,0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,2],
            amp=0.65, room=0.2)',
'Kalinka chorus, brass synth, C major, 130 BPM', '["preset","melody","folk","russian"]', 5, 'preset', datetime('now'), datetime('now')),

-- 15. Stranger Things (C major, 83 BPM, retrobass+strangerarp)
('stranger_things', 'Stranger Things Theme', 'Clock.clear(); Clock.bpm = 83
Root.default = "C"; Scale.default = "major"
d1 >> play("xx..", dur=0.25, sample=6, amp=0.32,
           amplify=var([0,1,0],[4,40,4]))
p1 >> bass([0,2,4,6,7,6,4,2], oct=4, dur=0.25, amp=0.28,
                lpf=500, amplify=var([0,1,0],[4,38,6]))
p2 >> varsaw([0,2,4,6,7,6,4,2], oct=3, dur=0.25, amp=0.24,
                  lpf=linvar([100,1000],10), room=0.28,
                  amplify=var([0,1,0],[4,38,6]))
p3 >> pads([0,2,4], oct=5, dur=8, sus=8, amp=0.16, lpf=900,
                       amplify=var([1,1,0],[4,38,6]))',
'Stranger Things intro, retrobass+strangerarp, C major, 83 BPM', '["preset","melody","synth","series"]', 5, 'preset', datetime('now'), datetime('now')),

-- === STRUDEL CONVERSIONS (from eefano/strudel-songs-collection) ===

-- 16. Pump Up The Jam (C minor, 125 BPM, saw+bass)
('pump_up_the_jam', 'Pump Up The Jam', 'Clock.clear(); Clock.bpm = 125
Root.default = "C"; Scale.default = "minor"
d1 >> play("X.X.X.X.", sample=5, amp=0.25)
d2 >> play("--.-", dur=1, sample=4, amp=0.12, delay=PWhite(-0.08, 0.08))
p1 >> saw([0,0,3,5,3,0,-2,0], dur=0.5, oct=3, amp=0.35, attack=0.02, decay=0.08, sus=0.05)
p2 >> bass([0,0,-2,3], dur=2, oct=2, amp=0.3)',
'Pump Up The Jam, saw lead + bass, C minor, 125 BPM', '["preset","dance","electronic","90s"]', 5, 'preset', datetime('now'), datetime('now')),

-- 17. Rhythm of the Night (Ab major, 128 BPM, varsaw+strings)
('rhythm_of_the_night', 'Rhythm of the Night', 'Clock.clear(); Clock.bpm = 128
Root.default = "Ab"; Scale.default = "major"
d1 >> play("X..X..o.", sample=2, amp=0.2)
d2 >> play("--.-", dur=1, sample=4, amp=0.1, delay=PWhite(-0.08, 0.08))
p1 >> varsaw([0,4,5,3,2,0], dur=0.75, oct=5, amp=0.4, room=0.3)
p2 >> dub([0,4,5,3], dur=2, oct=3, amp=0.3)
p3 >> strings(dur=4, amp=0.12, sus=4, room=0.5).follow(p1) + (0,2,4)',
'Rhythm of the Night, varsaw lead + strings, Ab major, 128 BPM', '["preset","dance","italo","90s"]', 5, 'preset', datetime('now'), datetime('now')),

-- 18. Tarantella (A minor, 140 BPM, clarinet+tuba)
('tarantella', 'Tarantella', 'Clock.clear(); Clock.bpm = 140
Root.default = "A"; Scale.default = "minor"
d1 >> play("X.oX.o..", sample=1, amp=0.18)
p1 >> karp([0,2,4,5,4,2,0,7,5,4,2,0], dur=0.25, oct=5, amp=0.5, room=0.15)
p2 >> dub([0,0,-3,0], dur=1, oct=2, amp=0.35)
p3 >> faim(dur=4, amp=0.1, sus=4, room=0.6).follow(p1) + (0,2,4)',
'Tarantella folk dance, karp clarinet + dub tuba, A minor, 140 BPM', '["preset","folk","italian","dance"]', 5, 'preset', datetime('now'), datetime('now'));
