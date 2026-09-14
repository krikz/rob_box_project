-- ============================================================================
-- Migration: 012_melody_library.sql
-- Purpose:   Seed well-known melodies (played by name) into music_tracks as
--            type='melody'. Each entry is exact Renardo code for a famous
--            tune; lookup_melody resolves name/alias → code so "сыграй
--            кузнечика" plays note-for-note instead of a guessed scale run
--            (issue #1810).
--
-- Target schema: music_tracks(name, title, code, description, tags, rating,
--                type, created_at, updated_at) — the ``type`` column is added
--                by 006_music_github_presets.sql, which runs before this.
--
-- Version: 12  (follows 011_agent_namespace.sql)
-- ============================================================================

INSERT OR IGNORE INTO music_tracks
    (name, title, code, description, tags, rating, type, created_at, updated_at)
VALUES

-- 1. Кузнечик (ступени, C major)
('kuznechik', 'В траве сидел кузнечик',
'Clock.clear()
Clock.bpm = 100
Root.default = "C"
Scale.default = "major"
p1 >> pluck([4,4,2,4,4,2,0,1,2,3,4,4,3,2,1,0], dur=[0.5,0.5,1,0.5,0.5,1,0.25,0.25,0.25,0.25,0.5,0.25,0.25,0.25,0.25,1.5], oct=5, amp=0.5)',
'Детская песенка, хук соль-соль-ми, соль-соль-ми',
'["melody","кузнечик","в траве сидел кузнечик","детская"]', 5, 'melody', datetime('now'), datetime('now')),

-- 2. Ёлочка (ступени, C major)
('yolochka', 'В лесу родилась ёлочка',
'Clock.clear()
Clock.bpm = 92
Root.default = "C"
Scale.default = "major"
p1 >> pianovel([2,1,0,1,2,2,2,1,1,1,2,4,4,3,2,1], dur=[0.5,0.5,0.5,0.5,0.5,0.5,1.5,0.5,0.5,2,0.5,0.5,1,0.5,0.5,1.5], oct=5, amp=0.5, room=0.3)',
'Новогодняя песенка',
'["melody","ёлочка","елочка","в лесу родилась ёлочка","новый год"]', 5, 'melody', datetime('now'), datetime('now')),

-- 3. Чижик-Пыжик (ступени, C major)
('chizhik', 'Чижик-Пыжик',
'Clock.clear()
Clock.bpm = 108
Root.default = "C"
Scale.default = "major"
p1 >> blip([2,0,2,0,3,2,1,4,4,4,5,6,7,7,7], dur=[0.5,0.5,0.5,0.5,0.5,0.5,1,0.5,0.5,0.5,0.5,0.5,0.5,0.5,1.5], oct=5, amp=0.5)',
'Народная песенка',
'["melody","чижик","чижик-пыжик"]', 5, 'melody', datetime('now'), datetime('now')),

-- 4. Собачий вальс (ступени, C majorPentatonic)
('sobachiy_vals', 'Собачий вальс',
'Clock.clear()
Clock.bpm = 116
Root.default = "C"
Scale.default = "majorPentatonic"
p1 >> karp([4,3,0,0,4,3,0], dur=[0.5,0.5,1,1,0.5,0.5,1.5], oct=5, amp=0.45)',
'Простейший пентатонический остинато',
'["melody","собачий вальс","flea waltz"]', 5, 'melody', datetime('now'), datetime('now')),

-- 5. Имперский марш (миди, G minor, brass)
('imperial_march', 'Имперский марш',
'Clock.clear()
Clock.bpm = 108
Root.default = "G"
Scale.default = "minor"
p1 >> brass(midinote=[67,67,67,63,70,67,63,70,67,70,75,74,73,72,71,72,65,66,63,66,70,67,70,76,75,74,70,66,63,70,67], dur=[1,1,1,0.75,0.25,1,0.75,0.25,2,0.5,1,1,0.25,0.25,0.25,1,1,1,1,0.25,1,1,0.25,2,1,1,1,0.75,0.25,1,2], amp=0.5, sus=0.46)
p2 >> marchstrings(midinote=[43,43,43,43,39,39,39,39,43,43,46,46], dur=[1,1,1,1,1,1,1,1,1,1,2,2], amp=0.22)',
'Тема Дарта Вейдера, лид brass, A+A+бридж+B-ответ',
'["melody","имперский марш","imperial march","star wars","дарт вейдер","darth vader","штурмовики"]', 5, 'melody', datetime('now'), datetime('now')),

-- 6. Happy Birthday (миди)
('happy_birthday', 'Happy Birthday',
'Clock.clear()
Clock.bpm = 100
p1 >> pianovel(midinote=[60,60,62,60,65,64,60,60,62,60,67,65,60,60,72,69,65,64,62,70,70,69,65,67,65], dur=[0.75,0.25,1,1,1,2,0.75,0.25,1,1,1,2,0.75,0.25,1,1,1,1,1,0.75,0.25,1,1,1,2], amp=0.6, room=0.3)',
'Поздравительная песенка',
'["melody","happy birthday","с днём рождения","с днем рождения"]', 5, 'melody', datetime('now'), datetime('now')),

-- 7. Jingle Bells (миди, припев)
('jingle_bells', 'Jingle Bells',
'Clock.clear()
Clock.bpm = 120
p1 >> bell(midinote=[64,64,64,64,64,64,64,67,60,62,64,65,65,65,65,65,64,64,64,64,62,62,64,62,67], dur=[0.5,0.5,1,0.5,0.5,1,0.5,0.5,1,0.5,2,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.25,0.25,0.5,0.5,0.5,0.5,2], amp=0.6, room=0.3)',
'Рождественский припев',
'["melody","jingle bells","джингл белс"]', 5, 'melody', datetime('now'), datetime('now')),

-- 8. Ода к радости (Бетховен, миди)
('ode_to_joy', 'Ода к радости',
'Clock.clear()
Clock.bpm = 120
p1 >> pianovel(midinote=[64,64,65,67,67,65,64,62,60,60,62,64,64,62,62], dur=[1,1,1,1,1,1,1,1,1,1,1,1,1,1,2], amp=0.55, room=0.3)',
'Бетховен, 9-я симфония',
'["melody","ода к радости","ode to joy","бетховен","beethoven"]', 5, 'melody', datetime('now'), datetime('now')),

-- 9. Stranger Things (миди, C minor арпеджио)
('stranger_things', 'Stranger Things',
'Clock.clear()
Clock.bpm = 84
Root.default = "C"
Scale.default = "minor"
p1 >> arpy([0,2,4,6,7,6,4,2], dur=[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5], oct=4, amp=0.4, lpf=1200, room=0.3)',
'Очень странные дела, интро-арпеджио',
'["melody","stranger things","очень странные дела","странные дела"]', 5, 'melody', datetime('now'), datetime('now')),

-- 10. В пещере горного короля (Григ, миди, best-effort)
('mountain_king', 'В пещере горного короля',
'Clock.clear()
Clock.bpm = 138
Root.default = "A"
Scale.default = "minor"
p1 >> saw(midinote=[69,71,72,75,76,77,76,74,72,76,69,69], dur=[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,2], amp=0.45, room=0.2)',
'Эдвард Григ, главная тема (best-effort)',
'["melody","в пещере горного короля","григ","grieg","in the hall of the mountain king"]', 5, 'melody', datetime('now'), datetime('now')),

-- 11. Бетховен 5 (мотив судьбы, миди)
('beethoven_5th', 'Бетховен — мотив судьбы',
'Clock.clear()
Clock.bpm = 96
Root.default = "C"
Scale.default = "minor"
p1 >> saw(midinote=[67,67,67,63,65,65,65,62], dur=[0.5,0.5,0.5,2,0.5,0.5,0.5,2], amp=0.5)',
'Симфония №5, та-та-та-тааа',
'["melody","бетховен 5","мотив судьбы","судьба","beethoven 5"]', 5, 'melody', datetime('now'), datetime('now'));
