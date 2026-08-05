-- ============================================================================
-- Migration: 006_music_github_presets.sql
-- Purpose:   Add curated compositions sourced from GitHub FoxDot/Renardo repos
--            (ported from feature/phase-3.2-music-testing → issue #1000).
--            Each preset is user-validated before inclusion.
--
-- Target schema (004_music_library.sql + runtime ALTER TABLE ADD COLUMN type):
--   music_tracks(name, title, code, description, tags, rating, type,
--                notes, play_count, created_at, updated_at)
-- TrackLibrary.__init__ adds the ``type`` column if missing (issue #935).
--
-- Version: 6  (follows 005_faq.sql)
-- ============================================================================

INSERT OR IGNORE INTO music_tracks
    (name, title, code, description, tags, rating, type, created_at, updated_at)
VALUES

-- 1. Club Energy Anthem (harmonic minor, 128 BPM, saw+pluck+bass)
('club_energy_128bpm', 'Club Energy Anthem', 'Clock.clear(); Clock.bpm = 128
Scale.default = Scale.harmonicMinor
Root.default = var([0, 2, -1, -3], 8)
p1 >> saw([0, 4, 7, 11], dur=8, sus=8, chop=4, amp=0.5, lpf=linvar([800, 4000], 32), room=0.6, verb=0.7, pan=linvar([-0.5, 0.5], 16))
p2 >> pluck(P[0, 2, 4, 7].arp([0, 1, 2, 3]).every(6, "mirror"), dur=PDur(3, 8), sus=0.2, amp=0.9, chop=4, echo=0.25, echotime=0.25, lpf=1200, pan=PSine(8))
s1 >> bass(var([0, -1, 0, -3], [4, 4, 4, 4]), dur=1/2, sus=0.4, amp=1.3, lpf=900, drive=0.3)
m1 >> sawbass(P[0, 2, -1, None], dur=2, sus=1.5, amp=0.8, lpf=700, distort=0.3, crush=8)
d1 >> play("x-ox xoxo", dur=1/2, amp=1.2)
h1 >> play("-(-=)-", dur=1/4, amp=0.6, pan=0.4)
c1 >> play("  *   * ", sample=2, dur=1/2, amp=0.9)
fx >> play("V[--]V ", dur=1, amp=0.6).every(8, "stutter", 4, dur=3)',
'Club energy, supersaw+pluck+bass, harmonic minor, 128 BPM', '["preset","club","energy","techno","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 2. Rypop (pop/ambient, 120 BPM, bass+viola+pluck+bell+drums)
('rypop_120bpm', 'Rypop', 'Clock.clear(); Clock.bpm = 120
ch = var([0,5,2,3],[8,4,2,2])
bb >> bass(ch, dur=PSum(3,2), oct=(4,5), amp=.2)
vv >> viola(ch + [[2,0,0],0,[0,2,2],[0,0,2]], dur=ch.dur, oct=(4,5)) + var([(0,2),(2,4)],[28,4])
vv.amp=var([.5,0],[48,16])
A = var([1,0],[16.5,15.5])
pp >> pluck([0,2,4,7,8,2], dur=1/2, sus=1/2, amp=A, pan=-1) + [0,0,[0,0,1]]
be >> bell([2,1,0,2,3], dur=PSum(5,4), oct=(5,6), amp=0.7) + var([0,2],[12,4.5,11.5,4])
bd >> play("(x )(-[-x]-)", sample=2, dur=1/2)
cp >> play("  H ")
sh >> play("s s( s)")
sn >> play(" (   I)I(   [II])")
tm >> play("m  m  m( [ m])")',
'Pop/ambient, bass+viola+pluck+bell, 120 BPM, by e-lie', '["preset","pop","ambient","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 3. Algorave (techno/dark, 150 BPM, sawbass+blip+pads+prophet+drums)
('algorave_150bpm', 'Algorave', 'Clock.clear(); Clock.bpm = 150
Scale.default = "minor"
p1 >> sawbass([0, -2], dur=8, sus=7, amp=0.5)
p2 >> blip(p1.pitch + [0, 2, 4, 6], drive=0.2, room=1, amp=0.4)
p3 >> pads(p1.pitch, dur=4, spin=4, oct=4, amp=0.6, chop=[8, 16], drive=0.3)
p4 >> prophet(p1.pitch + P[0:8], dur=PDur(3, 8), sus=1/4, amp=0.4, drive=0.1).penta().shuffle()
d1 >> play("X O( [ X])")
d2 >> play("----", dur=1/2).every(4, "stutter", 8)
d3 >> play("[^^]  ( [~~]) ")',
'Dark techno algorave, sawbass+blip+pads, 150 BPM, by diegodukao', '["preset","techno","dark","algorave","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 4. Rool (techno/jam, 120 BPM, minimal beat + bass + YEAH sample)
('rool_120bpm', 'Rool Jam', 'Clock.clear(); Clock.bpm = 120
Scale.default = "minor"
Root.default = -2
p1 >> play("x-o-", sample=0, amp=0.5)
p2 >> play("-  -  -  - -  ", sample=4, amp=0.3)
b1 >> play("V ", amp=0.5)
p4 >> play("  *(   YEAH)", sample=8, amp=0.4)
b0 >> bass(P[0].stretch(13)|P[0.5,0.75,P[1,4,P[1,1,1,7]]], delay=0.5, lpf=350, amp=0.4)',
'Techno jam, minimal beat + bass + YEAH sample, 120 BPM, by ekg', '["preset","techno","minimal","jam","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 5. Bargewave8 (dub techno, 105 BPM, dub bass + keys + pads + prophet)
('bargewave_105bpm', 'Bargewave Dub', 'Clock.clear(); Clock.bpm = 105
Scale.default = "minor"
Root.default = -3
d1 >> play("X-o-", sample=4, amp=0.5)
d2 >> play("V ", amp=0.5)
d3 >> play("  o ", sample=3, amp=0.3)
b1 >> dub([0, 2, 5], dur=[8, 4, 4], amp=0.3)
k1 >> keys(P(0, 2, 4) + b1.pitch, dur=0.5, amp=0.3, room=2, lpf=800)
kp >> pads(P(0, 2, 4), oct=3, dur=4, amp=0.4, sus=3.5)
zz >> prophet(P[:2], dur=2, sus=0.5, lpf=600, amp=0.3)',
'Dub techno, dub bass + keys + pads, 105 BPM, by ekg', '["preset","dub","techno","chill","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 6. Kine (spark + dbass, 112 BPM)
('kine_112bpm', 'Kine', 'Clock.clear(); Clock.bpm = 112
Scale.default = "minor"
Root.default = -2
d1 >> play("x-o-", sample=5, amp=0.5)
d2 >> play("  o ", sample=3, amp=0.3)
d3 >> play("V ", amp=0.5)
p1 >> spark(P[0, 2, 5], dur=4, amp=0.4, lpf=1200)
p2 >> dbass(P[0, 2, 4], dur=4, lpf=800, amp=0.4)
k1 >> keys(P(0, 2, 4), dur=0.5, amp=0.3, room=1)',
'Spark + dbass, minor groove, 112 BPM, by ekg', '["preset","minimal","groove","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 7. Bestbanger (dorian, 100 BPM, bass + marimba + keys)
('bestbanger_100bpm', 'Bestbanger', 'Clock.clear(); Clock.bpm = 100
Scale.default = "dorian"
Root.default = 0
d1 >> play("<X ><  + ><[ -&]><  * >", amp=0.5)
d2 >> play("V ", amp=0.5)
dx >> bass(P[8,7,6,4,5,2,2,2,1,1,1,0,0,0,0,0,0], delay=0.5, dur=4, lpf=600, oct=3, sus=1.5, amp=0.35)
k1 >> marimba(P[(0,2,4),(0,3,5),(2,4,6),(4,6,8)], dur=[2,0.5,1,0.5], amp=0.4, lpf=900, room=0.5)
p1 >> keys(P[0:5], dur=4, amp=0.3, room=1)',
'Dorian groove, descending bass + marimba, 100 BPM, by ekg', '["preset","dorian","groove","chill","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 8. Wamorning (blip + keys + donk, 93 BPM)
('wamorning_93bpm', 'Wamorning', 'Clock.clear(); Clock.bpm = 93
Scale.default = "minor"
Root.default = 0
d1 >> play("<  * ><[  +]><* * ><  - >", amp=0.4)
d2 >> play("V ", amp=0.4)
p1 >> blip(P[5,2,2,5,2,2,5,2,2,2,2,2,2,2,2,2,2,2,0,0,0,2,0,0,2,0,2,2], delay=0.25, dur=0.25, oct=5, room=0.5, amp=0.35, lpf=1200)
p2 >> keys(P[5,2,0,2], dur=4, amp=0.3, room=0.5, lpf=1000)
p3 >> donk(var([5, 0, 5, 3], 8), dur=4, delay=var([0, 0.5], [8, 4]), amp=0.3, room=1, oct=5)',
'Blip melody + keys bass + donk accents, 93 BPM, by ekg', '["preset","blip","keys","donk","chill","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 9. Owio (pads + saw bass, 108 BPM)
('owio_108bpm', 'Owio', 'Clock.clear(); Clock.bpm = 108
Scale.default = "minor"
Root.default = 0
d1 >> play("V ", amp=0.45)
d2 >> play("  - -- ", amp=0.2)
p1 >> pads([0, 0, 2, 0, 2, 4, 4, 5, 3], dur=[4, 4, 4, 2, 2, 2, 1, 1, 2], lpf=900, amp=0.3)
p2 >> sawbass(dur=16, lpf=400, sus=15, amp=0.25).follow(p1)
d3 >> play("  o ", sample=3, amp=0.25)',
'Pads + saw bass, chill minor, 108 BPM, by ekg', '["preset","pads","sawbass","chill","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 10. Darwinscouch (saw bass + pads, 109 BPM)
('darwinscouch_109bpm', 'Darwinscouch', 'Clock.clear(); Clock.bpm = 109
Scale.default = "minor"
Root.default = 0
d1 >> play("V ", amp=0.45)
d2 >> play("-- -  -", amp=0.25)
d3 >> play("  o  o ", amp=0.25)
b1 >> sawbass(P[0, 2, 4, 0, 4, 2], dur=4, lpf=600, amp=0.3, sus=1)
p1 >> pads(P[0:4], dur=4, amp=0.25, lpf=900, room=0.5)',
'Saw bass + pads, wild groove, 109 BPM, by ekg', '["preset","sawbass","pads","groove","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 11. Madness (dbass + ambi + pluck, 110 BPM)
('madness_110bpm', 'Madness', 'Clock.clear(); Clock.bpm = 110
Scale.default = "minor"
Root.default = 0
d1 >> play("X-X-", amp=0.4, lpf=200)
d2 >> dbass(P[0,2,3,4,5], lpf=500, dur=0.5, amp=0.3)
b1 >> bass(P[0:6], lpf=500, oct=3, amp=0.25)
a1 >> ambi(P[0,2,4], lpf=1000, sus=0.1, amp=0.2, oct=3)
p1 >> pluck(P[0,2,4,1,2,3,4], lpf=2000, sus=8, amp=0.2)',
'Dbass + ambi + pluck, industrial groove, 110 BPM, by ekg', '["preset","dbass","ambi","pluck","industrial","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 12. Illing (space + keys + bass, 95 BPM)
('illing_95bpm', 'Illing', 'Clock.clear(); Clock.bpm = 95
Scale.default = "minor"
Root.default = -2
d1 >> play("-(Xi)(A*o)", sample=var([1,2,3]), amp=0.4)
d2 >> play("V ", amp=0.4)
b1 >> space(P[0,2,4,0,6]-4, echo=0.5, delay=2, amp=0.2)
q1 >> keys(P(0,2,4)+P[1:6], dur=4, echo=1, amp=0.3, lpf=1000)
b2 >> bass([0,3,4], dur=[4,2,2], amp=0.25, lpf=600)',
'Space ambient + keys + bass, chill, 95 BPM, by ekg', '["preset","space","keys","ambient","chill","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 13. G Karp (karp + keys + bass, 84 BPM)
('g_karp_84bpm', 'G Karp', 'Clock.clear(); Clock.bpm = 84
Scale.default = "minor"
Root.default = 0
d1 >> play("x   o  -", sample=4, amp=0.4)
d2 >> play("x---*---", amp=0.35)
d3 >> play("V ", amp=0.4)
k1 >> karp(P(0,2,5)+P[0,3,4,2], delay=0.5, dur=0.25, amp=0.3, lpf=1500)
p1 >> keys(P[0:4], dur=4, amp=0.25, room=0.5, lpf=1000)
b1 >> bass(P[0,2,3,4], dur=4, amp=0.25, lpf=600)',
'Karp arpeggios + keys + bass, mellow, 84 BPM, from ekg/g', '["preset","karp","keys","mellow","chill","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 14. Proof (marimba + pluck + charm, 80 BPM)
('proof_80bpm', 'Proof', 'Clock.clear(); Clock.bpm = 80
Scale.default = "minor"
Root.default = 0
d1 >> play("x-o-x-o-", amp=0.4)
d2 >> play("V ", amp=0.4)
m1 >> marimba(P[0,1,2,3,4,5,6,7], dur=PRand([1,0.5,0.5]), amp=0.3, lpf=2000)
p1 >> pluck(P[7,6,5,4,3,2,1,0], dur=0.5, amp=0.25, lpf=1500)
c1 >> charm(P[0:4], dur=4, amp=0.2, room=0.5)
b1 >> bass(P[0,2,4,6], dur=4, amp=0.25, lpf=600)',
'Marimba + pluck + charm, mellow chill, 80 BPM, from ekg/proof', '["preset","marimba","pluck","charm","mellow","chill","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 15. Together (dub + keys + varsaw, 96 BPM)
('together_96bpm', 'Together', 'Clock.clear(); Clock.bpm = 96
Scale.default = "minor"
Root.default = 0
d1 >> play("x-o-x-o-", amp=0.4)
d2 >> play("V ", amp=0.4)
d3 >> dub(P[0,2,4,6]-4, dur=4, echo=0.75, delay=4, amp=0.2, lpf=800)
k1 >> keys(P[0,2,4,6,7,6,4,2], dur=0.5, amp=0.25, lpf=1200)
v1 >> varsaw(P[6,4,2,0], dur=2, amp=0.2, lpf=1500, room=0.3)
b1 >> bass(P[0,4,6,4], dur=[2,2,2,2], amp=0.25, lpf=600)',
'Dub + keys + varsaw, deep dub vibe, 96 BPM, from ekg/together', '["preset","dub","keys","varsaw","deep","chill","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 16. Celk (keys + blip + bass, 118 BPM)
('celk_118bpm', 'Celk', 'Clock.clear(); Clock.bpm = 118
Scale.default = "minor"
Root.default = 0
d1 >> play("x-o-x-o-", amp=0.4)
d2 >> play("V ", amp=0.4)
k1 >> keys(P[0,2,4,6], dur=2, amp=0.25, lpf=1200, room=0.3)
b1 >> blip(P[6,4,2,0,2,4,6,7], dur=0.5, amp=0.2, lpf=1500)
b2 >> bass(P[0,4,2,6], dur=[2,2,2,2], amp=0.25, lpf=600)',
'Keys + blip + bass, uptempo groove, 118 BPM, from ekg/celk', '["preset","keys","blip","bass","groove","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 17. Quoo (viola + keys + bass, 122 BPM)
('quoo_122bpm', 'Quoo', 'Clock.clear(); Clock.bpm = 122
Scale.default = "minor"
Root.default = 0
d1 >> play("x-o-x-o-", amp=0.4)
d2 >> play("V ", amp=0.4)
v1 >> viola(P[0,2,4,6,7,6,4,2], dur=1, amp=0.25, lpf=1200, room=0.4)
k1 >> keys(P[0,4,6,4], dur=4, amp=0.2, lpf=1000, room=0.3)
b1 >> bass(P[0,4,2,6], dur=[2,2,2,2], amp=0.25, lpf=600)',
'Viola + keys + bass, cinematic groove, 122 BPM, from ekg/quoo', '["preset","viola","keys","cinematic","groove","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 18. W1 (blip + pluck + bass, 100 BPM)
('w1_100bpm', 'W1', 'Clock.clear(); Clock.bpm = 100
Scale.default = "minor"
Root.default = 0
d1 >> play("x-o-x-o-", amp=0.4)
d2 >> play("V ", amp=0.4)
b1 >> blip(P[0,2,4,6,7,6,4,2], dur=0.5, amp=0.2, lpf=1500)
p1 >> pluck(P[7,6,5,4,3,2,1,0], dur=1, amp=0.2, lpf=1200)
b2 >> bass(P[0,4,2,6], dur=[2,2,2,2], amp=0.25, lpf=600)',
'Blip + pluck + bass, melodic groove, 100 BPM, from ekg/w1', '["preset","blip","pluck","melodic","groove","github"]', 5, 'preset', datetime('now'), datetime('now')),

-- 19. Bh19wow (bass + bell + gong, 134 BPM)
('bh19wow_134bpm', 'Bh19wow', 'Clock.clear(); Clock.bpm = 134
Scale.default = "minor"
Root.default = 0
d1 >> play("x-o-x-o-", amp=0.4)
d2 >> play("V ", amp=0.4)
b1 >> bass(P[0,2,4,6]-4, dur=2, amp=0.25, lpf=600)
b2 >> bell(P[0,2,4,6,7,6,4,2], dur=0.5, amp=0.2, lpf=1500, room=0.3)
g1 >> gong(P[0:4], dur=4, amp=0.15, room=0.5)',
'Bass + bell + gong, fast wild energy, 134 BPM, from ekg/bh19wow', '["preset","bass","bell","gong","fast","wild","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 20: Qirky chilled-vibes (creator of FoxDot!)
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'chilled_vibes_100bpm', 'Chilled Vibes',
'Clock.bpm=100; Scale.default="minor"
p1 >> pulse([0,-1,-2,-3], dur=8, lpf=600, lpr=0.2, crush=8) + (0,2,4,const(6))
p3 >> blip(p1.pitch, dur=8, sus=4, room=1, oct=6) + [0,0,0,P*(2,4,3,-1)]
p2 >> saw(P[:5][:9][:16], dur=1/4, oct=var([3,4],[12,4])).penta()
d1 >> play("(x )( x)o{ vx[xx]}", crush=16, rate=.8).every([24,5,3], "stutter", 4, dur=3)
d2 >> play("<-s>< ~*~>").every(30.5, "jump", cycle=32)',
'Pulse+blip+saw layered textures, crushed, stutter effects, 100 BPM, by Qirky (FoxDot creator)', '["preset","chill","pulse","saw","crush","stutter","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 21: Qirky alien-techno
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'alien_techno_140bpm', 'Alien Techno',
'Scale.default="minor"
Clock.bpm=140
c1 >> play("@", dur=1/4, sample=P[:8].rotate([0,1,3]), rate=4, pan=-0.5)
c2 >> play("#", dur=40, room=1, amp=2, pan=0.5)
d1 >> play("<V:><  * ><[--]>")
b1 >> dbass(dur=PDur(3,8), sus=2, chop=4, shape=PWhite(0,1/2), pan=PWhite(-1,1)).sometimes("offadd", 4) + var([0,2],4)
p1 >> space([7,6,4,P*(2,1),0], dur=8, pan=(-1,1))
Master().hpf=var([0,4000],[76,4])',
'Dbass+space synth, 140 BPM, alien textures with HPF sweep, by Qirky', '["preset","techno","alien","dbass","space","hpf","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 22: Qirky crazytown
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'crazytown', 'Crazytown',
'Scale.default = Scale.minor
p1 >> pluck([0,0, 3, 3, 4, -2,4, -2],dur=[1/2,1,1/2, 1, 1/2, 1, 1/2, 1/4, 1/4],oct=3, amp=1.3,lpf=linvar([400,4000],16)).every(4, "trim", 3)
d1 >> play("x-o-x-o-x-o-x-oo(oo)",room=0.8, mix=0.4,amp=[0.7, 0.6, 0.7, 0.8]).every(5, "trim", 2)
p2 >> pads([0,2,4,6], dur=4, spin=4, oct=4,amp=2.0, chop=[8,16], hpf=linvar([500,2000],16), hpr=0.2).every(8, "shuffle")
p3 >> karp([0,2,4,6,9], dur=[1,2,1.5], amp=0.5,echo=0.75, decay=0.5, sustain=0.1).every(8, "shuffle")',
'Pluck+pads+karp layered, minor key, chop+shuffle effects, by Qirky', '["preset","pluck","pads","karp","chop","shuffle","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 23: Qirky dark-trance
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'dark_trance', 'Dark Trance',
'Scale.default="minor"; Clock.bpm=126; Root.default=var([0,3],32)
b1 >> dirt([0,0,0.5], dur=PDur(3,8), sus=1, chop=2, drive=0.5, formant=1, oct=(5), room=0.2).spread()
b2 >> bell(var(P[2:10]), dur=2/3, oct=6, pan=[-1,1]).penta()
p1 >> pasha(var([0,2,0.5],[3,1/2,1/2]), dur=PDur(5,8), oct=6, sus=2, pan=PWhite(-1,1)).every(4, "dur.shuffle")
c1 >> play("#", rate=-1/2, hpf=1000, dur=4, amp=4, room=1, coarse=16).spread()
d1 >> play("+", dur=PDur(3,8,[0,2]), amp=2, sample=3).sometimes("rate.offadd", 1)
d2 >> play("nN", dur=1/4, sample=PRand(5, seed=1)[:16], pan=PWhite(-1,1), rate=PRand(1,5))
d3 >> play("<|x1||l(21)|><  *( =)>", formant=0, sample=2).every(14.5, "jump", cycle=16)
d4 >> play("<[--]>< +( +)[ +]>")',
'Dirt+bell+pasha, 126 BPM, minor, formant+chop+drive, 4 drum layers, by Qirky', '["preset","trance","dirt","bell","pasha","formant","chop","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 24: Qirky glow
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'glow', 'Glow',
'd1 >> play("x o x - o = x - - = o x ")
b1 >> pluck([0, 3, 2], dur=[1/2, 1], oct=4)
d2 >> play("--   -- ---")
p1 >> pads([0, 3, 7, 8, -2], dur=[4, 8], oct=5, amp=0.7)',
'Pluck+pads+play, gentle ambient groove, by Qirky', '["preset","pluck","pads","ambient","groove","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 25: Qirky luvly-nais
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'luvly_nais', 'Luvly Nais',
'p1 >> keys(([3,4,5,4],7,[9,9,9,10]), dur=4, spin=8, tremolo=4, room=1, amp=2)
p2 >> sawbass(var([1,0,-2,-4],4), dur=1/2, amp=[1,1,0,1]).sometimes("amp.trim", 3)
p3 >> star(p2.pitch + (P[1,0,1,0,1,0,0,2,1,0]), dur=PDur([5,1,2,2],8)*2, sus=2, oct=6)
d1 >> play("<Vs><[--]><.{.+[ +]}O( [( u)O])>", lpf=var([0,1000],[28,4]))
p3.sometimes("offadd", 4) + var([0,2],[PRand([0,2,4])])',
'Keys+sawbass+star, spin+tremolo+room effects, by Qirky', '["preset","keys","sawbass","star","tremolo","room","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 26: Qirky multiply
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'multiply', 'Multiply',
'Scale.default="minor"; Clock.bpm=144

p1 >> keys(var((P(0,1,2,3,4) * P[3,2,5,7]) % 15), dur=PDur(3,8), amp=1/2, rate=linvar([0,12],16)).spread()

p2 >> saw(var([0,-1]), amp=[0,1], vib=12, oct=6, drive=0.5, lpf=linvar([2000,6000],32), lpr=0.3) + var([0,-2,[0,-2]],[24,4,4])

b1 >> sawbass(var([3,2]), dur=PDur(3,8)*2, sus=1/2, cutoff=var([750,2000],32), delay=(0,0.5), oct=(5,[4,6]), amp=var([1,0],[15,1]))

d1 >> play("[--]~", sample=var([0,2],32))
d2 >> play("<X >< (nb)H(l[hI])>", sample=2)',
'Keys+saw+sawbass, PDur rhythms, 144 BPM minor, by Qirky', '["preset","keys","saw","sawbass","PDur","144bpm","minor","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 27: neb-sidnal glass_house
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'glass_house', 'Glass House',
'Clock.bpm = 100
Scale.default = "minor"

p1 >> play("<[cc] f [nnn] * ><sss   ><x X>",
    sample=-1,
    pan=[-1,0,1],
    amp=0.6)
    
a1 >> glass([0,0,[4,3],5,(2,-5)], 
    dur=4, 
    amp=linvar([1,1,2],24),
    echo=1,
    delay=linvar([0.5, 2], 12)) + [(-2,2),(0,2)]
    
a2 >> sinepad(P[[((0,-2),(4,-7),(-7,-5))],[(1,-1),(5,0)],[-4,0,-1]],
    dur=PDur([1,4],5),
    amp=linvar([1,1.2],6),
    lpf=1000).every(8, "stutter")',
'Glass+sinepad+play, 100 BPM minor, pan+echo+stutter, by neb-sidnal', '["preset","glass","sinepad","ambient","100bpm","minor","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 28: neb-sidnal synth_lofi_beat (volumes adjusted)
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'synth_lofi_beat', 'Synth Lo-Fi Beat',
'Clock.bpm=50
Scale.default = "minor"

a1 >> marimba(P[0,2,[3,4],[0,5]], 
    dur=PDur([3,8],4),
    chop=linvar([0.3,0.7],18),
    lpf=linvar([1000,4000],8),
    amp=0.25,) + ([2,4,0,0],[5,-7,5,-7])
    
a2 >> ambi([0,-4,[-2,2]], 
    dur=2,
    amp=linvar([0.15,0.35],12))
    
a3 >> pluck(P[[(2,0),(-2,4)],(0,3)],
    dur= PDur([2,3],8))
    
b1 >> pulse([-14,-7], 
    dur=8,
    amp=linvar([0.1,0.3],24))
    
p1 >> play("<x o x * ><f[---] n>",
    sample=1,
    pan=[-1,0,1])
    
p2 >> play("<f f b><---*><s O>", 
    bpm=100)',
'Lo-fi beat: marimba+ambi+pluck+pulse, 50 BPM minor, PDur rhythms, by neb-sidnal', '["preset","lofi","chill","marimba","ambi","50bpm","minor","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 29: neb-sidnal downtempo_chinese_scale (self-destruct stripped, volumes adjusted)
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'downtempo_chinese_scale', 'Downtempo Chinese Scale',
'Clock.bpm=110

Scale.default="chinese"
Root.default=0

var.chords = var([0,2],8)

p1 >> gong([0,1,2,-3,-4,5,-6,7], amp=linvar([0.2,0.4],14), dur=[2,1]).every(8, "reverse").every(5, "rotate")

p2 >> bass(var.chords, dur=PDur(2,3)*2, amp=[0.12,0.1,0.18], pan=linvar([-1,1],12))

p3 >> sinepad(var.chords + var([(0,2),(2,4)]), amp=0.2, dur=[1, 1/2], chop=linvar([0,1],20))

a1 >> pads(P[0,-2,2,3].loop(5)|P[2,3,4,1].stutter(3), dur=1/2, amp=0.2, lpf=linvar([1000,3000],12))

d1 >> play("<x{ o} -[---] ><s n sn ><d d d dd >", dur=1/2, sample=1, pan=(0,1), amp=0.2).every(8, "stutter")

d2 >> play("<X O ><tt tt tt >", amp=0.2)',
'Downtempo Chinese scale: gong+bass+sinepad+pads, 110 BPM, PDur+reverse+rotate, by neb-sidnal', '["preset","chinese","downtempo","gong","ambient","110bpm","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 30: neb-sidnal ambient_glitch (Clock.clear stripped, volumes adjusted)
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'ambient_glitch', 'Ambient Glitch',
'Scale.default=''minor''
Clock.bpm= linvar([100,130],44)

p1 >> pluck(P([0,2,3],[7,-7],(5,3),[0,-1,-2,-4]).shuffle(),
    delay=linvar([0.2, 1.3],18),
    sus=linvar([PDur(3,8),3],32),
    lpf=linvar([300,4000],9),
    amp=linvar([0.2,0.5],20))
    
p2 >> ambi(P([4,0,4],[7,2,7]).arp([0,3]),
    dur=PDur(1,8),
    chop=linvar([0,4],22),
    amp=0.3)

p3 >> sinepad(dur=5,
    sus=4,
    chop=linvar([0,2],17),
    delay=linvar([0.4,0.8],33),
    amp=0.35) + [3,5]
    
d1 >> play(''<XNNNONNN>< * * * * >'', sample=(-1,0),
    amp=linvar([0.15,0.35],22)).every(5, "stutter", 3)',
'Ambient glitch: pluck+ambi+sinepad, BPM 100-130, shuffle+stutter+chop, by neb-sidnal', '["preset","ambient","glitch","pluck","chop","100-130bpm","github"]', 5, 'preset', datetime('now'), datetime('now'));

-- Track 31: neb-sidnal gentle_sounds (.after timers stripped, volumes adjusted)
INSERT OR IGNORE INTO music_tracks (name, title, code, description, tags, rating, type, created_at, updated_at) VALUES (
'gentle_sounds', 'Gentle Sounds',
'Clock.bpm = 130
Scale.default = Pvar([Scale.minor, Scale.minorPentatonic],45)
Root.default = "A"

p1 >> zap([(0,-5),(-3,-8) + (-1,-7)],
    dur=[1,1,1/2],
    amp=linvar([0.15,0.3],28),
    sus=8,
    oct=4,
    chop=linvar([0,0.3],15),
    lpf=linvar([800,4500],19)).every(21, "stutter")

p2 >> ambi([0,0,-4,-1],
    dur=5,
    amp=0.2,
    room=0.8,
    chop=linvar([0,1],33),
    fmod=1).every(11, "stutter") + [-8,-5]

p3 >> glass(oct=linvar([5,4],18), dur=[1,5], sus=3, amp=linvar([0.2,0.35],35))

p4 >> space(P[-3,(-5,-3),-7,(-11,-5)].loop(8)|P[(-2,-4),(-2,-3),(0,-2),(-5,5)].stutter(4),
    amp=0.2,
    sus=linvar([1.5,2.5],19),
    delay=0)

d1 >> play("<x x [sn] x  >", amp=0.18, room=0.7)
d2 >> play("       {o }  ", amp=0.15, sus=2, room=0.9)

b1 >> marimba(P[0,0,0,[4,5]], amp=0.3, oct=3, dur=PDur([3,4],8)).every(9, "stutter")',
'Gentle ambient: zap+ambi+glass+space+marimba, 130 BPM, minor/pentatonic, by neb-sidnal', '["preset","ambient","gentle","zap","glass","space","130bpm","github"]', 5, 'preset', datetime('now'), datetime('now'));
