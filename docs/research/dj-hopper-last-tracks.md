# DJ Хопер — последние два трека (Kate Bush план)

Дата: 2026-06-12, переходы #6 и #7  
Персона: DJ Хопер (смена прошла), Тема: Kate Bush "Rolled Up the Hill" (смена ОТКЛОНЕНА)

---

## Трек #6 — Army Dreamers (переход #6, 100 BPM, G minor)

```python
# === ARMY DREAMERS — Kate Bush inspired ===
# Chords: Gm → Cm → D → F  (i → iv → V → VII)
# Descending energy, soft and intimate

chords = var([0, 3, -2, -1], 4)  # Gm → Cm → D → F

def verse():
    # Gentle brush kick — every other bar
    d1 >> play("A...", spack=0, sample=36, amp=0.2, room=0.3, mix=0.2)
    # Soft snare on 3rd beat — YamahaVintage dry
    d2 >> play("...i..i.", sample=33, amp=0.12, room=0.2, mix=0.15)
    # Gentle hi-hat
    d3 >> play("--.-", dur=1, sample=3, amp=0.08, room=0.1)
    # Acoustic guitar — karp plucks, descending melody
    p1 >> karp([0, -1, 4, -3, 2, 0, -2, 4], dur=0.5, oct=5, amp=0.35,
               room=0.35, mix=0.25, hpf=120)
    # Warm synth pad following chords
    p2 >> pads(dur=4, amp=0.25, room=0.6, mix=0.35, sus=5).follow(chords) + (0,2,4)
    # Soft bass — gentle, round
    p3 >> dub(chords, dur=2, oct=3, amp=0.3, room=0.2, mix=0.15)
    Clock.future(32, chorus)

def chorus():
    # Slightly more present drums
    d1 >> play("A..A....", spack=0, sample=36, amp=0.25, room=0.3, mix=0.2)
    d2 >> play("...i..i.", sample=33, amp=0.15, room=0.25, mix=0.2)
    d3 >> play("--.-", dur=1, sample=3, amp=0.1, room=0.15)
    # Guitar opens up — longer sustains
    p1 >> karp([0, 2, 4, 2, 0, -1, -3, 4], dur=1, oct=5, amp=0.4,
               room=0.4, mix=0.3, hpf=100)
    # Choir-like vocal texture — cello samples (NOT 'c' for variety!)
    p2 >> pads(dur=4, amp=0.3, room=0.7, mix=0.4, sus=6).follow(chords) + (0,2,4)
    # Bass breathes
    p3 >> dub(chords, dur=2, oct=3, amp=0.35, room=0.25, mix=0.2)
    Clock.future(32, bridge)

def bridge():
    # Strip back — just heartbeat kick and pads
    d1 >> play("A.....A", spack=0, sample=36, amp=0.15, room=0.5, mix=0.3)
    d2.stop()
    d3.stop()
    # Ethereal strings texture via pitchglitch 'g' (strings/horns)
    d4 >> play("  g    g ", spack=1, sample=var([0,1,2], 8), amp=0.12, room=0.8, mix=0.5, sus=4)
    # Sparse guitar notes — like Kate's piano, floating
    p1 >> karp([0, 4, -3, 2], dur=2, oct=5, amp=0.3, room=0.6, mix=0.4)
    p2 >> pads([0, 2, 4], dur=8, amp=0.2, room=0.8, mix=0.5, sus=7)
    p3 >> dub([0, -2], dur=4, oct=3, amp=0.2, room=0.3)
    Clock.future(16, outro)

def outro():
    # Final descent — everything fades
    d1 >> play("A...", spack=0, sample=36, amp=0.1, room=0.6, mix=0.4)
    d4.stop()
    # Last guitar echoes
    p1 >> karp([0, -3], dur=4, oct=5, amp=0.2, room=0.7, mix=0.5)
    p2 >> pads([0, 2], dur=8, amp=0.15, room=0.9, mix=0.6, sus=8)
    p3.stop()
    Clock.future(16, lambda: Clock.clear())

verse()
```

**Паттерн:** `army_dreamers`  
**MC-фраза:** "Army Dreamers. Мягкий спуск после бури."

---

## Трек #7 — Moments of Pleasure (переход #7, 80 BPM, F major)

```python
# "Moments of Pleasure" — Kate Bush inspired ambient ballad farewell
# Structure: Verse → Chorus → Bridge → Final Chorus → Warm Fadeout

def verse():
    d1 >> play("A   A   ", sample=24, amp=0.2, room=0.4, mix=0.3)
    d2 >> play("  i i ", sample=17, amp=0.1, room=0.5, mix=0.4)
    d3 >> play("--.-", dur=1, sample=3, amp=0.08, room=0.6, mix=0.3)
    p1 >> rhpiano([0,2,4,3,5,4,2,0], dur=2, oct=4, amp=0.4, room=0.7, mix=0.4,
                   sus=3)  # rhpiano = FM Rhodes, no clicking (was pianovel)
    p2 >> strings([0,4,2,5,3,7], dur=4, oct=4, amp=0.3, room=0.8, mix=0.5, sus=4)
    p3 >> bass([0,-1,0,2,3,0,-1,2], dur=4, oct=2, amp=0.3, room=0.3, sus=3)
    Clock.future(32, chorus)

def chorus():
    d1 >> play("A   A  A", sample=24, amp=0.25, room=0.4, mix=0.3)
    d2 >> play("  i i i ", sample=17, amp=0.12, room=0.5, mix=0.4)
    d3 >> play("--.-", dur=1, sample=3, amp=0.1, room=0.6, mix=0.3)
    p1 >> rhpiano([(0,2,4),(0,3,5),(0,2,4),(2,4,7)], dur=4, oct=4, amp=0.5,
                   room=0.7, mix=0.4, sus=5, lfospeed=0.5, lfodepth=0.12)
    p2 >> strings([0,7,5,4,2,0], dur=3, oct=5, amp=0.4, room=0.9, mix=0.55, sus=4,
                  lfospeed=0.3, lfodepth=0.06)
    p3 >> bass([0,3,5,3,0,-1,2,0], dur=4, oct=2, amp=0.35, room=0.3, sus=3)
    Clock.future(32, bridge)

def bridge():
    d1 >> play("A       ", sample=24, amp=0.12, room=0.5, mix=0.4)
    d2.stop()
    d3 >> play("  --.- ", dur=1, sample=3, amp=0.05, room=0.7, mix=0.4)
    p1 >> rhpiano([7,5,4,2,0,2,4,0], dur=3, oct=5, amp=0.35, room=0.8, mix=0.5,
                   sus=4, lfospeed=0.6, lfodepth=0.15)
    p2 >> space([0,4,7,5], dur=8, oct=4, amp=0.25, room=1, mix=0.7, sus=8)
    p3 >> bass([0,0,5,3], dur=6, oct=2, amp=0.2, room=0.4, sus=5)
    Clock.future(32, final_chorus)

def final_chorus():
    d1 >> play("A   A  A", sample=24, amp=0.2, room=0.4, mix=0.3)
    d2 >> play("  i i ", sample=17, amp=0.1, room=0.5, mix=0.4)
    d3 >> play("--.-", dur=1, sample=3, amp=0.08, room=0.6, mix=0.3)
    p1 >> rhpiano([(0,2,4),(0,3,5),(0,2,4),(2,4,7)], dur=4, oct=4, amp=0.45,
                   room=0.7, mix=0.4, sus=5, lfospeed=0.5, lfodepth=0.1)
    p2 >> strings([0,4,2,5,3,7,5,4], dur=3, oct=5, amp=0.35, room=0.9, mix=0.55, sus=4)
    p3 >> bass([0,3,5,3,0,-1,2,0], dur=4, oct=2, amp=0.3, room=0.3, sus=3)
    Clock.future(32, outro)

def outro():
    d1 >> play("A       ", sample=24, amp=0.1, room=0.6, mix=0.5)
    d2.stop()
    d3.stop()
    p1 >> rhpiano([0,4,2,0], dur=6, oct=4, amp=0.25, room=0.9, mix=0.6,
                   sus=8, lfospeed=0.3, lfodepth=0.08)
    p2 >> strings([0], dur=12, oct=4, amp=0.15, room=1, mix=0.7, sus=10)
    p3 >> bass([0], dur=8, oct=2, amp=0.12, room=0.5, sus=6)
    Clock.future(32, fade_away)

def fade_away():
    d1.amp = 0.05
    p1 >> rhpiano([0], dur=8, oct=4, amp=0.1, room=1, mix=0.8, sus=8)
    p2 >> strings([0], dur=16, oct=4, amp=0.06, room=1, mix=0.8, sus=14)
    p3.amp = 0.05
    Clock.future(32, lambda: Clock.clear())

verse()
```

**Паттерн:** `moments_full_track`  
**MC-фраза:** "Moments of Pleasure. Спасибо за этот вечер. Берегите себя."

---

## Заметки

- Оба трека используют `Clock.future()` для цепочки секций (verse→chorus→bridge→outro→fade→clear)
- Структура: ~32 beats × 6 секций = ~2.4 мин (80 BPM) для Moments of Pleasure
- Army Dreamers: ~32 beats × 4 секции + 16 outro + 16 fade = ~2 мин (100 BPM)
- `Clock.clear()` в конце каждого трека — корректное завершение
- Оба трека в amp-лимитах (drums ≤0.3, synths ≤0.5, bass ≤0.35)
- `_cap_amp()` НЕ должен был менять код — всё уже в пределах
- `pianovel` заменён на `rhpiano` — FM-синтез вместо физмодели MdaPiano (которая цокала)
- `vib`/`vibdepth` заменены на `lfospeed`/`lfodepth` — реальные параметры вибрато в rhpiano
