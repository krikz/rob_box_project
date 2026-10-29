"""What the face shows (ADR-0150 §5.5.2–5.5.5): owners' inputs -> scene + its live values.

Pure Python, no ROS: ``face_node`` turns messages into ``on_*`` calls and asks
``tick(now)`` once a frame. Time is epoch seconds — the same clock as the
envelope's ``t0`` and the player snapshot's ``started_at`` (Sound and Vision
clocks are kept in sync by chrony).

Priority, highest first: side test frame > ``sleep`` (no voice owner for
``voice_timeout_s``) > ``wakeup`` (start / voice is back) > LLM face scene (``apply_face``, slots of
``rob_box_dialog.face_scene``; ``clear_face`` on failed/silenced) > talking (``SpeechEvent`` / envelope) >
requested scene (``play_animation`` / ``ExpressionRequest``) > driving (turn signal / brake light by
``/cmd_vel``) > midnight of 31.12 > battery (charging / low) > silence (taped mouth, #3984) > music > idle. Rear lights outside the scenes
that draw their own: manual ``set_rear`` (for ``duration_s``) > ``rear_theme`` parameter > holiday calendar.
"""
from __future__ import annotations

import json
import math
from typing import Iterable, List, Mapping, Optional, Tuple

import numpy as np

from rob_box_core.speech_client import ANY_AGENT, SpeakingWatch, event_outcome
from rob_box_dialog import expression as _expression
from rob_box_dialog.knowledge import (FACE_SCENE_MAX_S, FACE_SHOW_DEFAULT_S, MUSIC_DEFAULT_BPM,
                                      MUSIC_SCENE_MIN_S)

from .face import rear as R
from .face.dsl import DSL_KEY, DSL_SPEC
from .face.face import Frame, tint_of
from .face.scene import FaceContext, Scene
from .face.sides import sides_frame
from .face.states import SCENES, resolve
from .outside import BatteryWatch, DriveTracker, calendar_rear, calendar_scene

#: A requested emotion while the robot speaks becomes the tone of the talking
#: face (the mouth keeps following the voice).
MOOD_OF = {'happy': 'happy', 'victory': 'happy', 'wink': 'happy', 'talk_happy': 'happy', 'surprised': 'excited'}
#: Requested scene without ``ttl_ms``: the same default as a show asked by a person in the dialogue (#4083).
REQUEST_DEFAULT_S = FACE_SHOW_DEFAULT_S
REQUEST_MAX_S = 60.0


def parse_animation_request(text: str) -> Tuple[str, Optional[float]]:
    """Debug ``~/scene`` (and the removed v1 ``/voice/animation/request``): ``"name"`` or ``"name:seconds"``."""
    name, _, dur = str(text or '').strip().partition(':')
    try:
        seconds = float(dur) if dur.strip() else None
    except ValueError:
        seconds = None
    return name.strip(), seconds


def blend(a: Frame, b: Frame, k: float) -> Frame:
    out = Frame()
    for name, arr in b.panels().items():
        setattr(out, name, getattr(a, name) * (1.0 - k) + arr * k)
    return out


class MouthTrack:
    """Mouth by the envelope of the real TTS audio (§5.5.4): window ``(now - t0) / frame``.

    A streamed reply (#3930) sends its chunks faster than they sound, each with its own ``t0`` ahead of the clock.
    So the track forgets a chunk by the playback clock ``now`` (it has sounded ``KEEP_S`` ago), never by the ``t0``
    of the newest chunk or by a count: those would throw out the chunks that sound now (#4144)."""

    ATTACK, RELEASE = 0.6, 0.35   # share of the step to the target per 40 ms (as in the lab)
    GATE = 0.12                   # share of the chunk peak below which the mouth is shut
    KEEP_S = 1.0                  # a chunk is kept this long after its end: a late tick still finds its window
    MAX_AHEAD_S = 900.0           # memory guard: a chunk starting later than this ahead of now is no reply (bad t0)

    def __init__(self) -> None:
        self._chunks: List[tuple] = []
        self._o = 0.0
        self._last: Optional[float] = None

    def add(self, speech_id: str, t0: float, frame_ms: int, levels: Iterable[float], vowels: Iterable[int],
            now: float) -> None:
        """A chunk of a reply sounding from ``t0``; ``now`` is the clock the chunks that have sounded are forgotten by."""
        lv = [float(v) for v in levels]
        if not lv or t0 > now + self.MAX_AHEAD_S:
            return
        frame_s = max(1, int(frame_ms)) / 1000.0
        self._chunks = [c for c in self._chunks if c[2] > now - self.KEEP_S]
        self._chunks.append((speech_id, t0, t0 + len(lv) * frame_s, frame_s, lv, max(max(lv), 1e-3), list(vowels)))

    def cancel(self, speech_id: str = '') -> None:
        """Speech cancelled: drop its envelope and shut the mouth at once (≤ 80 ms, A25)."""
        self._chunks = [c for c in self._chunks if speech_id and c[0] != speech_id]
        self._o = 0.0

    def until(self) -> float:
        return max((c[2] for c in self._chunks), default=-math.inf)

    def _window(self, now: float) -> Optional[Tuple[float, str]]:
        for _sid, t0, t1, frame_s, lv, peak, vw in self._chunks:
            if t0 <= now < t1:
                i = min(int((now - t0) / frame_s), len(lv) - 1)
                x = lv[i] / peak
                vowel = 'AOE'[vw[i] - 1] if i < len(vw) and 1 <= vw[i] <= 3 else 'A'
                return (0.0 if x < self.GATE else min(1.0, x)), vowel
        return None

    def level(self, now: float) -> Tuple[float, str]:
        """(openness 0..1, vowel) through an audio follower: fast attack, slower release."""
        dt = 0.04 if self._last is None else min(0.5, max(0.0, now - self._last))
        self._last = now
        target, vowel = self._window(now) or (0.0, 'A')
        k = self.ATTACK if target > self._o else self.RELEASE
        self._o += (target - self._o) * (1.0 - (1.0 - k) ** (dt / 0.04))
        return self._o, vowel


class MusicTrack:
    """The player owner's snapshot ``/voice/music/state`` (ADR-0141): playing, tempo, start.

    The music scene (§5.5.5, #4010) is chosen by ``expression.music_scene`` - the one table of knowledge in
    ``rob_box_dialog.knowledge`` (the tempo picks the first scene of a track, then the face walks the circle whatever the
    tempo is). This class only knows what the code of the face must keep: which track this is (``track_no`` - how many
    times the track changed since the music began), how long it plays, and the floor ``MUSIC_SCENE_MIN_S`` a scene is held."""

    def __init__(self) -> None:
        self.playing = False
        self.bpm: Optional[float] = None
        self.started_at: Optional[float] = None
        self.stops_at: Optional[float] = None
        self.track_id: Optional[str] = None
        self.track_no = 0
        self._seen_at: Optional[float] = None
        self._scene: Optional[Tuple[str, float]] = None   # (scene, since) shown now

    def on_state(self, raw: str, now: Optional[float] = None) -> bool:
        try:
            snap = json.loads(raw)
        except (TypeError, ValueError):
            return False
        if not isinstance(snap, dict):
            return False
        dj = snap.get('dj') if isinstance(snap.get('dj'), dict) else {}
        self.playing = snap.get('state') == 'playing'
        self.bpm, self.started_at, self.stops_at = _num(dj.get('bpm')), _num(dj.get('started_at')), _num(snap.get('stops_at'))
        self._follow_track((snap.get('track_id') or '') if self.playing else None, now)
        return True

    def _follow_track(self, track_id: Optional[str], now: Optional[float]) -> None:
        """A new track id while the music plays is the next track (the circle steps); silence forgets the set."""
        if track_id is None:
            self.track_id, self.track_no, self._seen_at, self._scene = None, 0, None, None
        elif track_id != self.track_id:
            self.track_no = self.track_no + 1 if self.track_id is not None else 0
            self.track_id, self._seen_at = track_id, now

    def scene(self, now: float) -> Optional[Tuple[str, float, Optional[float]]]:
        """(scene, bpm, beats since start) while a track plays, else None."""
        if not self.playing or (self.stops_at is not None and now > self.stops_at + 1.0):
            return None
        bpm = self.bpm if self.bpm and self.bpm > 0 else MUSIC_DEFAULT_BPM
        began = self.started_at if self.started_at is not None else self._seen_at
        wanted = _expression.music_scene(self.bpm, self.track_no, now - began if began is not None else 0.0)
        beats = (now - self.started_at) * bpm / 60.0 if self.started_at is not None else None
        return self._held(wanted, now), bpm, beats

    def _held(self, wanted: str, now: float) -> str:
        """The scene the face shows: a change waits until the current scene has played ``MUSIC_SCENE_MIN_S``."""
        if self._scene is None or (wanted != self._scene[0] and now - self._scene[1] >= MUSIC_SCENE_MIN_S):
            self._scene = (wanted, now)
        return self._scene[0]


def _num(v) -> Optional[float]:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


class GazeTrack:
    """Speaker direction: DOA counts only while VAD says someone speaks (TV noise has a DOA too)."""

    def __init__(self, front_deg: float = 0.0, ccw: bool = True, hold_s: float = 8.0, fresh_s: float = 1.5) -> None:
        self.front_deg, self.ccw, self.hold_s, self.fresh_s = front_deg, ccw, hold_s, fresh_s
        self._doa: Optional[Tuple[float, float]] = None
        self._vad = False
        self._speaker: Optional[Tuple[float, float]] = None

    def _relative(self, deg: float) -> float:
        rel = (deg - self.front_deg + 180.0) % 360.0 - 180.0
        return rel if self.ccw else -rel

    def on_direction(self, deg: float, now: float) -> None:
        self._doa = (self._relative(deg), now)
        if self._vad:
            self._speaker = self._doa

    def look_at(self, deg: float, now: float) -> None:
        """Собеседник известен (азимут его фразы из ``/dialog/session``, #3695) — смотреть на него и без VAD."""
        self._speaker = (self._relative(deg), now)

    def on_vad(self, active: bool, now: float) -> None:
        self._vad = bool(active)
        if self._vad and self._doa and now - self._doa[1] <= self.fresh_s:
            self._speaker = (self._doa[0], now)

    def gaze(self, now: float) -> Optional[float]:
        if self._speaker is None:
            return None
        rel, at = self._speaker
        return rel if (self._vad or now - at <= self.hold_s) else None


class FaceDirector:
    """Chooses the scene and renders it with a crossfade between scenes."""

    def __init__(self, now: float, *, mouth_design: str = 'jaw', thinking_design: str = 'thinking_teeth',
                 idle_bored_every_s: float = 60.0, voice_timeout_s: float = 10.0, crossfade_s: float = 0.12,
                 speaking_hang_s: float = 0.6, rear_theme: str = 'default', test_pattern: bool = False,
                 gaze: Optional[GazeTrack] = None, tz_offset_h: float = 3.0) -> None:
        self.mouth_design, self.thinking_design = mouth_design, thinking_design
        self.idle_bored_every_s, self.voice_timeout_s = idle_bored_every_s, voice_timeout_s
        self.crossfade_s, self.speaking_hang_s = crossfade_s, speaking_hang_s
        self.rear_theme, self.test_pattern = rear_theme, test_pattern
        self.mouth, self.music, self.gaze = MouthTrack(), MusicTrack(), gaze or GazeTrack()
        self.drive, self.battery, self.tz_offset_h = DriveTracker(), BatteryWatch(), tz_offset_h
        self._rear: Optional[Tuple[str, float]] = None
        # «говорит» — пока звучит реплика любого агента (SpeechEvent started ... finished|cancelled|rejected);
        # часы наблюдателя — время директора (epoch, как у огибающей), ими отсекается потерянный конец реплики
        self._now = now
        self._speech = SpeakingWatch(ANY_AGENT, clock=lambda: self._now)
        self._speech_hang_until = -math.inf
        self._voice_seen = now          # grace: the voice owner has voice_timeout_s to show up
        self._asleep = False
        self._wake_until = now + SCENES['wakeup'].period
        self._request: Optional[Tuple[str, float]] = None
        self._overlay: Optional[Tuple[Mapping[str, str], float]] = None
        self._replay = False
        self._silenced = False
        self._idle_since = now
        self.key = ''
        self._cur: Optional[Tuple[Scene, float, FaceContext]] = None
        self._prev: Optional[Tuple[Scene, float, FaceContext]] = None
        self._fade_at = -math.inf

    # -- inputs ---------------------------------------------------------------
    def voice_seen(self, now: float) -> None:
        """Any sign of the voice owner (a message, a publisher in the graph)."""
        self._voice_seen = now

    def on_speech_event(self, event, now: float) -> None:
        """``SpeechEvent`` of the speech owner: the face talks while a reply of any agent sounds
        (``SpeakingWatch``: ``started`` ... ``finished|cancelled|rejected``).

        The end of the last reply keeps the face talking for ``speaking_hang_s``: replies of one answer come one after
        another, without the hang the face flickers to idle at every boundary. ``cancelled`` drops that reply's
        envelope and, if nothing else sounds, shuts the mouth at once (≤ 80 ms, A25).
        """
        self.voice_seen(now)
        self._now = now
        edge = self._speech.on_event(event)
        if event_outcome(event)[0] == 'cancelled':
            self.mouth.cancel(str(getattr(event, 'speech_id', '') or ''))
            if not self._speech.speaking:
                self._speech_hang_until = now
        elif edge is False:
            self._speech_hang_until = now + self.speaking_hang_s

    def on_envelope(self, speech_id: str, t0: float, frame_ms: int, levels, vowels, now: float) -> None:
        self.voice_seen(now)
        self.mouth.add(speech_id, t0, frame_ms, levels, vowels, now)

    def on_music_state(self, raw: str, now: float) -> bool:
        return self.music.on_state(raw, now)

    def on_cmd_vel(self, linear_x: float, angular_z: float, now: float) -> None:
        self.drive.on_cmd(linear_x, angular_z, now)

    def on_battery(self, percent: Optional[float], status: int, now: float) -> None:
        self.battery.on_battery(percent, status, now)

    def set_rear(self, theme: str, seconds: float, now: float) -> bool:
        """Manual rear-light theme for ``seconds`` (FaceScene.rear, PR-3c); beats the calendar."""
        if theme not in R.THEMES:
            return False
        self._rear = (theme, now + min(max(seconds, 0.0), REQUEST_MAX_S))
        return True

    def rear_theme_at(self, now: float) -> str:
        if self._rear and now < self._rear[1]:
            return self._rear[0]
        if self.rear_theme != 'default':
            return self.rear_theme
        return calendar_rear(now, self.tz_offset_h) or 'default'

    def apply_face(self, slots: Mapping[str, str], seconds: float, now: float) -> None:
        """LLM face scene (§5.5.9) over the reflex for ``seconds`` (≤ 30); its ``rear`` slot sets the rear theme too."""
        seconds = min(max(float(seconds), 0.0), FACE_SCENE_MAX_S)
        self._overlay = (dict(slots), now + seconds)
        self._replay = True                 # a new scene replays from its start (a wink, the sunglasses drop)
        if slots.get('rear'):
            self.set_rear(R.REAR_OF_SLOT[slots['rear']], seconds, now)

    def clear_face(self) -> bool:
        """failed / silenced take the LLM scene (and its rear theme) off the face; True if there was one."""
        had = self._overlay is not None
        self._overlay, self._rear = None, None
        return had

    @property
    def silenced(self) -> bool:
        return self._silenced

    def set_silenced(self, silenced: bool) -> bool:
        """Silence scene is on the session stack (#3984): the face is the taped mouth until it ends; entering it takes
        the LLM scene and the requested reflex scene off. True if the state changed."""
        if silenced == self._silenced:
            return False
        self._silenced = silenced
        if silenced:
            self.clear_face()
            self._request = None
        return True

    def request(self, name: str, seconds: Optional[float], now: float) -> Optional[str]:
        """Show scene ``name`` (scene or v1 animation name) for ``seconds``; None if unknown."""
        scene = resolve(name, self.mouth_design, self.thinking_design)
        if scene is None:
            return None
        dur = REQUEST_DEFAULT_S if not seconds or seconds <= 0 else min(float(seconds), REQUEST_MAX_S)
        self._request = (scene, now + dur)
        return scene

    # -- choice ---------------------------------------------------------------
    def speaking(self, now: float) -> bool:
        self._now = now
        if self._speech.speaking or now < self._speech_hang_until:
            return True
        return now < self.mouth.until() + self.speaking_hang_s

    def _voice_alive(self, now: float) -> bool:
        alive = now - self._voice_seen < self.voice_timeout_s
        if alive and self._asleep:
            self._wake_until = now + SCENES['wakeup'].period
        self._asleep = not alive
        return alive

    def choose(self, now: float) -> Tuple[str, FaceContext]:
        rear = self.rear_theme_at(now)
        base = FaceContext(rear_theme=rear)
        if not self._voice_alive(now):
            return 'sleep', base
        if now < self._wake_until:
            return 'wakeup', base
        gaze = self.gaze.gaze(now)
        if self._overlay and now >= self._overlay[1]:
            self._overlay = None
        if self._overlay:
            return DSL_KEY, self._dsl_context(self._overlay[0], gaze, rear, now)
        req = self._request[0] if self._request and now < self._request[1] else None
        if self.speaking(now):
            level, vowel = self.mouth.level(now)
            return f'talk_{self.mouth_design}', FaceContext(
                gaze_deg=gaze, mouth_level=level, vowel=vowel, mood=MOOD_OF.get(req, 'neutral'), rear_theme=rear)
        if req:
            return req, FaceContext(gaze_deg=gaze, rear_theme=rear)
        outside = self.drive.scene(now) or calendar_scene(now, self.tz_offset_h) or self.battery.scene(now)
        if outside:
            return outside, base
        if self._silenced:
            return _expression.silent_face(), FaceContext(gaze_deg=gaze, rear_theme=rear)
        music = self.music.scene(now)
        if music:
            name, bpm, beats = music
            return name, FaceContext(bpm=bpm, beat_phase=beats, rear_theme=rear)
        return self._idle(now), FaceContext(gaze_deg=gaze, rear_theme=rear)

    def _dsl_context(self, slots: Mapping[str, str], gaze: Optional[float], rear: str, now: float) -> FaceContext:
        level, vowel = self.mouth.level(now) if self.speaking(now) else (None, 'A')
        return FaceContext(gaze_deg=gaze, mouth_level=level, vowel=vowel, tint=tint_of(slots.get('tint'), now),
                           rear_theme=rear, face=slots)

    def _idle(self, now: float) -> str:
        if not self.key.startswith('idle_'):
            self._idle_since = now
        bored_for = SCENES['idle_bored'].period
        since = now - self._idle_since
        if since >= self.idle_bored_every_s + bored_for:
            self._idle_since = now
            return 'idle_stare'
        return 'idle_bored' if since >= self.idle_bored_every_s else 'idle_stare'

    # -- output ---------------------------------------------------------------
    def tick(self, now: float) -> Frame:
        if self.test_pattern:
            self.key = 'test_pattern'
            return sides_frame()
        key, ctx = self.choose(now)
        if key != self.key or self._cur is None or self._replay:
            self._replay = False
            self._prev = self._cur
            self._fade_at = now
            self._cur = (Scene(DSL_SPEC if key == DSL_KEY else SCENES[key]), now, ctx)
            self.key = key
        scene, start, _ = self._cur
        self._cur = (scene, start, ctx)
        frame = scene.frame(now - start, ctx)
        k = (now - self._fade_at) / self.crossfade_s if self.crossfade_s > 0 else 1.0
        if self._prev is not None and k < 1.0:
            p_scene, p_start, p_ctx = self._prev
            frame = blend(p_scene.frame(now - p_start, p_ctx), frame, float(np.clip(k, 0.0, 1.0)))
        elif k >= 1.0:
            self._prev = None
        return frame
