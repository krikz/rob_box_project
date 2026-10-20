"""FaceDirector: which scene the face shows and with which live values (ADR-0150 §5.5)."""

from __future__ import annotations

import json

import numpy as np
import pytest

from rob_box_animations.face_director import (FaceDirector, GazeTrack, MouthTrack, MusicTrack,
                                              parse_animation_request)

T0 = 1_800_000_000.0     # epoch seconds, like envelope t0 / player started_at


def awake(**kw) -> FaceDirector:
    """Director past its start-up wakeup, voice owner present."""
    d = FaceDirector(T0, **kw)
    d.voice_seen(T0 + 7.0)
    assert d.choose(T0 + 7.0)[0] == 'idle_stare'
    return d


def test_start_shows_wakeup_then_idle():
    d = FaceDirector(T0)
    assert d.choose(T0)[0] == 'wakeup'
    d.voice_seen(T0 + 5)
    assert d.choose(T0 + 6.5)[0] == 'idle_stare'


def test_no_voice_owner_for_timeout_sleeps_then_wakes_when_it_returns():
    d = awake(voice_timeout_s=10.0)
    assert d.choose(T0 + 16.9)[0] == 'idle_stare'
    assert d.choose(T0 + 17.1)[0] == 'sleep'
    d.voice_seen(T0 + 30)
    assert d.choose(T0 + 30)[0] == 'wakeup'
    assert d.choose(T0 + 36.5)[0] == 'idle_stare'


def test_never_seen_voice_owner_sleeps_after_timeout():
    d = FaceDirector(T0, voice_timeout_s=10.0)
    assert d.choose(T0 + 10.5)[0] == 'sleep'


def test_tts_state_speaking_with_debounce_between_chunks():
    d = awake()
    now = T0 + 8
    d.on_tts_state('synthesizing', now)
    assert d.choose(now)[0] == 'talk_jaw'
    d.on_tts_state('ready', now + 1.0)
    assert d.choose(now + 1.3)[0] == 'talk_jaw'          # 0.6 s hang: no flicker between chunks
    d.on_tts_state('synthesizing', now + 1.35)
    assert d.choose(now + 2.5)[0] == 'talk_jaw'
    d.on_tts_state('ready', now + 3.0)
    assert d.choose(now + 3.7)[0] == 'idle_stare'


def test_stopped_ends_speech_at_once_and_shuts_the_mouth():
    d = awake()
    now = T0 + 8
    d.on_envelope('s1', now, 40, [0.3] * 100, [], now)
    d.on_tts_state('playing', now)
    for k in range(5):
        d.choose(now + 0.04 * k)
    assert d.mouth.level(now + 0.2)[0] > 0.5
    d.on_tts_state('stopped', now + 0.25)
    assert d.mouth.level(now + 0.26)[0] < 0.05
    assert not d.speaking(now + 0.26)


def test_mouth_follows_the_envelope_window_of_real_speech():
    m = MouthTrack()
    # 40 ms windows: loud, silence, loud
    m.add('s1', 100.0, 40, [0.2] * 5 + [0.0] * 5 + [0.2] * 5, [])
    opened = [m.level(100.0 + 0.04 * i)[0] for i in range(15)]
    assert max(opened[:5]) > 0.9           # opens with the voice (attack within a couple of frames)
    assert opened[9] < 0.15                # closes in the pause
    assert opened[14] > 0.9
    assert m.until() == pytest.approx(100.6)


def test_mouth_vowel_hint_and_cancel_by_speech_id():
    m = MouthTrack()
    m.add('a', 10.0, 40, [0.5] * 10, [2] * 10)
    m.add('b', 10.5, 40, [0.5] * 10, [])
    assert m.level(10.1)[1] == 'O'
    m.cancel('a')
    assert m.level(10.14)[0] == 0.0
    assert m.until() == pytest.approx(10.9)


def test_envelope_alone_makes_the_face_talk_with_hang():
    d = awake()
    now = T0 + 8
    d.on_envelope('s1', now + 0.55, 40, [0.2] * 25, [], now)   # t0 = now + output latency
    key, ctx = d.choose(now + 0.6)
    assert key == 'talk_jaw' and ctx.mouth_level is not None
    assert d.choose(now + 0.55 + 1.0 + 0.5)[0] == 'talk_jaw'
    assert d.choose(now + 0.55 + 1.0 + 0.7)[0] == 'idle_stare'


def test_cancelled_speech_event_closes_mouth():
    d = awake()
    now = T0 + 8
    d.on_envelope('s1', now, 40, [0.4] * 50, [], now)
    d.choose(now + 0.1)
    assert d.choose(now + 0.15)[1].mouth_level > 0.5
    d.on_speech_cancelled('s1', now + 0.2)
    assert d.mouth.level(now + 0.24)[0] == 0.0             # mouth shut within one frame (A25: <= 80 ms)
    assert d.choose(now + 0.24)[0] == 'idle_stare'         # and the face stops talking at once


def test_mouth_design_parameter():
    d = awake(mouth_design='voicebox')
    d.on_tts_state('playing', T0 + 8)
    assert d.choose(T0 + 8)[0] == 'talk_voicebox'


def test_request_v1_name_with_duration_and_unknown_name():
    d = awake()
    assert d.request('happy', 3.0, T0 + 8) == 'happy'
    assert d.choose(T0 + 9)[0] == 'happy'
    assert d.choose(T0 + 11.1)[0] == 'idle_stare'
    assert d.request('thinking', None, T0 + 12) == 'thinking_teeth'
    assert d.request('no_such', None, T0 + 12) is None
    assert parse_animation_request('angry:2.5') == ('angry', 2.5)
    assert parse_animation_request('sad') == ('sad', None)
    assert parse_animation_request('sad:x') == ('sad', None)


def test_requested_emotion_while_speaking_becomes_the_tone():
    d = awake()
    d.request('happy', 5.0, T0 + 8)
    d.on_tts_state('playing', T0 + 8)
    key, ctx = d.choose(T0 + 8.5)
    assert key == 'talk_jaw' and ctx.mood == 'happy'
    d.on_tts_state('stopped', T0 + 9)
    assert d.choose(T0 + 9.1)[0] == 'happy'               # the request outlives the speech


def _snap(**kw):
    base = {'state': 'playing', 'track_id': 't', 'stops_at': None, 'dj': {'enabled': True}, 'ts': T0}
    base.update(kw)
    return json.dumps(base)


@pytest.mark.parametrize('bpm,scene', [(90, 'music_vibe'), (118, 'music_groove'), (128, 'music_shades'), (None, 'music_groove')])
def test_music_scene_by_tempo(bpm, scene):
    d = awake()
    assert d.on_music_state(_snap(dj={'enabled': True, 'bpm': bpm, 'started_at': T0}), T0 + 8)
    key, ctx = d.choose(T0 + 10)
    assert key == scene
    if bpm:
        assert ctx.beat_phase == pytest.approx(10 * bpm / 60)


def test_music_idle_and_finished_track_and_bad_json():
    d = awake()
    d.on_music_state(_snap(stops_at=T0 + 9, dj={'bpm': 118}), T0 + 8)
    assert d.choose(T0 + 9.5)[0] == 'music_groove'
    assert d.choose(T0 + 10.5)[0] == 'idle_stare'          # stops_at + 1 s grace passed
    d.on_music_state(_snap(state='idle'), T0 + 11)
    assert d.choose(T0 + 11)[0] == 'idle_stare'
    assert not MusicTrack().on_state('not json')


def test_speech_wins_over_music():
    d = awake()
    d.on_music_state(_snap(dj={'bpm': 118}), T0 + 8)
    d.on_tts_state('playing', T0 + 8)
    assert d.choose(T0 + 8.1)[0] == 'talk_jaw'


def test_gaze_follows_doa_only_while_someone_speaks():
    g = GazeTrack(front_deg=0.0, ccw=True, hold_s=8.0)
    g.on_direction(40.0, 1.0)
    assert g.gaze(1.0) is None                      # TV noise has a DOA too
    g.on_vad(True, 1.1)
    assert g.gaze(1.1) == pytest.approx(40.0)
    g.on_direction(350.0, 1.2)
    assert g.gaze(1.2) == pytest.approx(-10.0)
    g.on_vad(False, 2.0)
    assert g.gaze(9.0) == pytest.approx(-10.0)      # looks at the last speaker a while
    assert g.gaze(10.5) is None
    flipped = GazeTrack(front_deg=90.0, ccw=False)
    flipped.on_vad(True, 0.0)
    flipped.on_direction(120.0, 0.0)
    assert flipped.gaze(0.0) == pytest.approx(-30.0)


def test_idle_gets_bored_and_comes_back():
    d = awake(idle_bored_every_s=60.0)
    d.tick(T0 + 7.0)
    for s in range(8, 70):
        d.voice_seen(T0 + s)
        d.tick(T0 + s)
    assert d.key == 'idle_bored'
    for s in range(70, 85):
        d.voice_seen(T0 + s)
        d.tick(T0 + s)
    assert d.key == 'idle_stare'


def test_tick_crossfades_between_scenes():
    d = awake(crossfade_s=0.12)
    a = d.tick(T0 + 7.0)
    d.request('angry', 5.0, T0 + 7.0)
    mid = d.tick(T0 + 7.06)
    after = d.tick(T0 + 7.2)
    assert d.key == 'angry'
    assert d._prev is None
    assert mid.mouth.shape == a.mouth.shape == after.mouth.shape


def test_test_pattern_overrides_everything():
    d = FaceDirector(T0, test_pattern=True)
    f = d.tick(T0 + 100)
    assert d.key == 'test_pattern'
    assert np.asarray(f.mouth).sum() > 0
