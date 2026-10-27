"""FaceDirector: which scene the face shows and with which live values (ADR-0150 §5.5)."""

from __future__ import annotations

import json

import numpy as np
import pytest

from rob_box_animations.face_director import (FaceDirector, GazeTrack, MouthTrack, MusicTrack,
                                              parse_animation_request)
from rob_box_core.msg_stub import build_dialog_msgs_module

T0 = 1_800_000_000.0     # epoch seconds, like envelope t0 / player started_at
SpeechEvent = build_dialog_msgs_module().SpeechEvent   # built from the real .msg: same kinds as the owner sends
QUEUED, SYNTHESIZED, STARTED = SpeechEvent.KIND_QUEUED, SpeechEvent.KIND_SYNTHESIZED, SpeechEvent.KIND_STARTED
FINISHED, CANCELLED, REJECTED = SpeechEvent.KIND_FINISHED, SpeechEvent.KIND_CANCELLED, SpeechEvent.KIND_REJECTED


def speech(d: FaceDirector, kind: int, now: float, sid: str = 's1', agent: str = 'personality') -> None:
    """SpeechEvent of the speech owner (tts_node) as face_node hands it to the director."""
    d.on_speech_event(SpeechEvent(kind=kind, speech_id=sid, agent=agent), now)


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


def test_speech_event_speaking_with_hang_between_replies():
    """PR-15 15-0b: «speaking» = a reply sounds (started ... finished); the hang bridges the replies of one answer."""
    d = awake()
    now = T0 + 8
    speech(d, STARTED, now)
    assert d.choose(now)[0] == 'talk_jaw'
    speech(d, FINISHED, now + 1.0)
    assert d.choose(now + 1.3)[0] == 'talk_jaw'          # 0.6 s hang: no flicker between replies
    speech(d, STARTED, now + 1.35, sid='s2')
    assert d.choose(now + 2.5)[0] == 'talk_jaw'
    speech(d, FINISHED, now + 3.0, sid='s2')
    assert d.choose(now + 3.7)[0] == 'idle_stare'


def test_synthesis_without_sound_is_not_speaking():
    """queued / synthesized (pre-synthesis of a performance minutes ahead) does not open the talking face."""
    d = awake()
    speech(d, QUEUED, T0 + 8)
    speech(d, SYNTHESIZED, T0 + 8.5)
    assert d.choose(T0 + 9)[0] == 'idle_stare'


def test_rejected_reply_ends_speech_like_finished():
    d = awake()
    speech(d, STARTED, T0 + 8)
    speech(d, REJECTED, T0 + 9)
    assert d.choose(T0 + 9.7)[0] == 'idle_stare'


def test_any_agent_speaks_and_overlapping_replies_end_with_the_last_one():
    """The speaker plays every agent (personality, operator, telegram /say, system): the face talks for all of them."""
    d = awake()
    speech(d, STARTED, T0 + 8, sid='p')
    speech(d, STARTED, T0 + 8.2, sid='op', agent='operator')
    speech(d, FINISHED, T0 + 9, sid='p')
    assert d.choose(T0 + 10)[0] == 'talk_jaw'             # operator still sounds
    speech(d, FINISHED, T0 + 10.5, sid='op', agent='operator')
    assert d.choose(T0 + 11.2)[0] == 'idle_stare'


def test_cancelled_ends_speech_at_once_and_shuts_the_mouth():
    d = awake()
    now = T0 + 8
    d.on_envelope('s1', now, 40, [0.3] * 100, [], now)
    speech(d, STARTED, now)
    for k in range(5):
        d.choose(now + 0.04 * k)
    assert d.mouth.level(now + 0.2)[0] > 0.5
    speech(d, CANCELLED, now + 0.25)
    assert d.mouth.level(now + 0.26)[0] < 0.05
    assert not d.speaking(now + 0.26)


def test_cancel_of_one_reply_keeps_the_face_talking_while_another_sounds():
    d = awake()
    speech(d, STARTED, T0 + 8, sid='p')
    speech(d, STARTED, T0 + 8.1, sid='op', agent='operator')
    speech(d, CANCELLED, T0 + 8.5, sid='p')
    assert d.speaking(T0 + 8.6)


def test_lost_end_of_reply_does_not_keep_the_face_talking_forever():
    """No finished (event lost): after SpeakingWatch's stale limit the face is idle again."""
    from rob_box_core.speech_client import SPEAKING_STALE_S

    d = awake()
    speech(d, STARTED, T0 + 8)
    assert d.speaking(T0 + 8 + SPEAKING_STALE_S - 1)
    assert not d.speaking(T0 + 8 + SPEAKING_STALE_S + 1)


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
    speech(d, CANCELLED, now + 0.2)
    assert d.mouth.level(now + 0.24)[0] == 0.0             # mouth shut within one frame (A25: <= 80 ms)
    assert d.choose(now + 0.24)[0] == 'idle_stare'         # and the face stops talking at once


def test_mouth_design_parameter():
    d = awake(mouth_design='voicebox')
    speech(d, STARTED, T0 + 8)
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


def test_4083_request_without_seconds_holds_the_dialogue_show_default():
    """Запрос без ``ttl_ms`` держится столько же, сколько показ по просьбе в диалоге (одно число в knowledge)."""
    from rob_box_dialog.knowledge import FACE_SHOW_DEFAULT_S, FACE_SHOW_MIN_S

    assert FACE_SHOW_DEFAULT_S >= FACE_SHOW_MIN_S
    d = awake()
    assert d.request('sad', None, T0 + 8) == 'sad'
    assert d.choose(T0 + 8 + FACE_SHOW_DEFAULT_S - 0.1)[0] == 'sad'
    assert d.choose(T0 + 8 + FACE_SHOW_DEFAULT_S + 0.1)[0] != 'sad'


def test_requested_emotion_while_speaking_becomes_the_tone():
    d = awake()
    d.request('happy', 5.0, T0 + 8)
    speech(d, STARTED, T0 + 8)
    key, ctx = d.choose(T0 + 8.5)
    assert key == 'talk_jaw' and ctx.mood == 'happy'
    speech(d, CANCELLED, T0 + 9)
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
    speech(d, STARTED, T0 + 8)
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


# -- тишина (#3984, решение Шифу 23.10): рот заклеен, пока в сессии сцена Silence -----------------------------------
def test_silence_shows_the_taped_mouth_instead_of_idle_and_ends_back_in_idle():
    d = awake()
    assert d.set_silenced(True) is True and d.set_silenced(True) is False
    assert d.silenced and d.choose(T0 + 9.0)[0] == 'taped_mouth'
    d.voice_seen(T0 + 399.0)
    assert d.choose(T0 + 400.0)[0] == 'taped_mouth'              # держится, пока Silence не снята (а не 8 с запроса)
    assert d.set_silenced(False) is True
    d.voice_seen(T0 + 400.5)
    assert d.choose(T0 + 401.0)[0] == 'idle_stare'


def test_entering_silence_takes_the_requested_scene_and_the_llm_scene_off():
    d = awake()
    d.request('listening', 8.0, T0 + 8.0)
    d.apply_face({'eyes': 'wide'}, 20.0, T0 + 8.0)
    assert d.choose(T0 + 8.5)[0] == 'face_dsl'
    d.set_silenced(True)
    assert d.choose(T0 + 8.6)[0] == 'taped_mouth'


def test_music_does_not_beat_the_taped_mouth():
    d = awake()
    d.set_silenced(True)
    d.on_music_state(json.dumps({'state': 'playing', 'dj': {'bpm': 118}}), T0 + 8.0)
    assert d.choose(T0 + 8.1)[0] == 'taped_mouth'


def test_taped_mouth_draws_tape_across_the_mouth_and_is_not_the_listening_face():
    from rob_box_animations.face.scene import Scene
    from rob_box_animations.face.states import SCENES

    taped, listening = Scene(SCENES['taped_mouth']).frame(1.0), Scene(SCENES['listening']).frame(1.0)
    lit = taped.mouth.sum(axis=2) > 0
    assert lit[1:4, 2:23].all() and not lit[4].any()             # полоса скотча на весь рот
    assert taped.mouth.max() <= 25 and taped.eye_l.max() <= 25   # уровни драйвера 0..25
    assert not np.array_equal(taped.mouth, listening.mouth)
