"""Трек кэша на деке владельца плеера (ADR-0158 §2.4, PR-2) — на фейковом Renardo, без ROS.

Тот же путь, что у синтеза: ``render_audio`` → ``PlayerOwner.play(once=True)`` → ``started`` по клоку → ``finished``
на конце файла; стоп снимает ноту файла группой и отдаёт буфер (кэш Renardo сам буферы не отпускает).
"""

import json
import os
import wave
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from rob_box_mcp_tools.engine.gen.audio import AudioUnavailable, audio_track, server_wav
from rob_box_mcp_tools.engine.player_owner import PlayerOwner
from rob_box_mcp_tools.engine.renardo_adapter import RenardoAdapter, audio_files
from rob_box_music.model import AudioTrack
from rob_box_music.render.audio import render_audio
from rob_box_voice.core.media_phrases import library_play_text, now_playing_track_text

from .test_engine_player_owner import FakeClock, FakePlayer

pytestmark = pytest.mark.unit

WAV = os.path.abspath("lib/abc/track.16k.wav")


class Samples:
    def __init__(self):
        self.loaded, self.freed = [], []

    def loadBuffer(self, filename, index=0, force=False):
        self.loaded.append(filename)
        return 0 if "missing" in filename else 42

    def free(self, filename):
        self.freed.append(filename)


def _deck():
    clock = FakeClock(beat=10.0)
    samples = Samples()
    ns = {"Clock": clock, "Samples": samples, "loop": lambda *a, **k: None, "var": lambda *a, **k: None,
          "c3": FakePlayer(clock),
          "d1": FakePlayer(clock), "p1": FakePlayer(clock)}
    sent, states, events = [], [], []
    adapter = RenardoAdapter(lambda: ns, lambda: frozenset(), lambda *a: sent.append(a))
    owner = PlayerOwner(adapter, states.append, events.append, clock=lambda: 1000.0)
    return SimpleNamespace(owner=owner, clock=clock, samples=samples, ns=ns, sent=sent, states=states,
                           events=[], raw=events)


def _fire(clock, until):
    """Исполнить запланированное на клоке до доли ``until`` (по порядку долей)."""
    while True:
        due = sorted((b, i) for i, (_, b) in enumerate(clock.scheduled) if b <= until)
        if not due:
            return
        _, i = due[0]
        fn, beat = clock.scheduled.pop(i)
        clock.beat = beat
        fn()


def _names(raw):
    return [json.loads(e)["event"] for e in raw]


def test_library_track_starts_finishes_and_frees_its_buffer():
    deck = _deck()
    program = render_audio(AudioTrack("gen_abc_1", WAV, 12.0), "A")
    facts = {"enabled": False, "title": "Космос", "source": "generated", "bpm": None}
    assert deck.owner.play(program, dj=facts, once=True)["ok"]
    assert deck.samples.loaded == [WAV]  # буфер — до exec, абсолютный путь как есть
    _fire(deck.clock, 24.0 + 0.5)  # форма встала на 12 (кратно 12), конец файла — 24
    assert _names(deck.raw) == ["started"]
    now = deck.owner.now_playing()
    assert now["playing"] and now["length_s"] == 12.0 and now["source"] == "generated"
    # темп деки (60 — доля = секунда) не выдаётся за темп трека: bpm неизвестен — о нём ни слова
    assert now_playing_track_text(now) == ("Сейчас играет «Космос» — сгенерированный трек из библиотеки. "
                                           "Играет уже 0 секунд, осталось 12 секунд.")
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        _fire(deck.clock, 36.0)
    assert _names(deck.raw) == ["started", "finished"]
    assert deck.samples.freed == [WAV] and not deck.owner.is_playing()
    assert ("/g_freeAll", 1) in deck.sent  # нота файла (sus = весь трек) снята группой


def test_user_stop_releases_the_file_and_a_new_track_does_not_play_over_it():
    deck = _deck()
    first = render_audio(AudioTrack("gen_a_1", WAV, 30.0), "A")
    deck.owner.play(first, once=True)
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        deck.owner.play(render_audio(AudioTrack("gen_b_2", WAV.replace("abc", "def"), 30.0), "A"), once=True)
        assert deck.samples.freed == [WAV]  # новый трек: нота прошлого файла снята, буфер отдан
        assert deck.sent.count(("/g_freeAll", 1)) == 1
        deck.owner.stop("user_stop")
    assert deck.samples.freed == [WAV, WAV.replace("abc", "def")]


def test_missing_wav_is_rejected_before_exec():
    deck = _deck()
    result = deck.owner.play(render_audio(AudioTrack("gen_m_1", os.path.abspath("missing.wav"), 5.0), "A"))
    assert result["ok"] is False and result["reason"] == "missing_sample"


def test_synth_programs_keep_no_audio_buffers():
    assert audio_files(SimpleNamespace(sample_files=frozenset({"dj_dave/x.wav"}))) == ()


def _fake_ffmpeg(seconds=2.0, calls=None):
    def run(cmd, **_):
        if calls is not None:
            calls.append(cmd)
        with wave.open(cmd[-1], "wb") as out:
            out.setnchannels(2)
            out.setsampwidth(2)
            out.setframerate(16000)
            out.writeframes(b"\x00\x00" * 2 * int(16000 * seconds))
    return run


def test_mp3_is_decoded_once_next_to_itself(tmp_path):
    mp3 = tmp_path / "track.mp3"
    mp3.write_bytes(b"ID3")
    calls = []
    track = audio_track({"id": "abc", "path": str(mp3), "bpm_nominal": 0}, "gen_abc_1", run=_fake_ffmpeg(2.0, calls))
    assert track.path == str(tmp_path / "track.16k.wav") and track.duration_s == 2.0 and track.bpm_nominal is None
    assert server_wav(str(mp3), run=_fake_ffmpeg(9.0, calls)) == track.path and len(calls) == 1
    assert not list(tmp_path.glob("*.part"))


def test_decode_failure_is_a_reason_not_silence(tmp_path):
    mp3 = tmp_path / "track.mp3"
    mp3.write_bytes(b"ID3")

    def broken(cmd, **_):
        raise OSError("ffmpeg not found")

    with pytest.raises(AudioUnavailable) as err:
        server_wav(str(mp3), run=broken)
    assert err.value.reason == "decode_failed"
    with pytest.raises(AudioUnavailable) as err:
        server_wav(str(tmp_path / "nope.mp3"), run=broken)
    assert err.value.reason == "no_file"


def test_launch_phrase_is_built_from_the_file():
    assert library_play_text("Космос", 89.4) == "Включаю «Космос» из библиотеки — 1 минута 29 секунд."
    assert library_play_text("", 3) == "Включаю трек из библиотеки — 3 секунды."
