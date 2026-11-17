"""#3930 — потоковый синтез MiniMax в динамик: буфер, срезка начальной тишины, воспроизведение, барж-ин."""

from __future__ import annotations

import threading
import time
import types
from unittest.mock import MagicMock

import numpy as np
import pytest

from rob_box_voice import audio_playback_manager as apm
from rob_box_voice.core import tts_stream
from rob_box_voice.core.tts_stream import PcmStream

SR = 16000


def _loud(seconds: float, level: float = 0.3) -> np.ndarray:
    return np.full(int(seconds * SR), level, dtype=np.float32)


def _quiet(seconds: float) -> np.ndarray:
    return np.zeros(int(seconds * SR), dtype=np.float32)


# -- буфер -----------------------------------------------------------------------


def test_lead_silence_is_cut_to_keep_and_the_rest_stays():
    """Первый кусок MiniMax — 0.35 с тишины перед речью (спайк): играем от 50 мс до первого звука."""
    s = PcmStream(SR)
    s.put(np.concatenate([_quiet(0.34), _loud(0.01)]))
    s.put(_quiet(0.2))  # пауза ПОСЛЕ первого звука — речь, не трогаем
    assert len(s) == round((tts_stream.LEAD_KEEP_S + 0.01 + 0.2) * SR)


def test_lead_trim_never_eats_more_than_the_cap():
    """Тихое начало дольше потолка (шёпот, пауза ``<#x#>``) срезается не больше ``LEAD_MAX_TRIM_S``."""
    s = PcmStream(SR)
    s.put(_quiet(0.3))
    s.put(_quiet(0.4))
    assert len(s) == round((0.7 - tts_stream.LEAD_MAX_TRIM_S) * SR)


def test_wait_ready_returns_at_prebuffer_and_read_spans_pieces():
    s = PcmStream(SR, prebuffer_s=0.5)
    s.put(_loud(0.3, 0.1))
    threading.Timer(0.05, lambda: s.put(_loud(0.3, 0.2))).start()
    s.wait_ready(2.0)
    assert len(s) == int(0.6 * SR)
    out = s.read(int(0.4 * SR))
    assert out.size == int(0.4 * SR) and out[0] == pytest.approx(0.1) and out[-1] == pytest.approx(0.2)
    assert s.read(SR).size == int(0.2 * SR) and s.read(SR).size == 0  # больше пока нет


def test_wait_ready_gives_up_at_timeout_without_audio():
    s = PcmStream(SR)
    s.wait_ready(0.05)
    assert len(s) == 0 and not s.done


def test_cancel_stops_the_producer_and_wakes_waiters():
    s = PcmStream(SR)
    future = MagicMock()
    s.future = future
    threading.Timer(0.05, s.cancel).start()
    s.wait_ready(5.0)
    assert s.cancelled and future.cancel.called
    assert s.put(_loud(0.1)) is False and len(s) == 0


def test_pieces_since_reports_offsets_for_the_envelope():
    s = PcmStream(SR)
    s.put(_loud(0.25))
    s.put(_loud(0.5))
    fresh, nxt = s.pieces_since(1)
    assert nxt == 2 and [round(off, 3) for off, _ in fresh] == [0.25]


# -- play_local --------------------------------------------------------------------


class _Owner:
    def __init__(self, start_ok=True, cancelled=False):
        self.start_ok, self.cancelled = start_ok, cancelled
        self.calls = []

    def started(self, speech_id, audio, sample_rate, duration_s=None):
        self.calls.append(("started", len(audio), duration_s))
        return self.start_ok

    def streamed(self, speech_id, audio, sample_rate, offset_s, total_s=None):
        self.calls.append(("streamed", None if audio is None else len(audio), round(offset_s, 3), total_s))

    def is_cancelled(self, speech_id):
        return self.cancelled


def _node():
    return types.SimpleNamespace(get_logger=MagicMock())


def test_play_local_started_before_sound_then_envelopes_then_duration():
    s = PcmStream(SR)
    s.put(_loud(0.5))
    owner = _Owner()

    def playback(source, rate, *, gain, poll, should_stop):
        assert owner.calls[0] == ("started", int(0.5 * SR), 0.0)  # started — до первого кадра, длина неизвестна
        source.put(_loud(1.0))
        source.finish()
        poll()
        return "done", 0

    outcome, duration = tts_stream.play_local(_node(), s, "sid", 1.0, owner, playback)
    assert (outcome, duration) == ("done", 1.5)
    assert owner.calls[1:] == [("streamed", SR, 0.5, None), ("streamed", None, 0.0, 1.5)]


def test_play_local_mid_stream_break_is_reported_not_finished():
    s = PcmStream(SR)
    s.put(_loud(0.5))
    s.finish(tts_stream.BROKEN)
    node = _node()
    outcome, _ = tts_stream.play_local(node, s, "sid", 1.0, _Owner(), lambda *a, **k: ("done", 0))
    assert outcome == tts_stream.BROKEN
    assert node.get_logger().warning.called


def test_play_local_cancelled_at_start_plays_nothing():
    s = PcmStream(SR)
    s.put(_loud(0.5))
    played = []
    outcome, _ = tts_stream.play_local(_node(), s, "sid", 1.0, _Owner(start_ok=False), lambda *a, **k: played.append(1))
    assert outcome == "cancelled" and played == [] and s.cancelled


def test_play_local_logs_underrun_honestly():
    s = PcmStream(SR)
    s.put(_loud(0.5))
    s.finish()
    node = _node()
    tts_stream.play_local(node, s, "sid", 1.0, _Owner(), lambda *a, **k: ("done", 1600))
    assert "100 мс" in node.get_logger().warning.call_args.args[0]


def test_play_local_stop_flag_of_node_stops_stream():
    """STOP пришёл между проверкой ``stop_requested`` и стартом потока: флаг ноды всё равно режет звук."""
    s = PcmStream(SR)
    s.put(_loud(0.5))
    node = _node()
    node.stop_requested = True
    seen = []

    def playback(source, rate, *, gain, poll, should_stop):
        seen.append(should_stop())
        return "stopped", 0

    outcome, _ = tts_stream.play_local(node, s, "sid", 1.0, _Owner(), playback)
    assert seen == [True] and outcome == "stopped" and s.cancelled


def test_drop_cancels_only_streams():
    s = PcmStream(SR)
    tts_stream.drop(s)
    tts_stream.drop(np.zeros(10, dtype=np.float32))
    tts_stream.drop(None)
    assert s.cancelled


def test_allowed_only_for_speaker_without_chipmunk():
    plain, chip = types.SimpleNamespace(), types.SimpleNamespace(chipmunk_mode=True)
    assert tts_stream.allowed(plain, "speaker")
    assert not tts_stream.allowed(plain, "headset")
    assert not tts_stream.allowed(chip, "speaker")

# -- AudioPlaybackManager.play_stream --------------------------------------------------


class _FakeOutputStream:
    """sounddevice.OutputStream: callback зовётся блоками по 1600 кадров в отдельном потоке."""

    def __init__(self, samplerate, channels, dtype, device, callback, finished_callback):
        self.callback, self.finished = callback, finished_callback
        self.blocks, self.aborted = [], False
        self._thread = threading.Thread(target=self._run)

    def _run(self):
        while not self.aborted:
            out = np.ones((1600, 2), dtype=np.float32)
            try:
                self.callback(out, 1600, None, None)
            except _CallbackStop:
                self.blocks.append(out.copy())
                break
            self.blocks.append(out.copy())
            if len(self.blocks) > 50:
                break
            time.sleep(0.01)
        self.finished()

    def abort(self):
        self.aborted = True

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._thread.join(timeout=2)


class _CallbackStop(Exception):
    pass


@pytest.fixture
def fake_sd(monkeypatch):
    made = []

    def factory(**kw):
        made.append(_FakeOutputStream(**kw))
        return made[-1]

    monkeypatch.setattr(apm, "sd", types.SimpleNamespace(OutputStream=factory, CallbackStop=_CallbackStop))
    return made


def _manager():
    m = apm.AudioPlaybackManager.__new__(apm.AudioPlaybackManager)
    m._playback_lock, m._current_stream, m._abort_stream = threading.Lock(), None, threading.Event()
    return m


def test_play_stream_plays_buffer_stereo_and_zero_fills_the_tail(fake_sd):
    s = PcmStream(SR)
    s.put(_loud(0.25, 0.5))  # 4000 кадров = 2.5 блока
    s.finish()
    outcome, underrun = _manager().play_stream(s, SR, gain=0.5)
    blocks = fake_sd[0].blocks
    assert (outcome, underrun) == ("done", 0)
    audio = np.concatenate(blocks)
    assert audio.shape[1] == 2 and np.allclose(audio[:4000], 0.25) and np.allclose(audio[4000:], 0.0)


def test_play_stream_counts_underrun_as_silence(fake_sd):
    s = PcmStream(SR)
    s.put(_loud(0.1))  # 1600 кадров, потом пусто, потом конец
    threading.Timer(0.3, s.finish).start()
    outcome, underrun = _manager().play_stream(s, SR)
    assert outcome == "done" and underrun > 0 and underrun % 1600 == 0


def test_abort_stream_stops_playback_within_a_poll(fake_sd):
    s = PcmStream(SR)
    s.put(_loud(0.1))
    m = _manager()
    threading.Timer(0.05, m.abort_stream).start()
    outcome, _ = m.play_stream(s, SR)
    assert outcome == "stopped" and fake_sd[0].aborted


def test_abort_helper_tolerates_stub_nodes_without_manager():
    tts_stream.abort(types.SimpleNamespace())
    node = types.SimpleNamespace(playback_manager=MagicMock())
    tts_stream.abort(node)
    assert node.playback_manager.abort_stream.called
