"""ADR-0158 PR-6: пресеты пения на все системные голоса + синтез вокала в файл.

Pure Python — без ROS/torch/grpc: бэкенды подменяются фейками.
"""

from __future__ import annotations

import sys
import wave
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice import sing_synth  # noqa: E402
from rob_box_voice.sing_synth import SingSynthError, synth_vocal_to_file  # noqa: E402
from rob_box_voice.tts_voice_registry import (  # noqa: E402
    _YANDEX_ROLES,
    MINIMAX_EMOTIONS,
    PROVIDER_VOICES,
    SING_PRESETS,
    SingPresetUnknown,
    sing_preset_for,
)

ALL_VOICES = [(p, v) for p, vs in PROVIDER_VOICES.items() for v in vs]


def test_catalog_size_is_what_adr_says() -> None:
    assert {p: len(v) for p, v in PROVIDER_VOICES.items()} == {"yandex": 13, "minimax": 10, "silero": 5}


def test_table_covers_exactly_the_catalog() -> None:
    assert set(SING_PRESETS) == set(ALL_VOICES)


@pytest.mark.parametrize("provider,voice", ALL_VOICES)
def test_every_voice_has_valid_preset(provider: str, voice: str) -> None:
    p = sing_preset_for(provider, voice)
    assert (p.provider, p.voice) == (provider, voice)
    assert p.vocal_gender in ("m", "f")
    assert 0.5 <= p.speed <= 2.0 and 0.0 <= p.volume <= 10.0
    assert 0.0 < p.audio_weight <= 1.0
    assert p.tags_hint and p.language
    if provider == "minimax":
        assert p.emotion in MINIMAX_EMOTIONS
    elif provider == "yandex":
        assert p.emotion is None or p.emotion in _YANDEX_ROLES[voice]
    else:  # silero не умеет подачу
        assert (p.emotion, p.speed, p.pitch, p.volume) == (None, 1.0, 0, 1.0)


def test_reference_reliable_man() -> None:
    p = sing_preset_for("minimax", "Russian_ReliableMan")
    assert (p.emotion, p.audio_weight, p.vocal_gender) == ("angry", 0.8, "m")


@pytest.mark.parametrize("key", [("minimax", "nope"), ("nope", "anton"), ("yandex", "")])
def test_unknown_voice_is_honest_error(key) -> None:
    with pytest.raises(SingPresetUnknown):
        sing_preset_for(*key)


class _Fake:
    def __init__(self, provider: str, fmt: str, data: bytes, dur=1.5) -> None:
        self.provider, self.fmt, self._data, self._dur = provider, fmt, data, dur
        self.seen = []

    def synth(self, text, preset):
        self.seen.append((text, preset))
        return self._data, 44100, self._dur


def test_synth_writes_file_with_backend_suffix(tmp_path) -> None:
    fake = _Fake("minimax", "mp3", b"ID3data")
    r = synth_vocal_to_file("minimax", "Russian_ReliableMan", "Я робот", tmp_path / "v.wav",
                            backends={"minimax": fake})
    assert r.path.endswith("v.mp3") and Path(r.path).read_bytes() == b"ID3data"
    assert (r.size_bytes, r.duration_s, r.fmt) == (7, 1.5, "mp3")
    assert fake.seen[0][1] is SING_PRESETS[("minimax", "Russian_ReliableMan")]


def test_unknown_voice_never_reaches_backend(tmp_path) -> None:
    fake = _Fake("yandex", "wav", b"x")
    with pytest.raises(SingPresetUnknown):
        synth_vocal_to_file("yandex", "ghost", "текст", tmp_path / "v", backends={"yandex": fake})
    assert fake.seen == [] and not list(tmp_path.iterdir())


@pytest.mark.parametrize("text", ["", "   \n"])
def test_empty_text_rejected(tmp_path, text) -> None:
    with pytest.raises(SingSynthError) as e:
        synth_vocal_to_file("silero", "aidar", text, tmp_path / "v",
                            backends={"silero": _Fake("silero", "wav", b"x")})
    assert e.value.reason == "empty_text"


def test_missing_backend_and_empty_audio(tmp_path) -> None:
    with pytest.raises(SingSynthError) as e:
        synth_vocal_to_file("silero", "aidar", "да", tmp_path / "v", backends={})
    assert e.value.reason == "no_backend"
    with pytest.raises(SingSynthError) as e:
        synth_vocal_to_file("silero", "aidar", "да", tmp_path / "v",
                            backends={"silero": _Fake("silero", "wav", b"")})
    assert e.value.reason == "empty_audio" and not list(tmp_path.iterdir())


def test_missing_minimax_key_is_loud(monkeypatch, tmp_path) -> None:
    monkeypatch.delenv("MINIMAX_API_KEY", raising=False)
    pytest.importorskip("rob_box_llm")
    with pytest.raises(SingSynthError) as e:
        synth_vocal_to_file("minimax", "Russian_ReliableMan", "да", tmp_path / "v")
    assert e.value.reason == "no_api_key"


def test_missing_yandex_key_is_loud(monkeypatch, tmp_path) -> None:
    pytest.importorskip("yandex.cloud.ai.tts.v3")
    monkeypatch.delenv("YANDEX_API_KEY", raising=False)
    with pytest.raises(SingSynthError) as e:
        synth_vocal_to_file("yandex", "anton", "да", tmp_path / "v")
    assert e.value.reason == "no_api_key"


class _FakeTensor:
    def __init__(self, arr) -> None:
        self._a = arr

    def numpy(self):
        return self._a


class _FakeSilero:
    def __init__(self) -> None:
        self.calls = []

    def apply_tts(self, **kw):
        import numpy as np

        self.calls.append(kw)
        return _FakeTensor(np.full(4800, 0.25, dtype="float32"))


def test_minimax_backend_passes_preset_and_reports_requested_rate(tmp_path) -> None:
    pytest.importorskip("rob_box_llm")
    from rob_box_llm.tts import TTSAudio, TTSFormat

    seen = {}

    class _Prov:
        async def synthesize(self, text, *, settings):
            seen["s"] = settings
            # как реальный провайдер: частоту API не прислал -> 32000
            return TTSAudio(samples=b"ID3xx", sample_rate=32000, format=TTSFormat.MP3,
                            raw={"extra_info": {"audio_length": 2500}})

    r = synth_vocal_to_file("minimax", "Russian_ReliableMan", "Я робот", tmp_path / "m",
                            backends={"minimax": sing_synth.MiniMaxSingBackend(provider=_Prov())})
    s = seen["s"]
    assert (s.voice, s.emotion, s.sample_rate, s.format) == ("Russian_ReliableMan", "angry", 44100, TTSFormat.MP3)
    assert (r.sample_rate, r.duration_s, r.fmt) == (44100, 2.5, "mp3")


def test_silero_backend_writes_real_wav(tmp_path) -> None:
    pytest.importorskip("numpy")
    model = _FakeSilero()
    r = synth_vocal_to_file("silero", "baya", "Раз. Два. Три.", tmp_path / "s",
                            backends={"silero": sing_synth.SileroSingBackend(model=model)})
    assert model.calls and all(c["speaker"] == "baya" for c in model.calls)
    with wave.open(r.path, "rb") as w:
        assert (w.getframerate(), w.getnchannels(), w.getsampwidth()) == (48000, 1, 2)
        assert w.getnframes() > 0
    assert r.duration_s and r.duration_s > 0
