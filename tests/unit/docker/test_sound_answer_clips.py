"""Заглушки режима «отвечай звуками» (#5067) лежат в звуковом пакете репо и на них указывает живой yaml.

Тест звукового узла (``rob_box_voice/test``) в CI видит только установленный пакет без корня репо
(``test_ws/src``), поэтому наличие файлов и путь в yaml держим здесь, где корень репо есть.
Файлы — приёмочные заглушки, не голоса существ. Выбор клипа здесь не проверяется.
"""

from __future__ import annotations

import wave
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
CLIP_DIR = REPO / "sound_pack" / "sound_answer"
LIVE = REPO / "docker/sound/config/voice_assistant"
MANIFEST = REPO / "src/rob_box_voice/config/sound_answer_manifest.json"
IDS = ("bird", "seal", "elephant", "alligator", "growl")
LIVE_DIR = "/ws/sound_pack/sound_answer"


def _wav(path: Path):
    with wave.open(str(path)) as handle:
        return (
            handle.getnchannels(),
            handle.getframerate(),
            handle.getsampwidth(),
            handle.getnframes(),
        )


def test_five_stub_wavs_are_short_16khz_mono():
    missing = [name for name in IDS if not (CLIP_DIR / f"{name}.wav").is_file()]
    assert missing == [], f"нет заглушек: {missing}"
    for name in IDS:
        channels, rate, width, frames = _wav(CLIP_DIR / f"{name}.wav")
        duration = frames / rate
        assert (channels, rate, width) == (1, 16000, 2), name
        assert 0.3 <= duration < 2.0, f"{name}: {duration:.3f} с"


def test_live_yaml_points_at_the_mounted_clip_dir():
    """Launch читает /config/voice_assistant, а это смонтированный docker/sound/config."""
    assert CLIP_DIR.is_dir()
    for name in ("dialog_node.yaml", "sound_node.yaml"):
        body = yaml.safe_load((LIVE / name).read_text(encoding="utf-8"))
        params = body[name.replace(".yaml", "")]["ros__parameters"]
        assert params["sound_answer_clip_dir"] == LIVE_DIR


def test_stub_names_match_the_manifest():
    import json

    clips = {
        row["id"]: row["file"]
        for row in json.loads(MANIFEST.read_text(encoding="utf-8"))["clips"]
    }
    assert {name: f"{name}.wav" for name in IDS} == {name: clips[name] for name in IDS}
