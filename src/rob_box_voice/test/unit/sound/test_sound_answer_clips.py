"""Карточка #5067: заглушки режима «отвечай звуками» читаются, и нода звука кладёт их в словарь.

Файлы — приёмочные заглушки, не голоса существ. Гард проверяет формат и то, что живой yaml
указывает на каталог, который в репо есть. Выбор клипа здесь не меняется.
"""

from __future__ import annotations

import wave
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import yaml

from rob_box_voice.core.sound_answer import load_manifest
from rob_box_voice.sound_node import load_answer_clips


def _repo_root() -> Path:
    """Корень репо: там лежит sound_pack. Счёт родителей здесь врёт на один уровень."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "sound_pack").is_dir() and (parent / "src").is_dir():
            return parent
    raise AssertionError(f"корень репо не найден от {here}")


_REPO = _repo_root()
_CLIP_DIR = _REPO / "sound_pack" / "sound_answer"
_LIVE = _REPO / "docker/sound/config/voice_assistant"
_MANIFEST = _REPO / "src/rob_box_voice/config/sound_answer_manifest.json"
_IDS = ("bird", "seal", "elephant", "alligator", "growl")
_LIVE_DIR = "/ws/sound_pack/sound_answer"


def _wav(path: Path):
    with wave.open(str(path)) as handle:
        return (
            handle.getnchannels(),
            handle.getframerate(),
            handle.getsampwidth(),
            handle.getnframes(),
        )


def test_five_stub_wavs_are_short_16khz_mono():
    missing = [name for name in _IDS if not (_CLIP_DIR / f"{name}.wav").is_file()]
    assert missing == [], f"нет заглушек: {missing}"
    for name in _IDS:
        channels, rate, width, frames = _wav(_CLIP_DIR / f"{name}.wav")
        duration = frames / rate
        assert (channels, rate, width) == (1, 16000, 2), name
        assert 0.3 <= duration < 2.0, f"{name}: {duration:.3f} с"


def test_live_yaml_points_at_the_mounted_clip_dir():
    """Launch читает /config/voice_assistant, а это смонтированный docker/sound/config."""
    assert _CLIP_DIR.is_dir()
    for name in ("dialog_node.yaml", "sound_node.yaml"):
        body = yaml.safe_load((_LIVE / name).read_text(encoding="utf-8"))
        params = body[name.replace(".yaml", "")]["ros__parameters"]
        assert params["sound_answer_clip_dir"] == _LIVE_DIR


def test_load_answer_clips_puts_ready_files_into_sounds(tmp_path):
    """Без этой загрузки select_sound пропустит клип, а play_sound_thread не найдёт его в словаре."""
    for name in ("bird", "seal"):
        source = _CLIP_DIR / f"{name}.wav"
        (tmp_path / f"{name}.wav").write_bytes(source.read_bytes())
    node = MagicMock()
    node.sound_answer_manifest = str(_MANIFEST)
    node.sound_answer_clip_dir = str(tmp_path)
    node.volume_db = 0
    node.sounds = {}

    loaded = load_answer_clips(node)

    assert node.get_logger().error.call_args_list == []
    assert loaded == 2
    assert set(node.sounds) == {"bird", "seal"}
    assert all(node.sounds[name].frame_rate == 16000 for name in ("bird", "seal"))


def test_load_answer_clips_skips_a_missing_file_without_error(tmp_path):
    node = MagicMock()
    node.sound_answer_manifest = str(_MANIFEST)
    node.sound_answer_clip_dir = str(tmp_path)
    node.volume_db = 0
    node.sounds = {}

    assert load_answer_clips(node) == 0
    assert node.sounds == {}
    node.get_logger().error.assert_not_called()
    assert len(load_manifest(str(_MANIFEST)).clips) == 5


@pytest.mark.parametrize("name", _IDS)
def test_stub_name_matches_the_manifest(name):
    clip = load_manifest(str(_MANIFEST)).by_id(name)
    assert clip is not None and clip.file == f"{name}.wav"
