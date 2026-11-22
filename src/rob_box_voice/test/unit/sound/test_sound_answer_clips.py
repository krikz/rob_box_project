"""Карточка #5067: нода звука кладёт готовые клипы режима в словарь.

Наличие файлов в репо и путь в живом yaml держит ``tests/unit/docker``: сборка пакета
в CI копирует только пакет, без корня репо. Выбор клипа здесь не меняется.
"""

from __future__ import annotations

import json
import wave
from pathlib import Path
from unittest.mock import MagicMock

from rob_box_voice.core.sound_answer import load_manifest
from rob_box_voice.sound_node import load_answer_clips

_MANIFEST = (
    Path(__file__).resolve().parents[3] / "config" / "sound_answer_manifest.json"
)


def _stub(path: Path, rate: int = 16000, frames: int = 1600) -> None:
    with wave.open(str(path), "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(rate)
        handle.writeframes(b"\x00\x00" * frames)


def test_load_answer_clips_puts_ready_files_into_sounds(tmp_path):
    """Без этой загрузки select_sound пропустит клип, а play_sound_thread не найдёт его в словаре."""
    for name in ("bird", "seal"):
        _stub(tmp_path / f"{name}.wav")
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


def test_manifest_next_to_the_package_names_the_five_stub_files():
    body = json.loads(_MANIFEST.read_text(encoding="utf-8"))
    assert {row["id"]: row["file"] for row in body["clips"]} == {
        "bird": "bird.wav",
        "seal": "seal.wav",
        "elephant": "elephant.wav",
        "alligator": "alligator.wav",
        "growl": "growl.wav",
    }
