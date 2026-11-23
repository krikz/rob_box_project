"""Нода звука берёт клипы Химеры из того же каталога, что профиль поставки (#5152).

``package_dir()`` — единственный путь. ``sound_pack_dir/chimera`` и ``/opt/rob_box/chimera`` сюда не входят:
два каталога снова разошлись бы (ADR-0148). Битый хеш и чужой wav в словарь не кладутся.
"""

from __future__ import annotations

import shutil
from pathlib import Path

from rob_box_mcp_tools.chimera_package import package_dir, verified_clip
from rob_box_voice.sound_node import load_chimera_clips

BIRD = "bird_chirps_16k_notail.wav"
SEAL = "seal_zap_16k_notail.wav"
REPO_CLIPS = Path(__file__).resolve().parents[5] / "sound_pack" / "chimera"


class _ClipNode:
    def __init__(self):
        self.sounds = {}
        self.volume_db = 0
        self.lines = []

    def get_logger(self):
        return self

    def info(self, line, *_a, **_k):
        self.lines.append(line)

    def error(self, line, *_a, **_k):
        self.lines.append(line)


def test_sound_node_loads_only_verified_clips_from_package_dir(tmp_path, monkeypatch):
    root = tmp_path / "chimera"
    shutil.copytree(REPO_CLIPS, root)
    seal = root / SEAL
    seal.write_bytes(seal.read_bytes()[:-1] + b"\x00")
    (root / "чужой.wav").write_bytes(b"RIFF")
    monkeypatch.setenv("CHIMERA_PACKAGE_DIR", str(root))

    node = _ClipNode()
    loaded = load_chimera_clips(node, package_dir())

    assert package_dir() == str(root)
    assert loaded == 4
    assert BIRD in node.sounds and SEAL not in node.sounds and "чужой.wav" not in node.sounds
    assert verified_clip(str(root), BIRD) is True
    assert verified_clip(str(root), SEAL) is False
    assert any(BIRD in line for line in node.lines)
