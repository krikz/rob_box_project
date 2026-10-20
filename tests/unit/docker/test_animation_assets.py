"""Guards for LED animation assets baked into the voice-assistant image.

``src/rob_box_animations/animations`` копируется в образ voice-assistant и
проигрывается ``animation_player`` как есть. Кадры и манифесты генерирует
``tools/led_face_lab/bake_manifests.py``; этот тест держит то, что плеер и
композитор ros2leds примут без молчаливых потерь:

* у каждой анимации все пять логических групп композитора;
* кадры каждой панели в сумме дают ``duration_ms`` (иначе панели
  рассинхронизируются на втором круге);
* каждая картинка существует, это 8-битный RGB PNG нужного размера —
  композитор отбрасывает изображение не того размера с WARN и панель
  застывает;
* в ``frames/`` нет файлов, на которые не ссылается ни один манифест.

PIL в job линта нет — размеры читаются из заголовка PNG (IHDR).
"""

from __future__ import annotations

import struct
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
ANIM = REPO_ROOT / "src" / "rob_box_animations" / "animations"
GROUP_SIZE = {
    "main_display": (25, 5),
    "wheel_front_left": (8, 8),
    "wheel_front_right": (8, 8),
    "wheel_rear_left": (8, 8),
    "wheel_rear_right": (8, 8),
}
MANIFESTS = sorted((ANIM / "manifests").glob("*.yaml"))


def png_header(path: Path):
    """(width, height, bit_depth, colour_type) из IHDR."""
    raw = path.read_bytes()[:33]
    assert raw[:8] == b"\x89PNG\r\n\x1a\n", f"{path} is not a PNG"
    assert raw[12:16] == b"IHDR", f"{path}: IHDR is not the first chunk"
    width, height, depth, colour = struct.unpack(">IIBB", raw[16:26])
    return width, height, depth, colour


def load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_manifest_set_is_not_empty():
    assert len(MANIFESTS) == 21


@pytest.mark.parametrize("path", MANIFESTS, ids=lambda p: p.stem)
def test_manifest_drives_every_panel_in_sync(path):
    m = load(path)
    assert m["name"] == path.stem
    groups = [p["logical_group"] for p in m["panels"]]
    assert sorted(groups) == sorted(GROUP_SIZE), groups
    for panel in m["panels"]:
        total = sum(f["duration_ms"] for f in panel["frames"])
        assert total == m["duration_ms"], (panel["logical_group"], total)


@pytest.mark.parametrize("path", MANIFESTS, ids=lambda p: p.stem)
def test_every_frame_is_an_rgb_png_of_the_panel_size(path):
    m = load(path)
    for panel in m["panels"]:
        want = GROUP_SIZE[panel["logical_group"]]
        for frame in panel["frames"]:
            img = ANIM / frame["image"]
            assert img.exists(), img
            width, height, depth, colour = png_header(img)
            assert (width, height) == want, (img, (width, height))
            assert (depth, colour) == (8, 2), (img, "expected 8-bit RGB")


def test_no_orphan_frames():
    used = {
        (ANIM / f["image"]).resolve()
        for path in MANIFESTS
        for panel in load(path)["panels"]
        for f in panel["frames"]
    }
    on_disk = {p.resolve() for p in (ANIM / "frames").rglob("*.png")}
    orphans = sorted(str(p.relative_to(ANIM.resolve())) for p in on_disk - used)
    assert not orphans, orphans[:10]
