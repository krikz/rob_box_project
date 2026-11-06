"""Набор для сторис: размер 1080x1920, длительность 15-30 с, видео собирается ffmpeg-ом."""

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

Image = pytest.importorskip("PIL.Image", reason="нужен Pillow (pip install pillow)")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import make_story  # noqa: E402


def test_duration_is_clamped_to_15_30():
    assert make_story.clamp_duration(5) == 15
    assert make_story.clamp_duration(60) == 30
    assert make_story.clamp_duration(20, available=18) == 18
    with pytest.raises(ValueError):
        make_story.clamp_duration(20, available=10)


def test_static_card_is_vertical_with_free_bottom():
    img = make_story.background("Крючок")
    assert img.size == (1080, 1920)
    bottom = img.crop((100, 1920 - make_story.SAFE_BOTTOM + 40, 980, 1920 - 80))
    assert len(set(bottom.getdata())) == 1  # зона под стикер-ссылку пустая
    assert Image.open(REPO / "docs" / "channel" / "media" / "stories" / "story-static.png").size == (1080, 1920)


@pytest.mark.skipif(not shutil.which("ffmpeg"), reason="нет ffmpeg")
def test_video_is_1080x1920_h264(tmp_path):
    src = tmp_path / "src.mp4"
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "lavfi", "-i", "testsrc=s=640x360:d=16:r=25", str(src)], check=True
    )
    out = tmp_path / "story.mp4"
    make_story.make_video(src, out, "Крючок", 0, 15)
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "stream=codec_name,width,height", "-of", "csv=p=0", str(out)],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    assert probe == "h264,1080,1920"
    assert abs(make_story.probe_duration(out) - 15) < 0.2
