#!/usr/bin/env python3
"""Набор для сторис Шифу (воронка подписчиков): вертикальное MP4 1080x1920 и статичный PNG в стиле канала.

Публикует сам Шифу из своего аккаунта; бот сторис не трогает. Сверху «РОББОКС» и строка-крючок, в середине
кадр ролика, внизу ~420 px свободны под стикер-ссылку Telegram (там же интерфейс сторис закрывает края).

    python -X utf8 scripts/telegram/make_story.py --out OUT_DIR [--video ролик.mp4 --start 5 --duration 20]
                                                  [--hook "Строка-крючок"]

Без --video делает только статичную карточку без кадра. Длительность видео — 15–30 с (границы проверяются).
Крючок должен совпадать с тем, что есть в посте: честность — как в docs/process/telegram-channel.md.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from infographic import wrap  # noqa: E402
from make_channel_media import BG, DIM, GREEN, PANEL, RAINBOW, font  # noqa: E402

W, H = 1080, 1920
SAFE_BOTTOM = 420  # свободно под стикер-ссылку
VIDEO_Y = 780
MIN_SEC, MAX_SEC = 15, 30
DEFAULT_HOOK = "Как РОББОКС снимает карту склада — в симуляции"


def clamp_duration(seconds: float, available: float | None = None) -> float:
    """Длительность сторис-ролика: 15–30 с, не больше длины исходника."""
    if available is not None and available < MIN_SEC:
        raise ValueError(f"в исходнике {available:.1f} с, нужно хотя бы {MIN_SEC}")
    top = MAX_SEC if available is None else min(MAX_SEC, available)
    return max(MIN_SEC, min(seconds, top))


def background(hook: str, frame: Image.Image | None = None) -> Image.Image:
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([40, 150, W - 40, H - 40], radius=40, fill=PANEL)
    for c in range((W - 160) // 38):  # светодиодная полоска, тот же приём, что у аватара и обложек
        x = 80 + c * 38
        d.ellipse([x, 230, x + 20, 250], fill=RAINBOW[c % len(RAINBOW)] if c % 3 else (40, 42, 50))
    d.text((80, 300), "РОББОКС", font=font(170), fill=GREEN)
    y = 520
    for line in wrap(d, hook, font(64), W - 160)[:3]:
        d.text((80, y), line, font=font(64), fill=(235, 238, 245))
        y += 82
    if frame is not None:
        frame = frame.convert("RGB")
        frame = frame.resize((W - 80, round(frame.height * (W - 80) / frame.width)), Image.LANCZOS)
        img.paste(frame, (40, VIDEO_Y))
    else:
        for r in range(6):  # без кадра ролика: набор светодиодов и подпись, ничего не выдумываем
            for c in range(18):
                x, y = 90 + c * 52, VIDEO_Y + 40 + r * 52
                d.ellipse([x, y, x + 34, y + 34], fill=RAINBOW[(c + r) % len(RAINBOW)] if (c * r) % 5 else (40, 42, 50))
        d.text((80, VIDEO_Y + 400), "Ролик — в канале", font=font(72), fill=(235, 238, 245))
    d.text((80, H - SAFE_BOTTOM - 70), "t.me/robbox_project", font=font(44), fill=DIM)
    return img


def probe_duration(video: Path) -> float:
    ffprobe = shutil.which("ffprobe") or "ffprobe"
    out = subprocess.run(
        [ffprobe, "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(video)],
        check=True,
        capture_output=True,
        text=True,
    )
    return float(out.stdout.strip())


def make_video(video: Path, out: Path, hook: str, start: float, duration: float) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise SystemExit("нужен ffmpeg в PATH")
    bg_path = out.with_suffix(".bg.png")
    background(hook).save(bg_path)
    vf = f"[1:v]scale={W - 80}:-2,fps=30[v];[0:v][v]overlay=40:{VIDEO_Y}:shortest=1,format=yuv420p"
    cmd = [ffmpeg, "-y", "-loop", "1", "-i", str(bg_path), "-ss", str(start), "-t", str(duration), "-i", str(video)]
    cmd += ["-filter_complex", vf, "-map", "1:a?", "-c:v", "libx264", "-crf", "23", "-preset", "medium"]
    cmd += ["-c:a", "aac", "-b:a", "128k", "-movflags", "+faststart", "-t", str(duration), str(out)]
    subprocess.run(cmd, check=True, capture_output=True)
    bg_path.unlink()


def middle_frame(video: Path, at: float) -> Image.Image:
    tmp = video.with_suffix(".frame.png")
    subprocess.run(
        [shutil.which("ffmpeg") or "ffmpeg", "-y", "-ss", str(at), "-i", str(video), "-frames:v", "1", str(tmp)],
        check=True,
        capture_output=True,
    )
    img = Image.open(tmp).copy()
    tmp.unlink()
    return img


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--video", type=Path)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--hook", default=DEFAULT_HOOK)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    frame = None
    if args.video:
        dur = clamp_duration(args.duration, probe_duration(args.video) - args.start)
        make_video(args.video, args.out / "story.mp4", args.hook, args.start, dur)
        frame = middle_frame(args.video, args.start + dur / 2)
        print(f"{args.out / 'story.mp4'}: {(args.out / 'story.mp4').stat().st_size} байт, {dur:g} с")
    background(args.hook, frame).save(args.out / "story-static.png", optimize=True)
    print(f"{args.out / 'story-static.png'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
