"""Сборка кадров в файл для Telegram: MP4 (ffmpeg) или GIF (PIL) в пределах лимитов.

Лимиты решения Шифу (#4513): MP4 не больше 50 МБ, GIF не больше 2 МБ. Если файл вышел больше, код
пересобирает его легче (выше CRF у MP4; меньше ширина, кадров и цветов у GIF) и только потом сдаётся
с честной причиной. ffmpeg и Popen подменяются в тестах.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Callable, Iterable

# Рисование кадров живёт рядом с инструментами HIL (scripts/hil/video/pose_anim.py).
HIL_VIDEO = Path(__file__).resolve().parents[2] / "scripts" / "hil" / "video"
if str(HIL_VIDEO) not in sys.path:
    sys.path.insert(0, str(HIL_VIDEO))

MP4_LIMIT = 50 * 1024 * 1024
GIF_LIMIT = 2 * 1024 * 1024
MP4_CRF = (23, 28, 33)  # лестница качества: следующая ступень — если файл не влез в лимит
# GIF: (ширина, каждый n-й кадр, цветов). Первая ступень — лучшая.
GIF_LADDER = ((640, 1, 64), (560, 1, 48), (480, 2, 48), (400, 2, 32), (360, 3, 24))


def mb(size: int) -> str:
    return f"{size / 1024 / 1024:.1f} МБ"


def encode_mp4(
    frames: Iterable, size: tuple, fps: int, out: Path, crf: int = 23, popen: Callable = subprocess.Popen
) -> str:
    """Кадры PIL → H.264 MP4 через stdin ffmpeg (rawvideo), без PNG на диске. Пустая строка — успех."""
    w, h = size
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
        "-c:v", "libx264", "-preset", "medium", "-crf", str(crf), "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(out),
    ]  # fmt: skip
    try:
        proc = popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    except OSError as exc:
        return f"ffmpeg не запустился: {exc}"
    count = 0
    try:
        for frame in frames:
            proc.stdin.write(frame.convert("RGB").resize(size).tobytes())
            count += 1
    except (BrokenPipeError, OSError) as exc:
        proc.kill()
        return f"ffmpeg оборвал приём кадров: {exc}"
    finally:
        try:
            proc.stdin.close()
        except OSError:
            pass
    err = (proc.stderr.read() or b"").decode("utf-8", "replace").strip() if proc.stderr else ""
    if proc.wait() != 0:
        return f"ffmpeg вернул ошибку: {err[:200] or 'без текста'}"
    if count == 0:
        return "нет кадров для ролика"
    if not out.is_file() or out.stat().st_size == 0:
        return "ffmpeg не записал файл"
    return ""


def fit_mp4(make_frames: Callable, size: tuple, fps: int, out: Path, limit: int = MP4_LIMIT, popen=subprocess.Popen):
    """MP4 в пределах limit: (путь или None, причина, заметка о ступени)."""
    last = 0
    for crf in MP4_CRF:
        reason = encode_mp4(make_frames(), size, fps, out, crf=crf, popen=popen)
        if reason:
            return None, reason, ""
        last = out.stat().st_size
        if last <= limit:
            return out, "", f"MP4 {mb(last)} (CRF {crf})"
    return None, f"MP4 вышел {mb(last)} даже при CRF {MP4_CRF[-1]} — больше лимита {mb(limit)}", ""


def fit_gif(make_frames: Callable, out: Path, fps: int, limit: int = GIF_LIMIT):
    """GIF в пределах limit по лестнице GIF_LADDER: (путь или None, причина, заметка о ступени)."""
    from pose_anim import save_gif

    last = 0
    for width, every, colors in GIF_LADDER:
        frames = []
        for i, frame in enumerate(make_frames()):
            if i % every:
                continue
            height = round(frame.height * width / frame.width)
            frames.append(frame.convert("RGB").resize((width, height)))
        if not frames:
            return None, "нет кадров для GIF", ""
        last = save_gif(frames, out, fps=max(2, fps // every), colors=colors)
        if last <= limit:
            return out, "", f"GIF {mb(last)}, {width} px, {len(frames)} кадров, {colors} цветов"
    return None, f"GIF вышел {mb(last)} даже на последней ступени — больше лимита {mb(limit)}", ""


def gif_to_mp4(gif: Path, out: Path, run: Callable = subprocess.run) -> str:
    """Готовый GIF (инфографика) → MP4, когда просили видео. Пустая строка — успех."""
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error", "-i", str(gif),
        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2,format=yuv420p", "-c:v", "libx264", "-crf", "23",
        "-movflags", "+faststart", str(out),
    ]  # fmt: skip
    try:
        done = run(cmd, capture_output=True, text=True, timeout=300, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"ffmpeg не запустился: {type(exc).__name__}"
    if done.returncode != 0:
        return f"ffmpeg вернул ошибку: {(done.stderr or '').strip()[:200]}"
    return ""


def images_to_mp4(pattern_dir: Path, fps: int, out: Path, run: Callable = subprocess.run) -> str:
    """Кадры PNG из каталога (compose.py, пропуски номеров допустимы) → MP4."""
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-pattern_type", "glob",
        "-i", str(Path(pattern_dir) / "*.png"), "-vf", "format=yuv420p", "-c:v", "libx264", "-crf", "23",
        "-movflags", "+faststart", str(out),
    ]  # fmt: skip
    try:
        done = run(cmd, capture_output=True, text=True, timeout=900, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"ffmpeg не запустился: {type(exc).__name__}"
    if done.returncode != 0:
        return f"ffmpeg вернул ошибку: {(done.stderr or '').strip()[:200]}"
    return ""


def check_mp4(path: Path, run: Callable = subprocess.run) -> tuple:
    """Проверка результата до показа Шифу (ffprobe): есть видеопоток, длительность больше нуля.

    (заметка для индикатора, причина отказа). Нет ffprobe — честная заметка «не проверял», а не «ок».
    """
    cmd = [
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height:format=duration", "-of", "json", str(path),
    ]  # fmt: skip
    try:
        done = run(cmd, capture_output=True, text=True, timeout=60, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"ffprobe недоступен ({type(exc).__name__}) — ролик не проверял", ""
    try:
        import json

        data = json.loads(done.stdout or "{}")
    except ValueError:
        return "", f"ffprobe не разобрал файл: {(done.stderr or '').strip()[:120]}"
    streams = data.get("streams") or []
    duration = float((data.get("format") or {}).get("duration") or 0)
    if done.returncode != 0 or not streams or duration <= 0:
        return "", f"в файле нет видео или длительность 0: {(done.stderr or '').strip()[:120]}"
    return f"проверил ffprobe: {streams[0].get('width')}×{streams[0].get('height')}, {duration:.1f} с", ""
