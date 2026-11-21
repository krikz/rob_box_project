#!/usr/bin/env python3
"""Заглушки клипов режима «отвечай звуками» (ADR-0175, карточка #5067).

Это НЕ настоящие голоса существ. Пять коротких синтезированных сигналов,
различимых на слух, чтобы приёмка режима шла на роботе до пакета §0.1.
Настоящие звуки кладутся в тот же каталог под теми же именами — код не меняется.

Детерминированно: numpy + stdlib wave, без скачивания и без чужих файлов.
Частота 16000 Гц, моно, PCM16 — робот выводит 16 кГц (ReSpeaker).

Запуск из корня репо:
    python3 scripts/dev/gen_sound_answer_clips.py
"""

from __future__ import annotations

import math
import wave
from pathlib import Path

import numpy as np

RATE = 16000
PEAK = 16000  # запас до клиппинга: полная шкала int16 — 32767
OUT_DIR = Path(__file__).resolve().parents[2] / "sound_pack" / "sound_answer"

# id клипа → как звучит заглушка. Имена совпадают с sound_answer_manifest.json.
CLIPS = ("bird", "seal", "elephant", "alligator", "growl")


def _tone(freq: float, seconds: float, fade: float = 0.02) -> np.ndarray:
    """Синус с коротким нарастанием и спадом, чтобы клип не щёлкал на стыке."""
    n = int(RATE * seconds)
    t = np.arange(n) / RATE
    wave_ = np.sin(2 * math.pi * freq * t)
    return _fade(wave_, fade)


def _fade(wave_: np.ndarray, fade: float) -> np.ndarray:
    edge = min(int(RATE * fade), len(wave_) // 2)
    ramp = np.linspace(0.0, 1.0, edge)
    out = wave_.copy()
    out[:edge] *= ramp
    out[-edge:] *= ramp[::-1]
    return out


def _chirp(start: float, end: float, seconds: float) -> np.ndarray:
    """Тон с плывущей частотой: слышно не как ровная нота."""
    n = int(RATE * seconds)
    freq = np.linspace(start, end, n)
    phase = 2 * math.pi * np.cumsum(freq) / RATE
    return _fade(np.sin(phase), 0.015)


def bird() -> np.ndarray:
    """Два коротких свиста вверх — выше всех остальных."""
    gap = np.zeros(int(RATE * 0.06))
    return np.concatenate([_chirp(1800, 3200, 0.12), gap, _chirp(2000, 3600, 0.14)])


def seal() -> np.ndarray:
    """Нисходящее «ау» средней высоты."""
    return _chirp(700, 280, 0.55)


def elephant() -> np.ndarray:
    """Низкий ровный гул — самый низкий и самый длинный."""
    body = _tone(90, 0.8, fade=0.05)
    overtone = 0.3 * _tone(180, 0.8, fade=0.05)
    return body + overtone


def alligator() -> np.ndarray:
    """Три сухих щелчка."""
    click = _fade(
        np.sin(2 * math.pi * 1400 * np.arange(int(RATE * 0.03)) / RATE), 0.004
    )
    gap = np.zeros(int(RATE * 0.12))
    return np.concatenate([click, gap, click, gap, click])


def growl() -> np.ndarray:
    """Низкое рычание: пила с медленной амплитудой."""
    n = int(RATE * 0.7)
    t = np.arange(n) / RATE
    saw = 2 * ((220 * t) % 1.0) - 1
    tremolo = 0.6 + 0.4 * np.sin(2 * math.pi * 8 * t)
    return _fade(saw * tremolo, 0.03)


def render(name: str) -> np.ndarray:
    sample = {
        "bird": bird,
        "seal": seal,
        "elephant": elephant,
        "alligator": alligator,
        "growl": growl,
    }[name]()
    pcm = np.clip(sample * PEAK, -32767, 32767).astype("<i2")
    return pcm


def write_wav(path: Path, pcm: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(RATE)
        handle.writeframes(pcm.tobytes())


def main() -> None:
    for name in CLIPS:
        pcm = render(name)
        path = OUT_DIR / f"{name}.wav"
        write_wav(path, pcm)
        print(
            f"{path.name}: {len(pcm)} сэмплов, {len(pcm) / RATE:.3f} с, {path.stat().st_size} байт"
        )


if __name__ == "__main__":
    main()
