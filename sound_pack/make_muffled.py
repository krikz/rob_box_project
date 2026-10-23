#!/usr/bin/env python3
"""Генератор ``muffled.mp3`` - earcon тишины (#3984, решение Шифу 23.10: «приглушённый пук» вместо пик-пик).

Звук короткий (~0,26 с), мягкий и низкий: тон съезжает с ~210 до ~130 Гц с лёгким «дрожанием» (амплитудная
модуляция ~36 Гц), поверх - вторая и третья гармоники (на маленькой колонке слышно именно их), немного
низкочастотного шума и общий НЧ-фильтр. Атака 12 мс, затухание экспонентой. Пик ~ -14 dBFS: на ~9 dB тише
``button_click`` (-5,3 dBFS), которым играет обычное «услышал».

Файл не в ``sound_catalog.json``: это звук кода (``knowledge.SILENCED_EARCONS``), LLM его не выбирает.
Нужны numpy и ``ffmpeg`` в PATH. Детерминирован (seed шума фиксирован): повторный запуск даёт тот же сигнал.

    python sound_pack/make_muffled.py            # пишет sound_pack/muffled.mp3
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
import wave
from pathlib import Path

import numpy as np

RATE = 16000                 # тот же, в котором sound_node играет через ReSpeaker
DURATION_S = 0.26
PEAK_DBFS = -14.0


def synth() -> np.ndarray:
    n = int(RATE * DURATION_S)
    t = np.arange(n) / RATE
    freq = 210.0 * (130.0 / 210.0) ** (t / DURATION_S)                  # экспоненциальный съезд вниз
    phase = 2 * np.pi * np.cumsum(freq) / RATE
    tone = np.sin(phase) + 0.55 * np.sin(2 * phase) + 0.3 * np.sin(3 * phase)
    tone *= 1.0 - 0.45 * (0.5 - 0.5 * np.cos(2 * np.pi * 36.0 * t))    # дрожание «пр-р»
    noise = np.random.default_rng(3984).standard_normal(n)
    noise = np.convolve(noise, np.ones(24) / 24, mode="same")          # шум только в низах (~< 700 Гц)
    sig = tone + 0.35 * noise
    env = np.minimum(1.0, t / 0.012) * np.exp(-t / 0.085) * np.minimum(1.0, (DURATION_S - t) / 0.02)
    sig = sig * env
    alpha = 1.0 - np.exp(-2 * np.pi * 1100.0 / RATE)                     # общий НЧ-фильтр: «в подушку»
    out = np.empty_like(sig)
    acc = 0.0
    for i, x in enumerate(sig):
        acc += alpha * (x - acc)
        out[i] = acc
    return out * (10 ** (PEAK_DBFS / 20) / np.max(np.abs(out)))


def write_mp3(samples: np.ndarray, target: Path) -> None:
    pcm = (np.clip(samples, -1.0, 1.0) * 32767).astype("<i2")
    with tempfile.TemporaryDirectory() as tmp:
        wav = Path(tmp) / "muffled.wav"
        with wave.open(str(wav), "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(RATE)
            w.writeframes(pcm.tobytes())
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(wav), "-codec:a", "libmp3lame", "-q:a", "4",
                        str(target)], check=True)


if __name__ == "__main__":
    out = Path(__file__).with_name("muffled.mp3")
    write_mp3(synth(), out)
    print(f"{out} ({out.stat().st_size} байт)")
    sys.exit(0)
