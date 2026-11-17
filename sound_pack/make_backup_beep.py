#!/usr/bin/env python3
"""Генератор бипера заднего хода (#4365, решение Шифу 05.11: «как у грузовика задний ход»).

Тревога зоны безопасности корпуса (``rob_box_teleop.proximity.ALARMS``) звучит как бипер грузовика:

* ``backup_beep.mp3`` (триггер ``robot_backup_beep``) — один бип 0,45 с; «внимание» повторяет его каждые 0,9 с
  (бип 0,45 / пауза 0,45 - классический ритм бипера);
* ``backup_beep_fast.mp3`` (триггер ``robot_backup_beep_fast``) — короткий бип 0,2 с; «стоп» повторяет его каждые
  0,4 с - частый бип.

Паузы в файле нет: ритм задаёт расписание тревоги (``Alarm.sound_every_s``), а канал звука не занят тишиной.

Звук: тон 1000 Гц плюс 3-я (3 кГц, 0,30) и 5-я (5 кГц, 0,12) гармоники - «пищалка» пьезо, всё ниже 8 кГц
(робот играет через ReSpeaker на 16 кГц, полоса до 8 кГц). Атака и спад 8 мс (косинус) - без щелчков.
Пик -12 dBFS (RMS ~ -13 dB): умеренно, без клиппинга (``error`` -6,5 dBFS, ``button_click`` -5,3 dBFS).
Синтез - только numpy и ``wave``; mp3 кодирует ``ffmpeg`` из PATH (как ``make_muffled.py``).
Детерминирован: повторный запуск даёт тот же сигнал.

    python sound_pack/make_backup_beep.py          # пишет sound_pack/backup_beep.mp3 и backup_beep_fast.mp3
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
import wave
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

RATE = 16000                 # тот же, в котором sound_node играет через ReSpeaker
TONE_HZ = 1000.0
#: (номер гармоники, амплитуда) - нечётные, как у меандра пьезо-пищалки; 5 кГц < 8 кГц (Найквист 16 кГц).
HARMONICS: Tuple[Tuple[int, float], ...] = ((1, 1.0), (3, 0.30), (5, 0.12))
RAMP_S = 0.008
PEAK_DBFS = -12.0
#: Файл -> длина бипа, с.
BEEPS: Dict[str, float] = {"backup_beep.mp3": 0.45, "backup_beep_fast.mp3": 0.20}


def synth(beep_s: float) -> np.ndarray:
    """Один бип длиной ``beep_s``: тон с гармониками, плавные края, пик ``PEAK_DBFS``."""
    n = int(round(RATE * beep_s))
    t = np.arange(n) / RATE
    sig = sum(a * np.sin(2 * np.pi * TONE_HZ * k * t) for k, a in HARMONICS)
    r = int(RATE * RAMP_S)
    ramp = 0.5 - 0.5 * np.cos(np.pi * np.arange(r) / r)
    env = np.ones(n)
    env[:r], env[-r:] = ramp, ramp[::-1]
    sig = sig * env
    return sig * (10 ** (PEAK_DBFS / 20) / np.max(np.abs(sig)))


def write_wav(samples: np.ndarray, target: Path) -> None:
    pcm = (np.clip(samples, -1.0, 1.0) * 32767).astype("<i2")
    with wave.open(str(target), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(pcm.tobytes())


def write_mp3(samples: np.ndarray, target: Path) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        wav = Path(tmp) / "beep.wav"
        write_wav(samples, wav)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(wav), "-codec:a", "libmp3lame", "-q:a", "4",
                        str(target)], check=True)


if __name__ == "__main__":
    for name, seconds in BEEPS.items():
        out = Path(__file__).with_name(name)
        write_mp3(synth(seconds), out)
        print(f"{out} ({out.stat().st_size} байт, бип {seconds} с)")
    sys.exit(0)
