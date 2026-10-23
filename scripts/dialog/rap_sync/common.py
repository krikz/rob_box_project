"""Общие данные спайка «речь в такт» (ADR-0150 §5.6.3 О11, PR-S1 #3709): куплеты, голоса, звук 16 кГц.

Куплет — сегмент выступления (``Performance.segments``): одна реплика ``SpeechRequest`` в текущем коде (PR-7c).
Строки куплета разделены ``\\n``; в TTS уходят через пробел, как сегмент от LLM.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Sequence

import numpy as np
import soundfile as sf
from scipy.signal import resample_poly

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src" / "rob_box_dialog"))
from rob_box_dialog.plan.rap_pauses import VOWELS, syllables  # noqa: E402,F401 — слоги: одно знание с продуктом

#: Частота робота: ReSpeaker и jack_rec — 16 кГц (память «Звук робота — 16 кГц»).
SR = 16000

VERSES: Sequence[str] = (
    "Я робот Робби, я читаю под бит,\nКолёса крутятся, мотор не молчит.\n"
    "Камера смотрит, микрофон не спит,\nКаждая строчка в такт звучит.",
    "А вот и енот, полосатый хвост,\nОн моет печенье и лезет на мост.\n"
    "Он ловкий и быстрый, он ночью не спит,\nЕнот за мною по рифме бежит.",
    "Комар над ухом поёт свой мотив,\nЯ машу рукой, но он снова жив.\n"
    "Он хочет крови, а я хочу спать,\nПридётся комару рэп прочитать.",
    "В мастерской паяльник, провода и дым,\nМастер паяет, и я становлюсь живым.\n"
    "Плата за платой, винтик за винтом,\nМы этот трек допишем потом.",
)

#: Голоса реестра (``tts_voice_registry.PROVIDER_VOICES["minimax"]``): текущий голос робота, низкий мужской, женский.
VOICES: Sequence[str] = ("male-qn-qingse", "Russian_ReliableMan", "Russian_BrightHeroine")


def lines(verse: str) -> list:
    return [ln.strip() for ln in verse.split("\n") if ln.strip()]


def tts_text(verse: str) -> str:
    return " ".join(lines(verse))


def read_mono(path: Path) -> np.ndarray:
    """Файл → моно float32 16 кГц."""
    x, sr = sf.read(str(path), dtype="float32", always_2d=True)
    x = x.mean(axis=1)
    if sr != SR:
        g = np.gcd(int(sr), SR)
        x = resample_poly(x, SR // g, int(sr) // g).astype(np.float32)
    return x


def rms_db(x: np.ndarray) -> float:
    return float(20 * np.log10(np.sqrt(np.mean(np.square(x))) + 1e-12))


def gain_to(x: np.ndarray, target_db: float) -> np.ndarray:
    return x * 10 ** ((target_db - rms_db(x)) / 20)


def write_audio(path: Path, x: np.ndarray, ogg: bool = True) -> None:
    """WAV 16 кГц PCM16 (+ OGG Opus для Telegram рядом)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    peak = float(np.max(np.abs(x)) or 1.0)
    if peak > 0.89:  # −1 дБFS потолок
        x = x * (0.89 / peak)
    sf.write(str(path), x.astype(np.float32), SR, subtype="PCM_16")
    if ogg:
        subprocess.run(["ffmpeg", "-y", "-v", "error", "-i", str(path), "-c:a", "libopus", "-b:a", "48k",
                        str(path.with_suffix(".ogg"))], check=True)
