"""Ключи кэша генераций (ADR-0158 §2.5): одно место, где строится ``request_key``."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import Union

#: Шаг корзины длительности, секунд: 150 с и 160 с — один запрос.
DURATION_BUCKET_S = 30

_WS = re.compile(r"\s+")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Union[str, Path]) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def normalize_theme(theme: str) -> str:
    """Тема запроса: нижний регистр, схлопнутые пробелы (леммы — PR-3, как ``theme_grounding``)."""
    return _WS.sub(" ", (theme or "").strip().lower())


def duration_bucket(duration_s: float, step: int = DURATION_BUCKET_S) -> int:
    return int(round(float(duration_s) / step)) * step


def request_key(
    *,
    kind: str,
    style_key: str = "",
    theme: str = "",
    work_id: str = "",
    instrument: str = "",
    lyrics: str = "",
    tts_provider: str = "",
    voice: str = "",
    gen_provider: str = "",
    model: str = "",
    duration_s: float = 0.0,
    source_audio_sha: str = "",
) -> str:
    """``sha256`` канонического JSON запроса: одинаковый запрос — одинаковый ключ.

    «Валькирии на ситаре» и «валькирии оркестром» различаются ``instrument`` и
    ``style_key``; смена вокала/исходника — ``voice`` и ``source_audio_sha``.
    """
    payload = {
        "kind": kind,
        "work_id": normalize_theme(work_id),
        "instrument": normalize_theme(instrument),
        "style_key": normalize_theme(style_key),
        "theme_norm": normalize_theme(theme),
        "lyrics_sha": sha256_bytes(lyrics.encode("utf-8")) if lyrics else "",
        "tts_provider": tts_provider,
        "voice": voice,
        "gen_provider": gen_provider,
        "model": model,
        "duration_bucket": duration_bucket(duration_s) if duration_s else 0,
        "source_audio_sha": source_audio_sha,
    }
    blob = json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(",", ":"))
    return sha256_bytes(blob.encode("utf-8"))


def pool_key(kind: str, style_key: str, instrumental: bool) -> str:
    """«Любой трек этого стиля» для сетов и мостов: ``(kind, style_key, instrumental)``."""
    return f"{kind}|{normalize_theme(style_key)}|{'inst' if instrumental else 'vocal'}"
