"""MiniMax T2A v2 с пословными метками времени — синтез куплетов спайка с кэшем и счётчиком вызовов.

Модель и голос — как у робота (``tts_node.yaml``: ``speech-02-hd``, ``minimax_language: ru``). Метки слов —
``subtitle_enable`` + ``subtitle_type=word`` (platform.minimax.io/docs/api-reference/speech-t2a-http): файл
субтитров с ``time_begin``/``time_end`` слов, мс. Каждый настоящий вызов API — строка в ``ledger.jsonl``;
потолок ``BUDGET`` вызовов на весь спайк (поручение координатора, ≤ 40). Ключ — из ``MINIMAX_API_KEY`` или
``--env-file`` (не печатается).
"""

from __future__ import annotations

import hashlib
import json
import os
import time
from pathlib import Path
from typing import Any, Dict, Optional

import httpx

URL = "https://api.minimax.io/v1/t2a_v2"
MODEL = "speech-02-hd"
BUDGET = 40


def load_key(env_file: Optional[str]) -> str:
    key = os.environ.get("MINIMAX_API_KEY", "")
    if not key and env_file:
        for ln in Path(env_file).read_text(encoding="utf-8").splitlines():
            if ln.startswith("MINIMAX_API_KEY="):
                key = ln.split("=", 1)[1].strip().strip('"')
    if not key:
        raise SystemExit("MINIMAX_API_KEY не задан (env или --env-file)")
    return key


class MiniMaxTTS:
    """``synth(voice, text, speed)`` → ``{"wav": путь, "words": [...], "synth_s": ..., "cached": bool}``."""

    def __init__(self, cache: Path, key: str) -> None:
        self.cache = cache
        self.cache.mkdir(parents=True, exist_ok=True)
        self.ledger = cache / "ledger.jsonl"
        self._key = key

    def calls(self) -> int:
        return len(self.ledger.read_text(encoding="utf-8").splitlines()) if self.ledger.exists() else 0

    def synth(self, voice: str, text: str, speed: float = 1.0) -> Dict[str, Any]:
        tag = hashlib.sha1(f"{MODEL}|{voice}|{speed:.3f}|{text}".encode()).hexdigest()[:12]
        wav, meta = self.cache / f"{tag}.wav", self.cache / f"{tag}.json"
        if wav.exists() and meta.exists():
            return dict(json.loads(meta.read_text(encoding="utf-8")), wav=str(wav), cached=True)
        if self.calls() >= BUDGET:
            raise SystemExit(f"бюджет синтезов {BUDGET} исчерпан")
        payload = {
            "model": MODEL, "text": text, "stream": False, "language_boost": "Russian",
            "voice_setting": {"voice_id": voice, "speed": speed, "vol": 1.0, "pitch": 0},
            "audio_setting": {"sample_rate": 16000, "format": "wav", "channel": 1},
            "subtitle_enable": True, "subtitle_type": "word",
        }
        t0 = time.monotonic()
        resp = httpx.post(URL, json=payload, timeout=60, headers={"Authorization": f"Bearer {self._key}"})
        synth_s = time.monotonic() - t0
        body = resp.json()
        with self.ledger.open("a", encoding="utf-8") as f:
            f.write(json.dumps({"tag": tag, "voice": voice, "speed": speed, "chars": len(text),
                                "status": (body.get("base_resp") or {}).get("status_code"),
                                "synth_s": round(synth_s, 2), "ts": time.time()}, ensure_ascii=False) + "\n")
        base = body.get("base_resp") or {}
        if base.get("status_code") != 0:
            raise RuntimeError(f"MiniMax: {base}")
        data = body["data"]
        wav.write_bytes(bytes.fromhex(data["audio"]))
        words = []
        if data.get("subtitle_file"):
            sub = httpx.get(data["subtitle_file"], timeout=30).json()
            for seg in sub if isinstance(sub, list) else []:
                for w in seg.get("timestamped_words") or []:
                    words.append({"word": w.get("word"), "t0": w["time_begin"] / 1000.0,
                                  "t1": w["time_end"] / 1000.0})
        info = {"voice": voice, "speed": speed, "text": text, "words": words, "synth_s": round(synth_s, 2),
                "extra_info": body.get("extra_info")}
        meta.write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")
        return dict(info, wav=str(wav), cached=False)
