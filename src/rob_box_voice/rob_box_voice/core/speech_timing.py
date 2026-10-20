"""speech_timing - «когда» звучит реплика (ADR-0150 §5.6.1 «Голос», PR-3 ч.3 #3688).

Три поля ``SpeechRequest`` сверх «сказать сейчас»:

* ``commit=false`` - только синтез: аудио держится здесь до второго запроса с
  тем же ``speech_id`` и ``commit=true``, наружу - ``queued`` и
  ``synthesized{duration_ms}``. Запрос с ``commit=true`` берёт готовое аудио
  вместо провайдера (``provider=prefetch`` в его ``synthesized``).
* ``at`` (ROS-секунды) - момент, когда звук должен прозвучать: перед
  ``play_audio`` ждём ``at - now - output_latency_ms``; отмена прерывает ожидание.
* ``boundary=sentence`` - отмена играющей реплики на ближайшей границе
  предложения (см. ``SpeechLedger.schedule_boundary_stop``).

Квантования на такт здесь нет (PR-7c), ждём не дольше ``MAX_AT_WAIT_S``.
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional

#: Дольше ждать ``at`` не будем: старт «через минуту» - ошибка вызывающего.
MAX_AT_WAIT_S = 30.0


def at_delay_s(now_s: float, at: float, output_latency_ms: int) -> float:
    """Сколько ждать перед ``play_audio``, чтобы звук вышел в момент ``at``."""
    return min(MAX_AT_WAIT_S, max(0.0, at - now_s - output_latency_ms / 1000.0))


class Presynth:
    """Пред-синтез ``commit=false``: аудио ждёт своего ``commit=true``."""

    def __init__(self) -> None:
        self._held: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()

    def claim(self, speech_id: Optional[str]) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._held.pop(speech_id or "", None)

    def run(self, node: Any, chunk: Dict[str, Any], done: Callable[[str, float, Dict[str, Any]], None]) -> None:
        """Синтезировать чанк провайдерами ``tts_node`` без воспроизведения.

        ``done(speech_id, duration_s, ctx)`` - успех; ошибка закрывает реплику
        ``rejected`` через воронку ``_publish_tts_finished``.
        """
        speech_id = chunk["speech_id"]
        executor = getattr(node, "_synthesis_executor", None)
        if executor is None or getattr(node, "minimax_streaming", False):
            # стрим-режим MiniMax публикует PCM в топик по ходу синтеза - это уже звук
            node._publish_tts_finished(speech_id, success=False, error="presynth_unavailable")
            return
        executor.submit(self._worker, node, chunk, done)

    def _worker(self, node: Any, chunk: Dict[str, Any], done: Callable[[str, float, Dict[str, Any]], None]) -> None:
        speech_id, ssml = chunk["speech_id"], chunk["ssml"]
        voice, language = chunk.get("voice"), chunk.get("language")
        text = node._extract_text_from_ssml(ssml)
        attrs = node._parse_ssml_attributes(ssml) or {}
        ctx: Dict[str, Any] = {"audio_np": None, "sample_rate": 16000, "used_provider": None,
                               "used_voice": None, "result": {}}
        try:
            node._sap_run_provider_chain(voice, language, attrs, text, "speaker", ctx)
            if ctx["audio_np"] is None:
                node._sap_silero_fallback(text, voice, language, attrs, None, ctx)
        except Exception as exc:  # noqa: BLE001 - честный rejected вместо тишины
            node._publish_tts_finished(speech_id, success=False, error=f"presynth: {exc}")
            return
        with self._lock:
            self._held[speech_id] = {
                "audio_np": ctx["audio_np"], "sample_rate": ctx["sample_rate"],
                "decision": "presynth", "confidence": 1.0, "basis": "commit=false",
            }
        done(speech_id, len(ctx["audio_np"]) / ctx["sample_rate"] if ctx["sample_rate"] else 0.0, ctx)
