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

Звучание никогда не стоит за пред-синтезом (#3834): ``commit=false`` синтезирует
свой пул :class:`Presynth`, а не пул ``tts_node._synthesis_executor``, где
``commit=true`` ждёт очереди, ``at`` и играет. Общий пул из двух потоков ставил
готовую реплику за правки «и про енота» - дыра 3.5 с посреди рэпа (#3700).
"""

from __future__ import annotations

import concurrent.futures
import threading
from typing import Any, Callable, Dict, Optional

#: Дольше ждать ``at`` не будем: старт «через минуту» - ошибка вызывающего.
MAX_AT_WAIT_S = 30.0
#: Потоков пред-синтеза: столько сегментов правки синтезируются одновременно.
PRESYNTH_MAX_WORKERS = 2
PRESYNTH_THREAD_NAME_PREFIX = "tts-presynth"


def at_delay_s(now_s: float, at: float, output_latency_ms: int) -> float:
    """Сколько ждать перед ``play_audio``, чтобы звук вышел в момент ``at``."""
    return min(MAX_AT_WAIT_S, max(0.0, at - now_s - output_latency_ms / 1000.0))


Done = Callable[[str, float, Dict[str, Any]], None]
_CLOSED = object()  # пул погашен


class Presynth:
    """Пред-синтез ``commit=false`` в своём пуле: аудио ждёт своего ``commit=true``.

    ``executor`` - для тестов; по умолчанию пул ``PRESYNTH_MAX_WORKERS``
    создаётся при первом запросе и гасится :meth:`shutdown`.
    """

    def __init__(self, executor: Any = None) -> None:
        self._held: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()
        self._executor = executor

    def claim(self, speech_id: Optional[str]) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._held.pop(speech_id or "", None)

    def run(self, node: Any, chunk: Dict[str, Any], done: Done, cancelled: Callable[[str], bool]) -> None:
        """Синтезировать чанк провайдерами ``tts_node`` без воспроизведения.

        ``done(speech_id, duration_s, ctx)`` - синтез кончился (или пропущен:
        реплику отменили в очереди, ``cancelled(speech_id)``); ошибка закрывает
        реплику ``rejected`` через воронку ``_publish_tts_finished``.
        """
        speech_id = chunk["speech_id"]
        try:
            self._pool().submit(self._worker, node, chunk, done, cancelled)
        except RuntimeError:  # пул погашен: нода уходит
            node._publish_tts_finished(speech_id, success=False, error="shutdown")

    def shutdown(self) -> None:
        """Погасить пул: ждущие в очереди сегменты не синтезируются (нода уходит)."""
        with self._lock:
            executor, self._executor = self._executor, _CLOSED
        if executor not in (None, _CLOSED):
            executor.shutdown(wait=False, cancel_futures=True)

    def _pool(self) -> Any:
        with self._lock:
            if self._executor is _CLOSED:
                raise RuntimeError("presynth pool is shut down")
            if self._executor is None:
                self._executor = concurrent.futures.ThreadPoolExecutor(
                    max_workers=PRESYNTH_MAX_WORKERS, thread_name_prefix=PRESYNTH_THREAD_NAME_PREFIX)
            return self._executor

    def _worker(self, node: Any, chunk: Dict[str, Any], done: Done, cancelled: Callable[[str], bool]) -> None:
        speech_id, ssml = chunk["speech_id"], chunk["ssml"]
        # без ``stream_ok``: пред-синтез - HTTP целиком, поток MiniMax звучит сразу (#3930)
        ctx: Dict[str, Any] = {"audio_np": None, "sample_rate": 16000, "used_provider": None,
                               "used_voice": None, "result": {}}
        if cancelled(speech_id):  # отменили, пока ждал пула: не платим за синтез
            done(speech_id, 0.0, ctx)
            return
        voice, language = chunk.get("voice"), chunk.get("language")
        text = node._extract_text_from_ssml(ssml)
        attrs = node._parse_ssml_attributes(ssml) or {}
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
