"""Стадии реплики ТАРС-в-шлем: speaking → idle (ADR-0078 §3.6, issue #3253 Ш2).

Чистая логика без rclpy/ws: quest_node кормит трекер фактами, которые он
реально видит, и переводит возвращённые события в WS. Время — аргументом
(монотонные секунды), поэтому последовательность проверяется без часов.

Откуда берутся факты (весь путь реплики — в ADR-0078 §3.6):

* ``on_request`` — ``/avatar/tts/request`` (sink=headset) от supervisor'а:
  ответ LLM готов и ушёл в синтез. ``speech_id`` из payload'а связывает
  реплику с ``/voice/tts/finished`` от tts_node.
* ``on_chunk`` — очередной ``/avatar/tts/audio`` доставлен в шлем. Первый
  чанк реплики → ``speaking``.
* ``on_finished`` — ``/voice/tts/finished`` с тем же ``speech_id``: синтез
  закончен (успех или ошибка).
* ``on_cancel`` — оператор зажал PTT (барж-ин): клиент уже оборвал звук.
* ``tick`` — периодически: выдаёт отложенный ``idle``.

Почему ``idle`` отложенный. Клиент играет чанки очередью, а на
``operator_tts_done`` зовёт ``operatorAudioSink.stop()`` — очередь
обрывается. Синтез (MiniMax-стрим) быстрее реального времени, поэтому
``finished`` приходит, когда в шлеме ещё звучат секунды речи. Трекер
считает оценку конца звука по байтам и частоте (int16 моно) и отдаёт
``idle`` + ``done`` только после неё плюс запас ``tail_margin_s``.

Страховки (экран не должен залипать, а реплика — обрываться молча):

* ``finished`` потерян (best-effort QoS, другой процесс) — ``idle`` по
  тишине: нет новых чанков ``silence_timeout_s`` после конца звука;
* синтез не дал ни одного чанка и молчит — ``idle`` через
  ``synth_timeout_s`` после запроса.

Каждое такое ``idle`` несёт ``reason``, чтобы в логах было видно, честный
это конец реплики или страховка.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Optional

# Байт на сэмпл PCM, который tts_node публикует в /avatar/tts/audio
# (int16 LE, моно — см. tts_node._publish_headset_audio).
PCM_BYTES_PER_SAMPLE = 2

DEFAULT_TAIL_MARGIN_S = 0.5
DEFAULT_SILENCE_TIMEOUT_S = 5.0
DEFAULT_SYNTH_TIMEOUT_S = 30.0

Event = dict[str, Any]


def pcm_duration_s(n_bytes: int, sample_rate: int) -> float:
    """Длительность int16-моно PCM в секундах (0 для мусорных входов)."""
    if n_bytes <= 0 or sample_rate <= 0:
        return 0.0
    return n_bytes / float(PCM_BYTES_PER_SAMPLE * sample_rate)


def stage_event(stage: str, request_id: str, reason: str = "") -> Event:
    event: Event = {"kind": "stage", "stage": stage, "request_id": request_id}
    if reason:
        event["reason"] = reason
    return event


@dataclass
class _Reply:
    request_id: str
    speech_id: Optional[str]
    requested_at: float
    speaking: bool = False
    audio_end: float = 0.0
    finished: bool = False
    error: str = ""


class TarsStageTracker:
    """Одна активная реплика ТАРС-в-шлем (оператор на Quest один).

    Потокобезопасен: факты приходят из ROS-экзекутора, барж-ин — из
    aiohttp-loop'а.
    """

    def __init__(
        self,
        *,
        tail_margin_s: float = DEFAULT_TAIL_MARGIN_S,
        silence_timeout_s: float = DEFAULT_SILENCE_TIMEOUT_S,
        synth_timeout_s: float = DEFAULT_SYNTH_TIMEOUT_S,
    ) -> None:
        self._tail_margin_s = tail_margin_s
        self._silence_timeout_s = silence_timeout_s
        self._synth_timeout_s = synth_timeout_s
        self._reply: Optional[_Reply] = None
        self._lock = threading.Lock()

    @property
    def active_request_id(self) -> Optional[str]:
        reply = self._reply
        return reply.request_id if reply is not None else None

    def on_request(self, request_id: str, speech_id: Any, now: float) -> list[Event]:
        """Новая реплика вытесняет прежнюю (её стадии клиент перекроет).

        ``speech_id`` не строка/пустой — конец синтеза не сопоставить, idle
        придёт по страховке (тишина после звука).
        """
        sid = speech_id if isinstance(speech_id, str) and speech_id else None
        with self._lock:
            self._reply = _Reply(request_id, sid, now)
        return []

    def on_chunk(self, request_id: str, n_bytes: int, sample_rate: int, now: float) -> list[Event]:
        """Чанк доставлен в шлем: двигаем оценку конца звука."""
        with self._lock:
            reply = self._reply
            if reply is None or reply.request_id != request_id:
                return []
            reply.audio_end = max(reply.audio_end, now) + pcm_duration_s(n_bytes, sample_rate)
            if reply.speaking:
                return []
            reply.speaking = True
        return [stage_event("speaking", request_id)]

    def on_finished(self, speech_id: str, success: bool, error: str, now: float) -> list[Event]:
        """Синтез закончен. Ошибка без единого чанка — сразу ``idle``."""
        with self._lock:
            reply = self._reply
            if reply is None or not speech_id or reply.speech_id != speech_id:
                return []
            reply.finished = True
            reply.error = "" if success else (error or "tts_failed")
        return self.tick(now)

    def on_cancel(self, reason: str, now: float) -> list[Event]:
        """Барж-ин/отмена: реплика закрывается сразу, без ожидания звука."""
        with self._lock:
            reply = self._reply
            self._reply = None
        if reply is None:
            return []
        return _close_events(reply, reason)

    def tick(self, now: float) -> list[Event]:
        """Отдать ``idle``, если реплика закончилась (или сработала страховка)."""
        with self._lock:
            reply = self._reply
            if reply is None:
                return []
            reason = self._due_reason(reply, now)
            if not reason:
                return []
            self._reply = None
        return _close_events(reply, reason)

    def _due_reason(self, reply: _Reply, now: float) -> str:
        if not reply.speaking:
            if reply.finished:
                return reply.error or "no_audio"
            if now - reply.requested_at >= self._synth_timeout_s:
                return "synth_timeout"
            return ""
        played_out = now >= reply.audio_end + self._tail_margin_s
        if reply.finished and played_out:
            return reply.error or "done"
        if now >= reply.audio_end + self._silence_timeout_s:
            return "silence_timeout"
        return ""


def _close_events(reply: _Reply, reason: str) -> list[Event]:
    """``idle`` + терминальное событие канала operator_tts.

    Ошибка синтеза → ``operator_tts_error`` (клиент чистит очередь), иначе
    ``operator_tts_done``. ``reason`` уходит и в ``tars_state``, и в лог.
    """
    events: list[Event] = [stage_event("idle", reply.request_id, reason)]
    if reply.error and reason == reply.error:
        events.append({"kind": "error", "request_id": reply.request_id, "reason": reason})
    else:
        events.append({"kind": "done", "request_id": reply.request_id, "reason": reason})
    return events
