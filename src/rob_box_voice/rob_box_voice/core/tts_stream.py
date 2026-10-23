"""tts_stream - потоковый синтез MiniMax в динамик: первый звук до конца синтеза (#3930, A2 ADR-0150 §14).

Было: HTTP T2A v2 целиком (2.2-3.6 с на 72 символа) -> ресэмпл -> ``play_audio``. Стало: SSE-поток
(``stream=true``) прямо в 16 кГц ReSpeaker, воспроизведение - как только набралось ``PREBUFFER_S`` речи.

Цифры спайка 23.10 (``docs/dialog/tts_stream_spike_2026-10-23.md``, 16 запросов): первый кусок 0.73-1.39 с
при любой длине текста; кусок 1 - 0.35 с аудио, дальше по ~0.8-1.3 с, поток идёт в 3-6 раз быстрее
реального времени; склейка кусков бит-в-бит равна целому (``exclude_aggregated_audio=false``); паузы
``<#x#>`` рэпа звучат как в HTTP. Первый кусок - почти целиком тишина MiniMax (0.30-0.38 с в начале любой
фразы, и в HTTP тоже): ``LEAD_KEEP_S`` от неё оставляем, остальное не играем.

Кто что делает:

* :class:`PcmStream` - буфер реплики: продюсер (вечный TTS-loop) кладёт куски mono float32 в частоте вывода,
  плеер (``AudioPlaybackManager.play_stream``) забирает кадры; без ROS и звука - тестируется сам.
* :func:`open_for_playback` - запуск потока из ``TTSNode._sap_synthesize_minimax``: ждёт предбуфер или
  отказ ДО первого звука (тогда исключение - цепочка уходит на Yandex/Silero, как у HTTP).
* :func:`play_local` - воспроизведение из ``TTSNode._sap_local_playback``: ``started`` перед первым
  кадром, огибающая рта по каждому куску, длительность реплики - когда поток кончился.

Обрыв потока ПОСЛЕ начала звука честно закрывает реплику ошибкой ``minimax_stream_broken`` (не ``finished``):
хвост фразы не прозвучал, и молча делать вид, что прозвучал, нельзя (capability-honest, tts-extension-points).
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Any, Callable, List, Optional, Tuple

import numpy as np

#: Сколько речи (после срезки начальной тишины) накопить до старта: второй кусок приходит через 0.1-0.6 с
#: после первого, а первый - 0.35 с (почти вся - тишина). С 0.5 с запас до опустошения буфера по спайку
#: >= 0.36 с (9 потоков), без него (старт на первом куске) - провалы до 0.29 с посреди слова.
PREBUFFER_S = 0.5
#: Начальную тишину MiniMax срезаем до стольких секунд (атака первого звука не обрезается).
LEAD_KEEP_S = 0.05
#: Больше этого в начале не срезаем, даже если тихо (шёпот / пауза ``<#x#>`` в начале - намеренные).
LEAD_MAX_TRIM_S = 0.5
#: Окно и порог «тишины» для срезки начала: RMS < 0.01 полной шкалы по окнам 20 мс.
LEAD_WINDOW_S = 0.02
LEAD_SILENCE_RMS = 0.01
#: Ошибка реплики, когда поток оборвался после начала звука.
BROKEN = "minimax_stream_broken"


class PcmStream:
    """Буфер одной потоковой реплики: mono float32 в ``sample_rate``, потокобезопасный."""

    def __init__(self, sample_rate: int, *, prebuffer_s: float = PREBUFFER_S) -> None:
        self.sample_rate = int(sample_rate)
        self._prebuffer = int(prebuffer_s * self.sample_rate)
        self._cond = threading.Condition()
        self._pieces: List[np.ndarray] = []
        self._offsets: List[int] = []  # кадр начала каждого куска в реплике
        self._frames = 0
        self._read_piece, self._read_pos = 0, 0
        self._lead_trimmed = 0
        self._voiced = False
        self.done = False
        self.cancelled = False
        self.error: Optional[str] = None
        self.future: Any = None  # concurrent.futures.Future продюсера (для отмены)

    # -- продюсер ------------------------------------------------------------------
    def put(self, pcm: np.ndarray) -> bool:
        """Положить кусок; ``False`` - реплику отменили, продюсеру пора остановиться."""
        pcm = self._trim_lead(np.asarray(pcm, dtype=np.float32).reshape(-1))
        with self._cond:
            if self.cancelled:
                return False
            if pcm.size:
                self._offsets.append(self._frames)
                self._pieces.append(pcm)
                self._frames += pcm.size
            self._cond.notify_all()
            return True

    def finish(self, error: Optional[str] = None) -> None:
        """Поток кончился (``error`` - оборвался)."""
        with self._cond:
            self.done, self.error = True, error
            self._cond.notify_all()

    def cancel(self) -> None:
        """Реплику отменили: продюсер бросает поток, ожидающие просыпаются."""
        with self._cond:
            self.cancelled = True
            self._cond.notify_all()
        if self.future is not None:
            self.future.cancel()

    def _trim_lead(self, pcm: np.ndarray) -> np.ndarray:
        """Срезать начальную тишину MiniMax (до ``LEAD_KEEP_S``, не больше ``LEAD_MAX_TRIM_S``)."""
        if self._voiced:
            return pcm
        win = max(1, int(LEAD_WINDOW_S * self.sample_rate))
        count = -(-pcm.size // win)  # хвост короче окна - тоже окно (дополняется нулями)
        padded = np.zeros(count * win, dtype=np.float32)
        padded[: pcm.size] = pcm
        rms = np.sqrt(np.mean(np.square(padded.reshape(count, win)), axis=1))
        loud = np.flatnonzero(rms >= LEAD_SILENCE_RMS)
        budget = max(0, int(LEAD_MAX_TRIM_S * self.sample_rate) - self._lead_trimmed)
        keep = int(LEAD_KEEP_S * self.sample_rate)
        cut = min(budget, max(0, int(loud[0]) * win - keep) if loud.size else pcm.size)
        self._voiced = bool(loud.size) or cut < pcm.size
        self._lead_trimmed += cut
        return pcm[cut:]

    # -- потребитель -----------------------------------------------------------------
    def wait_ready(self, timeout_s: float) -> None:
        """Ждать предбуфер, конец потока или отмену (не дольше ``timeout_s``)."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._cond:
            while not (self._frames >= self._prebuffer or self.done or self.cancelled):
                left = deadline - time.monotonic()
                if left <= 0:
                    return
                self._cond.wait(left)

    def read(self, frames: int) -> np.ndarray:
        """До ``frames`` следующих кадров (меньше - больше пока нет), не блокирует."""
        out: List[np.ndarray] = []
        need = int(frames)
        with self._cond:
            while need > 0 and self._read_piece < len(self._pieces):
                piece = self._pieces[self._read_piece]
                part = piece[self._read_pos:self._read_pos + need]
                out.append(part)
                need -= part.size
                self._read_pos += part.size
                if self._read_pos >= piece.size:
                    self._read_piece, self._read_pos = self._read_piece + 1, 0
        return np.concatenate(out) if out else np.zeros(0, dtype=np.float32)

    def pieces_since(self, index: int) -> Tuple[List[Tuple[float, np.ndarray]], int]:
        """Куски начиная с ``index``: ``[(смещение_с, pcm)]`` и следующий индекс."""
        with self._cond:
            fresh = [(self._offsets[i] / self.sample_rate, self._pieces[i]) for i in range(index, len(self._pieces))]
            return fresh, len(self._pieces)

    def snapshot(self) -> np.ndarray:
        """Всё, что пришло к этому моменту, одним массивом."""
        with self._cond:
            return np.concatenate(self._pieces) if self._pieces else np.zeros(0, dtype=np.float32)

    @property
    def duration_s(self) -> float:
        return self._frames / self.sample_rate if self.sample_rate else 0.0

    def __len__(self) -> int:
        return self._frames


def open_for_playback(node: Any, text: str, settings: Any, deadline: Optional[float]) -> dict:
    """Запустить поток MiniMax на вечном TTS-loop и дождаться предбуфера.

    Возвращает результат синтеза для ``ctx`` (``audio_np`` - сам :class:`PcmStream`). Ни одного кадра к
    сроку (``deadline`` облачного бюджета, иначе ``minimax_timeout``) или отказ до первого звука -
    исключение: цепочка провайдеров идёт дальше, как при отказе HTTP.
    """
    from rob_box_voice.tts_node import _ensure_tts_loop  # noqa: PLC0415 - tts_node импортирует этот модуль

    stream = PcmStream(node.audio_output_sample_rate)
    provider = node._ensure_minimax_provider()
    stream.future = asyncio.run_coroutine_threadsafe(_produce(node, provider, text, settings, stream),
                                                     _ensure_tts_loop())
    timeout_s = (deadline - time.monotonic()) if deadline is not None else float(node.minimax_timeout)
    stream.wait_ready(timeout_s)
    if not len(stream):
        stream.cancel()
        raise Exception(f"MiniMax stream: нет звука ({stream.error or f'не уложился в {timeout_s:.1f} с'})")
    return {"audio_np": stream, "sample_rate": stream.sample_rate, "already_published": True, "stream": stream}


async def _produce(node: Any, provider: Any, text: str, settings: Any, stream: PcmStream) -> None:
    """Куски провайдера -> топик ``/voice/audio/speech`` (как раньше) и буфер реплики."""
    chunks = provider.stream(text, settings=settings)
    try:
        async for chunk in chunks:
            if chunk.finish_reason == "error":
                stream.finish(BROKEN)
                return
            if chunk.samples:
                audio, rate = node._decode_minimax_audio(chunk.samples, chunk.format, chunk.sample_rate)
                audio = node._prepare_audio_for_topic(audio, rate)  # уже 16 кГц - без ресэмпла
                node._publish_audio(audio)
                if not stream.put(audio):
                    return
        stream.finish()
    except Exception as exc:  # noqa: BLE001 - до первого кадра это отказ провайдера, после - обрыв
        stream.finish(f"{type(exc).__name__}: {exc}")
    finally:
        await chunks.aclose()


def play_local(node: Any, stream: PcmStream, speech_id: Optional[str], gain: float,
               owner: Any, playback: Callable[..., Tuple[str, int]]) -> Tuple[str, float]:
    """Сыграть реплику из потока: ``started`` перед первым кадром, огибающая каждого куска, длительность -
    когда поток кончился. Возвращает (исход ``done``/``stopped``/``failed``/``BROKEN``, длительность с)."""
    first = stream.snapshot() * gain
    _, sent = stream.pieces_since(0)
    if not owner.started(speech_id, first, stream.sample_rate, 0.0):
        stream.cancel()
        return "cancelled", 0.0
    told = False

    def poll() -> None:
        nonlocal sent, told
        fresh, sent = stream.pieces_since(sent)
        for offset_s, pcm in fresh:
            owner.streamed(speech_id, pcm * gain, stream.sample_rate, offset_s)
        if stream.done and not told:
            owner.streamed(speech_id, None, stream.sample_rate, 0.0, stream.duration_s)
            told = True

    def should_stop() -> bool:
        # STOP/барж-ин мог прийти между проверкой ``stop_requested`` и стартом потока: ``abort`` тогда
        # уже сброшен ``play_stream`` - флаг ноды ловит и это окно
        return bool(owner.is_cancelled(speech_id) or getattr(node, "stop_requested", False))

    outcome, underrun = playback(stream, stream.sample_rate, gain=gain, poll=poll, should_stop=should_stop)
    if underrun:
        gap_ms = underrun / stream.sample_rate * 1000
        node.get_logger().warning(f"⚠️ [#3930] поток MiniMax не успел за динамиком: тишина {gap_ms:.0f} мс "
                                  f"посреди реплики speech_id={(speech_id or '')[:8]}")
    if outcome != "done":
        stream.cancel()
    elif stream.error:
        node.get_logger().warning(
            f"⚠️ [#3930] поток MiniMax оборвался после {stream.duration_s:.1f} с звука: {stream.error}")
        outcome = BROKEN
    return outcome, round(stream.duration_s, 2)


def drop(audio: Any) -> None:
    """Реплика не дойдёт до динамика (FIFO отверг, ошибка): поток бросить, чтобы не синтезировать впустую."""
    if isinstance(audio, PcmStream):
        audio.cancel()


def abort(node: Any) -> None:
    """Барж-ин / STOP: оборвать потоковое воспроизведение ноды (у тестовых заглушек его может не быть)."""
    abort_stream = getattr(getattr(node, "playback_manager", None), "abort_stream", None)
    if abort_stream is not None:
        abort_stream()
