"""speech_owner - tts_node как владелец речи (ADR-0150 §6.1, §5.5.4, PR-3 #3688).

Журнал реплик (:class:`SpeechLedger`, без ROS) и события ``SpeechEvent`` на
каждом переходе::

    queued -> synthesized{duration_ms} -> started{text_len}
           -> progress{spoken_chars} (раз в предложение) -> finished | cancelled{spoken_chars}
    rejected{reason}   пустой текст / занятость / ошибка провайдера / неподдержанное поле
    voice_changed{requested, applied, provider}

Перед воспроизведением - ``SpeechEnvelope``: RMS по окнам 40 мс из PCM, который
уходит в ``play_audio``, ``t0 = now + output_latency_ms``.

Отмена ``SpeechCancel`` - по ``speech_id`` или по эпохе агента (реплики не
новее ``epoch``); реплики другой эпохи и другого агента не трогаются. Играющая
глушится сразу, ожидающая снимается у FIFO-ворот ``tts_node`` без звука.

Конвейер ``tts_node`` ест словарь-чанк v1; поле ``_speech`` несёт метаданные
реплики (:func:`chunk_from_request`), старый JSON - :mod:`.speech_compat`.
``at``/``boundary=sentence``/``commit=false`` в этой части не исполняются и
получают честный ``rejected`` (следующая часть PR-3).
"""

from __future__ import annotations

import html
import json
import re
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from rob_box_core.speech_client import ANY_AGENT, EPOCH_ALL, PRIORITY_OPERATOR_U8
from rob_box_core.utterance import Utterance

from . import speech_compat

FRAME_MS = 40
#: Шаг таймера владельца: прогресс по предложениям.
TICK_S = 0.1
#: Запись без перехода дольше этого - потерянная (воркер упал до финала).
STALE_RECORD_S = 300.0
SPEAKER_SINKS = ("", "speaker", "speakers")
_KINDS = ("queued", "synthesized", "started", "progress", "finished", "cancelled", "voice_changed", "rejected")
_TAG_RE = re.compile(r"<[^>]+>")
_SENTENCE_END_RE = re.compile(r"[.!?…]+(?=\s|$)")
_CANCEL_ERRORS = ("stopped", "cancelled")


def plain_text(ssml: str) -> str:
    """Текст реплики без тегов SSML и XML-сущностей - по нему считаются символы."""
    return " ".join(html.unescape(_TAG_RE.sub(" ", ssml or "")).split())


def sentence_ends(text: str) -> Tuple[int, ...]:
    """Смещения концов предложений; последнее - длина текста."""
    ends = [m.end() for m in _SENTENCE_END_RE.finditer(text)]
    if not ends or ends[-1] != len(text):
        ends.append(len(text))
    return tuple(ends)


def envelope_levels(audio: Any, sample_rate: int, frame_ms: int = FRAME_MS) -> List[float]:
    """RMS 0..1 по окнам ``frame_ms`` (стерео - по первому каналу)."""
    samples = np.asarray(audio, dtype=np.float32)
    if samples.ndim > 1:
        samples = samples[:, 0]
    win = max(1, int(sample_rate * frame_ms / 1000))
    count = -(-samples.size // win)
    padded = np.zeros(count * win, dtype=np.float32)
    padded[: samples.size] = samples
    rms = np.sqrt(np.mean(np.square(padded.reshape(count, win)), axis=1)) if count else []
    return [round(float(v), 4) for v in np.clip(rms, 0.0, 1.0)]


@dataclass
class SpeechMeta:
    """Метаданные реплики из ``SpeechRequest`` (или выведенные из старого JSON)."""

    agent: str = "personality"
    epoch: int = 0
    turn_id: str = ""
    interruptible: bool = True

    @classmethod
    def from_chunk(cls, chunk: Dict[str, Any]) -> "SpeechMeta":
        raw = chunk.get("_speech")
        if isinstance(raw, dict):
            return cls(**{k: raw[k] for k in asdict(cls()) if k in raw})
        return speech_compat.meta_from_legacy(chunk, cls)


def unsupported_field(msg: Any) -> str:
    """Поле ``SpeechRequest``, которое владелец ещё не исполняет (``""`` - всё в порядке)."""
    if (msg.sink or "") not in SPEAKER_SINKS:
        return "sink_unsupported"
    if msg.at > 0 or msg.boundary != 0 or not msg.commit:
        return "timing_unsupported"  # at / boundary=sentence / commit=false
    return ""


def chunk_from_request(msg: Any) -> Dict[str, Any]:
    """Словарь-чанк конвейера ``tts_node`` из ``SpeechRequest``."""
    text = msg.text or ""
    meta = SpeechMeta(agent=msg.agent or "personality", epoch=int(msg.epoch),
                      turn_id=msg.turn_id, interruptible=bool(msg.interruptible))
    chunk: Dict[str, Any] = {
        "ssml": text if text.lstrip().startswith("<speak") else Utterance(text=text).ssml,
        "speech_id": msg.speech_id or str(uuid.uuid4()),
        "sink": "speaker",
        "priority": "operator" if msg.priority >= PRIORITY_OPERATOR_U8 else "normal",
        "emotion": msg.emotion or "neutral",
        "_speech": asdict(meta),
    }
    for key in ("voice", "language"):
        if getattr(msg, key):
            chunk[key] = getattr(msg, key)
    return chunk


@dataclass
class SpeechRecord:
    speech_id: str
    meta: SpeechMeta
    text_len: int
    ends: Tuple[int, ...]
    state: str = "queued"
    touched: float = 0.0
    duration_s: float = 0.0
    started_at: Optional[float] = None
    progress_idx: int = 0
    cancelled: bool = False


class SpeechLedger:
    """Журнал живых реплик: эпохи, отмена, произнесённые символы."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._records: Dict[str, SpeechRecord] = {}

    def admit(self, speech_id: str, meta: SpeechMeta, text: str) -> SpeechRecord:
        """Завести запись. Устаревшие эпохи здесь не отбрасываются: эпоху агента
        сбрасывает рестарт его ноды, владелец живёт дольше - «старая» эпоха
        заглушила бы робота. Хвосты отменённого хода режет сам агент."""
        now = self._clock()
        with self._lock:
            for sid in [s for s, r in self._records.items() if now - r.touched > STALE_RECORD_S]:
                del self._records[sid]
            rec = SpeechRecord(speech_id, meta, len(text), sentence_ends(text), touched=now)
            self._records[speech_id] = rec
            return rec

    def get(self, speech_id: Optional[str]) -> Optional[SpeechRecord]:
        with self._lock:
            return self._records.get(speech_id or "")

    def mark(self, speech_id: str, state: str, duration_s: float) -> Optional[SpeechRecord]:
        with self._lock:
            rec = self._records.get(speech_id)
            if rec is not None:
                rec.state, rec.touched, rec.duration_s = state, self._clock(), max(0.0, duration_s)
                rec.started_at = rec.touched if state == "playing" else rec.started_at
            return rec

    def select_cancel(self, speech_id: str, epoch: int, agent: str) -> List[SpeechRecord]:
        """Пометить отменёнными подходящие реплики и вернуть их.

        По ``speech_id`` - ровно её; иначе - реплики агента (``*`` - любого) с
        эпохой не новее ``epoch``, кроме непрерываемых.
        """
        with self._lock:
            if speech_id:
                hit = [self._records[speech_id]] if speech_id in self._records else []
            else:
                hit = [r for r in self._records.values()
                       if agent in (ANY_AGENT, r.meta.agent) and r.meta.epoch <= epoch and r.meta.interruptible]
            for rec in hit:
                rec.cancelled = True
            return hit

    def is_cancelled(self, speech_id: Optional[str]) -> bool:
        rec = self.get(speech_id)
        return bool(rec and rec.cancelled)

    def spoken_chars(self, rec: SpeechRecord) -> int:
        """Сколько символов прозвучало - доля текста по доле времени воспроизведения."""
        if rec.started_at is None or rec.duration_s <= 0:
            return 0
        return int(round(rec.text_len * min(1.0, self.elapsed_s(rec) / rec.duration_s)))

    def elapsed_s(self, rec: SpeechRecord) -> float:
        return 0.0 if rec.started_at is None else max(0.0, self._clock() - rec.started_at)

    def due_progress(self) -> List[Tuple[SpeechRecord, int]]:
        """Концы предложений, пройденные играющими репликами с прошлого вызова."""
        out: List[Tuple[SpeechRecord, int]] = []
        with self._lock:
            playing = [r for r in self._records.values() if r.state == "playing"]
        for rec in playing:
            spoken = self.spoken_chars(rec)
            while rec.progress_idx < len(rec.ends) - 1 and rec.ends[rec.progress_idx] <= spoken:
                out.append((rec, rec.ends[rec.progress_idx]))
                rec.progress_idx += 1
        return out

    def close(self, speech_id: str) -> Optional[SpeechRecord]:
        with self._lock:
            return self._records.pop(speech_id, None)


class SpeechOwner:
    """События, огибающая и отмена владельца речи ``tts_node``."""

    def __init__(self, node: Any, *, output_latency_ms: int = 0, ledger: Optional[SpeechLedger] = None) -> None:
        self._node = node
        self.output_latency_ms = int(output_latency_ms)
        self.ledger = ledger or SpeechLedger()
        self._event_pub: Any = None
        self._envelope_pub: Any = None
        self._msgs: Any = None

    def attach(self) -> None:
        """Подписки и публикаторы ``/voice/speech/*`` на ноде-владельце."""
        from rob_box_dialog_msgs import msg  # noqa: PLC0415 - образ voice-assistant

        node, self._msgs = self._node, msg
        self._event_pub = node.create_publisher(msg.SpeechEvent, "/voice/speech/event", 10)
        self._envelope_pub = node.create_publisher(msg.SpeechEnvelope, "/voice/speech/envelope", 10)
        node.create_subscription(msg.SpeechRequest, "/voice/speech/request", self.on_request, 10)
        node.create_subscription(msg.SpeechCancel, "/voice/speech/cancel", self.on_cancel, 10)
        node.create_timer(TICK_S, self.tick)

    def on_request(self, msg: Any) -> None:
        chunk = chunk_from_request(msg)
        reason = unsupported_field(msg)
        if reason:
            rec = SpeechRecord(chunk["speech_id"], SpeechMeta.from_chunk(chunk), 0, (0,))
            self._emit("rejected", rec, reason=reason)
            return
        self._node.dialogue_callback(_JsonMsg(json.dumps(chunk, ensure_ascii=False)))

    def on_cancel(self, msg: Any) -> None:
        targets = self.ledger.select_cancel(msg.speech_id, int(msg.epoch), msg.agent)
        self._log(
            f"🛑 SpeechCancel speech_id={msg.speech_id[:8] or '-'} epoch<={msg.epoch} "
            f"agent={msg.agent} reason={msg.reason or '-'}: отменено {len(targets)}"
        )
        if any(rec.state == "playing" for rec in targets):
            self._node._interrupt_playback()  # ожидающие снимут FIFO-ворота tts_node

    def cancel_all(self, reason: str) -> None:
        """Старый STOP: отменить всё и заглушить то, что звучит (даже без записи)."""
        self.ledger.select_cancel("", EPOCH_ALL, ANY_AGENT)
        self._log(f"🛑 STOP ({reason}): все реплики отменены")
        self._node._interrupt_playback()

    # -- переходы, которые зовёт конвейер tts_node -------------------------------

    def admit(self, chunk: Dict[str, Any], speech_id: str) -> None:
        meta = SpeechMeta.from_chunk(chunk)
        self._emit("queued", self.ledger.admit(speech_id, meta, plain_text(chunk.get("ssml", ""))))

    def synthesized(
        self, speech_id: Optional[str], duration_s: float, ctx: Dict[str, Any], voice: Optional[str]
    ) -> None:
        rec = self.ledger.mark(speech_id or "", "synthesized", duration_s)
        if rec is None:
            return
        provider, applied = ctx.get("used_provider") or "", ctx.get("used_voice") or ""
        self._emit("synthesized", rec, duration_ms=int(duration_s * 1000), provider=provider)
        if voice and applied and applied != voice:
            self._emit("voice_changed", rec, voice_requested=voice, voice_applied=applied,
                       provider=provider, reason="voice_unavailable")

    def started(self, speech_id: Optional[str], audio: Any, sample_rate: int) -> bool:
        """Огибающая и ``started`` перед ``play_audio``; ``False`` - реплику уже отменили."""
        rec = self.ledger.get(speech_id)
        if rec is None:
            return True
        if rec.cancelled:
            return False
        duration_s = len(audio) / sample_rate if sample_rate else 0.0
        self.ledger.mark(rec.speech_id, "playing", duration_s)
        self._publish_envelope(rec, audio, sample_rate)
        self._emit("started", rec, duration_ms=int(duration_s * 1000))
        return True

    def terminal(self, speech_id: str, success: bool, error: Optional[str], queued: bool) -> None:
        """Финал реплики из единой воронки ``/voice/tts/finished``."""
        rec = None if queued else self.ledger.close(speech_id)
        if rec is None:
            return
        if success:
            self._emit("finished", rec, spoken_chars=rec.text_len)
        elif error in _CANCEL_ERRORS or rec.cancelled:
            self._emit("cancelled", rec, spoken_chars=self.ledger.spoken_chars(rec), reason=error or "cancelled")
        else:
            self._emit("rejected", rec, reason=error or "error")

    def is_cancelled(self, speech_id: Optional[str]) -> bool:
        return self.ledger.is_cancelled(speech_id)

    def tick(self) -> None:
        for rec, chars in self.ledger.due_progress():
            self._emit("progress", rec, spoken_chars=chars)

    # -- публикация ---------------------------------------------------------------

    def _publish_envelope(self, rec: SpeechRecord, audio: Any, sample_rate: int) -> None:
        env = self._msgs.SpeechEnvelope()
        env.speech_id, env.turn_id, env.frame_ms = rec.speech_id, rec.meta.turn_id, FRAME_MS
        t0_ns = self._node.get_clock().now().nanoseconds + self.output_latency_ms * 1_000_000
        env.t0.sec, env.t0.nanosec = divmod(int(t0_ns), 1_000_000_000)
        env.levels = envelope_levels(audio, sample_rate)
        self._envelope_pub.publish(env)

    def _emit(self, kind: str, rec: SpeechRecord, **fields: Any) -> None:
        ev = self._msgs.SpeechEvent()
        ev.kind = _KINDS.index(kind)
        ev.speech_id, ev.turn_id = rec.speech_id, rec.meta.turn_id
        ev.epoch, ev.agent, ev.text_len = int(rec.meta.epoch), rec.meta.agent, rec.text_len
        ev.seconds = float(self.ledger.elapsed_s(rec))
        for key, value in fields.items():
            setattr(ev, key, value)
        ev.stamp = self._node.get_clock().now().to_msg()
        self._event_pub.publish(ev)
        extra = " ".join(f"{k}={v}" for k, v in fields.items())
        self._log(
            f"🗣️ SpeechEvent {kind} speech_id={rec.speech_id[:8]} agent={rec.meta.agent} "
            f"epoch={rec.meta.epoch} text_len={rec.text_len} {extra}"
        )

    def _log(self, text: str) -> None:
        self._node.get_logger().info(text)


class _JsonMsg:
    """Минимальный ``std_msgs/String``: конвейер tts_node читает только ``.data``."""

    __slots__ = ("data",)

    def __init__(self, data: str) -> None:
        self.data = data


class _NullOwner:
    """Владелец для тестовых заглушек ``TTSNode`` без ``__init__``: ничего не публикует."""

    def admit(self, *args: Any) -> None:
        return None

    def synthesized(self, *args: Any) -> None:
        return None

    def terminal(self, *args: Any) -> None:
        return None

    def is_cancelled(self, speech_id: Optional[str]) -> bool:
        return False

    def started(self, *args: Any) -> bool:
        return True


def owner_of(node: Any) -> Any:
    """Владелец речи ноды (пустой - у тестовой заглушки без ``__init__``)."""
    return getattr(node, "_speech", None) or _NullOwner()
