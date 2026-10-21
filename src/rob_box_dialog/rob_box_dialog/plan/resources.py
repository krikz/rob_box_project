"""Кто что занял — по событиям владельцев ресурсов (ADR-0150 §5.6.1). Статус задачи — только событие владельца.

:class:`ResourceState` кормят колбэки хоста (транспорт снаружи, модуль чистый): ``SpeechEvent`` владельца речи,
``SoundEvent`` звукового узла, снимок ``/voice/music/state`` плеера, результат/обратная связь Nav2. Он отвечает, какие
ресурсы (столбцы ``knowledge.COMPOSITION_BUSY``) заняты, каким блоком (``block_id`` = ``speech_id`` / ``request_id`` /
``track_id`` / ``goal_id``), до какой границы (``knowledge.RESOURCE_BOUNDARY``) и когда примерно освободятся.

Лицо и кольцо (столбец ``face``) не ведутся: у ``ExpressionEvent`` нет события конца, а все клетки столбца ``face`` в
таблице — MERGE (держит тест), ждать там нечего. Публикация запроса блок не открывает: открывает только ``started``
владельца (канал VOICE, который считал задачу выполненной по публикации, V28, — не повторяется).
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, replace
from typing import Callable, Dict, Mapping, Optional, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.plan.estimate import SpeechRate, speech_total_s

_LOG = logging.getLogger(__name__)
#: События ``SpeechEvent``/``SoundEvent``, после которых реплика/звук ресурс больше не занимает.
_SPEECH_END = ("finished", "cancelled", "rejected")
_SOUND_END = ("finished", "skipped")
#: Ресурсы, которые ведёт снимок (переиздаётся), а не пара событий start/end — потерянным не считаются.
_SNAPSHOT_FED = ("backing", "track")


@dataclass(frozen=True)
class Block:
    """Занятый ресурс: чем (``block_id``, ход, эпоха), с какого момента и до какого (оценка; ``None`` — неизвестно)."""

    resource: str
    block_id: str
    started_at: float
    ends_at: Optional[float] = None
    turn_id: str = ""
    epoch: int = 0

    @property
    def boundary(self) -> str:
        """Где ресурс можно сменить: конец предложения, сегмента, такт, фраза трека или сразу."""
        return kn.RESOURCE_BOUNDARY[self.resource]

    def eta_s(self, now: float) -> Optional[float]:
        """Через сколько секунд освободится (оценка); ``None`` — владелец не сообщил, а оценить не из чего."""
        return None if self.ends_at is None else max(0.0, self.ends_at - now)


class ResourceState:
    """Снимок занятости ресурсов одного агента по событиям владельцев. ``clock`` — монотонные секунды."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._blocks: Dict[str, Block] = {}
        self._speech_size: Dict[str, Tuple[int, int]] = {}  # speech_id → (text_len, duration_ms)
        self.rate = SpeechRate()

    def on_speech(self, kind: str, speech_id: str, *, turn_id: str = "", epoch: int = 0, text_len: int = 0,
                  duration_ms: int = 0, spoken_chars: int = 0, seconds: float = 0.0, perform: bool = False) -> None:
        """``SpeechEvent`` (вид — имя ``KIND_*`` в нижнем регистре). ``perform`` — сегмент выступления (PR-7c)."""
        col = "perform" if perform else "speech"
        if kind == "synthesized" and duration_ms > 0:
            text, _ = self._speech_size.get(speech_id, (text_len, 0))
            self._speech_size[speech_id] = (text, duration_ms)
        elif kind == "started":
            size = (text_len, duration_ms or self._speech_size.get(speech_id, (0, 0))[1])
            self._speech_size[speech_id] = size
            self._open(col, speech_id, speech_total_s(size[0], size[1], self.rate), turn_id, epoch)
        elif kind == "progress":
            self.rate.observe(spoken_chars, seconds)
            self._progress(col, speech_id, spoken_chars, seconds)
        elif kind in _SPEECH_END:
            if kind == "finished":
                self.rate.observe(spoken_chars or self._speech_size.get(speech_id, (0, 0))[0], seconds)
            self._speech_size.pop(speech_id, None)
            self._close(col, speech_id)

    def on_sound(self, kind: str, request_id: str) -> None:
        """``SoundEvent``: ``started`` занимает SFX, ``finished``/``skipped`` того же запроса освобождает."""
        if kind == "started":
            self._open("sfx", request_id, None)
        elif kind in _SOUND_END:
            self._close("sfx", request_id)

    def on_music(self, playing: bool, track_id: str = "", *, backing: bool = False,
                 remaining_s: Optional[float] = None) -> None:
        """Снимок плеера: играет ли, что (``track_id``), подложка ли выступления, сколько осталось (если конечный)."""
        if not playing:
            self._blocks.pop("track", None)
            self._blocks.pop("backing", None)
            return
        col = "backing" if backing else "track"
        self._blocks.pop("track" if backing else "backing", None)
        current = self._blocks.get(col)
        if current is not None and current.block_id == track_id:
            ends = None if remaining_s is None else self._clock() + remaining_s
            self._blocks[col] = replace(current, ends_at=ends)
        else:
            self._open(col, track_id, remaining_s)

    def on_motion(self, active: bool, goal_id: str = "", remaining_s: Optional[float] = None) -> None:
        """Nav2: цель принята/едем (``active``, ``estimated_time_remaining`` из feedback) или результат цели."""
        if active:
            current = self._blocks.get("motion")
            started = current.started_at if current is not None and current.block_id == goal_id else self._clock()
            ends = None if remaining_s is None else self._clock() + remaining_s
            self._blocks["motion"] = Block("motion", goal_id, started, ends)
        else:
            self._close("motion", goal_id)

    def busy(self) -> Mapping[str, Block]:
        """Занятые ресурсы сейчас. Блок из событий без конца дольше ``RESOURCE_STALE_S`` — потерян (WARNING)."""
        now = self._clock()
        for col, block in list(self._blocks.items()):
            if col not in _SNAPSHOT_FED and now - block.started_at > kn.RESOURCE_STALE_S:
                _LOG.warning(f"⚠️ [plan] {col} {block.block_id}: нет события конца {now - block.started_at:.0f} с —"
                             " считаю свободным")
                del self._blocks[col]
        return dict(self._blocks)

    def eta_s(self, resource: str) -> Optional[float]:
        """Через сколько секунд освободится ``resource``: свободен — 0, неизвестно — ``None``."""
        block = self.busy().get(resource)
        return 0.0 if block is None else block.eta_s(self._clock())

    def _open(self, col: str, block_id: str, total_s: Optional[float], turn_id: str = "", epoch: int = 0) -> None:
        now = self._clock()
        self._blocks[col] = Block(col, block_id, now, None if total_s is None else now + total_s, turn_id, epoch)

    def _progress(self, col: str, speech_id: str, spoken_chars: int, seconds: float) -> None:
        block = self._blocks.get(col)
        if block is None or block.block_id != speech_id:
            return
        text_len, duration_ms = self._speech_size.get(speech_id, (0, 0))
        if duration_ms > 0:
            left = duration_ms / 1000.0 - seconds
        elif text_len > 0:
            left = max(0, text_len - spoken_chars) / self.rate.chars_per_s
        else:
            return
        self._blocks[col] = replace(block, ends_at=self._clock() + max(0.0, left))

    def _close(self, col: str, block_id: str) -> None:
        block = self._blocks.get(col)
        if block is not None and block.block_id == block_id:
            del self._blocks[col]


__all__ = ["Block", "ResourceState"]
