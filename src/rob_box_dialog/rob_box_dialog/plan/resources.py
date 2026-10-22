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
from collections import deque
from dataclasses import dataclass, replace
from typing import Callable, Deque, Dict, Iterable, List, Mapping, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.plan.estimate import SpeechRate, remaining_s

_LOG = logging.getLogger(__name__)
#: События ``SpeechEvent``/``SoundEvent``, после которых реплика/звук ресурс больше не занимает.
_SPEECH_END = ("finished", "cancelled", "rejected")
_SPEECH_LIVE = ("queued", "synthesized", "started", "progress")
#: Сколько последних законченных реплик помнить: ``reserve`` после их конца не должен «оживить» реплику.
_ENDED_MEMORY = 64
_SOUND_END = ("finished", "skipped")
#: Ресурсы, которые ведёт снимок (переиздаётся) или владелец блока целиком, а не пара событий start/end одной
#: реплики, — потерянными не считаются: выступление (``perform``) длится минутами и само снимается по сроку.
_SNAPSHOT_FED = ("backing", "track", "perform")


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
    """Снимок занятости ресурсов одного агента по событиям владельцев. ``clock`` — монотонные секунды.

    Голос занят, пока у владельца речи есть живая реплика (``queued``/``started`` без конца): очередь tts_node — тоже
    занятость (владелец принял реплику). Реплики, которые хост только что попросил (id из ответа тула), он отмечает
    :meth:`reserve` — до ``queued`` от владельца, чтобы следующее действие хода не проскочило в эту щель."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._blocks: Dict[str, Block] = {}
        # голос: столбец → живые реплики в порядке очереди: speech_id → [text_len, duration_ms, spoken, seconds]
        self._speech: Dict[str, Dict[str, List[float]]] = {"speech": {}, "perform": {}}
        self._started: Dict[str, str] = {}
        self._ended: Deque[str] = deque(maxlen=_ENDED_MEMORY)
        self.rate = SpeechRate()

    def on_speech(self, kind: str, speech_id: str, *, turn_id: str = "", epoch: int = 0, text_len: int = 0,
                  duration_ms: int = 0, spoken_chars: int = 0, seconds: float = 0.0, perform: bool = False) -> None:
        """``SpeechEvent`` (вид — имя ``KIND_*`` в нижнем регистре). ``perform`` — сегмент выступления (PR-7c)."""
        col = "perform" if perform else "speech"
        live = self._speech[col]
        if kind in _SPEECH_END:
            if kind == "finished":
                size = live.get(speech_id)
                self.rate.observe(spoken_chars or int(size[0] if size else 0), seconds)
            live.pop(speech_id, None)
            self._ended.append(speech_id)
            if self._started.get(col) == speech_id:
                del self._started[col]
        elif kind in _SPEECH_LIVE and speech_id not in self._ended:
            size = live.setdefault(speech_id, [0, 0, 0, 0.0])
            size[0] = text_len or size[0]
            size[1] = duration_ms or size[1]
            if kind == "started":
                self._started[col] = speech_id
            elif kind == "progress":
                self.rate.observe(spoken_chars, seconds)
                size[2], size[3] = spoken_chars, seconds
        self._sync_voice(col, turn_id, epoch)

    def reserve(self, speech_ids: Iterable[str], perform: bool = False) -> None:
        """Реплики, которые хост только что отдал владельцу речи (до его ``queued``)."""
        col = "perform" if perform else "speech"
        for speech_id in speech_ids:
            if speech_id and speech_id not in self._ended:
                self._speech[col].setdefault(speech_id, [0, 0, 0, 0.0])
        self._sync_voice(col)

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
        self._blocks.pop("track" if backing else "backing", None)
        self.on_block("backing" if backing else "track", track_id, True, remaining_s)

    def on_block(self, resource: str, block_id: str, active: bool, remaining_s: Optional[float] = None) -> None:
        """Блок, который ведёт его владелец целиком (выступление, ``PlanEvent block_start/prediction/block_end``):
        занят до ``block_end``, конец — по прогнозу владельца."""
        if not active:
            self._close(resource, block_id)
            return
        current = self._blocks.get(resource)
        if current is not None and current.block_id == block_id:
            ends = None if remaining_s is None else self._clock() + remaining_s
            self._blocks[resource] = replace(current, ends_at=ends)
        else:
            self._open(resource, block_id, remaining_s)

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
                self._speech.get(col, {}).clear()
                self._started.pop(col, None)
        return dict(self._blocks)

    def eta_s(self, resource: str) -> Optional[float]:
        """Через сколько секунд освободится ``resource``: свободен — 0, неизвестно — ``None``."""
        block = self.busy().get(resource)
        return 0.0 if block is None else block.eta_s(self._clock())

    def _open(self, col: str, block_id: str, total_s: Optional[float], turn_id: str = "", epoch: int = 0) -> None:
        now = self._clock()
        self._blocks[col] = Block(col, block_id, now, None if total_s is None else now + total_s, turn_id, epoch)

    def _sync_voice(self, col: str, turn_id: str = "", epoch: int = 0) -> None:
        """Блок голоса — по живым репликам: звучащая (или первая в очереди), конец — сумма оставшегося."""
        live = self._speech[col]
        if not live:
            self._blocks.pop(col, None)
            return
        head = self._started.get(col) or next(iter(live))
        left = [remaining_s(int(s[0]), int(s[1]), int(s[2]), s[3], self.rate) for s in live.values()]
        now = self._clock()
        ends = None if any(x is None for x in left) else now + sum(left)
        current = self._blocks.get(col)
        if current is not None and current.block_id == head:
            self._blocks[col] = replace(current, ends_at=ends)
        else:
            self._blocks[col] = Block(col, head, now, ends, turn_id, epoch)

    def _close(self, col: str, block_id: str) -> None:
        block = self._blocks.get(col)
        if block is not None and block.block_id == block_id:
            del self._blocks[col]


__all__ = ["Block", "ResourceState"]
