"""Реплики выступления по порядку звучания (ADR-0150 §5.6.3): сегменты ``PENDING → ACTIVE → DONE`` и фразы кода.

Сегменты режет код: сегмент LLM длиннее ``SEGMENT_MAX_CHARS`` делится по строкам и предложениям (:func:`cut`).
:class:`SegmentList` знает, какая реплика следующая, когда граница сегмента (О7), какая версия сегмента звучит после
правки (``edit_pending``), и сколько ещё звучать. Состояния реплики — по событиям владельца речи (их разбирает
:mod:`.perform`); «отдана на звучание» (``committed``) — запрос ``commit=true`` ушёл, звука ещё нет.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Performance
from rob_box_dialog.plan.estimate import SpeechRate
from rob_box_dialog.plan.rap_pauses import breaks_to_newlines
from rob_box_dialog.plan.sync import BeatGrid, bar_at_or_after, layout

_LOG = logging.getLogger(__name__)
#: Знаки, которыми кончается предложение: по ним (и по строкам) :func:`cut` делит текст - одно знание о границе.
SENTENCE_END = ".!?…"
_SPLIT_RE = re.compile(rf"\n+|(?<=[{re.escape(SENTENCE_END)}])\s+")
#: Реплики, которые ещё прозвучат или звучат.
LIVE = ("pending", "committed", "active")
#: Реплики, отданные владельцу речи: граница пройдена, их не правят.
FIXED = ("committed", "active", "done", "cancelled")
#: Реплики, которые начали звучать.
HEARD = ("active", "done", "cancelled")

Row = Tuple[str, str, str]  # (текст, голос, эмоция)


def cut(text: str, limit: int = kn.SEGMENT_MAX_CHARS) -> Tuple[str, ...]:
    """Один сегмент LLM → куски не длиннее ``limit``: по строкам и предложениям, слово — неделимо. Пустой — ``()``."""
    text = text.strip()
    if len(text) <= limit:
        return (text,) if text else ()
    pieces: List[str] = []
    for part in (p.strip() for p in _SPLIT_RE.split(text)):
        while len(part) > limit:  # предложение без границ длиннее лимита — по слову
            head = part[:limit].rsplit(" ", 1)[0] or part[:limit]
            pieces.append(head)
            part = part[len(head):].strip()
        if part and pieces and len(pieces[-1]) + 1 + len(part) <= limit:
            pieces[-1] = f"{pieces[-1]} {part}"
        elif part:
            pieces.append(part)
    return tuple(pieces)


def ends_sentence(piece: str) -> bool:
    """Кусок :func:`cut` кончается знаком конца предложения, а не посреди него (обрыв по слову, строка без знака)."""
    return piece.rstrip().endswith(tuple(SENTENCE_END))


def segment_rows(perf: Performance, skip: int = 0) -> List[Row]:
    """Сегменты выступления после нарезки (первые ``skip`` сегментов LLM — мимо); голос и эмоция — сегмента LLM."""
    count = len(perf.segments)
    voices = perf.voice_per_segment or ("",) * count
    emotions = [e.emotion if e is not None and e.emotion != "neutral" else ""
                for e in (perf.expression_per_segment or (None,) * count)]
    rows = list(zip(perf.segments, voices, emotions))[skip:]
    return [(piece, voice or "", emotion) for text, voice, emotion in rows for piece in cut(text)]


@dataclass
class Line:
    """Реплика: сегмент (``no`` ≥ 1) или фраза кода (``no`` = 0, ``key``). ``fallback`` — прежняя версия сегмента
    на случай, если правка не синтезируется к границе (О7)."""

    no: int
    text: str
    speech_id: str
    voice: str = ""
    emotion: str = ""
    key: str = ""
    state: str = "pending"  # pending → committed → active → done | cancelled; replaced, dropped, late — не звучала
    ready: bool = False
    duration_s: Optional[float] = None
    at: float = 0.0
    synced: bool = False
    heard_at: Optional[float] = None
    spoken_chars: int = 0
    fallback: Optional["Line"] = None
    #: Текст, ушедший владельцу речи (с паузами MiniMax по сетке, :mod:`.rap_pauses`); ``""`` — ещё не отдан.
    speak: str = ""

    @property
    def tts_text(self) -> str:
        """Что синтезировать: отданный текст (пред-синтез и звучание — одним текстом) или исходный."""
        return self.speak or breaks_to_newlines(self.text)  # <br/> в синтез не уходит

    def length_s(self, rate: SpeechRate) -> float:
        """Длительность: синтезированная или оценка по скорости речи."""
        return self.duration_s or len(self.text) / rate.chars_per_s

    def ends_at(self) -> Optional[float]:
        """Когда звучащая реплика кончится (по факту старта); не начата или длительность неизвестна — ``None``."""
        return None if self.heard_at is None or not self.duration_s else self.heard_at + self.duration_s


class SegmentList:
    """Реплики выступления по порядку звучания; ``gone`` — не прозвучавшие версии (правка, отказ)."""

    def __init__(self, rows: Sequence[Row], new_id: Callable[[], str], rate: SpeechRate) -> None:
        self._new_id = new_id
        self.rate = rate
        self.lines: List[Line] = [Line(no, text, new_id(), voice, emotion)
                                  for no, (text, voice, emotion) in enumerate(rows, 1)]
        self.gone: List[Line] = []

    # -- кто где -------------------------------------------------------------------------------------------------------

    def find(self, speech_id: str) -> Optional[Line]:
        """Реплика (или запасная версия сегмента) по ``speech_id``; чужая — ``None``."""
        candidates = [*self.lines, *self.gone, *(line.fallback for line in self.lines if line.fallback)]
        return next((line for line in candidates if line.speech_id == speech_id), None)

    def top(self, states: Sequence[str]) -> int:
        """Наибольший номер сегмента в состояниях ``states`` (0 — нет таких)."""
        return max((line.no for line in self.lines if line.state in states), default=0)

    def first(self, state: str) -> Optional[Line]:
        return next((line for line in self.lines if line.state == state), None)

    def said(self) -> List[str]:
        """Тексты сегментов, что уже прозвучали или отданы на звучание (``FIXED``), по порядку; оборванный - сколько
        успел прозвучать. Правка не повторяет их (:func:`.repeats.drop_said`)."""
        return [line.text[:line.spoken_chars] if line.state == "cancelled" else line.text
                for line in self.lines if line.no and line.state in FIXED]

    def live(self) -> bool:
        return any(line.state in LIVE for line in self.lines)

    def segments(self) -> List[Line]:
        return [line for line in self.lines if line.no]

    def insert(self, line: Line) -> Line:
        """Фраза кода — перед первой не отданной репликой (там её поставит очередь владельца речи): после уже
        прозвучавшего (вопрос о музыке), до куплетов («Поехали!»)."""
        pending = next((i for i, x in enumerate(self.lines) if x.state == "pending"), len(self.lines))
        self.lines.insert(pending, line)
        return line

    # -- граница сегмента (О7) -----------------------------------------------------------------------------------------

    def due(self, now: float) -> Optional[Tuple[Line, float]]:
        """``(следующая, конец предыдущей)``, если пора отдавать следующую на звучание: ничего ещё не звучало;
        предыдущая кончилась; звучит и до конца ≤ ``PERFORM_COMMIT_AHEAD_S`` (или длительность неизвестна — порядок
        держит очередь владельца речи). Отданная, но не начатая — ждём её старта. Версия — по правилу О7
        (:meth:`_version`). Конец предыдущей — когда владелец речи её отпустит: звучание плюс
        ``SPEECH_RELEASE_TAIL_S`` (раньше следующая не зазвучит, такт в этом хвосте — опоздание); неизвестен — 0."""
        nxt = self.first("pending")
        prev = next((line for line in reversed(self.lines) if line.state in FIXED), None)
        if nxt is None or (prev is not None and not _boundary(prev, now)):
            return None
        line = self._version(nxt, prev)
        end = prev.ends_at() if prev is not None else None
        return None if line is None else (line, end + kn.SPEECH_RELEASE_TAIL_S if end else 0.0)

    def _version(self, line: Line, prev: Optional[Line]) -> Optional[Line]:
        """Правка, если синтезирована; иначе прежняя версия, если готова и граница уже есть; иначе ждать."""
        old = line.fallback
        if line.ready:
            if old is not None:
                old.state, line.fallback = "replaced", None
                self.gone.append(old)
            return line
        if old is None or not old.ready or prev is None:
            return None
        _LOG.warning(f"правка сегмента {line.no} не синтезирована к границе — звучит прежняя версия (О7)")
        line.state, line.fallback = "late", None
        self.lines[self.lines.index(line)] = old
        self.gone.append(line)
        return old

    def replace_tail(self, first_no: int, rows: Sequence[Row]) -> Tuple[List[Line], List[int]]:
        """Сегменты с ``first_no`` (все ещё не отданы на звучание) → новые; прежняя готовая версия — запасная на
        границу, остальные прежние — ``replaced``. ``(новые, номера заменённых и добавленных)``."""
        old: Dict[int, Line] = {line.no: line for line in self.lines if line.no >= first_no}
        new = [Line(no, text, self._new_id(), voice, emotion, fallback=_spare(old.get(no)))
               for no, (text, voice, emotion) in enumerate(rows, first_no)]
        spare = {id(line.fallback) for line in new}
        dropped = [line for line in old.values() if id(line) not in spare]
        for line in dropped:
            line.state = "replaced"
        self.gone += dropped
        self.lines = [line for line in self.lines if line.no < first_no] + new
        return new, sorted(set(old) | {line.no for line in new})

    # -- прогноз -------------------------------------------------------------------------------------------------------

    def predicted_end(self, now: float, grid: Optional[BeatGrid]) -> float:
        """Когда кончится речь: звучащая — по факту старта, остальные — раскладкой по тактам (:func:`.sync.layout`)."""
        active = self.first("active")
        t = now if active is None else max(now, active.ends_at() or now + active.length_s(self.rate))
        durations = [line.length_s(self.rate) for line in self.lines if line.state in ("pending", "committed")]
        if not durations:
            return t
        starts = layout(durations, grid, t if grid is None else bar_at_or_after(grid, t))
        return starts[-1] + durations[-1]

    def overdue(self, now: float) -> List[Line]:
        """Звучащие без ``finished`` дольше длительности + ``PLAN_WAIT_SLACK_S`` — событие потеряно."""
        return [line for line in self.lines
                if line.state == "active" and (line.ends_at() or now) + kn.PLAN_WAIT_SLACK_S < now]

    def heard_stats(self) -> Tuple[int, int, bool]:
        """``(сегментов дочитано, символов прозвучало, все начатые — на такте)``."""
        heard = [line for line in (*self.lines, *self.gone) if line.no and line.state in ("done", "cancelled")]
        done = sum(1 for line in heard if line.state == "done")
        return done, sum(line.spoken_chars for line in heard), bool(heard) and all(line.synced for line in heard)


def _spare(line: Optional[Line]) -> Optional[Line]:
    """Прежняя версия сегмента годится в запас (О7), только если уже синтезирована."""
    return line if line is not None and line.ready else None


def _boundary(prev: Line, now: float) -> bool:
    if prev.state in ("done", "cancelled"):
        return True
    if prev.state != "active":
        return False
    end = prev.ends_at()
    return end is None or now >= end - kn.PERFORM_COMMIT_AHEAD_S


__all__ = ["FIXED", "HEARD", "LIVE", "Line", "Row", "SegmentList", "cut", "segment_rows"]
