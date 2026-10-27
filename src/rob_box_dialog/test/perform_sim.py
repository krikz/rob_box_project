"""Фейковые владельцы для сценариев выступления (PR-7c #3700): дискретная симуляция времени.

* речь — как tts_node (PR-3): ``commit=false`` → ``synthesized{duration_ms}`` через ``synth_s``; ``commit=true`` —
  очередь FIFO, реплика звучит не раньше ``at`` (ждёт ``at − output_latency``), ``started`` → ``finished`` через
  длительность; отмена по ``speech_id``: не начатая — сразу, звучащая с ``sentence`` — на конце предложения;
* музыка — как ``PlayerOwner``: исход запроса подложки по сценарию попыток (``ok``/``rejected:<reason>``/``silent``)
  приходит ответом тула (:meth:`Sim.backing_outcome` → ``PerformBlock.on_backing``), ``started{bpm, beat_at,
  bar_beats, start_beat}``; события плеера о любой музыке — :meth:`Sim.music_event`; ``MusicStop`` записывается.

Хост — :meth:`Sim.apply`: эффекты блока → фейковые владельцы; события владельцев → блок; таймер блока — 0.1 с.
"""

import heapq
import itertools
import re
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from rob_box_dialog.command import Performance
from rob_box_dialog.plan.backing import Backing, MusicStop
from rob_box_dialog.plan.perform import AskReplace, Cancel, PerformBlock, Say
from rob_box_dialog.plan.plan_events import PlanEvent

LATENCY_MS = 550
_SENTENCE_END = re.compile(r"[.!?…]+(?=\s|$)")


@dataclass
class Heard:
    speech_id: str
    text: str
    key: str
    at: float
    start: float  # момент звучания (событие started + задержка вывода)
    end: Optional[float] = None
    outcome: str = ""
    spoken_chars: int = 0


@dataclass
class Sim:
    perf: Performance
    cps: float = 12.0                      # «настоящая» скорость голоса фейкового TTS
    synth_s: Callable[[str], float] = lambda text: 1.0
    music: Tuple[str, ...] = ("ok",)        # исход каждой попытки подложки
    music_delay_s: float = 1.5
    bpm: float = 120.0
    fail_presynth: Tuple[str, ...] = ()     # тексты, чей пред-синтез владелец речи отвергает
    hiccup: Dict[str, float] = field(default_factory=dict)  # текст → задержка старта сверх at (сбой очереди)
    tts_provider: str = ""                 # провайдер TTS у хоста: паузы по сетке строк — у minimax (О11)
    t: float = 1000.0
    block: PerformBlock = field(init=False)
    logs: List[str] = field(default_factory=list)
    heard: List[Heard] = field(default_factory=list)
    plan_events: List[PlanEvent] = field(default_factory=list)
    effects: List[Tuple[float, object]] = field(default_factory=list)
    music_log: List[Tuple[float, object]] = field(default_factory=list)
    asks: List[AskReplace] = field(default_factory=list)

    def __post_init__(self) -> None:
        ids = itertools.count(1)
        self.block = PerformBlock(self.perf, "blk-perform", output_latency_ms=LATENCY_MS,
                                  new_id=lambda: f"s{next(ids):02d}", log=self.logs.append,
                                  tts_provider=self.tts_provider)
        self._q: List[Tuple[float, int, Callable[[], None]]] = []
        self._seq = itertools.count()
        self._texts: Dict[str, Say] = {}
        self._ready: Dict[str, float] = {}       # speech_id → когда синтез готов
        self._fifo: List[Say] = []
        self._playing: Optional[Heard] = None
        self._attempt = 0
        self.track_started_at: Optional[float] = None

    # -- время -----------------------------------------------------------------------------------------------------

    def at(self, when: float, fn: Callable[[], None]) -> None:
        heapq.heappush(self._q, (when, next(self._seq), fn))

    def run(self, for_s: float = 400.0, stop: Optional[Callable[[], bool]] = None) -> None:
        """Крутить ``for_s`` секунд или до ``stop()``; таймер блока — каждые 0.1 с."""
        end = self.t + for_s
        next_tick = self.t
        while True:
            if stop is not None and stop():
                return
            due = self._q[0][0] if self._q else float("inf")
            if next_tick <= due:
                if next_tick > end or self.block.ended and not self._q:
                    return
                self.t = next_tick
                self.apply(self.block.tick(self.t))
                next_tick = round(next_tick + 0.1, 6)
                continue
            when, _, fn = heapq.heappop(self._q)
            if when > end:
                return
            self.t = max(self.t, when)
            fn()

    def start(self, playing: str = "") -> None:
        self.apply(self.block.start(self.t, playing))

    # -- хост ------------------------------------------------------------------------------------------------------

    def apply(self, effects) -> None:
        for effect in effects:
            self.effects.append((self.t, effect))
            if isinstance(effect, PlanEvent):
                self.plan_events.append(effect)
            elif isinstance(effect, Say):
                self._say(effect)
            elif isinstance(effect, Cancel):
                self._cancel(effect)
            elif isinstance(effect, Backing):
                self._backing(effect)
            elif isinstance(effect, MusicStop):
                self.music_log.append((self.t, effect))
            elif isinstance(effect, AskReplace):
                self.asks.append(effect)

    def speech_event(self, kind: str, speech_id: str, **kw) -> None:
        self.apply(self.block.on_speech(kind, speech_id, self.t, **kw))

    def music_event(self, kind: str, track_id: Optional[str], fields=None) -> None:
        """Событие плеера из топика — о любой музыке (своя подложка узнаётся только по её ``track_id``)."""
        self.apply(self.block.on_music(kind, track_id, self.t, fields or {}))

    def backing_outcome(self, kind: str, track_id: Optional[str], fields=None) -> None:
        """Исход нашего запроса подложки — ответ тула ``request_music(intent=backing)``."""
        self.apply(self.block.on_backing(kind, track_id, self.t, fields or {}))

    # -- фейковый tts_node -------------------------------------------------------------------------------------------

    def duration(self, text: str) -> float:
        return round(len(text) / self.cps, 3)

    def _say(self, say: Say) -> None:
        self._texts[say.speech_id] = say
        if not say.commit and say.text in self.fail_presynth:
            self.at(self.t + 0.2, lambda: self.speech_event("rejected", say.speech_id))
            return
        if not say.commit:
            ready = self.t + self.synth_s(say.text)
            self._ready[say.speech_id] = ready
            self.at(ready, lambda: self.speech_event("synthesized", say.speech_id,
                                                     duration_ms=int(self.duration(say.text) * 1000)))
            return
        self._ready.setdefault(say.speech_id, self.t + self.synth_s(say.text))  # фраза кода — синтез в пути
        self._fifo.append(say)
        self._next()

    def _next(self) -> None:
        if self._playing is not None or not self._fifo:
            return
        say = self._fifo[0]
        lat = LATENCY_MS / 1000.0
        begin = max(self.t, self._ready[say.speech_id], say.at - lat if say.at else 0.0) + self.hiccup.get(say.text, 0)
        self._playing = Heard(say.speech_id, say.text, say.key, say.at, begin + lat)
        self.at(begin, lambda: self._begin(say))

    def _begin(self, say: Say) -> None:
        playing = self._playing
        if playing is None or playing.speech_id != say.speech_id:
            return
        self._fifo.pop(0)
        self.heard.append(playing)
        dur = self.duration(say.text)
        self.speech_event("started", say.speech_id, duration_ms=int(dur * 1000))
        self.at(self.t + dur, lambda: self._finish(playing, "finished", len(say.text)))

    def _finish(self, heard: Heard, outcome: str, chars: int) -> None:
        if heard.outcome:
            return
        heard.outcome, heard.spoken_chars, heard.end = outcome, chars, self.t + LATENCY_MS / 1000.0
        self._playing = None
        self.speech_event(outcome, heard.speech_id, spoken_chars=chars)
        self._next()

    def _cancel(self, cancel: Cancel) -> None:
        waiting = [s for s in self._fifo if s.speech_id == cancel.speech_id]
        playing = self._playing
        if playing is not None and playing.speech_id == cancel.speech_id and playing in self.heard:
            self._cancel_playing(playing, cancel.boundary)
        elif waiting:
            self._fifo.remove(waiting[0])
            if playing is not None and playing.speech_id == cancel.speech_id:
                self._playing = None  # ждал at — снят без звука
            self.speech_event("cancelled", cancel.speech_id, spoken_chars=0)
            self._next()

    def _cancel_playing(self, playing: Heard, boundary: str) -> None:
        text, dur = playing.text, self.duration(playing.text)
        elapsed = self.t - (playing.start - LATENCY_MS / 1000.0)
        spoken = int(len(text) * min(1.0, elapsed / dur))
        if boundary != "sentence":
            self._finish(playing, "cancelled", spoken)
            return
        ends = [m.end() for m in _SENTENCE_END.finditer(text)] + [len(text)]
        stop_chars = next(e for e in ends if e >= spoken)
        self.at(self.t + max(0.0, (stop_chars - spoken) / len(text) * dur),
                lambda: self._finish(playing, "cancelled", stop_chars))

    # -- фейковый плеер ----------------------------------------------------------------------------------------------

    def _backing(self, backing: Backing) -> None:
        self.music_log.append((self.t, backing))
        if backing.extend:
            return
        outcome = self.music[min(self._attempt, len(self.music) - 1)]
        self._attempt += 1
        if outcome == "ok":
            track = f"trk{self._attempt}"
            when = self.t + self.music_delay_s

            def started() -> None:
                self.track_started_at = self.t + 0.5
                self.backing_outcome("started", track, {"bpm": self.bpm, "beat_at": self.t + 0.5, "bar_beats": 4,
                                                        "start_beat": 32.0})
            self.at(when, started)
        elif outcome.startswith("rejected:"):
            reason = outcome.split(":", 1)[1]
            self.at(self.t + 0.4, lambda: self.backing_outcome("rejected", None, {"reason": reason}))
        # "silent" — плеер молчит: started не придёт

    # -- выборки -----------------------------------------------------------------------------------------------------

    def segments_heard(self) -> List[Heard]:
        return [h for h in self.heard if not h.key]

    def events(self, kind: str) -> List[PlanEvent]:
        return [e for e in self.plan_events if e.kind == kind]

    def music_stops(self) -> List[MusicStop]:
        return [e for _, e in self.music_log if isinstance(e, MusicStop)]

    def backings(self) -> List[Backing]:
        return [e for _, e in self.music_log if isinstance(e, Backing)]

    def active_no(self) -> Optional[int]:
        active = self.block.lines.first("active")
        return None if active is None else active.no
