"""Подложка выступления на деке владельца плеера (ADR-0150 §5.6.3, §9.1 ревизия 5; PR-7c ч.2 #3700).

``request_music(intent=backing, mood, min_seconds)`` — бит под рэп/песню/сказку робота. Ставит его код выступления
(``rob_box_dialog.plan.perform``), а не выбор LLM:

* клубный трек по настроению собирается тем же путём, что ``request_music(intent=track)`` (синтез, без генерации и
  без LLM на пути звука) и играет повтором формы;
* играет столько проходов формы, сколько нужно речи (``min_seconds`` от сейчас — темп выступление до ``started`` не
  знает, долю считает движок), дальше — ``finished``; за ``BACKING_LEAD_BEATS`` до конца — ``nearly_finished``:
  речь вышла длиннее — выступление продлевает подложку тем же вызовом;
* outro (``stop_music(boundary=bar)``) — снять на ближайшей границе такта, а не посреди доли;
* играет другая музыка (трек, сет) — отказ ``busy``: молча её не вытесняем, спросить человека — дело диалога (О9 (б)).

Ответ — по событию ``started`` владельца (6 с, как у всех стартеров): ``bpm``, ``beat_at``, ``bar_beats``,
``start_beat``, ``form_beats`` — по ним выступление ставит речь на такт.
"""

from __future__ import annotations

import logging
import math
import threading
from typing import Any, Callable, Dict, Mapping, Optional

from rob_box_dialog.knowledge import BACKING_INTENT
from rob_box_music.model import BEATS_PER_BAR

_LOG = logging.getLogger(__name__)
#: Настроение подложки по умолчанию (``rob_box_music.knowledge.MOOD_ENERGY``).
DEFAULT_MOOD = "groove"
#: ``nearly_finished`` подложки — за столько долей до конца последнего прохода: вызов тула продления ≪ двух тактов.
BACKING_LEAD_BEATS = 2.0 * BEATS_PER_BAR
#: Поля ``started``, по которым выступление ставит речь на такт (ADR-0150 §5.6.3).
BEAT_FIELDS = ("bpm", "beat_at", "bar_beats", "start_beat", "form_beats")
#: Граница такта ближе этого к текущей доле — уже прошла: outro — на следующей.
_BAR_EPS = 1e-6


class BackingDeck:
    """Подложка одного выступления на деке ``PlayerOwner``: старт, продление, конец проходов, outro на такте.

    ``stage(mood)`` — собранный, но не запущенный клубный трек (``RequestMusicTool._stage_club``) или отказ;
    ``confirm(track_id)`` — событие ``started``/``rejected`` владельца (``MusicEventLog.wait``); ``busy()`` — играет
    ли другая музыка; ``adapter.at``/``beat_now`` — клок Renardo."""

    def __init__(self, owner: Any, adapter: Any, stage: Callable[[str], Dict[str, Any]],
                 confirm: Callable[[Optional[str]], Any], busy: Callable[[], bool], log: Any = None) -> None:
        self._owner = owner
        self._adapter = adapter
        self._stage = stage
        self._confirm = confirm
        self._busy = busy
        self._log = log or _LOG
        self._lock = threading.Lock()
        self.track_id = ""
        self._grid: Dict[str, float] = {}
        self._end_beat = 0.0

    @property
    def playing(self) -> bool:
        """Звучит ли наша подложка (а не трек, поставленный после неё)."""
        return bool(self.track_id) and self._owner.position(self.track_id) is not None

    def order(self, mood: Optional[str], min_seconds: Optional[float]) -> Dict[str, Any]:
        """Подложка должна звучать ещё ``min_seconds``: играет наша — продлить; дека свободна — поставить."""
        need = max(0.0, float(min_seconds or 0.0))
        if self.playing:
            return self._extend(need)
        if self._busy():
            return {"ok": False, "reason": "busy", "detail": "играет другая музыка — подложку поверх не ставлю"}
        return self._start(mood or DEFAULT_MOOD, need)

    def outro(self) -> Dict[str, Any]:
        """Снять подложку на ближайшей границе такта (конец выступления)."""
        if not self.playing:
            return {"ok": True, "was_playing": False}
        start, bar = self._grid.get("start_beat"), float(BEATS_PER_BAR)
        if start is None:  # сетки нет — граница такта неизвестна: снять сразу
            self._owner.finish(self.track_id)
            return {"ok": True, "was_playing": True, "track_id": self.track_id}
        now = float(self._adapter.beat_now())
        end = start + math.ceil((now - start) / bar + _BAR_EPS) * bar
        self._end_at(end, watch=False)
        self._log.info(f"🥁 [backing] outro track_id={self.track_id} на доле {end:g} (сейчас {now:.2f})")
        return {"ok": True, "was_playing": True, "track_id": self.track_id, "end_beat": end}

    def _start(self, mood: str, need: float) -> Dict[str, Any]:
        from .tools_v2 import track_facts  # noqa: PLC0415 — tools_v2 зовёт этот модуль

        staged = self._stage(mood)
        if "program" not in staged:  # не собралось: отказ владельца уже опубликован
            return staged
        program = staged.pop("program")
        played = self._owner.play(program, dj={**track_facts(staged), "backing": True}, once=False)
        if not played.get("ok"):
            return played
        event = self._confirm(played.get("track_id"))
        if event is None or event.event != "started":
            fields = {} if event is None else dict(event.fields)
            return {**played, "ok": False, "reason": fields.get("reason") or "not_started",
                    "detail": fields.get("detail") or "started не пришло"}
        return self._arm(str(played["track_id"]), dict(event.fields), need, mood)

    def _arm(self, track_id: str, fields: Mapping[str, Any], need: float, mood: str) -> Dict[str, Any]:
        """Встала: сетка из ``started``, конец — после стольких проходов формы, сколько нужно речи."""
        grid = {k: float(fields[k]) for k in BEAT_FIELDS if fields.get(k) is not None}
        self.track_id, self._grid, self._end_beat = track_id, grid, 0.0
        result: Dict[str, Any] = {"ok": True, "started": True, "track_id": track_id, "mood": mood, **grid}
        if not {"bpm", "start_beat", "form_beats"} <= set(grid):  # нет сетки — играет повтором до outro/стопа
            self._log.warning(f"⚠️ [backing] started {track_id} без сетки — конец только по outro")
            return result
        result["end_beat"] = self._extend(need)["end_beat"]
        return result

    def _extend(self, need: float) -> Dict[str, Any]:
        """Конец — на границе формы не раньше ``сейчас + need``; раньше назначенного не укорачивается."""
        start, form, bpm = self._grid["start_beat"], self._grid["form_beats"], self._grid["bpm"]
        target = float(self._adapter.beat_now()) + need * bpm / 60.0
        end = start + max(1, math.ceil((target - start) / form)) * form
        if end > self._end_beat:
            self._end_at(end, watch=True)
            self._log.info(f"🥁 [backing] track_id={self.track_id} до доли {end:g} ({(end - start) / form:g} "
                           f"проходов формы, нужно ещё {need:.1f} с)")
        return {"ok": True, "extended": True, "track_id": self.track_id, "end_beat": self._end_beat, **self._grid}

    def _end_at(self, end_beat: float, watch: bool) -> None:
        """Назначить конец подложки; прежний назначенный конец становится холостым."""
        with self._lock:
            self._end_beat = end_beat
        track_id = self.track_id
        self._adapter.at(end_beat, lambda: self._finish_if(track_id, end_beat))
        if watch:
            self._owner.watch(track_id, end_beat, BACKING_LEAD_BEATS, lambda *_: None)

    def _finish_if(self, track_id: str, end_beat: float) -> None:
        with self._lock:
            current = self._end_beat == end_beat and self.track_id == track_id
        if current:
            self._owner.finish(track_id)


def backing_order(deck: Optional[BackingDeck], mood: Optional[str], min_seconds: Optional[float]) -> Dict[str, Any]:
    """``request_music(intent=backing)``: подложка выступления; движка подложки нет — честный отказ."""
    if deck is None:
        return {"ok": False, "reason": "backing_unavailable", "detail": "подложка выступления не подключена"}
    return deck.order(mood, min_seconds)


__all__ = ["BACKING_INTENT", "BACKING_LEAD_BEATS", "BEAT_FIELDS", "BackingDeck", "DEFAULT_MOOD", "backing_order"]
