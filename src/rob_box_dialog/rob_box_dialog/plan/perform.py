"""Выступление — песня, рэп, сказка, стих как одно действие (ADR-0150 §5.6.3, §5.6.4; класс ``action.perform``).

Владелец выступления — этот модуль (решение О6); tts_node и плеер — исполнители. :class:`PerformBlock` — автомат
на событиях владельцев: хост кормит его ``SpeechEvent`` (:meth:`~PerformBlock.on_speech`), исходом своего запроса
подложки (:meth:`~PerformBlock.on_backing`), событиями плеера (:meth:`~PerformBlock.on_music`), таймером
(:meth:`~PerformBlock.tick`) и командами (правка, «хватит», «стой», «добавь бит» командой выступления), а в ответ
получает эффекты — что отправить владельцам: :class:`Say` (``SpeechRequest``),
:class:`Cancel` (``SpeechCancel``), :class:`~.backing.Backing` (``request_music(intent=backing)``),
:class:`~.backing.MusicStop` и ``PlanEvent``. Транспорт (ROS, тулы) — у хоста; часов и потоков здесь нет.

Жизненный цикл — ``Parallel(success_count=1)`` (BehaviorTree.CPP):

* сегменты по порядку уходят в пред-синтез (``commit=false``) окном ``PERFORM_PRESYNTH_WINDOW`` (#4151: пачка
  больше очереди ``/voice/speech/request`` теряла сегменты), подложка запрашивается параллельно (решение
  координатора 22.10: длина — по оценке речи); ``synthesized{duration_ms}`` дают длительности; голос MiniMax и бит —
  пред-синтез ждёт исхода подложки: куплет размечается паузами ``<#x#>`` по её такту (О11, :mod:`.rap_pauses`),
  без сетки или не у MiniMax — как есть, причина в логе ``perform: разметка пауз выключена (…)``;
* первая реплика ждёт исхода подложки: ``started`` — вступление «Поехали!» и сегменты на тактах (:mod:`.sync`);
  отказ — вторая попытка другим настроением (О10), затем фраза по причине ``perform.no_backing.{reason}`` и чтение
  без бита: рэп в тишину молча не начинается;
* сегмент ``PENDING → ACTIVE → DONE`` по событиям владельца речи (:class:`SpeechSide`); следующий отдаётся на
  звучание (``commit=true, at=такт``) за ``PERFORM_COMMIT_AHEAD_S`` до конца звучащего — граница сегмента (О7);
* речь дочитана — подложке outro на ближайшем такте; подложка кончается раньше речи — продление по
  ``nearly_finished``;
* сторож (#4151): чтение идёт, а речи нет дольше ``PERFORM_SILENCE_MAX_S`` — снять выступление с фразой
  ``perform.stalled``, подложку — сразу: без речи она не продлевается.

Правка (``edit_pending``, X3) меняет только ещё не отданные на звучание сегменты; не синтезированная к границе
правка уступает прежней версии (О7). «Хватит» (X5) — звучащий сегмент дочитывается до конца предложения, остальное
снимается, outro. «Стой» — всё сразу. «Добавь бит» командой выступления во время чтения (#993) — подложка без
остановки речи, следующий сегмент — на такте.

Снимает выступление только свою подложку (#4085): ту, что заказало само, по ``track_id`` исхода своего запроса.
Музыку, которую человек заказал тулом музыки, выступление не присваивает: конец, «хватит» с оговоркой «музыку оставь»
и отказ бита её не трогают (гасит её только общий стоп музыки Tier-1 или тул).
"""

from __future__ import annotations

import logging
import uuid
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple, Union

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.cast import voiced
from rob_box_dialog.command import Performance
from rob_box_dialog.plan.backing import Backing, BackingTrack, MusicStop
from rob_box_dialog.plan.estimate import SpeechRate
from rob_box_dialog.plan.plan_events import PlanEvent
from rob_box_dialog.plan.rap_pauses import PAUSE_PROVIDERS, markup
from rob_box_dialog.plan.segments import FIXED, HEARD, Line, SegmentList, segment_rows
from rob_box_dialog.plan.sync import BeatGrid, beat_offset_ms, next_bar

_LOG = logging.getLogger(__name__)
LOG_MARK = "🎤 [perform]"


@dataclass(frozen=True)
class Say:
    """Эффект ``SpeechRequest``: пред-синтез (``commit=False``) или звучание в момент ``at`` (0 — сразу за
    предыдущей). ``key`` — ключ фразы кода (``""`` — сегмент выступления)."""

    speech_id: str
    text: str
    commit: bool = True
    at: float = 0.0
    voice: str = ""
    emotion: str = ""
    key: str = ""


@dataclass(frozen=True)
class Cancel:
    """Эффект ``SpeechCancel`` реплики: ``boundary="sentence"`` — дочитать до конца предложения (X5)."""

    speech_id: str
    boundary: str = "none"


@dataclass(frozen=True)
class AskReplace:
    """Эффект: реплика ``speech_id`` — вопрос «Сейчас играет …, выключить и зачитать?» (О9 (б), О9′); когда она
    прозвучит, хост открывает окно ответа да/нет (``REPLY_WINDOW_S``), ответ — :meth:`PerformBlock.answer`."""

    speech_id: str
    title: str


Effect = Union[Say, Cancel, Backing, MusicStop, PlanEvent, AskReplace]
EventMaker = Callable[..., PlanEvent]


def _new_id() -> str:
    return uuid.uuid4().hex


def presynth(line: Line) -> Say:
    """Пред-синтез реплики: звука нет, владелец речи ответит ``synthesized{duration_ms}``."""
    line.speak = line.tts_text
    return Say(line.speech_id, line.speak, False, 0.0, line.voice, line.emotion, line.key)


def withdraw(lines: SegmentList, active_boundary: str) -> List[Cancel]:
    """Снять речь выступления: звучащую — с границей ``active_boundary`` (``sentence`` — дочитать предложение;
    состояние сменит её событие конца), отданные на звучание — сразу (звука ещё нет), не отданные — не отдавать
    (их отмена выбрасывает готовый пред-синтез у владельца речи)."""
    out: List[Cancel] = []
    for line in lines.lines:
        if line.state == "active":
            out.append(Cancel(line.speech_id, active_boundary))
        elif line.state in ("committed", "pending"):
            out += [Cancel(line.speech_id)] if line.speak or line.state == "committed" else []  # не отданную — молча
            line.state = "cancelled" if line.state == "committed" else "dropped"
    return out


def edit_range(lines: SegmentList, first: int) -> Tuple[str, int, int]:
    """Правка с сегмента ``first``: ``(причина отказа или "", сколько сегментов правки пропустить, с какого номера
    заменять)``. Начатый сегмент не правится; отданный на звучание, но не начатый — тоже (граница пройдена, О7):
    правка, нацеленная на него, идёт со следующего; номер дальше конца — продолжение в конец."""
    if first <= lines.top(HEARD):
        return "edit_started_segment", 0, 0
    fixed = lines.top(FIXED)
    return "", max(0, fixed + 1 - first), min(max(first, fixed + 1), lines.top(("pending",) + FIXED) + 1)


def edit_asks_backing(perf: Performance, foreign_playing: bool) -> bool:
    """Просит ли правка идущего выступления подложку (``request_music(intent=backing)``): только когда названа (настроение
    из ``kn.PERFORM_BACKING_MOODS``) и это не правка куплетов поверх чужой музыки. Живой 28.10 01:19:53: рэп поверх трека,
    который поставил человек (подложки у выступления нет), «и про комарика добавь!» - модель вернула в правке куплетов
    ``backing: beat``, хост заказал бит, плеер ответил ``busy`` - лишний вызов и «бит не заиграл» в логе. Играет чужое
    (``foreign_playing``) - правка с куплетами подложку не заказывает, она остаётся прежней (``none``): бит поверх чужой
    музыки правкой куплетов не спрашивается, это отдельная просьба («добавь бит» без куплетов, #993 - её решает человек и
    ответ плеера, как и раньше)."""
    return perf.backing in kn.PERFORM_BACKING_MOODS and not (perf.segments and foreign_playing)


def context_block(block: "PerformBlock", now: float) -> Dict[str, Any]:
    """Блок контекста LLM (риск §16): спетое и звучащее — дословно, номер первого не начатого сегмента, сколько
    осталось звучать, подложка. По нему LLM правит «с того места» (``edit_pending``)."""
    segments = block.lines.segments()
    active = next((line for line in segments if line.state == "active"), None)
    pending = next((line for line in segments if line.state == "pending"), None)
    return {
        "kind": block.kind,
        "done": [line.text for line in segments if line.state == "done"],
        "active": None if active is None else {"no": active.no, "text": active.text},
        "next_fixed": [line.text for line in segments if line.state == "committed"],
        "first_pending": None if pending is None else pending.no,
        "segments_total": len(segments),
        "remaining_s": round(max(0.0, block.predicted_end(now) - now), 1),
        "backing": block.backing.state,
        "synced": block.backing.grid is not None,
    }


class SpeechSide:
    """Речь выступления: что отдать владельцу речи и что значат его события (реплики — :class:`SegmentList`)."""

    def __init__(self, lines: SegmentList, output_latency_ms: int, new_id: Callable[[], str], event: EventMaker,
                 log: Callable[[str], None], grid: Callable[[], Optional[BeatGrid]]) -> None:
        self.lines = lines
        self._latency_ms = output_latency_ms
        self._new_id = new_id
        self._event = event
        self._log = log
        self._grid = grid
        self._released = 0
        #: Пред-синтез сегментов ждёт исхода подложки: от её сетки зависит разметка пауз MiniMax (О11).
        self.hold = False
        #: Такт подложки (с), по которому сегменты размечаются паузами; ``None`` — разметка выключена.
        self.pause_bar_s: Optional[float] = None

    def commit(self, now: float) -> List[Effect]:
        """На границе — следующую реплику на звучание: на ближайшем такте подложки (без неё — сразу за предыдущей)."""
        due = self.lines.due(now)
        if due is None:
            return []
        line, after = due
        slot = next_bar(self._grid(), now, self._latency_ms, after)
        line.state, line.at, line.synced = "committed", slot.at, slot.synced
        return [*self.release(), Say(line.speech_id, line.tts_text, True, slot.at, line.voice, line.emotion,
                                     line.key)]

    def feed(self) -> List[Say]:
        """Пред-синтез следующих реплик по порядку звучания, пока отданных владельцу речи, но ещё не ушедших на
        звучание меньше ``PERFORM_PRESYNTH_WINDOW`` (#4151): место освобождает реплика, отданная на звучание. Разметка
        включена — с паузами MiniMax по сетке строк. Пока ``hold`` — ничего: сегменты уйдут по исходу подложки
        (:meth:`unhold`)."""
        if self.hold:
            return []
        pending = [line for line in self.lines.lines if line.state == "pending"]
        room = kn.PERFORM_PRESYNTH_WINDOW - sum(1 for line in pending if line.speak)
        out = []
        for line in [line for line in pending if not line.speak][:max(0, room)]:
            mark = markup(line.text, self.pause_bar_s) if line.no and self.pause_bar_s else None
            if mark is not None:
                line.speak = mark.text
                self._log(f"{LOG_MARK} сегмент {line.no}: такты {mark.bars_per_line}, паузы {mark.pauses}")
            out.append(presynth(line))
        return out

    def unhold(self, bar_s: Optional[float]) -> List[Say]:
        """Исход подложки известен: пред-синтез отложенных сегментов (с разметкой по такту ``bar_s`` или без)."""
        self.hold, self.pause_bar_s = False, bar_s
        return self.feed()

    def release(self) -> List[Cancel]:
        """Версии сегментов, которые уже не прозвучат (заменены правкой, уступили на границе), — отмена: владелец речи
        выбрасывает их готовый пред-синтез. Не отданную владельцу (окно пред-синтеза её не дошло) — молча."""
        gone = self.lines.gone[self._released:]
        self._released = len(self.lines.gone)
        return [Cancel(line.speech_id) for line in gone if line.speak]

    def on_event(self, kind: str, line: Line, now: float, duration_ms: int, spoken_chars: int) -> List[Effect]:
        """Переход реплики по ``SpeechEvent``; ``progress`` и не подходящие состоянию события — без перехода."""
        if kind == "started" and line.state == "committed":
            return self._started(line, now, duration_ms)
        if kind in ("finished", "cancelled") and line.state in ("committed", "active"):
            done = kind == "finished"
            return self.over(line, "done" if done else "cancelled", len(line.text) if done else spoken_chars)
        if kind == "synthesized" and line.state == "pending":
            line.ready, line.duration_s = True, duration_ms / 1000.0 or line.duration_s
        elif kind == "rejected":
            self._rejected(line)
        return []

    def over(self, line: Line, state: str, spoken: int) -> List[Effect]:
        line.state, line.spoken_chars = state, spoken
        if not line.no:
            return []
        return [self._event("segment_finished", line.no, synced=line.synced, spoken_chars=spoken,
                            text_len=len(line.text), reason="" if state == "done" else "cancelled")]

    def _started(self, line: Line, now: float, duration_ms: int) -> List[Effect]:
        """Звучит. Встала позже своего такта (очередь, синтез в пути) — уже не на такте: ``synced=false`` честно."""
        line.state, line.heard_at = "active", now + self._latency_ms / 1000.0
        line.duration_s = duration_ms / 1000.0 or line.duration_s
        if line.synced and line.heard_at - line.at > kn.PERFORM_SYNC_TOLERANCE_S:
            _LOG.warning(f"{LOG_MARK} реплика {line.no} опоздала на такт: {line.heard_at - line.at:.3f} с")
            line.synced = False
        if not line.no:
            return []
        eta_ms = int(max(0.0, self.lines.predicted_end(now, self._grid()) - now) * 1000)
        return [self._event("segment_started", line.no, synced=line.synced, text_len=len(line.text),
                            offset_ms=beat_offset_ms(self._grid(), line.heard_at)),
                self._event("prediction", line.no, eta_ms=eta_ms)]

    def _rejected(self, line: Line) -> None:
        """Отказ владельца речи: пред-синтеза — реплика пойдёт синтезом в пути; звучания — пропуск с WARNING."""
        if line.state == "pending":
            line.ready = True
            self._log(f"{LOG_MARK} пред-синтез реплики {line.no} не вышел — синтез в пути")
        elif line.state in ("committed", "active"):
            _LOG.warning(f"{LOG_MARK} реплика {line.no} не прозвучала (rejected) — дальше")
            line.state = "cancelled"

    def phrase(self, key: str, **slots: Any) -> Say:
        """Фраза кода сразу (не пред-синтез): встаёт в очередь владельца речи перед следующим сегментом."""
        text = phrases.text(key, **slots)
        line = self.lines.insert(Line(0, text, self._new_id(), key=key, state="committed", ready=True))
        return Say(line.speech_id, line.text, key=key)

    def lead_in(self) -> Say:
        """Вступление «Поехали!» перед первым сегментом — пред-синтез; встанет на такт, как сегмент."""
        key = "perform.backing_started"
        return presynth(self.lines.insert(Line(0, phrases.text(key), self._new_id(), key=key)))

    def edit(self, perf: Performance) -> Tuple[List[Effect], str]:
        """MERGE в PENDING (:func:`edit_range`): ``(эффекты, причина отказа или "")``."""
        reason, skip, first_no = edit_range(self.lines, perf.from_segment)
        rows = [] if reason else segment_rows(perf, skip)
        if not rows:
            return [], reason or "edit_late"
        _, numbers = self.lines.replace_tail(first_no, rows)
        self._log(f"{LOG_MARK} правка — сегменты {numbers} заменены")
        out: List[Effect] = [*self.release(), *self.feed()]
        return out + [self._event("segment_replaced", no, reason="edit") for no in numbers], ""


class MusicSide:
    """Подложка выступления: заказ и попытки, вопрос о смене чужой музыки (О9 (б), О9′), сетка тактов, продление и
    конец. Данные подложки — :class:`~.backing.BackingTrack`; речь и жизненный цикл — у :class:`PerformBlock`."""

    def __init__(self, kind: str, block: "PerformBlock") -> None:
        self.track = BackingTrack(kind)
        self._block = block

    def begin(self, now: float, playing: str) -> List[Effect]:
        """Старт выступления: бит заказывается сразу; играет чужая музыка ``playing`` — сначала вопрос человеку."""
        if not self.track.wanted:
            return []
        if not playing:
            return [self.track.request(now, self.need(now))]
        self.track.ask(now)
        question = self._block.speech.phrase("perform.ask_replace_music", title=playing)
        return [question, AskReplace(question.speech_id, playing)]

    def on_music(self, kind: str, track_id: Optional[str], now: float, fields: Mapping[str, Any],
                 own: bool = False) -> List[Effect]:
        """Событие о нашей подложке (``BackingTrack.verdict``): ``own`` — исход нашего запроса, иначе — плеера."""
        verdict = self.track.verdict(kind, track_id, now, own)
        if verdict == "ignore":
            return []
        self._block.touch(now)
        if verdict == "late" or self._block.ended:  # встала, когда её уже не ждут — снять, а не оставить играть
            return [MusicStop("now", "backing_late")] if verdict == "late" else []
        handlers: Dict[str, Callable[[], List[Effect]]] = {
            "started": lambda: self._started(track_id or "", fields, now),
            "rejected": lambda: self._rejected(str(fields.get("reason") or "error"), now),
            "nearly": lambda: self._extend(fields, now),
            "ended": lambda: self._ended(now),
        }
        return handlers[verdict]()

    def answer(self, yes: bool, now: float) -> List[Effect]:
        """Ответ на «выключить и зачитать?»: да — снять играющую музыку и поставить бит; нет (или молчание) — читать
        без бита поверх неё, без второй попытки. Вопроса не было — ``[]``."""
        if self._block.ended or self.track.state != "asking":
            return []
        self._block.touch(now)
        self._block.log(f"ответ на смену музыки — {'да' if yes else 'нет'}")
        if yes:
            return [self.track.request(now, self.need(now), replace=True), *self._block.pump(now)]
        return self.failed("busy", now)

    def add(self, kind: str, now: float) -> List[Effect]:
        """«Добавь бит» командой выступления во время чтения (#993, ``perform{backing}`` без куплетов): своя подложка
        без остановки речи, следующий сегмент — на такте, снимается с концом выступления. Музыка, заказанная тулом
        музыки, сюда не идёт: она человека (#4085)."""
        if self._block.ended or self.track.sounding or self.track.unresolved:
            return []
        self.track = BackingTrack(kind if kind in kn.PERFORM_BACKING_MOODS else "beat")
        return [self.track.request(now, self.need(now))]

    def expired(self, now: float) -> List[Effect]:
        """Срок вышел: вопрос без ответа — «нет» (без бита поверх играющей музыки); бит не встал за
        ``PERFORM_BACKING_START_S`` — снять его и читать без."""
        if not self.track.timed_out(now):
            return []
        if self.track.state == "asking":
            return self.answer(False, now)
        return [MusicStop("now", "backing_timeout"), *self.failed("timeout", now)]

    def keep(self) -> None:
        """«А музыку оставь» (#3994): играющий бит доигрывает сам — снимать его концом выступления не будем."""
        if self.track.kept():
            self._block.log(f"бит {self.track.track_id} оставлен играть (музыку оставили)")

    def release(self, now: float, boundary: str, reason: str, states: Tuple[str, ...]) -> List[Effect]:
        """Выступление кончилось или снято: подложку в состояниях ``states`` снять на ``boundary``."""
        if self.track.state not in states:
            return []
        self.track.stopped(now)
        return [MusicStop(boundary, reason)]

    def failed(self, reason: str, now: float) -> List[Effect]:
        """Подложки не будет: фраза по причине и чтение без бита (рэп в тишину молча не начинается)."""
        why = self.track.fail(reason, now)
        self._block.log(f"подложки нет ({reason} → {why}) — читаю без неё")
        held = self._block.speech.hold  # отложенный пред-синтез — первым: фраза отказа синтезируется сама
        unheld: List[Effect] = list(self._block.pauses_off(f"подложки нет: {reason}")) if held else []
        if why == "busy":  # играет чужая музыка (спросили или успела встать) — читаем под неё, не в тишину
            return unheld + self._block.pump(now)
        return [*unheld, self._block.speech.phrase(f"perform.no_backing.{why}"), *self._block.pump(now)]

    def need(self, now: float) -> float:
        """Сколько секунд подложке звучать от сейчас: оценка речи × запас + outro (координатор 22.10)."""
        return (self._block.predicted_end(now) - now) * kn.PERFORM_FORM_MARGIN + kn.PERFORM_OUTRO_S

    def _started(self, track_id: str, fields: Mapping[str, Any], now: float) -> List[Effect]:
        self.track.started(track_id, fields)
        grid = self.track.grid
        self._block.log(f"подложка {track_id} started "
                        + (f"bpm={grid.bpm:g} такт={grid.bar_s:.2f} с" if grid else "без сетки — чтение без тактов"))
        if self._block.phase != "starting":  # #993: добавили во время чтения — следующий сегмент уже на такте
            self._block.pauses_off("бит добавлен посреди чтения — куплеты уже синтезированы")
            return self._block.pump(now)
        return [self._block.speech.lead_in(), *self._block.pauses_on(grid), *self._block.pump(now)]

    def _rejected(self, reason: str, now: float) -> List[Effect]:
        """Отказ подложки: вторая попытка другим настроением, если в ней есть смысл (``busy`` — нет)."""
        if self.track.can_retry() and reason not in kn.PERFORM_NO_RETRY_REASONS:
            self._block.log(f"подложка отказала ({reason}) — вторая попытка")
            return [self.track.request(now, self.need(now))]
        return self.failed(reason, now)

    def _extend(self, fields: Mapping[str, Any], now: float) -> List[Effect]:
        """``nearly_finished``: речь звучит дольше подложки — продлить её проходом формы."""
        if self._block.phase not in ("starting", "running"):
            return []
        extend = self.track.extension(now, fields, self._block.predicted_end(now), self.need(now))
        return [] if extend is None else [extend]

    def _ended(self, now: float) -> List[Effect]:
        self.track.ended()
        self._block.log("подложка кончилась — дальше без тактов (synced=false)")
        return self._block.pump(now)


_SOUNDING = ("requested", "playing")


class PerformBlock:
    """Одно выступление: сегменты, подложка, их общее время жизни. ``now`` — секунды ROS-часов хоста (стенные)."""

    def __init__(self, perf: Performance, block_id: str, *, turn_id: str = "", epoch: int = 0,
                 agent: str = kn.AGENTS[0], output_latency_ms: int = 0, rate: Optional[SpeechRate] = None,
                 new_id: Callable[[], str] = _new_id, log: Callable[[str], None] = _LOG.info,
                 tts_provider: str = "", voice_busy: Callable[[], bool] = lambda: False) -> None:
        self.block_id = block_id
        #: Голос занят чужой речью (снимок владельцев у хоста): наша реплика ждёт в очереди владельца — это не сбой.
        self._voice_busy = voice_busy
        self.kind = perf.kind
        self._ids: Dict[str, Any] = {"turn_id": turn_id, "epoch": epoch, "agent": agent}
        self._log = log
        self.lines = SegmentList(segment_rows(perf), new_id, rate if rate is not None else SpeechRate())
        self.music = MusicSide(perf.backing, self)
        self.speech = SpeechSide(self.lines, output_latency_ms, new_id, self._event, log, lambda: self.backing.grid)
        self.phase = "new"  # new → starting → running → stopping → ended
        self.end_reason = ""
        self._last_event = 0.0
        #: Последний ход речи выступления: старт чтения, ``started``/конец своей реплики — от него считает сторож.
        self._spoke_at = 0.0
        #: Провайдер TTS (фактический у хоста): паузы по сетке строк — только у понимающих ``<#x#>`` (О11).
        self.tts_provider = tts_provider
        #: Роли выступления и их голоса (``ключ роли → голос``, #4084): правка не перекрашивает их — это ``Busy.cast``.
        self.cast: Dict[str, str] = voiced(perf.role_per_segment, perf.voice_per_segment)

    # -- что видят хост, валидатор и LLM ----------------------------------------------------------------------------

    @property
    def backing(self) -> BackingTrack:
        """Подложка выступления (данные; поведение — :class:`MusicSide`)."""
        return self.music.track

    @property
    def ended(self) -> bool:
        return self.phase == "ended"

    @property
    def active_segment(self) -> Optional[int]:
        """``Busy.active_segment`` валидатора: номер последнего сегмента, отданного на звучание (его и раньше не
        правят, ``from_segment`` ≥ ACTIVE+1); 0 — ещё ни одного; ``None`` — выступления нет."""
        return None if self.ended else self.lines.top(FIXED)

    def predicted_end(self, now: float) -> float:
        """Когда кончится речь выступления (раскладка по тактам подложки, если она есть)."""
        return self.lines.predicted_end(now, self.backing.grid)

    def summary(self) -> Dict[str, Any]:
        """Итог для ``ActionResult.data``: сколько сегментов дочитано, символов прозвучало, все ли на такте."""
        done, chars, synced = self.lines.heard_stats()
        return {"segments_done": done, "spoken_chars": chars, "synced": synced, "backing": self.backing.state,
                "backing_track": self.backing.track_id, "backing_reason": self.backing.reason,
                "end_reason": self.end_reason}

    # -- команды хоста ------------------------------------------------------------------------------------------------

    def start(self, now: float, playing: str = "") -> List[Effect]:
        """Начать: пред-синтез сегментов (окном, :meth:`SpeechSide.feed`) и (если нужна) подложка — параллельно.
        ``playing`` — название чужой музыки, что играет сейчас: тогда бит — только если человек согласится её сменить
        (О9 (б), О9′)."""
        if self.phase != "new":
            return []
        self.phase, self._last_event = "starting", now
        out: List[Effect] = [self._event("block_start", reason=self.kind)]
        if self.backing.wanted and self.tts_provider in PAUSE_PROVIDERS:
            self.speech.hold = True  # разметка пауз — по сетке подложки: пред-синтез ждёт её исхода
        else:
            self.pauses_off("нет подложки" if not self.backing.wanted
                            else f"синтез {self.tts_provider or 'неизвестен'}, не MiniMax")
        out += self.speech.feed()
        out += self.music.begin(now, playing)
        self.log(f"{self.kind}: сегментов {len(self.lines.lines)}, подложка {self.backing.mood or 'нет'}")
        return out + self.pump(now)

    def tick(self, now: float) -> List[Effect]:
        """Таймер хоста (~0.1–0.2 с): срок подложки и вопроса, граница сегмента, потерянные события, сторож речи."""
        if self.phase in ("new", "ended"):
            return []
        if now - self._last_event > kn.RESOURCE_STALE_S:
            _LOG.warning(f"{LOG_MARK} {self.block_id[:8]}: нет событий {now - self._last_event:.0f} с — снимаю")
            return self.halt(now, reason="lost")
        if self.silent_s(now) > kn.PERFORM_SILENCE_MAX_S:
            if not self._voice_busy():
                return self._stall(now)
            self._spoke_at = now  # голос говорит чужое: наша реплика в очереди владельца речи, тишина — с его конца
        out: List[Effect] = list(self.music.expired(now))
        for line in self.lines.overdue(now):
            _LOG.warning(f"{LOG_MARK} {self.block_id[:8]}: нет finished реплики {line.no} — считаю дочитанной")
            out += self.speech.over(line, "done", len(line.text))
        return out + self.pump(now)

    def on_speech(self, kind: str, speech_id: str, now: float, *, duration_ms: int = 0,
                  spoken_chars: int = 0) -> List[Effect]:
        """``SpeechEvent`` владельца речи (вид — имя ``KIND_*`` в нижнем регистре). Чужие реплики — ``[]``. Реплику
        отменил не этот модуль (перебивание, «хватит» v1) — выступление останавливается с outro."""
        line = self.lines.find(speech_id)
        if line is None or self.ended:
            return []
        self._last_event = now
        if kind in ("started", "finished", "cancelled"):
            self._spoke_at = now
        out = self.speech.on_event(kind, line, now, duration_ms, spoken_chars)
        if kind == "cancelled" and line.state == "cancelled" and self.phase in ("starting", "running"):
            self.log(f"реплику {line.no} отменил владелец речи — останавливаю")
            return out + self.stop(now)
        return out + self.pump(now)

    def on_music(self, kind: str, track_id: Optional[str], now: float,
                 fields: Optional[Mapping[str, Any]] = None) -> List[Effect]:
        """Событие плеера (``started``/``nearly_finished``/``finished``/``idle``): о нашей подложке — только по её
        ``track_id``; ``started`` другого трека, пока наша играет, — деку заняла другая музыка. Чужой трек, вставший,
        пока бит просили, своим не становится (#4085: его снял бы конец выступления)."""
        return self.music.on_music(kind, track_id, now, fields or {})

    def on_backing(self, kind: str, track_id: Optional[str], now: float,
                   fields: Optional[Mapping[str, Any]] = None) -> List[Effect]:
        """Исход НАШЕГО запроса подложки (ответ ``request_music(intent=backing)``): ``started`` с ``track_id`` и сеткой
        или ``rejected`` с причиной. Только он делает трек своей подложкой (поздний — снимается)."""
        return self.music.on_music(kind, track_id, now, fields or {}, own=True)

    def answer(self, yes: bool, now: float) -> List[Effect]:
        """Ответ человека на «выключить и зачитать?» (:meth:`MusicSide.answer`)."""
        return self.music.answer(yes, now)

    def add_backing(self, backing: str, now: float) -> List[Effect]:
        """«Добавь бит» командой выступления во время чтения (#993, :meth:`MusicSide.add`)."""
        return self.music.add(backing, now)

    def edit(self, perf: Performance, now: float) -> List[Effect]:
        """MERGE в PENDING (``edit_pending``, X3): сегменты с ``perf.from_segment`` заменяются ``perf.segments``.
        Опоздала (сегмент уже звучит) — отказ с фразой ``perform.edit_rejected``; выступления нет — отказ молча."""
        out, reason = self.speech.edit(perf) if self.phase in ("starting", "running") else ([], "not_running")
        if not reason:
            self.cast.update(voiced(perf.role_per_segment, perf.voice_per_segment))
            return out
        self.log(f"правка с сегмента {perf.from_segment} отклонена ({reason})")
        rejected: List[Effect] = [self._event("rejected", perf.from_segment, reason=reason)]
        return rejected + ([self.speech.phrase("perform.edit_rejected")] if reason != "not_running" else [])

    def stop(self, now: float, keep_backing: bool = False) -> List[Effect]:
        """Tier-1 «хватит» (X5): звучащий сегмент — до конца предложения, остальное снимается, затем outro.
        ``keep_backing`` («хватит читать, а музыку оставь», #3994) — играющий бит без outro доигрывает сам; ещё не
        вставший бит снимается, как и без оговорки (оставлять нечего)."""
        if self.ended:
            return []
        self.phase = "stopping"
        out: List[Effect] = list(withdraw(self.lines, "sentence"))
        out += self.music.release(now, "now", "perform_stop", ("requested",))
        if keep_backing:
            self.music.keep()
        return out + self._maybe_end(now)

    def halt(self, now: float, reason: str = "halt", keep_backing: bool = False) -> List[Effect]:
        """«Стой» (Tier-1): речь и подложка — сразу; ``keep_backing`` — играющий бит остаётся (как у :meth:`stop`)."""
        if self.ended:
            return []
        out: List[Effect] = list(withdraw(self.lines, "none"))
        for line in self.lines.lines:
            line.state = "cancelled" if line.state == "active" else line.state
        if keep_backing:
            self.music.keep()
        out += self.music.release(now, "now", reason, _SOUNDING)
        return out + self._end(reason)

    # -- жизненный цикл ----------------------------------------------------------------------------------------------

    def touch(self, now: float) -> None:
        """Пришло событие владельца: выступление живо (срок ``RESOURCE_STALE_S`` — от последнего события)."""
        self._last_event = now

    def log(self, text: str) -> None:
        self._log(f"{LOG_MARK} {self.block_id[:8]}: {text}")

    def pauses_on(self, grid: Optional[BeatGrid]) -> List[Say]:
        """Подложка встала: отложенные сегменты — в пред-синтез, с паузами по её такту (сетки нет — без них)."""
        if not self.speech.hold:
            return []
        if grid is None:
            return self.pauses_off("подложка без сетки тактов")
        self.log(f"perform: разметка пауз MiniMax по сетке строк, такт {grid.bar_s:.2f} с")
        return self.speech.unhold(grid.bar_s)

    def pauses_off(self, reason: str) -> List[Say]:
        """Разметки пауз не будет — честно в логе; отложенные сегменты — в пред-синтез как есть."""
        self.log(f"perform: разметка пауз выключена ({reason})")
        return self.speech.unhold(None) if self.speech.hold else []

    def silent_s(self, now: float) -> float:
        """Сколько секунд чтение стоит без речи (сторож, #4151): идёт чтение, ничего своего не звучит, а реплики ещё
        остались; иначе — 0. Подложка до исхода, вопрос о музыке и «хватит» — не чтение: у них свои сроки."""
        if self.phase != "running" or self.lines.first("active") is not None or not self.lines.live():
            return 0.0
        return now - self._spoke_at

    def pump(self, now: float) -> List[Effect]:
        """Начать, когда решилась подложка; на границе отдать следующую реплику на звучание; дослать пред-синтез
        окном; всё — закончить."""
        if self.phase == "starting" and not self.backing.unresolved:
            self.phase, self._spoke_at = "running", now
        out = self.speech.commit(now) if self.phase == "running" else []
        out += self.speech.feed() if self.phase in ("starting", "running") else []
        return out or self._maybe_end(now)

    def _stall(self, now: float) -> List[Effect]:
        """Сторож (#4151): речи нет ``PERFORM_SILENCE_MAX_S`` — снять речь и подложку сразу, сказать фразу кода.
        В логе — какую реплику ждали и в каком она состоянии (пред-синтез не ответил, звучание не встало)."""
        wait = self.lines.first("committed") or self.lines.first("pending")
        what = "—" if wait is None else (f"реплика {wait.no} ({wait.state}, "
                                         f"{'синтез готов' if wait.ready else 'synthesized не пришёл'})")
        _LOG.warning(f"{LOG_MARK} {self.block_id[:8]}: речи нет {self.silent_s(now):.0f} с, жду: {what} — снимаю")
        self.log(f"сторож: речи нет {self.silent_s(now):.0f} с, жду: {what} — выступление снято")
        return [*self.halt(now, reason="stalled"), self.speech.phrase("perform.stalled")]

    def _maybe_end(self, now: float) -> List[Effect]:
        """Всё прозвучало (или снято): outro подложки на такте и ``block_end``."""
        if self.phase not in ("running", "stopping") or self.lines.live():
            return []
        out = self.music.release(now, "bar", "perform_end", _SOUNDING)
        return out + self._end("stopped" if self.phase == "stopping" else "done")

    def _end(self, reason: str) -> List[Effect]:
        self.phase, self.end_reason = "ended", reason
        done = self.summary()
        self.log(f"конец ({reason}) сегментов {done['segments_done']}, символов {done['spoken_chars']}, "
                 f"synced={done['synced']}, подложка {done['backing']}")
        return [self._event("block_end", done["segments_done"], synced=done["synced"], reason=reason,
                            spoken_chars=done["spoken_chars"])]

    def _event(self, kind: str, no: int = 0, **fields: Any) -> PlanEvent:
        return PlanEvent(kind, self.block_id, segment_no=no, **self._ids, **fields)


__all__ = ["AskReplace", "Cancel", "Effect", "LOG_MARK", "MusicSide", "PerformBlock", "Say", "SpeechSide",
           "context_block", "edit_asks_backing", "edit_range", "presynth", "withdraw"]
