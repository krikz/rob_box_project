"""Журнал ходов сессии и окно истории для LLM из того, что прозвучало (ADR-0150 §2.1 п.8, §7.3, §7.4; PR-8 #3702).

В историю модели попадает только то, что человек услышал, и структурные результаты действий:

* ``user`` — текст хода как его взял код (без вейка и без маркеров ``[Spkr:]``/``[TG]``/``[URGENT_BACKLOG]``: кто
  говорил и откуда — в снимке сессии, не в тексте); ход подхвата бэклога (В12″) пишет вместо одной реплики все
  подхваченные фразы, каждую с её автором на момент самой фразы (#3892);
* ``assistant`` — ``text[:spoken_chars]`` по событию владельца речи (``SpeechEvent`` ``finished/cancelled``): не
  прозвучало — нет и в истории; у текстового канала (телеграм) — весь текст, он доставлен. Ход отработан действиями,
  а ответа словами в окне нет — реплику робота строит код из исходов этих действий
  (``knowledge.WINDOW_ACTIONS_REPLY``: «[ответ действием: play_animation sad — сделано]»), иначе реплика человека висит
  в истории без ответа и модель повторяет прошлое действие (#4066);
* ``tool`` — :class:`~rob_box_dialog.respond.ActionResult` структурой (тул, статус, причина, данные); поздний итог
  действия (:meth:`TurnLog.late`) заменяет его запись на месте, а фраза кода об опровергнутом ``timeout`` из окна
  уходит (#3960: «Музыка пока не заиграла.», а трек заиграл через 1,4 с);
* ``tool`` ``perform`` — исход выступления (:meth:`TurnLog.performed`): сегменты его владельца звучат мимо реплик хода,
  и без этой записи просьба «расскажи сказку» висит в окне неотвеченной — модель исполняет её заново на чужую реплику
  (#3973). Статус пишет код из события владельца (:func:`perform_status`), одна запись на выступление.

Отвергнутые команды и невалидный ответ модели в окно не попадают (#3145), как и отказ памяти
``no_person``/``speaker_changed`` (#3881) — только в агрегат хода. Окно — не больше
``knowledge.TURN_WINDOW_MAX_TURNS`` ходов и ``TURN_WINDOW_MAX_TOKENS`` токенов, старые ходы выпадают целиком;
:meth:`TurnLog.reset` (новая сессия) очищает всё, и поздние события речи старой эпохи ничего не добавляют.

Журнал живёт в памяти процесса (директива Шифу 02.09, решение В8): на диск в проде уходят только агрегаты без
текста — :class:`TurnAggregate` (строка лога :data:`AGGREGATE_MARK`); текст хода пишет только e2e-БД (PR-8 ч.8c).
"""

from __future__ import annotations

import json
from collections import OrderedDict
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from rob_box_core.token_estimate import estimate_tokens
from rob_box_dialog import knowledge as kn

#: Строка лога агрегата хода (читает ``scripts/dialog`` для метрик §14 без текста турнов).
AGGREGATE_MARK = "📊 [turn]"
USER, ASSISTANT, TOOL = "user", "assistant", "tool"
#: Тул-запись исхода выступления в окне (``recent_actions``: ``{"tool": "perform", "status": ..., "data": {"kind"}}``).
PERFORM_TOOL = "perform"
#: Статус выступления в окне: идёт / досказано / остановил человек («хватит», «стой») / оборвано без его слова.
PERFORM_STARTED, PERFORM_DONE = "started", "done"
PERFORM_STOPPED, PERFORM_INTERRUPTED = "stopped_by_user", "interrupted"
#: ``block_end.reason`` выступления (``plan.perform.PerformBlock``) → статус окна. Причина вне таблицы — «оборвано».
PERFORM_END_STATUS: Mapping[str, str] = MappingProxyType(
    {"done": PERFORM_DONE, "stopped": PERFORM_STOPPED, "halt": PERFORM_STOPPED})


def perform_status(end_reason: str) -> str:
    """Статус выступления в окне по причине конца блока: досказано - ``done``, «хватит»/«стой» - ``stopped_by_user``,
    остальное (потеряна подложка, сбой) - ``interrupted``. Фраза об исходе строится кодом, не словами модели."""
    return PERFORM_END_STATUS.get(end_reason, PERFORM_INTERRUPTED)


@dataclass(frozen=True)
class WindowEntry:
    """Запись окна LLM: ``content`` — строка у ``user``/``assistant``, данные результата у ``tool``."""

    role: str
    content: Any
    speaker: str = ""  # у ``user``: кто говорил в этот ход (метка кода из снимка, #3881)

    def text(self) -> str:
        """Как запись уходит модели (и как считается её размер): данные ``tool`` — компактным JSON."""
        if isinstance(self.content, str):
            return self.content
        return json.dumps(self.content, ensure_ascii=False, separators=(",", ":"), default=_plain)

    def for_llm(self) -> Any:
        """Как запись уходит модели в истории: реплика человека - с тегом авторства (#3881)."""
        if self.role == USER and self.speaker:
            return kn.SPEAKER_TAG.format(who=self.speaker, text=self.content)
        return self.content


def speaker_label(speaker: Any) -> str:
    """Кто говорит в ход, словами кода: имя/кличка из снимка, нет - «незнакомый голос»."""
    return (getattr(speaker, "name", None) or kn.UNKNOWN_SPEAKER_LABEL).strip()


def phrase_label(speaker: Mapping[str, Any]) -> str:
    """Кто сказал фразу бэклога (метка ``speaker_adapter.phrase_speaker`` на момент самой фразы, #3892) словами
    той же схемы, что :func:`speaker_label`: узнан — имя, иначе «незнакомый голос» (гипотеза голоса — не имя, A30)."""
    name = speaker.get("name") if speaker.get("identified") else None
    return (name or kn.UNKNOWN_SPEAKER_LABEL).strip()


def _plain(value: Any) -> Any:
    """JSON для неизменяемых данных результата: ``MappingProxyType`` — словарём, остальное — строкой."""
    return dict(value) if isinstance(value, Mapping) else str(value)


@dataclass(frozen=True)
class TurnAggregate:
    """Агрегат хода без текста — единственное, что о ходе пишется на диск в проде (§7.4)."""

    turn_id: str
    epoch: int
    agent: str
    stage: str
    status: str
    latency_ms: int = 0
    llm_calls: int = 0
    llm_provider: str = ""
    tokens: int = 0

    def log_line(self) -> str:
        return (f"{AGGREGATE_MARK} turn_id={self.turn_id} epoch={self.epoch} agent={self.agent} stage={self.stage} "
                f"status={self.status} latency_ms={self.latency_ms} llm_calls={self.llm_calls} "
                f"llm_provider={self.llm_provider or '-'} tokens={self.tokens}")


@dataclass
class _Turn:
    turn_id: str
    epoch: int
    user: str
    speaker: str = ""
    entries: List[WindowEntry] = field(default_factory=list)
    heard: Tuple[WindowEntry, ...] = ()  # подхват бэклога: фразы с их авторами вместо реплики хода (#3892)
    superseded: Tuple[str, ...] = ()  # фразы кода об исходе, которые опроверг поздний итог (:meth:`TurnLog.late`)


def _is_perform(entry: WindowEntry) -> bool:
    return entry.role == TOOL and isinstance(entry.content, Mapping) and entry.content.get("tool") == PERFORM_TOOL


def _unsaid(entry: WindowEntry, superseded: Sequence[str]) -> Optional[WindowEntry]:
    """Реплика робота без фраз кода, которые опроверг поздний итог (#3960); от неё ничего не осталось - ``None``.
    Чужие записи и реплики без этих фраз - как есть."""
    if entry.role != ASSISTANT or not any(said in entry.content for said in superseded):
        return entry
    rest = entry.content
    for said in superseded:
        rest = rest.replace(said, " ")
    rest = " ".join(rest.split())
    return WindowEntry(ASSISTANT, rest) if rest else None


def _action_label(record: Mapping[str, Any]) -> str:
    """Одно действие хода словами кода: ``play_animation sad — сделано``, ``play_sound robot_alert — отказ (busy)``."""
    tool, status, reason = str(record.get("tool", "")), str(record.get("status", "")), record.get("reason")
    field_name = kn.WINDOW_ACTION_ARG.get(tool)
    arg = (record.get("data") or {}).get(field_name) if field_name else None
    head = f"{tool} {arg}" if arg not in (None, "") else tool
    said = kn.WINDOW_ACTION_STATUS.get(status, status)
    return f"{head} — {said} ({reason})" if reason else f"{head} — {said}"


def _action_reply(lead: Sequence[WindowEntry], entries: Sequence[WindowEntry]) -> List[WindowEntry]:
    """Ход отработан действиями, а ответа словами в окне нет: реплика робота из исходов этих действий (#4066).
    Без неё реплика человека висит в истории без ответа, и модель «догоняет» её - повторяет прошлое действие. Строится
    при каждой выдаче окна, поэтому поздний итог (:meth:`TurnLog.late`) и конец выступления видны в ней сразу. Ход
    без реплики человека, без действий или с прозвучавшим ответом - без неё."""
    if not lead or any(e.role == ASSISTANT for e in entries):
        return []
    labels = [_action_label(e.content) for e in entries if e.role == TOOL]
    return [WindowEntry(ASSISTANT, kn.WINDOW_ACTIONS_REPLY.format(actions="; ".join(labels)))] if labels else []


def _tool_content(result: Any) -> Mapping[str, Any]:
    data = dict(getattr(result, "data", None) or {})
    out: Dict[str, Any] = {"tool": result.tool, "status": result.status}
    if getattr(result, "reason", None):
        out["reason"] = result.reason
    if data:
        out["data"] = data
    return MappingProxyType(out)


class TurnLog:
    """Ходы одной сессии агента: что сказал человек, что прозвучало, чем кончились действия."""

    def __init__(self, max_turns: int = kn.TURN_WINDOW_MAX_TURNS, max_tokens: int = kn.TURN_WINDOW_MAX_TOKENS,
                 estimate: Callable[[str], int] = estimate_tokens) -> None:
        self._max_turns = max_turns
        self._max_tokens = max_tokens
        self._estimate = estimate
        self._turns: "OrderedDict[str, _Turn]" = OrderedDict()
        self._speech: Dict[str, Tuple[str, str]] = {}  # speech_id → (turn_id, текст)
        self._performs: Dict[str, str] = {}  # turn_id → kind выступления, запись которого в окне ещё можно обновить
        #: ``ActionResult.turn_id`` действия (``<ход>/<n>``) → (ход, его запись в окне, исход) — для :meth:`late`.
        self._actions: Dict[str, Tuple[str, WindowEntry, Any]] = {}

    def begin(self, turn_id: str, epoch: int, text: str, speaker: str = "",
              heard: Sequence[Tuple[str, str]] = ()) -> None:
        """Ход начался: ``text`` — реплика человека, как её взял код (пустая — ход без слов, в окно не идёт).
        ``heard`` — подхват бэклога ``(кто, фраза)``: в окно идут эти фразы со своими авторами, а не ``text``."""
        lead = tuple(WindowEntry(USER, t.strip(), who) for who, t in heard if t and t.strip())
        self._turns[turn_id] = _Turn(turn_id, int(epoch), (text or "").strip(), speaker, heard=lead)
        self._trim()

    def said(self, turn_id: str, speech_id: str, text: str, delivered: bool = False) -> None:
        """Агент отдал реплику владельцу речи. В окно она попадёт по :meth:`spoken`; ``delivered`` — текстовый
        канал (телеграм): текст доставлен целиком сразу."""
        if turn_id not in self._turns or not text:
            return
        if delivered:
            self._add(turn_id, WindowEntry(ASSISTANT, text))
        else:
            self._speech[speech_id] = (turn_id, text)

    def spoken(self, speech_id: str, spoken_chars: int) -> Optional[str]:
        """Реплика отзвучала (``finished``/``cancelled``/``rejected``): в окно — озвученная часть. Чужая или
        старой сессии реплика — ``None``, ничего не меняется."""
        turn_id, text = self._speech.pop(speech_id, (None, ""))
        part = text[:max(0, int(spoken_chars or 0))].strip()
        if turn_id is None or not part:
            return None
        self._add(turn_id, WindowEntry(ASSISTANT, part))
        return part

    def speech(self, speech_id: str) -> Optional[Tuple[str, str]]:
        """``(turn_id, текст)`` реплики, которая ещё не отзвучала; чужая или уже отзвучавшая — ``None``."""
        return self._speech.get(speech_id)

    def result(self, turn_id: str, result: Any) -> None:
        """Результат действия хода (``ActionResult``) — структурой. Отказ памяти из ``WINDOW_DROPPED_REJECTS`` в окно
        не идёт: модель не должна помнить отклонённый факт, чтобы не перенести его в ход другого человека (#3881)."""
        if result.status == "rejected" and result.reason in kn.WINDOW_DROPPED_REJECTS.get(result.tool, ()):
            return
        entry = self._add(turn_id, WindowEntry(TOOL, _tool_content(result)))
        if entry is not None:
            self._actions[result.turn_id] = (turn_id, entry, result)

    def late(self, result: Any, phrase_of: Callable[[Any], Optional[str]]) -> bool:
        """Поздний итог действия (``Executor.on_late``: ``done``/``rejected`` после ``timeout`` или ``pending``):
        запись его исхода в окне заменяется итогом владельца на месте. Фраза кода о прежнем ``timeout`` («Музыка пока
        не заиграла.», ``phrase_of`` — как её строит ход) из окна уходит — и прозвучавшая, и та, что ещё звучит: модель
        на следующем ходу видит правду, а не таймаут (n809, #3960). Фраза позднего итога входит в окно обычным путём
        (:meth:`said` → :meth:`spoken`). Действия нет в журнале (ход выпал из окна, новая сессия) — ``False``."""
        known = self._actions.get(result.turn_id)
        turn = self._turns.get(known[0]) if known is not None else None
        if turn is None:
            return False
        _, entry, before = known
        fresh = WindowEntry(TOOL, _tool_content(result))
        turn.entries = [fresh if e is entry else e for e in turn.entries]
        self._actions[result.turn_id] = (turn.turn_id, fresh, result)
        said = phrase_of(before) if before.status == "timeout" else None
        if said:
            turn.superseded += (said,)
            turn.entries = [e for e in (_unsaid(e, turn.superseded) for e in turn.entries) if e is not None]
        return True

    def performed(self, turn_id: str, kind: str, status: str, segments_done: Optional[int] = None) -> bool:
        """Исход выступления хода (``kind`` - сказка/рэп/..., ``status`` - ``PERFORM_*``): запись ``tool`` ``perform``
        закрывает просьбу, на которую выступление отвечает (#3973). Одна запись на выступление: старт кладёт её,
        конец/стоп обновляет на месте. Хода нет в журнале (выпал из окна, новая сессия) - ``False``, не пишется."""
        turn = self._turns.get(turn_id)
        if turn is None:
            return False
        data: Dict[str, Any] = {"kind": kind or self._performs.get(turn_id, "")}
        if segments_done is not None:
            data["segments_done"] = int(segments_done)
        entry = WindowEntry(TOOL, MappingProxyType({"tool": PERFORM_TOOL, "status": status, "data": data}))
        slot = next((i for i, e in enumerate(turn.entries) if _is_perform(e)), None)
        if slot is None:
            turn.entries.append(entry)
        else:
            turn.entries[slot] = entry
        self._performs[turn_id] = data["kind"]
        return True

    def performance_ended(self, turn_id: str, end_reason: str, segments_done: int = 0) -> bool:
        """Выступление хода кончилось (``block_end`` владельца: ``done``/``stopped``/``halt``/``lost``): статус строит
        код (:func:`perform_status`), а не слова модели (#3973)."""
        return self.performed(turn_id, "", perform_status(end_reason), segments_done)

    def performing(self) -> bool:
        """Выступление идёт: его запись в окне стоит на ``PERFORM_STARTED`` - старт владельцем взят
        (``note_perform_start``), конца (:meth:`performance_ended`: ``block_end`` владельца) ещё не было. Решает код по
        событиям владельца: по этому признаку исход «музыка пошла» посреди выступления не озвучивается (#4075)."""
        return any(_is_perform(e) and e.content.get("status") == PERFORM_STARTED
                   for turn in self._turns.values() for e in turn.entries)

    def window(self, exclude: str = "") -> Tuple[WindowEntry, ...]:
        """Окно для LLM: ходы по порядку, новые важнее; лишние по числу и токенам выпадают целиком с начала.
        ``exclude`` — ход, который сейчас спрашивает модель: он считается в размер, но в окно не выдаётся."""
        turns = [(t.turn_id, self._entries(t)) for t in self._turns.values()]
        turns = [t for t in turns if t[1]][-self._max_turns:]
        while len(turns) > 1 and self._tokens([t[1] for t in turns]) > self._max_tokens:
            turns.pop(0)
        return tuple(e for turn_id, t in turns if turn_id != exclude for e in t)

    def tokens(self) -> int:
        """Оценка размера окна в токенах (то, что уйдёт модели)."""
        return sum(self._estimate(e.text()) for e in self.window())

    def reset(self) -> None:
        """Новая сессия: окно и ожидающие реплики очищены."""
        self._turns.clear()
        self._speech.clear()
        self._performs.clear()
        self._actions.clear()

    def _entries(self, turn: _Turn) -> List[WindowEntry]:
        lead = list(turn.heard) or ([WindowEntry(USER, turn.user, turn.speaker)] if turn.user else [])
        return lead + list(turn.entries) + _action_reply(lead, turn.entries)

    def _tokens(self, turns: List[List[WindowEntry]]) -> int:
        return sum(self._estimate(e.text()) for t in turns for e in t)

    def _add(self, turn_id: str, entry: WindowEntry) -> Optional[WindowEntry]:
        """Запись в окно хода; реплика робота — без фраз, опровергнутых поздним итогом. Что легло — или ``None``."""
        turn = self._turns.get(turn_id)
        kept = _unsaid(entry, turn.superseded) if turn is not None else None
        if turn is not None and kept is not None:
            turn.entries.append(kept)
        return kept

    def _trim(self) -> None:
        """Память журнала ограничена окном: старше ``max_turns`` ходов не хранится (и их реплики — тоже)."""
        while len(self._turns) > self._max_turns:
            gone, _ = self._turns.popitem(last=False)
            self._speech = {sid: v for sid, v in self._speech.items() if v[0] != gone}
            self._actions = {ref: v for ref, v in self._actions.items() if v[0] != gone}
            self._performs.pop(gone, None)


__all__ = ["AGGREGATE_MARK", "ASSISTANT", "PERFORM_DONE", "PERFORM_INTERRUPTED", "PERFORM_STARTED", "PERFORM_STOPPED",
           "PERFORM_TOOL", "TOOL", "TurnAggregate", "TurnLog", "USER", "WindowEntry", "perform_status", "phrase_label",
           "speaker_label"]
