"""Журнал ходов сессии и окно истории для LLM из того, что прозвучало (ADR-0150 §2.1 п.8, §7.3, §7.4; PR-8 #3702).

В историю модели попадает то, что человек услышал, структурные результаты действий и метки кода в квадратных скобках
вместо реплики робота, когда ход ответил без слов (действием, лицом, молчанием тишины):

* ``user`` — текст хода как его взял код (без вейка и без маркеров ``[Spkr:]``/``[TG]``/``[URGENT_BACKLOG]``: кто
  говорил и откуда — в снимке сессии, не в тексте); ход подхвата бэклога (В12″) пишет вместо одной реплики все
  подхваченные фразы, каждую с её автором на момент самой фразы (#3892);
* ``assistant`` — ``text[:spoken_chars]`` по событию владельца речи (``SpeechEvent`` ``finished/cancelled``): не
  прозвучало — нет и в истории; у текстового канала (телеграм) — весь текст, он доставлен. Озвученное помечено
  (``WindowEntry.voiced``): его, и только его, повторяет «повтори» (:meth:`TurnLog.last_spoken`, #4028). Ход отработан
  действиями, а ответа словами в окне нет — реплику робота строит код из исходов этих действий
  (``knowledge.WINDOW_ACTIONS_REPLY``: «[ответ действием: play_animation sad — сделано]»; исход по данным —
  «stop_music — сделано, ничего не играло», #4069 п.4), иначе реплика человека висит в истории без ответа и модель
  повторяет прошлое действие (#4066); ход отвечен одним лицом (``command.expression`` без действий и слов) -
  ``knowledge.WINDOW_FACE_REPLY``: «[ответ лицом: sad — показано]» (:meth:`TurnLog.faced`, #4073);
* ``tool`` — :class:`~rob_box_dialog.respond.ActionResult` структурой (тул, статус, причина, данные); поздний итог
  действия (:meth:`TurnLog.late`) заменяет его запись на месте, а фраза кода об опровергнутом ``timeout`` из окна
  уходит (#3960: «Музыка пока не заиграла.», а трек заиграл через 1,4 с);
* ``tool`` ``perform`` — исход выступления (:meth:`TurnLog.performed`): сегменты его владельца звучат мимо реплик хода,
  и без этой записи просьба «расскажи сказку» висит в окне неотвеченной — модель исполняет её заново на чужую реплику
  (#3973). Статус пишет код из события владельца (:func:`perform_status`), одна запись на выступление.

Ход тишины (ход, проглоченный кадром ``Silence``, и Tier-1 включения/снятия тишины) ничего не говорит и не вызывает:
реплику робота после реплики человека ставит метка кода (:meth:`TurnLog.mark`, ``knowledge.WINDOW_MUTED_MARK``:
«[рот заклеен — молчу]»; #4106, решение Шифу 27.10), иначе реплика висит без ответа, и после «говори» модель отвечает на
старое.

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
from rob_box_dialog.respond import ActionResult, phrase_key

#: Строка лога агрегата хода (читает ``scripts/dialog`` для метрик §14 без текста турнов).
AGGREGATE_MARK = "📊 [turn]"
USER, ASSISTANT, TOOL = "user", "assistant", "tool"
#: Тул-запись исхода выступления в окне (``recent_actions``: ``{"tool": "perform", "status": ..., "data": {"kind"}}``).
PERFORM_TOOL = "perform"
#: Класс исхода выступления для наблюдателя исходов (``knowledge.ACTION_CLASSES``).
PERFORM_CLASS = "action.perform"
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
    #: у ``assistant``: реплика прозвучала (или доставлена текстом) - не метка кода; в сравнении записей не участвует
    voiced: bool = field(default=False, compare=False)

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
    faced: str = ""  # лицо, которым ход ответил без действий и слов (:meth:`TurnLog.faced`, #4073)


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
    return WindowEntry(ASSISTANT, rest, voiced=entry.voiced) if rest else None


def _supersede(turn: "_Turn", said: Optional[str]) -> None:
    """Фраза кода, которую опроверг поздний итог (``TurnLog.late``, #3960), уходит из реплик хода: из окна модели и из
    того, что повторит «повтори» (#4028). Фразы нет - ничего не меняется."""
    if said:
        turn.superseded += (said,)
        turn.entries = [e for e in (_unsaid(e, turn.superseded) for e in turn.entries) if e is not None]


def _last_voiced(turns: Sequence["_Turn"], exclude: str) -> str:
    """Озвученные реплики робота последнего хода, где он говорил (кроме ``exclude``), по порядку - одной строкой."""
    for turn in reversed(turns):
        said = " ".join(e.content for e in turn.entries if e.voiced)
        if said and turn.turn_id != exclude:
            return said
    return ""


def _action_head(tool: str, data: Mapping[str, Any]) -> str:
    """Тул, ключевой аргумент (``knowledge.WINDOW_ACTION_ARG``) и что именно заказано
    (``knowledge.WINDOW_ACTION_DETAILS``, #4069 п.4): ``request_music track (text: ровный ритм; mood: спокойная)``."""
    field_name = kn.WINDOW_ACTION_ARG.get(tool)
    arg = data.get(field_name) if field_name else None
    head = f"{tool} {arg}" if arg not in (None, "") else tool
    named = kn.WINDOW_ACTION_DETAILS.get(tool, ())
    details = [f"{name}: {data[name]}" for name in named if data.get(name) not in (None, "")]
    return f"{head} ({'; '.join(details)})" if details else head


def _outcome_note(tool: str, status: str, reason: Any, data: Mapping[str, Any]) -> str:
    """Исход по ДАННЫМ результата словами окна (``knowledge.WINDOW_OUTCOME_NOTES``, #4069 п.4): по ключу фразы исхода
    (``respond.phrase_key`` - одна таблица ``TOOL_PHRASES`` по данным и для речи, и для окна). Тула нет в каталоге
    (запись выступления ``perform``) - слов нет."""
    if tool not in kn.TOOL_CLASSES:
        return ""
    result = ActionResult("", 0, "", tool, kn.call_class(tool, data), status, reason, data)
    return kn.WINDOW_OUTCOME_NOTES.get(phrase_key(result) or "", "")


def _action_label(record: Mapping[str, Any]) -> str:
    """Одно действие хода словами кода: ``play_animation sad — сделано``, ``play_sound robot_alert — отказ (busy)``,
    ``stop_music — сделано, ничего не играло`` (исход по данным, #4069 п.4)."""
    tool, status, reason = str(record.get("tool", "")), str(record.get("status", "")), record.get("reason")
    data = record.get("data") or {}
    note = _outcome_note(tool, status, reason, data)
    said = kn.WINDOW_ACTION_STATUS.get(status, status) + (f", {note}" if note else "")
    return f"{_action_head(tool, data)} — {said}" + (f" ({reason})" if reason else "")


def _action_reply(lead: Sequence[WindowEntry], entries: Sequence[WindowEntry], face: str = "") -> List[WindowEntry]:
    """Ход отработан действиями, а ответа словами в окне нет: реплика робота из исходов этих действий (#4066).
    Без неё реплика человека висит в истории без ответа, и модель «догоняет» её - повторяет прошлое действие. Строится
    при каждой выдаче окна, поэтому поздний итог (:meth:`TurnLog.late`) и конец выступления видны в ней сразу. Действий
    нет, но ход ответил лицом (``face``, :meth:`TurnLog.faced`) - реплика о показанном лице (#4073: ответ одним лицом
    тоже ответ, иначе модель «догоняет» его - победа → грусть). Ход без реплики человека, без действий и лица или с
    прозвучавшим ответом - без неё."""
    if not lead or any(e.role == ASSISTANT for e in entries):
        return []
    labels = [_action_label(e.content) for e in entries if e.role == TOOL]
    if labels:
        return [WindowEntry(ASSISTANT, kn.WINDOW_ACTIONS_REPLY.format(actions="; ".join(labels)))]
    return [WindowEntry(ASSISTANT, kn.WINDOW_FACE_REPLY.format(face=face))] if face else []


def _tool_content(result: Any) -> Mapping[str, Any]:
    data = dict(getattr(result, "data", None) or {})
    out: Dict[str, Any] = {"tool": result.tool, "status": result.status}
    if getattr(result, "reason", None):
        out["reason"] = result.reason
    if data:
        out["data"] = data
    return MappingProxyType(out)


#: Наблюдатель исходов (``TurnLog(on_outcome=...)``): исход как его отдал исполнитель (``ActionResult``, ``turn_id`` —
#: id вызова ``<ход>/<n>``), выступление — ``ActionResult`` тула ``perform`` класса ``PERFORM_CLASS`` с id
#: ``<ход>/perform`` и видом в ``data["kind"]``. Наблюдатель судит исход сам (``respond.failed``/``asks``), поэтому
#: получает результат целиком, а не запись окна. Его ведёт журнал смены (``shift_log``, #4076).
OnOutcome = Callable[[ActionResult], None]


class TurnLog:
    """Ходы одной сессии агента: что сказал человек, что прозвучало, чем кончились действия.

    ``on_outcome`` получает каждый исход, что журнал принял (результат, выступление; поздний итог — даже когда ход уже
    выпал из окна): журнал смены живёт дольше сессии и окна (#4076)."""

    def __init__(self, max_turns: int = kn.TURN_WINDOW_MAX_TURNS, max_tokens: int = kn.TURN_WINDOW_MAX_TOKENS,
                 estimate: Callable[[str], int] = estimate_tokens, on_outcome: Optional[OnOutcome] = None) -> None:
        self._max_turns = max_turns
        self._max_tokens = max_tokens
        self._estimate = estimate
        self._on_outcome = on_outcome
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
            self._add(turn_id, WindowEntry(ASSISTANT, text, voiced=True))
        else:
            self._speech[speech_id] = (turn_id, text)

    def spoken(self, speech_id: str, spoken_chars: int) -> Optional[str]:
        """Реплика отзвучала (``finished``/``cancelled``/``rejected``): в окно — озвученная часть. Чужая или
        старой сессии реплика — ``None``, ничего не меняется."""
        turn_id, text = self._speech.pop(speech_id, (None, ""))
        part = text[:max(0, int(spoken_chars or 0))].strip()
        if turn_id is None or not part:
            return None
        self._add(turn_id, WindowEntry(ASSISTANT, part, voiced=True))
        return part

    def speech(self, speech_id: str) -> Optional[Tuple[str, str]]:
        """``(turn_id, текст)`` реплики, которая ещё не отзвучала; чужая или уже отзвучавшая — ``None``."""
        return self._speech.get(speech_id)

    def last_spoken(self, exclude: str = "") -> str:
        """Последняя ОЗВУЧЕННАЯ реплика робота в журнале сессии (#4028, Tier-1 «повтори»): то, что прозвучало
        (:meth:`spoken`; у текстового канала - доставленный текст) в последнем ходе, где робот говорил, по порядку.
        Метки кода в окне («[ответ действием: …]», «[рот заклеен — молчу]») - не слова робота, их не повторяем; фразу,
        которую опроверг поздний итог (:meth:`late`), - тоже. ``exclude`` - ход, который просит повтор. Робот ещё
        ничего не сказал (новая сессия) - «»."""
        return _last_voiced(list(self._turns.values()), exclude)

    def mark(self, turn_id: str, mark: str) -> bool:
        """Ход закрыт без слов робота по причине кода: ``mark`` (``knowledge.WINDOW_MUTED_MARK``,
        ``WINDOW_SILENCE_MARKS``) - реплика робота в окне после реплики человека (#4106: ход тишины - «рот заклеен»).
        Решает код по событию хода (``hold_silence``, Tier-1 тишины), не слова модели. Пустая метка или хода нет в
        журнале (выпал из окна, новая сессия) - ``False``, не пишется."""
        return bool(mark) and self._add(turn_id, WindowEntry(ASSISTANT, mark)) is not None

    def result(self, turn_id: str, result: Any) -> None:
        """Результат действия хода (``ActionResult``) — структурой. Отказ памяти из ``WINDOW_DROPPED_REJECTS`` в окно
        не идёт: модель не должна помнить отклонённый факт, чтобы не перенести его в ход другого человека (#3881)."""
        if result.status == "rejected" and result.reason in kn.WINDOW_DROPPED_REJECTS.get(result.tool, ()):
            return
        self._outcome(result)
        entry = self._add(turn_id, WindowEntry(TOOL, _tool_content(result)))
        if entry is not None:
            self._actions[result.turn_id] = (turn_id, entry, result)

    def late(self, result: Any, phrase_of: Callable[[Any], Optional[str]]) -> bool:
        """Поздний итог действия (``Executor.on_late``: ``done``/``rejected`` после ``timeout`` или ``pending``):
        запись его исхода в окне заменяется итогом владельца на месте. Фраза кода о прежнем ``timeout`` («Музыка пока
        не заиграла.», ``phrase_of`` — как её строит ход) из окна уходит — и прозвучавшая, и та, что ещё звучит: модель
        на следующем ходу видит правду, а не таймаут (n809, #3960). Фраза позднего итога входит в окно обычным путём
        (:meth:`said` → :meth:`spoken`). Действия нет в журнале (ход выпал из окна, новая сессия) — ``False``."""
        self._outcome(result)
        known = self._actions.get(result.turn_id)
        turn = self._turns.get(known[0]) if known is not None else None
        if turn is None:
            return False
        _, entry, before = known
        fresh = WindowEntry(TOOL, _tool_content(result))
        turn.entries = [fresh if e is entry else e for e in turn.entries]
        self._actions[result.turn_id] = (turn.turn_id, fresh, result)
        _supersede(turn, phrase_of(before) if before.status == "timeout" else None)
        return True

    def performed(self, turn_id: str, kind: str, status: str, segments_done: Optional[int] = None) -> bool:
        """Исход выступления хода (``kind`` - сказка/рэп/..., ``status`` - ``PERFORM_*``): запись ``tool`` ``perform``
        закрывает просьбу, на которую выступление отвечает (#3973). Одна запись на выступление: старт кладёт её,
        конец/стоп обновляет на месте. Хода нет в журнале (выпал из окна, новая сессия) - ``False``, в окно не пишется,
        но наблюдатель исходов исход получает: журнал смены живёт дольше окна, и его запись «идёт» иначе висела бы
        вечно (ревью PR #4110)."""
        turn = self._turns.get(turn_id)
        data: Dict[str, Any] = {"kind": kind or self._performs.get(turn_id, "")}
        if segments_done is not None:
            data["segments_done"] = int(segments_done)
        self._outcome(ActionResult(f"{turn_id}/{PERFORM_TOOL}", turn.epoch if turn is not None else 0, "",
                                   PERFORM_TOOL, PERFORM_CLASS, status, data=MappingProxyType(dict(data))))
        if turn is None:
            return False
        entry = WindowEntry(TOOL, MappingProxyType({"tool": PERFORM_TOOL, "status": status, "data": data}))
        slot = next((i for i, e in enumerate(turn.entries) if _is_perform(e)), None)
        if slot is None:
            turn.entries.append(entry)
        else:
            turn.entries[slot] = entry
        self._performs[turn_id] = data["kind"]
        return True

    def performance_ended(self, turn_id: str, end_reason: str, segments_done: int = 0) -> bool:
        """Выступление кончилось (``block_end`` владельца: ``done``/``stopped``/``halt``/``lost``): статус строит
        код (:func:`perform_status`), а не слова модели (#3973). ``turn_id`` - ход, чей старт открыл блок (его id несёт
        ``block_end``), но выступление одно за раз, и конец закрывает все записи ``PERFORM_STARTED``: правка идущего
        («добавь куплет», «добавь бит») тоже записана «идёт» своим ходом (``note_perform_start``), а владелец её конец
        под её ход не присылает. Без этого запись правки висела бы «идёт» до выпадения хода из окна, и
        :meth:`performing` молчал бы об успехе музыки уже после выступления (#4075). Закрыта хоть одна запись -
        ``True``."""
        status = perform_status(end_reason)
        closed = self.performed(turn_id, "", status, segments_done)
        for other in self._started_performs():
            closed = self.performed(other, "", status, segments_done) or closed
        return closed

    def faced(self, turn_id: str, face: str) -> bool:
        """Ход ответил лицом: ``face`` - показанное словами кода (``expression.face_label``), как его показал итог хода
        (``turn.done_expression``). Запись в окне - реплика робота после реплики человека, если у хода нет ни действий,
        ни прозвучавшей речи (:func:`_action_reply`, #4073). Хода нет в журнале (выпал из окна, новая сессия) или
        показывать нечего (``face`` пусто) - ``False``, не пишется."""
        turn = self._turns.get(turn_id)
        if turn is None or not face:
            return False
        turn.faced = face
        return True

    def performing(self) -> bool:
        """Выступление идёт: его запись в окне стоит на ``PERFORM_STARTED`` - старт владельцем взят
        (``note_perform_start``), конца (:meth:`performance_ended`: ``block_end`` владельца) ещё не было. Решает код по
        событиям владельца: по этому признаку исход «музыка пошла» посреди выступления не озвучивается (#4075)."""
        return bool(self._started_performs())

    def _started_performs(self) -> List[str]:
        """Ходы, чья запись выступления в окне стоит на ``PERFORM_STARTED`` (старт владелец взял, конца не было)."""
        return [turn.turn_id for turn in self._turns.values()
                if any(_is_perform(e) and e.content.get("status") == PERFORM_STARTED for e in turn.entries)]

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

    def _outcome(self, result: ActionResult) -> None:
        if self._on_outcome is not None:
            self._on_outcome(result)

    def _entries(self, turn: _Turn) -> List[WindowEntry]:
        lead = list(turn.heard) or ([WindowEntry(USER, turn.user, turn.speaker)] if turn.user else [])
        return lead + list(turn.entries) + _action_reply(lead, turn.entries, turn.faced)

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


__all__ = ["AGGREGATE_MARK", "ASSISTANT", "OnOutcome", "PERFORM_CLASS", "PERFORM_DONE", "PERFORM_INTERRUPTED",
           "PERFORM_STARTED", "PERFORM_STOPPED", "PERFORM_TOOL", "TOOL", "TurnAggregate", "TurnLog", "USER",
           "WindowEntry", "perform_status", "phrase_label", "speaker_label"]
