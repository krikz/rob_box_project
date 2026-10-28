#!/usr/bin/env python3
"""Разбор событий речи и хода робота для ожидания харнесса E2E (issue #3857, эпик #3683).

Харнесс не спит фиксированные ``E2E_REACTION_WINDOW`` секунд, а опрашивает ``docker logs`` робота и спрашивает
этот модуль: «ответ готов?», «робот ещё говорит?», «наступил якорь перебивания?». Решение принимает код по
строкам лога, а не оператор по таймеру.

Вход (stdin) -- вывод удалённой команды: первая строка ``<epoch робота>`` (``date +%s.%N``), дальше строки
``docker logs``, отфильтрованные ``grep`` по маркерам. Выход -- строки ``ключ=значение`` без пробелов.

Маркеры
=======
v2 (``dialog_engine: v2``):

* ``✅ ПРИНЯТО (...)`` -- фраза принята STT;
* ``📊 [turn] ... stage=done|failed|cancelled|silenced`` -- агрегат хода (``rob_box_dialog.turn_log.AGGREGATE_MARK``).
  Все -- КОНЕЦ хода (#3954): ``failed`` не заставляет харнесс ждать потолок, вердикт ``failed`` несёт причину
  (``status``/``reason``/``llm_calls`` из той же строки). ``cancelled`` -- конец хода, кроме шага-перебивателя
  (``--barge``): там «отменён» -- след ПРОШЛОЙ фразы, и концом хода этого шага он быть не может. ``silenced`` -- ход
  проглотила тишина (#3982: LLM не зовётся, речи нет);
* ``[backlog] accumulated reason=no_wake`` -- фраза принята без вейка и ушла в бэклог: хода не будет (#3989);
* ``🧭 [turn-v2] tier1 kind=<вид>`` -- ход Tier-1 (стоп, тишина, «говори», громкость...);
* ``🧭 [turn-v2] выступление старт kind=<вид> … {'status': 'pending', …}`` -- владелец взял выступление (сказку,
  стих, рэп); конец -- ``🎤 [perform] …: конец (<причина>)`` или ``🗓️ [plan] PlanEvent block_end block=perform-…``.
  Ход (``📊 [turn] stage=done``) кончается раньше выступления, а между куплетами нет открытых реплик: ответ готов
  только после конца выступления, потолок -- :data:`PERFORM_CEILING_S` от его старта (#3971 R4, n412);
* ``🗣️ SpeechEvent <kind> speech_id=<8> ...`` -- события владельца речи (``speech_owner._emit``): ``queued`` /
  ``synthesized`` / ``started`` / ``progress`` открывают или ведут реплику, ``finished`` / ``cancelled`` /
  ``rejected`` закрывают её. Пара -- по ``speech_id``. Каждый куплет выступления -- отдельный ``SpeechRequest``
  со своим ``speech_id``, значит и свой ``SpeechEvent started``.

v1: ``🔊 TTS: speech_id=<8>`` открывает реплику, ``Публикую TTS finished event: speech_id=<8>`` (и вариант
``(ошибка)``) закрывает.

В строке ``SpeechEvent`` нет ``turn_id`` и идентификатора выступления: «куплет именно этого выступления»
отличить от обычной реплики по логу нельзя. Окно лога ограничено фразой шага, поэтому ``segment_started``
считается как N-й ``started`` в окне (см. :func:`anchor`).

Восстановление шага (#3989)
===========================
Вердикты ``no_wake`` и ``silenced`` -- не «робот не ответил», а «до хода дело не дошло»: вейк потерян в STT либо
тишина, оставленная прошлым шагом/актом. Харнесс делает ОДНО восстановление (:data:`RECOVERY`) и помечает шаг
``OK recovered_<вид>``; какие виды разрешены шагу -- :func:`recover_kinds` (поле ``recover`` сценария). Тишину на
старте акта харнесс читает из снимка ``/dialog/session`` (:func:`session_report`). Маркер в логе акта один на все
восстановления -- ``E2E_RECOVER <вид> <шаг|at_act_start> …``; виды -- :data:`RECOVERY` (шаг) и :data:`ACT_RECOVERY`
(старт акта).

Вейк у голоса с акцентом (решение Шифу 28.10, n210)
===================================================
Голос Гриши STT слышит как «ребят»/«привет», и «Робот!» одним словом -- так же. Шаг такого голоса просит в ``recover``
ещё и ``wake_replay`` (:data:`WAKE_REPLAY`): вейк потерян, «Робот!» не помог -- фраза шага звучит ещё раз, один раз за
шаг; итог шага -- ``OK recovered_wake_replay`` (не чистый OK). Ретрай именно на промах вейка: неверный ответ робота
по-прежнему ретраит только ``retry_acceptance`` и только если шаг не менял состояние робота.

Предусловие шага (#3971 R21)
============================
Поле шага ``skip_when: {"reason": "KIE_QUOTA", "log": <regex>}`` -- без чего шаг теряет смысл: строка ``log`` в логе
робота за акт значит, что шаг не играется, вердикт ``SKIP <reason>`` (:func:`skip_rule`, :func:`skip_verdict`). С
``"absent": true`` наоборот: шаг не играется, если строки НЕТ -- предусловие так и не наступило (n805b «Робот, говори»
без тишины, которую должен был включить n805). С ``"deck": "ended"`` предусловие -- состояние деки: шаг не
играется, если последним событием деки за акт было ``finished`` (трек доиграл сам: «выключи» нечего выключать,
n312 28.10); снятая стопом или не заигравшая дека шаг не гасит -- красный там честный.

Ссылка шага на трек песни акта (#3710, #4077)
=============================================
Поле шага ``track_ref`` -- шаг про «эту песню» (n707 «расскажи», n708 «включи»), а не про любой трек библиотеки:
``{"reason": "CASCADE_N702", "log": <regex с одной группой -- id>, "tool": "music_library", "action": "info",
"wait_sec": N}``. Id берётся из лога робота за акт, где его пишет сам заказ песни (``🎤 [song] … track=<id>``,
:func:`track_ref_bind`; последняя такая строка); строки нет за ``wait_sec`` -- заказ n702 песни не дал, шаг не
играется: ``SKIP <reason>``, а не PASS. Шаг сыгран -- его вызов тула в окне шага (``📥 Запрос выполнения: <tool> с
параметрами {…}``) обязан нести ЭТОТ id при этом ``action`` (:func:`track_ref_verdict`): вызов на чужой трек или
выдуманный id шаг краснит. «Тул вызван» без id (ложный PASS n707/n708 27.10) шагу не засчитывается.

Вид результата музыки (#3971)
=============================
Поле шага ``music_compare`` (:data:`COMPARES`) -- результат, который виден только в сравнении с музыкой ДО шага:
«ускорь» -- тот же трек быстрее (правка на такте, #4029), «выключи» -- играла и выключена человеком. Судья читает
строки деки ``[music v2] started|edit applied|stop|finished`` за акт (:func:`music_compare`); паттерн шага (grep по
окну шага) этого не может: «что-то заиграло» и «быстрее того, что играло» в окне шага неотличимы.
"""

from __future__ import annotations

import ast
import json
import re
import sys
from datetime import datetime, timezone
from types import MappingProxyType
from typing import Any, Callable, Dict, List, Mapping, NamedTuple, Optional, Pattern, Tuple

TS_RE = re.compile(r"\[(\d{9,}\.\d+)\]")
SPEECH_V2 = re.compile(r"SpeechEvent (queued|synthesized|started|progress|finished|cancelled|rejected) speech_id=([0-9A-Za-z]+)")
V1_OPEN = re.compile(r"🔊 TTS: speech_id=([0-9a-f]{8})")
V1_CLOSE = re.compile(r"TTS finished event(?: \(ошибка\))?: speech_id=([0-9a-f]{8})")
OPENING = frozenset({"queued", "synthesized", "started"})
CLOSING = frozenset({"finished", "cancelled", "rejected"})
TURN_RE = re.compile(r"📊 \[turn\].*?\bstage=(\w+)")
TURN_FIELDS = ("status", "reason", "llm_calls")
#: Ход, проглоченный тишиной: ``rob_box_dialog.knowledge.SILENCED_TURN_STAGE`` (#3982; drift держит тест #3989).
SILENCED = "silenced"
TURN_END = frozenset({"done", "failed", "cancelled", SILENCED})
NO_WAKE_RE = re.compile(r"\[backlog\] accumulated reason=no_wake\b")
TIER1_RE = re.compile(r"\btier1 kind=(\w+)")
#: Tier-1 «замолчи» (ключ ``rob_box_dialog.knowledge.TIER1_KINDS``; drift держит тест #3989).
SILENCE_TIER1 = "silence"
#: Сколько секунд после конца фразы шага ждать, прежде чем «ушла в бэклог без вейка» окончательно: STT мог порезать
#: фразу, и следующий кусок ещё начнёт ход (#3989).
NO_WAKE_SETTLE_S = 4.0
#: Сцена тишины в снимке сессии: ``rob_box_core.voice_state.SILENCE_SCENE`` (drift держит тест #3989).
SILENCE_SCENE = "Silence"
#: Старт выступления, которое владелец ещё играет: ``turn.py`` -- ``выступление старт kind=<вид> … → {'status':
#: 'pending', 'block_id': 'perform-…'}``. ``'status': 'done'`` -- куплеты легли в уже идущее выступление (у него свой
#: старт и свой конец), отдельного конца у такой строки нет.
PERFORM_START_RE = re.compile(r"выступление старт kind=(\w+).*'status': 'pending'")
#: Конец выступления с любой причиной (done/stopped/failed): ``plan/perform.py`` -- ``🎤 [perform] …: конец (<причина>)``,
#: ``plan_events`` -- ``🗓️ [plan] PlanEvent block_end block=perform-… reason=<причина>`` (пишутся парой).
PERFORM_END_RE = re.compile(r"\[perform\] \S*: конец \((\w+)\)|PlanEvent block_end block=perform-\S* .*\breason=(\w+)")
#: Потолок ожидания выступления от его старта (часы робота): сказка марафона 23.10 шла 94-162 с (n412), а потолок
#: ответа (``E2E_REACTION_WINDOW``, 40 с на попытку) её обрывал -- ``cycle_unconfirmed_after_accept`` за 4 с до конца.
PERFORM_CEILING_S = 240.0
STALE_S = 60.0
MAX_SKEW_S = 3600.0
ANCHORS = ("speech_started", "llm_request", "segment_started")


def _ts(line: str) -> Optional[float]:
    found = TS_RE.search(line)
    return float(found.group(1)) if found else None


def split_input(text: str) -> Tuple[Optional[float], List[str]]:
    """``(now робота, строки лога)``; нет числа в первой строке -- ``now=None`` (чтение не удалось)."""
    lines = text.splitlines()
    if not lines:
        return None, []
    try:
        return float(lines[0].strip()), lines[1:]
    except ValueError:
        return None, lines


def speech_event(line: str, engine: str) -> Optional[Tuple[str, str]]:
    """``(вид, speech_id)`` события речи в строке или ``None``. Вид: queued/started/progress/finished/..."""
    if engine == "v2":
        found = SPEECH_V2.search(line)
        return (found.group(1), found.group(2)) if found else None
    opened = V1_OPEN.search(line)
    if opened:
        return "started", opened.group(1)
    closed = V1_CLOSE.search(line)
    return ("finished", closed.group(1)) if closed else None


class Window:
    """Состояние окна лога: принято ли, закончен ли ход, какие реплики открыты."""

    def __init__(self, engine: str, barge: bool = False) -> None:
        self.engine = engine
        self.barge = barge
        self.accepted = False
        self.turn_done = False
        self.turn_stage = ""  # стадия, которой закончился ход: done/failed/cancelled/silenced
        self.turn_info: Dict[str, str] = {}
        self.llm = 0
        self.started = 0
        self.ends = 0  # реплики, открытые в окне и закрытые в нём
        self.opened: Dict[str, Optional[float]] = {}  # speech_id -> ts последнего события
        self.closed: set = set()
        self.last_ts: Optional[float] = None  # последнее событие речи
        self.events = 0
        self.no_wake = False  # принятая фраза ушла в бэклог без вейка (#3989)
        self.tier1: List[str] = []  # виды ходов Tier-1 после принятой фразы
        self.performing = False  # выступление после принятой фразы взято и ещё не кончилось (#3971 R4)
        self.perform_since: Optional[float] = None  # метка лога его старта
        self.perform_kind = ""

    def feed(self, line: str) -> None:
        if "ПРИНЯТО" in line and not self.accepted:
            self.accepted = True
        if self.accepted:
            self._after_accept(line)
        event = speech_event(line, self.engine)
        if event:
            self._speech(event[0], event[1], _ts(line))

    def _after_accept(self, line: str) -> None:
        if "📊 [turn]" in line:
            self._turn(line)
        if "LLM REQUEST START" in line:
            self.llm += 1
        if NO_WAKE_RE.search(line):
            self.no_wake = True
        tier1 = TIER1_RE.search(line)
        if tier1:
            self.tier1.append(tier1.group(1))
        self._perform(line)

    def _perform(self, line: str) -> None:
        started = PERFORM_START_RE.search(line)
        if started:
            self.performing, self.perform_since, self.perform_kind = True, _ts(line), started.group(1)
        elif self.performing and PERFORM_END_RE.search(line):
            self.performing = False

    def _turn(self, line: str) -> None:
        found = TURN_RE.search(line)
        stage = found.group(1) if found else ""
        if stage not in TURN_END or (stage == "cancelled" and self.barge) or self.turn_stage == "failed":
            return
        self.turn_done, self.turn_stage = True, stage
        self.turn_info = {k: v for k, v in (m.groups() for m in re.finditer(r"\b(status|reason|llm_calls)=(\S+)", line))
                          if k in TURN_FIELDS}

    def _speech(self, kind: str, speech_id: str, ts: Optional[float]) -> None:
        self.events += 1
        if ts is not None:
            self.last_ts = ts
        if kind in CLOSING:
            if speech_id in self.opened:
                self.opened.pop(speech_id)
                self.closed.add(speech_id)
                self.ends += 1
            return
        if speech_id in self.closed:
            return
        if kind == "started":
            self.started += 1
        self.opened[speech_id] = ts if ts is not None else self.opened.get(speech_id)

    def busy(self) -> bool:
        """В окне есть ход: его конец, запрос к LLM, Tier-1 или речь."""
        return bool(self.turn_done or self.llm or self.tier1 or self.started or self.opened)

    def open_live(self, now: Optional[float]) -> int:
        """Открытые реплики; «протухшие» (последнее событие старше :data:`STALE_S`) не считаются говорящими."""
        if now is None:
            return len(self.opened)
        return sum(1 for ts in self.opened.values() if ts is None or now - ts <= STALE_S)

    def quiet_for(self, now: Optional[float]) -> float:
        """Секунд с последнего события речи; ``-1`` -- не определить (нет времени или часы разошлись)."""
        return _since(self.last_ts, now)

    def perform_for(self, now: Optional[float]) -> float:
        """Секунд с начала идущего выступления по часам робота; ``-1`` -- выступления нет или время не определить."""
        return _since(self.perform_since, now) if self.performing else -1.0


def _since(ts: Optional[float], now: Optional[float]) -> float:
    if now is None or ts is None:
        return -1.0
    gap = now - ts
    return gap if 0 <= gap <= MAX_SKEW_S else -1.0


def scan(text: str, engine: str, barge: bool = False) -> Tuple[Window, Optional[float]]:
    now, lines = split_input(text)
    window = Window(engine, barge)
    for line in lines:
        window.feed(line)
    return window, now


def _no_wake(window: Window, engine: str, waited: float) -> bool:
    """v2: фраза принята, ушла в бэклог без вейка, хода нет -- и STT за :data:`NO_WAKE_SETTLE_S` его не начал."""
    return engine == "v2" and window.no_wake and not window.busy() and waited >= NO_WAKE_SETTLE_S


def reply_verdict(window: Window, engine: str, silent_ok: bool, waited: float = 0.0, accept_window: float = 0.0,
                  now: Optional[float] = None, perform_ceiling: float = PERFORM_CEILING_S) -> str:
    """``done`` -- ответ состоялся и робот замолчал; ``nospeech`` -- ход закончен, речи не было (ждать льготу);
    ``silent`` -- ход закончен без речи и шаг это разрешает; ``failed`` / ``cancelled`` -- ход закончен сбоем /
    отменой (v2, #3954: терминально, ждать нечего); ``no_wake`` / ``silenced`` -- до хода не дошло: фраза ушла в бэклог
    без вейка / ход проглотила тишина (v2, #3989: терминально, харнесс решает о восстановлении шага; ``silent_ok`` их
    не прощает -- ответа на фразу шага не было); ``no_accept`` -- фразу STT не принял за ``accept_window`` секунд
    (переигрывать, не ждать потолок); ``perform_ceiling`` -- выступление идёт дольше ``perform_ceiling`` секунд от
    старта (терминально: повтор фразы заказал бы второе, #3971 R4); ``wait`` -- рано."""
    if not window.accepted:
        if accept_window > 0 and waited >= accept_window:
            return "no_accept"
        return "wait"
    if _no_wake(window, engine, waited):
        return "no_wake"
    if window.performing:
        return "perform_ceiling" if window.perform_for(now) >= perform_ceiling else "wait"
    if window.opened:
        return "wait"
    if engine == "v2":
        return _turn_verdict(window, silent_ok)
    return "done" if window.ends else "wait"


def _turn_verdict(window: Window, silent_ok: bool) -> str:
    """v2: вердикт по концу хода. Ход, проглоченный тишиной, которую включил САМ шаг («хватит, замолчи, посчитай»:
    ``tier1 kind=silence``, остаток фразы -- уже в тишине), -- не утечка, а ход без речи по просьбе шага."""
    if not window.turn_done:
        return "wait"
    if window.turn_stage in ("failed", "cancelled"):
        return window.turn_stage
    if window.turn_stage == SILENCED and SILENCE_TIER1 not in window.tier1:
        return SILENCED
    if window.ends:
        return "done"
    if window.started:
        return "wait"
    return "silent" if silent_ok else "nospeech"


def state(text: str, engine: str, silent_ok: bool, barge: bool = False, waited: float = 0.0,
          accept_window: float = 0.0, perform_ceiling: float = PERFORM_CEILING_S) -> Dict[str, object]:
    """Вердикт окна и счётчики для харнесса. ``performing=1`` -- идёт выступление: потолок ожидания харнесса --
    ``perform_ceiling`` (а не потолок ответа), судья сам скажет ``perform_ceiling`` по часам робота."""
    window, now = scan(text, engine, barge)
    live = window.open_live(now)
    if window.opened and live == 0:  # все открытые реплики протухли: говорящих нет
        window.opened.clear()
    verdict = reply_verdict(window, engine, silent_ok, waited, accept_window, now, perform_ceiling)
    info = window.turn_info
    return {
        "reason": _reason(window, verdict, now), "llm_calls": info.get("llm_calls", "-"),
        "fetch": 0 if now is None else 1, "verdict": verdict, "accepted": int(window.accepted),
        "turn_done": int(window.turn_done), "started": window.started, "ends": window.ends,
        "open": live, "llm": window.llm, "quiet_for": f"{window.quiet_for(now):.2f}",
        "performing": int(window.performing), "perform_for": f"{window.perform_for(now):.0f}",
        "perform_ceiling": f"{perform_ceiling:.0f}",
        "sig": f"{window.events}.{window.started}.{window.ends}.{int(window.turn_done)}",
    }


def _reason(window: Window, verdict: str = "", now: Optional[float] = None) -> str:
    """Причина вердикта без пробелов: ``no_wake``; ``perform_unfinished:kind=tale,for=241s``; конец хода из строки
    ``📊 [turn]`` -- ``failed:status=failed``; ``-`` если ход не кончился."""
    if verdict == "no_wake":
        return "no_wake"
    if verdict == "perform_ceiling":
        return f"perform_unfinished:kind={window.perform_kind},for={window.perform_for(now):.0f}s"
    if not window.turn_stage:
        return "-"
    parts = [f"{k}={v}" for k, v in window.turn_info.items() if k != "llm_calls"]
    return window.turn_stage + (":" + ",".join(parts) if parts else "")


def quiet(text: str, engine: str) -> Dict[str, object]:
    window, now = scan(text, engine)
    return {"fetch": 0 if now is None else 1, "open": window.open_live(now),
            "quiet_for": f"{window.quiet_for(now):.2f}", "sig": f"{window.events}"}


def anchor(text: str, engine: str, after: str, n: int) -> Dict[str, object]:
    """N-й ``SpeechEvent started`` (``speech_started``/``segment_started``) или N-я ``LLM REQUEST START``
    (``llm_request``) ПОСЛЕ первой принятой фразы окна. Метка времени -- из лога робота (0, если нет)."""
    now, lines = split_input(text)
    accepted, count = False, 0
    for line in lines:
        if "ПРИНЯТО" in line:
            accepted = True
            continue
        if not accepted:
            continue
        hit, speech_id = _anchor_hit(line, engine, after)
        if not hit:
            continue
        count += 1
        if count >= n:
            ts = _ts(line)
            return {"fetch": 1, "found": 1, "ts": f"{ts or 0:.3f}", "speech_id": speech_id or "-",
                    "now": f"{now or 0:.3f}"}
    return {"fetch": 0 if now is None else 1, "found": 0, "ts": "0", "speech_id": "-", "now": f"{now or 0:.3f}"}


def _anchor_hit(line: str, engine: str, after: str) -> Tuple[bool, Optional[str]]:
    if after == "llm_request":
        return ("LLM REQUEST START" in line), None
    event = speech_event(line, engine)
    if event and event[0] == "started":
        return True, event[1]
    return False, None


# -- восстановление шага (#3989) --------------------------------------------------------------------------


class Recovery(NamedTuple):
    """Что харнесс говорит роботу и повторяет ли потом фразу шага."""

    phrase: str
    replay: bool


#: ОДНО восстановление шага (#3989, решения Шифу 23.10 и 24.10) -- так сделал бы человек. ``no_wake`` -- фраза ушла
#: в бэклог без вейка: голое «Робот!» подхватывает свежий бэклог (В12, ``pickup``: фразы не старше
#: ``BACKLOG_PICKUP_S``), фразу шага не повторяем -- ответ на неё придёт по подхвату. ``silenced`` -- ход проглотила
#: тишина: снимает её только «Робот, говори» (вейк тишину не снимает), затем фраза шага ещё раз.
RECOVERY: Mapping[str, Recovery] = MappingProxyType({
    "no_wake": Recovery("Робот!", False),
    SILENCED: Recovery("Робот, говори.", True),
})
RECOVERABLE: Tuple[str, ...] = tuple(RECOVERY)
#: Восстановление ПО ВЫБОРУ шага (поле ``recover``: список; ``true`` его не включает): ``wake_replay`` -- вейк потерян, а
#: голое «Робот!» тем же голосом не помогло (n210 28.10: у голоса с акцентом STT путает и его) -- фраза шага звучит ещё
#: раз, ОДИН раз за шаг (решение Шифу 28.10: «пусть Гриша при отвале повторит»). Своей фразы у него нет: не строка
#: :data:`RECOVERY`, а повтор фразы шага.
WAKE_REPLAY = "wake_replay"
OPT_IN_RECOVERY: Tuple[str, ...] = (WAKE_REPLAY,)
#: Все виды, которые шаг может записать в ``recover`` и которые ставят маркер ``E2E_RECOVER <вид> <шаг>``.
STEP_RECOVERY_KINDS: Tuple[str, ...] = RECOVERABLE + OPT_IN_RECOVERY
#: Утечки прошлого акта, которые харнесс убирает на старте акта (#3989 п.5, #4027): тишина -- тем же видом и той же
#: репликой, что у шага (:data:`RECOVERY`), играющая музыка -- стопом через ``/mcp/execute``.
ACT_RECOVERY: Tuple[str, ...] = (SILENCED, "music")
#: Маркер восстановления в логе акта -- один вид на все места: ``E2E_RECOVER <вид> <метка шага|at_act_start> …``.
RECOVER_MARK = "E2E_RECOVER"
AT_ACT_START = "at_act_start"


def recover_marker(kind: str, where: str, detail: str = "") -> str:
    """Строка маркера восстановления (её печатает ``recover_mark`` харнесса). Вид -- из :data:`RECOVERY` у шага и из
    :data:`ACT_RECOVERY` на старте акта: «silence» вместо «silenced» -- ``ValueError``, а не второй словарь видов."""
    known = ACT_RECOVERY if where == AT_ACT_START else STEP_RECOVERY_KINDS
    if kind not in known:
        raise ValueError(f"{RECOVER_MARK} {kind} {where}: вид не из {list(known)}")
    if not re.fullmatch(r"\S+", where or ""):
        raise ValueError(f"{RECOVER_MARK} {kind}: где -- метка шага или {AT_ACT_START}, а не {where!r}")
    return " ".join(part for part in (RECOVER_MARK, kind, where, detail.strip()) if part)


def _norm(text: str) -> str:
    """Слова фразы в нижнем регистре без пунктуации: «Робот, говори.» -> «робот говори»."""
    return " ".join(re.findall(r"\w+", str(text).lower()))


def _kinds(raw: Any) -> Tuple[str, ...]:
    """Поле ``recover``: ``true`` -- все штатные виды (:data:`RECOVERABLE`), ``false``/``[]`` -- никаких, список --
    только эти (в нём можно назвать и вид по выбору шага, :data:`OPT_IN_RECOVERY`)."""
    if raw is True:
        return RECOVERABLE
    if raw is False:
        return ()
    if isinstance(raw, list) and all(isinstance(k, str) and k in STEP_RECOVERY_KINDS for k in raw):
        return tuple(k for k in STEP_RECOVERY_KINDS if k in raw)
    raise ValueError(f"recover: ожидается true|false|список из {list(STEP_RECOVERY_KINDS)}, а не {raw!r}")


def recover_kinds(scenario: Mapping[str, Any], step: Mapping[str, Any]) -> Tuple[str, ...]:
    """Какие восстановления разрешены шагу. Поле ``recover`` шага сильнее поля акта; нет ни того ни другого -- все.

    Кодом, без поля: шаг-перебиватель (``barge_in``) не восстанавливается -- речи, которую он перебивал, после
    восстановления уже нет, подхват бэклога встанет в очередь после неё; шаг, чья фраза и есть восстановление (n805b
    «Робот, говори.»), этим восстановлением не лечится -- иначе харнесс сделал бы шаг за робота."""
    kinds = _kinds(step["recover"] if "recover" in step else scenario.get("recover", True))
    if step.get("barge_in"):
        return ()
    own = _norm(step.get("text", ""))
    return tuple(k for k in kinds if k not in RECOVERY or _norm(RECOVERY[k].phrase) != own)


def recover_policy(scenario: Any) -> List[Tuple[str, Tuple[str, ...]]]:
    """``[(label, виды)]`` по шагам сценария; ошибка схемы поля ``recover`` -- ``ValueError``."""
    if not isinstance(scenario, Mapping):
        raise ValueError("сценарий -- не JSON-объект")
    _kinds(scenario.get("recover", True))
    steps = [s for s in scenario.get("steps", []) if isinstance(s, Mapping)]
    return [(str(s.get("label", f"s{i + 1}")), recover_kinds(scenario, s)) for i, s in enumerate(steps)]


# -- тишина в снимке сессии (#3989 п.5) -------------------------------------------------------------------


def silence_frame(scenes: Optional[List[Mapping[str, Any]]], now: float) -> Optional[Mapping[str, Any]]:
    """Живой кадр тишины в снимке ``/dialog/session`` или ``None``. Кадр с истёкшим ``ttl_until`` -- не тишина:
    владелец снимает истёкшие кадры лениво (``Session.expire`` на следующем ходе), latched-снимок остаётся прежним."""
    for scene in scenes or ():
        if scene.get("kind") != SILENCE_SCENE:
            continue
        ttl = float(scene.get("ttl_until") or 0.0)
        if ttl <= 0 or ttl > now:
            return scene
    return None


def _describe(frame: Optional[Mapping[str, Any]], now: float) -> str:
    if frame is None:
        return "тишины нет"
    ttl = float(frame.get("ttl_until") or 0.0)
    left = f"{int(ttl - now)}s" if ttl > 0 else "-"
    return f"{SILENCE_SCENE} set_by={frame.get('set_by') or '?'} ttl_left={left}"


def session_report(report: Any) -> Dict[str, object]:
    """Ответ рабочего процесса харнесса (``snapshot``, ``scenes``, ``now``; после реплики снятия -- ``after``) ->
    ``fetch`` (1 -- ответ разобран), ``silenced``, ``after`` (``-`` -- снятия не было), ``detail``.
    Нет latched-снимка -- тишины нет: тишина -- изменение сессии, а каждое изменение публикуется."""
    if not isinstance(report, Mapping) or "snapshot" not in report:
        return {"fetch": 0, "silenced": 0, "after": "-", "detail": "нет ответа"}
    now = float(report.get("now") or 0.0)
    frame = silence_frame(report.get("scenes"), now)
    after = report.get("after", "-")
    if after != "-":
        after = int(silence_frame(after, now) is not None)
    detail = _describe(frame, now) if report.get("snapshot") else "снимка нет (сессию с запуска ноды не меняли)"
    return {"fetch": 1, "silenced": int(frame is not None), "after": after, "detail": detail}


# -- предусловие шага: skip_when (#3971 R21) --------------------------------------------------------------------


class SkipRule(NamedTuple):
    """Шаг не играется, если в логе робота за акт есть строка ``log`` (``absent`` -- если её НЕТ): ``SKIP <reason>``."""

    reason: str
    log: Pattern[str]
    absent: bool = False
    deck: str = ""


#: Поле шага (пишет генератор марафона, таблица ``SKIP_WHEN``): ``{"reason": "KIE_QUOTA", "log": <regex>}``.
SKIP_FIELD = "skip_when"
_SKIP_REASON_RE = re.compile(r"[A-Z][A-Z0-9_]*")
#: ``deck`` правила: состояние деки, а не строка лога. ``ended`` -- последнее событие деки за акт ``finished``: трек
#: доиграл сам, и стопить уже нечего (n312 28.10: кавер на 92 с кончился в 16:05:11, команда пришла в 16:05:12).
DECK_ENDED = "ended"
SKIP_DECKS = (DECK_ENDED,)
#: Сколько символов строки-улики выводить в лог шага.
EVIDENCE_CHARS = 200


def skip_rule(step: Mapping[str, Any]) -> Optional[SkipRule]:
    """Правило пропуска шага или ``None``; кривое поле -- ``ValueError`` (молча сыграть шаг без предусловия нельзя:
    n710 удалял чужой трек, когда песни прогона не было)."""
    raw = step.get(SKIP_FIELD)
    if raw is None:
        return None
    label = step.get("label", "?")
    if not isinstance(raw, Mapping) or not isinstance(raw.get("reason"), str) or not isinstance(raw.get("log"), str):
        raise ValueError(f"{label}: {SKIP_FIELD} -- {{reason: ИМЯ, log: regex}}, а не {raw!r}")
    if not _SKIP_REASON_RE.fullmatch(raw["reason"]) or not raw["log"]:
        raise ValueError(f"{label}: {SKIP_FIELD}: причина -- ИМЯ_ЗАГЛАВНЫМИ, regex -- не пустой: {raw!r}")
    absent = raw.get("absent", False)
    if not isinstance(absent, bool):
        raise ValueError(f"{label}: {SKIP_FIELD}.absent -- true|false, а не {absent!r}")
    deck = raw.get("deck", "")
    if deck and deck not in SKIP_DECKS:
        raise ValueError(f"{label}: {SKIP_FIELD}.deck -- одно из {list(SKIP_DECKS)}, а не {deck!r}")
    try:
        return SkipRule(raw["reason"], re.compile(raw["log"]), absent, deck)
    except re.error as exc:
        raise ValueError(f"{label}: {SKIP_FIELD}.log -- не regex: {exc}") from exc


def skip_rules(scenario: Any) -> List[Tuple[str, SkipRule]]:
    """``[(label, правило)]`` шагов с предусловием; ошибка схемы любого шага -- ``ValueError`` до первого шага."""
    if not isinstance(scenario, Mapping):
        raise ValueError("сценарий -- не JSON-объект")
    steps = [s for s in scenario.get("steps", []) if isinstance(s, Mapping)]
    rules = [(str(s.get("label", f"s{i + 1}")), skip_rule(s)) for i, s in enumerate(steps)]
    return [(label, rule) for label, rule in rules if rule is not None]


def skip_verdict(rule: SkipRule, text: str) -> Dict[str, object]:
    """Лог робота за акт (первая строка -- часы робота) -> ``fetch`` (0 -- лог не прочитан, предусловие не
    проверено), ``skip`` (причина или ``-``), ``evidence`` (первая строка-улика)."""
    now, lines = split_input(text)
    if now is None:
        return {"fetch": 0, "skip": "-", "evidence": "-"}
    if rule.deck:
        return _deck_verdict(rule, lines)
    hit = next((line for line in lines if rule.log.search(line)), None)
    if rule.absent:
        if hit is not None:
            return {"fetch": 1, "skip": "-", "evidence": "-"}
        return {"fetch": 1, "skip": rule.reason, "evidence": f"нет строки /{rule.log.pattern}/"}
    if hit is None:
        return {"fetch": 1, "skip": "-", "evidence": "-"}
    return {"fetch": 1, "skip": rule.reason, "evidence": " ".join(hit.split())[:EVIDENCE_CHARS]}


def _deck_verdict(rule: SkipRule, lines: List[str]) -> Dict[str, object]:
    """Правило с ``deck``: судит ПОСЛЕДНЕЕ событие деки за акт (``started``/``stop``/``finished``; правка ``edit``
    трек не меняет). ``ended`` -- пропуск только если последним было ``finished`` (трек доиграл сам). Деку снял
    стоп или она не заигрывала -- шаг играется: красный там честный, а не «трек кончился сам»."""
    last: Optional[Tuple[MusicEvent, str]] = None
    for line in lines:
        event = music_event(line)
        if event is not None and event.kind != "edit":
            last = (event, line)
    if last is None or last[0].kind != "finished" or not rule.log.search(last[1]):
        return {"fetch": 1, "skip": "-", "evidence": "-"}
    return {"fetch": 1, "skip": rule.reason, "evidence": " ".join(last[1].split())[:EVIDENCE_CHARS]}


# -- ссылка шага на трек песни акта: track_ref (#3710, #4077) -----------------------------------------------------


class TrackRef(NamedTuple):
    """Шаг про трек, названный логом акта: ``log`` -- где робот называет id (группа 1), ``tool``/``action`` -- каким
    вызовом шаг обязан к нему обратиться, ``wait_sec`` -- сколько харнесс ждёт строку (песня пишется ~90-150 с)."""

    reason: str
    log: Pattern[str]
    tool: str
    action: str
    wait_sec: int = 0


TRACK_REF_FIELD = "track_ref"
#: Потолок ожидания id трека: дольше ждать песню шаг не станет, нет строки -- ``SKIP <reason>``.
TRACK_REF_MAX_WAIT_S = 600
_TOOL_NAME_RE = re.compile(r"[a-z][a-z0-9_]*")
#: Вызов тула в логе (``mcp_server``: ``📥 Запрос выполнения: <tool> с параметрами {'action': 'info', …}``).
TOOL_CALL_RE = re.compile(r"Запрос выполнения: (\w+) с параметрами (\{.*\})\s*$")


def track_ref(step: Mapping[str, Any]) -> Optional[TrackRef]:
    """Ссылка шага на трек или ``None``; кривое поле -- ``ValueError`` (молча сыграть шаг без сверки id нельзя: так
    n707/n708 27.10 были зелёными на любой вызов ``music_library``)."""
    raw = step.get(TRACK_REF_FIELD)
    if raw is None:
        return None
    where = f"{step.get('label', '?')}: {TRACK_REF_FIELD}"
    if not isinstance(raw, Mapping):
        raise ValueError(f"{where} -- объект {{reason, log, tool, action}}, а не {raw!r}")
    reason, log, tool, action = (raw.get(k) for k in ("reason", "log", "tool", "action"))
    if not isinstance(reason, str) or not _SKIP_REASON_RE.fullmatch(reason):
        raise ValueError(f"{where}.reason -- ИМЯ_ЗАГЛАВНЫМИ, а не {reason!r}")
    if not isinstance(tool, str) or not _TOOL_NAME_RE.fullmatch(tool):
        raise ValueError(f"{where}.tool -- имя тула, а не {tool!r}")
    if not isinstance(action, str) or not action:
        raise ValueError(f"{where}.action -- непустая строка, а не {action!r}")
    wait = raw.get("wait_sec", 0)
    if isinstance(wait, bool) or not isinstance(wait, int) or not 0 <= wait <= TRACK_REF_MAX_WAIT_S:
        raise ValueError(f"{where}.wait_sec -- целое 0..{TRACK_REF_MAX_WAIT_S}, а не {wait!r}")
    if not isinstance(log, str) or not log:
        raise ValueError(f"{where}.log -- regex с одной группой (id трека), а не {log!r}")
    try:
        pattern = re.compile(log)
    except re.error as exc:
        raise ValueError(f"{where}.log -- не regex: {exc}") from exc
    if pattern.groups != 1:
        raise ValueError(f"{where}.log -- ровно одна группа (id трека), а их {pattern.groups}: {log!r}")
    return TrackRef(reason, pattern, tool, action, wait)


def track_ref_rules(scenario: Any) -> List[Tuple[str, TrackRef]]:
    """``[(label, ссылка)]`` шагов с ``track_ref``; ошибка схемы любого шага -- ``ValueError`` до первого шага."""
    if not isinstance(scenario, Mapping):
        raise ValueError("сценарий -- не JSON-объект")
    steps = [s for s in scenario.get("steps", []) if isinstance(s, Mapping)]
    rules = [(str(s.get("label", f"s{i + 1}")), track_ref(s)) for i, s in enumerate(steps)]
    return [(label, rule) for label, rule in rules if rule is not None]


def track_ref_bind(rule: TrackRef, text: str) -> Dict[str, object]:
    """Лог робота за акт (первая строка -- часы робота) -> ``fetch`` (0 -- лог не прочитан), ``track`` (id из ПОСЛЕДНЕЙ
    строки-улики или ``-``), ``evidence`` (эта строка). Нет id -- харнесс ждёт ``wait_sec`` и спрашивает снова."""
    now, lines = split_input(text)
    if now is None:
        return {"fetch": 0, "track": "-", "evidence": "-"}
    hits = [(found, line) for line in lines for found in [rule.log.search(line)] if found]
    if not hits:
        return {"fetch": 1, "track": "-", "evidence": "-"}
    found, line = hits[-1]
    return {"fetch": 1, "track": found.group(1), "evidence": " ".join(line.split())[:EVIDENCE_CHARS]}


def tool_calls(lines: List[str], tool: str) -> List[Mapping[str, Any]]:
    """Параметры вызовов ``tool`` в строках лога (по порядку); строка с нечитаемым словарём вызовом не считается."""
    calls: List[Mapping[str, Any]] = []
    for line in lines:
        found = TOOL_CALL_RE.search(line)
        if found is None or found.group(1) != tool:
            continue
        try:
            params = ast.literal_eval(found.group(2))
        except (ValueError, SyntaxError):
            continue
        if isinstance(params, Mapping):
            calls.append(params)
    return calls


def track_ref_verdict(rule: TrackRef, expected: str, text: str) -> Dict[str, object]:
    """Лог окна шага (первая строка -- часы робота) и id песни акта -> ``fetch``, ``ok`` (1 -- есть вызов ``tool`` с
    ``action`` и ТЕМ ЖЕ ``track_id``), ``detail`` (что вызывал на деле)."""
    now, lines = split_input(text)
    if now is None:
        return {"fetch": 0, "ok": 0, "detail": "лог шага не прочитан"}
    calls = tool_calls(lines, rule.tool)
    same_action = [c for c in calls if c.get("action") == rule.action]
    if any(str(c.get("track_id", "")) == expected for c in same_action):
        return {"fetch": 1, "ok": 1, "detail": f"{rule.tool} {rule.action} track_id={expected}"}
    if not calls:
        detail = f"в окне шага нет вызова {rule.tool}; ждали {rule.action} track_id={expected}"
    elif not same_action:
        seen = sorted({str(c.get("action", "?")) for c in calls})
        detail = f"{rule.tool} вызван с action={seen}, ждали {rule.action} track_id={expected}"
    else:
        ids = sorted({str(c.get("track_id", "")) or "(пусто)" for c in same_action})
        detail = f"{rule.tool} {rule.action} на track_id={ids}, а песня акта -- {expected}"
    return {"fetch": 1, "ok": 0, "detail": detail}


# -- вид результата музыки: сверка с тем, что играло ДО шага (#3971) -----------------------------------------------


class MusicEvent(NamedTuple):
    """Событие деки v2: трек встал (``started``, с ``bpm``), играющий трек переправлен на такте (``edit``: тот же
    ``track_id``, новый ``bpm``), дека снята (``stop``, с ``reason``) или трек доиграл сам (``finished``)."""

    ts: float
    kind: str
    track_id: str
    bpm: float = 0.0
    reason: str = ""


#: Строки деки: ``player_owner.py`` -- ``🎵 [music v2] started track_id=<id> … bpm=<число> …`` / ``… stop track_id=<id>
#: reason=<причина>`` / ``… finished track_id=<id>``; ``engine/track_edit.py`` (#4029, правка играющего на такте без
#: нового ``started``) -- ``🎚️ [music v2] edit applied track_id=<id> at_beat=<доля> bpm=<число>``. ``⚠️ … started
#: track_id=<id> устарел`` (заменён до старта) -- без ``bpm``, это не событие деки.
MUSIC_EVENT_RE = re.compile(r"\[music v2\] (started|edit applied|stop|finished) track_id=(\S+)(.*)")
_BPM_RE = re.compile(r"\bbpm=([0-9.]+)")
_REASON_RE = re.compile(r"\breason=(\S+)")
#: Вид события по слову строки (``edit applied`` -> ``edit``).
_EVENT_KIND = MappingProxyType({"started": "started", "edit applied": "edit", "stop": "stop", "finished": "finished"})
#: Стоп по просьбе человека (``stop_music``, ``dj_set stop``). ``new_set`` / ``request_music`` / ``music_library`` --
#: деку занял другой заказ: это не «выключи».
USER_STOP = "user_stop"
#: Поле шага (пишет генератор марафона, таблица ``RESULT_KINDS``): какой сверкой судить результат.
COMPARE_FIELD = "music_compare"


def music_event(line: str) -> Optional[MusicEvent]:
    """Событие деки в строке лога или ``None`` (нет метки времени робота -- не событие: порядок не восстановить)."""
    found, ts = MUSIC_EVENT_RE.search(line), _ts(line)
    if found is None or ts is None:
        return None
    word, track_id, rest = found.groups()
    kind = _EVENT_KIND[word]
    if kind in ("stop", "finished"):
        reason = _REASON_RE.search(rest)
        return MusicEvent(ts, kind, track_id, reason=reason.group(1) if reason else "")
    bpm = _BPM_RE.search(rest)
    return MusicEvent(ts, kind, track_id, float(bpm.group(1))) if bpm else None


def deck_after(events: List[MusicEvent], deck: Optional[MusicEvent] = None) -> Optional[MusicEvent]:
    """Что играет после ``events`` (начиная с ``deck``): ``started`` ставит трек, ``edit`` того же трека меняет его
    ``bpm``, ``stop``/``finished`` снимают деку."""
    for event in events:
        if event.kind == "edit":
            deck = event if deck is not None and deck.track_id == event.track_id else deck
        else:
            deck = event if event.kind == "started" else None
    return deck


def _deck(event: Optional[MusicEvent]) -> str:
    return f"{event.track_id} bpm={event.bpm:g}" if event is not None else "тишина"


def _faster(before: MusicEvent, end: Optional[MusicEvent], _window: List[MusicEvent]) -> Tuple[bool, str]:
    """«Ускорь»: ТОТ ЖЕ трек (правка на такте, #4029) с темпом выше игравшего, а не новый трек."""
    if end is None:
        return False, "к концу шага музыка не играет"
    if end.track_id != before.track_id:
        return False, "трек сменился -- новый трек, а не правка играющего"
    if end.bpm <= before.bpm:
        return False, "темп не вырос"
    return True, ""


def _stopped(_before: MusicEvent, end: Optional[MusicEvent], window: List[MusicEvent]) -> Tuple[bool, str]:
    """«Выключи»: дека снята стопом человека с настоящим треком, и к концу шага ничего не играет."""
    if end is not None:
        return False, "к концу шага музыка играет"
    if not any(e.kind == "stop" and e.reason == USER_STOP and e.track_id != "None" for e in window):
        return False, f"нет стопа человека (reason={USER_STOP}) с треком на деке"
    return True, ""


#: Сверки результата с музыкой до шага: поле ``music_compare`` -> ``(до, в конце шага, события шага) -> (ok, почему)``.
COMPARES: Mapping[str, Callable[[MusicEvent, Optional[MusicEvent], List[MusicEvent]], Tuple[bool, str]]] = \
    MappingProxyType({"faster": _faster, "stopped": _stopped})


def compare_kind(step: Mapping[str, Any]) -> str:
    """Сверка шага (``""`` -- её нет); чужое имя -- ``ValueError``: молча засчитать шаг без сверки нельзя."""
    raw = step.get(COMPARE_FIELD)
    if raw is None:
        return ""
    if raw not in COMPARES:
        raise ValueError(f"{step.get('label', '?')}: {COMPARE_FIELD} -- одно из {list(COMPARES)}, а не {raw!r}")
    return str(raw)


def epoch_of(stamp: str) -> Optional[float]:
    """Начало окна шага: секунды epoch или ``STEP_BEFORE`` харнесса (``2026-10-27T23:24:30Z``, UTC робота)."""
    try:
        return float(stamp)
    except ValueError:
        pass
    try:
        return datetime.strptime(stamp, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc).timestamp()
    except ValueError:
        return None


def music_compare(kind: str, text: str, since: float) -> Dict[str, object]:
    """Лог робота за акт (первая строка -- часы робота) и начало окна шага -> ``fetch`` (0 -- лог не прочитан,
    сверка не доказана), ``ok`` (1 -- результат того вида, который просили), ``detail`` (что играло до и в конце)."""
    now, lines = split_input(text)
    if now is None:
        return {"fetch": 0, "ok": 0, "detail": "лог робота не прочитан -- сверка не доказана"}
    events = [e for e in (music_event(line) for line in lines) if e is not None]
    before = deck_after([e for e in events if e.ts < since])
    window = [e for e in events if e.ts >= since]
    end = deck_after(window, before)
    seen = f"до шага {_deck(before)}, в конце шага {_deck(end)}"
    if before is None:
        return {"fetch": 1, "ok": 0, "detail": f"до шага музыка не играла -- сверять не с чем ({seen})"}
    ok, why = COMPARES[kind](before, end, window)
    return {"fetch": 1, "ok": int(ok), "detail": f"{why}: {seen}" if why else seen}


# -- CLI ---------------------------------------------------------------------------------------------------


def _emit(values: Dict[str, object]) -> None:
    for key, value in values.items():
        sys.stdout.write(f"{key}={value}\n")


def _num(value: str) -> float:
    try:
        return float(value)
    except ValueError:
        return 0.0


def _cmd_state(opts: Dict[str, str], flags: List[str]) -> int:
    ceiling = _num(opts["--perform-ceiling"]) or PERFORM_CEILING_S  # пусто/0 -- потолок по умолчанию
    _emit(state(sys.stdin.read(), opts["engine"], "--silent-ok" in flags, "--barge" in flags, _num(opts["--waited"]),
                _num(opts["--accept-window"]), ceiling))
    return 0


def _cmd_quiet(opts: Dict[str, str], flags: List[str]) -> int:
    _emit(quiet(sys.stdin.read(), opts["engine"]))
    return 0


def _cmd_anchor(opts: Dict[str, str], flags: List[str]) -> int:
    try:
        n = max(1, int(opts["--n"]))
    except ValueError:
        n = 1
    _emit(anchor(sys.stdin.read(), opts["engine"], opts["--after"], n))
    return 0


def _cmd_policy(opts: Dict[str, str], flags: List[str]) -> int:
    """``label<TAB>no_wake,silenced`` (``-`` -- нельзя) по шагам; ошибка схемы -- rc 2 и причина в stderr."""
    try:
        with open(opts["--scenario"], encoding="utf-8") as fh:
            rows = recover_policy(json.load(fh))
    except (OSError, ValueError) as exc:
        sys.stderr.write(f"{opts['--scenario']}: {exc}\n")
        return 2
    for label, kinds in rows:
        sys.stdout.write(f"{label}\t{','.join(kinds) or '-'}\n")
    return 0


def _cmd_recovery(opts: Dict[str, str], flags: List[str]) -> int:
    """``--kind K`` -> ``phrase=`` / ``replay=``; без вида -- ``kinds=`` (все виды)."""
    recovery = RECOVERY.get(opts["--kind"])
    if recovery is None:
        _emit({"kinds": ",".join(RECOVERABLE)})
        return 0 if not opts["--kind"] else 2
    _emit({"phrase": recovery.phrase, "replay": int(recovery.replay)})
    return 0


def _cmd_session(opts: Dict[str, str], flags: List[str]) -> int:
    try:
        report = json.loads(sys.stdin.read())
    except ValueError:
        report = None
    _emit(session_report(report))
    return 0


def _cmd_mark(opts: Dict[str, str], flags: List[str]) -> int:
    """``--kind K --where W [--detail D]`` -> строка маркера ``E2E_RECOVER``; чужой вид -- rc 2 и причина в stderr."""
    try:
        sys.stdout.write(recover_marker(opts["--kind"], opts["--where"], opts["--detail"]) + "\n")
    except ValueError as exc:
        sys.stderr.write(f"{exc}\n")
        return 2
    return 0


def _load_scenario(path: str) -> Any:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _cmd_skips(opts: Dict[str, str], flags: List[str]) -> int:
    """``label<TAB>причина`` шагов с ``skip_when``; ошибка схемы -- rc 2 и причина в stderr."""
    try:
        rules = skip_rules(_load_scenario(opts["--scenario"]))
    except (OSError, ValueError) as exc:
        sys.stderr.write(f"{opts['--scenario']}: {exc}\n")
        return 2
    for label, rule in rules:
        sys.stdout.write(f"{label}\t{rule.reason}\n")
    return 0


def _step_of(opts: Dict[str, str]) -> Mapping[str, Any]:
    """Шаг ``--label`` сценария ``--scenario`` (нет такого -- пустой: правил у него нет)."""
    steps = _load_scenario(opts["--scenario"]).get("steps", [])
    return next((s for s in steps if isinstance(s, Mapping) and s.get("label") == opts["--label"]), {})


def _cmd_skip(opts: Dict[str, str], flags: List[str]) -> int:
    """``--scenario F --label L`` < лог акта -> ``fetch=`` / ``skip=`` / ``evidence=``; у шага нет правила -- ``skip=-``."""
    try:
        rule = skip_rule(_step_of(opts))
    except (OSError, ValueError, AttributeError) as exc:
        sys.stderr.write(f"{opts['--scenario']}: {exc}\n")
        return 2
    _emit(skip_verdict(rule, sys.stdin.read()) if rule else {"fetch": 1, "skip": "-", "evidence": "-"})
    return 0


def _cmd_compare(opts: Dict[str, str], flags: List[str]) -> int:
    """``--scenario F --label L`` -> ``compare=<сверка|->``; с ``--since S`` (начало окна шага) и логом деки за акт на
    stdin -- ещё ``fetch=`` / ``ok=`` / ``detail=``. Кривое поле или ``--since`` -- rc 2 и причина в stderr."""
    try:
        kind = compare_kind(_step_of(opts))
    except (OSError, ValueError, AttributeError) as exc:
        sys.stderr.write(f"{opts['--scenario']}: {exc}\n")
        return 2
    out: Dict[str, object] = {"compare": kind or "-"}
    if kind and opts["--since"]:
        since = epoch_of(opts["--since"])
        if since is None:
            sys.stderr.write(f"--since {opts['--since']!r}: ни epoch, ни YYYY-MM-DDTHH:MM:SSZ\n")
            return 2
        out.update(music_compare(kind, sys.stdin.read(), since))
    _emit(out)
    return 0


def _cmd_trackrefs(opts: Dict[str, str], flags: List[str]) -> int:
    """``label<TAB>причина<TAB>wait_sec`` шагов с ``track_ref``; ошибка схемы -- rc 2 и причина в stderr."""
    try:
        rules = track_ref_rules(_load_scenario(opts["--scenario"]))
    except (OSError, ValueError) as exc:
        sys.stderr.write(f"{opts['--scenario']}: {exc}\n")
        return 2
    for label, rule in rules:
        sys.stdout.write(f"{label}\t{rule.reason}\t{rule.wait_sec}\n")
    return 0


def _cmd_trackbind(opts: Dict[str, str], flags: List[str]) -> int:
    """``--scenario F --label L`` < лог акта -> ``fetch=`` / ``track=`` / ``evidence=``."""
    try:
        rule = track_ref(_step_of(opts))
    except (OSError, ValueError, AttributeError) as exc:
        sys.stderr.write(f"{opts['--scenario']}: {exc}\n")
        return 2
    _emit(track_ref_bind(rule, sys.stdin.read()) if rule else {"fetch": 1, "track": "-", "evidence": "-"})
    return 0


def _cmd_trackcheck(opts: Dict[str, str], flags: List[str]) -> int:
    """``--scenario F --label L --track ID`` < лог окна шага -> ``fetch=`` / ``ok=`` / ``detail=``."""
    try:
        rule = track_ref(_step_of(opts))
    except (OSError, ValueError, AttributeError) as exc:
        sys.stderr.write(f"{opts['--scenario']}: {exc}\n")
        return 2
    if rule is None or not opts["--track"]:
        sys.stderr.write(f"{opts['--label']}: нет track_ref или не назван --track\n")
        return 2
    _emit(track_ref_verdict(rule, opts["--track"], sys.stdin.read()))
    return 0


COMMANDS: Mapping[str, Callable[[Dict[str, str], List[str]], int]] = MappingProxyType({
    "state": _cmd_state, "quiet": _cmd_quiet, "anchor": _cmd_anchor,
    "policy": _cmd_policy, "recovery": _cmd_recovery, "session": _cmd_session,
    "mark": _cmd_mark, "skips": _cmd_skips, "skip": _cmd_skip, "compare": _cmd_compare,
    "trackrefs": _cmd_trackrefs, "trackbind": _cmd_trackbind, "trackcheck": _cmd_trackcheck,
})
USAGE = ("usage: e2e_events.py state|quiet|anchor --engine v1|v2 [--silent-ok] [--barge] [--waited S --accept-window S]"
         " [--perform-ceiling S] [--after A --n N] | policy --scenario FILE | recovery [--kind K] | session < report.json"
         " | mark --kind K --where W [--detail D] | skips --scenario FILE | skip --scenario FILE --label L < log"
         " | compare --scenario FILE --label L [--since S < log] | trackrefs --scenario FILE"
         " | trackbind --scenario FILE --label L < log | trackcheck --scenario FILE --label L --track ID < log\n")


def main(argv: List[str]) -> int:
    if len(argv) < 2 or argv[1] not in COMMANDS:
        sys.stderr.write(USAGE)
        return 2
    for stream in (sys.stdin, sys.stdout):  # фразы и метки -- кириллица; Windows-консоль по умолчанию не utf-8
        try:
            stream.reconfigure(encoding="utf-8", newline="\n" if stream is sys.stdout else None)
        except (AttributeError, ValueError):
            pass
    opts = {"--engine": "v1", "--after": "speech_started", "--n": "1", "--waited": "0", "--accept-window": "0",
            "--scenario": "", "--kind": "", "--perform-ceiling": "", "--where": "", "--detail": "", "--label": "",
            "--since": "", "--track": ""}
    flags = argv[2:]
    for i, tok in enumerate(flags):
        if tok in opts and i + 1 < len(flags):
            opts[tok] = flags[i + 1]
    opts["engine"] = "v2" if opts["--engine"] == "v2" else "v1"
    return COMMANDS[argv[1]](opts, flags)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
