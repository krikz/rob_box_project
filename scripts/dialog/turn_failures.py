#!/usr/bin/env python3
"""Детекторы сбоя хода кодом: «робот глух» и «ход без речи» (#4512, шаг 3; ADR-0148 «код решает»).

Ресёрч #4512 (``docs/research/2026-11-09-robot-event-model.md`` §4.3): сигналы механического сбоя в логе
``voice-assistant`` есть по отдельности (STT ``-> rejected``, бэклог ``no_wake``, ход без ``SpeechEvent``), но вывода
«это сбой» не делает никто. Здесь вывод делает код: детерминированная функция над потоком событий, без ROS и без LLM.

Ядро — :class:`TurnWatch`: ``feed(event)`` отдаёт вердикты по мере событий, ``flush()`` — то, что ждало льготы.
Вход — :class:`Event`; источник событий — строки лога (:func:`events_from_log`). Ядро текст реплик не читает: адаптер
берёт из строки только вид события, время и id, поэтому вердикт можно класть в публичный отчёт.

Вердикты (:class:`Verdict`):

* :data:`DEAF` — «робот глух»: фразы людей приходят (STT получил звук), но ни одна не стала ходом. Срабатывает, когда
  без хода между ними за :data:`DEAF_WINDOW_S` набралось :data:`DEAF_REJECTED_MIN` фраз, отклонённых STT (порог по
  бэклогу без вейка :data:`DEAF_NO_WAKE_MIN` по умолчанию выключен — см. его описание). Причина — счёт по исходам
  серии (``stt_rejected=3,stt_error=2,no_wake=1``; ``stt_error`` — из отклонённых те, где в цепочке STT был отказ
  провайдера: ``timeout``/``dead``/``error``).
  Не фразы человека: эхо своей речи (``🔁 [echo]``) и пустой STT под играющей музыкой (это сама музыка).
* :data:`NO_SPEECH` — «ход без речи»: ход закончился (``📊 [turn]``), за :data:`SPEECH_GRACE_S` после конца у него нет
  ни начатой, ни ждущей очереди реплики, нет выступления, и законной причины молчать нет (:func:`legit_silence`):
  ход тишины, перебитый ход, ответ текстом в чат, пас модели на подхват бэклога, Tier-1 без фразы
  (``turn.tier1_phrase``), ответ одним звуком или лицом на просьбу. Причина — чем ход кончился
  (:func:`silence_reason`).

Где включать — предложение в PR (Refs #4512): офлайн-отчёт марафона, смотритель логов #4033.
Робот модуль не импортирует.

    python scripts/dialog/turn_failures.py <voice_e2e.log> [...]    # вердикты строками, без текста реплик
"""

from __future__ import annotations

import argparse
import re
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Set, Tuple

REPO = Path(__file__).resolve().parents[2]
for _pkg in ("rob_box_core", "rob_box_dialog"):
    _src = str(REPO / "src" / _pkg)
    if _src not in sys.path:
        sys.path.insert(0, _src)
from rob_box_dialog import knowledge as kn  # noqa: E402

# --- пороги: одно место (начальные значения — по 77 прогонам марафона 03–06.11 с логом, таблица в PR #4603) ---
#: Фраз, отклонённых STT (``-> rejected``: пусто или отказ провайдера), без хода между ними — робот глух. Человек,
#: которого не слышат, повторяет 2–3 раза; меньше трёх — шум (в марафоне пустых фраз 108 из 1 578 попыток STT).
DEAF_REJECTED_MIN = 3
#: Фраз в бэклог без вейка, без хода между ними, — порог для вывода «глух»; ``None`` — серия без вейка вывода не даёт,
#: только идёт в причину. По событиям фраза с потерянным «Робот» и разговор людей между собой неотличимы: на архиве
#: любой порог, что ловит хоть один из 5 шагов ``turn_no_wake``, даёт ≥ 22 ложных на OK-шагах (сетка в PR #4603).
DEAF_NO_WAKE_MIN: Optional[int] = None
#: Окно серии: от первой фразы до последней, с. Человек, которого не слышат, повторяет в пределах минуты.
DEAF_WINDOW_S = 60.0
#: Льгота после конца хода: реплика хода встаёт в очередь до ``📊 [turn]`` или сразу после него, с.
SPEECH_GRACE_S = 5.0

# --- виды событий и вердиктов ---
STT_REJECTED, NO_WAKE, ECHO, MUSIC = "stt_rejected", "no_wake", "echo", "music"
#: Причина «отклонил»: хоть один провайдер STT отказал (``timeout``/``dead``/``error``), а не услышал пустоту. Под
#: музыкой такая фраза в серию идёт: отказ провайдера — не музыка (голосовой стек деградирует молча).
STT_ERROR, STT_FAILED = "stt_error", "failed"
STT_PROVIDER_FAILURES: Tuple[str, ...] = ("timeout", "dead", "error")
TURN_OPEN, TURN_END, TIER1, EXECUTED, SPEECH, PERFORM = (
    "turn_open",
    "turn_end",
    "tier1",
    "executed",
    "speech",
    "perform",
)
PASSED = "passed"
MUSIC_ON, MUSIC_OFF = "on", "off"
DEAF, NO_SPEECH = "deaf", "no_speech"

#: Tier-1 виды, у которых ``turn.tier1_phrase`` фразы нет (стоп речи, тишина, её снятие, ответ на вопрос тула). Сверку с
#: ``tier1_phrase`` держит ``tests/unit/dialog/test_turn_failures.py``: разойдутся — тест красный.
TIER1_SILENT_KINDS: Tuple[str, ...] = ("stop_speech", "silence", "unsilence", "answer")
#: Стоп всего молчит, только если тулов стопа музыки в ходе не было (``turn.music_stop_phrase``, решение Шифу 05.11).
STOP_ALL_KIND = "stop_all"
#: Стадии конца хода, которые молчат законно: ход тишины и перебитый ход.
SILENT_STAGES: Tuple[str, ...] = (kn.SILENCED_TURN_STAGE, "cancelled")
#: Закрытие реплики без звука (``speech_owner``): встала в очередь, но не прозвучала.
SPEECH_DEAD: Tuple[str, ...] = ("rejected", "cancelled")
#: Класс тула «прочитал данные» (``knowledge.TOOL_CLASSES``): прочитал и ответил одним звуком — слова были должны
#: (``turn.sound_owed``, М11 n109).
QUERY_CLASS = "query"


@dataclass(frozen=True)
class Event:
    """Событие потока: время робота, вид, id хода; ``value`` — стадия хода / тул / вид Tier-1 / вид речи / музыка;
    ``extra`` — статус хода или тула, id реплики; ``agent`` — агент хода (у конца хода)."""

    ts: float
    kind: str
    turn_id: str = ""
    value: str = ""
    extra: str = ""
    agent: str = ""


@dataclass(frozen=True)
class Verdict:
    """Вывод детектора: вид (:data:`DEAF` / :data:`NO_SPEECH`), время, причина без пробелов, ход (у ``no_speech``)."""

    kind: str
    ts: float
    reason: str
    turn_id: str = ""
    since: float = 0.0  # у ``deaf``: время первой фразы серии

    def line(self) -> str:
        turn = f" turn={self.turn_id}" if self.turn_id else ""
        since = f" since={self.since:.3f}" if self.since else ""
        return f"🚨 [turn-fail] {self.kind} ts={self.ts:.3f}{since}{turn} reason={self.reason}"


@dataclass
class _Turn:
    turn_id: str
    opened: float
    ended: Optional[float] = None
    stage: str = ""
    status: str = ""
    agent: str = ""
    tier1: str = ""
    passed: bool = False
    tools: List[Tuple[str, str]] = field(default_factory=list)
    queued: Set[str] = field(default_factory=set)
    started: Set[str] = field(default_factory=set)
    dead: Set[str] = field(default_factory=set)
    performing: bool = False

    def voiced(self) -> bool:
        """Реплика прозвучала, ещё ждёт очереди или идёт выступление."""
        return bool(self.started) or bool(self.queued - self.dead) or self.performing

    def tool_names(self) -> Set[str]:
        return {tool for tool, _ in self.tools}


def legit_silence(turn: _Turn) -> bool:
    """Ход молчит законно: тишина, перебивание, ответ текстом в чат, пас модели на подхват бэклога, Tier-1 без фразы,
    ответ одним звуком или лицом на просьбу (данных ход не читал, ``turn.sound_owed``)."""
    if turn.stage in SILENT_STAGES or turn.agent == kn.TEXT_CHANNEL_AGENT or turn.passed:
        return True
    if turn.tier1:
        return tier1_silent(turn.tier1, turn.tools)
    tools = turn.tool_names()
    return bool(tools) and tools <= set(kn.SILENT_DONE_TOOLS)


def tier1_silent(kind: str, tools: Sequence[Tuple[str, str]]) -> bool:
    """Tier-1 без фразы (``turn.tier1_phrase``): стоп речи, тишина, её снятие, ответ на вопрос тула; стоп всего без
    стопа музыки (``turn.music_stop_phrase``); очередь показа, все сцены которой приняты (``turn.show_queue_phrase``).
    """
    if kind in TIER1_SILENT_KINDS:
        return True
    if kind == STOP_ALL_KIND:
        return not any(tool in kn.MUSIC_STOP_TOOLS for tool, _ in tools)
    if kind == kn.SHOW_QUEUE_KIND:
        return all(status == "done" for _, status in tools)
    return False


def silence_reason(turn: _Turn) -> str:
    """Чем кончился ход без речи: ``failed``, ``speech_dropped`` (реплика встала в очередь и отклонена),
    ``tier1:<вид>``, ``sound_after_query`` / ``query_only`` (прочитал данные и не сказал), ``actions_only``, ``empty``.
    """
    if turn.stage == "failed":
        return "failed"
    if turn.dead:
        return "speech_dropped"
    if turn.tier1:
        return f"tier1:{turn.tier1}"
    tools = turn.tool_names()
    queries = {tool for tool in tools if kn.TOOL_CLASSES.get(tool) == QUERY_CLASS}
    if queries and tools <= queries | set(kn.SILENT_DONE_TOOLS):
        return "sound_after_query" if tools - queries else "query_only"
    return "actions_only" if tools else "empty"


class TurnWatch:
    """Поток событий → вердикты. События — по времени; ``flush`` — в конце потока или по часам."""

    def __init__(self) -> None:
        self._turns: Dict[str, _Turn] = {}
        self._latest: Optional[_Turn] = None
        self._speech_owner: Dict[str, _Turn] = {}
        self._pending: List[_Turn] = []
        self._streak: Deque[Tuple[float, str]] = deque()
        self._fired = False
        self._music = False

    def feed(self, event: Event) -> List[Verdict]:
        out = self._judge_due(event.ts)
        handler = _HANDLERS.get(event.kind)
        if handler is not None:
            verdict = handler(self, event)
            if verdict is not None:
                out.append(verdict)
        return out

    def flush(self, now: Optional[float] = None) -> List[Verdict]:
        """Судит ходы, чья льгота прошла к ``now`` (``None`` — все ждущие: поток кончился)."""
        return self._judge_due(float("inf") if now is None else now)

    # --- «робот глух» ---
    def _music_state(self, event: Event) -> None:
        self._music = event.value == MUSIC_ON

    def _phrase_lost(self, event: Event) -> Optional[Verdict]:
        """Фраза не стала ходом. Пустой STT под музыкой — сама музыка (``stt_node``: «Пустой STT (эхо/музыка)»), не
        человек: в серию не идёт."""
        cause = STT_ERROR if event.kind == STT_REJECTED and event.value == STT_FAILED else event.kind
        if cause == STT_REJECTED and self._music:
            return None
        self._streak.append((event.ts, cause))
        while self._streak and event.ts - self._streak[0][0] > DEAF_WINDOW_S:
            self._streak.popleft()
        counts = {k: sum(1 for _, kind in self._streak if kind == k) for k in (STT_REJECTED, STT_ERROR, NO_WAKE)}
        counts[STT_REJECTED] += counts[STT_ERROR]
        no_wake_hit = DEAF_NO_WAKE_MIN is not None and counts[NO_WAKE] >= DEAF_NO_WAKE_MIN
        if self._fired or (counts[STT_REJECTED] < DEAF_REJECTED_MIN and not no_wake_hit):
            return None
        self._fired = True
        reason = ",".join(f"{k}={v}" for k, v in counts.items() if v)
        return Verdict(DEAF, event.ts, reason, since=self._streak[0][0])

    # --- ход ---
    def _open(self, event: Event) -> None:
        if event.turn_id in self._turns:
            return
        turn = _Turn(event.turn_id, event.ts)
        self._turns[event.turn_id] = turn
        self._latest = turn
        self._streak.clear()
        self._fired = False

    def _turn_of(self, event: Event) -> _Turn:
        self._open(event)
        return self._turns[event.turn_id]

    def _end(self, event: Event) -> None:
        turn = self._turn_of(event)
        if turn.ended is not None:
            return
        turn.ended, turn.stage, turn.status, turn.agent = event.ts, event.value, event.extra, event.agent
        self._pending.append(turn)

    def _tier1(self, event: Event) -> None:
        self._turn_of(event).tier1 = event.value

    def _executed(self, event: Event) -> None:
        self._turn_of(event).tools.append((event.value, event.extra))

    def _perform(self, event: Event) -> None:
        self._turn_of(event).performing = True

    def _passed(self, event: Event) -> None:
        """Пас модели на подхват бэклога (``addressed=False``): в строке нет id хода — это последний открытый ход."""
        turn = self._turns.get(event.turn_id) if event.turn_id else self._latest
        if turn is not None and turn.ended is None:
            turn.passed = True

    def _speech(self, event: Event) -> None:
        """Реплика без id хода: хозяин — ход, при котором она встала в очередь (последний открытый, льгота не
        прошла)."""
        speech_id, kind = event.extra, event.value
        turn = self._speech_owner.get(speech_id)
        if turn is None:
            turn = self._latest
            if turn is None or (turn.ended is not None and event.ts > turn.ended + SPEECH_GRACE_S):
                return
            self._speech_owner[speech_id] = turn
        if kind == "started":
            turn.started.add(speech_id)
        elif kind in SPEECH_DEAD and speech_id not in turn.started:
            turn.dead.add(speech_id)
        turn.queued.add(speech_id)

    def _judge_due(self, now: float) -> List[Verdict]:
        due = [t for t in self._pending if t.ended is not None and now > t.ended + SPEECH_GRACE_S]
        if not due:
            return []
        self._pending = [t for t in self._pending if t not in due]
        return [
            Verdict(NO_SPEECH, t.ended or 0.0, silence_reason(t), t.turn_id)
            for t in due
            if not t.voiced() and not legit_silence(t)
        ]


_HANDLERS = {
    STT_REJECTED: TurnWatch._phrase_lost,
    NO_WAKE: TurnWatch._phrase_lost,
    TURN_OPEN: TurnWatch._open,
    TURN_END: TurnWatch._end,
    TIER1: TurnWatch._tier1,
    EXECUTED: TurnWatch._executed,
    SPEECH: TurnWatch._speech,
    PERFORM: TurnWatch._perform,
    MUSIC: TurnWatch._music_state,
    PASSED: TurnWatch._passed,
}


def detect(events: Iterable[Event]) -> List[Verdict]:
    """Все вердикты потока (события — по времени), включая ходы, что ждали льготы до конца потока."""
    watch = TurnWatch()
    out: List[Verdict] = []
    for event in events:
        out += watch.feed(event)
    return out + watch.flush()


# --- адаптер: строки лога voice-assistant → события (только вид, время, id; текст реплик не берётся) ---
_TS = re.compile(r"\[(\d{9,11}\.\d+)\]")
_STT_ATTEMPT = re.compile(r"\[stt_attempt\] (.*)-> (rejected|accepted)\b")
_STT_FAILURE = re.compile(r":(?:" + "|".join(STT_PROVIDER_FAILURES) + r")\(")
_NO_WAKE = re.compile(r"\[backlog\] accumulated reason=no_wake\b")
_ECHO = "🔁 [echo] фраза совпадает"
_TURN_END = re.compile(r"📊 \[turn\] turn_id=([0-9a-f]{12}) .*?\bagent=(\S+) stage=(\S+) status=(\S+)")
_TIER1 = re.compile(r"tier1 kind=(\S+) .*\bturn=([0-9a-f]{12})")
_EXECUTED = re.compile(r"📨 \[execute\] событие (\S+) ([0-9a-f]{12})/\d+ (\S+)")
_PERFORM = re.compile(r"выступление старт .*\bturn=([0-9a-f]{12})")
_PASSED = re.compile(r"\bpassed=\S+ addressed=False\b")
_MUSIC = re.compile(r"🎵 \[music\] (started|finished|stop) ")
_SPEECH = re.compile(r"SpeechEvent (queued|started|rejected|cancelled) speech_id=(\S+)")
_TURN_ID = re.compile(r"\bturn(?:_id)?=([0-9a-f]{12})\b")
_DIALOG = "[dialog_node]"


def _turn_line(ts: float, line: str) -> Optional[Event]:
    """Строка хода с его id: конец, Tier-1, выступление, исполнение тула."""
    found = _TURN_END.search(line)
    if found:
        return Event(ts, TURN_END, found.group(1), found.group(3), found.group(4), agent=found.group(2))
    found = _TIER1.search(line)
    if found:
        return Event(ts, TIER1, found.group(2), found.group(1))
    found = _PERFORM.search(line)
    if found:
        return Event(ts, PERFORM, found.group(1))
    found = _EXECUTED.search(line)
    if found:
        return Event(ts, EXECUTED, found.group(2), found.group(1), found.group(3))
    return None


def _dialog_event(ts: float, line: str) -> Optional[Event]:
    event = _turn_line(ts, line)
    if event is not None:
        return event
    if _NO_WAKE.search(line):
        return Event(ts, NO_WAKE)
    if _PASSED.search(line):
        return Event(ts, PASSED)
    found = _TURN_ID.search(line)
    return Event(ts, TURN_OPEN, found.group(1)) if found else None


def _other_event(ts: float, line: str) -> Optional[Event]:
    found = _STT_ATTEMPT.search(line)
    if found:
        if found.group(2) != "rejected":
            return None
        return Event(ts, STT_REJECTED, value=STT_FAILED if _STT_FAILURE.search(found.group(1)) else "")
    if _ECHO in line:
        return Event(ts, ECHO)
    found = _SPEECH.search(line)
    if found:
        return Event(ts, SPEECH, value=found.group(1), extra=found.group(2))
    found = _MUSIC.search(line)
    if found:
        return Event(ts, MUSIC, value=MUSIC_ON if found.group(1) == "started" else MUSIC_OFF)
    return None


def line_event(line: str) -> Optional[Event]:
    """Событие одной строки лога или ``None`` (строка без метки времени — без события)."""
    stamp = _TS.search(line)
    if not stamp:
        return None
    ts = float(stamp.group(1))
    return _dialog_event(ts, line) if _DIALOG in line else _other_event(ts, line)


def events_from_log(lines: Iterable[str]) -> List[Event]:
    """События лога по времени. Логи соседних актов перекрываются: дубль события (то же время и вид) — один раз."""
    seen: Set[Event] = set()
    out: List[Event] = []
    for line in lines:
        event = line_event(line)
        if event is not None and event not in seen:
            seen.add(event)
            out.append(event)
    out.sort(key=lambda e: e.ts)
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    parser.add_argument("logs", nargs="+", type=Path)
    args = parser.parse_args(argv)
    for path in args.logs:
        with path.open(encoding="utf-8", errors="replace") as handle:
            for verdict in detect(events_from_log(handle)):
                print(f"{path.name} {verdict.line()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
