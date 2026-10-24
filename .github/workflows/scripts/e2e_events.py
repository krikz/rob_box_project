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
* ``📊 [turn] ... stage=done|failed|cancelled`` -- агрегат хода (``rob_box_dialog.turn_log.AGGREGATE_MARK``).
  Все три -- КОНЕЦ хода (#3954): ``failed`` не заставляет харнесс ждать потолок, вердикт ``failed`` несёт причину
  (``status``/``reason``/``llm_calls`` из той же строки). ``cancelled`` -- конец хода, кроме шага-перебивателя
  (``--barge``): там «отменён» -- след ПРОШЛОЙ фразы, и концом хода этого шага он быть не может;
* ``🗣️ SpeechEvent <kind> speech_id=<8> ...`` -- события владельца речи (``speech_owner._emit``): ``queued`` /
  ``synthesized`` / ``started`` / ``progress`` открывают или ведут реплику, ``finished`` / ``cancelled`` /
  ``rejected`` закрывают её. Пара -- по ``speech_id``. Каждый куплет выступления -- отдельный ``SpeechRequest``
  со своим ``speech_id``, значит и свой ``SpeechEvent started``.

v1: ``🔊 TTS: speech_id=<8>`` открывает реплику, ``Публикую TTS finished event: speech_id=<8>`` (и вариант
``(ошибка)``) закрывает.

В строке ``SpeechEvent`` нет ``turn_id`` и идентификатора выступления: «куплет именно этого выступления»
отличить от обычной реплики по логу нельзя. Окно лога ограничено фразой шага, поэтому ``segment_started``
считается как N-й ``started`` в окне (см. :func:`anchor`).
"""

from __future__ import annotations

import re
import sys
from typing import Dict, List, Optional, Tuple

TS_RE = re.compile(r"\[(\d{9,}\.\d+)\]")
SPEECH_V2 = re.compile(r"SpeechEvent (queued|synthesized|started|progress|finished|cancelled|rejected) speech_id=([0-9A-Za-z]+)")
V1_OPEN = re.compile(r"🔊 TTS: speech_id=([0-9a-f]{8})")
V1_CLOSE = re.compile(r"TTS finished event(?: \(ошибка\))?: speech_id=([0-9a-f]{8})")
OPENING = frozenset({"queued", "synthesized", "started"})
CLOSING = frozenset({"finished", "cancelled", "rejected"})
TURN_RE = re.compile(r"📊 \[turn\].*?\bstage=(\w+)")
TURN_FIELDS = ("status", "reason", "llm_calls")
TURN_END = frozenset({"done", "failed", "cancelled"})
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
        self.turn_stage = ""  # стадия, которой закончился ход: done/failed/cancelled
        self.turn_info: Dict[str, str] = {}
        self.llm = 0
        self.started = 0
        self.ends = 0  # реплики, открытые в окне и закрытые в нём
        self.opened: Dict[str, Optional[float]] = {}  # speech_id -> ts последнего события
        self.closed: set = set()
        self.last_ts: Optional[float] = None  # последнее событие речи
        self.events = 0

    def feed(self, line: str) -> None:
        if "ПРИНЯТО" in line and not self.accepted:
            self.accepted = True
        if self.accepted and "📊 [turn]" in line:
            self._turn(line)
        if self.accepted and "LLM REQUEST START" in line:
            self.llm += 1
        event = speech_event(line, self.engine)
        if event:
            self._speech(event[0], event[1], _ts(line))

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

    def open_live(self, now: Optional[float]) -> int:
        """Открытые реплики; «протухшие» (последнее событие старше :data:`STALE_S`) не считаются говорящими."""
        if now is None:
            return len(self.opened)
        return sum(1 for ts in self.opened.values() if ts is None or now - ts <= STALE_S)

    def quiet_for(self, now: Optional[float]) -> float:
        """Секунд с последнего события речи; ``-1`` -- не определить (нет времени или часы разошлись)."""
        if now is None or self.last_ts is None:
            return -1.0
        gap = now - self.last_ts
        return gap if 0 <= gap <= MAX_SKEW_S else -1.0


def scan(text: str, engine: str, barge: bool = False) -> Tuple[Window, Optional[float]]:
    now, lines = split_input(text)
    window = Window(engine, barge)
    for line in lines:
        window.feed(line)
    return window, now


def reply_verdict(window: Window, engine: str, silent_ok: bool, waited: float = 0.0, accept_window: float = 0.0) -> str:
    """``done`` -- ответ состоялся и робот замолчал; ``nospeech`` -- ход закончен, речи не было (ждать льготу);
    ``silent`` -- ход закончен без речи и шаг это разрешает; ``failed`` / ``cancelled`` -- ход закончен сбоем /
    отменой (v2, #3954: терминально, ждать нечего); ``no_accept`` -- фразу STT не принял за ``accept_window``
    секунд (переигрывать, не ждать потолок); ``wait`` -- рано."""
    if not window.accepted:
        if accept_window > 0 and waited >= accept_window:
            return "no_accept"
        return "wait"
    if window.opened:
        return "wait"
    if engine == "v2":
        if not window.turn_done:
            return "wait"
        if window.turn_stage in ("failed", "cancelled"):
            return window.turn_stage
        if window.ends:
            return "done"
        if window.started:
            return "wait"
        return "silent" if silent_ok else "nospeech"
    return "done" if window.ends else "wait"


def state(text: str, engine: str, silent_ok: bool, barge: bool = False, waited: float = 0.0,
          accept_window: float = 0.0) -> Dict[str, object]:
    window, now = scan(text, engine, barge)
    live = window.open_live(now)
    if window.opened and live == 0:  # все открытые реплики протухли: говорящих нет
        window.opened.clear()
    verdict = reply_verdict(window, engine, silent_ok, waited, accept_window)
    info = window.turn_info
    return {
        "reason": _reason(window), "llm_calls": info.get("llm_calls", "-"),
        "fetch": 0 if now is None else 1, "verdict": verdict, "accepted": int(window.accepted),
        "turn_done": int(window.turn_done), "started": window.started, "ends": window.ends,
        "open": live, "llm": window.llm, "quiet_for": f"{window.quiet_for(now):.2f}",
        "sig": f"{window.events}.{window.started}.{window.ends}.{int(window.turn_done)}",
    }


def _reason(window: Window) -> str:
    """Причина конца хода из строки ``📊 [turn]`` без пробелов: ``failed:status=failed,llm_calls=3``; ``-`` если ход не кончился."""
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


def _emit(values: Dict[str, object]) -> None:
    for key, value in values.items():
        sys.stdout.write(f"{key}={value}\n")


def _num(value: str) -> float:
    try:
        return float(value)
    except ValueError:
        return 0.0


def main(argv: List[str]) -> int:
    if len(argv) < 2 or argv[1] not in ("state", "quiet", "anchor"):
        sys.stderr.write("usage: e2e_events.py state|quiet|anchor --engine v1|v2 [--silent-ok] [--barge] [--waited S --accept-window S] [--after A --n N]\n")
        return 2
    opts = {"--engine": "v1", "--after": "speech_started", "--n": "1", "--waited": "0", "--accept-window": "0"}
    flags = argv[2:]
    for i, tok in enumerate(flags):
        if tok in opts and i + 1 < len(flags):
            opts[tok] = flags[i + 1]
    engine = "v2" if opts["--engine"] == "v2" else "v1"
    text = sys.stdin.read()
    if argv[1] == "state":
        _emit(state(text, engine, "--silent-ok" in flags, "--barge" in flags, _num(opts["--waited"]),
                    _num(opts["--accept-window"])))
    elif argv[1] == "quiet":
        _emit(quiet(text, engine))
    else:
        try:
            n = max(1, int(opts["--n"]))
        except ValueError:
            n = 1
        _emit(anchor(text, engine, opts["--after"], n))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
