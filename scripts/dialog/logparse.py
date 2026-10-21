"""Разбор лога ``voice-assistant`` (ROS launch) на ходы диалога — общий для turn_metrics и honesty_audit.

ADR-0150 PR-0 (#3684). Только чтение лога: рантайм робота этот модуль не импортирует.

Формат строки лога::

    [dialogue_node-4] [INFO] [1792499438.750290025] [dialogue_node]: 📥 LLM INPUT: '...'

Строки без метки времени (дамп ``🔎 LLM REQUEST``, ``tools(48): ...``) тоже читаются: ts = None.

Ход = строка ``🚀 [turn] calling process_input``. Окно хода начинается с последней фразы ``🎤 STT:`` перед ним
(якорь) и кончается перед якорем следующего хода; без STT окно начинается со строки ``🚀``.
"""

from __future__ import annotations

import ast
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

_TS_LINE = re.compile(r"^\[(?P<node>[^\]]+)\] \[(?P<level>[A-Z]+)\] \[(?P<ts>\d+\.\d+)\] \[[^\]]*\]: (?P<msg>.*)$")
_RAW_LINE = re.compile(r"^\[(?P<node>[^\]]+)\] (?P<msg>.*)$")

STT_MARK = "🎤 STT:"
TURN_START_MARK = "🚀 [turn] calling process_input"
TURN_END_MARK = "✅ [turn] process_input returned"
LLM_REQUEST_MARK = "🔎 LLM REQUEST START"
TTS_MARK = "🔊 TTS: speech_id="
LLM_OUTPUT_MARK = "📤 LLM OUTPUT:"
CODE_PHRASE_MARK = "фраза кода вместо ответа модели:"
#: Записи бэклога, отданные LLM в ход: до ADR-0150 PR-5 — блок «flushed», после — снимок ``recent_unaddressed`` (≤ 1).
BACKLOG_FLUSH_MARKS = ("[backlog] flushed", "[backlog] recent_unaddressed")
BACKLOG_ACC_MARK = "[backlog] accumulated"
#: ADR-0150 PR-6 (#3696): Tier-1 команда кодом (0 LLM) и спорный стоп (В19), решённый Chooser.
TIER1_MARK = "🧩 [tier1] kind="
TIER1_DISPUTED_MARK = "disputed=True"
STOP_CHOOSER_MARK = "🤔 [stop-chooser] verdict="

_TOOLS_VISIBLE = re.compile(r"tools\((\d+)\):")
_TOOL_DONE = re.compile(r"Инструмент (\w+) выполнен успешно")
_TOOL_FAILED = re.compile(r"Инструмент (\w+) завершился с ошибкой")
_TURN_TOOLS = re.compile(r"tools=(\[[^\]]*\])")
_BACKLOG_ENTRIES = re.compile(r"entries=(\d+)")
_TTS_TEXT = re.compile(r"text='(?P<text>.*)$")
_QUOTED_TAIL = re.compile(r"'(?P<text>.*)$")


@dataclass(frozen=True)
class LogLine:
    lineno: int
    node: str
    ts: Optional[float]
    msg: str


@dataclass(frozen=True)
class Speech:
    """Реплика, которую робот произнёс или собирался произнести; ``lines`` — все строки лога с ней."""

    text: str
    lines: Tuple[int, ...]
    kinds: Tuple[str, ...]


@dataclass
class Turn:
    index: int
    anchor_line: int  # строка 🎤 STT (или сама 🚀, если STT нет)
    start_line: int  # строка 🚀
    end_line: int  # последняя строка окна
    stt_ts: Optional[float] = None
    start_ts: Optional[float] = None
    stt_text: str = ""
    llm_calls: int = 0
    tools_visible: Optional[int] = None
    tools_requested: List[str] = field(default_factory=list)  # из ✅ [turn] ... tools=[...]
    tools_executed: List[str] = field(default_factory=list)  # «Инструмент X выполнен успешно» в окне
    tools_failed: List[str] = field(default_factory=list)
    first_tts_ts: Optional[float] = None
    first_tts_line: Optional[int] = None
    tts_texts: List[Tuple[int, str]] = field(default_factory=list)
    backlog_entries: int = 0
    speeches: List[Speech] = field(default_factory=list)
    retry: bool = False  # ход без своей фразы STT (синтетический ретрай): STT-якорь унаследован от предыдущего

    @property
    def stt_to_first_tts_ms(self) -> Optional[float]:
        if self.stt_ts is None or self.first_tts_ts is None:
            return None
        return (self.first_tts_ts - self.stt_ts) * 1000.0


def read_lines(path: Path) -> List[LogLine]:
    out: List[LogLine] = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for lineno, raw in enumerate(fh, start=1):
            raw = raw.rstrip("\n")
            m = _TS_LINE.match(raw)
            if m:
                out.append(LogLine(lineno, m["node"], float(m["ts"]), m["msg"]))
                continue
            m = _RAW_LINE.match(raw)
            out.append(LogLine(lineno, m["node"] if m else "", None, m["msg"] if m else raw))
    return out


def _speech_text(line: LogLine) -> Optional[Tuple[str, str]]:
    """``(вид, текст)`` для строки с речью, иначе ``None``. Хвостовую кавычку убираем."""
    msg = line.msg
    if TTS_MARK in msg:
        m = _TTS_TEXT.search(msg)
        return ("tts", m["text"].rstrip("'")) if m else None
    if msg.startswith(LLM_OUTPUT_MARK):
        m = _QUOTED_TAIL.search(msg)
        return ("llm_output", m["text"].rstrip("'")) if m else None
    if CODE_PHRASE_MARK in msg:
        m = _QUOTED_TAIL.search(msg.split(CODE_PHRASE_MARK, 1)[1])
        return ("code_phrase", m["text"].rstrip("'")) if m else None
    return None


def _norm(text: str) -> str:
    return " ".join(text.lower().replace("ё", "е").split())[:40]


def _group_speeches(items: Sequence[Tuple[int, str, str]]) -> List[Speech]:
    """Склеивает строки с одной и той же репликой (LLM OUTPUT → TTS) по первым 40 символам."""
    order: List[str] = []
    grouped: Dict[str, List[Tuple[int, str, str]]] = {}
    for lineno, kind, text in items:
        key = _norm(text)
        if key not in grouped:
            grouped[key] = []
            order.append(key)
        grouped[key].append((lineno, kind, text))
    result = []
    for key in order:
        rows = grouped[key]
        full = max((r[2] for r in rows), key=len)
        result.append(Speech(full, tuple(r[0] for r in rows), tuple(r[1] for r in rows)))
    return result


def _fill_window(turn: Turn, window: Sequence[LogLine]) -> None:
    speech_items: List[Tuple[int, str, str]] = []
    in_turn = False
    for line in window:
        msg = line.msg
        if line.lineno == turn.start_line:
            in_turn = True
        if in_turn and LLM_REQUEST_MARK in msg:
            turn.llm_calls += 1
        if in_turn:
            m = _TOOLS_VISIBLE.search(msg)
            if m:
                turn.tools_visible = max(turn.tools_visible or 0, int(m[1]))
        if TURN_END_MARK in msg:
            in_turn = False  # запросы к LLM после «returned» — уже следующий ход (ретрай), не этот
            m = _TURN_TOOLS.search(msg)
            if m:
                turn.tools_requested = [str(t) for t in ast.literal_eval(m[1])]
        m = _TOOL_DONE.search(msg)
        if m:
            turn.tools_executed.append(m[1])
        m = _TOOL_FAILED.search(msg)
        if m:
            turn.tools_failed.append(m[1])
        if any(mark in msg for mark in BACKLOG_FLUSH_MARKS):
            m = _BACKLOG_ENTRIES.search(msg)
            turn.backlog_entries += int(m[1]) if m else 0
        speech = _speech_text(line)
        if speech:
            speech_items.append((line.lineno, speech[0], speech[1]))
            if speech[0] == "tts":
                turn.tts_texts.append((line.lineno, speech[1]))
                if turn.first_tts_ts is None and line.ts is not None:
                    turn.first_tts_ts, turn.first_tts_line = line.ts, line.lineno
    turn.speeches = _group_speeches(speech_items)


def split_turns(lines: Sequence[LogLine]) -> List[Turn]:
    """Режет лог на ходы. Окно хода: от якоря (STT) до якоря следующего хода."""
    starts = [i for i, ln in enumerate(lines) if TURN_START_MARK in ln.msg]
    anchors: List[int] = []
    prev = -1
    for s in starts:
        a = s
        for j in range(s - 1, prev, -1):
            if STT_MARK in lines[j].msg:
                a = j
                break
        anchors.append(a)
        prev = s
    turns: List[Turn] = []
    for n, (s, a) in enumerate(zip(starts, anchors)):
        end = anchors[n + 1] - 1 if n + 1 < len(anchors) else len(lines) - 1
        stt = lines[a] if a != s else None
        turn = Turn(
            index=n + 1,
            anchor_line=lines[a].lineno,
            start_line=lines[s].lineno,
            end_line=lines[end].lineno,
            stt_ts=stt.ts if stt else None,
            start_ts=lines[s].ts,
            stt_text=stt.msg.split(STT_MARK, 1)[1].strip() if stt else "",
        )
        if stt is None and turns:
            turn.retry, turn.stt_ts, turn.stt_text = True, turns[-1].stt_ts, turns[-1].stt_text
        _fill_window(turn, lines[a : end + 1])
        turns.append(turn)
    return turns


def load_turns(path: Path) -> List[Turn]:
    return split_turns(read_lines(path))


def count_marks(lines: Sequence[LogLine], mark: str) -> int:
    return sum(1 for ln in lines if mark in ln.msg)
