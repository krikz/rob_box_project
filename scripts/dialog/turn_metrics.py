#!/usr/bin/env python3
"""Метрики хода диалога по логу ``voice-assistant`` (ADR-0150 §14: A1, A2, A5, A29, A31; карточка #3684).

    python scripts/dialog/turn_metrics.py <log> [--per-turn]

Считает на каждый адресованный ход (``🚀 [turn]``): вызовы LLM, STT -> первая реплика TTS (мс), число схем тулов
в вызове (``tools(N)``), вопросы о личности в речи, записи бэклога (``entries=N`` у ``[backlog] flushed`` / ``[backlog] recent_unaddressed``).
Печатает p50/p95/max и markdown-таблицу A1/A2/A5. Рантайм робота скрипт не трогает и не импортирует.

Чего в логе НЕТ: число токенов (дамп ``🔎 LLM REQUEST`` усечён), поэтому A5 даётся прокси ``tools_visible``
и честной пометкой «не измеримо по логу».
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import List, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))
import logparse  # noqa: E402

# Вопрос о личности в озвучке (A29, ADR §7.5). Только для аудита логов, не для рантайма.
IDENTITY_QUESTION = re.compile(r"как тебя зовут|это ты\?", re.IGNORECASE)


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    """Линейная интерполяция между соседними рангами; пустой ряд -> None."""
    data = sorted(values)
    if not data:
        return None
    if len(data) == 1:
        return float(data[0])
    pos = (len(data) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(data) - 1)
    return data[lo] + (data[hi] - data[lo]) * (pos - lo)


def identity_questions(turn: logparse.Turn) -> List[int]:
    return [lineno for lineno, text in turn.tts_texts if IDENTITY_QUESTION.search(text)]


def _fmt(value: Optional[float], digits: int = 0) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def _stat_row(label: str, values: Sequence[float], digits: int = 0) -> str:
    n = len(values)
    return (
        f"| {label} | {n} | {_fmt(percentile(values, 0.5), digits)} | {_fmt(percentile(values, 0.95), digits)} "
        f"| {_fmt(max(values) if values else None, digits)} |"
    )


def _tier1_line(lines: Sequence[logparse.LogLine]) -> str:
    """A1 Tier-1 (#3696): команд кодом, из них спорных стопов (В19) и как их решил Chooser."""
    tier1 = [ln.msg for ln in lines if logparse.TIER1_MARK in ln.msg]
    disputed = sum(logparse.TIER1_DISPUTED_MARK in msg for msg in tier1)
    verdicts = [ln.msg.split(logparse.STOP_CHOOSER_MARK, 1)[1].split()[0] for ln in lines
                if logparse.STOP_CHOOSER_MARK in ln.msg]
    share = f"{100.0 * disputed / len(tier1):.0f} %" if tier1 else "n/a"
    return (f"- `tier1` (команды кодом, 0 LLM): {len(tier1)}; спорных стопов (В19): {disputed} ({share}); "
            f"Chooser: stop {verdicts.count('stop')}, part_of_command {verdicts.count('part_of_command')}.")


def render(turns: Sequence[logparse.Turn], lines: Sequence[logparse.LogLine]) -> str:
    llm = [float(t.llm_calls) for t in turns]
    lat = [t.stt_to_first_tts_ms for t in turns if t.stt_to_first_tts_ms is not None]
    vis = [float(t.tools_visible) for t in turns if t.tools_visible is not None]
    idq = [ln for t in turns for ln in identity_questions(t)]
    flushed = sum(t.backlog_entries for t in turns)
    acc = logparse.count_marks(lines, logparse.BACKLOG_ACC_MARK)
    out = [
        f"Строк в логе: {len(lines)}; адресованных ходов (`🚀 [turn]`): {len(turns)}; "
        f"фраз `🎤 STT`: {logparse.count_marks(lines, logparse.STT_MARK)}.",
        "",
        "| Метрика | ходов | p50 | p95 | max |",
        "|---|---|---|---|---|",
        _stat_row("A1 `llm_calls` на ход (вызовов LLM)", llm),
        _stat_row("A2 `stt_to_first_tts_ms` (мс, ходы с озвучкой)", lat),
        _stat_row("A5-прокси `tools_visible` (схем тулов в вызове; токены в логе не пишутся)", vis),
        "",
        f"- `identity_questions` («Как тебя зовут?»/«это ты?» в `🔊 TTS`): {len(idq)} (строки лога: "
        f"{', '.join(map(str, idq)) or '-'}).",
        f"- `backlog_entries`: отдано LLM {flushed} (`[backlog] flushed|recent_unaddressed … entries=N`), накоплено без вейка {acc} "
        f"(`[backlog] accumulated`).",
        _tier1_line(lines),
    ]
    return "\n".join(out)


def render_per_turn(turns: Sequence[logparse.Turn]) -> str:
    out = [
        "| # | STT-строка | 🚀-строка | вход | llm_calls | stt→TTS мс | tools_visible | тулы хода | вопр. личности | бэклог |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for t in turns:
        text = ("(ретрай) " if t.retry else "") + (t.stt_text or "-").replace("|", "/")[:40]
        out.append(
            f"| {t.index} | {t.anchor_line} | {t.start_line} | {text} | {t.llm_calls} | "
            f"{_fmt(t.stt_to_first_tts_ms)} | {t.tools_visible if t.tools_visible is not None else 'n/a'} | "
            f"{', '.join(t.tools_requested) or '-'} | {len(identity_questions(t))} | {t.backlog_entries} |"
        )
    return "\n".join(out)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log", type=Path)
    ap.add_argument("--per-turn", action="store_true", help="добавить таблицу по каждому ходу")
    args = ap.parse_args(argv)
    lines = logparse.read_lines(args.log)
    turns = logparse.split_turns(lines)
    print(render(turns, lines))
    if args.per_turn:
        print()
        print(render_per_turn(turns))
    return 0


if __name__ == "__main__":
    sys.exit(main())
