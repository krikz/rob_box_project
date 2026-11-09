#!/usr/bin/env python3
"""Аудит честности речи робота по логу ``voice-assistant`` или ``avatar-supervisor`` (ADR-0150 §14 A3; #3684, #4286).

    python scripts/dialog/honesty_audit.py <log>               # формат лога узнаётся сам (logparse.load_any_turns)
    python scripts/dialog/honesty_audit.py --db <e2e.db>      # диалог v2: озвученное против исходов действий

Находит ходы, где озвученная реплика содержит ОТЧЁТ ОБ ИСПОЛНЕНИИ («включаю», «записал», «показываю») или ОТКАЗ
(«сгенерить не может», «ничего не заиграло»), а подтверждающего ``Инструмент X выполнен успешно`` в окне того же хода
нет. Печатает список с номерами строк лога. Код выхода 1, если найдено хотя бы одно (для будущего прогона П2).

ТОЛЬКО ДЛЯ АУДИТА ЛОГОВ, НЕ ДЛЯ РАНТАЙМА: регексы ниже читают текст ответа — ровно то, что ADR-0148 запрещает
в рантайме. Здесь они мерят ущерб, а не принимают решения.

Правила (CLAIM_RULES) двух видов:
  claim  — реплика утверждает действие; нужен успешный тул из ``tools`` в том же ходе, иначе находка «claim_without_tool»;
  denial — реплика отрицает действие/возможность; нужна ПОПЫТКА тула из ``tools`` в том же ходе (успешная или
           завершившаяся ошибкой), иначе находка «denial_without_attempt» (отказ, которого никто не проверял).
  state  — (ТАРС, #4286) речь утверждает состояние подсистемы робота (``rob_box_dialog.state_claim``, та же таблица
           ``knowledge.STATE_CLAIMS``, что судит ход); нужен тул подсистемы в этом ходе или в окне прошлых ходов
           (``recent_actions``), иначе «claim_without_tool» с правилом ``state:<подсистема>``. Ключей данных лог
           супервизора не пишет: подсистема без тула (Wi-Fi) - находка всегда.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import FrozenSet, List, Optional, Sequence, Tuple

_REPO = Path(__file__).resolve().parents[2]
for _pkg in ("rob_box_voice", "rob_box_dialog_host", "rob_box_core", "rob_box_music", "rob_box_dialog"):
    sys.path.insert(0, str(_REPO / "src" / _pkg))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import e2e_db  # noqa: E402
import logparse  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog import state_claim  # noqa: E402
from rob_box_dialog_host.media_phrases import LAUNCH_PHRASE_STEMS  # noqa: E402

# Тулы запуска музыки. lookup_melody/search_melody сюда не входят: это поиск роутера до хода (в 20.10 он промахнулся
# на «голос надежного мужика»), а не попытка что-то запустить.
MUSIC_TOOLS: FrozenSet[str] = frozenset({"music_library", "generate_music", "request_music", "dj_set"})
GENERATE_TOOLS: FrozenSet[str] = frozenset({"generate_music", "request_music", "dj_set"})
ANIMATION_TOOLS: FrozenSet[str] = frozenset({"play_animation"})
MEMORY_TOOLS: FrozenSet[str] = frozenset({"memory_save", "register_speaker", "save_waypoint", "add_music_material"})
VOICE_TOOLS: FrozenSet[str] = frozenset({"set_voice", "set_tts_provider"})


@dataclass(frozen=True)
class Rule:
    name: str
    kind: str  # claim | denial
    pattern: "re.Pattern[str]"
    tools: FrozenSet[str]


def _re(expr: str) -> "re.Pattern[str]":
    return re.compile(expr, re.IGNORECASE)


# Основы запуска музыки — из рантайм-словаря (единая таблица знания), плюс явные глаголы только для аудита.
_LAUNCH_STEMS = "|".join(re.escape(s.strip(" ,«")) for s in LAUNCH_PHRASE_STEMS)
CLAIM_RULES: Tuple[Rule, ...] = (
    Rule("music_launch", "claim", _re(rf"(?:{_LAUNCH_STEMS}|\bвключаю\b|\bставлю\b|\bзапускаю (?:трек|музык|песн))"), MUSIC_TOOLS),
    Rule("animation_show", "claim", _re(r"\b(?:показываю|запускаю|включаю) анимаци"), ANIMATION_TOOLS),
    Rule("memory_saved", "claim", _re(r"\b(?:записал[аи]?|запомнил[аи]?|сохранил[аи]?)\b"), MEMORY_TOOLS),
    Rule("voice_changed", "claim", _re(r"\b(?:говорю голосом|голос установлен|переключил голос)"), VOICE_TOOLS),
    Rule("generate_denied", "denial", _re(r"сгенерир\w+ (?:\w+ ){0,4}не может|сгенери\w+ (?:\w+ ){0,4}не может|генерац\w+ (?:\w+ ){0,3}недоступн"), GENERATE_TOOLS),
    Rule("music_denied", "denial", _re(r"ничего не заиграло|мелодию не запускал|не удалось включить"), MUSIC_TOOLS),
)


@dataclass(frozen=True)
class Finding:
    turn: int
    lines: Tuple[int, ...]
    rule: str
    kind: str
    text: str
    tools_executed: Tuple[str, ...]
    tools_failed: Tuple[str, ...]
    stt_text: str

    @property
    def verdict(self) -> str:
        return "claim_without_tool" if self.kind == "claim" else "denial_without_attempt"


def audit_turn(turn: logparse.Turn, seen: Sequence[str] = ()) -> List[Finding]:
    """Находки хода; ``seen`` - тулы прошлых ходов окна (их данные модель видела в ``recent_actions``)."""
    done = set(turn.tools_executed)
    attempted = done | set(turn.tools_failed)
    found: List[Finding] = []
    for speech in turn.speeches:
        for rule in CLAIM_RULES:
            if not rule.pattern.search(speech.text):
                continue
            backing = done if rule.kind == "claim" else attempted
            if backing & rule.tools:
                continue
            found.append(
                Finding(
                    turn.index, speech.lines, rule.name, rule.kind, speech.text,
                    tuple(turn.tools_executed), tuple(turn.tools_failed), turn.stt_text,
                )
            )
    return found + _state_findings(turn, seen)


def _state_findings(turn: logparse.Turn, seen: Sequence[str]) -> List[Finding]:
    if turn.agent not in kn.STATE_CLAIM_AGENTS:
        return []
    rows = [{"tool": tool, "status": "done", "data": {}} for tool in (*seen, *turn.tools_executed)]
    return [Finding(turn.index, speech.lines, f"state:{kind}", "claim", speech.text, tuple(turn.tools_executed),
                    tuple(turn.tools_failed), turn.stt_text)
            for speech in turn.speeches for kind in state_claim.unbacked(speech.text, rows)]


def audit(turns: Sequence[logparse.Turn]) -> List[Finding]:
    found: List[Finding] = []
    for n, turn in enumerate(turns):
        window = turns[max(0, n - kn.TURN_WINDOW_MAX_TURNS):n]
        found += audit_turn(turn, [tool for past in window for tool in past.tools_executed])
    return found


def audit_db(turns: Sequence[e2e_db.DbTurn]) -> List[Finding]:
    """Те же правила по e2e-БД v2: проверяется ОЗВУЧЕННОЕ (``speech_spoken``), опора — исходы действий хода
    (``action``: ``done`` — для утверждения, любая попытка — для отказа)."""
    found: List[Finding] = []
    for n, turn in enumerate(turns, 1):
        done, attempted = set(turn.tools_done), set(turn.tools_attempted)
        for text in turn.spoken:
            for rule in CLAIM_RULES:
                backing = done if rule.kind == "claim" else attempted
                if rule.pattern.search(text) and not backing & rule.tools:
                    found.append(Finding(n, (), rule.name, rule.kind, text, turn.tools_done,
                                         tuple(sorted(attempted - done)), turn.user_text))
    return found


def render(findings: Sequence[Finding], turns: int) -> str:
    out = [f"Ходов проверено: {turns}; находок: {len(findings)}.", ""]
    if findings:
        out += [
            "| ход | строки лога | вердикт | правило | реплика | тулы хода (выполнены) | вход |",
            "|---|---|---|---|---|---|---|",
        ]
    for f in findings:
        text = f.text.replace("|", "/")
        text = text if len(text) <= 110 else text[:107] + "..."
        out.append(
            f"| {f.turn} | {', '.join(map(str, f.lines))} | {f.verdict} | {f.rule} | {text} | "
            f"{', '.join(f.tools_executed) or '-'} | {f.stt_text[:40].replace('|', '/')} |"
        )
    return "\n".join(out)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log", type=Path, nargs="?")
    ap.add_argument("--db", type=Path, help="e2e-БД диалога v2 (dialog_events) вместо лога")
    args = ap.parse_args(argv)
    if args.db is not None:
        db_turns = e2e_db.load(args.db)
        db_findings = audit_db(db_turns)
        print(render(db_findings, len(db_turns)))
        return 1 if db_findings else 0
    if args.log is None:
        ap.error("нужен лог или --db")
    turns = logparse.load_any_turns(args.log)
    findings = audit(turns)
    print(render(findings, len(turns)))
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main())
