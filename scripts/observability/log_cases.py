#!/usr/bin/env python3
"""Кейсы смотрителя логов (#4033, шаг 2а): серии сбоев и странные распознавания, дедуп по сигнатуре.

Поверх детекторов ``scripts/dialog/turn_failures.py`` (их не трогает). Из строк берёт только коды: причину
серии, слова команды, которых не хватило или которые лишние, имя тула вне среза. Полную фразу в кейс не
кладёт. Сигнатура — sha1 от (вид, нормализованный код, источник): одинаковые кейсы схлопываются в частоту
и первое/последнее время. Длительность серии в сигнатуру не входит (две серии одной причины — один кейс),
в журнале остаётся самая длинная.

Фраза целиком — только в приватный каталог и только вместе с пометкой харнесса ``🧪 [e2e]`` в том же окне.
Реплики людей мастерской в репо не попадают: журнал пишется вне репо, в сводке — счётчики и коды.
"""

from __future__ import annotations

import hashlib
import json
import re
import sys
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

REPO = Path(__file__).resolve().parents[2]
for _pkg in ("rob_box_dialog",):
    _src = str(REPO / "src" / _pkg)
    if _src not in sys.path:
        sys.path.insert(0, _src)
sys.path.insert(0, str(REPO / "scripts" / "dialog"))
import turn_failures as tf  # noqa: E402

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.words import name_tokens  # noqa: E402

SCHEMA = 1
CASE_SERIES, CASE_ODD, CASE_UNCOVERED = "series", "odd_stt", "uncovered"
SOURCE_LOKI = "loki:voice-assistant"
HARNESS_MARK = "🧪 [e2e]"
#: Короткая фраза: длиннее — болтовня, где слово команды попалось по пути, а не искажение команды.
MAX_TOKENS = 6
#: Лишних слов больше двух — уже не «недослышал команду», а другая речь.
MAX_EXTRA = 2
#: Близким считается слово команды от 4 букв и ровно на одну правку (замена, вставка или пропуск).
NEAR_MIN = 4

_QUOTED = re.compile(r"'([^']*)'")
_ACCEPTED = "-> accepted"
_WIDENED = re.compile(r"tool_not_available → в запрос добавлены тулы \[([^\]]*)\]")
_TOOL_NAME = re.compile(r"'([A-Za-z_][A-Za-z0-9_]*)'")

_FILLER: Set[str] = {
    w.lower().replace("ё", "е") for w in (*kn.GRAMMAR_FILLER, *kn.STOP_LEAD_WORDS, *kn.ADDRESS_FILLER_WORDS)
}
_WAKE: Set[str] = {w.lower().replace("ё", "е") for w in (*kn.WAKE_SLIP_TARGETS, *kn.MUSIC_ADDRESS_WORDS)}


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _signature(kind: str, code: str, source: str) -> str:
    raw = f"{kind}|{code}|{source}".encode("utf-8")
    return hashlib.sha1(raw).hexdigest()[:12]  # noqa: S324 - дедуп кейсов, не секрет


def command_lexicon() -> Tuple[str, ...]:
    """Ключевые слова команд из таблиц knowledge: точные формы и основы, без вейка и служебных."""
    forms = (
        *kn.STOP_WORDS,
        *kn.MUSIC_OFF_WORDS,
        *kn.MUSIC_MUTE_WORDS,
        *kn.YES_WORDS,
        *kn.NO_WORDS,
        *kn.MUSIC_AMBIGUOUS_WORDS,
        *kn.PLAY_VERBS,
    )
    stems = (
        *kn.MUSIC_STEMS,
        *kn.MUSIC_GROUP_EXTRA_STEMS,
        *kn.SILENCE_WORDS,
        *kn.UNSILENCE_WORDS,
        *(stem for stems in kn.VOLUME_STEMS.values() for stem in stems),
        *(word for words in kn.VOLUME_LEVEL_WORDS.values() for word in words),
        *kn.VOLUME_NOUN_STEMS,
        *kn.SHOW_QUEUE_VERB_STEMS,
        *kn.MOTION_VERB_STEMS,
        *kn.MOTION_TURN_STEMS,
    )
    seen: Dict[str, None] = {}
    for word in (*forms, *stems):
        norm = word.lower().replace("ё", "е")
        if norm and norm not in _FILLER and norm not in _WAKE and len(norm) >= 3:
            seen.setdefault(norm, None)
    return tuple(seen)


_LEXICON: Optional[Tuple[str, ...]] = None


def lexicon() -> Tuple[str, ...]:
    global _LEXICON
    if _LEXICON is None:
        _LEXICON = command_lexicon()
    return _LEXICON


def _one_off(left: str, right: str) -> bool:
    """Ровно одна правка: замена буквы, лишняя или пропущенная. Короткие слова не сравнивает."""
    if abs(len(left) - len(right)) > 1 or min(len(left), len(right)) < NEAR_MIN:
        return False
    if len(left) == len(right):
        return sum(a != b for a, b in zip(left, right)) == 1
    short, long = (left, right) if len(left) < len(right) else (right, left)
    skipped = False
    at = 0
    for ch in short:
        if at < len(long) and long[at] == ch:
            at += 1
            continue
        if skipped:
            return False
        skipped = True
        at += 1
        if at > len(long) or long[at - 1] == ch:
            return False
    return True


def _hits(tokens: Sequence[str]) -> Tuple[Tuple[str, ...], Set[str]]:
    """Слова команды, услышанные точно, и сами токены, которые их дали."""
    found: Dict[str, None] = {}
    matched: Set[str] = set()
    for token in tokens:
        for key in lexicon():
            if token == key or (len(key) >= NEAR_MIN and token.startswith(key) and len(token) - len(key) <= 3):
                found.setdefault(key, None)
                matched.add(token)
                break
    return tuple(sorted(found)), matched


def _near(tokens: Sequence[str], heard: Sequence[str]) -> Tuple[Tuple[str, str], ...]:
    """Недослышанные слова команды: токен на одну правку от словаря, сам в словарь не попал."""
    heard_set = set(heard)
    pairs: Dict[Tuple[str, str], None] = {}
    for token in tokens:
        if token in heard_set or token in _FILLER or token in _WAKE or len(token) < NEAR_MIN:
            continue
        for key in lexicon():
            if key not in heard_set and _one_off(token, key):
                pairs.setdefault((token, key), None)
    return tuple(sorted(pairs))


def _odd_code(tokens: Sequence[str]) -> Optional[str]:
    """Код странного распознавания или ``None``. Чистое слово команды и длинная болтовня — не кейс."""
    if not tokens or len(tokens) > MAX_TOKENS:
        return None
    heard, matched = _hits(tokens)
    near = _near(tokens, heard)
    extra = tuple(sorted(t for t in tokens if t not in matched and t not in _FILLER and t not in _WAKE))
    if not heard and not near:
        return None
    if not extra and not near:
        return None
    if len(extra) > MAX_EXTRA:
        return None
    missing = tuple(sorted({key for _, key in near}))
    extra_n = len(extra) - len(near)
    return f"missing={','.join(missing) or '-'};extra={extra_n};heard={','.join(heard) or '-'}"


def _widened_tools(line: str) -> Tuple[str, ...]:
    found = _WIDENED.search(line)
    if not found:
        return ()
    return tuple(sorted(set(_TOOL_NAME.findall(found.group(1)))))


def _accepted_text(line: str) -> str:
    """Текст принятой фразы из ``-> accepted '...'``; пусто, если кавычек нет."""
    if _ACCEPTED not in line:
        return ""
    found = _QUOTED.search(line.split(_ACCEPTED, 1)[1])
    return found.group(1) if found else ""


@dataclass
class Case:
    """Кейс после дедупа: вид, код без текста, источник, частота, первое и последнее время."""

    kind: str
    code: str
    source: str
    count: int = 0
    first_ts: float = 0.0
    last_ts: float = 0.0
    duration_s: int = 0
    signature: str = ""
    samples: List[str] = field(default_factory=list)

    def add(self, ts: float, sample: str = "", duration: int = 0) -> None:
        if not self.count:
            self.first_ts = ts
        self.count += 1
        self.last_ts = ts
        self.duration_s = max(self.duration_s, duration)
        if sample and sample not in self.samples:
            self.samples.append(sample)

    def as_dict(self, with_samples: bool) -> Dict[str, object]:
        out: Dict[str, object] = {
            "signature": self.signature,
            "kind": self.kind,
            "code": self.code,
            "source": self.source,
            "count": self.count,
            "first_utc": _iso(self.first_ts),
            "last_utc": _iso(self.last_ts),
        }
        if self.duration_s:
            out["duration_s"] = self.duration_s
        if with_samples and self.samples:
            out["samples"] = list(self.samples)
        return out


def _series_cases(verdicts: Sequence[tf.Verdict]) -> List[Tuple[str, str, float, int]]:
    """Серия сбоя: вид, причина без длительности (она не в сигнатуре), время конца, длина серии."""
    out: List[Tuple[str, str, float, int]] = []
    for verdict in verdicts:
        if verdict.kind == tf.DEAF:
            duration = max(0, int(round(verdict.ts - verdict.since))) if verdict.since else 0
            out.append((CASE_SERIES, f"deaf:{verdict.reason}", verdict.ts, duration))
        elif verdict.kind == tf.NO_SPEECH:
            out.append((CASE_SERIES, f"no_speech:{verdict.reason}", verdict.ts, 0))
    return out


def _line_cases(lines: Iterable[str]) -> Tuple[List[Tuple[str, str, float, str]], bool, bool]:
    """Кейсы строк и два флага: была ли пометка харнесса, была ли хоть одна строка стека."""
    out: List[Tuple[str, str, float, str]] = []
    harness = False
    saw = False
    for line in lines:
        if HARNESS_MARK in line:
            harness = True
        stamp = tf._TS.search(line)  # noqa: SLF001 - та же метка времени, что у детекторов
        if not stamp:
            continue
        saw = saw or "[stt_attempt]" in line or "[dialog_node]" in line
        ts = float(stamp.group(1))
        tools = _widened_tools(line)
        if tools:
            out.append((CASE_UNCOVERED, f"tool_not_available:{','.join(tools)}", ts, ""))
        text = _accepted_text(line)
        if not text:
            continue
        code = _odd_code(name_tokens(text))
        if code is not None:
            out.append((CASE_ODD, code, ts, text))
    return out, harness, saw


def build_cases(lines: Iterable[str]) -> Tuple[List[Case], bool]:
    """Строки окна → кейсы по сигнатуре и признак окна харнесса."""
    kept = list(lines)
    grouped: Dict[str, Case] = {}
    for kind, code, ts, duration in _series_cases(tf.detect(tf.events_from_log(kept))):
        _add(grouped, kind, code, ts, "", duration)
    line_cases, harness, _saw = _line_cases(kept)
    for kind, code, ts, sample in line_cases:
        _add(grouped, kind, code, ts, sample, 0)
    cases = sorted(grouped.values(), key=lambda c: (-c.count, c.kind, c.code))
    return cases, harness


def _add(grouped: Dict[str, Case], kind: str, code: str, ts: float, sample: str, duration: int) -> None:
    signature = _signature(kind, code, SOURCE_LOKI)
    case = grouped.get(signature)
    if case is None:
        case = Case(kind, code, SOURCE_LOKI, signature=signature)
        grouped[signature] = case
    case.add(ts, sample, duration)


def cases_summary(lines: Iterable[str], start_s: float, end_s: float, min_count: int = 1) -> Dict[str, object]:
    """Сводка кейсов: только счётчики и коды. Нет строк стека → ``status=no_events``."""
    kept = list(lines)
    cases, harness = build_cases(kept)
    _line, _mark, saw = _line_cases(kept)
    shown = [c for c in cases if c.count >= min_count]
    by_kind = Counter(c.kind for c in shown)
    return {
        "schema": SCHEMA,
        "mode": "cases",
        "window": {"start_utc": _iso(start_s), "end_utc": _iso(end_s)},
        "status": "ok" if saw or shown else "no_events",
        "harness_window": harness,
        "cases": {
            "total": len(shown),
            "by_kind": dict(sorted(by_kind.items())),
            "items": [c.as_dict(False) for c in shown],
        },
    }


def render_cases(summary: Dict[str, object]) -> str:
    """Короткий текст кейсов. Пустое окно — «событий нет», а не «0 кейсов»."""
    win = summary["window"]
    head = f"Смотритель логов, кейсы, {win['start_utc']} - {win['end_utc']} UTC"
    if summary["status"] == "no_events":
        return f"{head}\nСобытий голосового стека за окно нет: кейсов не считаю."
    cases = summary["cases"]
    out = [head, f"Кейсов {cases['total']} ({_counts(cases['by_kind'])})."]
    for item in cases["items"]:
        span = f" duration_s={item['duration_s']}" if item.get("duration_s") else ""
        out.append(
            f"- {item['signature']} {item['kind']} x{item['count']} {item['code']}{span} "
            f"{item['first_utc']}..{item['last_utc']}"
        )
    if not cases["items"]:
        out.append("Кейсов по порогу нет.")
    return "\n".join(out)


def _counts(by_kind: Dict[str, int]) -> str:
    return ", ".join(f"{k}={v}" for k, v in by_kind.items()) or "нет"


def write_journal(cases: Sequence[Case], journal_dir: Path, harness: bool, private_dir: Optional[Path]) -> Path:
    """Журнал вне репо: markdown и JSON, только счётчики и коды. Фразы — в ``private_dir`` и только из окна харнесса."""
    journal_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(tz=timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    payload = {"schema": SCHEMA, "written_utc": stamp, "cases": [c.as_dict(False) for c in cases]}
    path = journal_dir / f"cases-{stamp}.json"
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = [f"# Кейсы смотрителя {stamp}", ""]
    for case in cases:
        lines.append(f"- `{case.signature}` {case.kind} x{case.count} `{case.code}` {_iso(case.first_ts)}")
    (journal_dir / f"cases-{stamp}.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if private_dir is not None and harness:
        _write_private(cases, private_dir, stamp)
    return path


def _write_private(cases: Sequence[Case], private_dir: Path, stamp: str) -> None:
    private_dir.mkdir(parents=True, exist_ok=True)
    rows = [{"signature": c.signature, "samples": c.samples} for c in cases if c.samples]
    (private_dir / f"samples-{stamp}.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
