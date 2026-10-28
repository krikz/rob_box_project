#!/usr/bin/env python3
"""Корпус замера понимания, часть 3: реальные искажения STT фраз марафона (карточка #4163, этап 2).

    python scripts/dialog/understanding_corpus/build_stt.py --artifacts <tests/e2e/_artifacts>   # собрать
    python scripts/dialog/understanding_corpus/build_stt.py --check [--artifacts …]              # сверить

Только то, что STT робота реально выдал на фразы сценариев марафона; синтетических искажений нет. Источники по порядку:

1. логи актов живых марафонов ``<artifacts>/night_*/act<акт>_<run_id>.log``: харнесс пишет на каждую фразу строку
   ``>>> TRANSCRIPT[<метка>]: ожидалось «<текст сценария>», распознано «<текст STT>»``;
2. тела карточек GitHub с сырой строкой STT робота (:data:`ISSUE_ROWS` - строка, номер карточки, шаг).

Строка идёт в корпус, если: метка - шаг части 1, «ожидалось» совпадает с ТЕКУЩИМ текстом шага (по словам: старые
редакции фраз не годятся), шаг оцениваемый, слова распознанного отличаются от исходника. Отсеиваются (в файл попадает
только число и причина, без текста - там может быть речь живого человека из мастерской или музыка):
мало общего с исходником (похожесть строк < :data:`MIN_SIMILARITY`) и лишние слова по краям (≥ :data:`EDGE_WORDS`
вставленных слов в начале или в конце). У шага альтернативы - по убыванию числа прогонов, где искажение встретилось.
"""

from __future__ import annotations

import argparse
import difflib
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

import alternatives as alt  # noqa: E402
import build_marathon  # noqa: E402

PART = 3
OUT = alt.part_path(PART)
DEFAULT_ARTIFACTS = build_marathon.dm.REPO / "tests" / "e2e" / "_artifacts"
TRANSCRIPT = re.compile(r">>> TRANSCRIPT\[([^\]]+)\]: ожидалось «([^»]*)», распознано «([^»]*)»")
LOG_NAME = re.compile(r"act(\w+?)_(\d+)\.log$")
#: Похожесть распознанного на исходник (``difflib``, по нормализованным словам): ниже - скорее чужая речь или музыка.
MIN_SIMILARITY = 0.5
#: Столько и больше слов, вставленных в начале или в конце, - могла попасть речь живого человека: строку не берём.
EDGE_WORDS = 2

#: Сырые строки STT робота из тел карточек: ``(карточка, id шага, распознано, откуда)``. Только строки лога
#: (``🎤 STT: …``), которые карточка сама относит к шагу марафона; текст шага в ту ночь сверен с ``git log -S``.
ISSUE_ROWS: Tuple[Tuple[int, str, str, str], ...] = (
    (4175, "3/n301_wake_open", "мы тут сейчас немного большую мимо ты просто слушай не вылезай",
     "марафон-2 28.10, docker logs voice-assistant 15:58:01.884, вторая попытка n301"),
    (2779, "2/n209_recall_boris", "Робота, про меня что помнишь?",
     "лог контейнера, n209 третья попытка (текст шага не менялся с 7a8292b45, 10.09)"),
)
#: Строки карточек, которые НЕ взяты: ``(карточка, id шага, причина)``.
ISSUE_SKIPPED: Tuple[Tuple[int, str, str], ...] = (
    (4028, "8/n808_repeat_last", "в цитате нет вейка, а ход на роботе был адресован: по цитате не понять, срезал "
                                 "вейк STT или автор карточки"),
    (4127, "6/n604_dj_rock", "в карточке только фрагмент («в сторону рака … подруг»), целой строки нет; целая строка "
                             "этого шага есть в логах актов"),
    (4050, "7/n704_music_library", "строка с заглавной и пунктуацией - цитата старой редакции сценария, не вывод STT"),
)


def norm_words(text: str) -> List[str]:
    return alt.norm(text).split()


def edge_insertions(original: str, heard: str) -> int:
    """Сколько слов вставлено в начале или в конце распознанного (больше из двух краёв)."""
    ops = difflib.SequenceMatcher(None, norm_words(original), norm_words(heard), autojunk=False).get_opcodes()
    ends = [ops[0], ops[-1]] if ops else []
    return max([j2 - j1 for tag, i1, i2, j1, j2 in ends if tag == "insert"] or [0])


def similarity(original: str, heard: str) -> float:
    return difflib.SequenceMatcher(None, alt.norm(original), alt.norm(heard), autojunk=False).ratio()


def scan(artifacts: Path) -> Dict[Tuple[str, str, str], Dict[str, Any]]:
    """Уникальные строки TRANSCRIPT: ``(метка, ожидалось, распознано)`` → первый источник и множество run_id."""
    seen: Dict[Tuple[str, str, str], Dict[str, Any]] = {}
    for path in sorted(Path(artifacts).glob("night_*/act*.log")):
        name = LOG_NAME.search(path.name)
        run_id = name.group(2) if name else ""
        for n, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            match = TRANSCRIPT.search(line)
            if match is None:
                continue
            row = seen.setdefault(match.groups(), {"file": f"{path.parent.name}/{path.name}", "line": n,
                                                   "run_id": run_id, "runs": set()})
            row["runs"].add(run_id)
    return seen


def verdict(entry: Optional[Dict[str, Any]], expected: str, heard: str) -> str:
    """Почему строку не берём («» - берём)."""
    if entry is None:
        return "метки нет в части 1"
    if alt.norm(expected) != alt.norm(entry["step"]["text"]):
        return "старая редакция фразы сценария"
    if not entry["scored"]:
        return "акустический шаг: в метрику не входит"
    if alt.norm(heard) == alt.norm(expected):
        return "слова совпали с исходником (разница только в регистре и знаках)"
    if similarity(expected, heard) < MIN_SIMILARITY:
        return f"мало общего с исходником (< {MIN_SIMILARITY:g}): могла попасть чужая речь или музыка"
    if edge_insertions(expected, heard) >= EDGE_WORDS:
        return f"≥ {EDGE_WORDS} лишних слов по краям: могла попасть речь живого человека"
    return ""


def collect(base: Dict[str, Any], artifacts: Path) -> Tuple[Dict[str, List[Dict[str, Any]]], Dict[str, int]]:
    by_label = {s["step"]["label"]: s for a in base["acts"] for s in a["steps"]}
    index = alt.index_of(base)
    rows: Dict[str, List[Dict[str, Any]]] = {}
    tally: Dict[str, int] = {"transcript_lines_unique": 0}
    for (label, expected, heard), src in scan(artifacts).items():
        tally["transcript_lines_unique"] += 1
        why = verdict(by_label.get(label), expected, heard)
        tally[why or "взято"] = tally.get(why or "взято", 0) + 1
        if why:
            continue
        rows.setdefault(by_label[label]["id"], []).append({
            "text": heard, "source": {"kind": "artifact", "run_id": src["run_id"], "file": src["file"],
                                      "line": src["line"]},
            "runs_seen": len(src["runs"]), "similarity": round(similarity(expected, heard), 3)})
    for issue, sid, heard, where in ISSUE_ROWS:
        why = verdict(index.get(sid), index[sid]["step"]["text"], heard) if sid in index else "метки нет в части 1"
        known = {alt.norm(r["text"]) for r in rows.get(sid, [])}
        if why or alt.norm(heard) in known:
            tally[f"карточка #{issue}: {why or 'уже есть из логов'}"] = 1
            continue
        tally["взято из карточек"] = tally.get("взято из карточек", 0) + 1
        rows.setdefault(sid, []).append({"text": heard, "source": {"kind": "issue", "issue": issue, "where": where},
                                         "runs_seen": 1, "similarity": round(similarity(index[sid]["step"]["text"],
                                                                                        heard), 3)})
    for sid in rows:
        rows[sid].sort(key=lambda r: -r["runs_seen"])  # стабильная сортировка: при равенстве - порядок находки
    return dict(sorted(rows.items(), key=lambda kv: list(index).index(kv[0]))), tally


def build(artifacts: Path) -> Dict[str, Any]:
    base = json.loads(build_marathon.CORPUS.read_text(encoding="utf-8"))
    rows, tally = collect(base, artifacts)
    sizes = [len(r) for r in rows.values()]
    skipped = [{"issue": i, "id": sid, "reason": why} for i, sid, why in ISSUE_SKIPPED]
    return {"name": "understanding_stt_part3", "schema": alt.SCHEMA, "part": PART, "base": alt.base_ref(base),
            "generator": "scripts/dialog/understanding_corpus/build_stt.py",
            "method": {"artifacts": "tests/e2e/_artifacts/night_*/act*.log (TRANSCRIPT) + ISSUE_ROWS",
                       "min_similarity": MIN_SIMILARITY, "edge_words": EDGE_WORDS, "filter": tally},
            "counts": {"steps": len(rows), "alternatives": sum(sizes), "max_per_step": max(sizes or [0]),
                       "from_issues": sum(1 for r in rows.values() for x in r if x["source"]["kind"] == "issue")},
            "excluded": skipped, "alternatives": rows}


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--artifacts", default=str(DEFAULT_ARTIFACTS), help="каталог tests/e2e/_artifacts с night_*")
    ap.add_argument("--check", action="store_true", help="сверить файл: хеш; с логами - пересборка совпадает")
    ap.add_argument("--repoint-from", default="", help="старый marathon.json: перевесить файл на текущую часть 1 без "
                                                       "новых строк из логов (alternatives.repoint)")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    if args.repoint_from:
        base = json.loads(build_marathon.CORPUS.read_text(encoding="utf-8"))
        alt.repoint_file(PART, Path(args.out), Path(args.repoint_from), base)
        return 0
    if args.check:
        base = json.loads(build_marathon.CORPUS.read_text(encoding="utf-8"))
        have = alt.load_part(PART, base, Path(args.out))
        if not Path(args.artifacts).is_dir():
            print(f"часть {PART}: хеш в порядке, {have['counts']}; логов {args.artifacts} нет - пересборку не сверял")
            return 0
        fresh_data = build(Path(args.artifacts))
        fresh = alt.digest(fresh_data["alternatives"])
        same = fresh == have["corpus_sha256"]
        print(f"часть {PART} {'совпадает' if same else 'РАЗОШЛАСЬ'} с логами: {have['corpus_sha256'][:12]} против "
              f"{fresh[:12]}")
        if not same:  # логи пополняются каждой ночью: показать, что изменилось, а не просто «разошлась»
            added = sorted({(sid, r["text"]) for sid, rows in fresh_data["alternatives"].items() for r in rows}
                           - {(sid, r["text"]) for sid, rows in have["alternatives"].items() for r in rows})
            print(f"  в логах сейчас: {fresh_data['method']['filter']}; новых строк против файла: {len(added)}")
            for sid, text in added:
                print(f"  + {sid}: {text}")
        return 0 if same else 1
    data = build(Path(args.artifacts))
    sha = alt.write_part(Path(args.out), data)
    print(f"записан {args.out}: {data['counts']} sha256={sha[:12]}")
    print(json.dumps(data["method"]["filter"], ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
