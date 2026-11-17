"""Сверка вердиктов сухого стенда с живым марафоном (карточка #4163, этап 1).

Логи живого прогона лежат в ``tests/e2e/_artifacts/night_<дата>/act<ключ>_<run_id>.log``; итог шага - последняя строка
``>>> STEP <метка>: …`` по этой метке в логе акта (харнесс пишет её на каждой попытке, последняя - итог после повторов).
Здесь только разбор итога и подсчёт расхождений; правила проверки шага остаются в ``e2e_tool_match``.

Вердикт живой строки: ``❌`` (кроме «✅ OK … прежние попытки провалились») - FAIL, ``✅`` - PASS, иначе ``UNKNOWN``.
Шаг, который харнесс вернул в строй сам («OK после восстановления харнессом»), отмечен ``recovered``: живой PASS, но
не от модели.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

STEP_LINE = re.compile(r">>> STEP (n\w+): (.*)$")
LOG_FILE = re.compile(r"act(\w+?)_\d+\.log$")
PASS, FAIL, UNKNOWN = "PASS", "FAIL", "UNKNOWN"


def verdict_of(last_line: str) -> str:
    """Вердикт по последней строке шага: провал харнесса или успех (в том числе с повтором)."""
    if "✅ OK" in last_line or ("✅" in last_line and "❌" not in last_line):
        return PASS
    if "❌" in last_line:
        return FAIL
    return UNKNOWN


def live_verdicts(directory: Path) -> Dict[str, Dict[str, Any]]:
    """``{"<акт>/<метка>": {verdict, line, recovered}}`` по логам каталога живого прогона."""
    out: Dict[str, Dict[str, Any]] = {}
    for path in sorted(Path(directory).glob("act*.log")):
        match = LOG_FILE.search(path.name)
        if match is None:
            continue
        last: Dict[str, str] = {}
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            found = STEP_LINE.search(line)
            if found:
                last[found.group(1)] = found.group(2)
        for label, text in last.items():
            out[f"{match.group(1)}/{label}"] = {"verdict": verdict_of(text), "line": text,
                                                "recovered": "после восстановления" in text}
    return out


def compare(dry: Dict[str, str], live: Dict[str, Dict[str, Any]], ids: Iterable[str]) -> Dict[str, Any]:
    """Расхождения вердиктов стенда (``dry``: id -> PASS/FAIL/…) с живым прогоном по шагам ``ids``.

    Сравниваются только шаги, у которых есть оба вердикта (PASS/FAIL); остальные - в ``skipped`` с причиной."""
    rows: List[Dict[str, Any]] = []
    skipped: List[Tuple[str, str]] = []
    for item in ids:
        want, have = live.get(item), dry.get(item)
        if want is None or want["verdict"] == UNKNOWN:
            skipped.append((item, "в живом логе нет итога шага"))
        elif have not in (PASS, FAIL):
            skipped.append((item, f"на стенде {have or 'нет вердикта'}"))
        else:
            rows.append({"id": item, "live": want["verdict"], "dry": have, "agree": want["verdict"] == have,
                         "recovered": want["recovered"], "live_line": want["line"]})
    off = [r for r in rows if not r["agree"]]
    return {"compared": len(rows), "disagree": len(off), "share": len(off) / len(rows) if rows else None,
            "rows": rows, "skipped": skipped,
            "live_fail_dry_pass": [r["id"] for r in off if r["live"] == FAIL],
            "live_pass_dry_fail": [r["id"] for r in off if r["live"] == PASS]}


__all__ = ["compare", "live_verdicts", "verdict_of"]
