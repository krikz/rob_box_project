#!/usr/bin/env python3
"""Офлайн-вердикты детекторов сбоя хода по логу ``voice-assistant`` акта, без текстов реплик (#4512, шаг 1 PR #4603).

Лог акта (``/tmp/voice_e2e_<run>.log``, шаг «Collect robot logs» workflow) гонится через
``scripts/dialog/turn_failures.py`` (``detect`` + ``events_from_log``): «робот глух» и «ход без речи». Репо ПУБЛИЧНОЕ,
поэтому в ``turn_failures.json`` артефакта акта идут только счётчики и коды причин (``reason`` детектора — исходы серии
или чем кончился ход, текста реплик в нём нет); сами строки лога и ``Verdict.line()`` в отчёт не попадают.

Печатает одну строку для лога акта и сводки марафона (``marathon_summary_value turn_failures``):
``E2E_SUMMARY turn_failures=deaf:1,no_speech:2`` | ``E2E_SUMMARY turn_failures=0`` (события есть, сбоев нет) |
``E2E_SUMMARY turn_failures=unknown`` (событий нет: лог пуст или не снялся -- «0 сбоев» тут было бы враньём).

Использование: turn_failures_summary.py <voice_e2e.log> <out.json>. Код возврата всегда 0: отчёт не красит акт.
"""

from __future__ import annotations

import json
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, Optional, Sequence

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))
import turn_failures as tf  # noqa: E402  (подтягивает rob_box_dialog.knowledge из src/ репо)

#: Причина детектора в отчёт идёт только если состоит из кодов и счётчиков (``stt_rejected=3,stt_error=2``).
_SAFE_REASON = re.compile(r"^[a-z0-9_=,:.\-]{1,120}$")
KINDS = (tf.DEAF, tf.NO_SPEECH)


def summarize(lines) -> Dict[str, object]:
    """Счётчики вердиктов по виду и по ``вид/причина``; ``events`` -- сколько событий нашлось в логе вообще."""
    events = tf.events_from_log(lines)
    verdicts = tf.detect(events)
    by_kind = Counter(v.kind for v in verdicts)
    reasons: Counter = Counter()
    for verdict in verdicts:
        reason = verdict.reason if _SAFE_REASON.match(verdict.reason or "") else "other"
        reasons[f"{verdict.kind}/{reason}"] += 1
    return {
        "events": len(events),
        "turns": sum(1 for e in events if e.kind == tf.TURN_END),
        "verdicts": {kind: by_kind.get(kind, 0) for kind in KINDS},
        "reasons": dict(sorted(reasons.items())),
    }


def marker(report: Dict[str, object]) -> str:
    """Значение ключа ``turn_failures`` строки E2E_SUMMARY."""
    if report.get("error") or not report.get("events"):
        return "unknown"
    counts = report["verdicts"]  # type: ignore[index]
    parts = [f"{kind}:{counts[kind]}" for kind in KINDS if counts[kind]]  # type: ignore[index]
    return ",".join(parts) or "0"


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    log_path, out_path = Path(args[0]), Path(args[1])
    report: Dict[str, object]
    try:
        with log_path.open(encoding="utf-8", errors="replace") as handle:
            report = summarize(handle)
    except Exception as exc:  # отчёт не красит акт; причина -- в самом отчёте
        report = {"error": f"{type(exc).__name__}: {exc}"}
    report["source"] = "scripts/dialog/turn_failures.py"
    out_path.write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print(f"E2E_SUMMARY turn_failures={marker(report)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
