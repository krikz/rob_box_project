#!/usr/bin/env python3
"""
parse_duration_vs_score.py — issue #2769.

Парсит логи контейнера voice-assistant (Vision Pi) построчно и извлекает
последовательность (duration_sec, best_score, best_name, second_score, gap)
для каждой обработанной реплики (одна ROS-нода speaker_id_node, один поток
обработки в потоковом пуле — порядок строк лога совпадает с порядком
обработки реплик).

Источник: `docker logs -t voice-assistant` (Vision Pi, 10.1.1.21),
22.09.2026, окно ~12:49-13:22 UTC — единственное окно, доступное БЕЗ
перезапуска контейнера на момент замера (см. README.md рядом).

Использование:
    python3 parse_duration_vs_score.py raw-voice-assistant-log.txt
"""
from __future__ import annotations

import json
import re
import sys

RE_AUDIO = re.compile(r"Received speech audio: (\d+) bytes \(([\d.]+)s\)")
RE_CAND = re.compile(
    r"identify candidates: best='([^']*)'\(([0-9a-f]+)\) score=([\d.]+)"
    r"(?: \| second='([^']*)'\(([0-9a-f]+)\) score=([\d.]+) \| gap=([\d.-]+))?"
)
RE_SPEAKER_UNKNOWN = re.compile(r"👤 Speaker: unknown")


def parse(path: str) -> list[dict]:
    rows: list[dict] = []
    pending_duration = None
    pending_bytes = None
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = RE_AUDIO.search(line)
            if m:
                pending_bytes = int(m.group(1))
                pending_duration = float(m.group(2))
                continue
            m = RE_CAND.search(line)
            if m and pending_duration is not None:
                rows.append(
                    {
                        "duration_s": pending_duration,
                        "bytes": pending_bytes,
                        "best_name": m.group(1),
                        "best_id": m.group(2),
                        "best_score": float(m.group(3)),
                        "second_name": m.group(4),
                        "second_id": m.group(5),
                        "second_score": float(m.group(6)) if m.group(6) else None,
                        "gap": float(m.group(7)) if m.group(7) else None,
                    }
                )
                pending_duration = None
                pending_bytes = None
                continue
            m = RE_SPEAKER_UNKNOWN.search(line)
            if m and pending_duration is not None:
                rows.append(
                    {
                        "duration_s": pending_duration,
                        "bytes": pending_bytes,
                        "best_name": None,
                        "best_score": None,
                        "unknown": True,
                    }
                )
                pending_duration = None
                pending_bytes = None
                continue
    return rows


def main() -> None:
    rows = parse(sys.argv[1])
    print(json.dumps(rows, ensure_ascii=False, indent=2))

    scored = [r for r in rows if r.get("best_score") is not None]
    print(f"\n# всего реплик с duration+identify: {len(rows)}", file=sys.stderr)
    print(f"# из них со score (не unknown): {len(scored)}", file=sys.stderr)
    if scored:
        durs = [r["duration_s"] for r in scored]
        scores = [r["best_score"] for r in scored]
        print(f"# duration range: {min(durs):.1f}-{max(durs):.1f}s", file=sys.stderr)
        print(f"# score range: {min(scores):.3f}-{max(scores):.3f}", file=sys.stderr)
        n = len(scored)
        if n >= 2:
            mean_d = sum(durs) / n
            mean_s = sum(scores) / n
            cov = sum((d - mean_d) * (s - mean_s) for d, s in zip(durs, scores))
            sd_d = sum((d - mean_d) ** 2 for d in durs) ** 0.5
            sd_s = sum((s - mean_s) ** 2 for s in scores) ** 0.5
            r = cov / (sd_d * sd_s) if sd_d > 0 and sd_s > 0 else float("nan")
            print(f"# pearson r(duration, best_score) = {r:.3f} (n={n})", file=sys.stderr)


if __name__ == "__main__":
    main()
