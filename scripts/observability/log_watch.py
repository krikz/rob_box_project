#!/usr/bin/env python3
"""Смотритель логов, режим «вхолостую» (#4033, первый шаг; Refs #4512).

Раз в сутки читает из Loki строки ``voice-assistant`` (только чтение, ``query_range``), гонит их через детекторы
``scripts/dialog/turn_failures.py`` («робот глух», «ход без речи») и считает вердикты по видам и причинам. Ничего не
включает на роботе, в Telegram не пишет, пока не дан ``--send-tg``. Тексты реплик в сводку не попадают: адаптер строк
берёт из лога только вид события, время и id, а в сводке — счётчики и коды причин.

Честность пустого окна: нет событий в Loki (стек молчит или Loki недоступен) — это «событий нет», а не «0 сбоев».

    python -X utf8 scripts/observability/log_watch.py                       # окно 24 ч, сводка текстом
    python -X utf8 scripts/observability/log_watch.py --json out.json       # + JSON со счётчиками
    python -X utf8 scripts/observability/log_watch.py --lines-file x.log    # без Loki, из файла
    python -X utf8 scripts/observability/log_watch.py --send-tg             # + в Telegram (только по команде)
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))
import turn_failures as tf  # noqa: E402

LOKI_URL = os.environ.get("LOKI_URL", "http://10.1.1.249:3100")
CONTAINER = "voice-assistant"
#: Фильтр строк под события детекторов. Без эмодзи в начале: Loki с ним отдаёт 0 строк (ресёрч #4512, README).
LINE_FILTER = (
    r"stt_attempt|\[echo\] фраза|accumulated reason=no_wake|turn_id=|turn=|tier1 kind=|\[execute\] событие"
    r"|выступление старт|addressed=False|SpeechEvent"
)
CHUNK_S = 6 * 3600  # окна по 6 ч: за сутки одним запросом Loki отвечает 429
PAGE_LIMIT = 1000
SCHEMA = 1


def fetch_lines(start_s: float, end_s: float, loki_url: str = LOKI_URL) -> List[str]:
    """Строки ``voice-assistant`` окна из Loki, по времени, без дублей. Только GET ``query_range``."""
    seen: Dict[Tuple[int, str], None] = {}
    query = "{container=" + json.dumps(CONTAINER) + "} |~ " + json.dumps(LINE_FILTER)
    w0 = int(start_s * 1e9)
    end_ns = int(end_s * 1e9)
    while w0 < end_ns:
        w1 = min(w0 + CHUNK_S * 10**9, end_ns)
        cursor = w0
        while cursor < w1:
            batch = _query_page(loki_url, query, cursor, w1)
            for ts, line in batch:
                seen[(ts, line)] = None
            if len(batch) < PAGE_LIMIT:
                break
            cursor = max(ts for ts, _ in batch) + 1
        w0 = w1
    return [line for _, line in sorted(seen)]


def _query_page(loki_url: str, query: str, start_ns: int, end_ns: int) -> List[Tuple[int, str]]:
    params = urllib.parse.urlencode(
        dict(query=query, start=start_ns, end=end_ns, limit=PAGE_LIMIT, direction="forward")
    )
    url = f"{loki_url}/loki/api/v1/query_range?{params}"
    for attempt in range(6):
        try:
            with urllib.request.urlopen(url, timeout=300) as resp:  # noqa: S310 - адрес из конфига, только чтение
                data = json.load(resp)
            break
        except urllib.error.HTTPError as exc:
            if exc.code != 429:
                raise
            time.sleep(5 * (attempt + 1))
    else:
        raise SystemExit("Loki отвечает 429 и после повторов")
    return [(int(ts), line.rstrip()) for s in data["data"]["result"] for ts, line in s["values"]]


def _reason_parts(reason: str) -> Dict[str, int]:
    """``stt_rejected=3,stt_error=2`` -> словарь счётчиков (причина deaf, без текста)."""
    out: Dict[str, int] = {}
    for part in reason.split(","):
        name, _, num = part.partition("=")
        if name and num.isdigit():
            out[name] = int(num)
    return out


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M")


def summarize(lines: Iterable[str], start_s: float, end_s: float) -> Dict[str, Any]:
    """Строки окна -> сводка: только счётчики и коды. Нет событий -> ``status=no_events``."""
    events = tf.events_from_log(lines)
    verdicts = tf.detect(events)
    kinds = Counter(e.kind for e in events)
    deaf = [v for v in verdicts if v.kind == tf.DEAF]
    no_speech = [v for v in verdicts if v.kind == tf.NO_SPEECH]
    deaf_parts: Counter = Counter()
    for verdict in deaf:
        deaf_parts.update(_reason_parts(verdict.reason))
    turns = kinds[tf.TURN_END]
    signals = kinds[tf.STT_REJECTED] + kinds[tf.NO_WAKE] + turns
    return {
        "schema": SCHEMA,
        "mode": "dry_run",
        "window": {"start_utc": _iso(start_s), "end_utc": _iso(end_s)},
        "status": "ok" if signals else "no_events",
        "events": {
            "total": len(events),
            "turns": turns,
            "stt_rejected": kinds[tf.STT_REJECTED],
            "no_wake": kinds[tf.NO_WAKE],
        },
        "verdicts": {
            "total": len(verdicts),
            "deaf": {
                "count": len(deaf),
                "by_outcome": dict(sorted(deaf_parts.items())),
                "with_stt_error_ge_2": sum(1 for v in deaf if _reason_parts(v.reason).get(tf.STT_ERROR, 0) >= 2),
            },
            "no_speech": {
                "count": len(no_speech),
                "by_reason": dict(sorted(Counter(v.reason for v in no_speech).items())),
            },
        },
    }


def _breakdown(items: Dict[str, int]) -> str:
    return ", ".join(f"{k}={v}" for k, v in items.items())


def render_text(summary: Dict[str, Any]) -> str:
    """Короткая сводка для Telegram. Пустое окно — прямо «событий нет»."""
    win = summary["window"]
    head = f"Смотритель логов (вхолостую), {win['start_utc']} - {win['end_utc']} UTC"
    if summary["status"] == "no_events":
        return f"{head}\nСобытий голосового стека в Loki за окно нет: проверить, жив ли стек и Loki. Сбоев не считаю."
    ev, vd = summary["events"], summary["verdicts"]
    out = [head, f"Ходов {ev['turns']}, отклонённых STT {ev['stt_rejected']}, фраз без вейка {ev['no_wake']}."]
    deaf, ns = vd["deaf"], vd["no_speech"]
    line = f"Робот глух: {deaf['count']}"
    if deaf["count"]:
        line += f" ({_breakdown(deaf['by_outcome'])}; серий с stt_error>=2: {deaf['with_stt_error_ge_2']})"
    out.append(line)
    line = f"Ход без речи: {ns['count']}"
    if ns["count"]:
        line += f" ({_breakdown(ns['by_reason'])})"
    out.append(line)
    return "\n".join(out)


def send_tg(text: str) -> None:
    """Через ``scripts/robot_dev/tgsend.sh`` (бот робота). Вызывается только с ``--send-tg``."""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "log_watch_tg.txt"
        path.write_text(text, encoding="utf-8")
        subprocess.run(["bash", str(REPO / "scripts" / "robot_dev" / "tgsend.sh"), str(path)], check=True)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    parser.add_argument("--hours", type=float, default=24.0, help="окно назад от сейчас, ч (24)")
    parser.add_argument("--loki-url", default=LOKI_URL)
    parser.add_argument("--lines-file", type=Path, help="читать строки из файла вместо Loki")
    parser.add_argument("--json", type=Path, dest="json_out", help="записать сводку JSON")
    parser.add_argument("--send-tg", action="store_true", help="отправить текст в Telegram (только по команде)")
    args = parser.parse_args(argv)
    end_s = time.time()
    start_s = end_s - args.hours * 3600
    if args.lines_file:
        lines = args.lines_file.read_text(encoding="utf-8", errors="replace").splitlines()
    else:
        lines = fetch_lines(start_s, end_s, args.loki_url)
    summary = summarize(lines, start_s, end_s)
    text = render_text(summary)
    print(text)
    if args.json_out:
        args.json_out.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.send_tg:
        send_tg(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
