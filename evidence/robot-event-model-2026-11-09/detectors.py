"""Детекторы сбоя хода (#4512, шаг 3) на архиве марафонов: поймано / пропущено / ложные. Только чтение архива.

    python -X utf8 detectors.py <архив> [--grid]

Детекторы — ``scripts/dialog/turn_failures.py``. Шаг сопоставлен вердикту по времени: окно шага — от ``at`` прошлого
шага до своего ``at`` (``steps.jsonl``; у первого шага акта — 120 с до ``at``). Правда — класс шага из
``e2e_verdict.log``: ``FAIL no_accept`` и ``FAIL turn_no_wake`` — «робот глух», ``FAIL turn_nospeech`` — «ход без
речи». Шаг, где все попытки ``PLAY`` харнесса упали (``PLAY_FAIL``), отмечен отдельно: фраза не прозвучала. Печатает
только счётчики и метки шагов, текста реплик нет.
"""

import collections
import datetime
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "scripts", "dialog"))
import turn_failures as tf  # noqa: E402

DEAF_TRUTH = {"no_accept", "turn_no_wake"}
SPEECH_TRUTH = {"turn_nospeech"}
FIRST_STEP_LOOKBACK_S = 120.0


def _epoch(iso):
    return datetime.datetime.strptime(iso, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=datetime.timezone.utc).timestamp()


def steps_of(run_dir):
    """Шаги акта по порядку: метка, статус, класс, конец окна (``at``), все ли PLAY упали."""
    verdict_log = glob.glob(os.path.join(run_dir, "e2e-voice-logs-*", "e2e_verdict.log"))
    steps_jsonl = glob.glob(os.path.join(run_dir, "e2e-voice-artifacts-*", "*", "steps.jsonl"))
    if not verdict_log or not steps_jsonl:
        return []
    at = {}
    for line in open(steps_jsonl[0], encoding="utf-8"):
        row = json.loads(line)
        at[row["label"]] = _epoch(row["at"])
    out, plays = [], collections.defaultdict(lambda: [0, 0])
    for line in open(verdict_log[0], encoding="utf-8", errors="replace"):
        if ": PLAY attempt" in line and line.startswith(">>> STEP "):
            label = line.split()[2].rstrip(":")
            plays[label][0] += 1
            last_play = label
        elif line.startswith(">>>   PLAY_FAIL"):
            plays[last_play][1] += 1
        elif line.startswith("E2E_STEP "):
            parts = line.split()
            label = parts[1]
            if label in at:
                tried, failed = plays[label]
                out.append(
                    {
                        "label": label,
                        "status": parts[2],
                        "cls": parts[3] if len(parts) > 3 else "",
                        "at": at[label],
                        "not_played": tried > 0 and tried == failed,
                    }
                )
    out.sort(key=lambda s: s["at"])
    prev = None
    for step in out:
        step["from"] = prev if prev is not None else step["at"] - FIRST_STEP_LOOKBACK_S
        prev = step["at"]
    return out


def truth(step):
    if step["status"] == "FAIL" and step["cls"] in DEAF_TRUTH:
        return tf.DEAF
    if step["status"] == "FAIL" and step["cls"] in SPEECH_TRUTH:
        return tf.NO_SPEECH
    return ""


def hits(step, verdicts, kind):
    lo, hi = step["from"], step["at"]
    for v in verdicts:
        if v.kind != kind:
            continue
        start = v.since or v.ts
        if start <= hi and v.ts > lo:
            return True
    return False


def evaluate(root):
    table = collections.Counter()
    reasons = collections.Counter()
    caught_labels = collections.defaultdict(list)
    n_runs = 0
    for log in sorted(glob.glob(os.path.join(root, "*", "e2e-voice-logs-*", "voice_e2e_*.log"))):
        run_dir = os.path.dirname(os.path.dirname(log))
        steps = steps_of(run_dir)
        if not steps:
            continue
        n_runs += 1
        with open(log, encoding="utf-8", errors="replace") as handle:
            verdicts = tf.detect(tf.events_from_log(handle))
        lo, hi = steps[0]["from"], steps[-1]["at"]
        verdicts = [v for v in verdicts if lo < v.ts <= hi]
        for v in verdicts:
            reasons[(v.kind, v.reason.split("=")[0] if v.kind == tf.DEAF else v.reason)] += 1
        for step in steps:
            want = truth(step)
            for kind in (tf.DEAF, tf.NO_SPEECH):
                got = hits(step, verdicts, kind)
                if want == kind:
                    key = "поймано" if got else "пропущено"
                    sub = " (PLAY харнесса упал)" if step["not_played"] else ""
                    table[(kind, key + sub)] += 1
                    caught_labels[(kind, key + sub)].append(f"{os.path.basename(run_dir)}:{step['label']}")
                elif got:
                    grp = "OK" if step["status"] == "OK" else f"{step['status']} {step['cls'] or '-'}"
                    table[(kind, f"ложное на шаге {grp}")] += 1
                    caught_labels[(kind, f"ложное на шаге {grp}")].append(
                        f"{os.path.basename(run_dir)}:{step['label']}"
                    )
            table[("шаги", f"{step['status']} {step['cls'] or '-'}")] += 1
    return n_runs, table, reasons, caught_labels


def heard_in_unplayed(root):
    """Сколько фраз робот получил (``🎤 Получена фраза``) в окнах шагов, где все PLAY харнесса упали."""
    steps_n, phrases = 0, 0
    for log in sorted(glob.glob(os.path.join(root, "*", "e2e-voice-logs-*", "voice_e2e_*.log"))):
        steps = [s for s in steps_of(os.path.dirname(os.path.dirname(log))) if s["not_played"]]
        if not steps:
            continue
        stamps = []
        for line in open(log, encoding="utf-8", errors="replace"):
            if "🎤 Получена фраза" in line:
                event = tf._TS.search(line)
                if event:
                    stamps.append(float(event.group(1)))
        for step in steps:
            steps_n += 1
            phrases += sum(1 for t in stamps if step["from"] < t <= step["at"])
    return steps_n, phrases


def grid(root):
    """Порог серии против поймано/ложных: no_wake отдельно (отклонённые выключены) и отклонённые отдельно."""
    print(
        "сетка порогов: «глух» в архиве — 21 шаг, из них 16 с упавшим PLAY харнесса (ловить нечего) и 5 "
        "turn_no_wake; поймано — из всех 21; ложные — на шагах без класса «глух»:"
    )
    keep = (tf.DEAF_REJECTED_MIN, tf.DEAF_NO_WAKE_MIN, tf.DEAF_WINDOW_S)
    for name, values in (("DEAF_NO_WAKE_MIN", range(1, 9)), ("DEAF_REJECTED_MIN", range(1, 5))):
        for window in (60.0, 180.0):
            for n in values:
                tf.DEAF_REJECTED_MIN, tf.DEAF_NO_WAKE_MIN = (99, n) if name == "DEAF_NO_WAKE_MIN" else (n, None)
                tf.DEAF_WINDOW_S = window
                _, table, _, _ = evaluate(root)
                fp = sum(v for (k, key), v in table.items() if k == tf.DEAF and key.startswith("ложное"))
                fp_backlog = table.get((tf.DEAF, "ложное на шаге OK"), 0)
                print(
                    f"  {name}={n} окно={window:.0f}с: поймано {table.get((tf.DEAF, 'поймано'), 0)}, "
                    f"ложных {fp} (из них на OK-шагах {fp_backlog})"
                )
    tf.DEAF_REJECTED_MIN, tf.DEAF_NO_WAKE_MIN, tf.DEAF_WINDOW_S = keep


def live(path):
    """Живые строки из Loki (выгрузка в scratchpad): разметки нет — только счёт событий и вердиктов."""
    with open(path, encoding="utf-8", errors="replace") as handle:
        events = tf.events_from_log(handle)
    verdicts = tf.detect(events)
    ends = [e for e in events if e.kind == tf.TURN_END]
    print(
        f"живые: событий {len(events)}, ходов {len(ends)} "
        f"({dict(collections.Counter(e.agent for e in ends))}), отклонённых STT "
        f"{sum(1 for e in events if e.kind == tf.STT_REJECTED)}, "
        f"без вейка {sum(1 for e in events if e.kind == tf.NO_WAKE)}"
    )
    if events:
        print(
            f"живые: окно {datetime.datetime.utcfromtimestamp(events[0].ts):%d.%m %H:%M} – "
            f"{datetime.datetime.utcfromtimestamp(events[-1].ts):%d.%m %H:%M} UTC"
        )
    for (kind, reason), n in sorted(collections.Counter((v.kind, v.reason) for v in verdicts).items()):
        print(f"  {n:4d}  {kind:9s}  {reason}")
    stages = collections.Counter(
        (e.value, e.extra) for e in ends for v in verdicts if v.kind == tf.NO_SPEECH and v.turn_id == e.turn_id
    )
    print("  стадия/статус ходов без речи:", dict(stages))


def main():
    root = sys.argv[1]
    if "--live" in sys.argv:
        live(sys.argv[sys.argv.index("--live") + 1])
        return
    if "--grid" in sys.argv:
        grid(root)
        return
    steps_n, phrases = heard_in_unplayed(root)
    print(f"шагов, где все PLAY харнесса упали: {steps_n}; фраз, полученных роботом в их окнах: {phrases}")
    n_runs, table, reasons, labels = evaluate(root)
    print("прогонов с логом и шагами", n_runs)
    print(
        "пороги: DEAF_REJECTED_MIN",
        tf.DEAF_REJECTED_MIN,
        "DEAF_NO_WAKE_MIN",
        tf.DEAF_NO_WAKE_MIN,
        "DEAF_WINDOW_S",
        tf.DEAF_WINDOW_S,
        "SPEECH_GRACE_S",
        tf.SPEECH_GRACE_S,
    )
    for (kind, key), n in sorted(table.items()):
        print(f"  {n:4d}  {kind:9s}  {key}")
    print("причины вердиктов (в окнах шагов):")
    for (kind, reason), n in sorted(reasons.items()):
        print(f"  {n:4d}  {kind:9s}  {reason}")
    print("шаги по ячейкам:")
    for (kind, key), items in sorted(labels.items()):
        print(f"  {kind} | {key}: {' '.join(items)}")


if __name__ == "__main__":
    main()
