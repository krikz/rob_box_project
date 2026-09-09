#!/usr/bin/env python3
"""tars_stats.py — эмпирическая сводка STT-семплов «ТАРС» с шлема (ADR-0077 §2.3).

Читает JSONL-файл, который пишет ``rob_box_voice.core.tars_sample_logger``
когда ``ROBBOX_STT_COLLECT=1``. Делает **только** сухую сводку — никаких
правок YAML, никакой мутации. Эта утилита — материал для решения Шифу
о расширении operator wake-list.

Режимы:

* (по умолчанию) сводка: топ-N вариантов «как STT слышит ТАРС» с частотой
  и длительностью.
* ``--diff``: сверяет с текущим ``docker/vision/config/wake_words.yaml``
  operator namespace и печатает кандидаты на добавление (нечётко — это
  пишет Шифу, не эта утилита).

Запуск:

    python scripts/stt/tars_stats.py data/stt_tars_samples.jsonl

    python scripts/stt/tars_stats.py --diff data/stt_tars_samples.jsonl

Скрипт не зависит от rob_box_voice — намеренно, чтобы работал на любой
машине, где есть только дамп выборки.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path


def load_records(path: Path) -> list[dict]:
    """Прочитать JSONL. Битые строки пропускаем с предупреждением."""
    out: list[dict] = []
    if not path.exists():
        return out
    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError as e:
            print(f"⚠️  строка {lineno}: bad JSON ({e}); пропускаю", file=sys.stderr)
    return out


def summarize(records: list[dict], top: int = 15) -> None:
    """Сводка по фразам (нормализованный lower + strip)."""
    if not records:
        print("🔇 записей нет — выборка пуста или сбор отключён")
        return
    counter: Counter[tuple[str, bool]] = Counter()
    durations: dict[tuple[str, bool], list[float]] = {}
    raw_seen: int = 0
    for r in records:
        text = (r.get("raw_text") or "").strip().lower()
        has_wake = bool(r.get("has_operator_wake", False))
        counter[(text, has_wake)] += 1
        durations.setdefault((text, has_wake), []).append(float(r.get("duration_s") or 0.0))
        if text:
            raw_seen += 1

    print(f"📊 всего записей: {len(records)} (с непустым raw_text: {raw_seen})")
    print(f"📊 с has_operator_wake=True: {sum(1 for r in records if r.get('has_operator_wake'))}")
    print()
    print(f"{'count':>5}  {'wake':<5}  {'avg_dur':<7}  raw_text (нормализованный)")
    print(f"{'-----':>5}  {'-----':<5}  {'-------':<7}  ----------------------")
    for (text, has_wake), cnt in counter.most_common(top):
        dur = durations[(text, has_wake)]
        avg = sum(dur) / len(dur) if dur else 0.0
        wake_mark = "YES" if has_wake else "no"
        display = text[:60] + ("…" if len(text) > 60 else "")
        print(f"{cnt:>5}  {wake_mark:<5}  {avg:>6.2f}с  {display}")


def diff_against_yaml(records: list[dict], yaml_path: Path) -> None:
    """--diff: какие слова из выборки НЕ покрыты текущим operator namespace."""
    if not yaml_path.exists():
        print(f"⚠️ YAML не найден: {yaml_path}", file=sys.stderr)
        return
    import yaml  # noqa: PLC0415 — лениво

    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    operator = data.get("operator") or []
    operator_set = {w.lower() for w in operator}

    word_re = re.compile(r"\b[а-яёa-z]+\b", re.IGNORECASE)
    counts: Counter[str] = Counter()
    for r in records:
        if r.get("has_operator_wake"):
            continue  # уже покрыто
        text = (r.get("raw_text") or "").lower()
        for m in word_re.findall(text):
            counts[m] += 1

    candidates = [
        (w, c) for w, c in counts.most_common(40)
        if w not in operator_set and len(w) >= 2
    ]
    if not candidates:
        print("🟢 нет кандидатов на расширение (или выборка пуста)")
        return
    print(f"📌 top кандидатов на расширение operator namespace (топ-40, не в YAML):")
    print(f"{'count':>5}  слово")
    print(f"{'-----':>5}  -----")
    for w, c in candidates:
        marker = " 🚩" if len(w) <= 3 else ""  # короткие слова = риск 6a
        print(f"{c:>5}  {w}{marker}")
    if any(len(w) <= 3 for w, _ in candidates):
        print()
        print("⚠️  Слова длиной ≤3 букв помечены 🚩 — риск ложных wake на фоновой речи (инвариант 6a).")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jsonl", type=Path, help="путь к stt_tars_samples.jsonl")
    ap.add_argument("--diff", action="store_true",
                    help="сверить с docker/vision/config/wake_words.yaml::operator")
    ap.add_argument("--yaml", type=Path,
                    default=Path(__file__).resolve().parents[2] / "docker" / "vision" / "config" / "wake_words.yaml",
                    help="путь к wake_words.yaml (по умолчанию репо-relative)")
    ap.add_argument("--top", type=int, default=15, help="сколько строк в сводке (по умолчанию 15)")
    args = ap.parse_args()

    records = load_records(args.jsonl)
    summarize(records, top=args.top)
    if args.diff:
        print()
        diff_against_yaml(records, args.yaml)
    return 0


if __name__ == "__main__":
    sys.exit(main())
