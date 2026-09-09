#!/usr/bin/env python3
"""tars_stats.py — эмпирическая сводка STT-семплов «ТАРС» с шлема (ADR-0077 §2.3).

Читает JSONL-файл, который пишет ``rob_box_voice.core.tars_sample_logger``
когда ``ROBBOX_STT_COLLECT=1``. Делает **только** сухую сводку — никаких
правок YAML, никакой мутации. Эта утилита — материал для решения Шифу
о расширении operator wake-list.

Контракт сводки (ADR-0077 §2.3):

* читает JSONL, группирует по ``raw_text`` (нормализация lower + strip);
* печатает таблицу с **частотой, длительностью, распределением
  yandex-only vs yandex+vosk, долей wake-хитов**;
* в режиме ``--diff`` сравнивает с текущим ``docker/vision/config/wake_words.yaml``::operator
  и печатает **только подсказку** «вот слова, которых нет, но они встречаются
  N раз на M сегментов». Ничего не правит.

Режимы:

* (по умолчанию) сводка: топ-N вариантов «как STT слышит ТАРС» с частотой,
  длительностью и распределением провайдеров.
* ``--diff``: сверяет с текущим ``docker/vision/config/wake_words.yaml``
  operator namespace и печатает кандидаты на добавление (нечётко — это
  пишет Шифу, не эта утилита).

Запуск::

    python scripts/stt/tars_stats.py data/stt_tars_samples.jsonl
    python scripts/stt/tars_stats.py --diff data/stt_tars_samples.jsonl

Скрипт не зависит от rob_box_voice — намеренно, чтобы работал на любой
машине, где есть только дамп выборки.

Журнал изменений
----------------

* 2026-09-09 — issue #2223: добавлены колонки ``providers`` (y-only / y+vosk)
  и итоговая строка ``wake_rate``. ADR-0077 §2.3 теперь выполняется полностью.
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


def _provider_signature(attempts: list[dict] | None) -> str:
    """Классифицировать сегмент по списку attempts → 'y_only' / 'y+vosk' / 'none'.

    Логика (ADR-0077 §2.2: ``attempts: [{provider, ok, reason, latency_ms}]``):

    * ``y_only``  — сегмент распознан только Яндексом (vosk не подключался
      или не вернул ok=True);
    * ``y+vosk``  — vosk тоже вернул ok=True (fallback отработал);
    * ``none``    — ни один провайдер не дал ok=True (например пустой STT).

    Используется только поле ``ok`` — это снимок факта успеха, а не latency.
    """
    if not attempts:
        return "none"
    yandex_ok = any(a.get("provider") == "yandex" and a.get("ok") for a in attempts)
    vosk_ok = any(a.get("provider") == "vosk" and a.get("ok") for a in attempts)
    if yandex_ok and vosk_ok:
        return "y+vosk"
    if yandex_ok:
        return "y_only"
    return "none"


def summarize(records: list[dict], top: int = 15) -> None:
    """Сводка по фразам (нормализованный lower + strip).

    Колонки (ADR-0077 §2.3):

    * ``count``     — сколько раз этот raw_text встретился;
    * ``wake``      — yes/no — был ли в этом raw_text операторский wake;
    * ``avg_dur``   — средняя длительность wake-сегмента в секундах;
    * ``providers`` — ``y_only`` (только Яндекс дал ok) / ``y+vosk`` (оба);
      показывает, нужен ли был fallback на Vosk для данной фразы;
    * ``raw_text``  — нормализованный текст (lower + strip, обрезан до 60).

    В конце — блок итогов: ``wake_rate = wake_hits / total_with_text``
    в процентах, чтобы Шифу видел «сколько из услышанного — реальный wake».
    """
    if not records:
        print("🔇 записей нет — выборка пуста или сбор отключён")
        return

    counter: Counter[tuple[str, bool]] = Counter()
    durations: dict[tuple[str, bool], list[float]] = {}
    provider_buckets: dict[tuple[str, bool], Counter[str]] = {}
    raw_seen: int = 0
    wake_hits: int = 0

    for r in records:
        text = (r.get("raw_text") or "").strip().lower()
        has_wake = bool(r.get("has_operator_wake", False))
        sig = _provider_signature(r.get("attempts"))
        counter[(text, has_wake)] += 1
        durations.setdefault((text, has_wake), []).append(float(r.get("duration_s") or 0.0))
        provider_buckets.setdefault((text, has_wake), Counter())[sig] += 1
        if text:
            raw_seen += 1
        if has_wake:
            wake_hits += 1

    wake_rate = (wake_hits / raw_seen * 100.0) if raw_seen else 0.0
    print(f"📊 всего записей: {len(records)} (с непустым raw_text: {raw_seen})")
    print(f"📊 с has_operator_wake=True: {wake_hits}")
    print(f"📊 wake_rate = wake_hits / raw_seen = {wake_rate:.1f}%")
    print()
    print(f"{'count':>5}  {'wake':<5}  {'avg_dur':<7}  {'providers':<10}  raw_text (нормализованный)")
    print(f"{'-----':>5}  {'-----':<5}  {'-------':<7}  {'----------':<10}  ----------------------")
    for (text, has_wake), cnt in counter.most_common(top):
        dur = durations[(text, has_wake)]
        avg = sum(dur) / len(dur) if dur else 0.0
        buckets = provider_buckets[(text, has_wake)]
        # Главный сигнал: был ли задействован fallback (vosk).
        if buckets.get("y+vosk", 0) > 0:
            providers = f"y+vosk({buckets['y+vosk']})"
        elif buckets.get("y_only", 0) > 0:
            providers = f"y_only({buckets['y_only']})"
        else:
            providers = f"none({buckets.get('none', 0)})"
        wake_mark = "YES" if has_wake else "no"
        display = text[:60] + ("…" if len(text) > 60 else "")
        print(f"{cnt:>5}  {wake_mark:<5}  {avg:>6.2f}с  {providers:<10}  {display}")

    # Блок провайдеров по всей выборке — отдельный «снапшот», чтобы Шифу
    # видел общий фон: сколько сегментов вообще требовали fallback.
    total_buckets: Counter[str] = Counter()
    for c in provider_buckets.values():
        total_buckets.update(c)
    print()
    print(f"📊 провайдеры по всей выборке: "
          f"y_only={total_buckets.get('y_only', 0)}, "
          f"y+vosk={total_buckets.get('y+vosk', 0)}, "
          f"none={total_buckets.get('none', 0)}")


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
