#!/usr/bin/env python3
"""CLI: прогнать стенд по выборке архива и напечатать/сохранить отчёт.

Пример::

    python tools/music_bench/run_bench.py --count 50 --out report.csv

См. ``docs/music_bench.md`` — как запустить и как читать метрики.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import List

from . import _repo_paths  # noqa: F401
from .metrics import TrackMetrics
from .render import render_bench_track
from .sampler import iter_bench_specs

# Windows-консоль (cp1252/cp866) не кодирует кириллицу из docstring'ов ниже
# и из значений report (названия тем архива бывают не-ASCII) — переводим
# stdout/stderr в UTF-8 с заменой не кодируемых символов, а не падаем.
for _stream_name in ("stdout", "stderr"):
    _stream = getattr(sys, _stream_name)
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8", errors="replace")


def run(count: int, seed: int, out_csv: str | None, out_json: str | None) -> int:
    rows: List[dict] = []
    errors: List[str] = []
    for sample, combo, spec, exc in iter_bench_specs(count, seed=seed):
        label = f"{sample.name}[{combo.label}]"
        if exc is not None:
            errors.append(str(exc))
            print(f"FAIL  {label}: {exc}", file=sys.stderr)
            continue
        metrics: TrackMetrics = render_bench_track(spec, label)
        rows.append(metrics.as_row())
        print(
            f"OK    {label}: pre_tanh_peak={metrics.pre_tanh_peak:.3f} "
            f"pct>1={metrics.pct_time_over_1:.3%} "
            f"true_peak={metrics.true_peak:.3f} "
            f"crest={metrics.crest_factor_db:.1f}dB "
            f"max_layers={metrics.max_simultaneous_layers}"
        )

    if rows:
        fieldnames = sorted({k for row in rows for k in row})
        if out_csv:
            with open(out_csv, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            print(f"CSV: {out_csv}")
        if out_json:
            Path(out_json).write_text(
                json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            print(f"JSON: {out_json}")

        _print_summary(rows)

    print(f"\nИтого: {len(rows)} ок, {len(errors)} ошибок из {len(rows) + len(errors)} прогонов.")
    return 1 if errors and not rows else 0


def _print_summary(rows: List[dict]) -> None:
    import statistics

    def col(name: str) -> List[float]:
        return [r[name] for r in rows if isinstance(r.get(name), (int, float))]

    print("\n=== Сводка ===")
    for name in ("pre_tanh_peak", "pct_time_over_1", "true_peak", "crest_factor_db", "max_simultaneous_layers"):
        values = col(name)
        if not values:
            continue
        print(
            f"{name}: min={min(values):.4g} median={statistics.median(values):.4g} "
            f"max={max(values):.4g}"
        )
    over_1 = sum(1 for r in rows if r.get("pct_time_over_1", 0) > 0)
    print(f"Треков с ненулевым насыщением (сумма>1 до tanh хоть раз): {over_1}/{len(rows)}")


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=50, help="сколько тем архива (x3 комбо)")
    parser.add_argument("--seed", type=int, default=2977, help="сид выборки (воспроизводимость)")
    parser.add_argument("--out", dest="out_csv", default=None, help="путь к CSV-отчёту")
    parser.add_argument("--out-json", dest="out_json", default=None, help="путь к JSON-отчёту")
    args = parser.parse_args(argv)
    return run(args.count, args.seed, args.out_csv, args.out_json)


if __name__ == "__main__":
    raise SystemExit(main())
