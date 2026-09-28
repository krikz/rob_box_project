#!/usr/bin/env python3
"""measure_doa.py — замер точности DOA ReSpeaker (ADR-0137 §2.3, ADR-0130 §2.9 / Q17).

Оффлайн-инструмент оператора, не нода и не сервис: новых топиков не создаёт,
только слушает существующие ``/audio/direction`` (std_msgs/Int32, 0–359°) и
``/audio/vad`` (std_msgs/Bool, публикуется audio_node ТОЛЬКО по фронтам).

Методика (подробно — ADR-0137 §2.3):
  * Источник речи (человек или колонка) стоит на ~1.5 м от центра массива,
    на известном азимуте ``--angle`` (градусы, REP-103: 0° — вперёд по X
    base_link, против часовой положительно). Робот молчит (нет TTS/музыки —
    иначе audio_node гейтит VAD).
  * Один «замер» = одна VAD-фраза: DOA копится только пока VAD == True,
    первые ``--settle-s`` после фронта отбрасываются (оценка угла сходится),
    оценка замера — круговое среднее отсчётов фразы.
  * Нужно ≥ 20 замеров суммарно по ≥ 4 углам (минимум 0/90/180/270).

Запуск на Vision Pi (скрипт в контейнер не смонтирован — копируем):
    docker cp scripts/maintenance/measure_doa.py voice-assistant:/tmp/measure_doa.py
    docker exec -it voice-assistant bash -lc \\
      'source /opt/ros/humble/setup.bash && \\
       python3 /tmp/measure_doa.py measure --angle 90 --samples 3 --csv /tmp/doa.csv'
    ... повторить для других углов (CSV дописывается) ...
    docker exec voice-assistant python3 /tmp/measure_doa.py summary --csv /tmp/doa.csv

``summary`` не требует ROS и работает где угодно. Направление 0° прошивки и
знак отсчёта неизвестны (ADR-0137 §4), поэтому по умолчанию ``summary``
подбирает отображение ``bearing = offset + sign * doa`` по данным; если
оно уже известно — ``--offset/--sign`` фиксируют его.

Коды выхода: 0 — есть вердикт (или замер записан); 1 — нет rclpy;
2 — данных недостаточно для вердикта;
3 — ``/audio/direction`` молчит (см. ADR-0137 §2.3 «предусловие»).

Только стандартная библиотека (+ rclpy лениво внутри ``measure``).
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

TOLERANCE_DEG = 15.0  # ADR-0130 Q17
MIN_SEGMENTS = 20
MIN_DISTINCT_ANGLES = 4
CSV_HEADER = ["ts", "true_deg", "doa_deg", "err_deg", "segment"]

VERDICT_OK = "≤15°: DOA годится для выбора говорящего трека (ADR-0130 §2.9)"
VERDICT_TURN_ONLY = ">15°: только повернуться на звук"


# ── чистые функции (тестируются без ROS) ─────────────────────────────────────


def wrap180(deg: float) -> float:
    """Привести угол к (-180, 180]."""
    d = math.fmod(deg, 360.0)
    if d <= -180.0:
        d += 360.0
    elif d > 180.0:
        d -= 360.0
    return d


def circular_error(true_deg: float, measured_deg: float) -> float:
    """Знаковая круговая ошибка measured − true в (-180, 180]: 359 vs 1 → −2, а не 358."""
    return wrap180(measured_deg - true_deg)


def circular_mean(degs: Sequence[float]) -> float:
    """Круговое среднее в [0, 360). Пустой вход — ValueError."""
    if not degs:
        raise ValueError("circular_mean: пустой список")
    s = sum(math.sin(math.radians(d)) for d in degs)
    c = sum(math.cos(math.radians(d)) for d in degs)
    return math.degrees(math.atan2(s, c)) % 360.0


def percentile(values: Sequence[float], q: float) -> float:
    """Перцентиль с линейной интерполяцией (как numpy по умолчанию), q в [0, 100]."""
    if not values:
        raise ValueError("percentile: пустой список")
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    pos = (len(xs) - 1) * q / 100.0
    lo = math.floor(pos)
    hi = math.ceil(pos)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def apply_mapping(doa_deg: float, offset_deg: float, sign: int) -> float:
    """Азимут в base_link (REP-103) из сырого DOAANGLE: offset + sign·doa, в [0, 360)."""
    return (offset_deg + sign * doa_deg) % 360.0


def fit_mapping(pairs: Sequence[Tuple[float, float]]) -> Tuple[float, int]:
    """Подобрать (offset, sign) по парам (true_deg, doa_deg).

    Перебор sign ∈ {+1, −1} и offset по сетке 1°, критерий — сумма квадратов
    круговой ошибки. Подгонка на тех же данных, что и оценка, чуть занижает
    ошибку (2 параметра на ≥ 20 замеров) — это оговорено в ADR-0137 §2.3.
    """
    if not pairs:
        raise ValueError("fit_mapping: нет данных")
    best: Optional[Tuple[float, float, int]] = None
    for sign in (1, -1):
        for offset in range(360):
            cost = sum(
                circular_error(t, apply_mapping(d, offset, sign)) ** 2 for t, d in pairs
            )
            if best is None or cost < best[0]:
                best = (cost, float(offset), sign)
    assert best is not None
    return best[1], best[2]


@dataclass
class Segment:
    """Одна VAD-фраза = один замер."""

    index: int
    true_deg: float
    samples: List[Tuple[float, float]] = field(default_factory=list)  # (ts, doa_deg)


def segments_from_rows(rows: Iterable[Dict[str, str]]) -> List[Segment]:
    """Сгруппировать строки CSV в замеры по (true_deg, segment)."""
    by_key: Dict[Tuple[float, str], Segment] = {}
    for row in rows:
        true_deg = float(row["true_deg"])
        key = (true_deg, row["segment"])
        seg = by_key.get(key)
        if seg is None:
            seg = Segment(index=len(by_key), true_deg=true_deg)
            by_key[key] = seg
        seg.samples.append((float(row["ts"]), float(row["doa_deg"])))
    return list(by_key.values())


@dataclass
class Summary:
    n_segments: int
    n_samples: int
    distinct_angles: List[float]
    offset_deg: float
    sign: int
    fitted: bool
    p50: float
    p95: float
    max_err: float
    sample_p95: float
    enough_data: bool
    passed: bool

    @property
    def verdict(self) -> str:
        if not self.enough_data:
            return (
                f"НЕДОСТАТОЧНО ДАННЫХ: нужно ≥{MIN_SEGMENTS} замеров по ≥{MIN_DISTINCT_ANGLES} "
                f"углам (есть {self.n_segments} по {len(self.distinct_angles)})"
            )
        return VERDICT_OK if self.passed else VERDICT_TURN_ONLY


def summarize(
    segments: Sequence[Segment],
    offset_deg: Optional[float] = None,
    sign: Optional[int] = None,
    tolerance_deg: float = TOLERANCE_DEG,
) -> Summary:
    """Сводка по замерам. Критерий «прошло»: p95 |ошибки замера| ≤ tolerance.

    Если offset/sign не заданы — подбираются по данным (fit_mapping).
    """
    segs = [s for s in segments if s.samples]
    if not segs:
        raise ValueError("summarize: нет ни одного замера")
    fitted = offset_deg is None or sign is None
    if fitted:
        pairs = [(s.true_deg, circular_mean([d for _, d in s.samples])) for s in segs]
        offset_deg, sign = fit_mapping(pairs)
    assert offset_deg is not None and sign is not None

    seg_errs: List[float] = []
    sample_errs: List[float] = []
    for s in segs:
        mapped = [apply_mapping(d, offset_deg, sign) for _, d in s.samples]
        seg_errs.append(abs(circular_error(s.true_deg, circular_mean(mapped))))
        sample_errs.extend(abs(circular_error(s.true_deg, m)) for m in mapped)

    angles = sorted({s.true_deg % 360.0 for s in segs})
    enough = len(segs) >= MIN_SEGMENTS and len(angles) >= MIN_DISTINCT_ANGLES
    p95 = percentile(seg_errs, 95)
    return Summary(
        n_segments=len(segs),
        n_samples=len(sample_errs),
        distinct_angles=angles,
        offset_deg=offset_deg,
        sign=sign,
        fitted=fitted,
        p50=percentile(seg_errs, 50),
        p95=p95,
        max_err=max(seg_errs),
        sample_p95=percentile(sample_errs, 95),
        enough_data=enough,
        passed=enough and p95 <= tolerance_deg,
    )


class DoaCollector:
    """Копит DOA только внутри VAD-фраз. Без ROS: время передаётся явно.

    ``/audio/vad`` приходит только на фронтах, поэтому держим последнее
    состояние; до первого фронта считаем VAD == False.
    """

    def __init__(self, true_deg: float, settle_s: float = 0.3, min_samples: int = 3):
        self.true_deg = true_deg
        self.settle_s = settle_s
        self.min_samples = min_samples
        self.vad = False
        self._rise_t = 0.0
        self._current: List[Tuple[float, float]] = []
        self._seg_counter = 0
        self.completed: List[Segment] = []
        self.discarded = 0
        self.doa_msgs_total = 0

    def on_vad(self, active: bool, t: float) -> None:
        if active and not self.vad:
            self._rise_t = t
            self._current = []
        elif not active and self.vad:
            self._close()
        self.vad = active

    def on_doa(self, doa_deg: float, t: float) -> None:
        self.doa_msgs_total += 1
        if self.vad and (t - self._rise_t) >= self.settle_s:
            self._current.append((t, float(doa_deg) % 360.0))

    def _close(self) -> None:
        if len(self._current) >= self.min_samples:
            self._seg_counter += 1
            self.completed.append(
                Segment(
                    index=self._seg_counter,
                    true_deg=self.true_deg,
                    samples=self._current,
                )
            )
        else:
            self.discarded += 1
        self._current = []

    def rows(self, run_id: str) -> List[List[str]]:
        """Строки CSV (ts, true_deg, doa_deg, err_deg, segment) — err без отображения."""
        out = []
        for seg in self.completed:
            for ts, doa in seg.samples:
                out.append(
                    [
                        f"{ts:.3f}",
                        f"{self.true_deg:g}",
                        f"{doa:g}",
                        f"{circular_error(self.true_deg, doa):g}",
                        f"{run_id}-{seg.index}",
                    ]
                )
        return out


def append_csv(path: str, rows: Sequence[Sequence[str]]) -> None:
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new:
            w.writerow(CSV_HEADER)
        w.writerows(rows)


def read_csv(path: str) -> List[Dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def format_summary(s: Summary, tolerance_deg: float = TOLERANCE_DEG) -> str:
    mapping = "подобрано по данным" if s.fitted else "задано вручную"
    return "\n".join(
        [
            f"замеров (VAD-фраз): {s.n_segments}, отсчётов DOA: {s.n_samples}",
            f"углы, °: {', '.join(f'{a:g}' for a in s.distinct_angles)}",
            f"отображение: bearing = {s.offset_deg:g} + ({s.sign:+d})·doa  [{mapping}]",
            f"|ошибка замера|, °: p50={s.p50:.1f} p95={s.p95:.1f} max={s.max_err:.1f}",
            f"|ошибка отсчёта|, °: p95={s.sample_p95:.1f} (справочно)",
            f"допуск (Q17): p95 ≤ {tolerance_deg:g}°",
            f"ВЕРДИКТ: {s.verdict}",
        ]
    )


# ── ROS-часть (лениво) ───────────────────────────────────────────────────────


def _run_measure(args: argparse.Namespace) -> int:
    try:
        import rclpy  # noqa: PLC0415 — ленивый импорт: summary и тесты без ROS
        from std_msgs.msg import Bool, Int32  # noqa: PLC0415
    except ImportError as e:
        print(
            f"[measure_doa] rclpy недоступен ({e}): measure запускается в контейнере "
            "voice-assistant после `source /opt/ros/humble/setup.bash`",
            file=sys.stderr,
        )
        return 1

    collector = DoaCollector(
        args.angle, settle_s=args.settle_s, min_samples=args.min_samples
    )
    rclpy.init()
    node = rclpy.create_node("measure_doa")
    node.create_subscription(
        Bool, "/audio/vad", lambda m: collector.on_vad(bool(m.data), time.time()), 10
    )
    node.create_subscription(
        Int32, "/audio/direction", lambda m: collector.on_doa(m.data, time.time()), 10
    )

    start = time.time()
    print(
        f"[measure_doa] угол {args.angle:g}°: говорите фразы по одной, нужно {args.samples} замеров"
        f" (таймаут {args.timeout:g} с)"
    )
    reported = 0
    rc = 0
    try:
        while len(collector.completed) < args.samples:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if collector.doa_msgs_total == 0 and now - start > args.no_doa_timeout:
                print(
                    f"[measure_doa] FAIL: за {args.no_doa_timeout:g} с ни одного /audio/direction. "
                    "Предусловие ADR-0137 §2.3: audio_node должен публиковать DOA.",
                    file=sys.stderr,
                )
                rc = 3
                break
            if now - start > args.timeout:
                print(
                    f"[measure_doa] таймаут: собрано {len(collector.completed)} из {args.samples}",
                    file=sys.stderr,
                )
                break
            if len(collector.completed) > reported:
                seg = collector.completed[-1]
                est = circular_mean([d for _, d in seg.samples])
                print(
                    f"  замер {len(collector.completed)}: {len(seg.samples)} отсчётов, "
                    f"DOA≈{est:.0f}° (сырой, без отображения)"
                )
                reported = len(collector.completed)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    run_id = time.strftime("%Y%m%dT%H%M%S")
    rows = collector.rows(run_id)
    if rows:
        append_csv(args.csv, rows)
    print(
        f"[measure_doa] записано {len(rows)} строк в {args.csv}; "
        f"коротких фраз отброшено: {collector.discarded}; всего DOA-сообщений: {collector.doa_msgs_total}"
    )
    return rc


def _run_summary(args: argparse.Namespace) -> int:
    segs = segments_from_rows(read_csv(args.csv))
    if not segs:
        print("[measure_doa] в CSV нет замеров", file=sys.stderr)
        return 2
    if (args.offset is None) != (args.sign is None):
        print("[measure_doa] --offset и --sign задаются вместе", file=sys.stderr)
        return 2
    s = summarize(segs, args.offset, args.sign, args.tolerance)
    print(format_summary(s, args.tolerance))
    return 0 if s.enough_data else 2


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser(
        "measure", help="собрать замеры на одном известном угле (нужен ROS)"
    )
    m.add_argument(
        "--angle",
        type=float,
        required=True,
        help="истинный азимут, ° (REP-103, от X base_link)",
    )
    m.add_argument(
        "--samples", type=int, default=3, help="сколько VAD-фраз собрать на этом угле"
    )
    m.add_argument("--csv", default="doa_measurements.csv")
    m.add_argument("--timeout", type=float, default=180.0)
    m.add_argument(
        "--no-doa-timeout",
        type=float,
        default=10.0,
        help="FAIL, если за это время нет ни одного /audio/direction",
    )
    m.add_argument("--settle-s", type=float, default=0.3)
    m.add_argument(
        "--min-samples", type=int, default=3, help="мин. отсчётов DOA в фразе"
    )

    s = sub.add_parser("summary", help="сводка и вердикт по CSV (без ROS)")
    s.add_argument("--csv", default="doa_measurements.csv")
    s.add_argument("--tolerance", type=float, default=TOLERANCE_DEG)
    s.add_argument(
        "--offset", type=float, default=None, help="известный offset, ° (иначе подбор)"
    )
    s.add_argument(
        "--sign",
        type=int,
        choices=(1, -1),
        default=None,
        help="известный знак (иначе подбор)",
    )
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.cmd == "measure":
        return _run_measure(args)
    return _run_summary(args)


if __name__ == "__main__":
    sys.exit(main())
