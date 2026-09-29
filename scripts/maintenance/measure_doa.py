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
  * Один «замер» = одно окно речи, отмеренное оператором: скрипт ждёт
    ``--start-delay-s`` (гаснет эхо своего TTS, оператор готовится), затем
    ``--window-s`` секунд копит DOA. Отсчёт берётся, если VAD == True или с
    последнего спада VAD прошло ≤ ``--hold-s``: VAD audio_node на сплошной
    речи мерцает (фрагменты 0.1–0.3 с, 54–75 фронтов за 20 с — живой прогон
    29.09.2026), и прежнее «одна VAD-фраза = один замер» не давало ни одного
    замера. Окно делится на ``--splits`` равных подокон — каждое с
    ≥ ``--min-samples`` отсчётов даёт свой замер; оценка замера — круговое
    среднее его отсчётов.
  * Нужно ≥ 20 замеров суммарно по ≥ 4 углам (минимум 0/90/180/270).
    Подокна одного окна не независимы (тот же человек, та же поза) — см.
    ADR-0137 §2.3.
  * Помещение влияет на результат (отражения тянут DOA к стенам): в большом
    открытом помещении мерится точность самого массива, в рабочей комнате —
    то, что увидит этап 5. Разные места — разные ``--csv``.

Запуск на Vision Pi (скрипт в контейнер не смонтирован — копируем):
    docker cp scripts/maintenance/measure_doa.py voice-assistant:/tmp/measure_doa.py
    docker exec -it voice-assistant bash -lc \\
      'source /opt/ros/humble/setup.bash && \\
       python3 /tmp/measure_doa.py measure --angle 90 --window-s 20 --splits 4 --csv /tmp/doa.csv'
    ... повторить для других углов (CSV дописывается) ...
    docker exec voice-assistant python3 /tmp/measure_doa.py summary --csv /tmp/doa.csv

``summary`` не требует ROS и работает где угодно. Направление 0° прошивки и
знак отсчёта неизвестны (ADR-0137 §4), поэтому по умолчанию ``summary``
подбирает отображение ``bearing = offset + sign * doa`` по данным; если
оно уже известно — ``--offset/--sign`` фиксируют его.

Коды выхода: 0 — есть вердикт (или замер записан); 1 — нет rclpy;
2 — данных недостаточно для вердикта (или окно не дало ни одного замера);
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
    """Один замер: окно речи (или его подокно) на известном угле."""

    index: int
    true_deg: float
    samples: List[Tuple[float, float]] = field(default_factory=list)  # (ts, doa_deg)


def speaking_mask(
    vad_edges: Sequence[Tuple[float, bool]],
    times: Sequence[float],
    hold_s: float,
) -> List[bool]:
    """Для каждого момента ``times`` (по возрастанию) — «идёт речь?».

    ``vad_edges`` — (t, active); ``/audio/vad`` шлёт только фронты, до
    первого фронта считаем VAD == False. Речь идёт, если VAD == True или с
    последнего спада прошло ≤ ``hold_s`` — это склеивает мерцание VAD на
    сплошной речи в одно «говорит».
    """
    edges = sorted(vad_edges, key=lambda e: e[0])
    out: List[bool] = []
    i = 0
    vad = False
    last_fall: Optional[float] = None
    for t in times:
        while i < len(edges) and edges[i][0] <= t:
            active = bool(edges[i][1])
            if vad and not active:
                last_fall = edges[i][0]
            vad = active
            i += 1
        out.append(vad or (last_fall is not None and t - last_fall <= hold_s))
    return out


@dataclass
class WindowResult:
    segments: List[Segment]
    discarded: int  # подокна, где отсчётов < min_samples
    pooled: int  # отсчётов DOA в окне, пришедшихся на речь
    silent: int  # отсчётов DOA в окне вне речи


def pool_window(
    vad_edges: Sequence[Tuple[float, bool]],
    doa: Sequence[Tuple[float, float]],
    true_deg: float,
    start_t: float,
    window_s: float,
    splits: int = 1,
    hold_s: float = 0.5,
    min_samples: int = 3,
) -> WindowResult:
    """Нарезать одно окно речи [start_t, start_t + window_s) на замеры.

    Отсчёты DOA вне окна игнорируются, в окне — берутся, если идёт речь
    (``speaking_mask``). Окно делится на ``splits`` равных подокон; подокно
    с ≥ ``min_samples`` отсчётов — замер (index 1..splits по времени),
    иначе — в брак.
    """
    if window_s <= 0 or splits < 1:
        raise ValueError("pool_window: нужно window_s > 0 и splits ≥ 1")
    end_t = start_t + window_s
    in_win = sorted((p for p in doa if start_t <= p[0] < end_t), key=lambda p: p[0])
    mask = speaking_mask(vad_edges, [t for t, _ in in_win], hold_s)
    pooled = [(t, float(d) % 360.0) for (t, d), m in zip(in_win, mask) if m]

    sub_s = window_s / splits
    buckets: List[List[Tuple[float, float]]] = [[] for _ in range(splits)]
    for t, d in pooled:
        buckets[min(int((t - start_t) / sub_s), splits - 1)].append((t, d))

    segs: List[Segment] = []
    discarded = 0
    for k, b in enumerate(buckets, start=1):
        if len(b) >= min_samples:
            segs.append(Segment(index=k, true_deg=true_deg, samples=b))
        else:
            discarded += 1
    return WindowResult(
        segments=segs,
        discarded=discarded,
        pooled=len(pooled),
        silent=len(in_win) - len(pooled),
    )


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
    """Копит сырые события одного окна; нарезка — чистой ``pool_window``.

    Без ROS: время передаётся явно. Хранит всё пришедшее — что считать
    речью, решается один раз по полному окну.
    """

    def __init__(
        self,
        true_deg: float,
        start_t: float,
        window_s: float,
        splits: int = 1,
        hold_s: float = 0.5,
        min_samples: int = 3,
    ):
        self.true_deg = true_deg
        self.start_t = start_t
        self.window_s = window_s
        self.splits = splits
        self.hold_s = hold_s
        self.min_samples = min_samples
        self.vad_edges: List[Tuple[float, bool]] = []
        self.doa: List[Tuple[float, float]] = []
        self.doa_msgs_total = 0

    @property
    def end_t(self) -> float:
        return self.start_t + self.window_s

    def on_vad(self, active: bool, t: float) -> None:
        self.vad_edges.append((t, bool(active)))

    def on_doa(self, doa_deg: float, t: float) -> None:
        self.doa_msgs_total += 1
        self.doa.append((t, float(doa_deg)))

    def result(self) -> WindowResult:
        return pool_window(
            self.vad_edges,
            self.doa,
            self.true_deg,
            self.start_t,
            self.window_s,
            splits=self.splits,
            hold_s=self.hold_s,
            min_samples=self.min_samples,
        )

    def vad_edges_in_window(self) -> int:
        return sum(1 for t, _ in self.vad_edges if self.start_t <= t < self.end_t)

    def rows(self, run_id: str) -> List[List[str]]:
        """Строки CSV (ts, true_deg, doa_deg, err_deg, segment) — err без отображения."""
        out = []
        for seg in self.result().segments:
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
            f"замеров (окон/подокон): {s.n_segments}, отсчётов DOA: {s.n_samples}",
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

    rclpy.init()
    node = rclpy.create_node("measure_doa")
    collector = DoaCollector(
        args.angle,
        start_t=time.time() + args.start_delay_s,
        window_s=args.window_s,
        splits=args.splits,
        hold_s=args.hold_s,
        min_samples=args.min_samples,
    )
    node.create_subscription(
        Bool, "/audio/vad", lambda m: collector.on_vad(bool(m.data), time.time()), 10
    )
    node.create_subscription(
        Int32, "/audio/direction", lambda m: collector.on_doa(m.data, time.time()), 10
    )

    begin = time.time()
    print(
        f"[measure_doa] угол {args.angle:g}°: через {args.start_delay_s:g} с говорите "
        f"непрерывно {args.window_s:g} с (подокон: {args.splits})"
    )
    rc = 0
    announced = False
    try:
        while time.time() < collector.end_t:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if not announced and now >= collector.start_t:
                print("[measure_doa] ГОВОРИТЕ")
                announced = True
            if collector.doa_msgs_total == 0 and now - begin > args.no_doa_timeout:
                print(
                    f"[measure_doa] FAIL: за {args.no_doa_timeout:g} с ни одного /audio/direction. "
                    "Предусловие ADR-0137 §2.3: audio_node должен публиковать DOA.",
                    file=sys.stderr,
                )
                rc = 3
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    res = collector.result()
    print("[measure_doa] СТОП")
    for seg in res.segments:
        est = circular_mean([d for _, d in seg.samples])
        print(
            f"  замер {seg.index}/{args.splits}: {len(seg.samples)} отсчётов, "
            f"DOA≈{est:.0f}° (сырой, без отображения)"
        )
    run_id = time.strftime("%Y%m%dT%H%M%S")
    rows = collector.rows(run_id)
    if rows:
        append_csv(args.csv, rows)
    print(
        f"[measure_doa] записано {len(rows)} строк ({len(res.segments)} замеров) в {args.csv}; "
        f"подокон в брак (< {args.min_samples} отсчётов): {res.discarded}; "
        f"DOA в речи/вне речи: {res.pooled}/{res.silent}; "
        f"фронтов VAD в окне: {collector.vad_edges_in_window()}; "
        f"всего DOA-сообщений: {collector.doa_msgs_total}"
    )
    if rc == 0 and not res.segments:
        rc = 2
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
    m.add_argument("--csv", default="doa_measurements.csv")
    m.add_argument(
        "--window-s",
        type=float,
        default=20.0,
        help="длительность окна речи, с (оператор говорит непрерывно)",
    )
    m.add_argument(
        "--splits",
        type=int,
        default=4,
        help="на сколько равных подокон делить окно (каждое — отдельный замер)",
    )
    m.add_argument(
        "--start-delay-s",
        type=float,
        default=3.0,
        help="пауза до начала окна, с (эхо своего TTS, оператор готовится)",
    )
    m.add_argument(
        "--hold-s",
        type=float,
        default=0.5,
        help="сколько после спада VAD ещё считать речью, с (мерцание VAD)",
    )
    m.add_argument(
        "--no-doa-timeout",
        type=float,
        default=10.0,
        help="FAIL, если за это время нет ни одного /audio/direction",
    )
    m.add_argument(
        "--min-samples", type=int, default=3, help="мин. отсчётов DOA в подокне"
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
