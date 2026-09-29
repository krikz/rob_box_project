"""test_measure_doa.py — чистые функции ``scripts/maintenance/measure_doa.py`` (ADR-0137 §2.3).

Без ROS: rclpy импортируется в скрипте лениво (только в ``measure``), здесь
проверяются круговая ошибка, сводка/вердикт по допуску Q17 (15°), подбор
отображения ``offset + sign·doa`` и нарезка окна речи на замеры при
мерцающем VAD (живой прогон 29.09.2026).

Run:
  python -m pytest tests/unit/maintenance/test_measure_doa.py -v -p no:cacheprovider --no-cov -o addopts=""
"""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "maintenance" / "measure_doa.py"


def _load():
    spec = importlib.util.spec_from_file_location("measure_doa", SCRIPT_PATH)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod  # нужно для @dataclass
    spec.loader.exec_module(mod)
    return mod


md = _load()


# ── круговая арифметика ──────────────────────────────────────────────────────

@pytest.mark.parametrize(
    "true_deg, measured, expected",
    [
        (0, 10, 10),
        (10, 0, -10),
        (359, 1, 2),      # через 0, а не 358
        (1, 359, -2),
        (0, 180, 180),    # ровно напротив — +180, не −180
        (90, 270, 180),
        (350, 20, 30),
        (0, 720 + 5, 5),
    ],
)
def test_circular_error_wraps(true_deg, measured, expected):
    assert md.circular_error(true_deg, measured) == pytest.approx(expected)


def test_circular_mean_across_zero():
    assert md.circular_mean([350, 10]) % 360 == pytest.approx(0, abs=1e-9)
    assert md.circular_mean([80, 100]) == pytest.approx(90)


def test_circular_mean_empty_raises():
    with pytest.raises(ValueError):
        md.circular_mean([])


def test_percentile_linear_interpolation():
    xs = list(range(1, 21))  # 1..20
    assert md.percentile(xs, 50) == pytest.approx(10.5)
    assert md.percentile(xs, 95) == pytest.approx(19.05)
    assert md.percentile([7], 95) == 7


# ── отображение и подбор ─────────────────────────────────────────────────────

def test_apply_mapping_sign_and_offset():
    assert md.apply_mapping(30, 0, 1) == pytest.approx(30)
    assert md.apply_mapping(30, 90, -1) == pytest.approx(60)
    assert md.apply_mapping(0, -90, 1) == pytest.approx(270)


@pytest.mark.parametrize("offset, sign", [(0, 1), (90, -1), (270, 1), (45, -1)])
def test_fit_mapping_recovers_convention(offset, sign):
    # Прошивка отдаёт doa так, что true = offset + sign·doa.
    pairs = []
    for true_deg in (0, 45, 90, 135, 180, 225, 270, 315):
        doa = ((true_deg - offset) * sign) % 360
        pairs.append((true_deg, doa))
    got_offset, got_sign = md.fit_mapping(pairs)
    assert got_sign == sign
    assert got_offset % 360 == pytest.approx(offset % 360)


# ── сводка и вердикт ─────────────────────────────────────────────────────────

def _segments(errors_by_angle, offset=0, sign=1, per_seg=5):
    """Сегменты с заданной ошибкой: сырой doa так, что mapped = true + err."""
    segs = []
    idx = 0
    for true_deg, errs in errors_by_angle.items():
        for err in errs:
            idx += 1
            mapped = true_deg + err
            doa = ((mapped - offset) * sign) % 360
            segs.append(md.Segment(index=idx, true_deg=true_deg,
                                   samples=[(float(i), doa) for i in range(per_seg)]))
    return segs


def test_summary_pass_when_p95_within_tolerance():
    errs = {a: [3, -5, 8, -2, 6] for a in (0, 90, 180, 270)}  # 20 замеров, |err| ≤ 8
    s = md.summarize(_segments(errs), offset_deg=0, sign=1)
    assert s.enough_data and s.passed
    assert s.max_err == pytest.approx(8)
    assert s.verdict == md.VERDICT_OK


def test_summary_fail_when_p95_above_tolerance():
    errs = {a: [20, -25, 18, -30, 22] for a in (0, 90, 180, 270)}
    s = md.summarize(_segments(errs), offset_deg=0, sign=1)
    assert s.enough_data and not s.passed
    assert s.p95 > md.TOLERANCE_DEG
    assert s.verdict == md.VERDICT_TURN_ONLY


def test_summary_not_enough_data_is_not_a_pass():
    errs = {0: [1, 1, 1], 90: [1, 1, 1]}  # 6 замеров, 2 угла
    s = md.summarize(_segments(errs), offset_deg=0, sign=1)
    assert not s.enough_data
    assert not s.passed
    assert s.verdict.startswith("НЕДОСТАТОЧНО ДАННЫХ")


def test_summary_fits_unknown_convention():
    errs = {a: [2, -3, 4, -1, 0] for a in (0, 90, 180, 270)}
    segs = _segments(errs, offset=90, sign=-1)
    s = md.summarize(segs)  # без offset/sign — подбор
    assert s.fitted
    assert (s.offset_deg, s.sign) == (90.0, -1)
    assert s.passed


# ── окно речи при мерцающем VAD (живой прогон 29.09.2026) ────────────────────

# Фронты /audio/vad на сплошной речи, как записаны на Vision Pi:
# +1.2↑ 1.3↓ 1.6↑ 1.8↓ 2.0↑ 2.2↓ 2.5↑ 2.7↓ с — фрагменты VAD=1 по 0.1–0.3 с.
_FLICKER = [(0.0, True), (0.1, False), (0.4, True), (0.6, False),
            (0.8, True), (1.0, False), (1.3, True), (1.5, False)]
_CYCLE_S = 2.4  # цикл + вдох: 8⅓ цикла на 20 с → 68 фронтов (на роботе 54–75)


def _flicker_edges(t0, duration_s):
    edges = []
    k = 0
    while t0 + k * _CYCLE_S < t0 + duration_s:
        base = t0 + k * _CYCLE_S
        edges += [(base + dt, a) for dt, a in _FLICKER if base + dt < t0 + duration_s]
        k += 1
    return edges


def _doa_10hz(t0, duration_s, deg=90.0, jitter=(0, 4, -3, 2, -5)):
    n = int(round(duration_s * 10))
    return [(t0 + i * 0.1 + 0.05, (deg + jitter[i % len(jitter)]) % 360) for i in range(n)]


def test_flicker_pattern_matches_robot_edge_rate():
    edges = _flicker_edges(1.2, 20.0)
    assert 54 <= len(edges) <= 75
    assert [round(t, 1) for t, _ in edges[:8]] == [1.2, 1.3, 1.6, 1.8, 2.0, 2.2, 2.5, 2.7]


def test_old_per_phrase_rule_yields_nothing_on_flicker():
    """Регресс-свидетель: «одна VAD-фраза, settle 0.3 с, ≥ 3 отсчёта» → 0 замеров."""
    edges = _flicker_edges(1.2, 20.0)
    doa = _doa_10hz(1.2, 20.0)
    rises = [t for t, a in edges if a]
    falls = [t for t, a in edges if not a]
    per_phrase = [sum(1 for t, _ in doa if r + 0.3 <= t < f) for r, f in zip(rises, falls)]
    assert len(per_phrase) == 34
    assert max(per_phrase) < 3  # ни одна фраза не набирает min_samples


def test_pool_window_flicker_gives_measurement_per_split():
    start = 1.2
    res = md.pool_window(_flicker_edges(start, 20.0), _doa_10hz(start, 20.0),
                         true_deg=90, start_t=start, window_s=20.0,
                         splits=4, hold_s=0.5, min_samples=3)
    assert [s.index for s in res.segments] == [1, 2, 3, 4]
    assert res.discarded == 0
    # в цикле 2.4 с речью считается 1.5 + 0.5 (hold) = 2.0 с → ~5/6 отсчётов
    assert res.pooled + res.silent == 200
    assert 150 <= res.pooled <= 175
    for seg in res.segments:
        assert len(seg.samples) >= 30
        assert all(start + (seg.index - 1) * 5 <= t < start + seg.index * 5
                   for t, _ in seg.samples)
        assert abs(md.circular_error(90, md.circular_mean([d for _, d in seg.samples]))) < 2


def test_pool_window_hold_zero_keeps_only_vad_true():
    start = 1.2
    res = md.pool_window(_flicker_edges(start, 20.0), _doa_10hz(start, 20.0),
                         true_deg=90, start_t=start, window_s=20.0, splits=1, hold_s=0.0)
    # VAD=1 занимает 0.7 с из 2.4 с цикла → ~29 % отсчётов
    assert 45 <= res.pooled <= 70


def test_pool_window_ignores_tts_echo_before_window():
    # Эхо собственного TTS до старта окна: VAD=1 и DOA=270 — не должно попасть.
    edges = [(0.0, True), (0.8, False)] + _flicker_edges(3.0, 10.0)
    doa = [(t, 270.0) for t in (0.1, 0.3, 0.5, 0.7)] + _doa_10hz(3.0, 10.0)
    res = md.pool_window(edges, doa, true_deg=90, start_t=3.0, window_s=10.0,
                         splits=1, hold_s=0.5)
    assert len(res.segments) == 1
    assert all(d != 270.0 for _, d in res.segments[0].samples)


def test_pool_window_silence_longer_than_hold_is_dropped():
    edges = [(0.0, True), (1.0, False), (3.0, True), (4.0, False)]
    doa = [(t / 10, 45.0) for t in range(50)]  # 0.0 .. 4.9
    res = md.pool_window(edges, doa, true_deg=45, start_t=0.0, window_s=5.0,
                         splits=1, hold_s=0.5)
    times = [t for t, _ in res.segments[0].samples]
    assert all(t <= 1.5 or 3.0 <= t <= 4.5 for t in times)
    assert res.pooled == 16 + 16 and res.silent == 50 - 32


def test_pool_window_vad_rise_before_window_still_counts():
    res = md.pool_window([(-5.0, True)], [(0.5, 10.0), (1.5, 12.0), (2.5, 14.0)],
                         true_deg=10, start_t=0.0, window_s=3.0, splits=1)
    assert res.pooled == 3


def test_pool_window_no_vad_edges_means_no_speech():
    res = md.pool_window([], _doa_10hz(0.0, 5.0), true_deg=90, start_t=0.0, window_s=5.0)
    assert res.segments == [] and res.pooled == 0 and res.discarded == 1


def test_pool_window_quiet_split_is_discarded():
    # речь только в первой половине окна → второе подокно в брак
    edges = _flicker_edges(0.0, 10.0)
    res = md.pool_window(edges, _doa_10hz(0.0, 20.0), true_deg=90, start_t=0.0,
                         window_s=20.0, splits=2, hold_s=0.5)
    assert [s.index for s in res.segments] == [1]
    assert res.discarded == 1


@pytest.mark.parametrize("window_s, splits", [(0.0, 1), (-1.0, 1), (10.0, 0)])
def test_pool_window_rejects_bad_args(window_s, splits):
    with pytest.raises(ValueError):
        md.pool_window([], [], 0, 0.0, window_s, splits=splits)


def test_collector_rows_follow_pool_window():
    c = md.DoaCollector(true_deg=90, start_t=1.2, window_s=20.0, splits=4, hold_s=0.5)
    for t, a in _flicker_edges(1.2, 20.0):
        c.on_vad(a, t)
    for t, d in _doa_10hz(1.2, 20.0):
        c.on_doa(d, t)
    assert c.doa_msgs_total == 200
    assert c.vad_edges_in_window() == 68
    rows = c.rows("run")
    assert {r[4] for r in rows} == {"run-1", "run-2", "run-3", "run-4"}
    assert rows[0][1:4] == ["90", "90", "0"]
    segs = md.segments_from_rows(
        [dict(zip(md.CSV_HEADER, r)) for r in rows])
    assert len(segs) == 4


def test_csv_roundtrip_and_summary_cli(tmp_path):
    csv_path = tmp_path / "doa.csv"
    for angle in (0, 90, 180, 270):
        c = md.DoaCollector(true_deg=angle, start_t=0.0, window_s=20.0, splits=5, min_samples=1)
        for k in range(5):
            c.on_vad(True, k * 4.0)
            c.on_doa((angle + 4) % 360, k * 4.0 + 1)
            c.on_vad(False, k * 4.0 + 2)
        md.append_csv(str(csv_path), c.rows(f"a{angle}"))
    header = csv_path.read_text(encoding="utf-8").splitlines()[0]
    assert header == ",".join(md.CSV_HEADER)

    # summary — отдельным процессом, без ROS в окружении.
    res = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "summary", "--csv", str(csv_path),
         "--offset", "0", "--sign", "1"],
        capture_output=True, text=True, timeout=60,
    )
    assert res.returncode == 0, res.stderr
    assert "замеров (окон/подокон): 20" in res.stdout
    assert md.VERDICT_OK in res.stdout


def test_summary_cli_exit_2_on_insufficient_data(tmp_path):
    csv_path = tmp_path / "doa.csv"
    c = md.DoaCollector(true_deg=0, start_t=0.0, window_s=2.0, min_samples=1)
    c.on_vad(True, 0.0)
    c.on_doa(3, 0.5)
    c.on_vad(False, 1.0)
    md.append_csv(str(csv_path), c.rows("x"))
    res = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "summary", "--csv", str(csv_path)],
        capture_output=True, text=True, timeout=60,
    )
    assert res.returncode == 2
    assert "НЕДОСТАТОЧНО ДАННЫХ" in res.stdout
