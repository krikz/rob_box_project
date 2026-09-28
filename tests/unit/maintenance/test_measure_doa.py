"""test_measure_doa.py — чистые функции ``scripts/maintenance/measure_doa.py`` (ADR-0137 §2.3).

Без ROS: rclpy импортируется в скрипте лениво (только в ``measure``), здесь
проверяются круговая ошибка, сводка/вердикт по допуску Q17 (15°), подбор
отображения ``offset + sign·doa`` и сборщик замеров по фронтам VAD.

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


# ── сборщик по фронтам VAD ───────────────────────────────────────────────────

def test_collector_only_inside_vad_and_after_settle():
    c = md.DoaCollector(true_deg=90, settle_s=0.3, min_samples=3)
    c.on_doa(10, 0.0)                 # VAD ещё не было — игнор
    c.on_vad(True, 1.0)
    c.on_doa(200, 1.1)                # до settle — игнор
    for i, t in enumerate((1.4, 1.5, 1.6, 1.7)):
        c.on_doa(85 + i, t)
    c.on_vad(False, 2.0)
    c.on_doa(300, 2.1)                # после VAD — игнор
    assert c.doa_msgs_total == 7
    assert len(c.completed) == 1
    assert [d for _, d in c.completed[0].samples] == [85, 86, 87, 88]
    rows = c.rows("run")
    assert rows[0][1:5] == ["90", "85", "-5", "run-1"]


def test_collector_discards_short_phrase():
    c = md.DoaCollector(true_deg=0, settle_s=0.0, min_samples=3)
    c.on_vad(True, 0.0)
    c.on_doa(1, 0.1)
    c.on_vad(False, 0.2)
    assert c.completed == []
    assert c.discarded == 1


def test_csv_roundtrip_and_summary_cli(tmp_path):
    csv_path = tmp_path / "doa.csv"
    for angle in (0, 90, 180, 270):
        c = md.DoaCollector(true_deg=angle, settle_s=0.0, min_samples=1)
        for k in range(5):
            c.on_vad(True, k * 10.0)
            c.on_doa((angle + 4) % 360, k * 10.0 + 1)
            c.on_vad(False, k * 10.0 + 2)
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
    assert "замеров (VAD-фраз): 20" in res.stdout
    assert md.VERDICT_OK in res.stdout


def test_summary_cli_exit_2_on_insufficient_data(tmp_path):
    csv_path = tmp_path / "doa.csv"
    c = md.DoaCollector(true_deg=0, settle_s=0.0, min_samples=1)
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
