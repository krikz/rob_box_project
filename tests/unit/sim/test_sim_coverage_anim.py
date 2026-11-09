"""Анимация обхода покрытием (#4560): число кадров, совпадение с метрикой, лимит размера GIF. Без ROS."""

import json

import numpy as np
import pytest
from rob_box_sim.coverage_metric import coverage_result, covered_mask, parse_track_log
from rob_box_sim.coverage_plan import coverage_space, load_grid_npz, save_grid_npz
from rob_box_sim.map_check import OccupancyGrid

pytest.importorskip("PIL")  # в офлайн-CI Pillow нет; модуль — инструмент студии/разработчика

from PIL import Image  # noqa: E402
from rob_box_sim import coverage_anim as ca  # noqa: E402

RES = 0.05


def hall_grid(w=100, h=80):
    a = np.zeros((h, w), dtype=np.int16)
    a[0, :] = a[-1, :] = a[:, 0] = a[:, -1] = 100
    return OccupancyGrid(w, h, RES, 0.0, 0.0, a.reshape(-1).tolist())


def synth_log(n=120):
    """Лог как у route.py: проезд по полосе y=1.5 слева направо, стоп защиты посередине."""
    lines = []
    for i in range(n):
        x = 1.0 + 3.0 * i / (n - 1)
        lines.append(f"12:00:{i % 60:02d}.0 TRACK sim={100 + i * 0.2:.2f} x={x:.3f} y=1.500 yaw=0.0")
        if i == n // 2:
            lines.append(f"12:00:00.0 EM_ON sim={100 + i * 0.2:.2f}")
    return lines


@pytest.fixture
def run_files(tmp_path):
    save_grid_npz(hall_grid(), tmp_path / "plan_map.npz")
    (tmp_path / "coverage.log").write_text("\n".join(synth_log()), encoding="utf-8")
    poses = [{"x": 1.0, "y": 1.5, "yaw": 0.0}, {"x": 4.0, "y": 1.5, "yaw": 0.0}]
    (tmp_path / "plan.json").write_text(json.dumps({"poses": poses}), encoding="utf-8")
    return tmp_path


def test_frame_ends_count_and_last():
    assert len(ca.frame_ends(120, 10)) == 10
    assert ca.frame_ends(120, 10)[-1] == 120
    assert ca.frame_ends(3, 10) == [1, 2, 3]
    assert ca.frame_ends(1, 5) == [1]


def test_parse_em_times():
    assert ca.parse_em_times(["x EM_ON sim=12.5", "EM_OFF sim=13", "EM_ON sim=20.00"]) == [12.5, 20.0]


def test_build_frames_count_and_last_matches_metric(run_files):
    m, log, plan = run_files / "plan_map.npz", run_files / "coverage.log", run_files / "plan.json"
    frames, covered = ca.build(m, log, plan, frames=12)
    assert len(frames) == len(covered) == 12
    assert covered == sorted(covered) and covered[0] < covered[-1]  # покрытие только растёт
    track = parse_track_log(log.read_text(encoding="utf-8").splitlines())
    space = coverage_space(load_grid_npz(m), track[0][1:3])
    expected = coverage_result(space, covered_mask(space, track)).covered_m2
    assert expected > 1.0
    assert covered[-1] == pytest.approx(expected)


def test_chunked_accumulation_equals_whole_track(run_files):
    """Копление по кускам трека не теряет звенья на границах: 3 кадра и 40 кадров дают одно и то же."""
    m, log = run_files / "plan_map.npz", run_files / "coverage.log"
    assert ca.build(m, log, frames=3)[1][-1] == pytest.approx(ca.build(m, log, frames=40)[1][-1])


def test_empty_track_is_error(run_files):
    (run_files / "empty.log").write_text("nothing here", encoding="utf-8")
    with pytest.raises(ValueError):
        ca.build(run_files / "plan_map.npz", run_files / "empty.log")


def test_save_gif_respects_limit_and_keeps_last_frame(run_files, tmp_path):
    frames, _ = ca.build(run_files / "plan_map.npz", run_files / "coverage.log", frames=40)
    size_big, n_big, _ = ca.save_gif(frames, tmp_path / "big.gif", max_bytes=10**9)
    assert n_big == 40
    small = tmp_path / "small.gif"
    limit = size_big // 3
    size, n, width = ca.save_gif(frames, small, max_bytes=limit)
    assert size <= limit and size == small.stat().st_size
    assert n < n_big
    with Image.open(small) as g:
        assert g.n_frames == n and g.size[0] == width


def test_cli_writes_gif(run_files, tmp_path, capsys):
    out = tmp_path / "o.gif"
    rc = ca.main([str(run_files / "plan_map.npz"), str(run_files / "coverage.log"), str(out), "--frames", "10"])
    assert rc == 0 and out.stat().st_size > 0
    assert "covered_m2=" in capsys.readouterr().out
