"""Самопроверка diversity.py: ``python -m pytest scripts/music/live_dj/test_diversity.py -v``."""
import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import diversity as D  # noqa: E402


def _tone(freq, seconds=6.0, noise=0.0, seed=0):
    t = np.arange(int(D.SR * seconds)) / D.SR
    y = 0.3 * np.sin(2 * np.pi * freq * t)
    return (y + noise * np.random.default_rng(seed).standard_normal(len(t))).astype(np.float32)


def test_pair_distance_identical_is_zero():
    v = np.arange(5.0)
    assert D.pair_distance(v, v) == 0.0


def test_distance_stats_identical_tracks_zero_and_different_larger():
    same = np.tile(np.arange(6.0), (4, 1))
    assert D.distance_stats(same)["all_median"] == 0.0
    assert D.distance_stats(same)["adj_min"] == 0.0
    varied = np.arange(24.0).reshape(4, 6)
    st = D.distance_stats(varied)
    assert st["all_median"] > 0 and st["adj_min"] > 0
    assert st["all_median"] >= st["adj_min"]


def test_distance_stats_single_track_is_nan():
    assert np.isnan(D.distance_stats(np.zeros((1, 3)))["adj_median"])


def test_normalizer_constant_feature_does_not_blow_up():
    mu, sigma = D.normalizer(np.array([[1.0, 5.0], [3.0, 5.0]]))
    assert sigma[1] == 1.0 and sigma[0] == 1.0


def test_fixed_windows_drop_partial_tail():
    assert D.fixed_windows(150.0, 60.0) == [(0.0, 60.0), (60.0, 60.0)]
    assert D.fixed_windows(600.0, 60.0, limit=3) == [(0.0, 60.0), (60.0, 60.0), (120.0, 60.0)]


def test_diversity_index_counts_distinct_per_axis():
    vecs = [{"pad": "sinepad", "lead": "pluck"}, {"pad": "sinepad", "lead": "arpy"}]
    idx = D.diversity_index(vecs, ("pad", "lead"))
    assert idx["distinct"] == {"pad": 1, "lead": 2}
    assert idx["share"] == {"pad": 0.5, "lead": 1.0}
    assert idx["index"] == pytest.approx(0.75)


def test_audio_identical_tracks_zero_different_tracks_farther():
    pytest.importorskip("librosa")
    a = D.track_features(_tone(220, noise=0.01, seed=1))
    same = [a, a.copy(), a.copy()]
    other = [a, D.track_features(_tone(1800, noise=0.2, seed=2)), D.track_features(_tone(600, noise=0.05, seed=3))]
    pool = np.vstack(same + other)
    mu, sigma = D.normalizer(pool)
    st_same = D.distance_stats((np.vstack(same) - mu) / sigma)
    st_other = D.distance_stats((np.vstack(other) - mu) / sigma)
    assert st_same["all_median"] == pytest.approx(0.0, abs=1e-9)
    assert st_other["all_median"] > 0.1


def test_pick_is_seeded_and_theme_text_is_injection_safe():
    archive = {f"n{i}": {"title": f"T{i}", "rtttl": "x"} for i in range(50)}
    assert D.pick_names(archive, 5, 3) == D.pick_names(archive, 5, 3)
    assert D.pick_names(archive, 5, 3) != D.pick_names(archive, 5, 4)
    assert D.theme_text("Rock 'n' \"Roll\" (live)") == "Rock n Roll live"


def test_offline_set_vector_has_all_axes():
    pytest.importorskip("rob_box_music")
    rec = {"title": "Test Tune", "rtttl": "t:d=8,o=5,b=140:c,e,g,e,c,e,g,e,a,c6,a,e,a,c6,a,e,f,a,c6,a,f,a,c6,a"}
    vecs = D.offline_set(rec, "t", seed=7, tracks=3)
    assert len(vecs) == 3
    assert set(D.AXES) <= set(vecs[0])  # + hook_fp (отпечаток, не ось)


def test_log_composition_reads_every_axis_from_started_lines(tmp_path):
    comp = {a: f"{a}{i}" for i, a in enumerate(D.AXES)}
    comp.update(bpm=128, energy=3)
    other = dict(comp, pad="other")
    lines = []
    for n, c in enumerate((comp, other, comp)):
        lines.append(f"[mcp_server-10] [INFO] [17912121{n}0.1] [mcp_server]: [set v2] s трек {n} started "
                     f"track_id=s:0{n}:A:ab form_end_beat=1.0 source=theme "
                     f"composition={json.dumps(c, ensure_ascii=False)}")
    log = tmp_path / "set1.full.log"
    log.write_text("\n".join(lines), encoding="utf-8")
    vecs = D.log_composition(str(log))
    assert len(vecs) == 3 and all(set(v) == set(D.AXES) for v in vecs)
    idx = D.diversity_index(vecs)
    assert idx["distinct"]["pad"] == 2 and idx["distinct"]["lead"] == 1
