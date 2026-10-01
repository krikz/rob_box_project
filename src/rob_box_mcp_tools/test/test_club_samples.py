"""Тесты слоя сэмплов DJ_Dave в club (issue #3254, ADR-0146)."""

import re
from collections import Counter

import pytest

from rob_box_mcp_tools.core import sample_dave
from rob_box_mcp_tools.core.club_arranger import render_club
from rob_box_mcp_tools.core.club_history import remember_club
from rob_box_mcp_tools.core.club_samples import (
    SAMPLE_LAYERS,
    SAMPLE_SLOT,
    pick_club_sample,
    sample_sentence,
)
from rob_box_mcp_tools.core.music_diversity import MusicHistory
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

pytestmark = pytest.mark.unit


@pytest.fixture
def pack_root(tmp_path):
    """Корень сэмплов, где все файлы белого списка слоя «лежат на диске»."""
    for layer in SAMPLE_LAYERS.values():
        rel = sample_dave.find_sample(layer.sample).path.replace("../../", "", 1)
        path = tmp_path / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"RIFF")
    return str(tmp_path)


def test_whitelist_layers_reference_catalog_samples():
    for name, layer in SAMPLE_LAYERS.items():
        assert sample_dave.find_sample(layer.sample) is not None, name
        assert layer.kind in ("step", "stretch", "chop")


@pytest.mark.parametrize("name", sorted(SAMPLE_LAYERS))
def test_render_has_loop_with_whitelisted_path_after_sanitizer(name):
    code = render_club(seed=7, root="A#", sample=name)
    d3 = [line for line in code.splitlines() if line.startswith(f"{SAMPLE_SLOT} >>")]
    assert len(d3) == 1 and "loop(" in d3[0] and "play(" not in d3[0]
    result = sanitize_renando(code, 0.85, pack1_loops_enabled=True)
    assert not result.quality_errors and result.slot_error is None
    path = sample_dave.find_sample(SAMPLE_LAYERS[name].sample).path
    assert f"loop({path!r}" in result.code
    assert path.startswith("../../dj_dave/")


def test_render_without_sample_is_unchanged_clap():
    code = render_club(seed=7)
    assert code == render_club(seed=7, sample=None)
    assert "loop(" not in code and re.search(r'^d3 >> play\("', code, re.M)


def test_sample_level_follows_calibrated_kick_and_perc_lane():
    code = render_club(seed=7, sample="psr_10")
    d3 = next(line for line in code.splitlines() if line.startswith("d3 >>"))
    levels = [float(v) for v in re.search(r"amp=var\(\[([^\]]*)\]", d3).group(1).split(",")]
    assert max(levels) <= 0.85 and min(levels) == 0  # секции без перкуссии молчат
    assert "amplify=[" in d3  # пампинг бочки, как у баса/лида


def test_no_pack_flag_no_layer_with_reason(pack_root):
    pick = pick_club_sample(5, [], "A#", enabled=False, samples_root=pack_root)
    assert pick.name is None and "ROB_BOX_PACK1_LOOPS" in pick.reason
    assert pick.info()["layer"] is None
    assert "нет" in sample_sentence(pick.info())


def test_no_files_on_disk_no_layer_with_reason(tmp_path):
    pick = pick_club_sample(5, [], "A#", enabled=True, samples_root=str(tmp_path / "empty"))
    assert pick.name is None and "нет на диске" in pick.reason


def test_seed_zero_reference_without_samples(pack_root):
    assert pick_club_sample(0, [], "A#", enabled=True, samples_root=pack_root).name is None


def test_pack_present_picks_layer_and_reports_it(pack_root):
    pick = pick_club_sample(5, [], "A#", enabled=True, samples_root=pack_root)
    assert pick.name in SAMPLE_LAYERS
    info = pick.info()
    assert info["slot"] == "d3" and info["path"].startswith("../../dj_dave/")
    assert info["layer"] in sample_sentence(info)


def test_vocal_chop_only_in_native_key(pack_root):
    names = {pick_club_sample(s, [], "C", enabled=True, samples_root=pack_root).name for s in range(1, 200)}
    assert "spilltab_chop" not in names
    names_a = {pick_club_sample(s, [], "A#", enabled=True, samples_root=pack_root).name for s in range(1, 200)}
    assert "spilltab_chop" in names_a


def test_repeat_penalty_from_history(pack_root):
    recent = [{"sample": "psr_10"}] * 3
    fresh = Counter(pick_club_sample(s, [], "C", enabled=True, samples_root=pack_root).name for s in range(1, 301))
    penal = Counter(pick_club_sample(s, recent, "C", enabled=True, samples_root=pack_root).name for s in range(1, 301))
    assert fresh["psr_10"] > 20
    assert penal["psr_10"] < fresh["psr_10"] / 4


def test_remember_club_writes_sample_and_drops_clap():
    history = MusicHistory(":memory:")
    kit = {"template": "dj_dave_32", "kick": "by_design", "hats": "by_design",
           "lead": "pluck", "bass": "bass", "pad": "sinepad"}
    line = remember_club(history, {}, kit, 5, 124, None, [{"clap": "dry"}], "wun_beat")
    row = history.recent()[0]
    assert row["sample"] == "wun_beat" and row["clap"] is None
    assert "sample=wun_beat" in line
    remember_club(history, {}, kit, 5, 124, None, [{"clap": "dry"}])
    row = history.recent()[0]
    assert row["sample"] is None and row["clap"]
