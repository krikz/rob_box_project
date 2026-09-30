"""Issue #3219 — сэмплы DJ_Dave: каталог, белый список и проводка к санитайзеру.

Сами файлы в git не лежат (нет лицензии у источников), их ставит Ресурсный
пак. Тест держит три вещи:

* каталог ``data/sample_dave.json`` не расходится с lock-файлом фетчера —
  иначе ``loop('<имя>')`` указал бы на файл, которого хук не скачает;
* пак за тем же флагом, что лупы/FX пака 1 (по умолчанию выключен);
* санитайзер переписывает имя в путь от папки лупов пака 0.
"""

from __future__ import annotations

import json
from pathlib import Path

from rob_box_mcp_tools.core import sample_dave, sample_loops
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

MAX_AMP = 0.85
LOCK = (
    Path(__file__).resolve().parents[3]
    / "docker" / "vision" / "scripts" / "resource_pack" / "dj_dave_samples.lock.json"
)


def _lock_dests() -> set:
    return {e["dest"] for e in json.loads(LOCK.read_text(encoding="utf-8"))["files"]}


def test_catalog_matches_fetcher_lock_exactly():
    catalog_paths = {i.path.split("/", 3)[3] for i in sample_dave.sample_catalog().values()}
    assert catalog_paths == _lock_dests()


def test_catalog_path_leaves_loop_dir_of_pack0():
    info = sample_dave.sample_catalog()["algorave_spilltab"]
    assert info.path == "../../dj_dave/algorave/spilltab/spilltab.wav"
    assert info.channels == 2 and 19 < info.seconds < 20


def test_every_sample_has_a_described_group():
    groups = sample_dave.group_descriptions()
    assert {i.group for i in sample_dave.sample_catalog().values()} <= set(groups)
    assert sample_dave.names_in_group("dirt_psr") and len(sample_dave.names_in_group("dirt_psr")) == 30


def test_find_accepts_name_and_canonical_path_only():
    by_name = sample_dave.find_sample("array_perc_kick")
    assert by_name is not None
    assert sample_dave.find_sample(by_name.path) is by_name
    assert sample_dave.find_sample("no_such_thing") is None


def test_denied_without_flag_and_for_unknown_names():
    assert sample_dave.sample_denial("array_perc_kick", enabled=True) is None
    off = sample_dave.sample_denial("array_perc_kick", enabled=False)
    assert off is not None and sample_loops.PACK1_LOOPS_ENV in off
    assert "нет в каталоге" in sample_dave.sample_denial("nope", enabled=True)


def test_sanitizer_rewrites_name_to_path_when_enabled():
    result = sanitize_renando("d1 >> loop('array_perc_kick', dur=1, amp=0.5)", MAX_AMP, pack1_loops_enabled=True)
    assert result.quality_errors == ()
    assert "loop('../../dj_dave/array/perc/kick.mp3'" in result.code


def test_sanitizer_blocks_pack_when_flag_is_off():
    result = sanitize_renando("d1 >> loop('array_perc_kick', dur=1)", MAX_AMP)
    assert result.quality_errors and sample_loops.PACK1_LOOPS_ENV in result.quality_errors[0]


def test_sanitizer_is_stable_on_second_pass():
    once = sanitize_renando("d1 >> loop('algorave_spilltab', dur=8)", MAX_AMP, pack1_loops_enabled=True)
    twice = sanitize_renando(once.code, MAX_AMP, pack1_loops_enabled=True)
    assert twice.quality_errors == () and twice.code == once.code


def test_sanitizer_rejects_path_outside_the_whitelist():
    result = sanitize_renando("d1 >> loop('../../dj_dave/../../etc/passwd', dur=1)", MAX_AMP, pack1_loops_enabled=True)
    assert result.quality_errors
    assert "DJ_Dave" in result.quality_errors[0]


def test_unknown_name_error_points_to_dave_groups():
    result = sanitize_renando("d1 >> loop('nope', dur=1)", MAX_AMP, pack1_loops_enabled=True)
    assert "array_vox" in result.quality_errors[0]
