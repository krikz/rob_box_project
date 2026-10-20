"""Каталоги сэмпл-паков Sonic Pi и MuldjordKit (решение 07.10, ADR-0153 S3/S5 их ещё не подключили).

Файлов в репозитории нет: их ставит Ресурсный пак. Тест держит каталог равным lock-файлу фетчера —
иначе генератор указал бы на файл, которого хук не скачает.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
LOCK_DIR = REPO_ROOT / "docker" / "vision" / "scripts" / "resource_pack"
DATA_DIR = REPO_ROOT / "src" / "rob_box_music" / "rob_box_music" / "data"
DRUM_ROLES = {"kick", "snare", "tom", "hat", "ride", "crash", "break", "loop"}

PACKS = {
    "sonicpi": ("sonicpi_samples.lock.json", "sample_sonicpi.json", "sonicpi", 206),
    "muldjord": ("muldjord_kit.lock.json", "sample_muldjord.json", "muldjord", 32),
}


def _load(pack: str):
    lock_name, data_name, _pack_dir, _count = PACKS[pack]
    lock = json.loads((LOCK_DIR / lock_name).read_text(encoding="utf-8"))
    catalog = json.loads((DATA_DIR / data_name).read_text(encoding="utf-8"))
    return lock, catalog


@pytest.mark.parametrize("pack", sorted(PACKS))
def test_catalog_paths_equal_lock_audio_dests(pack):
    lock, catalog = _load(pack)
    lock_audio = {e["dest"] for e in lock["files"] if e["dest"].endswith((".flac", ".wav", ".mp3"))}
    assert {s["path"] for s in catalog["samples"].values()} == lock_audio
    assert len(catalog["samples"]) == PACKS[pack][3]
    assert catalog["pack_dir"] == PACKS[pack][2]


@pytest.mark.parametrize("pack", sorted(PACKS))
def test_catalog_entries_are_well_formed(pack):
    _, catalog = _load(pack)
    roles = set(catalog["roles"])
    assert DRUM_ROLES <= roles
    for name, info in catalog["samples"].items():
        assert name.startswith(pack), name
        assert info["role"] in roles, (name, info)
        assert info["group"] in catalog["groups"], (name, info)
        assert info["channels"] in (1, 2) and info["samplerate"] >= 16000, (name, info)
        assert 0.01 < info["seconds"] < 60, (name, info)
        assert all(math.isfinite(info[k]) and info[k] <= 0.5 for k in ("peak_db", "rms_db")), (name, info)
        assert info["rms_db"] <= info["peak_db"], (name, info)


def test_sonicpi_breaks_and_core_drums_have_roles():
    _, catalog = _load("sonicpi")
    s = catalog["samples"]
    assert {n for n, i in s.items() if i["role"] == "break"} == {
        "sonicpi_loop_amen", "sonicpi_loop_amen_full", "sonicpi_loop_breakbeat"}
    assert s["sonicpi_bd_tek"]["role"] == "kick"
    assert s["sonicpi_drum_tom_lo_hard"]["role"] == "tom"
    assert s["sonicpi_ride_tri"]["role"] == "ride"
    assert s["sonicpi_hat_zild"]["role"] == "hat"
    assert s["sonicpi_drum_splash_hard"]["role"] == "crash"
    assert s["sonicpi_sn_dub"]["role"] == "snare"


def test_muldjord_covers_a_rock_kit():
    _, catalog = _load("muldjord")
    by_role: dict = {}
    for info in catalog["samples"].values():
        by_role.setdefault(info["role"], []).append(info)
    assert set(by_role) == {"kick", "snare", "tom", "hat", "ride", "crash"}
    assert len(by_role["tom"]) == 8 and len(by_role["kick"]) == 4
    assert all(i["channels"] == 1 for i in catalog["samples"].values()), "один микрофон на инструмент"
