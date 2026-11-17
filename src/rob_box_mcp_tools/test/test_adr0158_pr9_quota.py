"""ADR-0158 PR-9 — квота диска кэша генераций: худшее и давно не игранное уходит первым, заказ — целиком.

Чистая логика, ROS 2 не нужен: ``engine.gen.quota`` читает только ``GeneratedMusicLibrary`` и диск.
"""

from pathlib import Path

import pytest

from rob_box_mcp_tools.engine.gen import quota
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_music import knowledge as kn

KB = 1000


@pytest.fixture
def lib(tmp_path):
    return GeneratedMusicLibrary(str(tmp_path / "lib"))


def add(lib, title, *, task=None, variant=None, rating=0, played=None, size=KB):
    """Трек кэша: mp3 + wav сервера рядом (как после первого проигрывания), вес каталога ≈ size * 2."""
    saved = lib.save_track(b"m" * size, prompt=title, title=title, task_id=task, variant=variant, rating=rating)
    (lib.root_dir / saved["id"] / "track.16k.wav").write_bytes(b"w" * size)
    if played:
        lib._conn.execute("UPDATE generated_tracks SET last_played_at = ? WHERE id = ?", (played, saved["id"]))
        lib._conn.commit()
    return saved["id"]


def titles(lib):
    return {t["title"] for t in lib.list_tracks(limit=100)["tracks"]}


def test_default_quota_is_a_knowledge_constant():
    assert kn.GEN_LIBRARY_QUOTA_MB == 2048 and quota.MIB == 1024 * 1024


def test_measures_mp3_wav_and_meta(lib):
    track_id = add(lib, "a")
    dir_size = quota.dir_bytes(lib.root_dir / track_id)
    assert dir_size >= 2 * KB and quota.used_bytes(lib) == dir_size


def test_under_quota_removes_nothing(lib):
    add(lib, "a")
    result = quota.enforce_quota(lib, max_bytes=10 * KB * KB)
    assert result["removed"] == [] and titles(lib) == {"a"}


def test_dislike_goes_first_then_the_longest_unplayed(lib):
    add(lib, "old", played="2026-10-01T00:00:00+00:00")
    add(lib, "recent", played="2026-10-13T00:00:00+00:00")
    add(lib, "bad", rating=kn.RATING_DISLIKE, played="2026-10-14T00:00:00+00:00")
    one_track = quota.orders(lib)[0].bytes
    first = quota.enforce_quota(lib, max_bytes=2 * one_track + 500)
    assert len(first["removed"]) == 1 and titles(lib) == {"old", "recent"}
    quota.enforce_quota(lib, max_bytes=one_track + 500)
    assert titles(lib) == {"recent"}


def test_liked_track_is_evicted_last(lib):
    add(lib, "liked", rating=kn.RATING_LIKE, played="2026-09-01T00:00:00+00:00")
    add(lib, "plain", played="2026-10-13T00:00:00+00:00")
    one_track = quota.orders(lib)[0].bytes
    quota.enforce_quota(lib, max_bytes=one_track + 500)
    assert titles(lib) == {"liked"}


def test_both_variants_of_one_task_leave_together_with_both_files(lib):
    a = add(lib, "v0", task="T1", variant=0, rating=kn.RATING_DISLIKE)
    b = add(lib, "v1", task="T1", variant=1, rating=kn.RATING_DISLIKE)
    add(lib, "other", task="T2", variant=0, played="2026-10-13T00:00:00+00:00")
    one_task = max(o.bytes for o in quota.orders(lib))
    result = quota.enforce_quota(lib, max_bytes=one_task // 2 + 1500)
    assert set(result["removed"]) == {a, b} and titles(lib) == {"other"}
    assert not Path(lib.root_dir / a).exists() and not Path(lib.root_dir / b).exists()


def test_a_liked_variant_keeps_its_disliked_sibling_until_last(lib):
    add(lib, "v0", task="T1", variant=0, rating=kn.RATING_DISLIKE)
    add(lib, "v1", task="T1", variant=1, rating=kn.RATING_LIKE)
    add(lib, "other", task="T2", variant=0)
    pair = max(o.bytes for o in quota.orders(lib))
    quota.enforce_quota(lib, max_bytes=pair + 500)
    assert titles(lib) == {"v0", "v1"}


def test_fresh_save_is_protected_even_over_quota(lib):
    old = add(lib, "old", played="2026-10-01T00:00:00+00:00")
    fresh = add(lib, "fresh")
    result = quota.enforce_quota(lib, max_bytes=0, protect=[fresh])
    assert result["removed"] == [old] and titles(lib) == {"fresh"}
