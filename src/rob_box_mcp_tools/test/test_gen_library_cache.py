"""Кэш генераций (ADR-0158 §2.5, §2.6): схема ADR, идемпотентная миграция
прежнего ``index.db`` (MiniMax, 3 трека на роботе), ключи, ledger, лимиты,
конвейер «заказ → кэш» на фейковом провайдере.
"""

from __future__ import annotations

import json
import sqlite3
from datetime import datetime, timezone

import pytest

from rob_box_mcp_tools.engine.gen import keys
from rob_box_mcp_tools.engine.gen.budget import Budget, BudgetPolicy
from rob_box_mcp_tools.engine.gen.library import SCHEMA_VERSION, GeneratedMusicLibrary
from rob_box_mcp_tools.engine.gen.pipeline import generate_into_cache
from rob_box_music.knowledge import GEN_MODELS
from rob_box_mcp_tools.engine.gen.provider import (
    GenJob,
    GenStatus,
    GenTicket,
    GenTrack,
    GenUnavailable,
)

#: Схема ДО ADR-0158 — как её создавал прежний ``generated_music_library.py``.
OLD_SCHEMA = """
CREATE TABLE generated_tracks (
    id TEXT PRIMARY KEY, title TEXT NOT NULL DEFAULT '', prompt TEXT NOT NULL,
    lyrics TEXT NOT NULL DEFAULT '', model TEXT NOT NULL DEFAULT '',
    duration_ms INTEGER NOT NULL DEFAULT 0, sample_rate INTEGER NOT NULL DEFAULT 44100,
    bitrate INTEGER NOT NULL DEFAULT 256000, file_size INTEGER NOT NULL DEFAULT 0,
    tags TEXT NOT NULL DEFAULT '[]', mood TEXT NOT NULL DEFAULT '', genre TEXT NOT NULL DEFAULT '',
    lang TEXT NOT NULL DEFAULT '', is_instrumental INTEGER NOT NULL DEFAULT 0,
    play_count INTEGER NOT NULL DEFAULT 0, rating INTEGER NOT NULL DEFAULT 0,
    notes TEXT NOT NULL DEFAULT '', created_at TEXT NOT NULL, updated_at TEXT NOT NULL
);
CREATE INDEX idx_generated_tracks_created_at ON generated_tracks (created_at DESC);
CREATE INDEX idx_generated_tracks_rating ON generated_tracks (rating DESC);
"""


def _make_old_library(root, n=3):
    root.mkdir(parents=True)
    conn = sqlite3.connect(str(root / "index.db"))
    conn.executescript(OLD_SCHEMA)
    for i in range(n):
        tid = f"{i:032x}"
        (root / tid).mkdir()
        (root / tid / "track.mp3").write_bytes(b"ID3old%d" % i)
        conn.execute(
            "INSERT INTO generated_tracks (id, title, prompt, model, play_count, rating, tags, created_at, updated_at) "
            "VALUES (?, ?, ?, 'music-2.0', ?, ?, '[\"x\"]', '2026-08-01T00:00:00+00:00', '2026-08-01T00:00:00+00:00')",
            (tid, f"old {i}", f"prompt {i}", i, i),
        )
    conn.commit()
    conn.close()


class TestMigration:
    def test_old_index_db_keeps_rows_and_gains_adr_columns(self, tmp_path):
        root = tmp_path / "music_library"
        _make_old_library(root)
        lib = GeneratedMusicLibrary(str(root))
        assert lib.count == 3
        cols = {r[1] for r in sqlite3.connect(str(root / "index.db")).execute("PRAGMA table_info(generated_tracks)")}
        for c in ("request_key", "pool_key", "gen_provider", "task_id", "variant", "credits", "tts_provider",
                  "voice", "work_id", "instrument", "source_audio_sha", "bpm_nominal", "loudness_lufs",
                  "last_played_at", "play_count", "rating", "meta_source", "kind", "style_key", "file_sha256"):
            assert c in cols, c
        info = lib.get_track_info(f"{1:032x}")
        assert info["title"] == "old 1" and info["play_count"] == 1 and info["rating"] == 1
        assert info["tags"] == ["x"] and info["model"] == "music-2.0"
        assert info["gen_provider"] == "minimax" and info["meta_source"] == "legacy"
        assert info["file_sha256"] == keys.sha256_bytes(b"ID3old1")  # дозаполнено для дедупликации
        assert info["exists_on_disk"] is True

    def test_migration_is_idempotent(self, tmp_path):
        root = tmp_path / "music_library"
        _make_old_library(root)
        GeneratedMusicLibrary(str(root))
        before = sqlite3.connect(str(root / "index.db")).execute("SELECT * FROM generated_tracks ORDER BY id").fetchall()
        lib = GeneratedMusicLibrary(str(root))  # второй и третий запуск
        lib = GeneratedMusicLibrary(str(root))
        after = sqlite3.connect(str(root / "index.db")).execute("SELECT * FROM generated_tracks ORDER BY id").fetchall()
        assert before == after and lib.count == 3
        conn = sqlite3.connect(str(root / "index.db"))
        assert conn.execute("PRAGMA user_version").fetchone()[0] == SCHEMA_VERSION

    def test_new_row_after_migration_is_not_relabelled_legacy(self, tmp_path):
        root = tmp_path / "music_library"
        _make_old_library(root, n=1)
        lib = GeneratedMusicLibrary(str(root))
        saved = lib.save_track(b"new", prompt="p", gen_provider="kie_suno", task_id="t", variant=0)
        lib2 = GeneratedMusicLibrary(str(root))
        info = lib2.get_track_info(saved["id"])
        assert info["gen_provider"] == "kie_suno" and info["meta_source"] == "live"

    def test_old_library_tools_contract_still_works(self, tmp_path):
        """list/search/play_count/delete — прежний контракт gen_* тулов."""
        root = tmp_path / "music_library"
        _make_old_library(root)
        lib = GeneratedMusicLibrary(str(root))
        assert lib.list_tracks(sort_by="rating")["tracks"][0]["title"] == "old 2"
        assert lib.search_tracks("prompt 1")["total"] == 1
        r = lib.increment_play_count(f"{0:032x}")
        assert r["play_count"] == 1 and r["last_played_at"]
        assert lib.delete_track(f"{0:032x}")["success"] and lib.count == 2


class TestSaveAndDedupe:
    def test_task_variant_is_idempotent(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "l"))
        a = lib.save_track(b"aaa", prompt="p", task_id="T1", variant=0)
        b = lib.save_track(b"aaa", prompt="p", task_id="T1", variant=0)
        c = lib.save_track(b"bbb", prompt="p", task_id="T1", variant=1)
        assert a["success"] and not a["duplicate"]
        assert b["duplicate"] and b["id"] == a["id"]
        assert c["id"] != a["id"] and lib.count == 2

    def test_null_task_id_dedupes_by_file_sha_only_when_asked(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "l"))
        a = lib.save_track(b"same", prompt="p", dedupe_sha=True)
        b = lib.save_track(b"same", prompt="p", dedupe_sha=True)
        c = lib.save_track(b"same", prompt="p")  # без dedupe_sha — отдельная запись (как раньше)
        assert b["duplicate"] and b["id"] == a["id"] and c["id"] != a["id"]
        assert lib.count == 2

    def test_find_by_request_key_least_recently_played_first(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "l"))
        a = lib.save_track(b"a", prompt="p", request_key="K", task_id="T", variant=0)
        b = lib.save_track(b"b", prompt="p", request_key="K", task_id="T", variant=1)
        lib.save_track(b"c", prompt="p", request_key="OTHER")
        lib.increment_play_count(a["id"])
        got = lib.find_by_request_key("K")
        assert [t["id"] for t in got["tracks"]] == [b["id"], a["id"]]
        assert lib.find_by_request_key("")["total"] == 0

    def test_meta_json_carries_cache_fields(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "l"))
        r = lib.save_track(b"x", prompt="p", kind="cover", work_id="W", instrument="sitar",
                           source_audio_sha="abc", task_id="T", variant=1, credits=12.0)
        meta = json.loads(open(r["meta_json"], encoding="utf-8").read())
        assert (meta["kind"], meta["work_id"], meta["instrument"], meta["variant"]) == ("cover", "W", "sitar", 1)


class TestKeys:
    BASE = dict(kind="cover", style_key="orchestral", work_id="Ride Of The Valkyries", gen_provider="kie_suno",
                model="V6", duration_s=90)

    def test_same_request_same_key_and_normalised(self):
        a = keys.request_key(**self.BASE)
        b = keys.request_key(**{**self.BASE, "work_id": "  ride of the  VALKYRIES "})
        assert a == b and len(a) == 64

    @pytest.mark.parametrize("change", [
        {"instrument": "sitar"}, {"style_key": "rock"}, {"work_id": "Toccata"}, {"voice": "v2"},
        {"model": "V6_MINI"}, {"source_audio_sha": "deadbeef"}, {"theme": "космос"},
        {"lyrics": "текст"}, {"tts_provider": "yandex"}, {"kind": "instrumental"}, {"duration_s": 180},
    ])
    def test_any_request_difference_changes_key(self, change):
        assert keys.request_key(**{**self.BASE, **change}) != keys.request_key(**self.BASE)

    def test_duration_bucket_groups_nearby_lengths(self):
        assert keys.request_key(**{**self.BASE, "duration_s": 150}) == keys.request_key(**{**self.BASE, "duration_s": 160})

    def test_pool_key(self):
        assert keys.pool_key("instrumental", "Grunge", True) == "instrumental|grunge|inst"
        assert keys.pool_key("robot_song", "grunge", False) == "robot_song|grunge|vocal"


class TestLedgerAndBudget:
    NOW = datetime(2026, 10, 14, 12, 0, tzinfo=timezone.utc)

    def _budget(self, tmp_path, **policy):
        lib = GeneratedMusicLibrary(str(tmp_path / "l"))
        return lib, Budget(lib, BudgetPolicy(**policy) if policy else BudgetPolicy(), now=lambda: self.NOW)

    def test_ledger_is_idempotent_per_task(self, tmp_path):
        lib, _ = self._budget(tmp_path)
        assert lib.ledger.record_spend("T1", 12.0, ts="2026-10-14T01:00:00+00:00") is True
        assert lib.ledger.record_spend("T1", 12.0, ts="2026-10-14T01:00:00+00:00") is False
        assert lib.ledger.spent_since("2026-10-14T00:00:00+00:00") == 12.0

    def test_yesterdays_spend_does_not_count(self, tmp_path):
        lib, b = self._budget(tmp_path, daily_credits=24.0)
        lib.ledger.record_spend("old", 12.0, ts="2026-10-13T23:59:00+00:00")
        lib.ledger.record_spend("today", 12.0, ts="2026-10-14T00:01:00+00:00")
        assert b.spent_today() == 12.0 and b.remaining_today() == 12.0
        b.check(12.0)

    def test_daily_limit_blocks_with_typed_error(self, tmp_path):
        lib, b = self._budget(tmp_path, daily_credits=24.0)
        lib.ledger.record_spend("a", 12.0, ts="2026-10-14T01:00:00+00:00")
        lib.ledger.record_spend("b", 12.0, ts="2026-10-14T02:00:00+00:00")
        with pytest.raises(GenUnavailable) as e:
            b.check(12.0)
        assert e.value.reason == "budget_exhausted"

    def test_set_limit(self, tmp_path):
        _, b = self._budget(tmp_path, daily_credits=1000.0, set_credits=24.0)
        b.check(12.0, set_spent=12.0)
        with pytest.raises(GenUnavailable) as e:
            b.check(12.0, set_spent=24.0)
        assert e.value.reason == "budget_exhausted"

    def test_defaults_live_in_one_place_and_cover_a_set(self):
        from rob_box_mcp_tools.engine.gen import budget
        p = BudgetPolicy()
        assert p.daily_credits == budget.DAILY_CREDITS_DEFAULT and p.set_credits == budget.SET_CREDITS_DEFAULT
        assert p.set_credits >= 10 * GEN_MODELS["V6"].credits_per_task  # 10 задач сета (В5)


class FakeProvider:
    name = "kie_suno"

    def __init__(self, credits=12.0, fail=None, stages=("queued", "complete")):
        self.credits, self.fail, self.stages = credits, fail, list(stages)
        self.submitted, self.downloads = [], []

    def cost(self, job):
        return 12.0

    def balance(self):
        return 100.0

    def submit(self, job):
        if self.fail:
            raise self.fail
        self.submitted.append(job)
        return GenTicket("TASK9", self.name, job, 0.0, 12.0)

    def poll(self, ticket):
        stage = self.stages.pop(0) if len(self.stages) > 1 else self.stages[0]
        if stage != "complete":
            return GenStatus(stage)
        tracks = tuple(GenTrack(i, f"id{i}", f"https://cdn/{i}.mp3", "", 60.0 + i, "Title") for i in range(2))
        return GenStatus("complete", tracks, credits=self.credits)

    def download(self, url):
        self.downloads.append(url)
        return b"MP3:" + url.encode()


class TestPipeline:
    JOB = GenJob(kind="instrumental", style_key="grunge", title="Robbox", style_tags="grunge", duration_s=60,
                 model="V6_MINI")

    def _env(self, tmp_path, **policy):
        lib = GeneratedMusicLibrary(str(tmp_path / "l"))
        return lib, Budget(lib, BudgetPolicy(**policy) if policy else BudgetPolicy())

    def test_both_variants_cached_with_ledger_and_keys(self, tmp_path):
        lib, budget = self._env(tmp_path)
        prov = FakeProvider()
        res = generate_into_cache(prov, lib, budget, self.JOB, theme="Батарейка", sleep=lambda s: None)
        assert res.task_id == "TASK9" and res.credits == 12.0 and not res.credits_estimated
        assert lib.count == 2 and prov.downloads == ["https://cdn/0.mp3", "https://cdn/1.mp3"]
        tracks = lib.find_by_request_key(res.request_key)["tracks"]
        assert sorted(t["variant"] for t in tracks) == [0, 1]
        t0 = next(t for t in tracks if t["variant"] == 0)
        assert (t0["task_id"], t0["gen_provider"], t0["kind"], t0["pool_key"], t0["meta_source"]) == (
            "TASK9", "kie_suno", "instrumental", "instrumental|grunge|inst", "live")
        assert lib.ledger.spent_since("2000-01-01T00:00:00+00:00") == 12.0

    def test_repeat_job_does_not_duplicate_tracks(self, tmp_path):
        lib, budget = self._env(tmp_path)
        generate_into_cache(FakeProvider(), lib, budget, self.JOB, sleep=lambda s: None)
        generate_into_cache(FakeProvider(), lib, budget, self.JOB, sleep=lambda s: None)
        assert lib.count == 2  # тот же task_id+variant (фейк отдаёт TASK9) — идемпотентно

    def test_budget_checked_before_any_provider_call(self, tmp_path):
        lib, budget = self._env(tmp_path, daily_credits=5.0)
        prov = FakeProvider()
        with pytest.raises(GenUnavailable) as e:
            generate_into_cache(prov, lib, budget, self.JOB, sleep=lambda s: None)
        assert e.value.reason == "budget_exhausted" and prov.submitted == [] and lib.count == 0

    @pytest.mark.parametrize("reason", ["no_key", "no_credits", "rate_limited", "provider_down", "rejected"])
    def test_provider_errors_propagate_without_cache_or_spend(self, tmp_path, reason):
        lib, budget = self._env(tmp_path)
        with pytest.raises(GenUnavailable) as e:
            generate_into_cache(FakeProvider(fail=GenUnavailable(reason)), lib, budget, self.JOB,
                                sleep=lambda s: None)
        assert e.value.reason == reason and lib.count == 0
        assert lib.ledger.spent_since("2000-01-01T00:00:00+00:00") == 0.0

    def test_missing_credits_in_response_is_marked_estimated(self, tmp_path):
        lib, budget = self._env(tmp_path)
        prov = FakeProvider(credits=None)
        res = generate_into_cache(prov, lib, budget, self.JOB, sleep=lambda s: None)
        assert res.credits_estimated and res.credits == 12.0

    def test_download_failure_keeps_the_spend(self, tmp_path):
        lib, budget = self._env(tmp_path)
        prov = FakeProvider()

        def boom(url):
            raise GenUnavailable("provider_down", "403")
        prov.download = boom
        with pytest.raises(GenUnavailable):
            generate_into_cache(prov, lib, budget, self.JOB, sleep=lambda s: None)
        assert lib.ledger.spent_since("2000-01-01T00:00:00+00:00") == 12.0  # кредиты потрачены — учтены
