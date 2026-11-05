#!/usr/bin/env python3
"""test_epithet_storage.py — хранение клички спикера в ``speakers.db`` (issue #1787).

Проверяет ``utils/speaker_embeddings.py``: миграция схемы и хранение эпитета. Чистая логика клички
(``rob_box_core.epithets``) — в ``rob_box_core/test/test_epithets.py`` (ADR-0164 этап 4).

``speaker_embeddings`` грузится по пути файла, минуя ``utils/__init__.py`` (он тянет
pyaudio через audio_utils) — тот же приём, что в ``test_speaker_embeddings``,
чтобы тест шёл в CI без тяжёлых зависимостей.
"""

from __future__ import annotations

import importlib.util
import sqlite3
import sys
from pathlib import Path

import numpy as np
import pytest

from rob_box_core import epithets as ep

_PKG_ROOT = Path(__file__).resolve().parents[1] / "rob_box_voice"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    # Регистрируем ДО exec_module: @dataclass читает sys.modules[__module__]
    # при разборе аннотаций и падает на AttributeError, если модуля там нет.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_se = _load(
    "rob_box_voice.utils.speaker_embeddings",
    _PKG_ROOT / "utils" / "speaker_embeddings.py",
)
SpeakerDatabase = _se.SpeakerDatabase


def _random_embedding(seed: int = 0, dim: int = 256) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(dim).astype(np.float32)
    return v / np.linalg.norm(v)


@pytest.fixture()
def db(tmp_path):
    database = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield database
    database.close()


# ── Хранение ─────────────────────────────────────────────────────────────────


def test_migration_adds_columns_to_legacy_db(tmp_path):
    """Старая БД (42 профиля на роботе) досоздаёт колонки без backfill."""
    path = tmp_path / "legacy.db"
    conn = sqlite3.connect(str(path))
    conn.executescript(
        "CREATE TABLE speakers (speaker_id TEXT PRIMARY KEY, name TEXT NOT NULL,"
        " created_at REAL NOT NULL);"
        "CREATE TABLE embeddings (id INTEGER PRIMARY KEY AUTOINCREMENT,"
        " speaker_id TEXT NOT NULL, embedding BLOB NOT NULL, created_at REAL"
        " NOT NULL);"
    )
    conn.execute(
        "INSERT INTO speakers VALUES ('old-1', 'Денчик', 1000.0)"
    )
    conn.commit()
    conn.close()

    database = SpeakerDatabase(str(path))
    try:
        columns = {
            row[1]
            for row in database._conn.execute("PRAGMA table_info(speakers)")
        }
        assert {"epithet", "epithet_history", "tags", "sentiment_score"} <= columns
        profile = database.get_speaker_profile("old-1")
        assert profile["name"] == "Денчик"
        assert profile["epithet"] is None  # backfill не нужен (research §5.5)
    finally:
        database.close()


def test_migration_is_idempotent(tmp_path):
    """Второй старт ноды на той же БД не должен падать на ADD COLUMN."""
    path = str(tmp_path / "twice.db")
    first = SpeakerDatabase(path)
    first.close()
    second = SpeakerDatabase(path)
    second.close()


def test_set_epithet_writes_history(db):
    sid = db.register("Денчик", _random_embedding(1))
    assert db.set_epithet(sid, "Гроссмейстер", ep.REASON_FIRST_SEEN) is True
    assert db.set_epithet(sid, "Дровосек", "new_topic:природа") is True

    profile = db.get_speaker_profile(sid)
    assert profile["epithet"] == "Дровосек"
    history = profile["epithet_history"]
    assert [h["new"] for h in history] == ["Гроссмейстер", "Дровосек"]
    assert history[0]["old"] is None
    assert history[1]["old"] == "Гроссмейстер"
    assert history[1]["reason"] == "new_topic:природа"


def test_set_epithet_is_idempotent_for_same_label(db):
    """Повтор той же клички не плодит записи, но двигает таймер пересмотра."""
    sid = db.register("Саша", _random_embedding(2))
    db.set_epithet(sid, "Конфуций", ep.REASON_FIRST_SEEN)
    before = db.get_speaker_profile(sid)["last_epithet_review"]
    assert db.set_epithet(sid, "Конфуций", ep.REASON_FIRST_SEEN) is True
    after = db.get_speaker_profile(sid)
    assert len(after["epithet_history"]) == 1
    assert after["last_epithet_review"] >= before


def test_set_epithet_rejects_unknown_speaker_and_empty_label(db):
    sid = db.register("Иван", _random_embedding(3))
    assert db.set_epithet("no-such-id", "Барон", "x") is False
    assert db.set_epithet(sid, "   ", "x") is False


def test_taken_epithets_excludes_self(db):
    a = db.register("Денис", _random_embedding(4))
    b = db.register("Денис", _random_embedding(5))
    db.set_epithet(a, "Гроссмейстер", ep.REASON_FIRST_SEEN)
    db.set_epithet(b, "Электрик", ep.REASON_FIRST_SEEN)

    assert sorted(db.taken_epithets()) == ["Гроссмейстер", "Электрик"]
    assert db.taken_epithets(exclude_speaker_id=a) == ["Электрик"]


def test_identify_returns_epithet(db):
    emb = _random_embedding(6)
    sid = db.register("Борис", emb)
    db.set_epithet(sid, "Дровосек", ep.REASON_FIRST_SEEN)

    match = db.identify(emb)
    assert match is not None
    assert match.epithet == "Дровосек"


def test_identify_epithet_is_none_before_assignment(db):
    emb = _random_embedding(7)
    db.register("Аноним", emb)
    match = db.identify(emb)
    assert match is not None
    assert match.epithet is None


def test_update_speaker_stats(db):
    sid = db.register("Женя", _random_embedding(8))
    assert db.update_speaker_stats(sid, tags=["техно", "код"], sentiment_score=0.4)
    profile = db.get_speaker_profile(sid)
    assert profile["tags"] == ["техно", "код"]
    assert profile["sentiment_score"] == pytest.approx(0.4)
    # Пустой вызов не ходит в БД.
    assert db.update_speaker_stats(sid) is False


def test_list_speakers_exposes_epithet(db):
    sid = db.register("Денчик", _random_embedding(9))
    db.set_epithet(sid, "Гроссмейстер", ep.REASON_FIRST_SEEN)
    db.update_speaker_stats(sid, tags=["шахматы"])
    row = next(r for r in db.list_speakers() if r["id"] == sid)
    assert row["epithet"] == "Гроссмейстер"
    assert row["tags"] == ["шахматы"]


def test_broken_history_json_does_not_crash(db):
    """История — диагностика; битый JSON не должен ронять профиль."""
    sid = db.register("Кто-то", _random_embedding(10))
    db._conn.execute(
        "UPDATE speakers SET epithet_history='{не json' WHERE speaker_id=?", (sid,)
    )
    db._conn.commit()
    assert db.get_speaker_profile(sid)["epithet_history"] == []
    # И следующая запись эпитета проходит нормально, начиная историю заново.
    assert db.set_epithet(sid, "Странник", ep.REASON_FIRST_SEEN) is True
    assert len(db.get_speaker_profile(sid)["epithet_history"]) == 1


def test_end_to_end_two_namesakes_stay_distinguishable(db):
    """Сквозной сценарий issue #1787 на уровне БД.

    Два «Дениса» с разными голосами и разными темами: имена в БД
    одинаковые, эпитеты — разные, и identify возвращает правильный.
    """
    emb_a, emb_b = _random_embedding(11), _random_embedding(12)
    a = db.register("Денис", emb_a)
    b = db.register("Денис", emb_b)

    chess = ep.extract_tags(["шахматы, дебют, эндшпиль, ферзь, ладья, гамбит"])
    tech = ep.extract_tags(["паяльник, микросхема, транзистор, прошивка, плата"])

    cand_a = ep.choose_epithet(chess, speaker_id=a, taken=db.taken_epithets(a))
    db.set_epithet(a, cand_a.label, ep.REASON_FIRST_SEEN)
    cand_b = ep.choose_epithet(tech, speaker_id=b, taken=db.taken_epithets(b))
    db.set_epithet(b, cand_b.label, ep.REASON_FIRST_SEEN)

    assert cand_a.label != cand_b.label
    assert db.identify(emb_a).epithet == cand_a.label
    assert db.identify(emb_b).epithet == cand_b.label
    # Имена при этом совпадают — то самое, из-за чего заводился эпитет.
    assert db.identify(emb_a).name == db.identify(emb_b).name == "Денис"
