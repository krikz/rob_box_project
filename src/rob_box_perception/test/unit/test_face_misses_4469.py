"""Журнал промахов узнавания (issue #4469, п. 2): склейка дописывает по строке на каждую слитую запись.

Без ROS и сети; только синтетические векторы (смеси базисных) в ``tmp_path``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_PKG_ROOT = Path(__file__).resolve().parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from rob_box_perception import face_merge as fm  # noqa: E402
from rob_box_perception import face_misses as fx  # noqa: E402
from rob_box_perception import people_service as ps  # noqa: E402
from rob_box_perception.face_store import FaceStore, _cosine  # noqa: E402

DIM = 512
NOON = 1_762_000_000.0


def _vec(*parts: tuple) -> np.ndarray:
    v = np.zeros(DIM, dtype=np.float32)
    for index, weight in parts:
        v[index] = weight
    return v / np.linalg.norm(v)


def _store(tmp_path) -> FaceStore:
    return FaceStore(root=str(tmp_path / "faces"), mode="workshop", clock=lambda: NOON)


def _met(store: FaceStore, emb: np.ndarray) -> str:
    return store.record_encounter(emb).person_id


def _world(tmp_path):
    """target + три дубля с известными сходствами + посторонний ``other``, к которому дубль ``c`` ближе всех."""
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)))
    a = _met(store, _vec((0, 1.0), (1, 2.0)))  # к target: 0.447
    b = _met(store, _vec((0, 1.0), (2, 2.0)))
    other = _met(store, _vec((3, 1.0)))
    c = _met(store, _vec((3, 1.0), (4, 1.5)))  # к other: 0.555 (ниже порога, потому и дубль)
    return store, target, [a, b, c], other


def test_merging_three_records_writes_three_rows_with_all_fields(tmp_path):
    store, target, ids, other = _world(tmp_path)
    journal = fx.MissJournal(tmp_path / "face_misses.jsonl")
    embeddings = {i: store.gallery(i)[0].copy() for i in ids}
    target_gallery = store.gallery(target)

    outcome = fm.merge_people(store, target, ids, tmp_path / "backup", lambda: NOON, journal)

    assert outcome.merged == 3
    rows = fx.read_rows(journal.path)
    assert [r["merged_id"] for r in rows] == ids
    for row in rows:
        assert row["target_id"] == target
        assert row["ts"].startswith("2025-") and row["ts"].endswith("+00:00")
        assert row["threshold"] == pytest.approx(0.6)
        assert np.allclose(row["embedding"], embeddings[row["merged_id"]], atol=1e-5)
        expected_target = max(_cosine(embeddings[row["merged_id"]], g) for g in target_gallery)
        assert row["target_similarity"] == pytest.approx(expected_target, abs=1e-3)
    by_id = {r["merged_id"]: r for r in rows}
    assert by_id[ids[2]]["best_id"] == other and by_id[ids[2]]["best_similarity"] == pytest.approx(0.555, abs=0.01)
    assert by_id[ids[0]]["best_id"] in (target, ids[1]) and by_id[ids[0]]["best_similarity"] < 0.6
    assert journal.written == 3 and journal.failures == 0


def test_best_candidate_comes_from_the_recognition_code_not_a_copy(tmp_path, monkeypatch):
    store, target, ids, other = _world(tmp_path)
    calls = {"score": 0, "disambiguate": 0}
    score_all, disambiguate = store._score_all, store._disambiguate

    def spy_score(emb):
        calls["score"] += 1
        return score_all(emb)

    def spy_dis(scored):
        calls["disambiguate"] += 1
        return disambiguate(scored)

    monkeypatch.setattr(store, "_score_all", spy_score)
    monkeypatch.setattr(store, "_disambiguate", spy_dis)

    rows = fx.snapshot_misses(store, target, ids, NOON)

    assert calls == {"score": 3, "disambiguate": 3}
    # слитая запись сама себя кандидатом не выбирает: промах - это ближайший ДРУГОЙ
    assert all(r["best_id"] != r["merged_id"] for r in rows)


def test_row_is_computed_before_the_merge_changes_the_target(tmp_path):
    store, target, ids, _ = _world(tmp_path)
    journal = fx.MissJournal(tmp_path / "face_misses.jsonl")
    emb_a = store.gallery(ids[0])[0].copy()
    only_target = max(_cosine(emb_a, g) for g in store.gallery(target))

    fm.merge_people(store, target, ids, tmp_path / "backup", lambda: NOON, journal)

    first = fx.read_rows(journal.path)[0]
    # после слияния галерея target включает сам вектор a (сходство 1.0): в журнале - значение ДО
    assert first["target_similarity"] == pytest.approx(only_target, abs=1e-3)
    assert first["target_similarity"] < 0.99


def test_journal_write_failure_does_not_break_the_merge(tmp_path, caplog):
    store, target, ids, _ = _world(tmp_path)
    blocker = tmp_path / "not_a_dir"
    blocker.write_text("x")
    journal = fx.MissJournal(blocker / "face_misses.jsonl")  # родитель - файл: запись невозможна

    with caplog.at_level("WARNING"):
        outcome = fm.merge_people(store, target, ids, tmp_path / "backup", lambda: NOON, journal)

    assert outcome.merged == 3 and {p["person_id"] for p in store.people()} >= {target}
    assert journal.failures == 1 and journal.written == 0
    assert any("face_misses" in r.message for r in caplog.records)


def test_journal_is_append_only_across_merges(tmp_path):
    store, target, ids, _ = _world(tmp_path)
    journal = fx.MissJournal(tmp_path / "face_misses.jsonl")
    fm.merge_people(store, target, ids[:1], tmp_path / "backup", lambda: NOON, journal)
    first = journal.path.read_bytes()
    fm.merge_people(store, target, ids[1:], tmp_path / "backup", lambda: NOON + 1, journal)

    data = journal.path.read_bytes()
    assert data.startswith(first) and len(fx.read_rows(journal.path)) == 3


def test_refused_merge_writes_nothing(tmp_path):
    store, target, ids, _ = _world(tmp_path)
    journal = fx.MissJournal(tmp_path / "face_misses.jsonl")

    assert fm.merge_people(store, target, [ids[0], ids[0]], tmp_path / "backup", lambda: NOON, journal) is None
    assert not journal.path.exists()


def test_service_journal_defaults_next_to_faces_dir_and_takes_config_path(tmp_path):
    store, target, ids, _ = _world(tmp_path)
    service = ps.PeopleService(store, lambda: NOON, backup_root=tmp_path / "backup")
    assert service._misses.path == tmp_path / fx.JOURNAL_NAME  # рядом с каталогом лиц, не внутри него
    custom = tmp_path / "elsewhere" / "m.jsonl"
    service = ps.PeopleService(store, lambda: NOON, backup_root=tmp_path / "backup", misses_path=custom)

    reply = service.handle(ps.OP_MERGE, ",".join(i[:8] for i in ids[:2]), target[:8])

    assert reply.ok and len(fx.read_rows(custom)) == 2
