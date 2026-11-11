"""«Найди похожих» и склейка дублей лиц (issue #4469): сходство, коллаж, слияние с бэкапом на настоящем ``FaceStore``.

Без ROS, камеры и сети: ``tmp_path`` вместо ``/data/faces``. Эмбеддинги - смеси базисных векторов с известным
косинусом: ``b0 + w*bi`` даёт косинус с ``b0``, равный ``1/sqrt(1+w^2)``.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

_PKG_ROOT = Path(__file__).resolve().parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

cv2 = pytest.importorskip("cv2")

from rob_box_perception import face_collage as fc  # noqa: E402
from rob_box_perception import face_merge as fm  # noqa: E402
from rob_box_perception import people_service as ps  # noqa: E402
from rob_box_perception.face_store import MODE_EXHIBITION, FaceStore  # noqa: E402

DIM = 512
NOON = 1_762_000_000.0


def _vec(*parts: tuple) -> np.ndarray:
    v = np.zeros(DIM, dtype=np.float32)
    for index, weight in parts:
        v[index] = weight
    return v / np.linalg.norm(v)


def _near(index: int, weight: float) -> np.ndarray:
    """Косинус с базисным ``b0`` равен ``1/sqrt(1+weight^2)``; друг с другом такие векторы далеки (<0.6)."""
    return _vec((0, 1.0), (index, weight))


def _jpeg(seed: int, size=(640, 480)) -> bytes:
    """Зашумлённый снимок: большой JPEG (сотни КБ), чтобы коллаж упирался в потолок размера."""
    noise = np.random.default_rng(seed).integers(0, 255, (size[1], size[0], 3), dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", noise, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return bytes(buf)


def _store(tmp_path, mode="workshop") -> FaceStore:
    return FaceStore(root=str(tmp_path / "faces"), mode=mode, clock=lambda: NOON)


def _met(store: FaceStore, emb: np.ndarray, snapshot=None, times: int = 1) -> str:
    match = None
    for _ in range(times):
        match = store.record_encounter(emb, snapshot=snapshot)
    return match.person_id


def _service(store: FaceStore, tmp_path) -> ps.PeopleService:
    return ps.PeopleService(store, lambda: NOON, backup_root=tmp_path / "backup")


def _row(store: FaceStore, pid: str) -> dict:
    return next(p for p in store.people() if p["person_id"] == pid)


# --- сходство: порог, порядок, потолок k -----------------------------------------------------------------------


def test_similarity_filters_by_threshold_and_sorts_by_similarity(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)))
    closer = _met(store, _near(5, 1.8))  # косинус 0.486
    farther = _met(store, _near(1, 2.0))  # косинус 0.447
    _met(store, _vec((9, 1.0)))  # косинус 0: чужой, под порог не проходит

    found = fm.similar_people(store, target)

    assert [c.person_id for c in found] == [closer, farther]
    assert found[0].similarity == pytest.approx(0.486, abs=0.01) and found[1].similarity == pytest.approx(
        0.447, abs=0.01
    )
    assert not any(c.same_name for c in found)


def test_namesakes_come_first_and_ignore_the_threshold(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)))
    store.attach_name(target, "Анна")
    similar = _met(store, _near(1, 2.0))
    namesake = _met(store, _vec((9, 1.0)))  # косинус 0, но то же имя (регистр не важен)
    store.attach_name(namesake, "анна")

    found = fm.similar_people(store, target)

    assert [c.person_id for c in found] == [namesake, similar]
    assert [c.same_name for c in found] == [True, False]


def test_k_is_capped_at_twelve_and_unknown_person_gives_nothing(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)))
    for i in range(1, 16):
        _met(store, _near(i, 2.0))

    assert len(fm.similar_people(store, target, k=50)) == fm.SIMILAR_K_MAX == 12
    assert len(fm.similar_people(store, target, k=3)) == 3
    assert fm.similar_people(store, "no-such-id") == []


# --- коллаж: размер, номера, плитки ----------------------------------------------------------------------------


def test_service_collage_is_a_jpeg_under_200kb_with_numbered_candidates(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)), snapshot=_jpeg(0))
    for i in range(1, 13):
        _met(store, _near(i, 2.0), snapshot=_jpeg(i))
    service = _service(store, tmp_path)

    reply = service.handle(ps.OP_SIMILAR, person_id=target[:8])

    assert reply.ok and reply.jpeg == b"" and reply.data["id"] == target[:8]
    assert [c["n"] for c in reply.data["candidates"]] == list(range(1, 13))
    assert all(len(c["id"]) == 8 and 0 < c["similarity"] <= 1 for c in reply.data["candidates"])
    collage = service.handle(ps.OP_COLLAGE, person_id=target[:8])
    assert collage.ok and collage.jpeg[:2] == b"\xff\xd8"
    assert 0 < len(collage.jpeg) <= ps.PHOTO_MAX_BYTES == 200 * 1024
    assert collage.data["size"] == len(collage.jpeg) == reply.data["size"]
    image = cv2.imdecode(np.frombuffer(collage.jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert image is not None and image.shape[1] > 0 and image.shape[0] > 0


def test_collage_grid_holds_target_plus_candidates_with_a_placeholder_for_a_missing_snapshot():
    tiles = [
        fc.Tile(fc.TARGET_MARK, "aaaaaaaa", None, _jpeg(1)),
        fc.Tile("1", "bbbbbbbb", 0.87, None),  # снимка нет: серая плитка, номер всё равно виден
        fc.Tile("2", "cccccccc", 0.5, b"not a jpeg"),
    ]  # битый снимок - то же

    raw = fc.build_collage(tiles)

    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    assert image.shape[1] == fc.COLUMNS * fc.CELL and image.shape[0] == fc.CELL + fc.CAPTION_H
    second = image[: fc.CELL, fc.CELL : 2 * fc.CELL]  # плитка 1: тёмно-серая подложка и белая цифра в чёрной рамке
    assert second[100:150, 100:150].mean() < 60 and second[:55, :55].max() > 200
    assert fc.build_collage([]) is None


def test_collage_without_candidates_is_an_empty_answer_not_an_error(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)), snapshot=_jpeg(0))
    _met(store, _vec((9, 1.0)), snapshot=_jpeg(1))

    reply = _service(store, tmp_path).handle(ps.OP_SIMILAR, person_id=target)

    assert reply.ok and reply.data["candidates"] == [] and reply.jpeg == b""


def test_collage_op_without_a_prior_search_refuses_and_exhibition_never_leaks_faces(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)), snapshot=_jpeg(0))
    _met(store, _near(1, 2.0), snapshot=_jpeg(1))
    service = _service(store, tmp_path)

    assert service.handle(ps.OP_COLLAGE, person_id=target).reason == "no_collage"
    store.set_mode(MODE_EXHIBITION)
    assert service.handle(ps.OP_SIMILAR, person_id=target).reason == "exhibition_mode"


# --- слияние: бэкап до, ракурсы раздельно, имя и голос, фраза-факт ---------------------------------------------


def test_merge_keeps_every_angle_backs_up_first_and_reports_the_stored_fact(tmp_path):
    store = _store(tmp_path)
    target_vec, a_vec, b_vec = _vec((0, 1.0)), _near(1, 2.0), _near(2, 2.0)
    target = _met(store, target_vec, snapshot=_jpeg(0), times=2)
    a = _met(store, a_vec, snapshot=_jpeg(1), times=3)
    b = _met(store, b_vec, snapshot=_jpeg(2))
    store.attach_name(a, "Тест-Имя", speaker_id="spk-9")
    root = tmp_path / "faces"
    before = {pid: store.gallery(pid) for pid in (target, a, b)}

    outcome = fm.merge_people(store, target, [a, b], tmp_path / "backup", lambda: NOON)

    assert outcome.merged == 2 and set(outcome.merged_ids) == {a, b}
    assert {p["person_id"] for p in store.people()} == {target}
    after = store.gallery(target)
    assert len(after) == sum(len(g) for g in before.values()), "ракурсы складываются, а не усредняются в один вектор"
    for original in (target_vec, a_vec, b_vec):
        assert any(np.allclose(original, vec, atol=1e-5) for vec in after)
    row = _row(store, target)
    assert row["name"] == "Тест-Имя" and row["speaker_id"] == "spk-9" and row["encounter_count"] == 6
    assert outcome.name == "Тест-Имя" and outcome.gallery == len(after)
    # бэкап: снаружи каталога лиц, с каталогами всех трёх записей и их эмбеддингами, снятыми ДО слияния
    backup = Path(outcome.backup_dir)
    assert backup.is_dir() and root not in backup.parents and backup.parent == tmp_path / "backup"
    for pid in (target, a, b):
        saved = np.load(backup / pid / "embeddings.npy")
        assert saved.shape[0] == len(before[pid]) and (backup / pid / "meta.json").is_file()
    involved = json.loads((backup / "involved.json").read_text(encoding="utf-8"))
    assert {r["person_id"] for r in involved} == {target, a, b}


def test_target_name_wins_over_the_merged_names(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)))
    other = _met(store, _near(1, 2.0))
    store.attach_name(target, "Первый")
    store.attach_name(other, "Второй")

    outcome = fm.merge_people(store, target, [other], tmp_path / "backup")

    assert outcome.name == "Первый"


def test_merge_refuses_bad_input_and_does_not_touch_the_store(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)))
    other = _met(store, _near(1, 2.0))

    for sources in ([], [target], [other, other], ["ghost"]):
        assert fm.merge_people(store, target, sources, tmp_path / "backup") is None
    assert fm.merge_people(store, "ghost", [other], tmp_path / "backup") is None
    assert len(store.people()) == 2 and not (tmp_path / "backup").exists(), "отказ - без бэкапа и без правок"


def test_service_merge_translates_prefixes_and_refuses_with_a_reason(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)))
    a = _met(store, _near(1, 2.0))
    b = _met(store, _near(2, 2.0))
    service = _service(store, tmp_path)

    assert service.handle(ps.OP_MERGE, filter="", person_id=target[:8]).reason == "bad_ids"
    assert service.handle(ps.OP_MERGE, filter=",".join([a[:8]] * 2), person_id=target[:8]).reason == "bad_ids"
    assert service.handle(ps.OP_MERGE, filter=target[:8], person_id=target[:8]).reason == "same_person"
    assert service.handle(ps.OP_MERGE, filter="deadbeef", person_id=target[:8]).reason == "person_not_found"
    assert service.handle(ps.OP_MERGE, filter="не-id", person_id=target[:8]).reason == "bad_id"
    assert len(store.people()) == 3

    reply = service.handle(ps.OP_MERGE, filter=f"{a[:8]}, {b[:8]}", person_id=target[:8])

    assert reply.ok and reply.data["merged"] == 2 and reply.data["id"] == target[:8]
    assert reply.data["gallery"] == len(store.gallery(target)) == 3
    assert set(reply.data["merged_ids"]) == {a[:8], b[:8]} and Path(reply.data["backup"]).is_dir()
    assert [p["person_id"] for p in store.people()] == [target]


def test_a_merge_invalidates_the_collage_of_the_merged_faces(tmp_path):
    store = _store(tmp_path)
    target = _met(store, _vec((0, 1.0)), snapshot=_jpeg(0))
    a = _met(store, _near(1, 2.0), snapshot=_jpeg(1))
    service = _service(store, tmp_path)
    assert service.handle(ps.OP_SIMILAR, person_id=target).ok and service.handle(ps.OP_COLLAGE, person_id=target).ok

    assert service.handle(ps.OP_MERGE, filter=a[:8], person_id=target).ok

    assert service.handle(ps.OP_COLLAGE, person_id=target).reason == "no_collage"
