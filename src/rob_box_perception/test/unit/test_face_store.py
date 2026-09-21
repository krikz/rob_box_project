"""Unit-тесты для ``FaceStore`` (ADR-0123 «Режимы приватности лицевого
канала», issue #2599).

Чисто pytest + numpy, ``tmp_path`` вместо ``/data/faces`` — ADR-0123 §5:
«Тесты режимов — тесты FaceStore, без ROS и без камеры». Никакого ROS,
cv2 или Hailo здесь нет и быть не должно.

Покрываем:
  1. Узнавание: новый эмбеддинг создаёт запись, похожий — матчится
     без дубля (§3/§6).
  2. Рост галереи: вытеснение самого похожего на остальные при
     переполнении ``max_embeddings`` (§4.1).
  3. Ротация снимков встреч, эталон не ротируется (§4.1).
  4. Гейтинг по режимам: workshop/exhibition/strict (§2).
  5. Переходы режимов: workshop→exhibition, *→strict, «мягче ничего
     не восстанавливает» (§5).
  6. attach_name переживает рестарт (persist на диск).
  7. merge переносит эмбеддинги и снимки, удаляет drop.
  8. forget стирает всё в любом режиме (§5/§11).
  9. max_strangers вытесняет самого давнего незнакомца, не трогая
     знакомых (§10).
  10. Битые meta.json не роняют конструктор (§5, требование карточки).

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_face_store.py -q
"""

from __future__ import annotations

import importlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

# ---------- import target under test (см. test_gaze_seam.py — тот же приём) --

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]


def _import_module():
    if str(_PKG_ROOT) not in sys.path:
        sys.path.insert(0, str(_PKG_ROOT))
    return importlib.import_module('rob_box_perception.face_store')


fs = _import_module()

DIM = 512  # реальный ArcFace HEF — см. докстринг face_store.py


# ============================================================================
# Хелперы построения детерминированных эмбеддингов
# ============================================================================

def _basis(index: int, dim: int = DIM) -> np.ndarray:
    """i-й вектор стандартного базиса — единичный, ортогональный
    остальным (косинус = 0), удобен как «явно другой человек»."""
    v = np.zeros(dim, dtype=np.float32)
    v[index] = 1.0
    return v


def _combo(theta: float, dim: int = DIM) -> np.ndarray:
    """Точка на единичной окружности в плоскости span(e0, e1).

    Косинус между ``_combo(a)`` и ``_combo(b)`` в точности ``cos(a - b)``
    — используем это для детерминированных, заранее просчитанных тестов
    близости (вместо случайных векторов, где пришлось бы гадать пороги).
    """
    e0 = _basis(0, dim)
    e1 = _basis(1, dim)
    return (np.cos(theta) * e0 + np.sin(theta) * e1).astype(np.float32)


def _noisy_copy(vec: np.ndarray, scale: float = 1e-4, seed: int = 0) -> np.ndarray:
    """«Почти тот же человек» — тот же вектор с исчезающе малым шумом,
    после нормировки косинус остаётся практически 1.0."""
    rng = np.random.RandomState(seed)
    noisy = vec + rng.normal(scale=scale, size=vec.shape).astype(np.float32)
    norm = np.linalg.norm(noisy)
    return (noisy / norm).astype(np.float32)


def _jpeg(tag: str) -> bytes:
    """Заглушка «уже закодированного JPEG» — модуль не смотрит внутрь."""
    return f'fake-jpeg:{tag}'.encode('utf-8')


# ============================================================================
# 1. Узнавание: новый vs. похожий
# ============================================================================

class TestIdentifyAndDedup:

    def test_unseen_embedding_creates_new_record(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        match = store.record_encounter(_basis(0))
        assert match.is_new is True
        assert match.similarity == 1.0
        assert match.encounter_count == 1
        assert match.name is None

    def test_near_identical_embedding_matches_no_duplicate(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        first = store.record_encounter(_basis(0))
        second = store.record_encounter(_noisy_copy(_basis(0)))

        assert second.is_new is False
        assert second.person_id == first.person_id
        assert second.encounter_count == 2
        assert len(store.people()) == 1

    def test_identify_is_read_only(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        created = store.record_encounter(_basis(0))

        match = store.identify(_noisy_copy(_basis(0)))
        assert match is not None
        assert match.person_id == created.person_id
        assert match.is_new is False
        # identify() не должен ничего дописывать
        assert store.identify(_noisy_copy(_basis(0))).encounter_count == 1
        assert len(store.gallery(created.person_id)) == 1

    def test_identify_unrelated_embedding_returns_none(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        store.record_encounter(_basis(0))
        assert store.identify(_basis(2)) is None


# ============================================================================
# 2. Рост галереи и вытеснение самого избыточного (ADR-0123 §4.1)
# ============================================================================

class TestGalleryEviction:

    def test_evicts_most_similar_to_rest_not_oldest(self, tmp_path):
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP, max_embeddings=3,
        )
        # Углы подобраны так, что при переполнении до 4 элементов у
        # theta=0.05 (index 2) строго наибольшая средняя близость к
        # остальным трём — см. расчёт в docstring карточки/PR.
        thetas = [0.0, 0.03, 0.05, 0.20]
        embeddings = [_combo(t) for t in thetas]

        match0 = store.record_encounter(embeddings[0])
        assert match0.is_new is True
        for emb in embeddings[1:]:
            m = store.record_encounter(emb)
            assert m.is_new is False, 'все точки лежат близко друг к другу'

        gallery = store.gallery(match0.person_id)
        assert len(gallery) == 3

        def _contains(vec):
            return any(np.allclose(g, vec, atol=1e-6) for g in gallery)

        assert _contains(embeddings[0])
        assert _contains(embeddings[1])
        assert _contains(embeddings[3])
        assert not _contains(embeddings[2]), (
            'theta=0.05 несёт меньше всего новой информации и должен '
            'быть вытеснен первым (ADR-0123 §4.1)'
        )


# ============================================================================
# 3. Ротация снимков встреч (ADR-0123 §4.1)
# ============================================================================

class TestSnapshotRotation:

    def test_reference_never_rotates_encounters_do(self, tmp_path):
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP, keep_encounters=3,
        )
        base = _combo(0.0)
        ref_bytes = _jpeg('reference')
        match = store.record_encounter(base, snapshot=ref_bytes)
        person_dir = tmp_path / match.person_id

        assert (person_dir / 'reference.jpg').read_bytes() == ref_bytes

        # 5 дополнительных встреч, каждая со своим снимком — эталон не
        # должен ротироваться, а в encounters/ должно остаться не больше
        # keep_encounters файлов.
        for i in range(5):
            theta = 0.001 * (i + 1)  # исчезающе малое смещение — тот же человек
            store.record_encounter(_combo(theta), snapshot=_jpeg(f'enc-{i}'))

        assert (person_dir / 'reference.jpg').read_bytes() == ref_bytes, (
            'эталонный снимок не ротируется никогда'
        )
        encounter_files = list((person_dir / 'encounters').glob('*_face.jpg'))
        assert len(encounter_files) == 3

        meta = json.loads((person_dir / 'meta.json').read_text(encoding='utf-8'))
        assert len(meta['encounters']) == 3
        # Последние по времени снимки должны остаться (enc-2, enc-3, enc-4)
        kept_names = {e['face_snapshot'] for e in meta['encounters']}
        assert kept_names == {
            p.name for p in encounter_files
        }


# ============================================================================
# 4. Гейтинг по режимам (ADR-0123 §2)
# ============================================================================

class TestModeGating:

    def test_workshop_persists_strangers_across_restart(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        match = store.record_encounter(_basis(0), snapshot=_jpeg('a'))
        assert (tmp_path / match.person_id / 'meta.json').exists()
        assert (tmp_path / match.person_id / 'embeddings.npy').exists()

        restarted = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        found = restarted.identify(_basis(0))
        assert found is not None
        assert found.person_id == match.person_id

    def test_exhibition_strangers_are_memory_only(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_EXHIBITION)
        match = store.record_encounter(_basis(0), snapshot=_jpeg('a'))

        # ничего не должно было попасть на диск для незнакомца
        assert not (tmp_path / match.person_id).exists()
        assert store.people()[0]['has_snapshot'] is False

        restarted = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_EXHIBITION)
        assert restarted.identify(_basis(0)) is None
        assert restarted.people() == []

    def test_exhibition_named_person_persists_with_snapshot(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_EXHIBITION)
        match = store.record_encounter(_basis(0))
        store.attach_name(match.person_id, 'Денис')
        store.record_encounter(_noisy_copy(_basis(0)), snapshot=_jpeg('a'))

        person_dir = tmp_path / match.person_id
        assert person_dir.exists()
        assert (person_dir / 'reference.jpg').exists()

    def test_strict_mode_never_creates_strangers(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_STRICT)
        m1 = store.record_encounter(_basis(0))
        m2 = store.record_encounter(_basis(0))
        assert m1.is_new is True and m2.is_new is True, (
            'strict не запоминает — каждая встреча с незнакомцем эфемерна'
        )
        assert store.people() == []
        assert store.identify(_basis(0)) is None
        assert list(tmp_path.iterdir()) == []

    def test_strict_mode_known_person_gets_single_centroid_no_snapshots(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        match = store.record_encounter(_basis(0))
        store.attach_name(match.person_id, 'Денис')
        store.set_mode(fs.MODE_STRICT)

        store.record_encounter(_noisy_copy(_basis(0)), snapshot=_jpeg('x'))
        store.record_encounter(_noisy_copy(_basis(0), seed=1), snapshot=_jpeg('y'))

        gallery = store.gallery(match.person_id)
        assert len(gallery) == 1, 'strict хранит один центроид, не галерею'
        person_dir = tmp_path / match.person_id
        assert not (person_dir / 'reference.jpg').exists()
        assert not (person_dir / 'encounters').exists() or not list(
            (person_dir / 'encounters').iterdir()
        )


# ============================================================================
# 5. Переходы режимов (ADR-0123 §5)
# ============================================================================

class TestModeTransitions:

    def test_workshop_to_exhibition_drops_strangers_keeps_named(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        stranger = store.record_encounter(_basis(0))
        named = store.record_encounter(_basis(2))
        store.attach_name(named.person_id, 'Денис')

        store.set_mode(fs.MODE_EXHIBITION)

        ids = {p['person_id'] for p in store.people()}
        assert named.person_id in ids
        assert stranger.person_id not in ids
        assert not (tmp_path / stranger.person_id).exists()
        assert (tmp_path / named.person_id).exists()

    def test_any_mode_to_strict_deletes_snapshots_and_strangers_collapses_gallery(
        self, tmp_path,
    ):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        stranger = store.record_encounter(_basis(0))
        named = store.record_encounter(_basis(2), snapshot=_jpeg('ref'))
        store.attach_name(named.person_id, 'Денис')
        store.record_encounter(_noisy_copy(_basis(2)), snapshot=_jpeg('enc-1'))

        store.set_mode(fs.MODE_STRICT)

        assert store.identify(_basis(0)) is None
        assert stranger.person_id not in {p['person_id'] for p in store.people()}
        assert not (tmp_path / stranger.person_id).exists()

        named_dir = tmp_path / named.person_id
        assert named_dir.exists()
        assert not (named_dir / 'reference.jpg').exists()
        assert len(store.gallery(named.person_id)) == 1

    def test_softer_mode_restores_nothing(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        stranger = store.record_encounter(_basis(0))
        named = store.record_encounter(_basis(2), snapshot=_jpeg('ref'))
        store.attach_name(named.person_id, 'Денис')

        store.set_mode(fs.MODE_STRICT)
        store.set_mode(fs.MODE_WORKSHOP)  # обратно в мягкий режим

        assert store.identify(_basis(0)) is None, 'незнакомец не воскресает'
        assert len(store.gallery(named.person_id)) == 1, (
            'галерея не восстанавливается до множественной'
        )
        assert not (tmp_path / named.person_id / 'reference.jpg').exists()


# ============================================================================
# 6. attach_name переживает рестарт
# ============================================================================

class TestAttachNameAndPersistence:

    def test_attach_name_survives_restart(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        match = store.record_encounter(_basis(0))
        ok = store.attach_name(match.person_id, 'Денис', speaker_id='sp-1')
        assert ok is True

        restarted = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        found = restarted.identify(_basis(0))
        assert found is not None
        assert found.person_id == match.person_id
        assert found.name == 'Денис'
        assert restarted.find_by_speaker('sp-1') == match.person_id

    def test_attach_name_unknown_person_returns_false(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        assert store.attach_name('does-not-exist', 'Кто-то') is False


# ============================================================================
# 7. merge
# ============================================================================

class TestMerge:

    def test_merge_moves_embeddings_and_snapshots_removes_drop(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        keep = store.record_encounter(_basis(0), snapshot=_jpeg('keep-ref'))
        store.attach_name(keep.person_id, 'Денис')
        drop = store.record_encounter(_basis(3), snapshot=_jpeg('drop-ref'))
        store.record_encounter(_noisy_copy(_basis(3)), snapshot=_jpeg('drop-enc'))

        ok = store.merge(keep.person_id, drop.person_id)
        assert ok is True

        assert not (tmp_path / drop.person_id).exists()
        ids = {p['person_id'] for p in store.people()}
        assert drop.person_id not in ids
        assert keep.person_id in ids

        gallery = store.gallery(keep.person_id)
        assert len(gallery) >= 2  # эмбеддинги drop'а перенесены

        keep_dir = tmp_path / keep.person_id
        assert (keep_dir / 'reference.jpg').read_bytes() == _jpeg('keep-ref')
        moved = list((keep_dir / 'encounters').glob('merged-*'))
        assert len(moved) >= 1

    def test_merge_unknown_ids_returns_false(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        real = store.record_encounter(_basis(0))
        assert store.merge(real.person_id, 'ghost') is False
        assert store.merge('ghost', real.person_id) is False
        assert store.merge(real.person_id, real.person_id) is False


# ============================================================================
# 8. forget
# ============================================================================

class TestForget:

    @pytest.mark.parametrize('mode', list(fs.VALID_MODES))
    def test_forget_removes_everything(self, tmp_path, mode):
        store = fs.FaceStore(root=str(tmp_path), mode=mode)
        match = store.record_encounter(_basis(0), snapshot=_jpeg('x'))
        if mode != fs.MODE_STRICT:
            # в strict незнакомец эфемерен — привяжем имя, чтобы было что забыть
            pass
        if mode == fs.MODE_STRICT:
            # strict не создаёт запись из незнакомца — сделаем известным заранее
            store2 = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
            match = store2.record_encounter(_basis(1))
            store2.attach_name(match.person_id, 'Денис')
            store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_STRICT)

        assert store.forget(match.person_id) is True
        assert match.person_id not in {p['person_id'] for p in store.people()}
        assert not (tmp_path / match.person_id).exists()
        assert store.forget(match.person_id) is False  # повторно — не падает


# ============================================================================
# 9. max_strangers
# ============================================================================

class TestMaxStrangers:

    def test_evicts_least_recently_seen_stranger_never_named(self, tmp_path):
        clock = {'t': 0.0}
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP, max_strangers=2,
            clock=lambda: clock['t'],
        )

        clock['t'] = 1.0
        named = store.record_encounter(_basis(5))
        store.attach_name(named.person_id, 'Денис')

        clock['t'] = 2.0
        oldest_stranger = store.record_encounter(_basis(0))
        clock['t'] = 3.0
        store.record_encounter(_basis(1))
        # третий незнакомец переполняет лимит (2) -> вытесняется самый давний
        clock['t'] = 4.0
        newest_stranger = store.record_encounter(_basis(2))

        ids = {p['person_id'] for p in store.people()}
        assert named.person_id in ids, 'знакомого никогда не вытесняют'
        assert oldest_stranger.person_id not in ids
        assert newest_stranger.person_id in ids
        strangers = [p for p in store.people() if p['is_stranger']]
        assert len(strangers) == 2


# ============================================================================
# 10. Битые метаданные не роняют конструктор
# ============================================================================

class TestCorruptMetadata:

    def test_corrupt_meta_json_is_skipped_not_fatal(self, tmp_path):
        good_dir = tmp_path / 'good-person'
        good_dir.mkdir()
        (good_dir / 'meta.json').write_text(
            json.dumps({
                'person_id': 'good-person',
                'name': 'Денис',
                'speaker_id': None,
                'created_at': 1.0,
                'encounter_count': 1,
                'last_encounter_ts': 1.0,
                'has_reference_snapshot': False,
                'encounters': [],
                'mode_recorded': 'workshop',
            }),
            encoding='utf-8',
        )
        np.save(good_dir / 'embeddings.npy', _basis(0).reshape(1, -1))

        bad_dir = tmp_path / 'bad-person'
        bad_dir.mkdir()
        (bad_dir / 'meta.json').write_text('{not valid json', encoding='utf-8')

        mismatched_dir = tmp_path / 'mismatched-person'
        mismatched_dir.mkdir()
        (mismatched_dir / 'meta.json').write_text(
            json.dumps({'person_id': 'someone-else'}), encoding='utf-8',
        )

        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)

        ids = {p['person_id'] for p in store.people()}
        assert ids == {'good-person'}
        found = store.identify(_basis(0))
        assert found is not None
        assert found.person_id == 'good-person'
        assert found.name == 'Денис'


# ============================================================================
# Разное: валидация входа
# ============================================================================

class TestValidation:

    def test_invalid_mode_raises(self, tmp_path):
        with pytest.raises(ValueError):
            fs.FaceStore(root=str(tmp_path), mode='ultra-secret')

    def test_embedding_dim_mismatch_raises(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        store.record_encounter(_basis(0))
        with pytest.raises(ValueError):
            store.record_encounter(np.zeros(10, dtype=np.float32))

    def test_stats_reports_mode_and_counts(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        match = store.record_encounter(_basis(0))
        store.attach_name(match.person_id, 'Денис')
        store.record_encounter(_basis(3))  # незнакомец

        stats = store.stats()
        assert stats['mode'] == fs.MODE_WORKSHOP
        assert stats['people'] == 2
        assert stats['named'] == 1
        assert stats['strangers'] == 1
        assert stats['embeddings'] == 2
        assert stats['disk_bytes'] >= 0
