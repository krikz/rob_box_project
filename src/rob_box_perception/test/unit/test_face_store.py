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
import logging
import math
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

    def test_identify_reports_runner_up_candidate(self, tmp_path):
        """issue #2771: identify() должен отдавать второго кандидата с
        его score — та же диагностика, что speaker_id_node уже печатает
        для голоса (``best=... second=... gap=...``)."""
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        alice = store.record_encounter(_combo(0.0))
        store.attach_name(alice.person_id, 'Алиса')
        bob = store.record_encounter(_combo(1.2))
        store.attach_name(bob.person_id, 'Боб')

        # Запрос ближе к Алисе (theta=0.1), но не идентичен — Боб (theta=1.2,
        # далеко) должен остаться единственным «вторым кандидатом», раз
        # других записей в базе больше нет.
        match = store.identify(_combo(0.1))
        assert match is not None
        assert match.person_id == alice.person_id
        assert match.runner_up_person_id == bob.person_id
        assert match.runner_up_similarity == pytest.approx(
            math.cos(1.2 - 0.1), abs=1e-5
        )

    def test_identify_runner_up_is_none_with_single_known_person(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        store.record_encounter(_basis(0))
        match = store.identify(_noisy_copy(_basis(0)))
        assert match is not None
        assert match.runner_up_person_id is None
        assert match.runner_up_similarity is None


# ============================================================================
# 1b. Развод порогов identify/enroll (issue #2772)
# ============================================================================

class TestIdentifyEnrollThresholdSplit:

    def test_score_between_identify_and_enroll_names_but_does_not_enroll(
        self, tmp_path,
    ):
        """Косинус, который проходит identify_threshold, но не дотягивает
        до enroll_threshold: встреча называется по имени (и засчитывается),
        но эмбеддинг НЕ попадает в галерею — это и есть фикс #2772
        (раньше один и тот же порог одновременно называл имя и дописывал
        чужое лицо в эталон).

        ``gallery_warmup_size=1`` выключает прогрев (issue #2771) — этот
        тест про СТРОГИЙ режим, который действует на созревшей галерее.
        На молодой галерее поведение теперь другое и проверяется в
        ``TestGalleryWarmup`` ниже: пока векторов мало, дозапись идёт по
        факту узнавания, иначе запись не может вырасти дальше семени.
        """
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP,
            identify_threshold=0.6, enroll_threshold=0.9,
            gallery_warmup_size=1,
        )
        seed = store.record_encounter(_combo(0.0))
        store.attach_name(seed.person_id, 'Деньчик')

        theta = float(np.arccos(0.75))  # между identify (0.6) и enroll (0.9)
        match = store.record_encounter(_combo(theta))

        assert match.is_new is False
        assert match.person_id == seed.person_id
        assert match.name == 'Деньчик', 'имя называется — identify прошёл'
        assert match.similarity == pytest.approx(0.75, abs=1e-3)
        assert len(store.gallery(seed.person_id)) == 1, (
            'enroll не прошёл — галерея не должна вырасти'
        )
        assert store.stats()['enroll_rejected_total'] == 1

    def test_score_above_enroll_threshold_does_enroll(self, tmp_path):
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP,
            identify_threshold=0.6, enroll_threshold=0.9,
        )
        seed = store.record_encounter(_combo(0.0))
        theta = float(np.arccos(0.95))  # выше enroll_threshold
        store.record_encounter(_combo(theta))

        assert len(store.gallery(seed.person_id)) == 2
        assert store.stats()['enroll_rejected_total'] == 0

    def test_mature_gallery_still_rejects_below_enroll(self, tmp_path):
        """Прогрев не отменяет защиту #2772, а откладывает её: как только
        галерея набрала ``gallery_warmup_size``, двойное согласие снова
        обязательно."""
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP,
            identify_threshold=0.6, enroll_threshold=0.9,
            gallery_warmup_size=3,
        )
        # Ракурсы разносим шагом 0.5 рад: соседи дают cos(0.5)=0.878 —
        # выше identify (0.6), ниже enroll (0.9). Шаг обязателен: если
        # слать один и тот же вектор, он после первой же дозаписи совпадёт
        # сам с собой и best-of-gallery станет 1.0, что проверяло бы
        # артефакт теста, а не строгий режим.
        seed = store.record_encounter(_combo(0.0))
        store.record_encounter(_combo(0.5))
        store.record_encounter(_combo(1.0))
        assert len(store.gallery(seed.person_id)) == 3, 'прогрев набрал галерею'
        assert store.stats()['enroll_rejected_total'] == 0

        # Галерея созрела — такой же по величине косинус теперь отвергается.
        store.record_encounter(_combo(1.5))
        assert len(store.gallery(seed.person_id)) == 3, (
            'после прогрева двойное согласие снова в силе'
        )
        assert store.stats()['enroll_rejected_total'] == 1

    def test_first_embedding_of_new_record_is_always_seeded(self, tmp_path):
        """Новая запись сеется первым эмбеддингом безусловно — enroll_threshold
        к семени не применяется (сравнивать ещё не с чем)."""
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP, enroll_threshold=0.99,
        )
        match = store.record_encounter(_basis(0))
        assert match.is_new is True
        assert len(store.gallery(match.person_id)) == 1
        assert store.stats()['enroll_rejected_total'] == 0


# ============================================================================
# 1b. Прогрев галереи (issue #2771)
# ============================================================================


class TestGalleryWarmup:
    """Пока галерея молодая, дозапись идёт по факту УЗНАВАНИЯ.

    Почему это вообще понадобилось — живой замер на Vision Pi 22.09.2026.
    Один человек за пять минут превратился в ТРИ записи: анфас, опущенная
    голова и тёмные очки. Попарные косинусы между сохранёнными векторами —
    0.538, 0.582 и 0.461, то есть ВЕСЬ внутриперсонный разброс лёг ниже
    ``enroll_threshold`` (0.75). Ни одна галерея не выросла дальше семени
    (`gal=1` у всех трёх), и в сводке ноды копился ``enroll_отклонено``.

    Ключевой момент, который эти тесты и стерегут: запись, которую УЖЕ
    узнали (``sim=0.737`` в живом логе), не попадала в галерею только
    из-за второго порога — 0.737 < 0.75. Именно этот отказ и держал
    галерею размером 1, а галерея размером 1 не переживает смену ракурса.
    """

    def test_warmup_enrolls_score_below_enroll_threshold(self, tmp_path):
        """Косинус между identify и enroll: на молодой галерее дописываем."""
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP,
            identify_threshold=0.6, enroll_threshold=0.75,
            gallery_warmup_size=5,
        )
        seed = store.record_encounter(_combo(0.0))
        # 0.737 — ровно тот скор, который живой робот отверг 22.09.2026.
        theta = float(np.arccos(0.737))
        match = store.record_encounter(_combo(theta))

        assert match.is_new is False, 'узнали того же человека'
        assert match.person_id == seed.person_id
        assert len(store.gallery(seed.person_id)) == 2, (
            'на прогреве узнанный эмбеддинг обязан попасть в галерею'
        )
        assert store.stats()['enroll_rejected_total'] == 0
        assert store.stats()['enroll_warmup_total'] == 1

    def test_warmup_stops_at_cap(self, tmp_path):
        """Прогрев не бесконечен — ровно ``gallery_warmup_size`` векторов."""
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP,
            identify_threshold=0.6, enroll_threshold=0.99,
            gallery_warmup_size=4,
        )
        # Шаг 0.2 рад: ближайший сосед даёт cos(0.2)=0.980 — identify
        # (0.6) проходит всегда, enroll (0.99) не проходит никогда, так
        # что после потолка каждая встреча обязана быть отвергнутой.
        #
        # Верхняя граница k подобрана так, чтобы САМЫЙ дальний ракурс всё
        # ещё узнавался галереей, застывшей на [0, 0.2, 0.4, 0.6]:
        # cos(1.4 - 0.6) = 0.697 > identify. Уехав дальше, тест перестал бы
        # проверять прогрев одного человека и начал бы заводить второго —
        # чей прогрев подмешался бы в тот же счётчик. Поэтому ниже стоит и
        # явная проверка, что человек в базе ровно один.
        seed = store.record_encounter(_combo(0.0))
        for k in range(1, 8):
            store.record_encounter(_combo(0.2 * k))

        stats = store.stats()
        assert stats['people'] == 1, 'все встречи — один и тот же человек'
        assert len(store.gallery(seed.person_id)) == 4
        assert stats['enroll_warmup_total'] == 3, 'семя прогревом не считается'
        assert stats['enroll_rejected_total'] == 4, 'после потолка — строгий режим'

    def test_warmup_does_not_apply_to_identify(self, tmp_path):
        """Прогрев трогает ТОЛЬКО дозапись. Порог узнавания не ослабляется:
        кандидат ниже ``identify_threshold`` по-прежнему заводит новую
        запись, даже если галерея молодая. Это сознательная граница —
        иначе прогрев начал бы склеивать разных людей, а не наращивать
        ракурсы одного."""
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP,
            identify_threshold=0.6, enroll_threshold=0.75,
            gallery_warmup_size=5,
        )
        seed = store.record_encounter(_combo(0.0))
        theta = float(np.arccos(0.50))  # ниже identify_threshold
        match = store.record_encounter(_combo(theta))

        assert match.is_new is True, 'ниже identify — это другой человек'
        assert match.person_id != seed.person_id
        assert len(store.gallery(seed.person_id)) == 1
        assert store.stats()['enroll_warmup_total'] == 0

    def test_warmup_size_is_reported_in_stats(self, tmp_path):
        """Размер прогрева видно в сводке — иначе по логу не отличить
        «галереи молодые» от «прогрев выключен конфигом»."""
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP, gallery_warmup_size=7,
        )
        assert store.stats()['gallery_warmup_size'] == 7


# ============================================================================
# 2. Рост галереи и вытеснение самого далёкого от медоида (issue #2772)
# ============================================================================

class TestGalleryEviction:

    def test_evicts_farthest_from_medoid_not_most_typical(self, tmp_path):
        """issue #2772: старое правило вытесняло самый ТИПИЧНЫЙ вектор
        («несёт меньше всего новой информации») — ровно наоборот тому,
        что нужно галерее одной личности. Новое правило вытесняет
        вектор, дальше всех от медоида галереи (выброс), а якорь
        (типичный, многократно подтверждённый ракурс) остаётся.

        Углы подобраны так же, как в прежнем тесте на вытеснение: при
        4 элементах медоид — theta=0.05 (index 2, максимальная сумма
        сходств к остальным трём), а дальше всех от него — theta=0.20
        (index 3, единственный «выброс» на отшибе кластера)."""
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP, max_embeddings=3,
        )
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
        assert _contains(embeddings[2]), (
            'theta=0.05 — медоид галереи (якорь), должен остаться'
        )
        assert not _contains(embeddings[3]), (
            'theta=0.20 — дальше всех от медоида (выброс), должен быть '
            'вытеснен первым (issue #2772)'
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
                # issue #2772/#2773: без совпадающей версии галерея этой
                # записи была бы отброшена при загрузке (см.
                # TestEmbeddingVersion ниже) — этот тест проверяет ДРУГОЕ
                # поведение (устойчивость к битым СОСЕДНИМ записям), поэтому
                # версия здесь намеренно актуальная.
                'embedding_version': fs.CURRENT_EMBEDDING_VERSION,
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
# 11. Версия эмбеддинга (issue #2772, #2773)
# ============================================================================

def _write_legacy_meta(person_dir: Path, person_id: str, **overrides: object) -> None:
    """Записать meta.json «руками», как лежал бы он на диске ДО этой
    карточки — помогает тестам подделать старую/отсутствующую версию
    эмбеддинга, не проходя через актуальный ``FaceStore._persist_record``
    (который теперь всегда пишет ``embedding_version`` сам)."""
    payload = {
        'person_id': person_id,
        'name': 'Деньчик',
        'speaker_id': 'sp-denchik',
        'created_at': 1.0,
        'encounter_count': 23,
        'last_encounter_ts': 5.0,
        'has_reference_snapshot': True,
        'encounters': [],
        'mode_recorded': 'workshop',
    }
    payload.update(overrides)
    (person_dir / 'meta.json').write_text(
        json.dumps(payload), encoding='utf-8',
    )


class TestEmbeddingVersion:

    def test_stale_version_drops_gallery_keeps_identity(self, tmp_path, caplog):
        """issue #2772/#2773: запись с версией эмбеддинга, не совпадающей
        с текущей, теряет ГАЛЕРЕЮ (несравнимые вектора), но сохраняет имя,
        speaker_id, эталонный снимок и счётчик встреч — это по-прежнему
        тот же человек, просто без (пока ещё) ни одного вектора в новой
        системе координат."""
        person_dir = tmp_path / 'b49470e1'
        person_dir.mkdir()
        _write_legacy_meta(person_dir, 'b49470e1', embedding_version=1)
        np.save(
            person_dir / 'embeddings.npy',
            np.stack([_basis(0), _basis(1), _basis(2)]),
        )
        (person_dir / 'reference.jpg').write_bytes(_jpeg('ref'))

        with caplog.at_level(logging.WARNING):
            store = fs.FaceStore(
                root=str(tmp_path), mode=fs.MODE_WORKSHOP,
                embedding_version=2,
            )

        people = store.people()
        assert len(people) == 1
        person = people[0]
        assert person['person_id'] == 'b49470e1'
        assert person['name'] == 'Деньчик'
        assert person['has_snapshot'] is True
        assert person['embeddings'] == 0
        assert person['gallery_cohesion'] is None
        assert store.gallery('b49470e1') == []
        assert store.find_by_speaker('sp-denchik') == 'b49470e1'
        assert (person_dir / 'reference.jpg').exists()
        assert any(
            'b49470e1' in rec.message and '1' in rec.message and '2' in rec.message
            for rec in caplog.records
        ), 'предупреждение обязано называть person_id и обе версии'

    def test_missing_version_field_is_treated_as_stale(self, tmp_path):
        """meta.json старее самого поля ``embedding_version`` (до этой
        карточки) — трактуется как несовпадение, не как «версия 1 по
        умолчанию»: угадывать здесь опаснее, чем честно сбросить галерею."""
        person_dir = tmp_path / 'legacy-no-field'
        person_dir.mkdir()
        _write_legacy_meta(person_dir, 'legacy-no-field')  # без embedding_version
        np.save(person_dir / 'embeddings.npy', _basis(0).reshape(1, -1))

        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)

        person = store.people()[0]
        assert person['person_id'] == 'legacy-no-field'
        assert person['name'] == 'Деньчик'
        assert person['embeddings'] == 0

    def test_matching_version_keeps_gallery(self, tmp_path):
        person_dir = tmp_path / 'fresh-person'
        person_dir.mkdir()
        _write_legacy_meta(
            person_dir, 'fresh-person', embedding_version=fs.CURRENT_EMBEDDING_VERSION,
        )
        np.save(person_dir / 'embeddings.npy', _basis(0).reshape(1, -1))

        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)

        person = store.people()[0]
        assert person['embeddings'] == 1
        assert store.identify(_basis(0)) is not None

    def test_record_survives_restart_with_empty_gallery_after_drop(self, tmp_path):
        """После сброса галереи (версия устарела) запись не мертва и не
        теряется на следующей загрузке: имя/speaker_id/эталон пережили
        и первый, и повторный рестарт — пустая галерея персистентна, а
        не «забывается» откатом к диску. Заново копить эмбеддинги в неё
        может либо обычный ``record_encounter`` с достаточно похожим
        вектором (сработает как для любой галереи из 0 элементов — она
        не участвует в ``_score_all``, поэтому первая же встреча после
        сброса заведёт СВОЙ person_id, а не допишется сюда сама по
        себе), либо явный ``merge()`` от шва «Знакомый» — то и другое
        вне контракта этого теста."""
        person_dir = tmp_path / 'b49470e1'
        person_dir.mkdir()
        _write_legacy_meta(person_dir, 'b49470e1', embedding_version=1)
        np.save(person_dir / 'embeddings.npy', _basis(0).reshape(1, -1))

        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        assert store.gallery('b49470e1') == []

        restarted = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        person = restarted.people()[0]
        assert person['person_id'] == 'b49470e1'
        assert person['embeddings'] == 0
        assert person['name'] == 'Деньчик'


# ============================================================================
# 12. gallery_cohesion (issue #2772, #2775)
# ============================================================================

class TestGalleryCohesion:

    def test_none_for_zero_or_one_embeddings(self, tmp_path):
        store = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        store.record_encounter(_basis(0))

        person = store.people()[0]
        assert person['gallery_cohesion'] is None
        assert store.stats()['gallery_cohesion'] is None

    def test_is_median_pairwise_cosine(self, tmp_path):
        store = fs.FaceStore(
            root=str(tmp_path), mode=fs.MODE_WORKSHOP, enroll_threshold=0.5,
        )
        thetas = [0.0, 0.1, 0.2]
        store.record_encounter(_combo(thetas[0]))
        for t in thetas[1:]:
            store.record_encounter(_combo(t))

        # Косинус между _combo(a) и _combo(b) — ровно cos(a - b) (см.
        # докстринг _combo), независимая от модуля формула для ожидания.
        pairwise = sorted([
            math.cos(thetas[1] - thetas[0]),
            math.cos(thetas[2] - thetas[0]),
            math.cos(thetas[2] - thetas[1]),
        ])
        expected_median = pairwise[1]

        person = store.people()[0]
        assert person['gallery_cohesion'] == pytest.approx(expected_median, abs=1e-5)
        assert store.stats()['gallery_cohesion'] == pytest.approx(
            expected_median, abs=1e-5
        )

    def test_healthy_vs_poisoned_gallery_cohesion_contrast(self, tmp_path):
        """Синтетическая версия живого разбора issue #2772: здоровая
        галерея (один и тот же человек, микро-шум) должна давать
        cohesion в районе ~0.9+, отравленная (несколько ортогональных
        «личностей» под одним именем) — заметно ниже. Числа взяты из
        карточки: здоровая ~0.9, отравленная ~0.3."""
        healthy = fs.FaceStore(
            root=str(tmp_path / 'healthy'), mode=fs.MODE_WORKSHOP,
            enroll_threshold=0.5,
        )
        healthy.record_encounter(_basis(0))
        for seed in range(1, 6):
            healthy.record_encounter(_noisy_copy(_basis(0), scale=0.01, seed=seed))
        healthy_cohesion = healthy.people()[0]['gallery_cohesion']

        poisoned = fs.FaceStore(
            root=str(tmp_path / 'poisoned'), mode=fs.MODE_WORKSHOP,
            identify_threshold=0.0, enroll_threshold=0.0,
        )
        # identify_threshold=0.0 заставляет ВСЁ подряд матчиться в одну
        # запись — синтетическая имитация «трёх разных людей под одним
        # именем» из живого разбора #2772 (там порог 0.45 сделал то же
        # самое на реальных лицах).
        poisoned.record_encounter(_basis(0))
        poisoned.record_encounter(_basis(1))
        poisoned.record_encounter(_basis(2))
        poisoned_cohesion = poisoned.people()[0]['gallery_cohesion']

        assert healthy_cohesion > 0.9
        assert poisoned_cohesion < 0.3
        assert healthy_cohesion > poisoned_cohesion


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
