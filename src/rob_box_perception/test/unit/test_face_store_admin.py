"""Юнит-тесты для scripts/maintenance/face_store_admin.py (issue #2775).

``scripts/maintenance/`` — не пакет (нет ``__init__.py``), поэтому модуль
импортируется напрямую по пути importlib'ом — тот же приём, что
``scripts/ci/tests/test_gen_build_matrix.py`` (см. его докстринг: «см.
tests/unit/scripts/test_tars_stats.py, scripts/testing/
test_patch_rtabmap_launch.py»).

Работаем ТОЛЬКО с синтетическими записями на ``tmp_path`` — ADR-0123 §5 /
докстринг ``test_face_store.py``: «Тесты режимов — тесты FaceStore, без
ROS и без камеры», этот файл следует тому же правилу для CLI поверх неё.
Ничего не подключается к живому роботу, ничего не запускалось на Vision
Pi — см. отчёт агента в PR (issue #2775: «на живом роботе ты ничего не
запускаешь»).

Покрываем (по списку из issue #2775):
  1. ``list`` — не падает, показывает person_id/name/speaker_id/встречи/
     галерею/косинус на синтетическом наборе (здоровая vs. "отравленная"
     галерея).
  2. ``forget`` — dry-run ничего не удаляет; ``--apply --yes`` удаляет
     запись целиком через настоящий ``FaceStore.forget()``.
  3. ``reset-gallery`` — dry-run ничего не меняет; ``--apply --yes``
     сносит галерею/encounters, СОХРАНЯЯ name/speaker_id/reference.jpg —
     и, важно, после сброса свежий ``FaceStore`` на том же root грузит
     запись корректно (round-trip через настоящий загрузчик face_store.py,
     не только "файл записался").
  4. Отказ без ``--force``, когда "нода похоже запущена" (через
     подставной ``node_check``, без реального ``pgrep``).
  5. ``rebuild_gallery()`` — чистая функция, подставные embedder/
     image_loader, без cv2 и без Hailo.
  6. CLI-обвязка ``rebuild`` честно отказывает (не падает трейсбеком),
     если embedder/cv2 недоступны — проверяем через monkeypatch на
     ``_default_image_loader``/``_default_embedder``, не через реальный
     cv2/Hailo (которых в тестовом окружении нет и не должно быть).

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_face_store_admin.py -q --no-cov
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np
import pytest

# ---------- импорт face_store.py (для построения синтетических данных) ----
_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]          # src/rob_box_perception
_REPO_ROOT = _HERE.parents[4]         # корень репозитория

if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))
import rob_box_perception.face_store as fs  # noqa: E402

# ---------- импорт scripts/maintenance/face_store_admin.py напрямую -------
_SCRIPT_PATH = _REPO_ROOT / 'scripts' / 'maintenance' / 'face_store_admin.py'
_SPEC = importlib.util.spec_from_file_location('face_store_admin_under_test', _SCRIPT_PATH)
assert _SPEC is not None and _SPEC.loader is not None, f'{_SCRIPT_PATH}: spec failed'
admin = importlib.util.module_from_spec(_SPEC)
# face_store_admin.py использует @dataclass (RebuildResult), а dataclasses
# резолвит аннотации типов через sys.modules[cls.__module__] — модуль
# должен быть зарегистрирован там ДО exec_module, иначе падает
# AttributeError на module_from_spec без sys.modules записи.
sys.modules[_SPEC.name] = admin
_SPEC.loader.exec_module(admin)  # type: ignore[union-attr]

DIM = 512  # тот же реальный размер ArcFace, что test_face_store.py


# ============================================================================
# Хелперы построения синтетических галерей (тот же приём, что test_face_store.py)
# ============================================================================

def _basis(index: int, dim: int = DIM) -> np.ndarray:
    v = np.zeros(dim, dtype=np.float32)
    v[index] = 1.0
    return v


def _noisy_copy(vec: np.ndarray, scale: float = 1e-4, seed: int = 0) -> np.ndarray:
    rng = np.random.RandomState(seed)
    noisy = vec + rng.normal(scale=scale, size=vec.shape).astype(np.float32)
    norm = np.linalg.norm(noisy)
    return (noisy / norm).astype(np.float32)


def _jpeg(tag: str) -> bytes:
    return f'fake-jpeg:{tag}'.encode('utf-8')


def _make_healthy_person(store: 'fs.FaceStore', tag: str) -> str:
    """Здоровая запись: несколько встреч почти одного и того же вектора —
    внутренний косинус галереи должен получиться близко к 1.0 (аналог
    "здоровая ~0.9" из issue #2775, здесь без шума порога специально
    выше, чтобы тест не был хрупким к точным цифрам)."""
    base = _basis(0)
    match = None
    for i in range(4):
        match = store.record_encounter(
            _noisy_copy(base, seed=i), snapshot=_jpeg(f'{tag}-{i}'),
            meta={'tag': tag},
        )
    assert match is not None
    store.attach_name(match.person_id, f'Здоровый-{tag}', speaker_id=f'spk-{tag}')
    return match.person_id


def _make_poisoned_person(store: 'fs.FaceStore', root: Path, tag: str) -> str:
    """"Отравленная" запись: галерея из трёх ОРТОГОНАЛЬНЫХ (разных)
    людей склеена в одну запись. В реальности так получилось на роботе
    через несколько ошибочных ``merge()`` (issue #2775: "Деньчик" — три
    разных человека в одной галерее) — ``FaceStore`` сама себя от этого
    не защищает, ``identify()`` не откажется добавить эмбеддинг, если
    вызывающий код (шов «Знакомый») ошибся с решением "это один человек".
    Здесь это моделируется напрямую: пишем первую встречу через
    настоящий API, затем ПОДМЕНЯЕМ embeddings.npy на диске тремя
    ортогональными векторами — тот же итоговый артефакт, без
    необходимости повторять весь путь ложных merge()."""
    match = store.record_encounter(_basis(1), snapshot=_jpeg(f'{tag}-0'), meta={'tag': tag})
    assert match is not None
    person_id = match.person_id
    store.attach_name(person_id, f'Смешанный-{tag}', speaker_id=f'spk-{tag}')

    person_dir = root / person_id
    arr = np.stack([_basis(1), _basis(2), _basis(3)]).astype(np.float32)
    np.save(person_dir / 'embeddings.npy', arr)
    return person_id


# ============================================================================
# Фикстуры
# ============================================================================

@pytest.fixture()
def store(tmp_path) -> 'fs.FaceStore':
    return fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)


def _never_running(pattern: str) -> Optional[bool]:
    return False


def _always_running(pattern: str) -> Optional[bool]:
    return True


def _make_args(**kwargs):
    """argparse.Namespace-подобный объект с дефолтами разрушительных подкоманд."""
    import argparse
    defaults = dict(
        apply=False, yes=True, force=False,
        node_process_pattern=admin.DEFAULT_NODE_PROCESS_PATTERN,
    )
    defaults.update(kwargs)
    return argparse.Namespace(**defaults)


# ============================================================================
# 1. list
# ============================================================================

class TestList:
    def test_list_runs_and_reports_both_records(self, store, tmp_path, capsys):
        healthy_id = _make_healthy_person(store, 'a')
        poisoned_id = _make_poisoned_person(store, tmp_path, 'b')

        args = admin.build_parser().parse_args(['--root', str(tmp_path), 'list'])
        rc = admin.cmd_list(args)
        out = capsys.readouterr().out

        assert rc == 0
        assert healthy_id in out
        assert poisoned_id in out
        assert 'Здоровый-a' in out
        assert 'Смешанный-b' in out

    def test_healthy_gallery_has_high_cohesion(self, store, tmp_path):
        healthy_id = _make_healthy_person(store, 'a')
        cohesion = admin._person_gallery_cohesion(store, {}, healthy_id)
        assert cohesion is not None
        assert cohesion['median'] > 0.9

    def test_poisoned_gallery_has_low_cohesion(self, store, tmp_path):
        poisoned_id = _make_poisoned_person(store, tmp_path, 'b')
        # Перечитываем store, чтобы подхватить эмбеддинги, дописанные
        # _make_poisoned_person напрямую на диск в обход API.
        store2 = fs.FaceStore(root=str(tmp_path))
        cohesion = admin._person_gallery_cohesion(store2, {}, poisoned_id)
        assert cohesion is not None
        # Ортогональные вектора -> косинус 0 между всеми парами.
        assert cohesion['median'] < 0.2

    def test_gallery_cohesion_reuses_existing_median_field_if_present(self):
        # face_store.py (параллельная карточка issue #2772/#2775) уже
        # отдаёт gallery_cohesion в people() как ОДИН float (медиана) —
        # не как dict. Наш код обязан переиспользовать именно это
        # значение для медианы, не пересчитывать её самостоятельно, и
        # досчитать только min (которого в апстриме нет).
        class _FakeStore:
            def gallery(self, person_id):
                return [_basis(1), _basis(2), _basis(3)]  # реальная медиана была бы 0.0

        fake_summary = {'gallery_cohesion': 0.42}  # заведомо не совпадает с реальным подсчётом
        result = admin._person_gallery_cohesion(_FakeStore(), fake_summary, 'whatever')
        assert result is not None
        assert result['median'] == 0.42  # переиспользовано, не пересчитано
        assert result['min'] == pytest.approx(0.0)  # досчитано локально из ортогональных векторов

    def test_gallery_cohesion_computes_median_itself_when_field_absent(self):
        class _FakeStore:
            def gallery(self, person_id):
                return [_basis(1), _basis(2), _basis(3)]

        result = admin._person_gallery_cohesion(_FakeStore(), {}, 'whatever')
        assert result is not None
        assert result['median'] == pytest.approx(0.0)
        assert result['min'] == pytest.approx(0.0)

    def test_empty_store_reports_and_does_not_crash(self, tmp_path, capsys):
        args = admin.build_parser().parse_args(['--root', str(tmp_path), 'list'])
        rc = admin.cmd_list(args)
        assert rc == 0
        assert 'Нет записей' in capsys.readouterr().out


# ============================================================================
# 2. forget
# ============================================================================

class TestForget:
    def test_dry_run_does_not_touch_disk(self, store, tmp_path):
        person_id = _make_healthy_person(store, 'a')
        person_dir = tmp_path / person_id
        before = sorted(p.name for p in person_dir.rglob('*'))

        args = _make_args(person_id=person_id, root=str(tmp_path), apply=False)
        rc = admin.cmd_forget(args)

        assert rc == 0
        assert person_dir.exists()
        after = sorted(p.name for p in person_dir.rglob('*'))
        assert before == after

    def test_apply_removes_record_via_face_store_forget(self, store, tmp_path, monkeypatch):
        person_id = _make_healthy_person(store, 'a')
        person_dir = tmp_path / person_id
        assert person_dir.exists()

        # Реальный pgrep недоступен в тестовом окружении (в т.ч. на CI
        # Windows-раннере) -> честный отказ "не удалось проверить" сам
        # по себе покрыт отдельным тестом (test_unknown_node_state_...);
        # здесь явно подтверждаем "нода не запущена", чтобы проверить
        # именно путь применения.
        monkeypatch.setattr(admin, '_node_running_via_pgrep', _never_running)

        args = _make_args(person_id=person_id, root=str(tmp_path), apply=True, yes=True)
        rc = admin.cmd_forget(args)

        assert rc == 0
        assert not person_dir.exists()

    def test_refuses_without_force_when_node_running(self, store, tmp_path):
        person_id = _make_healthy_person(store, 'a')
        args = _make_args(person_id=person_id, root=str(tmp_path), apply=True, yes=True, force=False)

        err = admin._ensure_node_not_running(
            args.force, args.node_process_pattern, node_check=_always_running,
        )
        assert err is not None
        assert 'нода' in err

        # Убеждаемся, что cmd_forget действительно использует этот гейт и
        # ничего не удаляет, если нода "запущена" (monkeypatch реального
        # _node_running_via_pgrep на всегда True).
        person_dir = tmp_path / person_id
        original = admin._node_running_via_pgrep
        admin._node_running_via_pgrep = _always_running  # type: ignore[assignment]
        try:
            rc = admin.cmd_forget(args)
        finally:
            admin._node_running_via_pgrep = original  # type: ignore[assignment]

        assert rc == 3
        assert person_dir.exists()  # ничего не тронуто

    def test_unknown_node_state_also_requires_force(self):
        def _unknown(pattern: str) -> Optional[bool]:
            return None

        err = admin._ensure_node_not_running(False, 'vision_face_node', node_check=_unknown)
        assert err is not None
        assert 'не удалось проверить' in err

    def test_force_bypasses_node_check(self, store, tmp_path):
        person_id = _make_healthy_person(store, 'a')
        args = _make_args(person_id=person_id, root=str(tmp_path), apply=True, yes=True, force=True)

        original = admin._node_running_via_pgrep
        admin._node_running_via_pgrep = _always_running  # type: ignore[assignment]
        try:
            rc = admin.cmd_forget(args)
        finally:
            admin._node_running_via_pgrep = original  # type: ignore[assignment]

        assert rc == 0
        assert not (tmp_path / person_id).exists()

    def test_missing_person_id_is_clean_error(self, tmp_path):
        args = _make_args(person_id='does-not-exist', root=str(tmp_path), apply=True, yes=True)
        rc = admin.cmd_forget(args)
        assert rc == 2


# ============================================================================
# 3. reset-gallery
# ============================================================================

class TestResetGallery:
    def test_dry_run_does_not_touch_disk(self, store, tmp_path):
        person_id = _make_healthy_person(store, 'a')
        person_dir = tmp_path / person_id
        before = {p: p.read_bytes() for p in person_dir.rglob('*') if p.is_file()}

        args = _make_args(person_id=person_id, root=str(tmp_path), apply=False)
        rc = admin.cmd_reset_gallery(args)

        assert rc == 0
        after = {p: p.read_bytes() for p in person_dir.rglob('*') if p.is_file()}
        assert before == after

    def test_apply_wipes_gallery_keeps_name_speaker_reference(self, store, tmp_path, monkeypatch):
        person_id = _make_healthy_person(store, 'a')
        person_dir = tmp_path / person_id
        reference_before = (person_dir / 'reference.jpg').read_bytes()
        assert (person_dir / 'embeddings.npy').exists()
        assert (person_dir / 'encounters').exists()
        assert any((person_dir / 'encounters').iterdir())

        monkeypatch.setattr(admin, '_node_running_via_pgrep', _never_running)
        args = _make_args(person_id=person_id, root=str(tmp_path), apply=True, yes=True)
        rc = admin.cmd_reset_gallery(args)
        assert rc == 0

        # embeddings.npy и encounters/ снесены...
        assert not (person_dir / 'embeddings.npy').exists()
        assert not (person_dir / 'encounters').exists() or not any((person_dir / 'encounters').iterdir())
        # ...reference.jpg — цел и байт-в-байт тот же...
        assert (person_dir / 'reference.jpg').read_bytes() == reference_before

        meta = json.loads((person_dir / 'meta.json').read_text(encoding='utf-8'))
        assert meta['name'] == 'Здоровый-a'
        assert meta['speaker_id'] == 'spk-a'
        assert meta['person_id'] == person_id
        assert meta['encounter_count'] == 0
        assert meta['encounters'] == []
        assert meta['has_reference_snapshot'] is True

    def test_round_trip_through_real_face_store_after_reset(self, store, tmp_path, monkeypatch):
        """Ключевая проверка совместимости формата: после reset-gallery
        НАСТОЯЩИЙ FaceStore, открытый заново на том же root (как сделает
        нода при перезапуске), обязан загрузить запись без ошибок, с
        пустой галереей и сохранённым именем/speaker_id — а не просто
        "файл похож на правильный"."""
        monkeypatch.setattr(admin, '_node_running_via_pgrep', _never_running)
        person_id = _make_healthy_person(store, 'a')
        args = _make_args(person_id=person_id, root=str(tmp_path), apply=True, yes=True)
        assert admin.cmd_reset_gallery(args) == 0

        reopened = fs.FaceStore(root=str(tmp_path), mode=fs.MODE_WORKSHOP)
        people = {p['person_id']: p for p in reopened.people()}
        assert person_id in people
        assert people[person_id]['name'] == 'Здоровый-a'
        assert people[person_id]['embeddings'] == 0
        assert people[person_id]['encounter_count'] == 0
        assert reopened.find_by_speaker('spk-a') == person_id

    def test_refuses_without_force_when_node_running(self, store, tmp_path):
        person_id = _make_healthy_person(store, 'a')
        person_dir = tmp_path / person_id
        args = _make_args(person_id=person_id, root=str(tmp_path), apply=True, yes=True, force=False)

        original = admin._node_running_via_pgrep
        admin._node_running_via_pgrep = _always_running  # type: ignore[assignment]
        try:
            rc = admin.cmd_reset_gallery(args)
        finally:
            admin._node_running_via_pgrep = original  # type: ignore[assignment]

        assert rc == 3
        assert (person_dir / 'embeddings.npy').exists()  # ничего не тронуто

    def test_missing_person_id_is_clean_error(self, tmp_path):
        args = _make_args(person_id='does-not-exist', root=str(tmp_path), apply=True, yes=True)
        rc = admin.cmd_reset_gallery(args)
        assert rc == 2


# ============================================================================
# 4. rebuild_gallery — чистая функция, подставные embedder/image_loader
# ============================================================================

class _FakeEmbedder:
    """Подставной эмбеддер: детерминированно превращает "изображение"
    (у нас — просто маленький ndarray-маркер) в вектор через его первый
    байт как индекс базиса. Имитирует контракт ArcFaceEmbedder.embed()
    без cv2/Hailo."""

    def __init__(self, fail_indices=()):
        self._fail_indices = set(fail_indices)
        self.closed = False

    def embed(self, crops: List[np.ndarray]) -> List[Optional[np.ndarray]]:
        out = []
        for i, crop in enumerate(crops):
            if i in self._fail_indices:
                out.append(None)
                continue
            marker = int(crop.flat[0])
            out.append(_basis(marker % DIM))
        return out

    def close(self):
        self.closed = True


def _fake_image_loader(data: bytes) -> Optional[np.ndarray]:
    if data == b'undecodable':
        return None
    # Кодируем "какой это маркер" в первый байт данных, чтобы
    # _FakeEmbedder мог его прочитать без реального декодирования JPEG.
    return np.array([data[0]], dtype=np.uint8)


class TestRebuildGallery:
    def test_rebuilds_from_reference_and_encounter_faces(self, tmp_path):
        person_dir = tmp_path / 'person-1'
        (person_dir / 'encounters').mkdir(parents=True)
        (person_dir / 'reference.jpg').write_bytes(bytes([10]))
        (person_dir / 'encounters' / '000001_face.jpg').write_bytes(bytes([11]))
        (person_dir / 'encounters' / '000002_face.jpg').write_bytes(bytes([12]))
        # Кроп тела — не лицо, ДОЛЖЕН быть проигнорирован.
        (person_dir / 'encounters' / '000002_body.jpg').write_bytes(bytes([99]))

        embedder = _FakeEmbedder()
        result = admin.rebuild_gallery(person_dir, embedder, _fake_image_loader)

        assert result.embedded == 3  # reference + 2 face-снимка, body исключён
        assert result.skipped == 0
        assert result.dim == DIM

        arr = np.load(person_dir / 'embeddings.npy')
        assert arr.shape == (3, DIM)

    def test_skips_undecodable_and_failed_embeddings(self, tmp_path):
        person_dir = tmp_path / 'person-2'
        (person_dir / 'encounters').mkdir(parents=True)
        (person_dir / 'reference.jpg').write_bytes(b'undecodable')
        (person_dir / 'encounters' / '000001_face.jpg').write_bytes(bytes([1]))
        (person_dir / 'encounters' / '000002_face.jpg').write_bytes(bytes([2]))

        # embed() провалит первый успешно декодированный кроп (индекс 0
        # среди decoded crops == 000001_face.jpg).
        embedder = _FakeEmbedder(fail_indices={0})
        result = admin.rebuild_gallery(person_dir, embedder, _fake_image_loader)

        # reference.jpg не декодировался (undecodable) -> skip.
        # 000001_face.jpg декодировался, но embed() дал None -> skip.
        # 000002_face.jpg -> успех.
        assert result.embedded == 1
        assert result.skipped == 2

    def test_no_sources_writes_empty_gallery(self, tmp_path):
        person_dir = tmp_path / 'person-3'
        person_dir.mkdir()
        embedder = _FakeEmbedder()
        result = admin.rebuild_gallery(person_dir, embedder, _fake_image_loader)

        assert result.embedded == 0
        arr = np.load(person_dir / 'embeddings.npy')
        assert arr.shape[0] == 0

    def test_max_embeddings_caps_to_most_recent(self, tmp_path):
        person_dir = tmp_path / 'person-4'
        (person_dir / 'encounters').mkdir(parents=True)
        for i in range(5):
            (person_dir / 'encounters' / f'{i:06d}_face.jpg').write_bytes(bytes([i]))

        embedder = _FakeEmbedder()
        result = admin.rebuild_gallery(person_dir, embedder, _fake_image_loader, max_embeddings=2)

        assert result.embedded == 2
        arr = np.load(person_dir / 'embeddings.npy')
        assert arr.shape == (2, DIM)


# ============================================================================
# 5. rebuild — CLI-обвязка честно отказывает без embedder/cv2
# ============================================================================

class TestRebuildCliHonestFailure:
    def test_refuses_cleanly_when_image_loader_unavailable(self, store, tmp_path, monkeypatch):
        person_id = _make_healthy_person(store, 'a')
        monkeypatch.setattr(admin, '_node_running_via_pgrep', _never_running)

        def _boom():
            raise ImportError('cv2 недоступен (симулировано тестом)')

        monkeypatch.setattr(admin, '_default_image_loader', _boom)

        args = _make_args(
            person_id=person_id, root=str(tmp_path), apply=True, yes=True,
            hef_path='/opt/rob_box/models/arcface_mobilefacenet.hef',
        )
        rc = admin.cmd_rebuild(args)

        assert rc == 5  # честный отказ, не traceback/exit по исключению

    def test_dry_run_never_touches_embedder(self, store, tmp_path, monkeypatch):
        person_id = _make_healthy_person(store, 'a')
        called = {'n': 0}

        def _should_not_be_called():
            called['n'] += 1
            raise AssertionError('dry-run не должен трогать embedder/cv2')

        monkeypatch.setattr(admin, '_default_image_loader', _should_not_be_called)
        monkeypatch.setattr(admin, '_default_embedder', lambda hef: _should_not_be_called())

        args = _make_args(
            person_id=person_id, root=str(tmp_path), apply=False,
            hef_path='/opt/rob_box/models/arcface_mobilefacenet.hef',
        )
        rc = admin.cmd_rebuild(args)

        assert rc == 0
        assert called['n'] == 0

    def test_missing_person_id_is_clean_error(self, tmp_path):
        args = _make_args(
            person_id='does-not-exist', root=str(tmp_path), apply=True, yes=True,
            hef_path='/opt/rob_box/models/arcface_mobilefacenet.hef',
        )
        rc = admin.cmd_rebuild(args)
        assert rc == 2


# ============================================================================
# 6. Подтверждение / --yes
# ============================================================================

class TestConfirm:
    def test_yes_flag_skips_prompt(self):
        assert admin._confirm('irrelevant', assume_yes=True) is True

    def test_no_tty_without_yes_is_a_refusal(self, monkeypatch):
        def _raise_eof(_prompt):
            raise EOFError()

        monkeypatch.setattr('builtins.input', _raise_eof)
        assert admin._confirm('irrelevant', assume_yes=False) is False
