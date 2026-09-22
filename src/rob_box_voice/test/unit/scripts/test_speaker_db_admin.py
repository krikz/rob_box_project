"""Юнит-тесты для scripts/maintenance/speaker_db_admin.py (issue #2777).

Почему тест лежит именно здесь
-------------------------------
``speaker_db_admin.py`` живёт в ``scripts/maintenance/`` (корень
репозитория), а не внутри пакета ``rob_box_voice`` — как и его сосед для
лиц, ``face_store_admin.py``. Для тестов ИМЕННО таких repo-root-скриптов,
принадлежащих голосовому пакету, в этом репозитории уже есть устоявшееся
место: ``test/unit/scripts/`` (см. ``test_chunk_latency_bench_stats.py`` —
тестирует ``scripts/tts_bench/chunk_latency_bench.py`` тем же приёмом:
``REPO_ROOT = Path(__file__).resolve().parents[5]``). ``test/unit/utils/``
— для внутренних модулей самого пакета (``rob_box_voice/utils/*.py``,
там же тесты ``speaker_embeddings.py``/``identity_seam.py``/
``legacy_voice_facts.py``, которые этот CLI использует, но не является
частью пакета сам). Кладём тест туда, где уже есть прецедент для того же
жанра файла, а не изобретаем новую директорию.

Работаем ТОЛЬКО с синтетическими БД на ``tmp_path``. Ничего не подключается
к живому роботу, ничего не запускалось на Vision Pi — см. отчёт агента в
PR (issue #2777: «на живом роботе ты ничего не запускаешь»).

Покрываем (по списку из issue #2777):
  1. ``list`` — таблица профилей, попарная диагностика дублей (пара
     похожих голосов подсвечена, пара разных — нет), пустая/отсутствующая БД.
  2. ``merge`` — dry-run ничего не меняет (побитово); ``--apply --yes``
     переносит эмбеддинги (src исчезает, имя dst сохраняется) И факты из
     ОБОИХ писателей (``harness_voice.db`` через ``VoiceIdentitySeam`` /
     ``voice_memory.db`` легаси voice_facts, issue #2751) — то есть тест
     проверяет, что CLI действительно зовёт ``VoiceIdentitySeam.merge``,
     а не голый ``SpeakerDatabase.merge_speakers()``; отказ без ``--force``
     при "нода похоже запущена"/"не удалось проверить"; src==dst и
     несуществующие id — честные отказы без всякой записи на диск.
  3. ``delete`` — dry-run не трогает диск; ``--apply --yes`` удаляет профиль.
  4. Бэкап перед мутацией — создаётся, имя НЕ совпадает с форматом чужого
     ``.bak-%Y%m%dT%H%M%SZ`` (issue #2750).
  5. Гейт на "живую ноду" (тот же приём, что ``face_store_admin.py``).
  6. Регрессия issue #2775 (тот же класс бага пойман и здесь ДО того, как
     он успел повториться): ``_guess_pkg_root`` не падает ``IndexError``
     на мелких путях, модуль, "доставленный" в мелкий путь, доходит до
     ``build_parser()``/``parse_args()``, если пакеты уже импортируемы.

Запуск:
    python -m pytest src/rob_box_voice/test/unit/scripts/test_speaker_db_admin.py -q --no-cov
"""

from __future__ import annotations

import asyncio
import importlib.machinery
import importlib.util
import sqlite3
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import pytest

# ---------- пути ------------------------------------------------------------
_HERE = Path(__file__).resolve()
_REPO_ROOT = _HERE.parents[5]  # test/unit/scripts/этот_файл -> .../scripts -> ... -> repo root
_VOICE_PKG_ROOT = _REPO_ROOT / 'src' / 'rob_box_voice' / 'rob_box_voice'
_SCRIPT_PATH = _REPO_ROOT / 'scripts' / 'maintenance' / 'speaker_db_admin.py'


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None, f'{path}: spec failed'
    mod = importlib.util.module_from_spec(spec)
    # dataclasses резолвит аннотации типов через sys.modules[cls.__module__] —
    # модуль должен быть в sys.modules ДО exec_module (тот же приём, что
    # test_face_store_admin.py / test_speaker_embeddings.py этого репозитория).
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------- rob_box_voice.utils.speaker_embeddings — минуя utils/__init__.py
# (тянет pyaudio, см. test_speaker_embeddings.py) --------------------------
if 'rob_box_voice.utils.speaker_embeddings' in sys.modules:
    se = sys.modules['rob_box_voice.utils.speaker_embeddings']
else:
    se = _load_module(
        'rob_box_voice.utils.speaker_embeddings', _VOICE_PKG_ROOT / 'utils' / 'speaker_embeddings.py'
    )

# rob_box_harness пип-установлен editable в дев-окружении этого репозитория
# (см. отчёт агента) — обычный импорт, без файловых трюков.
from rob_box_harness.memory import SQLiteVoiceMemory, touch_speaker  # noqa: E402

# ---------- speaker_db_admin.py напрямую по пути ---------------------------
admin = _load_module('speaker_db_admin_under_test', _SCRIPT_PATH)

DIM = 32


# ============================================================================
# Хелперы
# ============================================================================

def _basis(index: int, dim: int = DIM) -> np.ndarray:
    v = np.zeros(dim, dtype=np.float32)
    v[index] = 1.0
    return v


def _noisy_copy(vec: np.ndarray, scale: float = 1e-3, seed: int = 0) -> np.ndarray:
    rng = np.random.RandomState(seed)
    noisy = vec + rng.normal(scale=scale, size=vec.shape).astype(np.float32)
    return (noisy / np.linalg.norm(noisy)).astype(np.float32)


def _never_running(pattern: str) -> Optional[bool]:
    return False


def _always_running(pattern: str) -> Optional[bool]:
    return True


def _unknown_running(pattern: str) -> Optional[bool]:
    return None


def _make_args(**kwargs):
    import argparse

    defaults = dict(
        apply=False, yes=True, force=False,
        node_process_pattern=admin.DEFAULT_NODE_PROCESS_PATTERN,
        memory_db_path=None, legacy_facts_db_path=None,
        no_similarity=False, similarity_threshold=None,
    )
    defaults.update(kwargs)
    return argparse.Namespace(**defaults)


def _make_voice_memory_db(path: Path) -> None:
    """Легаси-схема voice_facts (core/voice_memory.py), см. test_legacy_voice_facts.py."""
    conn = sqlite3.connect(str(path))
    conn.executescript(
        """
        CREATE TABLE voice_facts (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            fact TEXT NOT NULL,
            category TEXT NOT NULL DEFAULT 'general',
            speaker_id TEXT,
            created_at REAL NOT NULL,
            updated_at REAL NOT NULL
        );
        """
    )
    conn.commit()
    conn.close()


def _insert_legacy_fact(path: Path, fact: str, speaker_id: str) -> None:
    conn = sqlite3.connect(str(path))
    conn.execute(
        "INSERT INTO voice_facts (fact, category, speaker_id, created_at, updated_at) "
        "VALUES (?, 'general', ?, 0, 0)",
        (fact, speaker_id),
    )
    conn.commit()
    conn.close()


def _run(coro):
    return asyncio.run(coro)


# ============================================================================
# 1. list
# ============================================================================

class TestList:
    def test_missing_db_reports_and_does_not_create_file(self, tmp_path, capsys):
        db_path = tmp_path / 'speakers.db'
        args = admin.build_parser().parse_args(['--db-path', str(db_path), 'list'])
        rc = admin.cmd_list(args)
        out = capsys.readouterr().out
        assert rc == 0
        assert 'БД не найдена' in out
        assert not db_path.exists(), 'list не должен создавать БД, только читать'

    def test_empty_db_reports_and_does_not_crash(self, tmp_path, capsys):
        db_path = tmp_path / 'speakers.db'
        db = se.SpeakerDatabase(str(db_path))
        db.close()

        args = admin.build_parser().parse_args(['--db-path', str(db_path), 'list'])
        rc = admin.cmd_list(args)
        out = capsys.readouterr().out
        assert rc == 0
        assert 'нет ни одного профиля' in out

    def test_lists_profiles_and_flags_similar_pair(self, tmp_path, capsys):
        db_path = tmp_path / 'speakers.db'
        db = se.SpeakerDatabase(str(db_path))
        base = _basis(0)
        src_id = db.register('Деньчик', _noisy_copy(base, seed=1))
        dst_id = db.register('Дэнчик', _noisy_copy(base, seed=2))
        other_id = db.register('Саша', _basis(5))
        db.close()

        args = admin.build_parser().parse_args(['--db-path', str(db_path), 'list'])
        rc = admin.cmd_list(args)
        out = capsys.readouterr().out

        assert rc == 0
        assert src_id in out and dst_id in out and other_id in out
        assert 'Возможные дубли' in out
        # Похожая пара — оба в предупреждении; несвязанный профиль — не пара с ними.
        assert src_id in out.split('Возможные дубли')[1]
        assert dst_id in out.split('Возможные дубли')[1]
        assert other_id not in out.split('Возможные дубли')[1]

    def test_no_similarity_flag_skips_pairwise_section(self, tmp_path, capsys):
        db_path = tmp_path / 'speakers.db'
        db = se.SpeakerDatabase(str(db_path))
        base = _basis(0)
        db.register('Деньчик', _noisy_copy(base, seed=1))
        db.register('Дэнчик', _noisy_copy(base, seed=2))
        db.close()

        args = admin.build_parser().parse_args(['--db-path', str(db_path), 'list', '--no-similarity'])
        rc = admin.cmd_list(args)
        out = capsys.readouterr().out
        assert rc == 0
        assert 'Возможные дубли' not in out
        assert 'пропущено' in out


# ============================================================================
# 2. Попарная близость — тестируем логику отдельно от list (детерминированно)
# ============================================================================

class TestPairwiseSimilarity:
    def test_orthogonal_vectors_score_zero(self):
        emb = {'a': [_basis(0)], 'b': [_basis(1)]}
        pairs = admin._pairwise_speaker_similarity(emb)
        assert pairs == [('a', 'b', pytest.approx(0.0, abs=1e-6))]

    def test_near_identical_vectors_score_near_one(self):
        base = _basis(0)
        emb = {'a': [_noisy_copy(base, seed=1)], 'b': [_noisy_copy(base, seed=2)]}
        pairs = admin._pairwise_speaker_similarity(emb)
        assert pairs[0][2] > 0.99

    def test_uses_max_across_gallery_not_mean(self):
        # Галерея 'a' содержит один вектор, похожий на 'b', и один — непохожий.
        # MAX должен взять похожий, а не средний (см. докстринг функции).
        base = _basis(0)
        emb = {
            'a': [_noisy_copy(base, seed=1), _basis(7)],
            'b': [_noisy_copy(base, seed=2)],
        }
        pairs = admin._pairwise_speaker_similarity(emb)
        assert pairs[0][2] > 0.99


# ============================================================================
# 3. merge
# ============================================================================

class TestMerge:
    def _two_speakers(self, tmp_path):
        db_path = tmp_path / 'speakers.db'
        db = se.SpeakerDatabase(str(db_path))
        base = _basis(0)
        src_id = db.register('Деньчик', _noisy_copy(base, seed=1))
        dst_id = db.register('Дэнчик', _noisy_copy(base, seed=2))
        db.close()
        return db_path, src_id, dst_id

    def test_dry_run_does_not_touch_disk(self, tmp_path):
        db_path, src_id, dst_id = self._two_speakers(tmp_path)
        before = db_path.read_bytes()

        args = _make_args(
            db_path=str(db_path), src=src_id, dst=dst_id,
            memory_db_path=str(tmp_path / 'harness_voice.db'),
            legacy_facts_db_path=str(tmp_path / 'voice_memory.db'),
            apply=False,
        )
        rc = admin.cmd_merge(args)

        assert rc == 0
        assert db_path.read_bytes() == before
        assert not (tmp_path / 'harness_voice.db').exists()
        assert not (tmp_path / 'voice_memory.db').exists()

    def test_apply_moves_embeddings_src_disappears_dst_name_kept(self, tmp_path, monkeypatch):
        db_path, src_id, dst_id = self._two_speakers(tmp_path)
        monkeypatch.setattr(admin, '_node_running_via_pgrep', _never_running)

        args = _make_args(
            db_path=str(db_path), src=src_id, dst=dst_id,
            memory_db_path=str(tmp_path / 'harness_voice.db'),
            legacy_facts_db_path=str(tmp_path / 'voice_memory.db'),
            apply=True, yes=True,
        )
        rc = admin.cmd_merge(args)
        assert rc == 0

        db = se.SpeakerDatabase(str(db_path))
        try:
            speakers = {s['id']: s for s in db.list_speakers()}
        finally:
            db.close()
        assert src_id not in speakers
        assert dst_id in speakers
        assert speakers[dst_id]['name'] == 'Дэнчик'  # имя dst сохранено, не переписано
        assert speakers[dst_id]['embeddings'] == 2  # оба эмбеддинга под dst

    def test_apply_moves_facts_from_both_writers(self, tmp_path, monkeypatch):
        """Ключевая проверка: CLI обязан звать VoiceIdentitySeam.merge (эмбеддинги +
        ОБА писателя фактов), а не голый SpeakerDatabase.merge_speakers() — см.
        докстринг модуля §«Почему merge зовёт VoiceIdentitySeam.merge»."""
        db_path, src_id, dst_id = self._two_speakers(tmp_path)
        monkeypatch.setattr(admin, '_node_running_via_pgrep', _never_running)

        memory_db = tmp_path / 'harness_voice.db'
        legacy_db = tmp_path / 'voice_memory.db'

        # 1 факт профиля src в harness_voice.db (через тот же touch_speaker,
        # что использует шов идентичности в проде).
        store = SQLiteVoiceMemory(db_path=str(memory_db))
        _run(store.init())
        _run(touch_speaker(store, src_id, now=100.0))
        _run(store.teardown())

        # 2 живых факта src в voice_memory.db (легаси-писатель memory_save, issue #2751).
        _make_voice_memory_db(legacy_db)
        _insert_legacy_fact(legacy_db, 'не ест лук', src_id)
        _insert_legacy_fact(legacy_db, 'пьёт чай без сахара', src_id)

        args = _make_args(
            db_path=str(db_path), src=src_id, dst=dst_id,
            memory_db_path=str(memory_db), legacy_facts_db_path=str(legacy_db),
            apply=True, yes=True,
        )
        rc = admin.cmd_merge(args)
        assert rc == 0

        conn = sqlite3.connect(str(memory_db))
        rows = conn.execute("SELECT scope FROM facts").fetchall()
        conn.close()
        assert rows, 'профильный факт должен был переехать в harness_voice.db'
        assert all(r[0] == f'speaker:{dst_id}' for r in rows)

        conn = sqlite3.connect(str(legacy_db))
        rows = conn.execute("SELECT speaker_id FROM voice_facts").fetchall()
        conn.close()
        assert len(rows) == 2
        assert {r[0] for r in rows} == {dst_id}

    def test_backup_created_with_non_colliding_name(self, tmp_path, monkeypatch):
        db_path, src_id, dst_id = self._two_speakers(tmp_path)
        monkeypatch.setattr(admin, '_node_running_via_pgrep', _never_running)

        args = _make_args(
            db_path=str(db_path), src=src_id, dst=dst_id,
            memory_db_path=str(tmp_path / 'harness_voice.db'),
            legacy_facts_db_path=str(tmp_path / 'voice_memory.db'),
            apply=True, yes=True,
        )
        assert admin.cmd_merge(args) == 0

        backups = list(tmp_path.glob('speakers.db.*'))
        assert len(backups) == 1
        name = backups[0].name
        assert admin._BACKUP_TAG in name
        assert '.bak-' not in name  # issue #2750 — не путать с чужим форматом

    def test_refuses_without_force_when_node_running(self, tmp_path):
        db_path, src_id, dst_id = self._two_speakers(tmp_path)
        before = db_path.read_bytes()

        args = _make_args(
            db_path=str(db_path), src=src_id, dst=dst_id,
            memory_db_path=str(tmp_path / 'harness_voice.db'),
            legacy_facts_db_path=str(tmp_path / 'voice_memory.db'),
            apply=True, yes=True, force=False,
        )
        original = admin._node_running_via_pgrep
        admin._node_running_via_pgrep = _always_running  # type: ignore[assignment]
        try:
            rc = admin.cmd_merge(args)
        finally:
            admin._node_running_via_pgrep = original  # type: ignore[assignment]

        assert rc == 3
        assert db_path.read_bytes() == before  # ничего не тронуто

    def test_unknown_node_state_also_requires_force(self, tmp_path):
        db_path, src_id, dst_id = self._two_speakers(tmp_path)

        args = _make_args(
            db_path=str(db_path), src=src_id, dst=dst_id,
            memory_db_path=str(tmp_path / 'harness_voice.db'),
            legacy_facts_db_path=str(tmp_path / 'voice_memory.db'),
            apply=True, yes=True, force=False,
        )
        original = admin._node_running_via_pgrep
        admin._node_running_via_pgrep = _unknown_running  # type: ignore[assignment]
        try:
            rc = admin.cmd_merge(args)
        finally:
            admin._node_running_via_pgrep = original  # type: ignore[assignment]
        assert rc == 3

    def test_force_bypasses_node_check(self, tmp_path):
        db_path, src_id, dst_id = self._two_speakers(tmp_path)

        args = _make_args(
            db_path=str(db_path), src=src_id, dst=dst_id,
            memory_db_path=str(tmp_path / 'harness_voice.db'),
            legacy_facts_db_path=str(tmp_path / 'voice_memory.db'),
            apply=True, yes=True, force=True,
        )
        original = admin._node_running_via_pgrep
        admin._node_running_via_pgrep = _always_running  # type: ignore[assignment]
        try:
            rc = admin.cmd_merge(args)
        finally:
            admin._node_running_via_pgrep = original  # type: ignore[assignment]
        assert rc == 0

    def test_src_equals_dst_is_a_clean_refusal(self, tmp_path):
        db_path, src_id, _dst_id = self._two_speakers(tmp_path)
        args = _make_args(db_path=str(db_path), src=src_id, dst=src_id, apply=True, yes=True)
        assert admin.cmd_merge(args) == 2

    def test_missing_src_is_a_clean_refusal(self, tmp_path):
        db_path, _src_id, dst_id = self._two_speakers(tmp_path)
        args = _make_args(db_path=str(db_path), src='does-not-exist', dst=dst_id, apply=True, yes=True)
        assert admin.cmd_merge(args) == 2

    def test_missing_dst_is_a_clean_refusal(self, tmp_path):
        db_path, src_id, _dst_id = self._two_speakers(tmp_path)
        args = _make_args(db_path=str(db_path), src=src_id, dst='does-not-exist', apply=True, yes=True)
        assert admin.cmd_merge(args) == 2

    def test_missing_db_is_a_clean_refusal(self, tmp_path):
        args = _make_args(db_path=str(tmp_path / 'nope.db'), src='a', dst='b', apply=True, yes=True)
        assert admin.cmd_merge(args) == 2


# ============================================================================
# 4. delete
# ============================================================================

class TestDelete:
    def _one_speaker(self, tmp_path):
        db_path = tmp_path / 'speakers.db'
        db = se.SpeakerDatabase(str(db_path))
        speaker_id = db.register('Тест', _basis(0))
        db.close()
        return db_path, speaker_id

    def test_dry_run_does_not_touch_disk(self, tmp_path):
        db_path, speaker_id = self._one_speaker(tmp_path)
        before = db_path.read_bytes()

        args = _make_args(db_path=str(db_path), speaker_id=speaker_id, apply=False)
        rc = admin.cmd_delete(args)

        assert rc == 0
        assert db_path.read_bytes() == before

    def test_apply_removes_speaker(self, tmp_path, monkeypatch):
        db_path, speaker_id = self._one_speaker(tmp_path)
        monkeypatch.setattr(admin, '_node_running_via_pgrep', _never_running)

        args = _make_args(db_path=str(db_path), speaker_id=speaker_id, apply=True, yes=True)
        rc = admin.cmd_delete(args)
        assert rc == 0

        db = se.SpeakerDatabase(str(db_path))
        try:
            speakers = {s['id'] for s in db.list_speakers()}
        finally:
            db.close()
        assert speaker_id not in speakers

    def test_backup_created_with_non_colliding_name(self, tmp_path, monkeypatch):
        db_path, speaker_id = self._one_speaker(tmp_path)
        monkeypatch.setattr(admin, '_node_running_via_pgrep', _never_running)

        args = _make_args(db_path=str(db_path), speaker_id=speaker_id, apply=True, yes=True)
        assert admin.cmd_delete(args) == 0

        backups = list(tmp_path.glob('speakers.db.*'))
        assert len(backups) == 1
        assert admin._BACKUP_TAG in backups[0].name
        assert '.bak-' not in backups[0].name

    def test_refuses_without_force_when_node_running(self, tmp_path):
        db_path, speaker_id = self._one_speaker(tmp_path)
        before = db_path.read_bytes()

        args = _make_args(db_path=str(db_path), speaker_id=speaker_id, apply=True, yes=True, force=False)
        original = admin._node_running_via_pgrep
        admin._node_running_via_pgrep = _always_running  # type: ignore[assignment]
        try:
            rc = admin.cmd_delete(args)
        finally:
            admin._node_running_via_pgrep = original  # type: ignore[assignment]

        assert rc == 3
        assert db_path.read_bytes() == before

    def test_missing_speaker_id_is_a_clean_refusal(self, tmp_path):
        db_path = tmp_path / 'speakers.db'
        db = se.SpeakerDatabase(str(db_path))
        db.close()
        args = _make_args(db_path=str(db_path), speaker_id='does-not-exist', apply=True, yes=True)
        assert admin.cmd_delete(args) == 2

    def test_missing_db_is_a_clean_refusal(self, tmp_path):
        args = _make_args(db_path=str(tmp_path / 'nope.db'), speaker_id='x', apply=True, yes=True)
        assert admin.cmd_delete(args) == 2


# ============================================================================
# 5. Гейт на живую ноду / подтверждение — тот же приём, что face_store_admin.py
# ============================================================================

class TestNodeGate:
    def test_running_blocks_without_force(self):
        err = admin._ensure_node_not_running(False, 'speaker_id_node', node_check=_always_running)
        assert err is not None and 'нода' in err

    def test_unknown_state_blocks_without_force(self):
        err = admin._ensure_node_not_running(False, 'speaker_id_node', node_check=_unknown_running)
        assert err is not None and 'не удалось проверить' in err

    def test_not_running_allows(self):
        assert admin._ensure_node_not_running(False, 'speaker_id_node', node_check=_never_running) is None

    def test_force_always_allows(self):
        assert admin._ensure_node_not_running(True, 'speaker_id_node', node_check=_always_running) is None


class TestConfirm:
    def test_yes_flag_skips_prompt(self):
        assert admin._confirm('irrelevant', assume_yes=True) is True

    def test_no_tty_without_yes_is_a_refusal(self, monkeypatch):
        def _raise_eof(_prompt):
            raise EOFError()

        monkeypatch.setattr('builtins.input', _raise_eof)
        assert admin._confirm('irrelevant', assume_yes=False) is False


# ============================================================================
# 6. Регрессия issue #2775 — доставка docker cp в /tmp не должна ронять
#    модуль IndexError'ом (тот же баг, что живой прогон агента поймал на
#    face_store_admin.py; здесь он пойман профилактически, до повторения).
# ============================================================================

class TestShallowLocationImport:
    def test_guess_pkg_root_returns_none_for_shallow_path_not_indexerror(self):
        shallow = Path('/tmp/speaker_db_admin.py')
        assert admin._guess_pkg_root(shallow, 'rob_box_voice') is None

    def test_guess_pkg_root_resolves_normal_checkout_layout(self):
        assert admin._guess_pkg_root(_SCRIPT_PATH, 'rob_box_voice') == _REPO_ROOT / 'src' / 'rob_box_voice'

    def test_shallow_copy_reaches_arg_parsing_when_packages_already_importable(self):
        """rob_box_voice.utils.speaker_embeddings уже в sys.modules (шапка этого
        файла) — "скопированный" в мелкий путь speaker_db_admin.py обязан
        дойти до build_parser()/parse_args(), используя обычный импорт
        первым (sys.path вообще не трогается в этой ветке)."""
        assert 'rob_box_voice.utils.speaker_embeddings' in sys.modules

        fake_file = str(Path(_SCRIPT_PATH.anchor) / 'speaker_db_admin.py')
        module_name = 'speaker_db_admin_shallow_probe'
        loader = importlib.machinery.SourceFileLoader(module_name, str(_SCRIPT_PATH))
        spec = importlib.util.spec_from_loader(module_name, loader, origin=fake_file)
        mod = importlib.util.module_from_spec(spec)
        mod.__file__ = fake_file
        sys.modules[module_name] = mod
        try:
            spec.loader.exec_module(mod)
            parsed = mod.build_parser().parse_args(['list'])
            assert parsed.command == 'list'
        finally:
            sys.modules.pop(module_name, None)

    def test_loader_returns_sys_modules_object_not_parent_attribute(self, monkeypatch):
        """Загрузчик обязан отдавать объект из ``sys.modules``, а не атрибут
        родительского пакета (CI-провал PR #2790).

        `import a.b.c as x` связывает `x` с АТРИБУТОМ родителя (`a.b.c`),
        а не с `sys.modules['a.b.c']`. Обычно это один объект, поэтому на
        dev-машине всё проходило. Но под colcon symlink-install тот же
        файл достижим двумя путями, и если запись в sys.modules успели
        подменить второй копией, атрибут родителя остаётся указывать на
        первую — в процессе живут два модуля с одинаковым ``__name__`` и
        раздельным состоянием уровня модуля. В логе CI это выглядело
        абсурдно: ``assert X is X`` падал при визуально одинаковых repr,
        потому что repr модуля копии не различает.

        Иерархия здесь СИНТЕТИЧЕСКАЯ целиком: три записи в sys.modules
        (`rob_box_voice`, `.utils`, `.utils.speaker_embeddings`), поэтому
        ни настоящий пакет, ни pyaudio, ни colcon не нужны и тест идёт в
        любом окружении, а не только в CI. `importlib.import_module`
        находит имя уже в sys.modules и до файловой системы не доходит.
        """
        import types

        name = 'rob_box_voice.utils.speaker_embeddings'
        pkg = types.ModuleType('rob_box_voice')
        pkg.__path__ = []
        utils = types.ModuleType('rob_box_voice.utils')
        utils.__path__ = []
        canonical = types.ModuleType(name)
        stale = types.ModuleType(name)      # «первая копия» из другого пути

        monkeypatch.setitem(sys.modules, 'rob_box_voice', pkg)
        monkeypatch.setitem(sys.modules, 'rob_box_voice.utils', utils)
        monkeypatch.setitem(sys.modules, name, canonical)
        pkg.utils = utils
        utils.speaker_embeddings = stale    # атрибут родителя разошёлся

        # Страховка от ложного прохождения: копии обязаны быть
        # неразличимы по repr — иначе тест не воспроизводит CI-ситуацию.
        assert repr(canonical) == repr(stale)

        result = admin._load_speaker_embeddings_module()

        assert result is canonical, (
            'загрузчик вернул атрибут родительского пакета вместо записи '
            'в sys.modules — значит вернулся `import a.b.c as x` вместо '
            'importlib.import_module (PR #2790)'
        )
        assert result is not stale

    def test_load_speaker_embeddings_module_skips_path_guessing_when_plain_import_succeeds(self, monkeypatch):
        called = {'n': 0}

        def _boom(here, pkg_dir_name):
            called['n'] += 1
            raise AssertionError('_guess_pkg_root не должен зваться, когда обычный импорт уже сработал')

        monkeypatch.setattr(admin, '_guess_pkg_root', _boom)
        result = admin._load_speaker_embeddings_module()
        assert result is sys.modules['rob_box_voice.utils.speaker_embeddings']
        assert called['n'] == 0
