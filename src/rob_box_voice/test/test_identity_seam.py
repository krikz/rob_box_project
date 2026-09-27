#!/usr/bin/env python3
"""test_identity_seam.py — acceptance тест шва идентичности (issue #2440).

Воспроизводит сценарий-заказчик программно, без resemblyzer/ROS:

  (a) регистрируем голосовой профиль и подтверждаем ``resolve()`` под
      Yandex-tag ``"0"`` — получаем стабильный ``Знакомый.id``;
  (b) вызываем ``note_seen``;
  (c) «рестарт»: новые инстансы БД на тех же файлах (эмуляция новой
      сессии), подаём ТОТ ЖЕ голосовой эмбеддинг под ДРУГИМ tag ``"1"``;
  (d) ``resolve()`` обязан вернуть тот же ``Знакомый.id``;
  (e) ``since_last_seen(Знакомый)`` — реальный положительный интервал
      (не None, не 0, не пересозданный first_seen).

На коде до issue #2440 тест красный: профиль лежал бы под
``speaker_scope("0")`` и ``speaker_scope("1")`` раздельно (ключ — Yandex
tag), и ``since_last_seen`` после «рестарта» вернул бы None. Шов ключует
всё стабильным биометрическим id, поэтому tag в операции шва не участвует.

Модули ``utils.*`` грузятся напрямую по пути файла (utils/__init__.py
тянет pyaudio) — тот же приём, что в test_speaker_embeddings.py.
"""

from __future__ import annotations

import asyncio
import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
import pytest

from rob_box_harness.memory.sqlite_voice import SQLiteVoiceMemory

_PKG_ROOT = Path(__file__).resolve().parents[1] / "rob_box_voice"

# Фейковые пакеты, чтобы относительный импорт ``from .speaker_embeddings``
# внутри identity_seam.py резолвился без выполнения utils/__init__.py (pyaudio).
_rbv = types.ModuleType("rob_box_voice")
_rbv.__path__ = [str(_PKG_ROOT)]
sys.modules.setdefault("rob_box_voice", _rbv)
_utils = types.ModuleType("rob_box_voice.utils")
_utils.__path__ = [str(_PKG_ROOT / "utils")]
sys.modules.setdefault("rob_box_voice.utils", _utils)


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


_se = _load_module(
    "rob_box_voice.utils.speaker_embeddings",
    _PKG_ROOT / "utils" / "speaker_embeddings.py",
)
_ism = _load_module(
    "rob_box_voice.utils.identity_seam", _PKG_ROOT / "utils" / "identity_seam.py"
)

SpeakerDatabase = _se.SpeakerDatabase
VoiceIdentitySeam = _ism.VoiceIdentitySeam


def _random_embedding(seed: int, dim: int = 256) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(dim).astype(np.float32)
    return v / np.linalg.norm(v)


def _run(coro):
    return asyncio.run(coro)


def test_acceptance_stable_identity_across_restart(tmp_path):
    """Сценарий-заказчик issue #2440: один голос — один id между сессиями."""
    speakers_db = str(tmp_path / "speakers.db")
    memory_db = str(tmp_path / "memory.db")
    embedding = _random_embedding(42)

    # ── Сессия 1 (Yandex tag «0») ────────────────────────────────────────
    db1 = SpeakerDatabase(speakers_db)
    mem1 = SQLiteVoiceMemory(db_path=memory_db)
    _run(mem1.init())
    seam1 = VoiceIdentitySeam(db1, mem1)

    registered = seam1.register(embedding, "Иван")  # (a) регистрация профиля
    resolved1 = seam1.resolve(embedding)  # resolve() под tag "0"
    assert resolved1 is not None
    assert resolved1.id == registered.id
    _run(seam1.note_seen(resolved1, now=1000.0))  # (b) note_seen

    db1.close()
    _run(mem1.teardown())

    # ── «Рестарт»: новые инстансы на тех же файлах, Yandex tag «1» ───────
    db2 = SpeakerDatabase(speakers_db)
    mem2 = SQLiteVoiceMemory(db_path=memory_db)
    _run(mem2.init())
    seam2 = VoiceIdentitySeam(db2, mem2)

    resolved2 = seam2.resolve(embedding)  # (c/d) ТОТ ЖЕ голос, tag «1»
    assert resolved2 is not None
    assert resolved2.id == registered.id, (
        "resolve() после рестарта вернул другой id — шов ключует по "
        "нестабильному tag вместо биометрического UUID"
    )

    gap = _run(seam2.since_last_seen(resolved2, now=1360.0))  # (e)
    assert gap is not None, "since_last_seen вернул None после рестарта"
    assert gap == pytest.approx(360.0)
    assert gap > 0

    db2.close()
    _run(mem2.teardown())


def test_voice_seam_merge_moves_embeddings_and_facts(tmp_path):
    """Дефект C из issue #2440: merge переносит и эмбеддинги, и факты."""
    speakers_db = str(tmp_path / "speakers.db")
    memory_db = str(tmp_path / "memory.db")

    db = SpeakerDatabase(speakers_db)
    mem = SQLiteVoiceMemory(db_path=memory_db)
    _run(mem.init())
    seam = VoiceIdentitySeam(db, mem)

    src = seam.register(_random_embedding(1), "Денчик")
    dst = seam.register(_random_embedding(2), "Эйджик")
    # Факт под старым id — как накопленный профиль до склейки.
    _run(seam.note_seen(src, now=100.0))

    emb_moved, facts_moved = _run(seam.merge(src.id, dst.id))
    assert emb_moved == 1  # эмбеддинг перенесён
    assert facts_moved == 1  # profile-факт перенесён
    assert seam.resolve(_random_embedding(1)) is not None  # голос src теперь под dst

    db.close()
    _run(mem.teardown())


def test_voice_seam_merge_also_moves_legacy_voice_facts(tmp_path):
    """Issue #2751: merge() без ``legacy_facts_db_path`` теряет живые факты.

    ``harness_voice.db`` (переданный конструктору ``mem``) — не единственный
    писатель фактов на проде: MCP-инструмент ``memory_save`` пишет в
    ``voice_facts`` отдельного файла (``voice_memory.db``), про который шов
    ничего не знает, если ему не передать ``legacy_facts_db_path``. Тест
    воспроизводит ровно замер issue #2751 в миниатюре: 1 факт в
    harness-БД (переживший из старого сценария) + 2 живых факта в
    voice_facts — оба писателя должны быть перенесены одним ``merge()``.
    """
    import sqlite3

    speakers_db = str(tmp_path / "speakers.db")
    memory_db = str(tmp_path / "memory.db")  # harness_voice.db
    voice_memory_db = str(tmp_path / "voice_memory.db")  # легаси MCP-писатель

    db = SpeakerDatabase(speakers_db)
    mem = SQLiteVoiceMemory(db_path=memory_db)
    _run(mem.init())

    # Легаси-БД: та же схема voice_facts, что core/voice_memory.py создаёт
    # в проде (см. test_legacy_voice_facts.py — единственное поле нам
    # важное здесь: speaker_id).
    conn = sqlite3.connect(voice_memory_db)
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

    seam = VoiceIdentitySeam(db, mem, legacy_facts_db_path=voice_memory_db)

    src = seam.register(_random_embedding(11), "Саша")
    dst = seam.register(_random_embedding(12), "Борис")
    _run(seam.note_seen(src, now=100.0))  # 1 факт в harness_voice.db

    conn = sqlite3.connect(voice_memory_db)
    conn.execute(
        "INSERT INTO voice_facts (fact, category, speaker_id, created_at, updated_at) "
        "VALUES ('не ест лук', 'general', ?, 0, 0)",
        (src.id,),
    )
    conn.execute(
        "INSERT INTO voice_facts (fact, category, speaker_id, created_at, updated_at) "
        "VALUES ('пьёт чай без сахара', 'general', ?, 0, 0)",
        (src.id,),
    )
    conn.commit()
    conn.close()

    emb_moved, facts_moved = _run(seam.merge(src.id, dst.id))

    assert emb_moved == 1
    assert facts_moved == 3, (
        "ожидали 1 (harness profile-факт) + 2 (voice_facts) = 3 — "
        "живые факты не должны теряться при склейке (issue #2751)"
    )

    conn = sqlite3.connect(voice_memory_db)
    rows = conn.execute("SELECT speaker_id FROM voice_facts").fetchall()
    conn.close()
    assert {r[0] for r in rows} == {dst.id}, (
        "voice_facts должны принадлежать dst после merge — src оставил хвост"
    )

    db.close()
    _run(mem.teardown())


def test_voice_seam_merge_without_legacy_path_stays_backward_compatible(tmp_path):
    """``legacy_facts_db_path=None`` (по умолчанию) — старое поведение,
    без него не ходит: тест-акцептанс issue #2440 не должен ломаться."""
    speakers_db = str(tmp_path / "speakers.db")
    memory_db = str(tmp_path / "memory.db")

    db = SpeakerDatabase(speakers_db)
    mem = SQLiteVoiceMemory(db_path=memory_db)
    _run(mem.init())
    seam = VoiceIdentitySeam(db, mem)  # без legacy_facts_db_path

    src = seam.register(_random_embedding(21), "Денчик")
    dst = seam.register(_random_embedding(22), "Эйджик")
    _run(seam.note_seen(src, now=100.0))

    emb_moved, facts_moved = _run(seam.merge(src.id, dst.id))
    assert emb_moved == 1
    assert facts_moved == 1

    db.close()
    _run(mem.teardown())
