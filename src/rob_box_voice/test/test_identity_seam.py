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
