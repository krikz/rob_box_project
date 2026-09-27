"""test_legacy_voice_facts.py — перенос строк voice_facts при склейке (issue #2751).

``merge_legacy_voice_facts`` — прямой sqlite-доступ к ``voice_facts``
(``voice_memory.db``, писатель — MCP-инструмент ``memory_save``), в
отличие от ``merge_speaker_facts`` (модель scope/key харнесса,
``harness_voice.db``). Модуль грузится напрямую по пути файла — тот же
приём, что в ``test_speaker_embeddings.py`` и ``test_identity_seam.py``
(``utils/__init__.py`` тянет pyaudio).
"""

from __future__ import annotations

import importlib.util
import sqlite3
import sys
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[3] / "rob_box_voice"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


_lvf = _load_module(
    "rob_box_voice.utils.legacy_voice_facts",
    _PKG_ROOT / "utils" / "legacy_voice_facts.py",
)
merge_legacy_voice_facts = _lvf.merge_legacy_voice_facts


def _make_voice_memory_db(path: str) -> None:
    """Воспроизводит схему ``core/voice_memory.py`` (fallback DDL)."""
    conn = sqlite3.connect(path)
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


def _insert_fact(path: str, fact: str, speaker_id: str) -> None:
    conn = sqlite3.connect(path)
    conn.execute(
        "INSERT INTO voice_facts (fact, category, speaker_id, created_at, updated_at) "
        "VALUES (?, 'general', ?, 0, 0)",
        (fact, speaker_id),
    )
    conn.commit()
    conn.close()


def test_moves_all_facts_for_source_speaker(tmp_path):
    db_path = str(tmp_path / "voice_memory.db")
    _make_voice_memory_db(db_path)
    _insert_fact(db_path, "не ест лук", "src-id")
    _insert_fact(db_path, "пьёт зелёный чай без сахара", "src-id")
    _insert_fact(db_path, "болеет за Спартак", "dst-id")  # уже у получателя

    moved = merge_legacy_voice_facts(db_path, "src-id", "dst-id")

    assert moved == 2
    conn = sqlite3.connect(db_path)
    rows = conn.execute(
        "SELECT fact, speaker_id FROM voice_facts ORDER BY id"
    ).fetchall()
    conn.close()
    assert {r[1] for r in rows} == {"dst-id"}
    facts = {r[0] for r in rows}
    assert facts == {"не ест лук", "пьёт зелёный чай без сахара", "болеет за Спартак"}


def test_no_facts_for_source_returns_zero(tmp_path):
    db_path = str(tmp_path / "voice_memory.db")
    _make_voice_memory_db(db_path)
    _insert_fact(db_path, "факт другого спикера", "other-id")

    moved = merge_legacy_voice_facts(db_path, "src-id", "dst-id")

    assert moved == 0


def test_missing_table_returns_zero_not_raises(tmp_path):
    """Легаси-писатель мог ни разу не создать voice_facts — не ошибка."""
    db_path = str(tmp_path / "empty.db")
    sqlite3.connect(db_path).close()

    moved = merge_legacy_voice_facts(db_path, "src-id", "dst-id")

    assert moved == 0


def test_missing_file_returns_zero_not_raises(tmp_path):
    db_path = str(tmp_path / "does_not_exist" / "voice_memory.db")

    moved = merge_legacy_voice_facts(db_path, "src-id", "dst-id")

    assert moved == 0


def test_same_speaker_id_is_noop(tmp_path):
    db_path = str(tmp_path / "voice_memory.db")
    _make_voice_memory_db(db_path)
    _insert_fact(db_path, "факт", "same-id")

    moved = merge_legacy_voice_facts(db_path, "same-id", "same-id")

    assert moved == 0


def test_empty_ids_are_noop(tmp_path):
    db_path = str(tmp_path / "voice_memory.db")
    _make_voice_memory_db(db_path)

    assert merge_legacy_voice_facts(db_path, "", "dst-id") == 0
    assert merge_legacy_voice_facts(db_path, "src-id", "") == 0
