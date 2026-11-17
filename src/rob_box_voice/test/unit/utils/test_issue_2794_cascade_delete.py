"""issue #2794 — удаление профиля обязано уносить его эмбеддинги.

Схема `embeddings` объявляет `REFERENCES speakers(speaker_id) ON DELETE
CASCADE`, но SQLite держит внешние ключи выключенными по умолчанию и
включает их отдельно на каждое соединение. Пока `PRAGMA foreign_keys`
не выставлен, объявленный каскад — мёртвая декларация.

Почему это поймали поздно: на узнавание сироты не влияют — `_score_all`
берёт векторы через `JOIN speakers`, который их отбрасывает. Зато
попарная диагностика `speaker_db_admin.py list` читает `embeddings`
напрямую и показывала удалённые профили как «возможные дубли одного
человека» — ровно в тот момент, когда по этой таблице решают, кого с кем
сливать. Замер на боевой БД 22.09.2026: 9 осиротевших векторов от трёх
удалённых профилей сценария.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

# Тот же приём, что в соседнем test_gallery_warmup.py: грузим модуль по
# пути, минуя пакет `rob_box_voice`, чей `__init__` тянет pyaudio —
# на dev-машине его нет, и обычный импорт уронил бы весь файл на сборе.
_PKG_ROOT = Path(__file__).resolve().parents[3] / "rob_box_voice"
_SPEAKER_EMB = _PKG_ROOT / "utils" / "speaker_embeddings.py"

_spec = importlib.util.spec_from_file_location(
    "rob_box_voice.utils.speaker_embeddings", _SPEAKER_EMB
)
_se = importlib.util.module_from_spec(_spec)
sys.modules["rob_box_voice.utils.speaker_embeddings"] = _se
_spec.loader.exec_module(_se)

SpeakerDatabase = _se.SpeakerDatabase

DIM = 256


def _vec(seed: int) -> np.ndarray:
    rng = np.random.RandomState(seed)
    v = rng.normal(size=DIM).astype(np.float32)
    return v / np.linalg.norm(v)


@pytest.fixture()
def db(tmp_path) -> SpeakerDatabase:
    return SpeakerDatabase(db_path=str(tmp_path / "speakers.db"))


def test_foreign_keys_pragma_is_on(db: SpeakerDatabase) -> None:
    """Прямая проверка причины, а не только следствия: если однажды кто-то
    уберёт PRAGMA, этот тест назовёт виновника сразу, не заставляя
    разбираться, почему «каскад вдруг не работает»."""
    assert db._conn.execute("PRAGMA foreign_keys").fetchone()[0] == 1


def test_delete_speaker_removes_embeddings(db: SpeakerDatabase) -> None:
    sid = db.register("Дэнчик", _vec(1))
    db.register("Дэнчик", _vec(2), speaker_id=sid)
    assert db.gallery_size(sid) == 2

    assert db.delete_speaker(sid) is True

    left = db._conn.execute(
        "SELECT COUNT(*) FROM embeddings WHERE speaker_id=?", (sid,)
    ).fetchone()[0]
    assert left == 0, "эмбеддинги удалённого профиля обязаны уйти каскадом"


def test_delete_does_not_touch_other_speakers(db: SpeakerDatabase) -> None:
    """Каскад узкий: чужие галереи переживают удаление соседа."""
    victim = db.register("Саша", _vec(3))
    keeper = db.register("Борис", _vec(4))
    db.register("Борис", _vec(5), speaker_id=keeper)

    db.delete_speaker(victim)

    assert db.gallery_size(keeper) == 2
    assert db.orphaned_embedding_count() == 0


def test_orphaned_embedding_count_sees_manual_damage(db: SpeakerDatabase) -> None:
    """Счётчик сирот честно показывает расхождение, если строки в
    `speakers` удалили мимо API (так и выглядела боевая БД до фикса —
    её чистили ручными sqlite-командами, см. #2750)."""
    sid = db.register("Дэнчик", _vec(6))
    db.register("Дэнчик", _vec(7), speaker_id=sid)
    assert db.orphaned_embedding_count() == 0

    # Мимо delete_speaker и мимо каскада — как это делали руками.
    db._conn.execute("PRAGMA foreign_keys = OFF")
    db._conn.execute("DELETE FROM speakers WHERE speaker_id=?", (sid,))
    db._conn.commit()
    db._conn.execute("PRAGMA foreign_keys = ON")

    assert db.orphaned_embedding_count() == 2
