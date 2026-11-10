#!/usr/bin/env python3
"""face_misses.py - журнал промахов узнавания лиц (issue #4469, п. 2; ADR-0148: код решает).

Каждое слияние ``merge_people(target, ids)`` - подтверждённый человеком промах: «это был X, а робот завёл новую
запись». Владелец ``FaceStore`` (Vision) дописывает в ``face_misses.jsonl`` по одной строке на каждую слитую запись:

* ``ts`` - время UTC (ISO 8601);
* ``target_id`` / ``merged_id`` - полные id;
* ``embedding`` - медоид слитой записи (то, чем она была для узнавания);
* ``best_id`` / ``best_similarity`` - кого выбрало бы узнавание для этого эмбеддинга ДО слияния (без самой слитой
  записи: она-то и есть промах); считается тем же кодом, что и узнавание: ``FaceStore._score_all`` +
  ``FaceStore._disambiguate``, порог не применяется - важно, кто был ближайшим и насколько;
* ``target_similarity`` - сходство с ``target`` по тому же счёту узнавания (лучший вектор галереи);
* ``threshold`` - текущий порог узнавания (``identify_threshold``) на момент слияния.

Журнал append-only: одна строка = один ``os.write`` в файл ``O_APPEND``, затем ``fsync``. Ошибка записи слияния не
роняет: WARN в лог и счётчик ``failures``. Файл лежит рядом с базой лиц (путь из конфига), только на роботе:
в репозиторий и артефакты CI он не попадает (``.gitignore``). Читателя у журнала пока нет: его заведёт подстройка
порога (#4469, п. 3).
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Sequence

from .face_store import FaceStore, _medoid

_LOG = logging.getLogger(__name__)

#: Имя файла журнала по умолчанию (рядом с каталогом лиц).
JOURNAL_NAME = "face_misses.jsonl"
_EMBEDDING_DIGITS = 6


def default_journal_path(faces_root: Any) -> Path:
    """``<родитель каталога лиц>/face_misses.jsonl``: тот же ``/data``, но вне корня лиц (он читается как записи)."""
    return Path(faces_root).parent / JOURNAL_NAME


def snapshot_misses(store: FaceStore, target: str, merged_ids: Sequence[str], now: float) -> List[Dict[str, Any]]:
    """Строки журнала по состоянию ДО слияния: считать нужно, пока записи и галерея ``target`` не изменились."""
    rows: List[Dict[str, Any]] = []
    stamp = datetime.fromtimestamp(now, tz=timezone.utc).isoformat()
    with store._lock:
        for merged_id in merged_ids:
            rec = store._records.get(merged_id)
            if rec is None or not rec.embeddings:
                continue
            emb = store._prepare_embedding(_medoid(rec.embeddings))
            scored = [s for s in store._score_all(emb) if s[0] != merged_id]
            best_id, best_sim = None, None
            if scored:
                idx, _ = store._disambiguate(scored)
                best_id, best_sim = scored[idx]
            target_sim = next((sim for pid, sim in scored if pid == target), None)
            rows.append(
                {
                    "ts": stamp,
                    "target_id": target,
                    "merged_id": merged_id,
                    "embedding": [round(float(x), _EMBEDDING_DIGITS) for x in emb],
                    "best_id": best_id,
                    "best_similarity": None if best_sim is None else round(float(best_sim), 4),
                    "target_similarity": None if target_sim is None else round(float(target_sim), 4),
                    "threshold": float(store._identify_threshold),
                }
            )
    return rows


class MissJournal:
    """Append-only журнал промахов; сбой записи не бросает исключение, а считается в ``failures``."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.failures = 0
        self.written = 0

    def append(self, rows: Sequence[Dict[str, Any]]) -> int:
        """Дописать строки одним ``write``. Возвращает число записанных; сбой - 0, WARN и ``failures += 1``."""
        if not rows:
            return 0
        try:
            data = "".join(json.dumps(r, ensure_ascii=False, separators=(",", ":")) + "\n" for r in rows).encode()
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(str(self.path), os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
            try:
                os.write(fd, data)
                os.fsync(fd)
            finally:
                os.close(fd)
        except Exception as exc:  # noqa: BLE001 - журнал вторичен, слияние важнее
            self.failures += 1
            _LOG.warning(
                "face_misses: не записал журнал %s (%s: %s); сбоев %d",
                self.path,
                type(exc).__name__,
                exc,
                self.failures,
            )
            return 0
        self.written += len(rows)
        return len(rows)


def read_rows(path: Path) -> List[Dict[str, Any]]:
    """Строки журнала (для тестов и будущей подстройки порога); нет файла - пусто."""
    p = Path(path)
    if not p.is_file():
        return []
    return [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]


__all__ = ["JOURNAL_NAME", "MissJournal", "default_journal_path", "read_rows", "snapshot_misses"]
