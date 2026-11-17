#!/usr/bin/env python3
"""face_merge.py - «найди похожих» и склейка дублей лиц (issue #4469, ADR-0148: код решает, LLM говорит).

Владелец данных - ``FaceStore`` на Vision Pi; здесь считает код, а не модель:

* :func:`similar_people` - кто из записей похож на данную: косинус медоидов галерей, порог и ``k`` - константы этого
  модуля; записи с тем же именем идут первыми и порогом не отсекаются (два «Денчика» - первые кандидаты на склейку);
* :func:`merge_people` - склейка записей в одну: бэкап затронутых записей ДО операции, затем ``FaceStore.merge``
  (эмбеддинги каждой записи входят в галерею отдельными ракурсами, не усредняются; встречи, снимки и связь с
  голосом переносятся), имя берёт ``target`` (нет - первое непустое из склеенных). Исход читается из хранилища после
  записи: «склеено N, теперь у <имя> M снимков» строит код по этому факту.

Функции модуля, а не методы ``FaceStore``: читают записи под замком хранилища, но не раздувают класс
(ADR-0145, class_budget) - так же, как ``face_store.best_snapshot_bytes``. Ракурсы приходят из хранилища уже с потолком
``max_embeddings`` (самый далёкий от медоида вытесняется): подстройка потолка и порога по журналу промахов - отдельная
задача (#4469, следующий шаг).
"""

from __future__ import annotations

import json
import shutil
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

from .face_misses import MissJournal, snapshot_misses
from .face_store import FaceStore, _cosine, _medoid

#: Потолок кандидатов коллажа: сетка 4x3 влезает в JPEG до 200 КБ и читается глазом.
SIMILAR_K_MAX = 12
#: Порог сходства медоидов для кандидата (косинус). Ниже ``identify_threshold`` хранилища (0.6) намеренно: дубль
#: существует именно потому, что робот не узнал человека; решает человек по коллажу, код лишь отсекает чужих.
#: Стартовое значение: подстройка по журналу склеек - отдельная задача #4469.
SIMILAR_THRESHOLD = 0.40
#: Сколько склеек за раз: коллаж показывает до ``SIMILAR_K_MAX``, больше выбрать нельзя.
MERGE_MAX = SIMILAR_K_MAX


@dataclass(frozen=True)
class Candidate:
    """Кандидат на склейку: полный id, имя, косинус медоидов, то же ли имя, что у искомой записи."""

    person_id: str
    name: Optional[str]
    similarity: float
    same_name: bool


@dataclass(frozen=True)
class MergeOutcome:
    """Исход склейки, прочитанный из хранилища после записи (факт, а не пересказ)."""

    merged: int
    name: Optional[str]
    gallery: int
    backup_dir: str
    merged_ids: Sequence[str]


def _same_name(a: Optional[str], b: Optional[str]) -> bool:
    return bool(a) and bool(b) and str(a).strip().casefold() == str(b).strip().casefold()


def similar_people(
    store: FaceStore,
    person_id: str,
    k: int = SIMILAR_K_MAX,
    threshold: float = SIMILAR_THRESHOLD,
) -> List[Candidate]:
    """До ``k`` (не больше :data:`SIMILAR_K_MAX`) записей, похожих на ``person_id``.

    Сходство - косинус медоидов галерей. Порядок: тёзки (то же имя) первыми, внутри группы - по убыванию сходства.
    Тёзка попадает в список при любом сходстве; остальные - только не ниже ``threshold``. Записи без эмбеддингов
    пропускаются (сравнивать нечем). Неизвестный ``person_id`` - пустой список.
    """
    limit = max(0, min(int(k), SIMILAR_K_MAX))
    with store._lock:
        base = store._records.get(person_id)
        if base is None or not base.embeddings:
            return []
        anchor = _medoid(base.embeddings)
        found = [
            Candidate(
                rec.person_id, rec.name, _cosine(anchor, _medoid(rec.embeddings)), _same_name(base.name, rec.name)
            )
            for rec in store._records.values()
            if rec.person_id != person_id and rec.embeddings
        ]
    found = [c for c in found if c.same_name or c.similarity >= threshold]
    found.sort(key=lambda c: (not c.same_name, -c.similarity, c.person_id))
    return found[:limit]


def _utc_stamp(now: float) -> str:
    return datetime.fromtimestamp(now, tz=timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def backup_records(store: FaceStore, person_ids: Sequence[str], backup_root: Path, now: float) -> Path:
    """Копия записей ``person_ids`` в ``backup_root/<метка UTC>-<id>/`` ДО склейки.

    Каталог записи на диске копируется целиком (meta, эмбеддинги, снимки); записи, которой на диске нет (незнакомец
    режима ``exhibition`` живёт в памяти), - ``embeddings.npy`` из памяти. Рядом ``involved.json`` со строками
    ``FaceStore.people()`` этих записей: по нему видно, что и чем было. Бэкап не удалился бы сам: чистит человек.
    """
    dest = Path(backup_root) / f"{_utc_stamp(now)}-{person_ids[0][:8]}"
    dest.mkdir(parents=True, exist_ok=True)
    wanted = set(person_ids)
    rows: List[Dict[str, Any]] = [p for p in store.people() if p["person_id"] in wanted]
    for pid in person_ids:
        source = store._person_dir(pid)
        if source.is_dir():
            shutil.copytree(source, dest / pid, dirs_exist_ok=True)
        else:
            (dest / pid).mkdir(parents=True, exist_ok=True)
            gallery = store.gallery(pid)
            if gallery:
                np.save(dest / pid / "embeddings.npy", np.stack(gallery).astype(np.float32))
    (dest / "involved.json").write_text(json.dumps(rows, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    return dest


def _attach_inherited_name(store: FaceStore, target: str, names: Sequence[Optional[str]]) -> None:
    """У ``target`` нет имени - берёт первое непустое имя из склеенных (``FaceStore.merge`` имя не переносит)."""
    inherited = next((n for n in names if n), None)
    if inherited and not next((p["name"] for p in store.people() if p["person_id"] == target), None):
        store.attach_name(target, inherited)


def _mergeable(known: Dict[str, Any], target: str, ids: Sequence[str]) -> bool:
    """``target`` и все ``ids`` существуют, ``ids`` не пуст, без дублей и без самого ``target``."""
    if target not in known or not ids or len(set(ids)) != len(ids) or target in ids:
        return False
    return all(i in known for i in ids)


def merge_people(
    store: FaceStore,
    target: str,
    sources: Sequence[str],
    backup_root: Path,
    clock: Callable[[], float] = time.time,
    misses: Optional[MissJournal] = None,
) -> Optional[MergeOutcome]:
    """Склеить записи ``sources`` в ``target``: бэкап, затем слияние по одной. Возвращает исход по данным хранилища
    или ``None``, если нечего склеивать (записи ``target`` нет, ``sources`` пуст, дубль или сам ``target`` в нём).

    Вызывающий (``people_service``) уже проверил id по существующим записям; здесь повторная проверка под замком
    не нужна: ``FaceStore.merge`` сам отказывает по неизвестному id, а сколько склеено - считается по его ответам.
    """
    ids = list(sources)
    known = {p["person_id"]: p["name"] for p in store.people()}
    if not _mergeable(known, target, ids):
        return None
    backup = backup_records(store, [target, *ids], backup_root, clock())
    names = [known[i] for i in ids]
    # журнал промахов (#4469 п. 2): строки считаются ДО слияния, пишутся только по реально слитым записям
    rows = snapshot_misses(store, target, ids, clock()) if misses is not None else []
    done = [i for i in ids if store.merge(target, i)]
    if misses is not None:
        misses.append([r for r in rows if r["merged_id"] in done])
    _attach_inherited_name(store, target, names)
    after = next((p for p in store.people() if p["person_id"] == target), None)
    return MergeOutcome(
        len(done), after["name"] if after else None, len(store.gallery(target)), str(backup), tuple(done)
    )


__all__ = [
    "Candidate",
    "MERGE_MAX",
    "MergeOutcome",
    "SIMILAR_K_MAX",
    "SIMILAR_THRESHOLD",
    "backup_records",
    "merge_people",
    "similar_people",
]
