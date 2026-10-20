#!/usr/bin/env python3
"""face_roster.py — read-only чтение лицевой базы с диска (``/data/faces``).

Общий код для всех потребителей «кто есть в лицевой базе»: Telegram
``/faces`` / ``/face`` (#3025, ``rob_box_telegram.face_card``) и инструмент
ТАРС ``list_faces`` (#3299, ``rob_box_supervisor``). Раньше жил внутри
``face_card`` пакета Telegram — но образ supervisor его не содержит, поэтому
читалка переехала сюда, а ``face_card`` её реэкспортирует.

Только чтение: ни одна функция не пишет в каталог лиц (том монтируется
``:ro``). Без ROS и без обязательного numpy/PIL (numpy грузится лениво).
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional

if TYPE_CHECKING:
    import numpy as np

logger = logging.getLogger(__name__)

META_FILENAME = "meta.json"
EMBEDDINGS_FILENAME = "embeddings.npy"
REFERENCE_FILENAME = "reference.jpg"
ENCOUNTERS_DIRNAME = "encounters"


@dataclass(frozen=True)
class FaceSummary:
    """Compact row for the ``/faces`` table.

    Mirrors the columns the task spec calls out: имя (или «незнакомец»),
    person_id (короткий), speaker_id, число встреч, размер галереи
    (count of embeddings, NOT byte size), gallery_cohesion.

    ``gallery_cohesion`` is the median pairwise cosine from ``FaceStore.
    people()`` when available; ``None`` for single-vector galleries (a
    pairwise cosine is not defined for one vector — see
    ``face_store_admin._gallery_cohesion``). On disk ``FaceStore`` does
    not persist this; it's recomputed by ``_cohesion_from_embeddings``
    when ``embeddings.npy`` is present.
    """

    person_id: str
    person_id_short: str
    name: Optional[str]
    speaker_id: Optional[str]
    encounter_count: int
    embeddings_count: int
    gallery_cohesion_median: Optional[float]
    gallery_bytes: int
    is_stranger: bool
    has_reference: bool
    # #3299: время последней встречи (по лицу) и создания записи, epoch.
    last_encounter_ts: Optional[float] = None
    created_at: Optional[float] = None


def _safe_read_json(path: Path) -> Dict[str, Any]:
    """Read meta.json, return ``{}`` if the file is missing or corrupt.

    Same forgiveness policy as ``face_store_admin._read_meta``:
    a single broken record must not bring down ``/faces`` for everyone
    else. ``/face <id>`` then returns "not found" on the empty dict
    (the handler decides what to do).
    """
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
            if isinstance(data, dict):
                return data
            return {}
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError):
        logger.warning("face_card: failed to parse %s", path, exc_info=True)
        return {}


def _read_meta(person_dir: Path) -> Dict[str, Any]:
    return _safe_read_json(person_dir / META_FILENAME)


def _gallery_byte_size(person_dir: Path) -> int:
    """Bytes of ``embeddings.npy`` (0 if missing). Used by the ``/faces``
    table so the operator can see how heavy each gallery is on disk."""
    p = person_dir / EMBEDDINGS_FILENAME
    try:
        return p.stat().st_size
    except OSError:
        return 0


def _load_embeddings(person_dir: Path) -> Optional[List["np.ndarray"]]:
    """Load ``embeddings.npy`` into a list-of-vectors, or ``None``.

    Returns ``None`` if the file is missing or unloadable. We import
    numpy lazily inside the function so the rest of this module is
    usable in environments without numpy (the Telegram image runs with
    numpy present, but the unit tests that exercise the PIL collage
    builder do not need it).
    """
    p = person_dir / EMBEDDINGS_FILENAME
    if not p.exists():
        return None
    try:
        import numpy as np  # local import — see docstring

        arr = np.load(p)
        if arr.size == 0:
            return []
        return [arr[i].astype("float32").copy() for i in range(arr.shape[0])]
    except Exception:
        logger.warning("face_card: failed to load %s", p, exc_info=True)
        return None


def _cohesion_median(embeddings: Optional[List["np.ndarray"]]) -> Optional[float]:
    """Median pairwise cosine of an embeddings list, or ``None`` if
    fewer than two vectors (a pairwise cosine is not defined for one
    vector — see ``face_store._gallery_cohesion``)."""
    if not embeddings or len(embeddings) < 2:
        return None
    try:
        import numpy as np

        stack = np.stack(embeddings).astype("float32")
        # L2-normalize defensively (real ArcFace vectors are already
        # normalized, but a corrupt npy might not be).
        norms = np.linalg.norm(stack, axis=1, keepdims=True)
        norms = np.where(norms < 1e-9, 1.0, norms)
        normed = stack / norms
        sims = normed @ normed.T
        n = sims.shape[0]
        iu = np.triu_indices(n, k=1)
        if iu[0].size == 0:
            return None
        return float(np.median(sims[iu]))
    except Exception:
        logger.warning("face_card: cohesion computation failed", exc_info=True)
        return None


def _short_person_id(person_id: str) -> str:
    """8-char short id, matching what ``/face 4ff0ddc5`` already accepts
    from operators. We don't truncate silently — full id is also
    available in the detail view."""
    return person_id[:8]


def _has_reference(person_dir: Path) -> bool:
    return (person_dir / REFERENCE_FILENAME).is_file()


def _num_or_none(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def list_people(
    root: str,
    *,
    root_exists: Optional[Callable[[str], bool]] = None,
    load_gallery: bool = True,
) -> List[FaceSummary]:
    """Return one ``FaceSummary`` per record directory under ``root``.

    Sorted with named people first, strangers after (stable by person_id
    ascending within each group) — matches the spec in issue #3025
    ("Именованные — первыми").

    ``load_gallery=False`` не открывает ``embeddings.npy`` (число
    эмбеддингов и cohesion остаются 0/None): нужно тем, кому важны только
    имя и счётчики, — ТАРС ``list_faces`` не тащит numpy ради списка.

    ``root_exists(root)`` is injected for tests so we can exercise both
    "directory missing" (empty list, not crash) and "directory present"
    without touching the real filesystem.
    """
    root_path = Path(root)
    if root_exists is not None and not root_exists(root):
        return []
    if not root_path.is_dir():
        return []

    out: List[FaceSummary] = []
    for entry in sorted(root_path.iterdir()):
        if not entry.is_dir():
            continue
        # Person_id is the directory name. The hex string lives in meta.json
        # too, but FaceStore always uses the dir name as the canonical id.
        person_id = entry.name
        meta = _read_meta(entry)
        if not meta:
            # Broken record — still surface it with whatever defaults,
            # so the operator knows it's there. Better noisy than invisible.
            name = None
            speaker_id = None
            encounter_count = 0
            created_at = None
            is_stranger = True
        else:
            name = meta.get("name") if meta.get("name") else None
            speaker_id = meta.get("speaker_id")
            try:
                encounter_count = int(meta.get("encounter_count") or 0)
            except (TypeError, ValueError):
                encounter_count = 0
            created_at = meta.get("created_at")
            is_stranger = name is None

        embeddings = _load_embeddings(entry) if load_gallery else None
        emb_count = len(embeddings) if embeddings is not None else 0
        cohesion = _cohesion_median(embeddings) if emb_count >= 2 else None
        bytes_ = _gallery_byte_size(entry)

        out.append(
            FaceSummary(
                person_id=person_id,
                person_id_short=_short_person_id(person_id),
                name=name,
                speaker_id=(str(speaker_id) if speaker_id else None),
                encounter_count=encounter_count,
                embeddings_count=emb_count,
                gallery_cohesion_median=cohesion,
                gallery_bytes=bytes_,
                is_stranger=is_stranger,
                has_reference=_has_reference(entry),
                last_encounter_ts=_num_or_none(meta.get("last_encounter_ts")),
                created_at=_num_or_none(created_at),
            )
        )

    out.sort(key=lambda s: (s.is_stranger, s.person_id))
    return out


def _resolve_by_query(records: List[FaceSummary], query: str) -> Optional[FaceSummary]:
    """Pick a record by short id (8 chars) OR by exact name (case-insensitive).
    Returns ``None`` when nothing matches. ``query`` is already stripped by
    the handler."""
    q = query.strip()
    if not q:
        return None
    q_lower = q.lower()
    # short id first (operator habit: paste 4ff0ddc5)
    for s in records:
        if s.person_id_short == q or s.person_id == q:
            return s
    for s in records:
        if s.name and s.name.lower() == q_lower:
            return s
    return None
