"""Библиотека партитур на устройстве (ADR-0154 §3.2, §3.5; PR-5): индекс ``score_index`` и JSON ``ScoreMaterial``.

Каталог библиотеки — один, самодостаточный: ``score_index.db`` (таблица ``score_index``, пишет офлайн-импортёр
``scripts/music/score_import.py --db <каталог>/score_index.db``) и JSON-материалы рядом (``--out-dir <каталог>``,
имя файла — колонка ``file``). Путь — :data:`ENV` или :data:`DEFAULT_DIR`: ресурсный пак на хосте, как сэмплы
(решение Шифу по В7, 07.10: вариант «б»), bind-mount в контейнер голоса тем же путём, ``:ro``. Сам пак (запись
манифеста и фетчер) — отдельный PR.

Нет каталога, индекса или таблицы — :meth:`ScoreLibrary.rows` пуст, а :attr:`ScoreLibrary.state` — честная строка
причины для лога: сет играет хуки RTTTL, как до PR-5. Битый JSON или материал, не прошедший валидатор, — пропуск с
причиной (:meth:`ScoreLibrary.load`), не падение сета. В git партитур и библиотеки нет (ADR-0154 В1, ADR-0155 В1).
"""

from __future__ import annotations

import os
import pathlib
import sqlite3
import threading
import time
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from rob_box_music.material import MaterialError, ScoreMaterial, from_json
from rob_box_music.set_plan import seeded_plan

#: Переменная окружения с каталогом библиотеки (перекрывает :data:`DEFAULT_DIR`).
ENV = "ROB_BOX_SCORE_LIBRARY"
#: Ресурсный пак на хосте (В7 «б»), рядом с ``/opt/rob_box/samples`` и ``/opt/rob_box/models``.
DEFAULT_DIR = "/opt/rob_box/scores"
INDEX_FILE = "score_index.db"
_COLUMNS = ("material_id", "title", "composer", "rating", "n_ratings", "file")


def library_dir() -> str:
    """Каталог библиотеки: :data:`ENV`, иначе :data:`DEFAULT_DIR`."""
    return os.environ.get(ENV, "").strip() or DEFAULT_DIR


class ScoreLibrary:
    """Индекс партитур читается один раз (при первой теме), материалы — по запросу, с кэшем."""

    def __init__(self, path: Optional[str] = None) -> None:
        self.path = pathlib.Path(path or library_dir())
        self.state = "не открыта"
        self._rows: Optional[Tuple[Dict[str, Any], ...]] = None
        self._cache: Dict[str, ScoreMaterial] = {}
        self._lock = threading.Lock()

    def rows(self) -> Tuple[Dict[str, Any], ...]:
        """Строки ``score_index`` (``material_id, title, composer, rating, n_ratings, file``); нет — пусто."""
        with self._lock:
            if self._rows is None:
                self._rows = self._read_index()
            return self._rows

    def _read_index(self) -> Tuple[Dict[str, Any], ...]:
        index = self.path / INDEX_FILE
        if not index.is_file():
            self.state = f"нет {index} — хуки RTTTL"
            return ()
        try:
            conn = sqlite3.connect(f"file:{index.as_posix()}?mode=ro", uri=True)
            try:
                rows = conn.execute(f"SELECT {','.join(_COLUMNS)} FROM score_index").fetchall()
            finally:
                conn.close()
        except sqlite3.Error as exc:
            self.state = f"{index}: {type(exc).__name__}: {exc} — хуки RTTTL"
            return ()
        self.state = f"{index}: партитур {len(rows)}"
        return tuple(dict(zip(_COLUMNS, r)) for r in rows)

    def titles(self, ids: Sequence[str]) -> Dict[str, str]:
        """``{material_id: название}`` для голоса диджея (``engine.dj_lines``)."""
        by_id = {r["material_id"]: str(r["title"] or "") for r in self.rows()}
        return {i: by_id[i] for i in ids if by_id.get(i)}

    def load(self, ids: Sequence[str]) -> Tuple[Dict[str, ScoreMaterial], List[str]]:
        """``({material_id: материал}, [причины пропуска])``: JSON по колонке ``file``, валидатор ``material.py``."""
        files = {r["material_id"]: r["file"] for r in self.rows()}
        out: Dict[str, ScoreMaterial] = {}
        skipped: List[str] = []
        for mid in ids:
            if mid in self._cache:
                out[mid] = self._cache[mid]
                continue
            try:
                material = from_json((self.path / str(files[mid])).read_text(encoding="utf-8"))
            except (KeyError, OSError, ValueError, MaterialError) as exc:
                skipped.append(f"{mid}: {type(exc).__name__}: {exc}")
                continue
            if material.material_id != mid:
                skipped.append(f"{mid}: в файле {material.material_id}")
                continue
            out[mid] = self._cache[mid] = material
        return out, skipped


class PlanMaterials(Mapping[str, ScoreMaterial]):
    """``{material_id: ScoreMaterial}`` треков плана для ``compose(materials=)``: JSON читается при первом обращении
    (компоновка трека N+1 — в фоне, не на пути звука), пропуск — строкой лога (``logger``), трек берёт хук темы."""

    def __init__(self, library: ScoreLibrary, ids: Sequence[str], logger: Any = None) -> None:
        self._library = library
        self._ids = tuple(dict.fromkeys(ids))
        self._log = logger

    def __getitem__(self, material_id: str) -> ScoreMaterial:
        if material_id not in self._ids:
            raise KeyError(material_id)
        found, skipped = self._library.load([material_id])
        if skipped and self._log is not None:
            self._log.warning(f"⚠️ [dj_set] материал пропущен: {'; '.join(skipped)}")
        return found[material_id]

    def __iter__(self):
        return iter(self._ids)

    def __len__(self) -> int:
        return len(self._ids)


def seed_plan(library: ScoreLibrary, profile: Any, seed: int, length: int, set_id: str, history: Sequence[Mapping],
              logger: Any) -> Any:
    """``seeded_plan`` с отбором годных материалов (#3500): негодные в очередь не попадают, каждый — строкой лога
    с причиной (та же, что отказ ``hook.from_material``), отбор — с замером (M7: ≤ 50 мс на запрос)."""
    rejected: Dict[str, str] = {}
    started = time.perf_counter()
    materials = PlanMaterials(library, profile.materials, logger) if profile.materials else None
    plan = seeded_plan(profile, seed, n_tracks=length, set_id=set_id, history=history, materials=materials,
                       rejected=rejected)  # темп и окно — на сет
    for mid, why in rejected.items():
        logger.info(f"🎼 [dj_set] материал {mid} не годится: {why}")
    if profile.materials:
        logger.info(f"🎼 [dj_set] {set_id} отбор материалов {len(profile.materials)} шт. за "
                    f"{(time.perf_counter() - started) * 1000:.1f} мс: в плане "
                    f"{sum(bool(t.material) for t in plan.tracks)}, негодных {len(rejected)}")
    return plan


__all__ = ["DEFAULT_DIR", "ENV", "INDEX_FILE", "ScoreLibrary", "PlanMaterials", "library_dir", "seed_plan"]
