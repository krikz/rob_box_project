"""Библиотека партитур на устройстве (ADR-0154 §3.2, §3.5; PR-5): индекс ``score_index`` и JSON ``ScoreMaterial``.

Каталог библиотеки — один, самодостаточный: ``score_index.db`` (таблица ``score_index``, пишет офлайн-импортёр
``scripts/music/score_import.py --db <каталог>/score_index.db``) и JSON-материалы рядом (``--out-dir <каталог>``,
имя файла — колонка ``file``). Путь — :data:`ENV` или :data:`DEFAULT_DIR`: ресурсный пак на хосте, как сэмплы
(решение Шифу по В7, 07.10: вариант «б»), bind-mount в контейнер голоса тем же путём, ``:ro``. Сам пак (запись
манифеста и фетчер) — отдельный PR.

Поиск (#3512): тема разрешается в строки поиска ОДИН раз — ``search.part_query`` (семена и связи реестра, алиасы,
слова архива по звуку, жанр); мелодии RTTTL и партитуры ищутся по одним строкам (``ThemeHits.query``).
:class:`ScoreIndex` — словарный индекс по звуковым ключам слов названия и композитора и по жанрам PDMX, строится
один раз (в фоне при старте ноды): запрос — пересечение списков, без прохода по всем строкам (M7 ≤ 100 мс при
135 тыс. партитур).

Нет каталога, индекса или таблицы — :meth:`ScoreLibrary.rows` пуст, а :attr:`ScoreLibrary.state` — честная строка
причины для лога: сет играет хуки RTTTL, как до PR-5. Битый JSON или материал, не прошедший валидатор, — пропуск с
причиной (:meth:`ScoreLibrary.load`), не падение сета. В git партитур и библиотеки нет (ADR-0154 В1, ADR-0155 В1).
"""

from __future__ import annotations

import bisect
import heapq
import os
import pathlib
import re
import sqlite3
import threading
import time
from typing import Any, Dict, List, Mapping, Optional, Sequence, Set, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.material import MaterialError, ScoreMaterial, from_json
from rob_box_music.rtttl import THEME_HOOKS
from rob_box_music.set_plan import seeded_plan

from ..core.translit_ru import transliterate_ru
from .search import (_PREFIX_SLACK, _WHOLE_WORD_MAX, PartQuery, Term, ThemeQuery, round_robin, sound_key,
                     title_key)

#: Переменная окружения с каталогом библиотеки (перекрывает :data:`DEFAULT_DIR`).
ENV = "ROB_BOX_SCORE_LIBRARY"
#: Ресурсный пак на хосте (В7 «б»), рядом с ``/opt/rob_box/samples`` и ``/opt/rob_box/models``.
DEFAULT_DIR = "/opt/rob_box/scores"
INDEX_FILE = "score_index.db"
_COLUMNS = ("material_id", "title", "composer", "genres", "rating", "n_ratings", "file")
_CYR_RE = re.compile(r"[а-яё]", re.IGNORECASE)


def _keys(text: str, memo: Dict[str, str]) -> Set[str]:
    """Звуковые ключи слов текста (русские буквы — транслитом): «J. S. Bach» → {j, s, ba4}; ``memo`` — ключ слова
    считается один раз на постройку индекса (слова названий повторяются)."""
    text = str(text or "")
    out = set()
    for word in title_key(transliterate_ru(text) if _CYR_RE.search(text) else text).split():
        key = memo.get(word)
        if key is None:
            key = memo[word] = sound_key(word)
        out.add(key)
    return out


class ScoreIndex:
    """Словарный индекс партитур: звуковой ключ слова названия/композитора → строки, жанр PDMX → строки.

    Строки пронумерованы в порядке ранга (рейтинг PDMX, число оценок, ``material_id``): меньший номер — выше, и
    отбор лучших — ``heapq.nsmallest`` по найденным номерам, без сортировки всего каталога."""

    def __init__(self, rows: Sequence[Mapping[str, Any]]) -> None:
        ranked = sorted(rows, key=lambda r: (-float(r.get("rating") or 0.0), -int(r.get("n_ratings") or 0),
                                             str(r["material_id"])))
        self.ids: Tuple[str, ...] = tuple(str(r["material_id"]) for r in ranked)
        self._title_len: List[int] = []
        self._words: Dict[str, List[int]] = {}
        self._genres: Dict[str, List[int]] = {}
        memo: Dict[str, str] = {}
        for i, row in enumerate(ranked):
            title = _keys(row.get("title"), memo)
            self._title_len.append(len(title))
            for key in title | _keys(row.get("composer"), memo):
                self._words.setdefault(key, []).append(i)
            for genre in str(row.get("genres") or "").lower().split("-"):
                if genre:
                    self._genres.setdefault(genre, []).append(i)
        self._sorted_keys = sorted(self._words)

    def _word(self, key: str, start: str = "", by_start: bool = False) -> Set[int]:
        """Строки со словом ``key`` (или ``start``); ``by_start`` — и словом, начинающимся с ``start`` (падеж,
        «интерстеллара»), не длиннее на :data:`search._PREFIX_SLACK` букв."""
        out = set(self._words.get(key, ())) | set(self._words.get(start, ()) if start else ())
        if by_start and start:
            i = bisect.bisect_left(self._sorted_keys, start)
            while i < len(self._sorted_keys) and self._sorted_keys[i].startswith(start):
                word = self._sorted_keys[i]
                if len(word) - len(start) <= _PREFIX_SLACK:
                    out.update(self._words[word])
                i += 1
        return out

    def _phrase(self, text: str) -> Set[int]:
        """Строки, где есть все слова строки поиска; длинное слово (> :data:`search._WHOLE_WORD_MAX`) — и по
        началу, короткое — только целым («bach» ≠ «Bachelor»)."""
        keys = [sound_key(w) for w in title_key(text).split()]
        if not keys:
            return set()
        rows: Optional[Set[int]] = None
        for key, word in zip(keys, title_key(text).split()):
            hits = self._word(key, key, len(word) > _WHOLE_WORD_MAX)
            rows = hits if rows is None else rows & hits
            if not rows:
                return set()
        return rows or set()

    def _term(self, term: Term) -> Set[int]:
        """Строки с любым написанием слова: строки поиска (семена, алиасы, слова архива) или само слово по звуку."""
        rows: Set[int] = set()
        for alt in term.alts:
            rows |= self._phrase(alt)
        word, word_stem, by_start = term.latin
        if word:
            rows |= self._word(sound_key(word), sound_key(word_stem), by_start)
        return rows

    def _words_of(self, terms: Sequence[Term]) -> Set[int]:
        """Строки, где есть все слова (пересечение, пустое — сразу)."""
        rows: Optional[Set[int]] = None
        for term in terms:
            hits = self._term(term)
            rows = hits if rows is None else rows & hits
            if not rows:
                return set()
        return rows or set()

    def _part(self, part: PartQuery) -> Tuple[Set[int], int]:
        """``(строки части, слов части для «название — ровно слова темы»)``: все значимые слова части или любая
        строка связи реестра; часть из одного жанра — партитуры жанра (:data:`knowledge.SCORE_GENRES`). Слово жанра
        в части со словами («кино про космос») — окно каталога, не слово названия."""
        if part.genre_only:
            rows = set().union(*(self._genres.get(g, ()) for g in kn.SCORE_GENRES.get(part.genre or "", ())))
            return rows, 0
        content = [t for t in part.terms if not part.genre or not _genre_word(t.word)]
        rows = self._words_of(content) if content else set()
        for query in part.links:
            rows |= self._phrase(query)
        return rows, len(content)

    def search(self, query: Optional[ThemeQuery], limit: int = THEME_HOOKS) -> Tuple[str, ...]:
        """Материалы темы: по частям (лучшие первыми: название ровно из слов части, затем ранг), слияние по кругу —
        «Бах и Моцарт» чередует Баха и Моцарта."""
        lists = []
        for part in (query.parts if query else ()):
            rows, n_words = self._part(part)
            # «название — ровно слова части» только у части со словами: у жанра (0 слов) первыми встали бы
            # названия без единого читаемого слова (кракозябры PDMX, 07.10)
            best = heapq.nsmallest(limit, rows, key=lambda i: (bool(n_words) and self._title_len[i] != n_words, i))
            lists.append([self.ids[i] for i in best])
        return tuple(round_robin(lists, limit))


def _genre_word(word: str) -> bool:
    """Слово жанра или слово при жанре («кино», «классической», «музыки»)."""
    return word in kn.GENRE_FILLER or any(word.startswith(key) for key in kn.GENRE_TAGS)


def library_dir() -> str:
    """Каталог библиотеки: :data:`ENV`, иначе :data:`DEFAULT_DIR`."""
    return os.environ.get(ENV, "").strip() or DEFAULT_DIR


class ScoreLibrary:
    """Индекс партитур читается один раз (при первой теме), материалы — по запросу, с кэшем."""

    def __init__(self, path: Optional[str] = None) -> None:
        self.path = pathlib.Path(path or library_dir())
        self.state = "не открыта"
        self._rows: Optional[Tuple[Dict[str, Any], ...]] = None
        self._by_id: Dict[str, Dict[str, Any]] = {}
        self._index: Optional[ScoreIndex] = None
        self.index_ms = 0.0  # сколько строился индекс поиска (лог старта: M7 считает запрос, не постройку)
        self._cache: Dict[str, ScoreMaterial] = {}
        self._lock = threading.Lock()

    def rows(self) -> Tuple[Dict[str, Any], ...]:
        """Строки ``score_index`` (``material_id, title, composer, genres, rating, n_ratings, file``); нет — пусто."""
        with self._lock:
            if self._rows is None:
                self._rows = self._read_index()
                self._by_id = {r["material_id"]: r for r in self._rows}
            return self._rows

    def index(self) -> ScoreIndex:
        """Индекс поиска (:class:`ScoreIndex`), строится при первом обращении; ``warm`` — заранее, в фоне."""
        rows = self.rows()
        with self._lock:
            if self._index is None:
                started = time.perf_counter()
                self._index = ScoreIndex(rows)
                self.index_ms = (time.perf_counter() - started) * 1000
            return self._index

    def warm(self) -> None:
        """Прочитать индекс и построить поиск в фоне (старт ноды): первая тема сета их не ждёт."""
        threading.Thread(target=self.index, name="rbx-score-index", daemon=True).start()

    def search(self, query: Optional[ThemeQuery], limit: int = THEME_HOOKS) -> Tuple[str, ...]:
        """Материалы темы по её строкам поиска (``ThemeHits.query``, одни с поиском мелодий RTTTL)."""
        return self.index().search(query, limit)

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
        self.rows()
        return {i: str(self._by_id[i]["title"]) for i in ids if i in self._by_id and self._by_id[i]["title"]}

    def load(self, ids: Sequence[str]) -> Tuple[Dict[str, ScoreMaterial], List[str]]:
        """``({material_id: материал}, [причины пропуска])``: JSON по колонке ``file``, валидатор ``material.py``."""
        self.rows()
        files = {mid: self._by_id[mid]["file"] for mid in ids if mid in self._by_id}
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


__all__ = ["DEFAULT_DIR", "ENV", "INDEX_FILE", "ScoreIndex", "ScoreLibrary", "PlanMaterials", "library_dir",
           "seed_plan"]
