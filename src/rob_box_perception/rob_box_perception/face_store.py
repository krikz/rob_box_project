#!/usr/bin/env python3
"""face_store.py — единственный модуль, который пишет в ``/data/faces/``.

ADR-0123 («Режимы приватности лицевого канала», заменяет ADR-0089 §8),
issue #2599 (лицо, Phase 2). Читать ADR-0123 целиком, здесь только то,
что нужно для понимания реализации:

* §2 — таблица режимов (``workshop``/``exhibition``/``strict``): кто
  хранится, какие снимки, куда уходит облако. Облачное сравнение
  «было/стало» (§7) сюда не входит — это отдельный модуль, ``FaceStore``
  только решает, что ЛОЖИТСЯ НА ДИСК, а не что уходит наружу.
* §4 — что хранится на человека: галерея эмбеддингов (§4.1), эталонный
  снимок (не ротируется), снимки встреч (ротация по ``keep_encounters``).
* §5 — граница режима: режим передаётся ``FaceStore`` при создании,
  КАЖДОЕ решение «писать ли на диск» принимается внутри него, а не
  вызывающим кодом. Тесты режимов — тесты этого модуля, без ROS и без
  камеры (см. ``test/unit/test_face_store.py``).
* §6 — узнавание (порог косинусной близости) и слияние (голос+лицо,
  через ``merge()`` — сюда шов «Знакомый», ADR-0106, передаёт решение
  «это один человек»; сам ``FaceStore`` о голосе ничего не знает).

Важное ограничение (ADR-0089 §8, переподтверждено ADR-0123 §4.1 и §11):
``last_seen``/``seen_count``/счётчики диалога — собственность шва
«Знакомый» (``rob_box_harness.identity``, ADR-0106), НЕ этого модуля.
``FaceStore`` не заводит собственный ``person_id`` в смысле идентичности
человека между голосом и лицом — merge() принимает уже готовое решение
«слить A и B» от вызывающего кода (адаптера ADR-0106), а не сам его
принимает по голосу. Единственная временная метка, которую хранит этот
модуль — ``last_encounter_ts`` — служебная и нужна ИСКЛЮЧИТЕЛЬНО для
вытеснения самых давних незнакомцев при переполнении ``max_strangers``
(ADR-0123 §10 «минусы»); наружу через публичный API она не отдаётся
(нет ни в ``FaceMatch``, ни в ``people()``) — это не «last_seen» шва,
а внутренняя бухгалтерия хранилища для собственных лимитов на диск.
``encounter_count`` в ``FaceMatch``/``people()`` — тоже не «seen_count»
шва: это подсчёт встреч, накопленных ИМЕННО этой лицевой записью (нужен
вызывающему коду, чтобы решить «выбросить в галерею /perception/health»),
а не кросс-модальная история диалога — той владеет «Знакомый».

Модуль сознательно не знает ни про ROS, ни про cv2/Hailo — тестируется
чистым pytest + numpy (ADR-0123 §5: «Тесты режимов — тесты FaceStore,
без ROS и без камеры»). Снимки приходят уже закодированными JPEG-байтами
(или ``None``) — модуль их не декодирует и не проверяет валидность,
только пишет как есть.
"""

from __future__ import annotations

import json
import logging
import os
import shutil
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

logger = logging.getLogger(__name__)

# ── Режимы (ADR-0123 §2) ─────────────────────────────────────────────────────
MODE_WORKSHOP = 'workshop'
MODE_EXHIBITION = 'exhibition'
MODE_STRICT = 'strict'
VALID_MODES = (MODE_WORKSHOP, MODE_EXHIBITION, MODE_STRICT)

# ADR-0123 §8: «Файлы на Vision Pi: /data/faces/<person>/ — эталон,
# снимки встреч, meta.json». Volume, не образ — переживает пересоздание
# контейнера.
DEFAULT_ROOT = '/data/faces'

_META_FILENAME = 'meta.json'
_EMBEDDINGS_FILENAME = 'embeddings.npy'
_REFERENCE_FILENAME = 'reference.jpg'
_ENCOUNTERS_DIRNAME = 'encounters'


@dataclass(frozen=True)
class FaceMatch:
    """Результат ``record_encounter()``/``identify()``.

    ``similarity`` — косинусная близость к галерее совпавшей записи;
    для только что созданной записи (``is_new=True``) по определению
    1.0 (сравнивать не с чем — сама с собой).
    """

    person_id: str
    name: Optional[str]
    similarity: float
    is_new: bool
    encounter_count: int


@dataclass
class _Record:
    """Внутреннее представление одной записи человека (знакомого или
    незнакомца). Не публичный API — наружу отдаются только ``FaceMatch``
    и словари ``people()``.

    ``persisted`` — живёт ли запись прямо сейчас на диске. В ``workshop``
    персистентны все; в ``exhibition`` — только именованные (§2/§4.3);
    в ``strict`` персистентны только именованные (незнакомцы там вообще
    не заводятся, см. ``FaceStore.record_encounter``).
    """

    person_id: str
    name: Optional[str] = None
    speaker_id: Optional[str] = None
    created_at: float = 0.0
    encounter_count: int = 0
    last_encounter_ts: float = 0.0  # служебное, см. докстринг модуля
    embeddings: List[np.ndarray] = field(default_factory=list)
    encounters: List[Dict[str, Any]] = field(default_factory=list)
    has_reference_snapshot: bool = False
    persisted: bool = False


def _normalize(vec: np.ndarray) -> np.ndarray:
    """L2-нормировать вектор; нулевой вектор возвращает как есть (защита
    от деления на ноль на битом эмбеддинге — не должно случаться в
    реальном пайплайне ArcFace, но тест не обязан это гарантировать)."""
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9:
        return vec
    return vec / norm


def _cosine(a: np.ndarray, b: np.ndarray) -> float:
    """Косинусная близость. Эмбеддинги в ADR-0123/§4.1 уже L2-нормированы
    (реальный ArcFace HEF, 512-dim — см. докстринг ниже), но пересчитываем
    норму защитно: та же тактика, что в ``speaker_embeddings._score_all``
    (голосовой аналог) — дешевле перенормировать, чем завести отдельный
    режим "доверяю/не доверяю входу"."""
    return float(np.dot(_normalize(a), _normalize(b)))


def _atomic_write_bytes(path: Path, data: bytes) -> None:
    """Атомарная запись байтов: temp-файл + ``os.replace`` (ADR-0123 —
    Vision Pi может потерять питание в любой момент, полузаписанный файл
    не должен быть виден под финальным именем)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    with open(tmp, 'wb') as fh:
        fh.write(data)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def _atomic_write_json(path: Path, obj: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, ensure_ascii=False, indent=2, sort_keys=True)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def _atomic_write_npy(path: Path, arr: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    with open(tmp, 'wb') as fh:
        np.save(fh, arr)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


class FaceStore:
    """Единственная точка входа для записи/чтения лицевых данных на диск.

    Параметры порогов и лимитов — стартовые значения из ADR-0123 §4.1/§10
    (``max_embeddings=20``, ``keep_encounters=10``, ``max_strangers=500``);
    ``identify_threshold`` калибруется на реальных данных отдельной
    карточкой (§6 — «как голосовой в #2348, таблица sweep»), здесь —
    только разумный дефолт для тестов и первого запуска.

    ``clock`` — инъекция времени для тестов (по умолчанию ``time.time``);
    используется только для служебного ``last_encounter_ts`` (см.
    докстринг модуля) — не для чего-либо, что отдаётся наружу как
    «когда видели последний раз» (это домен шва «Знакомый», ADR-0106).
    """

    def __init__(
        self,
        root: str = DEFAULT_ROOT,
        mode: str = MODE_WORKSHOP,
        *,
        identify_threshold: float = 0.45,
        max_embeddings: int = 20,
        keep_encounters: int = 10,
        max_strangers: int = 500,
        clock=time.time,
    ) -> None:
        if mode not in VALID_MODES:
            raise ValueError(
                f'face_store: unknown mode {mode!r}, expected one of {VALID_MODES} '
                '(ADR-0123 §2)'
            )
        self._root = Path(root)
        self._root.mkdir(parents=True, exist_ok=True)
        self._mode = mode
        self._identify_threshold = identify_threshold
        self._max_embeddings = max(1, int(max_embeddings))
        self._keep_encounters = max(0, int(keep_encounters))
        self._max_strangers = max(0, int(max_strangers))
        self._clock = clock
        self._lock = threading.Lock()
        self._records: Dict[str, _Record] = {}
        self._embedding_dim: Optional[int] = None
        self._load_from_disk()

    # ── Свойства/статистика ──────────────────────────────────────────────

    @property
    def mode(self) -> str:
        return self._mode

    def stats(self) -> Dict[str, Any]:
        """Сводка для ``/perception/health`` (ADR-0123 §2: «робот обязан
        уметь сказать, в каком он режиме»)."""
        with self._lock:
            named = sum(1 for r in self._records.values() if r.name is not None)
            strangers = len(self._records) - named
            embeddings = sum(len(r.embeddings) for r in self._records.values())
            disk_bytes = 0
            if self._root.exists():
                for p in self._root.rglob('*'):
                    if p.is_file():
                        try:
                            disk_bytes += p.stat().st_size
                        except OSError:
                            continue
            return {
                'mode': self._mode,
                'people': len(self._records),
                'named': named,
                'strangers': strangers,
                'embeddings': embeddings,
                'disk_bytes': disk_bytes,
            }

    # ── Загрузка с диска ─────────────────────────────────────────────────

    def _load_from_disk(self) -> None:
        if not self._root.exists():
            return
        for entry in sorted(self._root.iterdir()):
            if not entry.is_dir():
                continue
            person_id = entry.name
            meta_path = entry / _META_FILENAME
            if not meta_path.exists():
                continue
            try:
                rec = self._load_one_record(person_id, entry, meta_path)
            except Exception as exc:  # noqa: BLE001 — намеренно широкий catch
                # ADR-0123 §5 / требование карточки: битая/неполная запись
                # НЕ должна ронять конструктор — Vision Pi перезапускает
                # ноду часто, а FaceStore открывается на её старте.
                logger.warning(
                    'face_store: пропускаю повреждённую запись %s (%s) — '
                    'corrupt/partial metadata не должно ронять конструктор',
                    person_id, exc,
                )
                continue
            self._records[rec.person_id] = rec

    def _load_one_record(self, person_id: str, person_dir: Path, meta_path: Path) -> _Record:
        with open(meta_path, 'r', encoding='utf-8') as fh:
            data = json.load(fh)
        if data.get('person_id') != person_id:
            raise ValueError('person_id в meta.json не совпадает с именем каталога')

        embeddings: List[np.ndarray] = []
        emb_path = person_dir / _EMBEDDINGS_FILENAME
        if emb_path.exists():
            arr = np.load(emb_path)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            embeddings = [row.astype(np.float32) for row in arr]
            if embeddings and self._embedding_dim is None:
                self._embedding_dim = int(embeddings[0].size)

        return _Record(
            person_id=person_id,
            name=data.get('name'),
            speaker_id=data.get('speaker_id'),
            created_at=float(data.get('created_at', 0.0)),
            encounter_count=int(data.get('encounter_count', 0)),
            last_encounter_ts=float(data.get('last_encounter_ts', 0.0)),
            embeddings=embeddings,
            encounters=list(data.get('encounters', [])),
            has_reference_snapshot=bool(data.get('has_reference_snapshot', False)),
            persisted=True,
        )

    # ── Пути на диске ────────────────────────────────────────────────────

    def _person_dir(self, person_id: str) -> Path:
        return self._root / person_id

    def _encounters_dir(self, person_id: str) -> Path:
        return self._person_dir(person_id) / _ENCOUNTERS_DIRNAME

    # ── Валидация эмбеддингов ────────────────────────────────────────────

    def _prepare_embedding(self, embedding: Any) -> np.ndarray:
        """Привести вход к float32 1-D ndarray и проверить размерность.

        Размерность НЕ хардкодится (карточка: «128-dim» из ADR-0123/0089
        — ошибка, реальный ArcFace HEF отдаёт 512) — она фиксируется по
        первому увиденному эмбеддингу (записанному или считанному с
        диска) и дальше все входящие эмбеддинги обязаны ей соответствовать.
        """
        arr = np.asarray(embedding, dtype=np.float32).reshape(-1)
        if arr.size == 0:
            raise ValueError('face_store: embedding пуст')
        if self._embedding_dim is None:
            self._embedding_dim = int(arr.size)
        elif arr.size != self._embedding_dim:
            raise ValueError(
                f'face_store: размерность эмбеддинга не совпадает: ожидалось '
                f'{self._embedding_dim}, пришло {arr.size}'
            )
        return _normalize(arr)

    # ── Узнавание ────────────────────────────────────────────────────────

    def _score_all(self, embedding: np.ndarray) -> List[tuple]:
        """Best-of-gallery косинусная близость к каждой известной записи,
        отсортировано по убыванию (тот же приём, что ``SpeakerDatabase.
        _score_all`` в голосовом аналоге — max, а не mean: одна неудачная
        встреча не должна размывать уже подтверждённое совпадение)."""
        scored = []
        for person_id, rec in self._records.items():
            if not rec.embeddings:
                continue
            best = max(_cosine(embedding, e) for e in rec.embeddings)
            scored.append((person_id, best))
        scored.sort(key=lambda t: -t[1])
        return scored

    def identify(self, embedding: Any) -> Optional[FaceMatch]:
        """Только чтение — сравнить эмбеддинг с известными галереями, ничего
        не создавать и не дописывать (ADR-0123 §5 контракт: ``identify``
        нужен, например, статусным/просмотровым инструментам, которым
        нельзя случайно завести запись побочным эффектом)."""
        emb = self._prepare_embedding(embedding)
        with self._lock:
            scored = self._score_all(emb)
            if not scored or scored[0][1] < self._identify_threshold:
                return None
            person_id, sim = scored[0]
            rec = self._records[person_id]
            return FaceMatch(
                person_id=person_id,
                name=rec.name,
                similarity=sim,
                is_new=False,
                encounter_count=rec.encounter_count,
            )

    def record_encounter(
        self,
        embedding: Any,
        *,
        snapshot: Optional[bytes] = None,
        body_snapshot: Optional[bytes] = None,
        meta: Optional[dict] = None,
    ) -> FaceMatch:
        """Записать встречу (ADR-0123 §3 — единица хранения, не кадр).

        Решает, с кем сопоставить эмбеддинг, обновляет/создаёт запись и,
        согласно текущему режиму, решает — писать ли что-то на диск.
        Единственное место в модуле, которое имеет право что-то менять
        (кроме ``attach_name``/``merge``/``forget``/``set_mode``).
        """
        emb = self._prepare_embedding(embedding)
        with self._lock:
            now = self._clock()
            scored = self._score_all(emb)

            if scored and scored[0][1] >= self._identify_threshold:
                person_id, sim = scored[0]
                rec = self._records[person_id]
                is_new = False
            else:
                # Незнакомец. ADR-0123 §2: workshop — на диск как знакомый,
                # без имени; exhibition — только в памяти сессии; strict —
                # не хранится вовсе (эфемерный матч, не заводим запись).
                if self._mode == MODE_STRICT:
                    return FaceMatch(
                        person_id=str(uuid.uuid4()),
                        name=None,
                        similarity=1.0,
                        is_new=True,
                        encounter_count=1,
                    )
                person_id = str(uuid.uuid4())
                rec = _Record(person_id=person_id, created_at=now)
                rec.persisted = self._mode == MODE_WORKSHOP
                self._records[person_id] = rec
                is_new = True
                sim = 1.0
                if rec.persisted:
                    self._person_dir(person_id).mkdir(parents=True, exist_ok=True)

            rec.encounter_count += 1
            rec.last_encounter_ts = now

            if self._mode == MODE_STRICT:
                # §4.1: у знакомых в strict — один центроид, не галерея.
                self._fold_into_centroid(rec, emb)
            else:
                rec.embeddings.append(emb)
                self._evict_most_redundant(rec)

            self._maybe_store_snapshots(rec, snapshot, body_snapshot, meta, now)

            if rec.persisted:
                self._persist_record(rec)

            if is_new and rec.name is None:
                self._enforce_max_strangers()

            return FaceMatch(
                person_id=rec.person_id,
                name=rec.name,
                similarity=sim,
                is_new=is_new,
                encounter_count=rec.encounter_count,
            )

    # ── Галерея эмбеддингов ──────────────────────────────────────────────

    def _evict_most_redundant(self, rec: _Record) -> None:
        """ADR-0123 §4.1: при переполнении ``max_embeddings`` вытесняется
        САМЫЙ ПОХОЖИЙ на остальные (он несёт меньше всего новой информации)
        — не самый старый. Похожесть на «остальных» — средняя косинусная
        близость к каждому другому эмбеддингу галереи."""
        while len(rec.embeddings) > self._max_embeddings:
            n = len(rec.embeddings)
            avg_sims = []
            for i in range(n):
                others = [rec.embeddings[j] for j in range(n) if j != i]
                avg_sims.append(
                    sum(_cosine(rec.embeddings[i], o) for o in others) / len(others)
                )
            worst_idx = max(range(n), key=lambda i: avg_sims[i])
            rec.embeddings.pop(worst_idx)

    def _fold_into_centroid(self, rec: _Record, emb: np.ndarray) -> None:
        """``strict``: схлопнуть галерею в один вектор — скользящее среднее
        по числу встреч, а не простое ``(old+new)/2`` (иначе поздние
        встречи получали бы непропорционально много веса)."""
        if not rec.embeddings:
            rec.embeddings = [emb]
            return
        weight = max(1, rec.encounter_count)
        centroid = rec.embeddings[0]
        blended = centroid * ((weight - 1) / weight) + emb * (1.0 / weight)
        rec.embeddings = [_normalize(blended)]

    # ── Снимки ───────────────────────────────────────────────────────────

    def _maybe_store_snapshots(
        self,
        rec: _Record,
        snapshot: Optional[bytes],
        body_snapshot: Optional[bytes],
        meta: Optional[dict],
        now: float,
    ) -> None:
        """ADR-0123 §2/§4.2 — снимки хранятся ТОЛЬКО:
        * в ``workshop`` — для всех (знакомых и незнакомцев);
        * в ``exhibition`` — только для именованных («только знакомых»);
        * никогда в ``strict``.

        Полные кадры не хранятся ни в одном режиме (§4.2) — сюда попадают
        только уже вырезанные кропы, переданные вызывающим кодом.
        """
        if self._mode == MODE_STRICT:
            return
        if self._mode == MODE_EXHIBITION and rec.name is None:
            return
        if not rec.persisted:
            return

        entry: Dict[str, Any] = {'ts': now, 'meta': dict(meta or {})}

        if snapshot and not rec.has_reference_snapshot:
            # §4.1: «эталонный снимок — первый снимок при создании записи.
            # Не ротируется». Трактовка: первый снимок, который вообще
            # достался записи (не обязательно ровно в момент создания —
            # первая встреча вполне может прийти без кадра, например, при
            # плохом ракурсе).
            ref_path = self._person_dir(rec.person_id) / _REFERENCE_FILENAME
            _atomic_write_bytes(ref_path, snapshot)
            rec.has_reference_snapshot = True

        idx = rec.encounter_count
        if snapshot:
            face_name = f'{idx:06d}_face.jpg'
            _atomic_write_bytes(self._encounters_dir(rec.person_id) / face_name, snapshot)
            entry['face_snapshot'] = face_name
        else:
            entry['face_snapshot'] = None

        if body_snapshot:
            body_name = f'{idx:06d}_body.jpg'
            _atomic_write_bytes(self._encounters_dir(rec.person_id) / body_name, body_snapshot)
            entry['body_snapshot'] = body_name
        else:
            entry['body_snapshot'] = None

        rec.encounters.append(entry)
        self._rotate_encounter_snapshots(rec)

    def _rotate_encounter_snapshots(self, rec: _Record) -> None:
        """§4.1: «по одному лучшему на встречу за последние keep_encounters
        встреч. Старые удаляются». Эталонный снимок сюда не входит — он
        живёт в отдельном файле и этой ротацией не затрагивается."""
        while len(rec.encounters) > self._keep_encounters:
            oldest = rec.encounters.pop(0)
            self._delete_encounter_files(rec.person_id, oldest)

    def _delete_encounter_files(self, person_id: str, entry: Dict[str, Any]) -> None:
        enc_dir = self._encounters_dir(person_id)
        for key in ('face_snapshot', 'body_snapshot'):
            name = entry.get(key)
            if not name:
                continue
            path = enc_dir / name
            if path.exists():
                try:
                    path.unlink()
                except OSError:
                    pass

    def _delete_all_snapshots(self, rec: _Record) -> None:
        if not rec.persisted:
            return
        person_dir = self._person_dir(rec.person_id)
        ref = person_dir / _REFERENCE_FILENAME
        if ref.exists():
            ref.unlink()
        enc_dir = self._encounters_dir(rec.person_id)
        if enc_dir.exists():
            shutil.rmtree(enc_dir, ignore_errors=True)

    # ── Персистентность записи ───────────────────────────────────────────

    def _persist_record(self, rec: _Record) -> None:
        person_dir = self._person_dir(rec.person_id)
        person_dir.mkdir(parents=True, exist_ok=True)
        meta = {
            'person_id': rec.person_id,
            'name': rec.name,
            'speaker_id': rec.speaker_id,
            'created_at': rec.created_at,
            'encounter_count': rec.encounter_count,
            'last_encounter_ts': rec.last_encounter_ts,
            'has_reference_snapshot': rec.has_reference_snapshot,
            'encounters': rec.encounters,
            'mode_recorded': self._mode,
        }
        _atomic_write_json(person_dir / _META_FILENAME, meta)
        dim = self._embedding_dim or 0
        arr = (
            np.stack(rec.embeddings).astype(np.float32)
            if rec.embeddings
            else np.zeros((0, dim), dtype=np.float32)
        )
        _atomic_write_npy(person_dir / _EMBEDDINGS_FILENAME, arr)

    def _remove_record(self, rec: _Record) -> None:
        self._records.pop(rec.person_id, None)
        if rec.persisted:
            shutil.rmtree(self._person_dir(rec.person_id), ignore_errors=True)

    # ── Незнакомцы: верхний предел (ADR-0123 §10) ───────────────────────

    def _enforce_max_strangers(self) -> None:
        strangers = [r for r in self._records.values() if r.name is None]
        overflow = len(strangers) - self._max_strangers
        if overflow <= 0:
            return
        strangers.sort(key=lambda r: r.last_encounter_ts)
        for rec in strangers[:overflow]:
            logger.info(
                'face_store: max_strangers=%d превышен, вытесняю самого '
                'давнего незнакомца id=%s', self._max_strangers, rec.person_id,
            )
            self._remove_record(rec)

    # ── Имя / шов «Знакомый» ─────────────────────────────────────────────

    def attach_name(
        self, person_id: str, name: str, *, speaker_id: Optional[str] = None
    ) -> bool:
        """Привязать имя к записи (превращает незнакомца в знакомого).

        ``FaceStore`` не решает, ЧТО это за человек — имя и опциональный
        ``speaker_id`` (для склейки с голосовым профилем, ADR-0106 §4.1)
        приходит от вызывающего кода (шва «Знакомый»/§6 этого ADR).
        Именованная запись персистентна на диске в ЛЮБОМ режиме (§2 —
        даже ``strict`` хранит «эмбеддинг + имя» знакомых), поэтому
        промоутит ранее in-memory-only запись (незнакомец в ``exhibition``).
        """
        with self._lock:
            rec = self._records.get(person_id)
            if rec is None:
                return False
            rec.name = name
            if speaker_id is not None:
                rec.speaker_id = speaker_id
            if not rec.persisted:
                rec.persisted = True
            self._persist_record(rec)
            return True

    def find_by_speaker(self, speaker_id: str) -> Optional[str]:
        with self._lock:
            for rec in self._records.values():
                if rec.speaker_id == speaker_id:
                    return rec.person_id
            return None

    def merge(self, keep_id: str, drop_id: str) -> bool:
        """Слить ``drop_id`` в ``keep_id`` (ADR-0123 §6 — вызывается после
        того, как шов «Знакомый» решил, что это один человек по голосу +
        ровно одно лицо в кадре). Переносит эмбеддинги и файлы снимков,
        ``drop_id`` удаляется полностью."""
        with self._lock:
            if not keep_id or not drop_id or keep_id == drop_id:
                return False
            keep = self._records.get(keep_id)
            drop = self._records.get(drop_id)
            if keep is None or drop is None:
                return False

            keep.embeddings.extend(drop.embeddings)
            if self._mode == MODE_STRICT:
                self._fold_all_into_one_centroid(keep)
            else:
                self._evict_most_redundant(keep)

            if drop.persisted:
                self._migrate_snapshot_files(keep, drop)
                keep.encounters.extend(drop.encounters)
                self._rotate_encounter_snapshots(keep)

            keep.encounter_count += drop.encounter_count
            if keep.speaker_id is None:
                keep.speaker_id = drop.speaker_id
            keep.last_encounter_ts = max(keep.last_encounter_ts, drop.last_encounter_ts)

            self._remove_record(drop)

            if keep.persisted:
                self._persist_record(keep)
            return True

    def _fold_all_into_one_centroid(self, rec: _Record) -> None:
        if len(rec.embeddings) <= 1:
            return
        centroid = _normalize(np.sum(np.stack(rec.embeddings), axis=0))
        rec.embeddings = [centroid]

    def _migrate_snapshot_files(self, keep: _Record, drop: _Record) -> None:
        drop_dir = self._person_dir(drop.person_id)
        keep_dir = self._person_dir(keep.person_id)
        keep_dir.mkdir(parents=True, exist_ok=True)

        drop_ref = drop_dir / _REFERENCE_FILENAME
        if not keep.has_reference_snapshot and drop_ref.exists():
            shutil.move(str(drop_ref), str(keep_dir / _REFERENCE_FILENAME))
            keep.has_reference_snapshot = True

        drop_enc_dir = drop_dir / _ENCOUNTERS_DIRNAME
        if not drop_enc_dir.exists():
            return
        keep_enc_dir = self._encounters_dir(keep.person_id)
        keep_enc_dir.mkdir(parents=True, exist_ok=True)
        for entry in drop.encounters:
            for key in ('face_snapshot', 'body_snapshot'):
                fname = entry.get(key)
                if not fname:
                    continue
                src = drop_enc_dir / fname
                if not src.exists():
                    continue
                new_name = f'merged-{drop.person_id[:8]}-{fname}'
                shutil.move(str(src), str(keep_enc_dir / new_name))
                entry[key] = new_name

    def forget(self, person_id: str) -> bool:
        """«Забудь меня» (ADR-0123 §5/§11) — работает во всех режимах,
        стирает галерею, снимки и саму запись целиком."""
        with self._lock:
            rec = self._records.get(person_id)
            if rec is None:
                return False
            self._remove_record(rec)
            return True

    # ── Просмотр ─────────────────────────────────────────────────────────

    def gallery(self, person_id: str) -> List[np.ndarray]:
        with self._lock:
            rec = self._records.get(person_id)
            if rec is None:
                return []
            return [e.copy() for e in rec.embeddings]

    def people(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [
                {
                    'person_id': rec.person_id,
                    'name': rec.name,
                    'encounter_count': rec.encounter_count,
                    'has_snapshot': rec.has_reference_snapshot,
                    'is_stranger': rec.name is None,
                    'persisted': rec.persisted,
                    'embeddings': len(rec.embeddings),
                }
                for rec in self._records.values()
            ]

    # ── Смена режима (ADR-0123 §5) ───────────────────────────────────────

    def set_mode(self, mode: str) -> None:
        """Применить правила перехода режима (ADR-0123 §5):

        * ``workshop -> exhibition``: записи незнакомцев с диска
          УДАЛЯЮТСЯ (иначе выставочный режим начнётся с чужих снимков
          из мастерской); знакомые остаются.
        * ``* -> strict``: удаляются все снимки и все незнакомцы; у
          знакомых остаётся один эмбеддинг-центроид.
        * Переход в более мягкий режим НИЧЕГО не восстанавливает —
          удалённое остаётся удалённым, будущие встречи просто начинают
          жить по новым (более мягким) правилам.
        """
        if mode not in VALID_MODES:
            raise ValueError(
                f'face_store: unknown mode {mode!r}, expected one of {VALID_MODES}'
            )
        with self._lock:
            old = self._mode
            if old == mode:
                return
            if old == MODE_WORKSHOP and mode == MODE_EXHIBITION:
                self._drop_all_strangers()
            if mode == MODE_STRICT:
                self._collapse_to_strict()
            self._mode = mode

    def _drop_all_strangers(self) -> None:
        for pid in [pid for pid, r in self._records.items() if r.name is None]:
            self._remove_record(self._records[pid])

    def _collapse_to_strict(self) -> None:
        self._drop_all_strangers()
        for rec in self._records.values():
            self._delete_all_snapshots(rec)
            if rec.embeddings:
                centroid = _normalize(np.sum(np.stack(rec.embeddings), axis=0))
                rec.embeddings = [centroid]
            rec.encounters = []
            rec.has_reference_snapshot = False
            if rec.persisted:
                self._persist_record(rec)


__all__ = [
    'MODE_WORKSHOP',
    'MODE_EXHIBITION',
    'MODE_STRICT',
    'VALID_MODES',
    'DEFAULT_ROOT',
    'FaceMatch',
    'FaceStore',
]
