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

# Версия схемы эмбеддинга (issue #2772, #2773). Инкрементируется КАЖДЫЙ
# раз, когда меняется то, ЧТО именно превращается в вектор — кроп,
# выравнивание, препроцессинг перед ArcFace HEF. Старые векторы после
# такой смены живут в другом пространстве и молча сравнивать их с
# новыми нельзя — тот же класс ошибки, что сравнить 128-dim с 512-dim
# эмбеддингом (см. предупреждение про embedding_dim в hailo_models.yaml).
#
#   1 — невыровненный кроп: bbox детектора + resize до входа ArcFace,
#       без выравнивания по landmark'ам (весь период до #2773). Именно
#       на этой версии отравилась живая галерея «Деньчика» (#2772) —
#       разные ракурсы одного и того же кропа давали разъезжающиеся
#       вектора, и порог 0.45 не мог их отличить от чужого лица.
#   2 — кроп выровнен по landmark'ам перед ArcFace (issue #2773).
#
# При загрузке записи с диска несовпадение версии (в т.ч. её отсутствие
# — meta.json старее этого поля) НЕ ронятет запись целиком: имя,
# speaker_id, эталонный снимок и встречи остаются (это по-прежнему тот
# же человек), но галерея эмбеддингов отбрасывается — см.
# ``FaceStore._load_one_record``. Побочный эффект, который и нужен:
# отравленная галерея с реального робота вычищается сама на первом
# старте после апгрейда, без ручной чистки диска.
CURRENT_EMBEDDING_VERSION = 2

#: Сколько эмбеддингов запись набирает «по прогреву» — то есть по одному
#: лишь факту узнавания, без второго, более строгого порога
#: ``enroll_threshold`` (issue #2771, см. ``FaceStore._should_enroll``).
#:
#: Пять — не подбор по данным, а наименьшее число, при котором галерея
#: перестаёт быть одним ракурсом: анфас, пол-оборота влево и вправо,
#: голова выше и ниже камеры. Ровно столько же берёт голосовой тракт
#: этого репозитория (``speaker_embeddings.GALLERY_WARMUP_SIZE = 5``),
#: и по той же причине — чтобы у профиля появился разброс раньше, чем
#: к нему начнут применять строгие пороги.
#:
#: Живой замер 22.09.2026, который и потребовал прогрева: три записи
#: ОДНОГО человека, снятые за пять минут, дали попарные косинусы 0.538 /
#: 0.582 / 0.461 — весь внутриперсонный разброс ниже ``enroll_threshold``
#: (0.75). Ни одна галерея не выросла дальше семени, и каждый новый
#: ракурс заводил нового «человека».
DEFAULT_GALLERY_WARMUP_SIZE = 5

# issue #2771 (живая проверка 23.09.2026, develop 816013f92): прогрев
# галереи (выше) чинит РОСТ галереи молодой записи, но не чинит уже
# заведённый дубль. Живой сценарий - "Дэнчик"/"a659ddab": очки роняют
# сходство с настоящей записью с 0.737 до 0.582 (промах на 0.018 мимо
# identify=0.60), заводится третья запись "в очках"; на следующей
# встрече настоящая запись УЖЕ проходит порог (rsim=0.610..0.650), но
# свежий дубль набирает больше (0.618..0.721) и выигрывает по ``max``
# (``_score_all``) - человек остаётся незнакомцем, хотя правильный
# ответ лежит в базе и проходит порог. Полная хронология - комментарии
# issue #2771 от 22-23.09.2026.
#
# ``_disambiguate`` разруливает ровно этот случай: если лучший скор -
# у БЕЗЫМЯННОЙ записи, но ИМЕНОВАННАЯ запись тоже проходит
# ``identify_threshold`` и отстаёт не больше чем на
# ``DEFAULT_DISAMBIGUATION_GAP`` - отвечаем именем и НЕ дописываем
# эмбеддинг в галерею безымянного дубля (реассайн на именованную запись
# означает, что ``_should_enroll``/прогрев в ``record_encounter``
# применяются к НЕЙ, а не к дублю).
#
# Цифра 0.10 - не подбор по данным (настоящий sweep всё ещё issue
# #2771), а покрытие двух живых зазоров из хронологии 22-23.09.2026, где
# правильный ответ уже лежал в базе и проходил порог: ``gap=0.047``
# (07:07:48Z, "Дэнчик" 0.610 против дубля 0.657) и ``gap=0.087``
# (16:45:23, "Дэнчик" 0.634 против дубля 0.721) - оба заведомо меньше
# 0.10 с запасом. Больший зазор (``gap=0.083`` в 06:35:55Z) правило и не
# должно трогать: там именованная запись (0.536) вообще НЕ проходила
# ``identify_threshold`` - это отдельная, не покрытая пока часть
# проблемы (см. §6 ADR-0123: настоящий sweep решит и её).
#
# Второе условие ("слипается", не просто "близко по одному кадру"):
# кросс-сходство ГАЛЕРЕЙ безымянной и именованной записи (см.
# ``_gallery_cross_similarity``) обязано само по себе пройти
# ``identify_threshold``. Без этого условия правило могло бы сработать
# на ЧУЖОМ человеке, который просто оказался на полпути между двумя
# случайными записями в один-единственный момент (одна встреча - не
# доказательство, что записи вообще похожи ДРУГ НА ДРУГА); риск
# остаточный при малых галереях (см. докстринг ``_disambiguate`` и PR).
DEFAULT_DISAMBIGUATION_GAP = 0.10

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

    ``runner_up_person_id``/``runner_up_similarity`` — второй кандидат
    по ``_score_all`` и его score (issue #2771: «логировать не только
    победителя, но и второго кандидата с зазором — как уже делает
    ``speaker_id_node``», см. его ``🔍 identify candidates: best=...
    second=... gap=...``). ``None``, если известных записей меньше двух
    (не с кем сравнивать) — в т.ч. когда ``scored`` вообще пуст. Поля
    добавлены в конец и с дефолтом ``None`` намеренно: это не новый
    контракт, а необязательная диагностика, старые вызывающие вправе её
    игнорировать.
    """

    person_id: str
    name: Optional[str]
    similarity: float
    is_new: bool
    encounter_count: int
    runner_up_person_id: Optional[str] = None
    runner_up_similarity: Optional[float] = None
    #: issue #2771: True, если это совпадение - результат
    #: ``_disambiguate`` (безымянный дубль проигнорирован в пользу
    #: именованной записи, прошедшей ``identify_threshold`` в пределах
    #: ``disambiguation_gap``). Диагностика для лога "Встреча" и
    #: сводки ``[лицо]`` - не влияет на остальной контракт FaceMatch.
    disambiguated: bool = False


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
    # Версия схемы эмбеддингов, которым ЭТА галерея сейчас соответствует
    # (issue #2772/#2773) — см. CURRENT_EMBEDDING_VERSION выше. Не путать
    # с версией конкретного вектора: FaceStore не хранит смешанные
    # версии в одной галерее, поэтому одного поля на запись достаточно.
    embedding_version: int = CURRENT_EMBEDDING_VERSION


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
    (``max_embeddings=20``, ``keep_encounters=10``, ``max_strangers=500``).

    Два РАЗНЫХ порога узнавания (issue #2772 — раньше это был один и тот
    же порог, и это была ошибка):

    * ``identify_threshold`` — «похож достаточно, чтобы НАЗВАТЬ имя».
      Best-of-gallery score (``_score_all``) должен пройти этот порог,
      чтобы ``record_encounter``/``identify`` сочли эмбеддинг известным
      человеком.
    * ``enroll_threshold`` — «похож достаточно, чтобы ДОПИСАТЬ эмбеддинг
      в галерею» этого человека. Заметно строже ``identify_threshold`` и
      требует согласия ДВУХ проверок (``_should_enroll``): best-of-gallery
      score и косинус к медоиду галереи. Встреча, прошедшая ``identify``,
      но не ``enroll``, всё равно называется по имени и считается
      встречей — просто не расширяет и не пачкает галерею.

    Оба значения — 0.6/0.75 — ЗАГЛУШКА до настоящего sweep по ADR-0123
    §6 (issue #2771), не калиброванные числа. Прежний дефолт 0.45 пускал
    чужих: на живом роботе 22.09.2026 тёща владельца была опознана как
    «Деньчик» при score=0.483 (issue #2771), а запись «Деньчика» — уже
    отравленная этим же режимом (issue #2772) — после апгрейда версии
    эмбеддинга (см. ``CURRENT_EMBEDDING_VERSION``) начнёт копить галерею
    заново уже под этими порогами.

    ``embedding_version`` — версия схемы эмбеддинга, которую ожидает ЭТОТ
    инстанс (см. модульную ``CURRENT_EMBEDDING_VERSION``). Записи на
    диске с другой версией (или без поля вовсе) при загрузке теряют
    галерею, но не имя — см. ``_load_one_record``.

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
        identify_threshold: float = 0.6,
        enroll_threshold: float = 0.75,
        gallery_warmup_size: int = DEFAULT_GALLERY_WARMUP_SIZE,
        disambiguation_gap: float = DEFAULT_DISAMBIGUATION_GAP,
        max_embeddings: int = 20,
        keep_encounters: int = 10,
        max_strangers: int = 500,
        embedding_version: int = CURRENT_EMBEDDING_VERSION,
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
        self._enroll_threshold = enroll_threshold
        self._gallery_warmup_size = max(1, int(gallery_warmup_size))
        self._disambiguation_gap = max(0.0, float(disambiguation_gap))
        self._max_embeddings = max(1, int(max_embeddings))
        self._keep_encounters = max(0, int(keep_encounters))
        self._max_strangers = max(0, int(max_strangers))
        self._embedding_version = int(embedding_version)
        self._clock = clock
        self._lock = threading.Lock()
        self._records: Dict[str, _Record] = {}
        self._embedding_dim: Optional[int] = None
        # issue #2772: сколько раз встреча назвала имя (identify прошёл),
        # но эмбеддинг НЕ попал в галерею (enroll не прошёл). Живой
        # диагностический сигнал — если счётчик растёт быстро, порог
        # identify/enroll разошлись с реальными данными сильнее, чем
        # рассчитывали (или в кадре реально часто мелькают чужие).
        self._enroll_rejected_total = 0
        # issue #2771: сколько эмбеддингов дописано ПО ПРОГРЕВУ, то есть
        # по одному лишь факту узнавания, пока галерея не набрала
        # ``gallery_warmup_size``. Пара к ``enroll_rejected_total``:
        # вместе они показывают, в каком режиме живёт база. Если растёт
        # только прогрев, а отказы стоят на нуле — галереи ещё молодые;
        # если прогрев замер, а отказы растут — галереи созрели и
        # работает строгий режим #2772.
        self._enroll_warmup_total = 0
        # issue #2771: сколько раз ``_disambiguate`` отдало предпочтение
        # именованной записи вместо безымянного дубля, набравшего более
        # высокий best-of-gallery score. Диагностический аналог
        # ``enroll_rejected_total``/``enroll_warmup_total`` - если этот
        # счётчик растёт быстро, значит база копит дубли одного и того
        # же человека и по ней пора делать merge (см. face_store_admin.py
        # ``suspicious``), а не считать правило "починкой" сам по себе.
        self._disambig_named_total = 0
        self._load_from_disk()

    # ── Свойства/статистика ──────────────────────────────────────────────

    @property
    def mode(self) -> str:
        return self._mode

    def stats(self) -> Dict[str, Any]:
        """Сводка для ``/perception/health`` (ADR-0123 §2: «робот обязан
        уметь сказать, в каком он режиме»).

        ``gallery_cohesion`` — медиана per-записи ``gallery_cohesion``
        (см. ``people()``/``_gallery_cohesion``) по всем записям с ≥2
        эмбеддингами; ``None``, если таких записей нет. Это главный
        диагностический показатель отравления галереи (issue #2772,
        #2775): у живой отравленной записи «Деньчика» он был 0.302,
        должен быть ~0.9. ``enroll_rejected_total`` — счётчик встреч,
        которые назвали имя, но не расширили галерею (issue #2772,
        см. докстринг конструктора)."""
        with self._lock:
            named = sum(1 for r in self._records.values() if r.name is not None)
            strangers = len(self._records) - named
            embeddings = sum(len(r.embeddings) for r in self._records.values())
            cohesions = [
                c for c in (
                    self._gallery_cohesion(r.embeddings)
                    for r in self._records.values()
                )
                if c is not None
            ]
            gallery_cohesion = float(np.median(cohesions)) if cohesions else None
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
                'gallery_cohesion': gallery_cohesion,
                'enroll_rejected_total': self._enroll_rejected_total,
                'enroll_warmup_total': self._enroll_warmup_total,
                'disambig_named_total': self._disambig_named_total,
                'gallery_warmup_size': self._gallery_warmup_size,
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

        # issue #2772/#2773: галерея грузится с диска ТОЛЬКО если её
        # версия совпадает с текущей (``self._embedding_version``).
        # Отсутствие поля (meta.json старее, чем это поле) трактуется
        # как несовпадение, а не как «версия 1 по умолчанию» — молчаливое
        # угадывание тут опаснее честного сброса галереи.
        stored_version = data.get('embedding_version')
        version_matches = stored_version == self._embedding_version

        embeddings: List[np.ndarray] = []
        emb_path = person_dir / _EMBEDDINGS_FILENAME
        if emb_path.exists():
            arr = np.load(emb_path)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            if version_matches:
                embeddings = [row.astype(np.float32) for row in arr]
                if embeddings and self._embedding_dim is None:
                    self._embedding_dim = int(embeddings[0].size)
            elif arr.shape[0] > 0:
                # Имя/speaker_id/эталон/встречи — сохраняются ниже как
                # есть, это по-прежнему тот же человек. Только галерея
                # эмбеддингов отбрасывается: сравнивать вектора из
                # разных версий препроцессинга молча нельзя (см.
                # CURRENT_EMBEDDING_VERSION). Именно так после апгрейда
                # на выравнивание по landmark'ам (#2773) самоочистится
                # отравленная живая галерея «Деньчика» (#2772).
                logger.warning(
                    'face_store: person_id=%s embedding_version на диске=%r, '
                    'текущая=%d (issue #2772/#2773) — %d эмбеддингов '
                    'отброшены, галерея начнётся заново; имя/speaker_id/'
                    'эталон/встречи сохранены',
                    person_id, stored_version, self._embedding_version,
                    arr.shape[0],
                )

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
            # Начиная с этой загрузки запись живёт под версией ЭТОГО
            # инстанса: либо галерея реально ей соответствует
            # (version_matches), либо она пуста и следующий же
            # ``record_encounter`` засеет её заново под текущей версией.
            embedding_version=self._embedding_version,
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
        встреча не должна размывать уже подтверждённое совпадение).

        Намеренно НЕ переведено на медоид/медиану в этой карточке —
        issue #2772 просит развести пороги и вытеснение, а смена самого
        скоринга (max → медоид) обсуждается отдельно, с sweep по
        реальным данным (issue #2771 «Счёт по медоиду/медиане, а не по
        max — обсуждаемо»). Разводить обе смены в одном PR — терять
        возможность откатить одну независимо от другой."""
        scored = []
        for person_id, rec in self._records.items():
            if not rec.embeddings:
                continue
            best = max(_cosine(embedding, e) for e in rec.embeddings)
            scored.append((person_id, best))
        scored.sort(key=lambda t: -t[1])
        return scored

    def _gallery_cross_similarity(
        self, a: '_Record', b: '_Record',
    ) -> Optional[float]:
        """Кросс-сходство ГАЛЕРЕЙ двух записей - максимум косинуса по всем
        парам (вектор из ``a.embeddings``, вектор из ``b.embeddings``), тот
        же приём ``max``, что ``_score_all``/докстринг там же: одна удачно
        совпавшая пара ракурсов - уже достаточное свидетельство, что две
        записи "слипаются" и, вероятно, описывают одного человека, а не
        просто оказались случайно похожи в один конкретный момент.

        ``None``, если у любой из записей нет ни одного эмбеддинга -
        сравнивать не с чем (тот же принцип, что ``_gallery_cohesion``:
        отсутствие измерения - не 0.0, а именно ``None``).

        Используется ТОЛЬКО ``_disambiguate`` (issue #2771) как вторая,
        независимая от текущей встречи проверка "это правда дубль", а не
        совпадение по одному неудачному кадру - см. докстринг
        ``DEFAULT_DISAMBIGUATION_GAP`` про остаточный риск без неё.
        """
        if not a.embeddings or not b.embeddings:
            return None
        return max(_cosine(x, y) for x in a.embeddings for y in b.embeddings)

    def _disambiguate(self, scored: List[tuple]) -> tuple:
        """issue #2771 - разрешить неоднозначность «безымянный дубль
        обошёл именованную запись по max score» (см. докстринг
        ``DEFAULT_DISAMBIGUATION_GAP`` выше - живой сценарий «Дэнчик»/
        «a659ddab»).

        ``scored`` - результат ``_score_all`` (отсортирован по убыванию).
        Возвращает ``(index, disambiguated)``: ``index`` - позиция в
        ``scored``, которую следует считать победителем;
        ``disambiguated=True``, только если победитель СМЕНИЛСЯ с
        безымянного (позиция 0) на именованного кандидата.

        Правило срабатывает, только если ВЫПОЛНЕНЫ ВСЕ условия сразу:

        1. Лучший скор (``scored[0]``) принадлежит записи БЕЗ имени -
           если топ уже именован, менять нечего, правило не трогает
           обычное узнавание вообще.
        2. Есть именованный кандидат, чей score сам по себе проходит
           ``identify_threshold`` (не только «был бы похож, если бы не
           дубль» - он обязан быть похож независимо).
        3. Зазор между лучшим (безымянным) и этим именованным кандидатом
           не превышает ``disambiguation_gap``.
        4. Галереи безымянного и именованного кандидата «слипаются» -
           ``_gallery_cross_similarity`` между ними сама проходит
           ``identify_threshold``. Это условие отделяет настоящий дубль
           («тот же человек, две записи») от совпадения по одной
           случайной встрече: без него можно было бы по ошибке присвоить
           имя человеку, который просто один раз оказался похож сразу на
           обе записи, не будучи похож на именованную запись вообще
           (риск описан честно в PR - при МАЛЫХ галереях с 1-2 векторами
           это условие мало что фильтрует, полноценная защита появится
           вместе со sweep из ADR-0123 §6).

        Среди нескольких именованных кандидатов, прошедших (2)-(4),
        выбирается тот, что стоит РАНЬШЕ в ``scored`` (то есть с более
        высоким score) - естественный порядок, `scored` уже отсортирован.
        """
        if not scored:
            return 0, False
        best_id, best_sim = scored[0]
        if self._records[best_id].name is not None:
            return 0, False
        best_rec = self._records[best_id]
        for idx in range(1, len(scored)):
            person_id, sim = scored[idx]
            if sim < self._identify_threshold:
                # scored отсортирован по убыванию - дальше будет только хуже.
                break
            rec = self._records[person_id]
            if rec.name is None:
                continue
            if best_sim - sim > self._disambiguation_gap:
                continue
            cross = self._gallery_cross_similarity(best_rec, rec)
            if cross is None or cross < self._identify_threshold:
                continue
            return idx, True
        return 0, False

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
            idx, disambiguated = self._disambiguate(scored)
            if idx == 0:
                person_id, sim = scored[0]
                runner_up_person_id, runner_up_similarity = (
                    scored[1] if len(scored) > 1 else (None, None)
                )
            else:
                # issue #2771: безымянный дубль (scored[0]) обошёл
                # именованную запись только по max score - runner-up
                # теперь честно показывает ИМЕННО этот бывший "победитель",
                # а не scored[1] (см. докстринг _disambiguate/FaceMatch).
                person_id, sim = scored[idx]
                runner_up_person_id, runner_up_similarity = scored[0]
            rec = self._records[person_id]
            return FaceMatch(
                person_id=person_id,
                name=rec.name,
                similarity=sim,
                is_new=False,
                encounter_count=rec.encounter_count,
                runner_up_person_id=runner_up_person_id,
                runner_up_similarity=runner_up_similarity,
                disambiguated=disambiguated,
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

            disambiguated = False
            if scored and scored[0][1] >= self._identify_threshold:
                idx, disambiguated = self._disambiguate(scored)
                if idx == 0:
                    person_id, sim = scored[0]
                    runner_up_person_id, runner_up_similarity = (
                        scored[1] if len(scored) > 1 else (None, None)
                    )
                else:
                    # issue #2771: не растим и не пачкаем галерею
                    # безымянного дубля (scored[0]) - вся дальнейшая
                    # логика (encounter_count/_should_enroll/снимки)
                    # ниже работает с ИМЕНОВАННОЙ записью, выбранной
                    # _disambiguate. Дубль остаётся как есть: следующая
                    # встреча снова попробует его же, и если человек
                    # опять не пройдёт disambiguation - дубль по-прежнему
                    # доступен для ручного merge (face_store_admin.py).
                    person_id, sim = scored[idx]
                    runner_up_person_id, runner_up_similarity = scored[0]
                    self._disambig_named_total += 1
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
                rec = _Record(
                    person_id=person_id, created_at=now,
                    embedding_version=self._embedding_version,
                )
                rec.persisted = self._mode == MODE_WORKSHOP
                self._records[person_id] = rec
                is_new = True
                sim = 1.0
                # Не «нет второго кандидата» — лучший ИЗВЕСТНЫЙ, пусть и
                # не прошедший identify_threshold. Диагностически ценно:
                # именно так выглядела бы предупреждающая строка «чуть не
                # ложное отклонение» (issue #2771).
                runner_up_person_id, runner_up_similarity = (
                    scored[0] if scored else (None, None)
                )
                if rec.persisted:
                    self._person_dir(person_id).mkdir(parents=True, exist_ok=True)

            rec.encounter_count += 1
            rec.last_encounter_ts = now

            if self._mode == MODE_STRICT:
                # §4.1: у знакомых в strict — один центроид, не галерея.
                self._fold_into_centroid(rec, emb)
            elif is_new:
                # Первый эмбеддинг новой записи — семя галереи, кладётся
                # безусловно (сравнивать не с чем, см. докстринг FaceMatch).
                rec.embeddings.append(emb)
                self._evict_most_distant_from_medoid(rec)
            elif self._should_enroll(rec, emb, sim):
                rec.embeddings.append(emb)
                self._evict_most_distant_from_medoid(rec)
            else:
                # issue #2772: identify прошёл (имя будет названо и встреча
                # засчитана ниже), а enroll — нет. Галерея НЕ растёт и НЕ
                # пачкается этим эмбеддингом.
                self._enroll_rejected_total += 1

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
                runner_up_person_id=runner_up_person_id,
                runner_up_similarity=runner_up_similarity,
                disambiguated=disambiguated,
            )

    # ── Галерея эмбеддингов ──────────────────────────────────────────────

    def _medoid(self, embeddings: List[np.ndarray]) -> np.ndarray:
        """Эмбеддинг галереи с максимальной суммой косинусных сходств к
        остальным — канонический, наиболее «согласованный со всеми»
        ракурс. В отличие от геометрического центроида (среднего вектора,
        который может не совпадать ни с одним реальным эмбеддингом),
        медоид — это конкретная, когда-то реально записанная встреча,
        поэтому годится и как точка допуска в галерею (``_should_enroll``),
        и как якорь при вытеснении (``_evict_most_distant_from_medoid``).
        Для галереи из одного элемента медоид — он сам."""
        if len(embeddings) == 1:
            return embeddings[0]
        n = len(embeddings)
        sums = [
            sum(_cosine(embeddings[i], embeddings[j]) for j in range(n) if j != i)
            for i in range(n)
        ]
        best_idx = max(range(n), key=lambda i: sums[i])
        return embeddings[best_idx]

    def _gallery_cohesion(self, embeddings: List[np.ndarray]) -> Optional[float]:
        """Медиана попарного косинуса внутри галереи — единственный
        числовой показатель «эта запись описывает одного человека, а не
        нескольких» (issue #2772, #2775 — нужен и для ``/perception/health``,
        и для инструмента чистки). Здоровая запись держится в районе
        ~0.9; живая отравленная галерея «Деньчика» (#2772, до фикса) —
        медиана 0.302, min −0.007. Для 0-1 эмбеддингов попарных пар нет
        — ``None``, а не 0.0/1.0 (оба значения были бы враньём об
        измерении, которого не существует)."""
        n = len(embeddings)
        if n < 2:
            return None
        sims = [
            _cosine(embeddings[i], embeddings[j])
            for i in range(n)
            for j in range(i + 1, n)
        ]
        return float(np.median(sims))

    def _should_enroll(self, rec: _Record, emb: np.ndarray, best_score: float) -> bool:
        """issue #2772: дописывать эмбеддинг в СУЩЕСТВУЮЩУЮ галерею только
        при двойном согласии — одного порога мало.

        1. ``best_score`` (уже посчитан в ``_score_all``, тот же
           best-of-gallery score, что дал совпадение identify) обязан
           пройти ``enroll_threshold`` — заметно строже
           ``identify_threshold``.
        2. Новый вектор обязан быть похож на МЕДОИД галереи (канонический
           ракурс), а не просто оказаться чуть ближе к чьему-то одному
           случайно затесавшемуся туда вектору. Проверка только по (1)
           именно так и ломалась на живых данных: best-of-gallery
           («максимум по галерее») можно превысить, попав рядом с уже
           присутствующим мусорным эмбеддингом — так галерея «Деньчика»
           сама себя убедила дописать тёщу поверх собственного мусора.

        Первый эмбеддинг НОВОЙ записи сюда не попадает вовсе — он сеется
        безусловно, см. вызывающий код в ``record_encounter``.

        ПРОГРЕВ ГАЛЕРЕИ (issue #2771, живой замер 22.09.2026)
        -----------------------------------------------------
        Двойное согласие выше защищает ЗРЕЛУЮ галерею, но молодую оно
        убивает: пока в записи один вектор, «медоид» — это он сам, и
        оба условия вырождаются в одно и то же сравнение с
        единственным ракурсом. Замер на роботе: три записи одного
        человека, снятые за пять минут, дали попарные косинусы 0.538
        (анфас против опущенной головы), 0.582 (анфас против тёмных
        очков) и 0.461. Весь внутриперсонный разброс лёг НИЖЕ
        ``enroll_threshold`` (0.75), поэтому ``_should_enroll``
        отказывал всегда, галерея у всех трёх записей осталась
        размером 1 — и следующий ракурс не дотягивал уже до
        ``identify_threshold``, заводя очередного «нового человека».
        Петля замыкалась сама на себе: галерея из одного вектора →
        промах → новая запись → снова галерея из одного вектора.

        Поэтому пока галерея меньше ``gallery_warmup_size``, дозапись
        идёт по факту УЗНАВАНИЯ, без второго, более строгого порога.
        Это не ослабление защиты от отравления, а тот же приём, что уже
        принят в этом репозитории для голоса (``speaker_embeddings``:
        галерея растёт по якорю сессии, а не по косинусу): доверяем
        решению, которое уже принято — ``record_encounter`` зовёт
        ``_should_enroll`` ТОЛЬКО после того, как ``best_score`` прошёл
        ``identify_threshold``. Отравить запись этим можно ровно в той
        же мере, в какой можно ошибиться самим узнаванием, — нового
        класса ошибок прогрев не добавляет, а вот выйти из петли
        позволяет. Как только галерея набрала ``gallery_warmup_size``
        векторов, двойное согласие возвращается в полную силу и дальше
        работает как раньше (issue #2772).
        """
        if len(rec.embeddings) < self._gallery_warmup_size:
            self._enroll_warmup_total += 1
            return True
        if best_score < self._enroll_threshold:
            return False
        medoid = self._medoid(rec.embeddings)
        return _cosine(emb, medoid) >= self._enroll_threshold

    def _evict_most_distant_from_medoid(self, rec: _Record) -> None:
        """ADR-0123 §4.1, issue #2772: при переполнении ``max_embeddings``
        вытесняется вектор, ДАЛЬШЕ ВСЕХ от медоида галереи (эмбеддинг с
        максимальной суммой сходств к остальным) — не самый похожий на
        остальные, и не самый старый.

        Раньше (см. историю метода — ``_evict_most_redundant``)
        вытеснялся САМЫЙ ТИПИЧНЫЙ эмбеддинг под лозунгом «несёт меньше
        всего новой информации». Для датасета, который должен покрывать
        разнообразие, это разумная цель. Галерея личности — не датасет,
        а эталон ОДНОГО человека: цель здесь — ЧИСТОТА личности, а не
        разнообразие ракурсов. Многократно подтверждённый, типичный
        ракурс — это якорь, который держит запись рядом с её настоящим
        владельцем; выброс (случайный чужой кадр, неудачный угол,
        промах трекера) — и есть та примесь, которая размывает запись.
        Старое правило систематически сносило якорь и берегло примесь:
        живая галерея «Деньчика» (issue #2772) при ``max_embeddings=20``
        накопила внутренний попарный косинус медиана 0.302 (min −0.007)
        вместо ожидаемых ~0.9 — потому что именно типичные,
        подтверждающие личность векторы вымывались первыми, а случайные
        примеси оставались и множились."""
        while len(rec.embeddings) > self._max_embeddings:
            medoid = self._medoid(rec.embeddings)
            n = len(rec.embeddings)
            worst_idx = min(
                range(n), key=lambda i: _cosine(rec.embeddings[i], medoid)
            )
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
            'embedding_version': rec.embedding_version,
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
                self._evict_most_distant_from_medoid(keep)

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
                    # issue #2772/#2775: главный диагностический показатель
                    # здоровья галереи, см. докстринг _gallery_cohesion.
                    'gallery_cohesion': self._gallery_cohesion(rec.embeddings),
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
    'CURRENT_EMBEDDING_VERSION',
    'FaceMatch',
    'FaceStore',
]
