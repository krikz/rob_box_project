#!/usr/bin/env python3
"""face_store_admin.py — CLI поверх публичного API ``FaceStore`` для ремонта
испорченных лицевых записей (issue #2775, ADR-0123).

Почему этот скрипт вообще нужен
--------------------------------
На живом роботе в ``/data/faces`` бывают испорченные записи: фантом из
десятков встреч чёрного кадра (issue #2749) или запись, чья галерея
незаметно смешала несколько разных людей (см. issue #2775 — «Деньчик»,
три человека в одной галерее). У ``FaceStore`` есть ``forget()`` и
``merge()``, но они живут внутри процесса ноды ``vision_face`` и наружу
не выведены — единственный доступный сейчас способ починки это ``ssh`` +
``rm -rf`` по каталогу записи. Это уже один раз привело к беде на другой
БД этого репозитория (issue #2750 — стёрли живой профиль диктора ручной
командой), и вдобавок ``rm -rf`` рвёт связку «лицо ↔ голос» (``name`` +
``speaker_id``) и выбрасывает ``reference.jpg``, по которому запись можно
было бы пересобрать. Этот скрипт — не новая логика поверх ``FaceStore``,
а вывод наружу того, что уже есть (``forget``), плюс две операции,
которых в публичном API ``FaceStore`` нет и которые поэтому работают
через прямую, документированную on-disk-раскладку модуля (``meta.json``,
``embeddings.npy``, ``reference.jpg``, ``encounters/*.jpg`` — формат,
который сам ``face_store.py`` пишет в ``_persist_record``/
``_maybe_store_snapshots``; см. docstring там же).

Подкоманды
----------
``list``
    Таблица: ``person_id``, имя, ``speaker_id``, число встреч, размер
    галереи и **внутренний косинус галереи** (медиана/min попарных
    косинусов эмбеддингов). Это главный диагностический показатель:
    здоровая запись ~0.9, отравленная (несколько разных людей) ~0.3,
    фантом (много почти одинаковых кадров чёрного экрана) подозрительно
    ровные ~0.95. ``FaceStore.people()`` к моменту написания этого файла
    уже отдаёт готовую МЕДИАНУ (``gallery_cohesion``, float, issue
    #2772/#2775) — она переиспользуется как есть, не пересчитывается;
    ``min`` в публичном API не появился, поэтому досчитывается здесь же
    через ``FaceStore.gallery()`` (см. ``_person_gallery_cohesion``
    ниже — там же обработан и случай, когда апстрим ещё не даёт
    ``gallery_cohesion`` вовсе, тогда медиана тоже считается на месте).

``forget <person_id>``
    Удаляет запись целиком через ``FaceStore.forget()`` — НЕ ``rm -rf``.
    Разрушительно, необратимо.

``reset-gallery <person_id>``
    Основной сценарий после инцидента с отравленной галереей: сносит
    ``embeddings.npy`` и ``encounters/`` (снимки встреч), но СОХРАНЯЕТ
    ``name``, ``speaker_id`` и ``reference.jpg`` — человек переучивается
    с чистого листа, не теряя связку с голосовым профилем. Публичного
    метода ``FaceStore`` для частичного сброса нет (``forget`` стирает
    всё, включая имя и эталонный снимок), поэтому здесь — прямая правка
    ``meta.json``/удаление файлов по документированной раскладке.

``rebuild <person_id>``
    Пересчитывает галерею эмбеддингов из сохранённых на диске снимков
    (``reference.jpg`` + ``encounters/*_face.jpg``) ТЕКУЩИМ эмбеддером.
    Это же и будущая миграция при смене ``embedding_version`` (см. текст
    issue #2775). Эмбеддер — ArcFace на Hailo (``rob_box_perception.
    face_embedding.ArcFaceEmbedder``) — существует только внутри
    контейнера ``vision-face`` (нужны ``hailo_platform`` и ``cv2``).
    Основная логика вынесена в свободную функцию ``rebuild_gallery()``,
    которая принимает эмбеддер и загрузчик изображений параметрами —
    её можно протестировать с подставным эмбеддером, без Hailo и без
    cv2 (см. ``test_face_store_admin.py``). При запуске из CLI без
    железа/cv2 — понятная ошибка и ненулевой exit code, а не traceback.

ВАЖНАЯ ОГОВОРКА — живая нода держит /data/faces в памяти
-----------------------------------------------------------
Нода ``vision_face_node`` (``rob_box_perception.vision_face_node``,
entry point ``vision_face``) грузит ``/data/faces`` в память при
старте (``FaceStore.__init__`` → ``_load_from_disk``) и переписывает
``meta.json``/``embeddings.npy`` конкретной записи при следующей же
встрече этого человека (``record_encounter`` → ``_persist_record``).
Значит: правка диска ЭТИМ скриптом, пока нода работает, будет молча
затёрта при первой же новой встрече — самый честный вариант был бы
починить это сервисом внутри ноды, но issue #2775 сознательно откладывает
это отдельной карточкой и выбирает CLI. Поэтому каждая разрушительная
подкоманда (``forget``/``reset-gallery``/``rebuild --apply``) ПЕРЕД
записью на диск проверяет, не запущен ли процесс ноды
(``pgrep -f vision_face_node`` внутри контейнера — тот же PID namespace,
что и у ``docker exec``), и ОТКАЗЫВАЕТСЯ работать без ``--force``, если
процесс похоже запущен или это не удалось проверить (например, нет
``pgrep`` в окружении — тогда это тоже отказ, не тихое «наверное, ок»).
``--force`` не выключает эту проверку молча — он явный флаг «я убедился
сам». Скрипт не умеет и не пытается сам остановить ноду — это отдельное
действие оператора (``docker exec vision-face pkill -f vision_face_node``
или временный ``docker stop``/уменьшение реплик, смотри по обстоятельствам
на роботе).

Как это запускают на Vision Pi
-------------------------------
::

    docker exec vision-face bash -c \
        "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
         python3 /tmp/face_store_admin.py list"

Команда ОБЯЗАНА быть полной, с ОБОИМИ ``source`` — без них
``rob_box_perception`` не найден ВООБЩЕ, независимо от подкоманды: живой
прогон агента (issue #2775), доставившего файл голым ``docker cp`` и
запустившего голым ``python3 /tmp/face_store_admin.py list`` без
``source``, упал с ``ModuleNotFoundError: No module named
'rob_box_perception'`` ровно по этой причине — это не баг скрипта, а
недостающий шаг в инструкции по запуску (исправлено здесь же).

Путь ВНУТРИ контейнера и путь НА ХОСТЕ — разные вещи:

* на Vision Pi (хост) репозиторий обычно лежит в
  ``~/rob_box_project``, и этот файл — по пути ``~/rob_box_project/
  scripts/maintenance/face_store_admin.py``, но в образ контейнера
  ``vision-face`` содержимое ``scripts/`` НЕ копируется (Dockerfile
  копирует только ``src/rob_box_perception`` через colcon build, см.
  ``docker/vision/vision-hailo/Dockerfile``);
* значит, чтобы запустить это ВНУТРИ контейнера, файл нужно либо
  смонтировать волюмом, либо скопировать внутрь разовым
  ``docker cp face_store_admin.py vision-face:/tmp/face_store_admin.py``
  и запускать оттуда — сам скрипт от этого не страдает (см. ниже,
  почему он вообще не требует ``rob_box_perception`` быть pip-installed).
  ВАЖНО (issue #2775, живой прогон агента 22.09.2026): скопированный в
  ``/tmp`` файл лежит НЕ на той глубине вложенности, что в чекауте
  репозитория (``<repo>/scripts/maintenance/<файл>``) — импорт НИКОГДА
  не вычисляет путь к пакету из глубины вложенности ЭТОГО файла
  заранее/безусловно именно из-за этого: сперва всегда пробуется обычный
  ``import rob_box_perception.face_store`` (который срабатывает сам,
  если ``source`` выполнен — пакет уже на ``PYTHONPATH``), и только если
  он не сработал, скрипт пытается достроить путь от чекаута — а если и
  это невозможно (нет ``git``-чекаута рядом, как для копии в ``/tmp``),
  честно поднимается обычный ``ModuleNotFoundError``, а не
  ``IndexError`` (см. ``_guess_pkg_root``/``_import_face_store`` ниже);
* ROS-окружение ВНУТРИ контейнера ``vision-face`` живёт в
  ``/ws/install`` (см. ``docker/vision/scripts/vision-hailo/
  start_vision_face.sh``: ``source /ws/install/setup.bash``), а НЕ в
  ``/ros2_ws/install`` — это другой контейнер/другой сервис в этом
  репозитории, не путать при отладке ``PYTHONPATH``. ``list``/
  ``forget``/``reset-gallery`` этого скрипта не трогают ROS-РАНТАЙМ
  (``rclpy``/Hailo) — но САМ ПАКЕТ ``rob_box_perception`` в контейнере
  всё равно лежит только в ``/ws/install`` и без ``source`` не виден
  вообще, для ЛЮБОЙ подкоманды, включая ``list``. ``/ws/install``
  дополнительно важен для ``rebuild``, которому нужен ещё и
  ``rob_box_perception.face_embedding`` (Hailo).
* том с данными внутри контейнера смонтирован как ``/data/faces``
  (``docker/vision/docker-compose.yaml``: ``./data/faces:/data/faces``
  относительно каталога ``docker/vision``) — то есть на хосте это
  ``~/rob_box_project/docker/vision/data/faces``. ``--root`` по
  умолчанию берётся из ``FACE_STORE_ROOT`` (тот же ENV, что видит сама
  нода, см. ``hailo_models.yaml``/``start_vision_face.sh``) или, если
  переменной нет, ``/data/faces`` — то есть дефолт уже правильный
  ВНУТРИ контейнера и неправильный, если запускать этот файл прямо на
  хосте без ``--root``.

Тестируемость без ROS/cv2/Hailo
---------------------------------
Модуль сознательно копирует приём ``face_store.py``/
``test_face_store.py``: импортирует ``rob_box_perception.face_store``
через ``sys.path.insert`` на ``src/rob_box_perception`` вместо обычного
``import rob_box_perception`` (пакет не pip-installed вне ROS-окружения),
и НЕ импортирует ``cv2``/``rob_box_perception.face_embedding`` на уровне
модуля — только лениво, внутри ``_default_image_loader``/
``_default_embedder_factory``, вызываемых исключительно из
CLI-обвязки ``rebuild``, а не из тестируемой функции ``rebuild_gallery``.
Запуск тестов:

::

    python -m pytest src/rob_box_perception/test/unit/test_face_store_admin.py -q --no-cov

Честность (AGENTS.md: «честный FAIL лучше красивого PASS»)
-------------------------------------------------------------
Этот файл писался и тестировался ТОЛЬКО на синтетических данных во
временном каталоге (``tmp_path``) в рамках issue #2775. На живом
роботе (Vision Pi, реальный ``/data/faces``, реальная нода
``vision_face``, реальный Hailo) он НЕ запускался и НЕ проверялся —
ни одна подкоманда, включая ``list``. См. отчёт агента в PR.
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# Импорт FaceStore без установки пакета (тот же приём, что
# test_face_store.py — см. его комментарий "см. test_gaze_seam.py — тот же
# приём"). НЕ импортируем ROS/cv2/Hailo здесь — face_store.py сам по себе
# на них не завязан (см. его модульный docstring), значит и этот файл не
# завязан транзитивно.
#
# ВАЖНО (issue #2775, живой прогон агента 22.09.2026): раньше здесь БЕЗУСЛОВНО
# вычислялся ``_HERE.parents[2]`` как корень репозитория — предполагая, что
# файл лежит РОВНО по пути ``<repo>/scripts/maintenance/face_store_admin.py``.
# При доставке документированным ``docker cp .../tmp/face_store_admin.py``
# (см. докстринг модуля выше) у файла в ``/tmp`` нет двух предков-каталогов —
# ``parents[2]`` кидал ``IndexError`` ДО разбора argparse, то есть падение
# происходило даже на ``--help``. Порядок исправлен на обратный: СНАЧАЛА
# пробуем обычный импорт (внутри контейнера ``rob_box_perception`` уже виден
# на ``PYTHONPATH`` после ``source /opt/ros/humble/setup.bash && source
# /ws/install/setup.bash`` — sys.path вообще не нужен), и ТОЛЬКО если он не
# сработал, пытаемся достроить путь от расположения этого файла — а если
# посчитать этот путь невозможно (как для копии в ``/tmp`` — предков не
# хватает), просто пропускаем эту попытку и даём обычному импорту поднять
# честный ``ModuleNotFoundError`` вместо ``IndexError``.
# ---------------------------------------------------------------------------
_HERE = Path(__file__).resolve()


def _guess_pkg_root(here: Path, pkg_dir_name: str) -> Optional[Path]:
    """``<repo_root>/src/<pkg_dir_name>``, ЕСЛИ ``here`` лежит по своему
    обычному пути ``<repo>/scripts/maintenance/<файл>`` (то есть ровно на
    два уровня вложенности под корнем репозитория). Возвращает ``None``
    вместо ``IndexError``, если предков не хватает (файл доставлен отдельно
    от чекаута, например ``docker cp`` в ``/tmp`` — см. большой комментарий
    выше). Чистая функция (не трогает sys.path) — специально ради теста
    без реального копирования файла на диск, см. test_face_store_admin.py.
    """
    parents = here.parents
    if len(parents) <= 2:
        return None
    return parents[2] / 'src' / pkg_dir_name


def _import_face_store():
    """Импортировать ``rob_box_perception.face_store``.

    Порядок вызовов — суть исправления issue #2775: обычный импорт ПЕРВЫЙ
    (работает без вычисления вообще каких-либо путей, если пакет уже на
    ``PYTHONPATH`` — ровно так внутри контейнера после ``source``, см.
    докстринг модуля), достройка ``sys.path`` от расположения ЭТОГО файла —
    только запасной вариант для локального запуска из чекаута репозитория
    без ROS вообще (тесты, разработка). Если даже это невозможно (путь не
    вычислить — см. ``_guess_pkg_root``) или пакета всё равно нет — наружу
    уходит обычный ``ModuleNotFoundError`` от финального ``import_module``,
    а не самодельная ошибка: сообщение Python'а само по себе достаточно
    понятно ("No module named 'rob_box_perception'") и не маскирует причину.
    """
    try:
        return importlib.import_module('rob_box_perception.face_store')
    except ModuleNotFoundError:
        pass
    pkg_root = _guess_pkg_root(_HERE, 'rob_box_perception')
    if pkg_root is not None and str(pkg_root) not in sys.path:
        sys.path.insert(0, str(pkg_root))
    return importlib.import_module('rob_box_perception.face_store')


fs = _import_face_store()
FaceStore = fs.FaceStore
DEFAULT_ROOT = fs.DEFAULT_ROOT

# Имена файлов — те же константы, что face_store.py использует внутри
# (``_META_FILENAME`` и т.д. там приватные, поэтому продублированы здесь;
# это НЕ дублирование логики, это имена частей документированного формата
# на диске, см. docstring face_store.py §4 и docstring этого файла выше).
_META_FILENAME = 'meta.json'
_EMBEDDINGS_FILENAME = 'embeddings.npy'
_REFERENCE_FILENAME = 'reference.jpg'
_ENCOUNTERS_DIRNAME = 'encounters'

#: Паттерн для ``pgrep -f``, которым ищем работающий процесс ноды
#: vision_face (entry point ``vision_face = rob_box_perception.
#: vision_face_node:main`` в setup.py — ros2 run/launch порождает процесс
#: с этим именем в аргументах командной строки).
DEFAULT_NODE_PROCESS_PATTERN = 'vision_face_node'

#: ArcFace HEF по умолчанию — тот же путь, что face_embedding.py
#: (``DEFAULT_ARCFACE_HEF``), продублирован как строка, чтобы не тянуть
#: face_embedding на уровне модуля (см. докстринг выше — импорт только
#: лениво внутри rebuild).
DEFAULT_ARCFACE_HEF = '/opt/rob_box/models/arcface_mobilefacenet.hef'


# =============================================================================
# Мелкие файловые хелперы (без побочных эффектов, кроме отмеченных явно)
# =============================================================================

def _person_dir(root: str, person_id: str) -> Path:
    return Path(root) / person_id


def _dir_size(path: Path) -> int:
    """Суммарный размер файлов в каталоге (тот же приём, что
    ``FaceStore.stats()`` использует для ``disk_bytes``, но по одному
    человеку, а не по всему root)."""
    if not path.exists():
        return 0
    total = 0
    for p in path.rglob('*'):
        if p.is_file():
            try:
                total += p.stat().st_size
            except OSError:
                continue
    return total


def _read_meta(person_dir: Path) -> Dict[str, Any]:
    """Прочитать ``meta.json`` напрямую. ``FaceStore.people()`` не отдаёт
    ``speaker_id`` (не входит в публичный контракт, см. face_store.py
    ``people()``), а он нужен для ``list`` — поэтому читаем файл, а не
    придумываем API поверх FaceStore. Возвращает ``{}``, если файла нет
    или он битый (та же терпимость к повреждённым записям, что
    ``FaceStore._load_from_disk`` — не должно ронять весь ``list``)."""
    meta_path = person_dir / _META_FILENAME
    if not meta_path.exists():
        return {}
    try:
        with open(meta_path, 'r', encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, json.JSONDecodeError):
        return {}


def _atomic_write_json(path: Path, obj: Dict[str, Any]) -> None:
    """Тот же приём атомарной записи, что ``face_store._atomic_write_json``
    (temp-файл + ``os.replace``) — переиспользуем идею, а не приватную
    функцию модуля (не хотим завязываться на internals face_store.py,
    которые могут поменяться без объявления в публичном API)."""
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


# =============================================================================
# Косинус галереи — диагностика "здорова / отравлена / фантом"
# =============================================================================

def _cosine(a: np.ndarray, b: np.ndarray) -> float:
    an = np.linalg.norm(a)
    bn = np.linalg.norm(b)
    if an < 1e-9 or bn < 1e-9:
        return 0.0
    return float(np.dot(a, b) / (an * bn))


def _gallery_cohesion(embeddings: Sequence[np.ndarray]) -> Optional[Dict[str, float]]:
    """Медиана и минимум попарных косинусов внутри галереи одного
    человека. ``None``, если в галерее меньше двух векторов — попарного
    косинуса тогда просто не существует (не «0», не «неизвестно» — само
    понятие неприменимо, поэтому явный ``None``, а не magic value).

    Эталонные диапазоны из issue #2775 (замерено на живых испорченных
    записях робота, не выдумано): здоровая запись ~0.9, отравленная
    (галерея смешала разных людей) ~0.3, фантом (десятки почти
    одинаковых кадров чёрного экрана) подозрительно ровные ~0.95 —
    отличить фантом от здоровой записи по одной медиане нельзя, для
    этого в таблице ``list`` есть отдельная колонка числа встреч
    (фантом — сотни встреч почти без роста галереи).
    """
    n = len(embeddings)
    if n < 2:
        return None
    sims: List[float] = []
    for i in range(n):
        for j in range(i + 1, n):
            sims.append(_cosine(embeddings[i], embeddings[j]))
    arr = np.asarray(sims, dtype=np.float64)
    return {'median': float(np.median(arr)), 'min': float(np.min(arr)), 'pairs': len(sims)}


def _person_gallery_cohesion(
    store: FaceStore, person_summary: Dict[str, Any], person_id: str
) -> Optional[Dict[str, float]]:
    """Медиана + min попарных косинусов галереи для строки ``list``.

    Параллельная карточка (issue #2772/#2775) уже приземлила
    ``gallery_cohesion`` в ``FaceStore.people()``/``stats()`` — но ТОЛЬКО
    медиану, одним float'ом (``face_store.py``, ``_gallery_cohesion``).
    Issue #2775 явно просит и медиану, и min ("медиана/min попарных
    косинусов"), а min в публичном API не появился — поэтому здесь не
    дублируем подсчёт медианы (переиспользуем готовое значение из
    ``person_summary['gallery_cohesion']``, если оно есть и не ``None``),
    но досчитываем min сами через публичный ``FaceStore.gallery()``
    (тот же список векторов, который использовало само ``FaceStore``,
    чтобы посчитать свою медиану — не рассинхронизируется). Если поле
    в ``people()`` ещё не появится (старая версия face_store.py) —
    считаем медиану тоже сами, по тем же ``sims``."""
    embeddings = store.gallery(person_id)
    n = len(embeddings)
    if n < 2:
        return None
    sims = [_cosine(embeddings[i], embeddings[j]) for i in range(n) for j in range(i + 1, n)]
    upstream_median = person_summary.get('gallery_cohesion')
    median = float(upstream_median) if upstream_median is not None else float(np.median(sims))
    return {'median': median, 'min': float(np.min(sims)), 'pairs': len(sims)}


# =============================================================================
# Проверка "не запущена ли нода" (обязательный гейт перед любой записью)
# =============================================================================

def _node_running_via_pgrep(pattern: str) -> Optional[bool]:
    """``True``/``False``, если ``pgrep -f <pattern>`` удалось выполнить;
    ``None``, если проверить не удалось вовсе (нет ``pgrep`` в PATH —
    например, локальный запуск не в контейнере, или Windows-хост
    разработчика). ``None`` — не «наверное, не запущена», а «не знаю»;
    вызывающий код обязан требовать ``--force`` и в этом случае тоже
    (см. ``_ensure_node_not_running``)."""
    try:
        result = subprocess.run(
            ['pgrep', '-f', pattern],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5,
        )
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired):
        return None
    return result.returncode == 0


def _ensure_node_not_running(
    force: bool,
    node_process_pattern: str,
    node_check: Optional[Callable[[str], Optional[bool]]] = None,
) -> Optional[str]:
    """Вернуть текст ошибки, если писать на диск сейчас небезопасно,
    иначе ``None``. ``node_check`` — параметр специально ради тестов
    (см. test_face_store_admin.py): можно подставить фейковую проверку
    и не полагаться на реальный ``pgrep``/процессы.

    ``None`` (дефолт) читает модульный ``_node_running_via_pgrep`` ПРЯМО
    ЗДЕСЬ, при каждом вызове, а не как значение параметра по умолчанию
    в сигнатуре — иначе тесты не смогли бы подменить его через
    ``monkeypatch.setattr(admin, '_node_running_via_pgrep', ...)``
    (default-значения в Python связываются один раз при определении
    функции, а не при каждом вызове)."""
    if force:
        return None
    if node_check is None:
        node_check = _node_running_via_pgrep
    running = node_check(node_process_pattern)
    if running is True:
        return (
            f"нода (процесс, подходящий под 'pgrep -f {node_process_pattern}') "
            "похоже запущена. Она держит /data/faces в памяти и перепишет "
            "meta.json/embeddings.npy этой записи при следующей же встрече — "
            "правка диска сейчас будет молча затёрта. Останови ноду "
            "vision_face (например, `docker exec vision-face pkill -f "
            f"{node_process_pattern}` или перезапуск контейнера без "
            "автозапуска ноды) и повтори, либо передай --force, если "
            "точно знаешь, что делаешь."
        )
    if running is None:
        return (
            "не удалось проверить, запущена ли нода vision_face "
            f"('pgrep -f {node_process_pattern}' недоступен в этом "
            "окружении). Честный отказ вместо угадывания: проверь вручную "
            "(например, `docker exec vision-face pgrep -f "
            f"{node_process_pattern}`) и передай --force, если убедился, "
            "что нода остановлена."
        )
    return None


def _confirm(prompt: str, assume_yes: bool) -> bool:
    """Интерактивное подтверждение разрушительной операции. ``--yes``
    (``assume_yes=True``) пропускает подтверждение — для неинтерактивного
    запуска (``docker exec`` из скрипта/CI, где нет TTY для ``input()``).
    """
    if assume_yes:
        return True
    try:
        answer = input(f'{prompt} [yes/N]: ')
    except EOFError:
        # Нет TTY и не передали --yes — трактуем как отказ, а не как
        # тихое "да" (тот же принцип честного отказа, что у _ensure_node_not_running).
        return False
    return answer.strip().lower() == 'yes'


# =============================================================================
# list
# =============================================================================

def _format_cohesion(cohesion: Optional[Dict[str, float]]) -> str:
    if cohesion is None:
        return '—'
    return f"med={cohesion['median']:.2f} min={cohesion['min']:.2f}"


def _cohesion_flag(cohesion: Optional[Dict[str, float]], encounter_count: int) -> str:
    """Метка-подсказка рядом со строкой таблицы (issue #2775 диагностика).
    Пороги — не калиброванный классификатор, а грубая эвристика из
    диапазонов, названных в самой карточке; решение по конкретной записи
    всегда принимает человек, не скрипт."""
    if cohesion is None:
        return ''
    median = cohesion['median']
    if median < 0.5:
        return '  ⚠ ОТРАВЛЕНА? (низкий косинус галереи — похоже на разных людей)'
    if median > 0.93 and encounter_count > 50:
        return '  ⚠ ФАНТОМ? (подозрительно ровная галерея при большом числе встреч)'
    return ''


def cmd_list(args: argparse.Namespace) -> int:
    store = FaceStore(root=args.root)
    people = store.people()
    if not people:
        print(f'Нет записей под {args.root!r}.')
        return 0

    header = (
        f"{'person_id':38} {'name':16} {'speaker_id':14} "
        f"{'encounters':10} {'gallery':7} {'cohesion':18} {'disk':>9}"
    )
    print(header)
    print('-' * len(header))
    for p in sorted(people, key=lambda x: x['person_id']):
        person_id = p['person_id']
        person_dir = _person_dir(args.root, person_id)
        meta = _read_meta(person_dir)
        cohesion = _person_gallery_cohesion(store, p, person_id)
        name = p['name'] or '<без имени>'
        speaker_id = meta.get('speaker_id') or '-'
        disk_kb = _dir_size(person_dir) // 1024
        flag = _cohesion_flag(cohesion, p['encounter_count'])
        print(
            f"{person_id:38} {name:16.16} {str(speaker_id):14.14} "
            f"{p['encounter_count']:>10} {p['embeddings']:>7} "
            f"{_format_cohesion(cohesion):18} {disk_kb:>7} KB{flag}"
        )

    stats = store.stats()
    print('-' * len(header))
    print(
        f"итого: {stats['people']} записей ({stats['named']} именованных, "
        f"{stats['strangers']} незнакомцев), {stats['embeddings']} "
        f"эмбеддингов всего, {stats['disk_bytes'] // 1024} KB на диске."
    )
    return 0


# =============================================================================
# forget
# =============================================================================

def cmd_forget(args: argparse.Namespace) -> int:
    store = FaceStore(root=args.root)
    people_by_id = {p['person_id']: p for p in store.people()}
    if args.person_id not in people_by_id:
        print(f'person_id {args.person_id!r} не найден под {args.root!r}.', file=sys.stderr)
        return 2

    person_dir = _person_dir(args.root, args.person_id)
    summary = people_by_id[args.person_id]
    size_kb = _dir_size(person_dir) // 1024
    print(
        f"План: FaceStore.forget({args.person_id!r}) удалит запись целиком — "
        f"имя={summary['name']!r}, {summary['embeddings']} эмбеддингов, "
        f"{summary['encounter_count']} встреч, {size_kb} KB на диске "
        f"({person_dir}). НЕОБРАТИМО."
    )

    if not args.apply:
        print('Dry-run: ничего не удалено. Повтори с --apply (и --yes для неинтерактивного запуска).')
        return 0

    err = _ensure_node_not_running(args.force, args.node_process_pattern)
    if err:
        print(f'ОТКАЗ: {err}', file=sys.stderr)
        return 3

    if not _confirm(f'Точно безвозвратно удалить запись {args.person_id}?', args.yes):
        print('Отменено.')
        return 1

    ok = store.forget(args.person_id)
    print(f'forget({args.person_id!r}) -> {ok}')
    return 0 if ok else 4


# =============================================================================
# reset-gallery
# =============================================================================

def _npy_row_count(path: Path) -> int:
    if not path.exists():
        return 0
    try:
        arr = np.load(path)
    except (OSError, ValueError):
        return 0
    if arr.ndim == 1:
        return 1 if arr.size else 0
    return int(arr.shape[0])


def cmd_reset_gallery(args: argparse.Namespace) -> int:
    person_dir = _person_dir(args.root, args.person_id)
    meta_path = person_dir / _META_FILENAME
    if not meta_path.exists():
        print(f'person_id {args.person_id!r} не найден под {args.root!r} (нет {meta_path}).', file=sys.stderr)
        return 2

    meta = _read_meta(person_dir)
    embeddings_path = person_dir / _EMBEDDINGS_FILENAME
    encounters_dir = person_dir / _ENCOUNTERS_DIRNAME
    reference_path = person_dir / _REFERENCE_FILENAME

    n_embeddings = _npy_row_count(embeddings_path)
    n_encounter_files = sum(1 for _ in encounters_dir.glob('*')) if encounters_dir.exists() else 0
    has_ref = reference_path.exists()

    print(
        f"План: reset-gallery({args.person_id!r}) — снесёт {embeddings_path.name} "
        f"({n_embeddings} векторов) и {encounters_dir.name}/ ({n_encounter_files} файлов). "
        f"СОХРАНИТ: name={meta.get('name')!r}, speaker_id={meta.get('speaker_id')!r}, "
        f"person_id={meta.get('person_id')!r}, reference.jpg="
        f"{'сохранится (' + str(reference_path.stat().st_size) + ' байт)' if has_ref else 'ОТСУТСТВУЕТ — восстанавливать записи по фото будет нечем'}."
    )

    if not args.apply:
        print('Dry-run: ничего не изменено. Повтори с --apply (и --yes для неинтерактивного запуска).')
        return 0

    err = _ensure_node_not_running(args.force, args.node_process_pattern)
    if err:
        print(f'ОТКАЗ: {err}', file=sys.stderr)
        return 3

    if not _confirm(
        f'Точно сбросить галерею {args.person_id} (имя/speaker_id/reference.jpg сохранятся)?',
        args.yes,
    ):
        print('Отменено.')
        return 1

    if embeddings_path.exists():
        embeddings_path.unlink()
    if encounters_dir.exists():
        shutil.rmtree(encounters_dir, ignore_errors=True)

    new_meta = dict(meta)
    new_meta['encounter_count'] = 0
    new_meta['encounters'] = []
    new_meta['has_reference_snapshot'] = has_ref
    _atomic_write_json(meta_path, new_meta)

    print(
        f'reset-gallery({args.person_id!r}) выполнен: галерея и снимки встреч удалены, '
        'name/speaker_id/reference.jpg сохранены. Нода при следующем старте загрузит '
        'запись как знакомого без галереи — переучивание начнётся с чистого листа.'
    )
    return 0


# =============================================================================
# rebuild
# =============================================================================

@dataclass
class RebuildResult:
    person_id: str
    embedded: int
    skipped: int
    dim: int


def rebuild_gallery(
    person_dir: Path,
    embedder: Any,
    image_loader: Callable[[bytes], Optional[np.ndarray]],
    max_embeddings: Optional[int] = None,
) -> RebuildResult:
    """Пересчитать галерею эмбеддингов из снимков на диске.

    Чистая функция без CLI/argparse/print — тестируется напрямую с
    подставным ``embedder`` (любой объект с методом
    ``embed(list[ndarray]) -> list[Optional[ndarray]]``, тот же
    контракт, что ``ArcFaceEmbedder.embed``) и подставным
    ``image_loader`` (``bytes -> Optional[ndarray]``), без cv2 и без
    Hailo (issue #2775 требование). Источники — ``reference.jpg`` (если
    есть) и ``encounters/*_face.jpg`` (снимки встреч; ``*_body.jpg`` —
    это кропы тела, не лица, см. ``face_embedding``/``face_store``
    докстринги про ``face_snapshot``/``body_snapshot`` — намеренно
    исключены).

    ``max_embeddings`` — необязательный кап на итоговую галерею (если
    не задан, в галерею идут ВСЕ успешно посчитанные эмбеддинги без
    вытеснения «самого избыточного», в отличие от
    ``FaceStore._evict_most_redundant`` — это осознанное упрощение
    инструмента ремонта, не влияет на работающую ноду: она сама
    вытеснит лишнее при первой новой встрече по своим текущим лимитам).

    Ничего не решает про ``meta.json`` — трогает только
    ``embeddings.npy``, атомарно.
    """
    ref_path = person_dir / _REFERENCE_FILENAME
    encounters_dir = person_dir / _ENCOUNTERS_DIRNAME

    sources: List[Path] = []
    if ref_path.exists():
        sources.append(ref_path)
    if encounters_dir.exists():
        sources.extend(sorted(p for p in encounters_dir.iterdir() if p.name.endswith('_face.jpg')))

    crops: List[np.ndarray] = []
    decode_failures = 0
    for src in sources:
        img = image_loader(src.read_bytes())
        if img is None:
            decode_failures += 1
            continue
        crops.append(img)

    vectors = embedder.embed(crops) if crops else []
    good = [v for v in vectors if v is not None]
    embed_failures = len(crops) - len(good)

    if max_embeddings is not None and len(good) > max_embeddings:
        good = good[-max_embeddings:]  # свежие снимки важнее старых при капе

    dim = int(np.asarray(good[0]).reshape(-1).shape[0]) if good else 0
    if good:
        arr = np.stack([np.asarray(v, dtype=np.float32).reshape(-1) for v in good])
    else:
        arr = np.zeros((0, dim), dtype=np.float32)
    _atomic_write_npy(person_dir / _EMBEDDINGS_FILENAME, arr)

    return RebuildResult(
        person_id=person_dir.name,
        embedded=len(good),
        skipped=decode_failures + embed_failures,
        dim=dim,
    )


def _default_image_loader() -> Callable[[bytes], Optional[np.ndarray]]:
    """Ленивый JPEG-декодер через cv2. Поднимает ``ImportError`` с понятным
    сообщением, если cv2 недоступен — вызывается ТОЛЬКО из CLI-обвязки
    ``cmd_rebuild``, никогда из тестируемой ``rebuild_gallery``."""
    try:
        import cv2  # type: ignore[import-not-found]
    except ImportError as exc:
        raise ImportError(
            'rebuild: cv2 недоступен в этом окружении. Подкоманда rebuild '
            'декодирует JPEG и запускается ТОЛЬКО внутри контейнера '
            'vision-face (docker exec vision-face python3 ... rebuild '
            '<person_id> --apply).'
        ) from exc

    def _load(data: bytes) -> Optional[np.ndarray]:
        arr = np.frombuffer(data, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    return _load


def _default_embedder(hef_path: str) -> Any:
    """Ленивый ArcFaceEmbedder. Поднимает ``ImportError`` с понятным
    сообщением, если модуль/железо недоступны — ArcFace работает только
    на Hailo внутри контейнера vision-face (issue #2775: «спроектируй
    так, чтобы отсутствие железа давало внятный отказ, а не падение»).
    Реальная ошибка инициализации HailoRT (например, HEF не найден)
    всплывёт позже, при первом ``embed()`` — это поведение самого
    ``ArcFaceEmbedder`` (lazy-init, см. его докстринг), здесь её не
    перехватываем и не глушим."""
    try:
        from rob_box_perception.face_embedding import ArcFaceEmbedder
    except ImportError as exc:
        raise ImportError(
            'rebuild: rob_box_perception.face_embedding недоступен в этом '
            'окружении (нужны hailo_platform и остальной ROS-стек). rebuild '
            'рассчитан на запуск ТОЛЬКО внутри контейнера vision-face, где '
            'ROS сидит в /ws/install (см. docstring этого файла).'
        ) from exc
    return ArcFaceEmbedder(hef_path=hef_path)


def cmd_rebuild(args: argparse.Namespace) -> int:
    person_dir = _person_dir(args.root, args.person_id)
    meta_path = person_dir / _META_FILENAME
    if not meta_path.exists():
        print(f'person_id {args.person_id!r} не найден под {args.root!r} (нет {meta_path}).', file=sys.stderr)
        return 2

    ref_path = person_dir / _REFERENCE_FILENAME
    encounters_dir = person_dir / _ENCOUNTERS_DIRNAME
    n_sources = (1 if ref_path.exists() else 0)
    if encounters_dir.exists():
        n_sources += sum(1 for p in encounters_dir.iterdir() if p.name.endswith('_face.jpg'))

    print(
        f'План: rebuild({args.person_id!r}) пересчитает embeddings.npy из {n_sources} '
        f'снимков (reference.jpg + encounters/*_face.jpg) текущим ArcFace '
        f'(HEF={args.hef_path}). Старая галерея будет перезаписана.'
    )
    if n_sources == 0:
        print('ПРЕДУПРЕЖДЕНИЕ: снимков на диске нет — пересобирать не из чего.', file=sys.stderr)

    if not args.apply:
        print('Dry-run: ничего не изменено (Hailo/cv2 не трогались). Повтори с --apply.')
        return 0

    err = _ensure_node_not_running(args.force, args.node_process_pattern)
    if err:
        print(f'ОТКАЗ: {err}', file=sys.stderr)
        return 3

    if not _confirm(f'Точно пересчитать галерею {args.person_id} текущим эмбеддером?', args.yes):
        print('Отменено.')
        return 1

    try:
        image_loader = _default_image_loader()
        embedder = _default_embedder(args.hef_path)
    except ImportError as exc:
        print(f'ОТКАЗ: {exc}', file=sys.stderr)
        return 5

    try:
        result = rebuild_gallery(person_dir, embedder, image_loader)
    except Exception as exc:  # noqa: BLE001 — capability-honest: печатаем и выходим не 0
        print(f'ОШИБКА при пересчёте: {exc!r}', file=sys.stderr)
        return 6
    finally:
        close = getattr(embedder, 'close', None)
        if callable(close):
            close()

    print(
        f'rebuild({args.person_id!r}) выполнен: {result.embedded} эмбеддингов записано '
        f'(dim={result.dim}), {result.skipped} снимков пропущено (не декодировались/не '
        'дали эмбеддинг).'
    )
    return 0


# =============================================================================
# argparse
# =============================================================================

def _add_common_mutation_args(sub: argparse.ArgumentParser) -> None:
    sub.add_argument('person_id', help='person_id записи (имя каталога под --root)')
    sub.add_argument(
        '--apply', action='store_true',
        help='Реально выполнить операцию (по умолчанию — dry-run, ничего не меняет).',
    )
    sub.add_argument(
        '--yes', action='store_true',
        help='Не спрашивать интерактивное подтверждение (для неинтерактивного запуска).',
    )
    sub.add_argument(
        '--force', action='store_true',
        help=(
            'Пропустить проверку "не запущена ли нода vision_face". Использовать, только '
            'если сам убедился, что нода остановлена — иначе правка будет затёрта при '
            'следующей встрече (см. докстринг модуля).'
        ),
    )
    sub.add_argument(
        '--node-process-pattern', default=DEFAULT_NODE_PROCESS_PATTERN,
        help=f'Паттерн для "pgrep -f" при проверке живой ноды (по умолчанию {DEFAULT_NODE_PROCESS_PATTERN!r}).',
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='face_store_admin',
        description=(
            'CLI поверх публичного API FaceStore для ремонта лицевых записей '
            '/data/faces (issue #2775) — не rm -rf. Запуск ВНУТРИ контейнера '
            'ОБЯЗАН включать оба source, иначе "ModuleNotFoundError: No '
            "module named 'rob_box_perception'\" (пакет живёт в /ws/install, "
            'не /ros2_ws/install, и без source его нет на PYTHONPATH ни для '
            'одной подкоманды): docker exec vision-face bash -c "source '
            '/opt/ros/humble/setup.bash && source /ws/install/setup.bash && '
            'python3 /tmp/face_store_admin.py list". См. docstring модуля '
            'для остальных оговорок (живая нода держит данные в памяти; '
            'путь внутри контейнера vision-face vs на хосте Vision Pi).'
        ),
    )
    parser.add_argument(
        '--root',
        default=os.environ.get('FACE_STORE_ROOT', DEFAULT_ROOT),
        help=(
            'Корень хранилища FaceStore. По умолчанию берётся из ENV '
            'FACE_STORE_ROOT (тот же, что видит сама нода), иначе '
            f'{DEFAULT_ROOT!r} — этот дефолт верен ТОЛЬКО внутри контейнера '
            'vision-face (docker-compose монтирует туда ./data/faces с хоста); '
            'при запуске не в контейнере передай --root явно.'
        ),
    )

    sub = parser.add_subparsers(dest='command', required=True)

    p_list = sub.add_parser('list', help='Таблица всех записей: person_id, имя, speaker_id, встречи, галерея, косинус.')
    p_list.set_defaults(func=cmd_list)

    p_forget = sub.add_parser('forget', help='Удалить запись целиком через FaceStore.forget() (не rm -rf).')
    _add_common_mutation_args(p_forget)
    p_forget.set_defaults(func=cmd_forget)

    p_reset = sub.add_parser(
        'reset-gallery',
        help='Снести галерею эмбеддингов и снимки встреч, сохранив name/speaker_id/reference.jpg.',
    )
    _add_common_mutation_args(p_reset)
    p_reset.set_defaults(func=cmd_reset_gallery)

    p_rebuild = sub.add_parser(
        'rebuild',
        help='Пересчитать галерею из сохранённых снимков текущим ArcFace-эмбеддером (только в контейнере vision-face).',
    )
    _add_common_mutation_args(p_rebuild)
    p_rebuild.add_argument(
        '--hef-path',
        default=os.environ.get('ARCFACE_HEF_PATH', DEFAULT_ARCFACE_HEF),
        help=f'Путь к ArcFace HEF внутри контейнера (по умолчанию {DEFAULT_ARCFACE_HEF!r}).',
    )
    p_rebuild.set_defaults(func=cmd_rebuild)

    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == '__main__':
    sys.exit(main())
