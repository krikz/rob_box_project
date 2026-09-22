#!/usr/bin/env python3
"""speaker_db_admin.py — CLI поверх публичного API ``SpeakerDatabase`` /
``VoiceIdentitySeam`` для обслуживания голосовой БД дикторов
``/data/speakers.db`` (issue #2777, добивка к PR #2777 — сосед issue #2775
для лиц). Сделан по образцу ``scripts/maintenance/face_store_admin.py``
(та же структура, конвенции argparse, тот же приём импорта без
pip-install) — читай его докстринг параллельно, здесь не повторяется то,
что там объяснено общо.

Зачем этот скрипт вообще нужен
------------------------------
22.09.2026 в БОЕВОЙ ``/data/speakers.db`` на роботе оказалось 5 профилей,
из которых половина — мусор E2E-сценария («Саша», «Борис», «Ночной
инженер» — персонажи, не люди), а хозяин («Деньчик»/«Дэнчик») развалился
на ДВА профиля, дерущихся между собой с зазором 0.012–0.015:

::

    identify candidates: best='Дэнчик'(1ae4b0ac) score=0.743 |
        second='Деньчик'(d9b91d1c) score=0.731 | gap=0.012

Это уже не узнавание, а жребий: оба скора выше ``IDENTIFY_THRESHOLD``
(0.72, см. ``utils/speaker_embeddings.py``), и «Знакомый» на грани
меняется от реплики к реплике. Починить это ssh-командой по живой БД
нельзя — issue #2750: боевую ``speakers.db`` трижды за 12 часов обнуляли
ручными инлайновыми ``docker exec ... sh -c "cp speakers.db
speakers.db.bak-<UTC>Z" && sqlite3 ... "DELETE FROM ..."`` командами, и
одна из них стёрла живой профиль человека через 19 минут после
регистрации, а виновника найти так и не смогли (никаких следов вне
ssh-истории оператора). Этот скрипт — не новая логика поверх
``SpeakerDatabase``/``VoiceIdentitySeam``, а безопасная обвязка вокруг
уже существующего публичного API, которая (а) оставляет след
(атрибутированный бэкап + понятный план перед любым действием) и (b) не
даёт выстрелить в ногу (dry-run по умолчанию, гейт на живую ноду,
подтверждение для разрушительных операций).

Подкоманды
----------
``list``
    Таблица: ``speaker_id``, имя, эпитет, дата создания, число
    эмбеддингов. Плюс — БЕСПЛАТНАЯ попарная диагностика: максимальный
    косинус между галереями ЛЮБЫХ двух профилей (используя уже
    сохранённые в БД эмбеддинги — никакого resemblyzer считать не нужно,
    сравниваются готовые float32-векторы, которые и так лежат в таблице
    ``embeddings``; на типичном размере галереи робота — единицы
    профилей, до ``GALLERY_WARMUP_SIZE=5`` эмбеддингов на профиль — это
    O(n²·k²) с крошечными n/k, то есть буквально бесплатно, поэтому
    считается ВСЕГДА, без отдельного флага «--slow-mode»; ``--no-similarity``
    даёт просто пропустить его, если БД неожиданно огромная). Пары с
    максимальным косинусом ``>= IDENTIFY_THRESHOLD`` (тот же порог, что
    ``identify()`` использует в проде, реэкспортирован из
    ``speaker_embeddings.py``, а не изобретён заново) помечаются как
    вероятные дубли одного человека — ровно то, что 22.09.2026 бросалось
    в глаза только числом, без инструмента.

``merge --src <id> --dst <id>``
    Слить профиль ``src`` в ``dst`` (имя ``dst`` считается основным и
    сохраняется): эмбеддинги голоса + факты профиля. См. отдельный
    большой блок ниже — «Почему merge зовёт VoiceIdentitySeam.merge, а не
    ``merge_speakers()``+``merge_legacy_voice_facts()`` по отдельности».

``delete <speaker_id>``
    Удалить профиль целиком (``SpeakerDatabase.delete_speaker`` —
    эмбеддинги уходят каскадом). Разрушительно, необратимо. НЕ трогает
    факты профиля (``scope=speaker:<id>`` в ``harness_voice.db`` /
    ``voice_memory.db``) — они останутся осиротевшими; точечной чистки
    фактов в скоупе этого инструмента нет (задача просила удаление
    голосового профиля, не аудит слоя памяти).

Почему merge зовёт VoiceIdentitySeam.merge, а не две функции по отдельности
----------------------------------------------------------------------------
Задача разрешала оба варианта — вызвать ``SpeakerDatabase.merge_speakers()``
и ``legacy_voice_facts.merge_legacy_voice_facts()`` напрямую, либо
разобраться, как их полагается звать вместе, и повторить принятый
порядок. Разобрался: в кодовой базе это уже сделано, дважды.

1. ``rob_box_voice.utils.identity_seam.VoiceIdentitySeam.merge()`` — сам
   метод переносит ОБА писателя фактов, не один: ``merge_speaker_facts()``
   (слой ``rob_box_harness.memory`` / ``MemoryStore``, файл
   ``harness_voice.db``) И, если передан ``legacy_facts_db_path``,
   ``merge_legacy_voice_facts()`` (легаси-писатель MCP-инструмента
   ``memory_save``, файл ``voice_memory.db``). Замер issue #2751 на
   проде: ``harness_voice.db`` — 10 старых фактов, ``voice_facts`` в
   ``voice_memory.db`` — 100 ЖИВЫХ. Если звать только
   ``merge_legacy_voice_facts()`` напрямую (как предлагала задача первым
   вариантом), эти 10 профильных фактов из ``harness_voice.db`` (имя,
   last_seen, dialog_count — то, что пишет ``touch_speaker``/
   ``note_seen`` шва идентичности) остались бы неперенесёнными — «тихая»
   потеря, тот же класс бага, что issue #2751 нашла для другой пары БД.
2. ``rob_box_voice.speaker_id_node._merge_identity_async()`` — тот же
   метод узла, который дёргает оператор через сервис ``merge`` НА
   ЖИВОМ РОБОТЕ, устроен ИМЕННО так: поднимает
   ``SQLiteVoiceMemory(db_path=memory_db_path)``,
   ``VoiceIdentitySeam(self._db, store, legacy_facts_db_path=...)`` и
   вызывает ``await seam.merge(src, dst)`` через ``asyncio.run``.

То есть узел, который сегодня реально мержит профили на роботе, УЖЕ не
использует «две функции по отдельности» — использует шов. Повторять
низкоуровневый путь означало бы: (а) разойтись с поведением ноды при
малейшем будущем изменении шва (например, если у ``merge()`` появится
третий перенос — этот скрипт molча отстанет), и (б) заново решать
вопрос «а как насчёт harness_voice.db», на который у задачи уже есть
готовый ответ в виде существующего кода. Здесь просто повторена ТА ЖЕ
последовательность конструирования объектов, что в
``speaker_id_node._merge_identity_async`` (см. код ниже, ``cmd_merge``).

Гейт на живую ноду
------------------
``speaker_id_node`` держит ``SpeakerDatabase`` открытой в памяти процесса
(``self._db``) всё время работы. Правка ``speakers.db`` файлом на диске
мимо ноды, пока она жива, либо (а) потеряется — нода перезапишет свою
in-memory копию поверх при следующем ``register()``/``append_reference_
embedding()``, либо (б) устроит гонку двух писателей одного sqlite-файла
одновременно. Поэтому КАЖДАЯ разрушительная подкоманда (``merge --apply``,
``delete --apply``) ПЕРЕД записью проверяет, не запущен ли процесс ноды
(``pgrep -f speaker_id_node`` внутри контейнера — тот же PID namespace,
что и у ``docker exec``, entry point ``speaker_id_node`` из
``rob_box_voice/setup.py``), и ОТКАЗЫВАЕТСЯ работать без ``--force``, если
похоже, что нода запущена, ИЛИ если проверить не удалось (нет ``pgrep`` в
окружении) — честный отказ вместо угадывания «наверное, не запущена»,
три состояния (запущена/не запущена/не удалось проверить), и только
СРЕДНЕЕ разрешает работу без ``--force``. Скрипт не умеет и не пытается
сам остановить ноду — это отдельное действие оператора (``docker exec
voice-assistant pkill -f speaker_id_node`` или временный ``docker stop``,
смотри по обстоятельствам на роботе).

Бэкап перед записью — и почему НЕ ``.bak-<UTC>Z``
---------------------------------------------------
Перед ЛЮБОЙ мутацией скрипт САМ копирует затрагиваемые sqlite-файлы рядом
и печатает, куда. Имя бэкапа сознательно НЕ совпадает с форматом
``<файл>.bak-%Y%m%dT%H%M%SZ`` — под этим именем в истории репозитория
(issue #2750, комментарии ``speaker_id_node.py`` про переключение
e2e_mode) ходят бэкапы НЕИЗВЕСТНОГО чистильщика, который трижды обнулял
боевую БД вручную и которого так и не опознали; если бэкапы этого
инструмента будут называться так же, следующий разбор инцидента снова
не сможет отличить «это сделал CLI осознанно, с планом и логом» от «это
опять безымянная ручная команда». Формат здесь —
``<файл>.speaker_db_admin-backup-<UTC>Z`` (без подстроки ``.bak-``, с
именем инструмента прямо в имени файла — авторство видно на диске без
дополнительного контекста). Если у файла есть sqlite WAL-сайдкары
(``-wal``/``-shm`` — ``SQLiteVoiceMemory`` включает ``PRAGMA
journal_mode=WAL``, у ``speakers.db`` их обычно нет, но проверяется
всегда), они бэкапятся тоже — иначе бэкап основного файла может не
содержать самых свежих закоммиченных, но ещё не checkpoint'нутых строк.

Как это запускают на Vision Pi
-------------------------------
::

    docker exec voice-assistant bash -c \\
        "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \\
         python3 /tmp/speaker_db_admin.py list"

Команда ОБЯЗАНА включать ОБА ``source`` — без них ``rob_box_voice``/
``rob_box_harness`` не найдены НИ ДЛЯ ОДНОЙ подкоманды (см. живой
инцидент с соседним ``face_store_admin.py``, issue #2775: агент запустил
скопированный файл голым ``python3`` без ``source`` и получил
``ModuleNotFoundError`` — это не баг, а недостающий шаг в команде запуска,
теперь прописанный явно и здесь).

* содержимое ``scripts/`` НЕ копируется в образ ``voice-assistant``
  (``docker/vision/voice_assistant/Dockerfile`` копирует только
  ``src/rob_box_core``, ``src/rob_box_voice``, ``src/rob_box_harness`` и
  т. д. через colcon build) — доставка ТОЛЬКО разовым ``docker cp
  speaker_db_admin.py voice-assistant:/tmp/speaker_db_admin.py`` (или
  volume-монтированием), запускать оттуда;
* этот файл может оказаться на ЛЮБОЙ глубине вложенности после
  ``docker cp`` (``/tmp/speaker_db_admin.py`` — один каталог-предок).
  Импорт НИКОГДА не вычисляет путь к пакетам из глубины вложенности
  ЭТОГО файла безусловно — см. ``_guess_pkg_root``/``_import_voice_module``
  ниже: сначала ВСЕГДА пробуется обычный ``import``, который срабатывает
  сам, если ``source`` выполнен (пакеты уже на ``PYTHONPATH``), достройка
  ``sys.path`` от расположения файла — только запасной путь для локальной
  разработки/тестов из чекаута репозитория (см. issue #2775, тот же класс
  бага уже ловился на ``face_store_admin.py`` и исправлен там же);
* ROS-окружение внутри контейнера ``voice-assistant`` живёт в
  ``/ws/install`` (``docker/vision/voice_assistant/Dockerfile``:
  ``colcon build`` пишет туда), а НЕ в ``/ros2_ws/install`` — это другой
  контейнер/другой сервис в этом репозитории;
* путь к БД — параметром ``--db-path``, дефолт ``/data/speakers.db`` (тот
  же дефолт, что у ROS-параметра ``db_path`` в ``speaker_id_node.py``).
  ОТДЕЛЬНОЕ ПРЕДУПРЕЖДЕНИЕ (issue #2759): ``/data/speakers.e2e.db`` —
  ЭТО ДРУГАЯ БАЗА, база E2E-прогонов (``speaker_id_node`` переключается
  на неё параметром ``e2e_mode`` и ГАРАНТИРОВАННО чистит её при каждом
  включении режима, см. ``_apply_e2e_mode`` в ``speaker_id_node.py``) —
  спутать пути значит либо чистить чужие тестовые данные, либо решить,
  что боевая БД пуста, глядя на пустую E2E-копию. Этот скрипт НИКОГДА не
  выбирает БД сам — только то, что передано в ``--db-path``.

Тестируемость без ROS/resemblyzer/pyaudio
--------------------------------------------
``SpeakerDatabase``/``VoiceIdentitySeam`` целиком на чистом Python +
sqlite3 + numpy — resemblyzer нужен только для ``embed_audio()``
(превращение сырого звука в вектор), которым этот CLI не пользуется
вообще: он работает с уже сохранёнными эмбеддингами. Единственная
реальная преграда — ``rob_box_voice.utils.__init__`` импортирует
``pyaudio``/``sounddevice`` (аудио-железо) при обычном ``import
rob_box_voice.utils.speaker_embeddings`` — на dev-машине/CI их нет.
Обходится тем же приёмом, что ``test_identity_seam.py``/
``test_speaker_embeddings.py`` этого репозитория: если обычный импорт не
удался, нужные файлы (``speaker_embeddings.py``, ``legacy_voice_facts.py``,
``identity_seam.py``) грузятся напрямую по пути, минуя
``utils/__init__.py``. Внутри контейнера ``voice-assistant`` pyaudio
РЕАЛЬНО установлен (аппаратный контейнер) — там обычный импорт просто
срабатывает первым, никакого обхода не требуется. Запуск тестов:

::

    python -m pytest src/rob_box_voice/test/unit/scripts/test_speaker_db_admin.py -q --no-cov

Честность (AGENTS.md: «честный FAIL лучше красивого PASS»)
-------------------------------------------------------------
Этот файл писался и тестировался ТОЛЬКО на синтетических БД во временном
каталоге (``tmp_path``) в рамках issue #2777. На живом роботе (Vision Pi,
реальная ``/data/speakers.db``, реальная нода ``speaker_id_node``) он НЕ
запускался и НЕ проверялся — ни одна подкоманда, включая ``list``.
Реальный прогон делает оркестратор под присмотром владельца.
"""

from __future__ import annotations

import argparse
import datetime
import importlib
import importlib.util
import os
import shutil
import sqlite3
import subprocess
import sys
import time
import types
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Импорт без установки пакета — см. большой блок докстринга модуля выше
# («Как это запускают на Vision Pi») и исправленный после живого инцидента
# приём в scripts/maintenance/face_store_admin.py (issue #2775): СНАЧАЛА
# всегда пробуем обычный импорт (работает без единого вычисления путей,
# если ROS-окружение source'нуто — ровно так внутри контейнера), и ТОЛЬКО
# если он не сработал, пытаемся достроить sys.path от расположения ЭТОГО
# файла — а если даже это невозможно (мало предков-каталогов, как для
# копии в /tmp), просто пропускаем попытку и даём обычному импорту поднять
# честный ModuleNotFoundError, а не IndexError.
# ---------------------------------------------------------------------------
_HERE = Path(__file__).resolve()


def _guess_pkg_root(here: Path, pkg_dir_name: str) -> Optional[Path]:
    """``<repo_root>/src/<pkg_dir_name>``, ЕСЛИ ``here`` лежит по своему
    обычному пути ``<repo>/scripts/maintenance/<файл>`` (ровно два уровня
    вложенности под корнем репозитория). ``None`` вместо ``IndexError``,
    если предков не хватает (файл доставлен отдельно от чекаута —
    ``docker cp`` в ``/tmp``). Чистая функция, sys.path не трогает —
    специально ради теста без реального копирования файла на диск."""
    parents = here.parents
    if len(parents) <= 2:
        return None
    return parents[2] / 'src' / pkg_dir_name


def _ensure_on_sys_path(path: Optional[Path]) -> None:
    if path is not None and str(path) not in sys.path:
        sys.path.insert(0, str(path))


def _load_module_from_file(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None, f'{path}: spec failed'
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _ensure_fake_voice_utils_package(voice_pkg_root: Path) -> None:
    """Подложить ``rob_box_voice``/``rob_box_voice.utils`` в sys.modules
    БЕЗ выполнения настоящего ``utils/__init__.py`` (тянет pyaudio) — тот
    же приём, что ``test_identity_seam.py`` использует для тех же файлов.
    ``rob_box_voice/__init__.py`` сам по себе пустой и безопасный — если
    он уже успел импортироваться обычным способом (и упасть только на
    подпакете ``utils``), готовую запись в sys.modules не трогаем."""
    if 'rob_box_voice' not in sys.modules:
        pkg = types.ModuleType('rob_box_voice')
        pkg.__path__ = [str(voice_pkg_root)]
        sys.modules['rob_box_voice'] = pkg
    if 'rob_box_voice.utils' not in sys.modules:
        subpkg = types.ModuleType('rob_box_voice.utils')
        subpkg.__path__ = [str(voice_pkg_root / 'utils')]
        sys.modules['rob_box_voice.utils'] = subpkg


def _load_speaker_embeddings_module():
    """``rob_box_voice.utils.speaker_embeddings`` — SpeakerDatabase и
    пороги. Нужен для ``list``/``merge``/``delete``."""
    try:
        import rob_box_voice.utils.speaker_embeddings as se  # type: ignore
        return se
    except ImportError:
        pass
    if 'rob_box_voice.utils.speaker_embeddings' in sys.modules:
        return sys.modules['rob_box_voice.utils.speaker_embeddings']
    pkg_root = _guess_pkg_root(_HERE, 'rob_box_voice')
    voice_pkg_root = (pkg_root / 'rob_box_voice') if pkg_root is not None else None
    if voice_pkg_root is None or not voice_pkg_root.exists():
        # Ни обычный импорт, ни локальный чекаут не сработали — честный
        # ModuleNotFoundError, а не самодельная ошибка/IndexError.
        import rob_box_voice.utils.speaker_embeddings as se  # type: ignore  # noqa: F401
    _ensure_fake_voice_utils_package(voice_pkg_root)  # type: ignore[arg-type]
    return _load_module_from_file(
        'rob_box_voice.utils.speaker_embeddings',
        voice_pkg_root / 'utils' / 'speaker_embeddings.py',  # type: ignore[operator]
    )


def _ensure_harness_importable() -> None:
    try:
        import rob_box_harness  # noqa: F401
        return
    except ImportError:
        pass
    harness_root = _guess_pkg_root(_HERE, 'rob_box_harness')
    _ensure_on_sys_path(harness_root)


def _load_identity_seam_module():
    """``rob_box_voice.utils.identity_seam`` (+ зависимые
    ``speaker_embeddings``/``legacy_voice_facts``) — нужен ТОЛЬКО для
    ``merge`` (единственная подкоманда, которой нужны факты/harness)."""
    try:
        import rob_box_voice.utils.identity_seam as ism  # type: ignore
        return ism
    except ImportError:
        pass
    if 'rob_box_voice.utils.identity_seam' in sys.modules:
        return sys.modules['rob_box_voice.utils.identity_seam']

    pkg_root = _guess_pkg_root(_HERE, 'rob_box_voice')
    voice_pkg_root = (pkg_root / 'rob_box_voice') if pkg_root is not None else None
    if voice_pkg_root is None or not voice_pkg_root.exists():
        import rob_box_voice.utils.identity_seam as ism  # type: ignore  # noqa: F401

    _ensure_fake_voice_utils_package(voice_pkg_root)  # type: ignore[arg-type]
    utils_dir = voice_pkg_root / 'utils'  # type: ignore[operator]
    if 'rob_box_voice.utils.speaker_embeddings' not in sys.modules:
        _load_module_from_file(
            'rob_box_voice.utils.speaker_embeddings', utils_dir / 'speaker_embeddings.py'
        )
    if 'rob_box_voice.utils.legacy_voice_facts' not in sys.modules:
        _load_module_from_file(
            'rob_box_voice.utils.legacy_voice_facts', utils_dir / 'legacy_voice_facts.py'
        )
    _ensure_harness_importable()  # identity_seam.py делает `from rob_box_harness...`
    return _load_module_from_file('rob_box_voice.utils.identity_seam', utils_dir / 'identity_seam.py')


# ---------------------------------------------------------------------------
# Константы
# ---------------------------------------------------------------------------

#: Тот же дефолт, что ROS-параметр ``db_path`` в speaker_id_node.py.
DEFAULT_DB_PATH = '/data/speakers.db'
#: Тот же дефолт, что ROS-параметр ``memory_db_path`` (issue #2440).
DEFAULT_MEMORY_DB_PATH = '/data/harness_voice.db'
#: Тот же дефолт, что ROS-параметр ``voice_facts_db_path`` (issue #2751).
DEFAULT_LEGACY_FACTS_DB_PATH = '/data/voice_memory.db'
#: Entry point из src/rob_box_voice/setup.py — `speaker_id_node = ...:main`.
DEFAULT_NODE_PROCESS_PATTERN = 'speaker_id_node'
#: НЕ '.bak-...Z' — см. докстринг модуля §«Бэкап перед записью».
_BACKUP_TAG = 'speaker_db_admin-backup'


# =============================================================================
# Бэкап — см. докстринг модуля
# =============================================================================

def _backup_sqlite_file(path: Path) -> Path:
    """Скопировать ``path`` (+ ``-wal``/``-shm`` сайдкары, если есть) рядом
    под именем ``<path>.speaker_db_admin-backup-<UTC>Z``. Возвращает путь
    к бэкапу основного файла."""
    ts = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
    dst = path.with_name(f'{path.name}.{_BACKUP_TAG}-{ts}')
    shutil.copy2(path, dst)
    for suffix in ('-wal', '-shm'):
        side = path.with_name(path.name + suffix)
        if side.exists():
            shutil.copy2(side, Path(str(dst) + suffix))
    return dst


# =============================================================================
# Проверка "не запущена ли нода" — тот же приём, что face_store_admin.py
# =============================================================================

def _node_running_via_pgrep(pattern: str) -> Optional[bool]:
    """``True``/``False``, если ``pgrep -f <pattern>`` удалось выполнить;
    ``None``, если проверить не удалось вовсе (нет ``pgrep`` в PATH —
    например, Windows-хост разработчика)."""
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
    иначе ``None``. ``node_check`` — параметр ради тестов (можно
    подставить фейковую проверку вместо реального ``pgrep``).

    ``node_check=None`` (дефолт) читает модульный
    ``_node_running_via_pgrep`` ПРЯМО ЗДЕСЬ, при каждом вызове — иначе
    тесты не смогли бы подменить его через ``monkeypatch.setattr``
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
            "похоже запущена. Она держит /data/speakers.db открытой в памяти и "
            "перепишет её поверх правки этого скрипта при следующей же регистрации "
            "— правка диска сейчас будет либо молча затёрта, либо подерётся с "
            "нодой за один sqlite-файл. Останови ноду speaker_id_node (например, "
            f"`docker exec voice-assistant pkill -f {node_process_pattern}` или "
            "временный docker stop) и повтори, либо передай --force, если точно "
            "знаешь, что делаешь."
        )
    if running is None:
        return (
            "не удалось проверить, запущена ли нода speaker_id_node "
            f"('pgrep -f {node_process_pattern}' недоступен в этом окружении). "
            "Честный отказ вместо угадывания: проверь вручную (например, "
            f"`docker exec voice-assistant pgrep -f {node_process_pattern}`) и "
            "передай --force, если убедился, что нода остановлена."
        )
    return None


def _confirm(prompt: str, assume_yes: bool) -> bool:
    """Интерактивное подтверждение разрушительной операции. ``--yes``
    пропускает его — для неинтерактивного запуска (``docker exec`` из
    скрипта/CI, где нет TTY для ``input()``)."""
    if assume_yes:
        return True
    try:
        answer = input(f'{prompt} [yes/N]: ')
    except EOFError:
        # Нет TTY и не передали --yes — трактуем как отказ, не как тихое "да".
        return False
    return answer.strip().lower() == 'yes'


# =============================================================================
# Диагностика: попарная близость профилей — см. докстринг §list
# =============================================================================

def _cosine(a: np.ndarray, b: np.ndarray) -> float:
    an = np.linalg.norm(a)
    bn = np.linalg.norm(b)
    if an < 1e-9 or bn < 1e-9:
        return 0.0
    return float(np.dot(a, b) / (an * bn))


def _read_embeddings_by_speaker(db_path: str) -> Dict[str, List[np.ndarray]]:
    """Прочитать все эмбеддинги из ``embeddings`` НАПРЯМУЮ, в обход
    ``SpeakerDatabase`` (у которой нет публичного метода "дай мне все
    векторы всех спикеров") — только на чтение (``mode=ro`` в URI —
    гарантия, что диагностика ``list`` физически не может ничего
    изменить, даже случайно). Формат blob — тот же, что
    ``SpeakerDatabase._blob_to_ndarray``: ``np.frombuffer(blob,
    dtype=np.float32)`` — задокументированный, стабильный формат
    (``_ndarray_to_blob`` = ``arr.astype(np.float32).tobytes()``), не
    приватная деталь реализации, которая может измениться без объявления.
    """
    uri = f'file:{Path(db_path).resolve().as_posix()}?mode=ro'
    conn = sqlite3.connect(uri, uri=True, timeout=2.0)
    try:
        rows = conn.execute('SELECT speaker_id, embedding FROM embeddings').fetchall()
    finally:
        conn.close()
    out: Dict[str, List[np.ndarray]] = {}
    for speaker_id, blob in rows:
        out.setdefault(speaker_id, []).append(np.frombuffer(blob, dtype=np.float32))
    return out


def _pairwise_speaker_similarity(
    embeddings_by_speaker: Dict[str, List[np.ndarray]]
) -> List[Tuple[str, str, float]]:
    """Для каждой пары РАЗНЫХ спикеров — максимальный косинус между ЛЮБОЙ
    парой их эмбеддингов (тот же MAX-of-cosine принцип, что
    ``SpeakerDatabase._score_all`` использует для identify() — «лучший
    матч важнее среднего»: если хоть один эмбеддинг одного профиля похож
    на хоть один эмбеддинг другого, это уже повод присмотреться).
    Отсортировано по убыванию похожести."""
    ids = sorted(embeddings_by_speaker)
    out: List[Tuple[str, str, float]] = []
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a_id, b_id = ids[i], ids[j]
            best = None
            for a in embeddings_by_speaker[a_id]:
                for b in embeddings_by_speaker[b_id]:
                    score = _cosine(a, b)
                    if best is None or score > best:
                        best = score
            if best is not None:
                out.append((a_id, b_id, best))
    out.sort(key=lambda t: -t[2])
    return out


# =============================================================================
# Best-effort read-only превью числа фактов для dry-run merge
# =============================================================================

def _preview_fact_count(db_path: Optional[str], table: str, column: str, value: str) -> str:
    """Лучшая попытка честной оценки для dry-run: сколько строк из
    ``table`` придётся ПОПЫТАТЬСЯ перенести. НЕ гарантирует итоговое
    число после ``--apply`` — ``merge_speaker_facts()`` отбрасывает
    часть при конфликте ключей с dst (см. докстринг ``identity_seam.py``:
    "при конфликте ключей выигрывает dst") — только верхняя граница.
    Деградирует до ``'?'``/явного текста на любой проблеме (файла нет,
    таблицы нет, БД занята) — диагностика для человека, не источник
    истины, падать из-за неё нельзя."""
    if not db_path or not os.path.exists(db_path):
        return '0 (БД отсутствует)'
    try:
        uri = f'file:{Path(db_path).resolve().as_posix()}?mode=ro'
        conn = sqlite3.connect(uri, uri=True, timeout=2.0)
    except sqlite3.Error:
        return '? (не удалось открыть)'
    try:
        exists = conn.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (table,)
        ).fetchone()
        if not exists:
            return '0 (таблицы ещё нет)'
        row = conn.execute(
            f'SELECT COUNT(*) FROM {table} WHERE {column}=?', (value,)  # noqa: S608 — table/column наши константы
        ).fetchone()
        return str(row[0]) if row else '0'
    except sqlite3.Error:
        return '? (ошибка чтения)'
    finally:
        conn.close()


# =============================================================================
# list
# =============================================================================

def _format_created_at(ts: float) -> str:
    return datetime.datetime.fromtimestamp(ts, tz=datetime.timezone.utc).strftime('%Y-%m-%d %H:%M:%SZ')


def cmd_list(args: argparse.Namespace) -> int:
    if not os.path.exists(args.db_path):
        print(
            f'БД не найдена: {args.db_path!r} — возможно, ещё не создана '
            '(нода создаст её при первой регистрации). Ничего не делаю.'
        )
        return 0

    se = _load_speaker_embeddings_module()
    db = se.SpeakerDatabase(args.db_path)
    try:
        speakers = db.list_speakers()
    finally:
        db.close()

    if not speakers:
        print(f'В {args.db_path!r} нет ни одного профиля.')
        return 0

    header = f"{'speaker_id':38} {'name':16} {'epithet':14} {'created_at (UTC)':20} {'embeddings':>10}"
    print(header)
    print('-' * len(header))
    for s in sorted(speakers, key=lambda x: x['created_at']):
        created = _format_created_at(s['created_at'])
        print(
            f"{s['id']:38} {s['name']:16.16} {(s['epithet'] or '-'):14.14} "
            f"{created:20} {s['embeddings']:>10}"
        )
    print('-' * len(header))
    print(f'итого: {len(speakers)} профилей.')

    if args.no_similarity:
        print('Попарное сравнение галерей пропущено (--no-similarity).')
        return 0

    threshold = args.similarity_threshold if args.similarity_threshold is not None else se.IDENTIFY_THRESHOLD
    embeddings = _read_embeddings_by_speaker(args.db_path)
    pairs = _pairwise_speaker_similarity(embeddings)
    names = {s['id']: s['name'] for s in speakers}
    suspects = [(a, b, score) for a, b, score in pairs if score >= threshold]

    print()
    if suspects:
        print(
            f'⚠ Возможные дубли одного человека (макс. косинус галерей >= {threshold:.2f}, '
            'тот же порог, что identify() в проде):'
        )
        for a, b, score in suspects:
            print(
                f"  {names.get(a, a)!r} ({a}) ~ {names.get(b, b)!r} ({b}): score={score:.3f}  "
                f"→ merge --src {a} --dst {b}  (или наоборот — реши, чьё имя основное)"
            )
    else:
        print(f'Пар с косинусом >= {threshold:.2f} не найдено — явных дублей не видно.')
    return 0


# =============================================================================
# merge — см. докстринг модуля §«Почему merge зовёт VoiceIdentitySeam.merge»
# =============================================================================

def cmd_merge(args: argparse.Namespace) -> int:
    if not os.path.exists(args.db_path):
        print(f'ОТКАЗ: БД не найдена: {args.db_path!r}.', file=sys.stderr)
        return 2
    if args.src == args.dst:
        print('ОТКАЗ: --src и --dst совпадают — сливать нечего.', file=sys.stderr)
        return 2

    se = _load_speaker_embeddings_module()
    db = se.SpeakerDatabase(args.db_path)
    try:
        speakers = {s['id']: s for s in db.list_speakers()}
    finally:
        db.close()

    if args.src not in speakers:
        print(f'ОТКАЗ: src={args.src!r} не найден в {args.db_path!r}.', file=sys.stderr)
        return 2
    if args.dst not in speakers:
        print(f'ОТКАЗ: dst={args.dst!r} не найден в {args.db_path!r}.', file=sys.stderr)
        return 2

    src_info, dst_info = speakers[args.src], speakers[args.dst]
    harness_preview = _preview_fact_count(args.memory_db_path, 'facts', 'scope', f'speaker:{args.src}')
    legacy_preview = _preview_fact_count(args.legacy_facts_db_path, 'voice_facts', 'speaker_id', args.src)

    print(
        f"План: merge(src={args.src!r} {src_info['name']!r}, dst={args.dst!r} {dst_info['name']!r}) — "
        f"перенесёт {src_info['embeddings']} эмбеддинг(ов) голоса под dst, имя dst "
        f"({dst_info['name']!r}) сохраняется, профиль src удаляется целиком — НЕОБРАТИМО. "
        f"Плюс до {harness_preview} факт(ов) профиля из {args.memory_db_path!r} и "
        f"до {legacy_preview} факт(ов) из {args.legacy_facts_db_path!r} "
        "(точное число после отбрасывания конфликтов ключей с dst известно только после --apply)."
    )

    if not args.apply:
        print('Dry-run: ничего не изменено и не создано. Повтори с --apply (и --yes для неинтерактивного запуска).')
        return 0

    try:
        ism = _load_identity_seam_module()
        from rob_box_harness.memory import SQLiteVoiceMemory  # type: ignore  # noqa: E402
    except ImportError as exc:
        print(
            f'ОТКАЗ: rob_box_harness/rob_box_voice недоступны для переноса фактов ({exc!r}). '
            'merge требует оба пакета (docker exec внутри voice-assistant, где ROS сидит в '
            '/ws/install — см. докстринг модуля про ОБА source).',
            file=sys.stderr,
        )
        return 5

    err = _ensure_node_not_running(args.force, args.node_process_pattern)
    if err:
        print(f'ОТКАЗ: {err}', file=sys.stderr)
        return 3

    if not _confirm(
        f"Точно слить {args.src} ({src_info['name']!r}) в {args.dst} (имя {dst_info['name']!r} останется)?",
        args.yes,
    ):
        print('Отменено.')
        return 1

    backups = []
    for path in (args.db_path, args.memory_db_path, args.legacy_facts_db_path):
        if path and os.path.exists(path):
            backups.append(_backup_sqlite_file(Path(path)))
    if backups:
        print('Бэкапы перед изменением: ' + ', '.join(str(b) for b in backups))

    import asyncio

    async def _do_merge() -> Tuple[int, int]:
        db2 = se.SpeakerDatabase(args.db_path)
        store = SQLiteVoiceMemory(db_path=args.memory_db_path)
        try:
            await store.init()
            seam = ism.VoiceIdentitySeam(db2, store, legacy_facts_db_path=args.legacy_facts_db_path)
            return await seam.merge(args.src, args.dst)
        finally:
            db2.close()
            try:
                await store.teardown()
            except Exception:  # noqa: BLE001 — очистка не должна маскировать ошибку слияния
                pass

    try:
        embeddings_moved, facts_moved = asyncio.run(_do_merge())
    except Exception as exc:  # noqa: BLE001 — capability-honest: печатаем и выходим не 0
        print(f'ОШИБКА при слиянии: {exc!r}', file=sys.stderr)
        return 6

    print(
        f'merge выполнен: перенесено {embeddings_moved} эмбеддинг(ов), {facts_moved} факт(ов), '
        f"профиль {args.src} удалён (имя {dst_info['name']!r} сохранено под {args.dst})."
    )
    return 0


# =============================================================================
# delete
# =============================================================================

def cmd_delete(args: argparse.Namespace) -> int:
    if not os.path.exists(args.db_path):
        print(f'ОТКАЗ: БД не найдена: {args.db_path!r}.', file=sys.stderr)
        return 2

    se = _load_speaker_embeddings_module()
    db = se.SpeakerDatabase(args.db_path)
    try:
        speakers = {s['id']: s for s in db.list_speakers()}
    finally:
        db.close()

    if args.speaker_id not in speakers:
        print(f'ОТКАЗ: speaker_id={args.speaker_id!r} не найден в {args.db_path!r}.', file=sys.stderr)
        return 2

    info = speakers[args.speaker_id]
    print(
        f"План: delete({args.speaker_id!r}) удалит профиль {info['name']!r} целиком "
        f"({info['embeddings']} эмбеддинг(ов)) из {args.db_path!r}. НЕОБРАТИМО. "
        f"ПРИМЕЧАНИЕ: факты профиля (scope=speaker:{args.speaker_id} в harness_voice.db/"
        "voice_memory.db) этой командой НЕ удаляются и НЕ переносятся — останутся "
        "осиротевшими; точечной чистки фактов в скоупе этого инструмента нет."
    )

    if not args.apply:
        print('Dry-run: ничего не удалено. Повтори с --apply (и --yes для неинтерактивного запуска).')
        return 0

    err = _ensure_node_not_running(args.force, args.node_process_pattern)
    if err:
        print(f'ОТКАЗ: {err}', file=sys.stderr)
        return 3

    if not _confirm(f"Точно безвозвратно удалить профиль {args.speaker_id} ({info['name']!r})?", args.yes):
        print('Отменено.')
        return 1

    backup = _backup_sqlite_file(Path(args.db_path))
    print(f'Бэкап перед удалением: {backup}')

    db2 = se.SpeakerDatabase(args.db_path)
    try:
        ok = db2.delete_speaker(args.speaker_id)
    finally:
        db2.close()

    print(f'delete({args.speaker_id!r}) -> {ok}')
    return 0 if ok else 4


# =============================================================================
# argparse
# =============================================================================

def _add_common_mutation_args(sub: argparse.ArgumentParser) -> None:
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
            'Пропустить проверку "не запущена ли нода speaker_id_node". Использовать, только '
            'если сам убедился, что нода остановлена — иначе правка будет затёрта/подерётся '
            'с нодой за файл (см. докстринг модуля).'
        ),
    )
    sub.add_argument(
        '--node-process-pattern', default=DEFAULT_NODE_PROCESS_PATTERN,
        help=f'Паттерн для "pgrep -f" при проверке живой ноды (по умолчанию {DEFAULT_NODE_PROCESS_PATTERN!r}).',
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='speaker_db_admin',
        description=(
            'CLI поверх публичного API SpeakerDatabase/VoiceIdentitySeam для обслуживания '
            'голосовой БД дикторов /data/speakers.db (issue #2777) — не ssh+sqlite3 вручную '
            '(issue #2750). Запуск ВНУТРИ контейнера ОБЯЗАН включать оба source, иначе '
            '"ModuleNotFoundError: No module named \'rob_box_voice\'" (пакет живёт в '
            '/ws/install, не /ros2_ws/install): docker exec voice-assistant bash -c '
            '"source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && '
            'python3 /tmp/speaker_db_admin.py list". '
            'ВАЖНО: /data/speakers.e2e.db — ДРУГАЯ база (E2E-прогоны, issue #2759), не путать '
            'с боевой. См. docstring модуля для остальных оговорок (живая нода держит БД '
            'в памяти; формат имени бэкапа не совпадает с чужим ".bak-...Z" из issue #2750).'
        ),
    )
    parser.add_argument(
        '--db-path', default=DEFAULT_DB_PATH,
        help=(
            f'Путь к speakers.db. По умолчанию {DEFAULT_DB_PATH!r} — тот же дефолт, что ROS-'
            'параметр db_path у speaker_id_node. НЕ /data/speakers.e2e.db (issue #2759, другая '
            'база) — передавай явно и осознанно, если действительно нужна E2E-копия.'
        ),
    )

    sub = parser.add_subparsers(dest='command', required=True)

    p_list = sub.add_parser(
        'list', help='Таблица профилей + попарная диагностика возможных дублей одного человека.'
    )
    p_list.add_argument(
        '--no-similarity', action='store_true',
        help='Пропустить попарное сравнение галерей (по умолчанию считается всегда — дёшево, см. докстринг).',
    )
    p_list.add_argument(
        '--similarity-threshold', type=float, default=None,
        help='Порог для пометки пары как вероятного дубля (по умолчанию — IDENTIFY_THRESHOLD из speaker_embeddings.py).',
    )
    p_list.set_defaults(func=cmd_list)

    p_merge = sub.add_parser(
        'merge',
        help='Слить профиль --src в --dst: эмбеддинги + факты (через VoiceIdentitySeam.merge, см. докстринг).',
    )
    p_merge.add_argument('--src', required=True, help='speaker_id профиля-источника (будет удалён).')
    p_merge.add_argument('--dst', required=True, help='speaker_id профиля-получателя (имя сохраняется).')
    p_merge.add_argument(
        '--memory-db-path', default=DEFAULT_MEMORY_DB_PATH,
        help=f'Путь к harness_voice.db (issue #2440). По умолчанию {DEFAULT_MEMORY_DB_PATH!r}.',
    )
    p_merge.add_argument(
        '--legacy-facts-db-path', default=DEFAULT_LEGACY_FACTS_DB_PATH,
        help=f'Путь к voice_memory.db, легаси voice_facts (issue #2751). По умолчанию {DEFAULT_LEGACY_FACTS_DB_PATH!r}.',
    )
    _add_common_mutation_args(p_merge)
    p_merge.set_defaults(func=cmd_merge)

    p_delete = sub.add_parser('delete', help='Удалить профиль целиком (SpeakerDatabase.delete_speaker).')
    p_delete.add_argument('speaker_id', help='speaker_id записи для удаления.')
    _add_common_mutation_args(p_delete)
    p_delete.set_defaults(func=cmd_delete)

    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == '__main__':
    sys.exit(main())
