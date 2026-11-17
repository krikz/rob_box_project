#!/usr/bin/env python3
"""hailo_shm_reap.py — убрать из /dev/shm буферы HailoRT, брошенные прошлым запуском.

Issue #4277 (03.11.2026): vision-hailo и vision-face падали в цикле с
SIGBUS (``exit code -7``) сразу после первого кадра. Причина — не образ,
а забитый tmpfs: ``/dev/shm`` хоста Vision Pi (3.9 ГБ) на 100 % занят
файлами вида ``yolov8n_conv52_97_4_628_628``,
``retinaface_mobilenet_v1_input_layer1_7_53_53``,
``arcface_mobilefacenet_fc1_0_101_101`` — 38 тыс. штук с 19.10 (uptime
хоста 15 дней). Это буферы потоков сети, которые клиент HailoRT заводит
под ``multi_process_service`` (hailo_device.py, режим ``service``). Когда
процесс умирает (рестарт контейнера, деплой, падение), файлы остаются:
``/dev/shm`` смонтирован с хоста (docker-compose.yaml), и чистит его только
перезагрузка Pi. Новый запуск создаёт свои буферы, ``mmap`` проходит,
а первая запись в страницу, на которую у tmpfs уже нет места, — SIGBUS.

Каждая модель живёт ровно в одном контейнере (yolov8n — vision-hailo,
retinaface/arcface — vision-face), и в момент старта контейнера прошлый
процесс этого контейнера уже мёртв. Значит, всё с префиксом «своих» HEF
— мусор прошлых запусков, и его можно удалить до запуска ноды. Чужие
модели и прочие файлы (SuperCollider, jack) не трогаем.

Имя буфера: ``<стем HEF>_<поток>_<индекс>_<N>_<N>``. Удаляем только
обычные файлы, у которых имя начинается с ``<стем>_`` и кончается двумя
числами через ``_`` — это сужает совпадение до формата HailoRT.

Никогда не роняет старт: ошибки по файлам пишутся в лог, код выхода 0.
После уборки печатает свободное место в /dev/shm; если его меньше
``--min-free-mb``, пишет WARN с крупнейшими владельцами — оператор видит,
почему нода упадёт, вместо голого ``exit code -7``.

Использование:
    hailo_shm_reap.py [--shm-dir /dev/shm] [--tag TAG] HEF [HEF ...]
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

DEFAULT_SHM_DIR = '/dev/shm'
#: Ниже этого после уборки — WARN. yolov8n берёт ~45 МБ на запуск,
#: retinaface+arcface ~300 МБ (замер по файлам 03.11.2026).
DEFAULT_MIN_FREE_MB = 512

_TAIL = re.compile(r'_\d+_\d+$')


def model_stems(hef_paths: Iterable[str]) -> List[str]:
    """Стемы HEF (``/x/yolov8n.hef`` → ``yolov8n``), без пустых и дублей."""
    stems: List[str] = []
    for raw in hef_paths:
        raw = (raw or '').strip()
        if not raw:
            continue
        stem = Path(raw).name
        if stem.endswith('.hef'):
            stem = stem[: -len('.hef')]
        if stem and stem not in stems:
            stems.append(stem)
    return stems


def is_stale_buffer(name: str, stems: Sequence[str]) -> bool:
    """Имя — буфер HailoRT одной из ``stems``?"""
    if not _TAIL.search(name):
        return False
    return any(name.startswith(stem + '_') for stem in stems)


def _used_bytes(st: os.stat_result) -> int:
    # Буферы tmpfs разреженные: размер по ls врёт, считаем блоки.
    blocks = getattr(st, 'st_blocks', None)
    return blocks * 512 if blocks is not None else st.st_size


def reap(shm_dir: str, stems: Sequence[str]) -> Tuple[int, int, List[str]]:
    """Удалить буферы ``stems`` в ``shm_dir``.

    Возвращает (удалено файлов, освобождено байт, ошибки).
    """
    removed = 0
    freed = 0
    errors: List[str] = []
    if not stems:
        return removed, freed, errors
    try:
        entries = list(os.scandir(shm_dir))
    except OSError as exc:
        return removed, freed, [f'{shm_dir}: {exc}']
    for entry in entries:
        if not is_stale_buffer(entry.name, stems):
            continue
        try:
            if not entry.is_file(follow_symlinks=False):
                continue
            used = _used_bytes(entry.stat(follow_symlinks=False))
            os.unlink(entry.path)
        except OSError as exc:
            errors.append(f'{entry.name}: {exc}')
            continue
        removed += 1
        freed += used
    return removed, freed, errors


def free_mb(shm_dir: str) -> Optional[float]:
    try:
        st = os.statvfs(shm_dir)
    except (OSError, AttributeError):
        return None
    return st.f_bavail * st.f_frsize / (1024 * 1024)


def top_owners(shm_dir: str, limit: int = 5) -> List[Tuple[str, float]]:
    """Крупнейшие группы файлов по префиксу до первого числа (МБ)."""
    usage: Counter = Counter()
    try:
        entries = list(os.scandir(shm_dir))
    except OSError:
        return []
    for entry in entries:
        try:
            if not entry.is_file(follow_symlinks=False):
                continue
            used = _used_bytes(entry.stat(follow_symlinks=False))
        except OSError:
            continue
        key = re.split(r'_\d', entry.name, maxsplit=1)[0]
        usage[key] += used
    return [(k, v / (1024 * 1024)) for k, v in usage.most_common(limit)]


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('hef', nargs='*', help='пути HEF этого контейнера')
    ap.add_argument('--shm-dir', default=os.environ.get(
        'HAILO_SHM_DIR', DEFAULT_SHM_DIR))
    ap.add_argument('--tag', default='[hailo_shm_reap]')
    ap.add_argument('--min-free-mb', type=float, default=DEFAULT_MIN_FREE_MB)
    args = ap.parse_args(argv)

    tag = args.tag
    stems = model_stems(args.hef)
    if not stems:
        print(f'{tag} shm reap: HEF не заданы — убирать нечего')
        return 0

    before = free_mb(args.shm_dir)
    removed, freed, errors = reap(args.shm_dir, stems)
    after = free_mb(args.shm_dir)

    def _fmt(v: Optional[float]) -> str:
        return 'n/a' if v is None else f'{v:.0f}'

    print(
        f'{tag} shm reap {args.shm_dir}: models={",".join(stems)} '
        f'removed={removed} freed={freed / (1024 * 1024):.0f}MB '
        f'free_before={_fmt(before)}MB free_after={_fmt(after)}MB'
    )
    for err in errors[:10]:
        print(f'{tag} WARN: shm reap: {err}', file=sys.stderr)
    if len(errors) > 10:
        print(f'{tag} WARN: shm reap: ещё {len(errors) - 10} ошибок',
              file=sys.stderr)
    if after is not None and after < args.min_free_mb:
        owners = ', '.join(f'{k}={v:.0f}MB' for k, v in
                           top_owners(args.shm_dir))
        print(
            f'{tag} WARN: в {args.shm_dir} свободно {after:.0f} МБ '
            f'< {args.min_free_mb:.0f} МБ — HailoRT может упасть с SIGBUS '
            f'(exit code -7, #4277). Крупнейшие: {owners}',
            file=sys.stderr,
        )
    return 0


if __name__ == '__main__':
    sys.exit(main())
