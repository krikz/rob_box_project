"""sample_dave.py — белый список сэмплов DJ_Dave для ``loop()`` (#3219).

Пак ``dj_dave`` (вокальные стемы algorave-dave, стемы Array от Lil Data,
tech/psr/hh/cp из Dirt-Samples, хэты и клэпы TR-808/DDM110) кладёт на хост
Ресурсный пак (запись ``dj-dave-samples``, ``/opt/rob_box/samples/dj_dave``).
Файлов в git нет: у источников нет лицензии, репозиторий публичный.

Как сэмпл доезжает до звука — тот же путь, что у pack 1
(:mod:`core.sample_loops`): ``loop()`` ищет файл от папки лупов пака 0, и
относительный путь через ``..`` (``../../dj_dave/<файл>``) резолвит
файловая система. ``spack=`` тут не поможет (no-op, см. #2841).

Флаг тот же, что у лупов и FX (``ROB_BOX_PACK1_LOOPS``): ни один сэмпл не
прослушан на 16 kHz DAC робота, отдельная ручка для «ещё не прослушано»
только множит состояния.

Каталог — одна таблица знания ``rob_box_music.knowledge.SAMPLE_CATALOG``
(ADR-0149 §3.11, §8.1, PR-3d; данные ``rob_box_music/data/sample_dave.json``):
роль, длина, темп и тональность там; здесь — только вид каталога для
санитайзера ``loop()``. Соответствие имён файлам на хосте держит тест против
lock-файла фетчера (``docker/vision/scripts/resource_pack/dj_dave_samples.lock.json``).
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, Optional, Tuple

from rob_box_music import knowledge as kn

from .sample_loops import PACK1_LOOPS_ENV


@dataclass(frozen=True)
class DaveSample:
    """Один сэмпл пака.

    Attributes:
        name: короткое имя (``algorave_spilltab``) — то, что пишет модель.
        group: группа каталога (``array_vox``, ``dirt_psr`` …).
        seconds: длина файла (ffprobe с реально скачанного файла).
        channels: число каналов файла (loop читает буфер как стерео).
        path: аргумент ``loop(...)``: путь от папки лупов пака 0.
    """

    name: str
    group: str
    seconds: float
    channels: int
    path: str


@lru_cache(maxsize=1)
def sample_catalog() -> Dict[str, DaveSample]:
    """Каталог пака — вид ``knowledge.SAMPLE_CATALOG`` (кешируется)."""
    return {
        name: DaveSample(name=name, group=info.group, seconds=info.seconds, channels=info.channels,
                         path=info.loop_arg)
        for name, info in kn.SAMPLE_CATALOG.items()
    }


def group_descriptions() -> Dict[str, str]:
    """Группа → человекочитаемое описание (для подсказок модели)."""
    return dict(kn.SAMPLE_GROUPS)


def find_sample(arg: str) -> Optional[DaveSample]:
    """Найти сэмпл по имени или по каноническому пути (повторный прогон)."""
    text = arg.strip()
    catalog = sample_catalog()
    info = catalog.get(text)
    if info is not None:
        return info
    return next((i for i in catalog.values() if i.path == text), None)


def sample_denial(arg: str, enabled: bool) -> Optional[str]:
    """Причина отказа для ``loop(arg)`` или ``None``, если можно играть."""
    info = find_sample(arg)
    if info is None:
        return (
            f"Сэмпла DJ_Dave {arg!r} нет в каталоге — Renardo его не найдёт. "
            f"Группы: {', '.join(sorted(group_descriptions()))}."
        )
    if not enabled:
        return (
            f"Сэмпл DJ_Dave {info.name!r} ещё не прослушан на роботе (16 kHz "
            f"DAC) — пак выключен (флаг {PACK1_LOOPS_ENV}). Играй без него."
        )
    return None


def names_in_group(group: str) -> Tuple[str, ...]:
    """Имена сэмплов группы (пусто, если группы нет)."""
    return tuple(i.name for i in sample_catalog().values() if i.group == group)


__all__ = [
    "DaveSample",
    "find_sample",
    "group_descriptions",
    "names_in_group",
    "sample_catalog",
    "sample_denial",
]
