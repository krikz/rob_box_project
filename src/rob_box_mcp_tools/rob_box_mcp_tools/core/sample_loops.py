"""sample_loops.py — каталог лупов для ``loop()`` и белый список pack 1 (#2841).

Почему модуль вообще нужен
--------------------------
Живой сет 23.09.2026: 178 плееров, ``loop()`` — 0 раз, ``spack=`` — 0 раз.
На роботе при этом лежат 25 жанровых лупов пака ``1_pitchglitch_samples``.
До них нельзя было дотянуться двумя путями сразу:

1. ``spack=1`` жёстко запрещён санитайзером (с 20.08). Но и без запрета он
   бы не помог: в установленной сборке ``renardo_lib``
   ``BufferManager.getBufferFromSymbol(symbol, spack, index)`` принимает
   ``spack`` и НЕ использует его — путь всегда строится от
   ``0_foxdot_default`` (``renardo_gatherer.collections.sample_path_from_symbol``
   вызывается без ``spack_path``). ``spack=1`` молча играл бы пак 0.
2. ``loop('dnb_1')`` ищет файл только в ``default_loop_path()`` =
   ``samples/0_foxdot_default/_loop_``. Имя лупа пака 1 не находится,
   ``loadBuffer`` возвращает буфер 0 — тишина без ошибки.

Рабочий путь один: ``loop()`` с путём ОТНОСИТЕЛЬНО папки лупов пака 0
(``../../1_pitchglitch_samples/_loop_/dnb_1``) — ``BufferManager._searchPaths``
делает ``join(root, filename)``, и ``..`` разрешается файловой системой.
Путь относительный, а не абсолютный, чтобы не зависеть от ``$HOME``
контейнера (сейчас оба контейнера — voice-assistant и supercollider —
монтируют ``/opt/rob_box/samples`` в ``/root/.config/renardo/samples``).

Флаг
----
Исходный бан pack 1 объяснён словами («сырой глитчевый звук на 16 kHz
DAC»), но не записью звука — на слух его никто не проверял. Поэтому весь
pack-1 список — только КАНДИДАТЫ и включается переменной окружения
``ROB_BOX_PACK1_LOOPS=1``; по умолчанию выключен.

За тем же флагом и ``foxdot`` из пака 0: ВСЕ 26 файлов каталога моно
(python wave на Vision Pi, 23.09.2026), а renardo-синт ``loop`` читает
буфер как ``PlayBuf.ar(2, buf, ...)`` (sclang_code/scsynth/loop.scd).
Буфер грузится ``/b_allocRead`` без размножения каналов, т.е. одноканальным,
а в ``DelayUGens.so`` scsynth есть проверка "Buffer UGen channel mismatch:
expected %i, yet buffer has %i channels" — по исходникам SC такая UGen
молчит. На слух не проверено, поэтому «всегда можно» было бы тихой
деградацией: тул рапортует «играю», слой беззвучен (capability-honest).
Имя флага историческое — теперь он открывает весь каталог.

Каталог — данные (``data/sample_loops.json``), а не код: после прослушки
лишние кандидаты удаляются из файла без правки логики.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, Mapping, Optional

#: Переменная окружения, включающая белый список pack 1.
PACK1_LOOPS_ENV = "ROB_BOX_PACK1_LOOPS"
_TRUTHY = frozenset({"1", "true", "yes", "on"})

_CATALOG_FILE = Path(__file__).resolve().parent.parent / "data" / "sample_loops.json"

#: Допустимые длины лупа в битах: степени двойки от доли до 8 тактов.
#: Не-степень двойки (3, 6, 12 битов) рвёт квадрат относительно ударных.
_LOOP_BEAT_CHOICES = (1.0, 2.0, 4.0, 8.0, 16.0, 32.0)


@dataclass(frozen=True)
class LoopInfo:
    """Один луп из каталога.

    Attributes:
        name: короткое имя (``dnb_1``) — то, что пишет модель.
        pack: 0 — дефолтный пак (всегда можно), 1 — pitchglitch (флаг).
        seconds: длина файла в секундах (снята на роботе).
        path: аргумент для ``loop(...)``, который Renardo реально найдёт.
    """

    name: str
    pack: int
    seconds: float
    path: str


@lru_cache(maxsize=1)
def loop_catalog() -> Dict[str, LoopInfo]:
    """Прочитать каталог лупов из ``data/`` пакета (кешируется).

    Путь от ``__file__``, а не ``importlib.resources``: тесты музыки
    подменяют часть ``sys.modules``, и ``resources.files`` там падает на
    импорте ``importlib.readers``. Пакет не zip-safe (setup.py), файл
    всегда лежит рядом на диске.
    """
    raw = json.loads(_CATALOG_FILE.read_text(encoding="utf-8"))
    pack1_dir = str(raw["pack1_dir"])
    catalog: Dict[str, LoopInfo] = {}
    for name, meta in raw["pack0"].items():
        catalog[name] = LoopInfo(name, 0, float(meta["seconds"]), name)
    for name, meta in raw["pack1"].items():
        path = f"../../{pack1_dir}/_loop_/{name}"
        catalog[name] = LoopInfo(name, 1, float(meta["seconds"]), path)
    return catalog


def pack1_loops_enabled(environ: Optional[Mapping[str, str]] = None) -> bool:
    """Включён ли белый список pack 1 (``ROB_BOX_PACK1_LOOPS``)."""
    env = os.environ if environ is None else environ
    return env.get(PACK1_LOOPS_ENV, "").strip().lower() in _TRUTHY


def find_loop(arg: str) -> Optional[LoopInfo]:
    """Найти луп по тому, что написано в ``loop(...)``.

    Принимает короткое имя (``dnb_1``), имя с ``.wav`` и канонический путь
    из :attr:`LoopInfo.path` — последнее нужно, чтобы уже переписанный
    санитайзером код проходил повторную проверку.
    """
    text = arg.strip()
    if text.lower().endswith(".wav"):
        text = text[:-4]
    catalog = loop_catalog()
    info = catalog.get(text)
    if info is not None:
        return info
    return next((i for i in catalog.values() if i.path == text), None)


def loop_denial(arg: str, enabled: bool) -> Optional[str]:
    """Причина отказа для ``loop(arg)`` или ``None``, если луп можно играть."""
    info = find_loop(arg)
    if info is None:
        known = ", ".join(sorted(loop_catalog()))
        return (
            f"Лупа {arg!r} нет в каталоге — Renardo его не найдёт и сыграет "
            f"тишину без ошибки. Доступные имена: {known}."
        )
    if not enabled:
        return (
            f"Луп {info.name!r} ещё не прослушан на роботе (16 kHz DAC, "
            "моно-файл в стерео-плеере loop) — лупы выключены "
            f"(флаг {PACK1_LOOPS_ENV}). Играй без лупа."
        )
    return None


def loop_beats(info: LoopInfo, bpm: float) -> float:
    """Длина лупа в битах под темп: ближайшая степень двойки.

    ``loop(..., beat_stretch=1)`` растягивает файл ровно на ``dur`` битов
    через скорость воспроизведения (``PlayBuf`` — меняется и высота).
    Ближайшая в логарифмической шкале степень двойки даёт наименьшее
    отклонение скорости от исходной, то есть наименьший сдвиг высоты.
    """
    native = max(info.seconds * float(bpm) / 60.0, 1e-6)
    return min(_LOOP_BEAT_CHOICES, key=lambda beats: abs(math.log2(beats / native)))


def loop_rate(info: LoopInfo, bpm: float) -> float:
    """Во сколько раз ``beat_stretch`` ускорит файл (>1 — быстрее и выше)."""
    native = info.seconds * float(bpm) / 60.0
    return native / loop_beats(info, bpm)


__all__ = [
    "LoopInfo",
    "PACK1_LOOPS_ENV",
    "find_loop",
    "loop_beats",
    "loop_catalog",
    "loop_denial",
    "loop_rate",
    "pack1_loops_enabled",
]
