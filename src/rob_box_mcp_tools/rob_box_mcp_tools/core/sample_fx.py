"""sample_fx.py — белый список FX-одиночек pack 1 (#2968).

Почему модуль вообще нужен
--------------------------
Живой сет 24.09.2026: Шифу попросил «добавь выстрелы пистолетов» на
гангста-вечеринке — модель не вызвала ни одного тула (см. #2968), но даже
попытавшись, упёрлась бы в то же самое, что #2841 нашёл для лупов:
``search_samples`` для пака 1 отдаёт ``play_code`` со ``spack=1``, а в
установленной ``renardo_lib`` ``spack`` — no-op (``getBufferFromSymbol``
всегда строит путь от ``0_foxdot_default``, см. ``core/sample_loops.py``).

Рабочий путь тот же, что у лупов: относительный путь от папки лупов пака 0
через ``..`` — ``BufferManager._searchPaths`` резолвит его файловой
системой, и ``loop(...)`` синт проигрывает файл ОДИН раз за ``dur``-битовый
цикл, если не растягивать его (``beat_stretch`` не задан/0) — то есть тот
же синт, что несёт жанровые лупы, годится и для редкого одиночного акцента,
если не просить его растягиваться и звучать в каждом такте.

Флаг — тот же, что у лупов (:data:`core.sample_loops.PACK1_LOOPS_ENV`):
докстринг ``sample_loops.py`` уже фиксирует «имя флага историческое —
теперь он открывает весь каталог [пака 1]». Отдельного флага для FX заводить
не нужно и вредно — две ручки для одного и того же «пак 1 не прослушан»
только множат состояния, которые надо держать синхронными.

Пути в ``data/sample_fx.json`` сняты чтением файлового дерева в контейнере
``voice-assistant`` (``ssh 10.1.1.21`` → ``docker exec ... ls``, 24.09.2026,
живое подтверждение в PR) — имена файлов точные, но НИ ОДИН сэмпл не
прослушан на 16 kHz DAC робота. Каталог — данные, не код: лишнее при
прослушке удаляется из JSON без правки этого модуля.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple

from .sample_loops import PACK1_LOOPS_ENV, pack1_loops_enabled

_CATALOG_FILE = Path(__file__).resolve().parent.parent / "data" / "sample_fx.json"


@dataclass(frozen=True)
class FxInfo:
    """Один FX-сэмпл из каталога.

    Attributes:
        name: короткое имя (``gunshot_1``) — то, что пишет модель.
        tags: жанровые метки (``gangsta``, ``dnb`` …) для подбора без
            перечисления имён в промпте.
        path: аргумент для ``loop(...)``, который Renardo реально найдёт —
            относительный путь от папки лупов пака 0 (та же схема, что
            :attr:`core.sample_loops.LoopInfo.path`).
    """

    name: str
    tags: Tuple[str, ...] = field(default_factory=tuple)
    path: str = ""


@lru_cache(maxsize=1)
def fx_catalog() -> Dict[str, FxInfo]:
    """Прочитать каталог FX из ``data/`` пакета (кешируется).

    Путь от ``__file__``, не ``importlib.resources`` — та же причина, что в
    :func:`core.sample_loops.loop_catalog`: тесты музыки подменяют часть
    ``sys.modules``, и пакет не zip-safe.
    """
    raw = json.loads(_CATALOG_FILE.read_text(encoding="utf-8"))
    pack1_dir = str(raw["pack1_dir"])
    catalog: Dict[str, FxInfo] = {}
    for name, meta in raw["fx"].items():
        rel = str(meta["path"])
        tags = tuple(meta.get("tags") or ())
        catalog[name] = FxInfo(name, tags, f"../../{pack1_dir}/{rel}")
    return catalog


def find_fx(arg: str) -> Optional[FxInfo]:
    """Найти FX по тому, что написано в ``fx=``/``loop(...)``.

    Принимает короткое имя (``gunshot_1``) и канонический путь из
    :attr:`FxInfo.path` — последнее нужно, чтобы уже переписанный
    санитайзером код проходил повторную проверку (см. :func:`find_loop`).

    🔴 В отличие от :func:`core.sample_loops.find_loop`, канонический путь
    FX САМ заканчивается на ``.wav`` (лупы адресуются без расширения). Имя
    сначала ищется как есть и как путь целиком, и только потом со снятым
    ``.wav`` — иначе повторный прогон уже переписанного пути ломался: снятие
    расширения раньше сравнения с путём превращало
    ``".../005_..._Kaonaya.wav"`` в несуществующий ключ без совпадения ни с
    одним именем, ни с одним путём каталога.
    """
    text = arg.strip()
    catalog = fx_catalog()
    info = catalog.get(text)
    if info is not None:
        return info
    by_path = next((i for i in catalog.values() if i.path == text), None)
    if by_path is not None:
        return by_path
    if text.lower().endswith(".wav"):
        return catalog.get(text[:-4])
    return None


def fx_denial(arg: str, enabled: bool) -> Optional[str]:
    """Причина отказа для FX ``arg`` или ``None``, если можно играть."""
    info = find_fx(arg)
    if info is None:
        known = ", ".join(sorted(fx_catalog()))
        return (
            f"FX {arg!r} нет в белом списке — Renardo его не найдёт. "
            f"Доступные имена: {known}."
        )
    if not enabled:
        return (
            f"FX {info.name!r} ещё не прослушан на роботе (16 kHz DAC) — "
            f"FX-одиночки выключены (флаг {PACK1_LOOPS_ENV}, тот же, что и "
            "у лупов пака 1). Играй без fx."
        )
    return None


def fx_enabled(environ: Optional[Mapping[str, str]] = None) -> bool:
    """Включён ли белый список FX — тот же флаг, что у лупов пака 1."""
    return pack1_loops_enabled(environ)


def tags_for(name: str) -> Tuple[str, ...]:
    """Жанровые метки FX-сэмпла (пусто, если имени нет в каталоге)."""
    info = fx_catalog().get(name)
    return info.tags if info is not None else ()


def fx_by_genre(genre: str) -> Tuple[str, ...]:
    """Имена FX, у которых в тегах есть данный жанр (без учёта регистра)."""
    needle = genre.strip().lower()
    if not needle:
        return ()
    return tuple(
        info.name
        for info in fx_catalog().values()
        if needle in {t.lower() for t in info.tags}
    )


__all__ = [
    "FxInfo",
    "find_fx",
    "fx_by_genre",
    "fx_catalog",
    "fx_denial",
    "fx_enabled",
    "tags_for",
]
