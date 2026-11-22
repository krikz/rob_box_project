"""Путь базы rtabmap на место (#4346, инкремент 1).

Чистые функции: имя места → безопасный slug → ``/maps/<slug>/rtabmap.db``.
Файлов не открывает и не создаёт — только строки. Тулы ``load_map`` /
``start_mapping`` этот модуль пока не зовут: стартовый файл контейнера
остаётся корнем, переключение файла — следующий инкремент.

Корень ``/maps/rtabmap.db`` — наследие одной базы (#4392), не место.
:func:`place_db_path` его не возвращает и не удаляет. Сам путь наследия —
:data:`LEGACY_DB_PATH` / :func:`legacy_db_path`, чтобы в коде было видно,
что это не путь места.
"""

from __future__ import annotations

import re
import unicodedata

#: Каталог баз на томе ``./maps:/maps``. Не корень файла наследия.
MAPS_ROOT = "/maps"
#: Имя файла базы внутри каталога места. То же имя, что у наследия, но не в корне.
PLACE_DB_FILENAME = "rtabmap.db"

#: Наследие одной базы (#4392). Не путь места: :func:`place_db_path` его не строит.
LEGACY_DB_PATH = "/maps/rtabmap.db"

#: Что остаётся в slug после нормализации. ``.`` нет — иначе ``..`` пролезет.
_SLUG_ALPHABET = "abcdefghijklmnopqrstuvwxyz0123456789_-"
_SLUG_RE = re.compile(rf"[^{re.escape(_SLUG_ALPHABET)}]+")
_MULTI_DASH = re.compile(r"-{2,}")

#: Кириллица → латиница, побуквенно и без регистра на входе (ниже casefold).
#: «х»/«Х» оба дают ``h``: путь места не должен зависеть от регистра имени.
_CYRILLIC_TO_LATIN = {
    "а": "a",
    "б": "b",
    "в": "v",
    "г": "g",
    "д": "d",
    "е": "e",
    "ё": "e",
    "ж": "zh",
    "з": "z",
    "и": "i",
    "й": "j",
    "к": "k",
    "л": "l",
    "м": "m",
    "н": "n",
    "о": "o",
    "п": "p",
    "р": "r",
    "с": "s",
    "т": "t",
    "у": "u",
    "ф": "f",
    "х": "h",
    "ц": "c",
    "ч": "ch",
    "ш": "sh",
    "щ": "sch",
    "ъ": "",
    "ы": "y",
    "ь": "",
    "э": "e",
    "ю": "ju",
    "я": "ja",
}


class PlacePathError(ValueError):
    """Имя места нельзя превратить в безопасный путь базы."""


def legacy_db_path() -> str:
    """Путь наследия одной базы. Не путь места и не кандидат на удаление."""
    return LEGACY_DB_PATH


def is_legacy_db_path(path: str) -> bool:
    """``True``, если ``path`` — корень наследия, а не база места."""
    return path == LEGACY_DB_PATH


def place_slug(name: str) -> str:
    """Имя места → slug из ``[a-z0-9_-]``.

    Регистр снимается, пробелы становятся ``-``, кириллица транслитерируется
    побуквенно. Пустое имя, путь (``/``, ``\\``), ``..`` и всё, из чего после
    нормализации не осталось безопасных символов, — :class:`PlacePathError`.
    Файл не создаёт.
    """
    if not isinstance(name, str):
        raise PlacePathError("имя места должно быть строкой")
    stripped = name.strip()
    if not stripped:
        raise PlacePathError("пустое имя места")
    if "/" in stripped or "\\" in stripped or ".." in stripped:
        raise PlacePathError("имя места не путь: слэш и '..' запрещены")

    folded = stripped.casefold()
    transliterated = "".join(_CYRILLIC_TO_LATIN.get(ch, ch) for ch in folded)
    normalized = unicodedata.normalize("NFKD", transliterated)
    without_marks = "".join(ch for ch in normalized if not unicodedata.combining(ch))
    lowered = without_marks.lower().replace(" ", "-")
    slug = _SLUG_RE.sub("-", lowered)
    slug = _MULTI_DASH.sub("-", slug).strip("-_")
    if not slug or not all(ch in _SLUG_ALPHABET for ch in slug):
        raise PlacePathError("из имени места не получился безопасный slug")
    if slug in {".", ".."} or ".." in slug:
        raise PlacePathError("slug места не может быть '..'")
    return slug


def place_db_path(name: str) -> str:
    """Путь базы места: ``/maps/<slug>/rtabmap.db``. Корень наследия не возвращает."""
    slug = place_slug(name)
    path = f"{MAPS_ROOT}/{slug}/{PLACE_DB_FILENAME}"
    if path == LEGACY_DB_PATH or is_legacy_db_path(path):
        raise PlacePathError("путь места совпал с наследием одной базы")
    return path
