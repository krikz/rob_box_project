"""Профиль продукта: что делает ядро диалога роботом, колонкой или чужим ассистентом (ADR-0167 §2.4, шаг Ш1б).

Профиль - каталог продукта с ``assistant.yaml`` и файлами рядом; лежит в репо продукта, ядро его только читает.
Формат версии :data:`FORMAT`::

    format: 1
    name: РОББОКС               # имя продукта (как он себя называет)
    wake_words: wake_words.yaml # таблица вейков рядом с профилем (формат ядра: personality / operator)

Три случая, решает :func:`resolve_profile`:

* путь не задан - :data:`NEUTRAL`, нейтральные умолчания ядра: без имени и без вейков. Пустая таблица вейков -
  режим «без вейка» (``words.has_wake_word``: адресовано всё) - честное поведение консоли без продукта: она отвечает
  на каждую реплику, а не молчит в ожидании чужого имени. Строку лога об этом строит код (:data:`NO_PROFILE_LOG`);
* путь задан, а файла нет или он битый - :class:`ProfileError` с понятной причиной: молча откатиться к умолчаниям
  при сломанном профиле - значит запустить робота глухим к своему имени;
* путь задан и файл цел - :class:`ProductProfile`.

Ключи провайдеров в профиль не пишутся никогда (окружение / ``.env`` продукта).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Tuple

from rob_box_dialog.knowledge import WAKE_NAMESPACES, load_wake_word_namespaces

#: Версия формата ``assistant.yaml``. Ломка формата - новая версия и мажор пакета ядра (ADR-0167 §2.4).
FORMAT = 1
#: Имя файла профиля в каталоге продукта.
PROFILE_FILE = "assistant.yaml"
#: Ключи формата: обязательные и необязательные. Чужой ключ - ошибка (опечатка не должна молча выключать вейки).
REQUIRED_KEYS: Tuple[str, ...] = ("format", "name", "wake_words")
OPTIONAL_KEYS: Tuple[str, ...] = ()
#: Строка лога «профиля нет» - одна на все хосты.
NO_PROFILE_LOG = "профиль продукта не задан - умолчания ядра: без имени, вейк выключен (адресовано всё)"


class ProfileError(ValueError):
    """Профиль задан, но прочитать его нельзя: нет файла, не YAML, не тот формат, нет обязательного поля."""


@dataclass(frozen=True)
class ProductProfile:
    """Прочитанный профиль. ``root`` - каталог профиля (``None`` у нейтральных умолчаний)."""

    name: str
    wake_personality: Tuple[str, ...]
    wake_operator: Tuple[str, ...]
    root: Optional[Path] = None

    @property
    def neutral(self) -> bool:
        return self.root is None


#: Нейтральные умолчания ядра: без продукта.
NEUTRAL = ProductProfile(name="", wake_personality=(), wake_operator=())


def _profile_file(path: "str | os.PathLike[str]") -> Path:
    p = Path(path)
    return p / PROFILE_FILE if p.is_dir() else p


def _read_yaml(file: Path) -> Mapping[str, Any]:
    import yaml  # noqa: PLC0415 - лениво, как у таблицы вейков

    if not file.is_file():
        raise ProfileError(f"профиль продукта {file}: файла нет")
    try:
        data = yaml.safe_load(file.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ProfileError(f"профиль продукта {file}: не читается как YAML ({exc})") from exc
    if not isinstance(data, dict):
        raise ProfileError(f"профиль продукта {file}: ждали словарь ключей, а там {type(data).__name__}")
    return data


def _check_keys(file: Path, data: Mapping[str, Any]) -> None:
    missing = [k for k in REQUIRED_KEYS if k not in data]
    unknown = sorted(set(data) - set(REQUIRED_KEYS) - set(OPTIONAL_KEYS))
    if missing or unknown:
        raise ProfileError(f"профиль продукта {file}: нет ключей {missing}, лишние ключи {unknown}")
    if data["format"] != FORMAT:
        raise ProfileError(f"профиль продукта {file}: format {data['format']!r}, ядро понимает {FORMAT}")
    if not isinstance(data["name"], str) or not data["name"].strip():
        raise ProfileError(f"профиль продукта {file}: name - непустая строка")
    if not isinstance(data["wake_words"], str) or not data["wake_words"].strip():
        raise ProfileError(f"профиль продукта {file}: wake_words - имя файла таблицы вейков рядом с профилем")


def _wake_table(file: Path, name: str) -> Tuple[Tuple[str, ...], Tuple[str, ...]]:
    table_file = file.parent / name
    if not table_file.is_file():
        raise ProfileError(f"профиль продукта {file}: таблицы вейков {table_file} нет")
    table = load_wake_word_namespaces(str(table_file))
    if not table.get(WAKE_NAMESPACES[0]):
        raise ProfileError(f"профиль продукта {file}: в {table_file} нет ни одного вейка {WAKE_NAMESPACES[0]}")
    return tuple(table[WAKE_NAMESPACES[0]]), tuple(table.get(WAKE_NAMESPACES[1], ()))


def load_profile(path: "str | os.PathLike[str]") -> ProductProfile:
    """Прочитать профиль: ``path`` - файл ``assistant.yaml`` или каталог с ним. Любая беда - :class:`ProfileError`."""
    file = _profile_file(path)
    data = _read_yaml(file)
    _check_keys(file, data)
    personality, operator = _wake_table(file, data["wake_words"])
    return ProductProfile(
        name=data["name"].strip(), wake_personality=personality, wake_operator=operator, root=file.parent
    )


def resolve_profile(
    path: "str | os.PathLike[str] | None", log: Callable[[str], Any] = lambda _: None
) -> ProductProfile:
    """Профиль хоста: путь не задан - :data:`NEUTRAL` (и строка :data:`NO_PROFILE_LOG` в ``log``), иначе
    :func:`load_profile` (битый профиль - :class:`ProfileError`, без отката к умолчаниям)."""
    if not path or not str(path).strip():
        log(NO_PROFILE_LOG)
        return NEUTRAL
    return load_profile(path)


__all__ = [
    "FORMAT",
    "NEUTRAL",
    "NO_PROFILE_LOG",
    "PROFILE_FILE",
    "ProductProfile",
    "ProfileError",
    "load_profile",
    "resolve_profile",
]
