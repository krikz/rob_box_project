"""Поставка готовых клипов Химеры (ADR-0175 шаг 1.1, карточка #5152).

Профиль собирается из ``chimera_package``: вызов объявляется, только если файл прошёл сверку.
Битый или чужой файл в список не попадает — объявить его было бы ложью.
Синтеза здесь нет: речь, пение и игра по названию этот пакет не обещает.
"""

from __future__ import annotations

import json
import os
from typing import Optional, Sequence, Tuple

from rob_box_mcp_tools import chimera_package as package

from . import chimera_decide as decide

PACKAGE_DIR = package.PACKAGE_DIR
PROFILE_NAME = package.PROFILE_NAME
RATE_HZ = package.RATE_HZ
CHANNELS = package.CHANNELS
SAMPLE_WIDTH = package.SAMPLE_WIDTH

DeliveryCalls = Tuple[Tuple[str, str, str], ...]  # (существо, вызов, имя файла)


def package_dir() -> str:
    """Каталог поставки: переменная окружения перекрывает /ws/sound_pack/chimera, чтобы тест не писал в /ws."""
    return package.package_dir()


def load_delivery(directory: Optional[str] = None) -> decide.ChimeraProfile:
    """Профиль из поставки. Каталога или сверенных вызовов нет — заглушка, ни одного настоящего вызова."""
    root = directory if directory is not None else package_dir()
    calls = package.read_calls(root)
    if not calls:
        return decide.stub_profile()
    return _profile(calls, _body(root))


def delivery_enums(directory: Optional[str] = None) -> Tuple[Tuple[str, ...], Tuple[str, ...], Tuple[str, ...]]:
    """Тройка схемы: действия, существа, вызовы. Тот же каталог, что читает тул. Нет поставки — пусто."""
    return package.enum_values(directory)


def advertised_calls(profile: decide.ChimeraProfile) -> DeliveryCalls:
    """Вызовы, которые профиль реально отдаёт: существо, имя вызова, имя файла. Заглушка — пусто."""
    if profile.stub:
        return ()
    found: list = []
    for creature, names in profile.creatures.items():
        for name in names:
            clip = profile.calls.get(f"{creature}:{name}") or ""
            if clip and not clip.startswith("clip:"):
                found.append((creature, name, clip))
    return tuple(sorted(found))


def verified_clip(directory: str, filename: str) -> bool:
    """Файл объявлен поставкой и прошёл сверку. Нода звука зовёт это, не свой список имён."""
    return package.verified_clip(directory, filename)


def _body(root: str) -> dict:
    path = os.path.join(root, PROFILE_NAME)
    try:
        body = json.loads(open(path, encoding="utf-8").read())
    except (OSError, ValueError):
        return {}
    return body if isinstance(body, dict) else {}


def _profile(calls: DeliveryCalls, body: dict) -> decide.ChimeraProfile:
    creatures: dict = {}
    clips: dict = {}
    for creature, name, filename in calls:
        creatures.setdefault(creature, []).append(name)
        clips[f"{creature}:{name}"] = filename
    note = (
        f"поставка {body.get('package')} {body.get('version')}, "
        f"исходник {body.get('source_sha')}, карточка #{body.get('source_issue')}"
    )
    return decide.ChimeraProfile(
        actions=("call",),  # готовые WAV; речь, пение и игра этот пакет не умеет и не объявляет
        creatures={key: tuple(names) for key, names in creatures.items()},
        calls=clips,
        stub=False,
        note=note,
    )


def call_names(profile: decide.ChimeraProfile) -> Sequence[str]:
    """Имена вызовов для схемы инструмента. Повтор имени у разных существ — один раз."""
    return tuple(dict.fromkeys(name for _creature, name, _file in advertised_calls(profile)))


__all__ = [
    "PACKAGE_DIR",
    "PROFILE_NAME",
    "package_dir",
    "load_delivery",
    "delivery_enums",
    "verified_clip",
    "advertised_calls",
    "call_names",
]
