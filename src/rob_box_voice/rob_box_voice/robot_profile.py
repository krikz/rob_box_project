"""Профиль продукта робота: где он лежит у хостов робота (ADR-0167 §2.4, шаг Ш1б).

Формат и загрузчик - в ядре (``rob_box_dialog.profile``); здесь только знание робота: в контейнерах ``voice-assistant``
и ``supervisor`` каталог ``docker/sound/config`` смонтирован в ``/config``, профиль - :data:`PATH`. Env :data:`ENV`
подменяет путь (тесты, стенд из чекаута), как ``VOICE_MEMORY_DB_PATH`` у БД инструментов. Хосты робота
(``dialog_node``, ``stt_node``, ТАРС супервизора) грузят профиль строго: нет или битый - ``ProfileError`` на старте,
а не нейтральные умолчания ядра (вейк выключен), на которых робот отвечал бы на каждую фразу мастерской.
"""

from __future__ import annotations

import os
from typing import Optional

from rob_box_dialog.profile import ProductProfile, load_profile

#: Профиль робота в контейнере (``./config:/config:ro``).
PATH = "/config/assistant.yaml"
#: Env, подменяющий :data:`PATH` (пустое значение - как не задан).
ENV = "ROB_BOX_PROFILE"


def path() -> str:
    """Путь к профилю робота: :data:`ENV`, если задан и не пуст, иначе :data:`PATH`."""
    return os.environ.get(ENV, "").strip() or PATH


def load(where: Optional[str] = None) -> ProductProfile:
    """Профиль робота по ``where`` (параметр хоста) или :func:`path`; нет или битый - ``ProfileError``."""
    return load_profile(where or path())


__all__ = ["ENV", "PATH", "load", "path"]
