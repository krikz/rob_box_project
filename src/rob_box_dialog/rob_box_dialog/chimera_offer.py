"""Текст и поля ``chimera_perform``, которые видит модель, из списка действий схемы (ADR-0148, #5152).

Один модуль на описание и на то, какие поля этому списку принадлежат: каталог, тул и форма хода
импортируют его и не держат свою копию. Без ROS и без ``rob_box_music``: ядро ставит этот пакет целиком.
"""

from __future__ import annotations

import copy
from typing import Any, Dict, Mapping, Sequence, Tuple

#: Действия одного инструмента. Тот же порядок, что ``chimera_decide.ACTIONS``; сверку держит тест.
ACTIONS: Tuple[str, ...] = ("call", "speak", "sing", "play")

_ACTION_LINE = {
    "call": "call — короткий вызов существа, creature и call из того же списка",
    "speak": "speak — подготовленная фраза, phrase_id и тембр из того же списка",
    "sing": "sing — пение по названию, title, тембр и слоги из того же списка",
    "play": "play — игра по названию, title из того же списка",
}
_NOT_OFFERED = {
    "call": "короткий вызов",
    "speak": "речь",
    "sing": "пение",
    "play": "игра по названию",
}
#: Поле схемы принадлежит действию. Нет действия в enum — поля в схеме модели нет: иначе «только call»
#: всё равно показывает слоги и название пения. ``execute`` аргументы принимает: сужает показ, не вызов.
_FIELD_ACTIONS = {
    "creature": ("call",),
    "call": ("call",),
    "phrase_id": ("speak",),
    "timbre": ("speak", "sing"),
    "title": ("sing", "play"),
    "syllables": ("sing",),
}


def fields_for(actions: Sequence[str]) -> Tuple[str, ...]:
    """Имена полей, которые этот список действий вправе показать модели.

    ``action`` здесь нет: он сам и есть список. Неизвестное имя действия полей не добавляет.
    """
    offered = set(actions)
    return tuple(name for name, owners in _FIELD_ACTIONS.items() if offered.intersection(owners))


def offered_schema(schema: Mapping[str, Any], actions: Sequence[str]) -> Dict[str, Any]:
    """Схема с полями только объявленных действий. Чужие ключи ``properties`` уходят, ``required`` тоже.

    Списка действий нет — остаются только обязательные поля исходной схемы (сам ``action``).
    """
    out = copy.deepcopy(dict(schema))
    props = out.get("properties")
    if not isinstance(props, dict):
        return out
    keep = set(fields_for(actions)) | {"action"}
    for name in [name for name in props if name not in keep]:
        props.pop(name, None)
    required = out.get("required")
    if isinstance(required, list):
        out["required"] = [name for name in required if name in props]
    return out


def perform_description(actions: Sequence[str]) -> str:
    """Текст тула из того же списка действий, что enum схемы.

    Действия вне ``ACTIONS`` в текст не входят: их нет в инструменте. Пустой список —
    честный отказ, не обещание call.
    """
    offered = [action for action in ACTIONS if action in set(actions)]
    if not offered:
        return (
            "Разовое выступление Химерой. Поставка не объявила ни одного действия: "
            "вызов, речь, пение и игра не обещаны. «Сказал» — только после конца "
            "проигрывания, не после старта. Произвольный русский текст не поддержан."
        )
    missing = [_NOT_OFFERED[action] for action in ACTIONS if action not in offered]
    denied = ""
    if missing:
        denied = " " + ", ".join(missing).capitalize() + " эта поставка не обещает и в список действий не входит."
    lines = "; ".join(_ACTION_LINE[action] for action in offered)
    return (
        "Разовое выступление Химерой. action берётся из поставки: сейчас это "
        f"{lines}.{denied} «Сказал» — только после конца проигрывания, не после старта. "
        "Произвольный русский текст не поддержан."
    )


#: Точные формы глагола, которым реплика просит Химеру. Не основы и не вся таблица ``SPEECH_ASK_VERBS``:
#: «расскажи анекдот химерой» — речь робота, не подготовленная фраза. «сыграй» — те же четыре формы, что
#: ``rob_box_music.knowledge.SING_PLAY_VERBS``; диалог музыку не импортирует, копия живёт здесь, сверку держит тест.
#: «спой» здесь нет: пение решает живой маршрут хоста (``sing_family``, голова ``live``).
SAY_FORMS: Tuple[str, ...] = ("скажи", "скажите", "сказать", "скажи-ка")
PLAY_FORMS: Tuple[str, ...] = ("сыграй", "сыграйте", "сыграть", "сыграешь")
_VERB_ACTION = (("speak", SAY_FORMS), ("play", PLAY_FORMS))


def verb_action(text: str) -> str:
    """Действие Химеры, которое реплика просит глаголом, или ``""``.

    «скажи» / «скажите» — ``speak``, «сыграй» / «сыграйте» — ``play``. Отрицание прямо перед глаголом
    («не скажи», «не сыграй») просьбой не является. Два разных глагола — ``""``: код не выбирает молча.
    Новой регулярки нет: формы ловит :func:`rob_box_dialog.words.has_form`.
    """
    from rob_box_dialog.words import has_form, name_tokens  # noqa: PLC0415 — words импортирует knowledge

    tokens = name_tokens(text)
    hits = []
    for action, forms in _VERB_ACTION:
        if not has_form(text or "", forms):
            continue
        if any(token in forms and (i == 0 or tokens[i - 1] != "не") for i, token in enumerate(tokens)):
            hits.append(action)
    return hits[0] if len(hits) == 1 else ""


__all__ = [
    "ACTIONS",
    "PLAY_FORMS",
    "SAY_FORMS",
    "fields_for",
    "offered_schema",
    "perform_description",
    "verb_action",
]
