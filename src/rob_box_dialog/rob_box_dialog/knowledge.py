"""Одна таблица знания диалога v2 (ADR-0150 §2.2, §2.1 п.10; ADR-0148).

Вейк-слова, слова тишины и её снятия, Tier-1 слова политики планировщика, слова и каналы громкости. Остальные
модули — и старый путь v1 — только импортируют отсюда; копий нет. Таблицы команды LLM, классов действий,
экспрессии, рефлексов, сцен и одновременности добавляются следующими частями PR-1 (ADR-0150 §11).

Сверка текста со словами (границы слов) — :mod:`rob_box_dialog.words`.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from types import MappingProxyType
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------------------------------------------
# Адресация (§4.2, решение В2: только по вейк-слову)
# ---------------------------------------------------------------------------------------------------------------

#: Вейк-слова личности. Порядок значим: ``strip_wake_word`` берёт первое совпадение альтернации, длинные и
#: специфичные варианты стоят раньше коротких («роб» не должен съесть «роб бокс»). Искажения STT добавляются по
#: логам e2e, не придумываются. В проде список читается из ``docker/sound/config/wake_words.yaml``
#: (:func:`load_wake_word_namespaces`), здесь — тот же список байт-в-байт для dev и тестов (держит
#: ``test/test_knowledge.py::test_wake_words_yaml_matches_code``).
DEFAULT_WAKE_WORDS = (
    "роб бокс",
    "роббокс",
    "робокос",
    "rob box",
    "робокс",
    # искажения «робокс» у vosk/yandex (e2e ww01): замены гласной
    "робэкс",
    "робекс",
    "робакс",
    "рабокс",
    "рубокс",
    "роблокс",
    "роберт",
    "роббос",
    "robbox",
    # «Робота, про меня что помнишь?» — vosk слышит винительный падеж; стоит раньше «робот»
    "робота",
    "робот",
    # «Робби, добрый вечер» — yandex перед звонкой согласной
    "робби",
    "робок",
    "рабок",
    "робак",
    "рыбок",
    "робик",
    "робо",
    "рома",
    "бот",
    "роб",
)
#: Вейк агента оператора: слышит только микрофон шлема (namespace привязан к источнику аудио).
DEFAULT_OPERATOR_WAKE_WORDS: Tuple[str, ...] = ("тарс", "tars")
WAKE_NAMESPACES: Tuple[str, ...] = ("personality", "operator")

# ---------------------------------------------------------------------------------------------------------------
# Tier-1 словарь (§4.2, §5.6.4, §6.3): ОСНОВЫ слов, матч по левой границе слова (``words.has_word``)
# ---------------------------------------------------------------------------------------------------------------

#: Тишина: «помолчи», «замолчите», «хватит». Матч по границе слова: «захватит» — не команда.
SILENCE_WORDS: Tuple[str, ...] = ("помолч", "замолч", "хватит")
#: Снятие тишины (в состоянии SILENCED): «говори», «включайся», «работай», «отвечай», «разговаривай».
UNSILENCE_WORDS: Tuple[str, ...] = ("говори", "включ", "работ", "отвеч", "разговар")


#: Политики BML при новой команде во время исполнения (§5.6.4) и границы, на которых они срабатывают.
POLICIES: Tuple[str, ...] = ("APPEND", "MERGE", "REPLACE", "REJECT")
BOUNDARIES: Tuple[str, ...] = ("now", "sentence", "segment", "bar", "phrase", "block")


@dataclass(frozen=True)
class PolicyWords:
    """Tier-1 переопределение политики планировщика (§5.6.4): закрытый список слов → политика и граница."""

    words: Tuple[str, ...]
    policy: str
    boundary: str


#: «стоп/хватит/замолчи» — заменить на границе предложения, «стой» — сразу, «потом» — после блока, «сейчас» —
#: на границе. Правила планировщика берут политику отсюда, а не из текста ответа LLM.
TIER1_POLICY: Mapping[str, PolicyWords] = MappingProxyType({
    "stop": PolicyWords(("хватит", "стоп", "замолчи", "отставить"), "REPLACE", "sentence"),
    "halt": PolicyWords(("стой",), "REPLACE", "now"),
    "later": PolicyWords(("потом", "после", "когда закончишь"), "APPEND", "block"),
    "now": PolicyWords(("сейчас", "прямо сейчас"), "REPLACE", "sentence"),
})

#: Громкость: (намерение, фразы) в порядке проверки — «максимальная громкость» раньше «громко». Фразы ищутся
#: подстрокой (как в v1, ``dialogue_helpers.detect_volume_intent``); канал выбирает код по снимку (§4.2).
VOLUME_WORDS: Tuple[Tuple[str, Tuple[str, ...]], ...] = (
    ("max", ("максимальная громкость", "максимум громк")),
    ("normal", ("нормальная громкость", "стандартная громкость", "обычная громкость")),
    ("max", ("говори громко",)),
    ("louder", ("громче", "громко")),
    ("quieter", ("тише", "потише")),
)
VOLUME_INTENTS: Tuple[str, ...] = ("max", "normal", "louder", "quieter")


@dataclass(frozen=True)
class VolumeChannel:
    """Канал громкости (§2.3): у каждого свой владелец, общей «громкости робота» нет."""

    owner: str
    tool: Optional[str]


#: Три канала у трёх владельцев. «Какой канал» решает код: играет музыка → music, звучит речь → tts.
VOLUME_CHANNELS: Mapping[str, VolumeChannel] = MappingProxyType({
    "tts": VolumeChannel(owner="tts_node", tool="set_volume"),
    "music": VolumeChannel(owner="PlayerOwner", tool="set_music_volume"),
    "sfx": VolumeChannel(owner="sound_node", tool=None),
})

# ---------------------------------------------------------------------------------------------------------------
# Загрузка вейк-слов из YAML (прод: /config/wake_words.yaml)
# ---------------------------------------------------------------------------------------------------------------


def load_wake_word_namespaces(path: "str | os.PathLike[str] | None") -> Dict[str, List[str]]:
    """Прочитать ``wake_words.yaml`` в ``{namespace: [words]}``, порядок — как в файле.

    Пустой путь, нет файла, битый YAML, чужая структура — ``{}`` (не падает): вызывающий берёт дефолты кода.
    """
    if not path:
        return {}
    import yaml  # noqa: PLC0415 — лениво: без пути модуль не тянет YAML

    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(data, dict):
        return {}
    out: Dict[str, List[str]] = {}
    for namespace in WAKE_NAMESPACES:
        words = data.get(namespace)
        if isinstance(words, list):
            cleaned = [w for w in words if isinstance(w, str) and w.strip()]
            if cleaned:
                out[namespace] = cleaned
    return out


def resolve_wake_word_namespaces(
    path: "str | os.PathLike[str] | None",
    personality_fallback: Sequence[str] = DEFAULT_WAKE_WORDS,
    operator_fallback: Sequence[str] = DEFAULT_OPERATOR_WAKE_WORDS,
) -> Tuple[List[str], List[str]]:
    """``(personality, operator)``: из файла, а где namespace нет — из переданных фолбеков (порядок важен)."""
    loaded = load_wake_word_namespaces(path)
    personality = loaded.get("personality") or list(personality_fallback)
    operator = loaded.get("operator") or list(operator_fallback)
    return personality, operator
