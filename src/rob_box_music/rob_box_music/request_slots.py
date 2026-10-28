"""Слоты заказа музыки, которые заполняет LLM: схема тула, промпт, валидатор (#3609; ADR-0157 §2.1, ADR-0148).

Тот же механизм, что у поиска темы (``theme_queries`` → ``engine.theme_links``): LLM получает фразу и непонятые коду
слова и отдаёт **узкую схему** — композитор, произведение (оригинальное/международное название), инструмент, стиль,
настроение, режим; всё остальное («давай», «врубай», эмоции) в схему не помещается и отброшено. Инструмент, стиль,
настроение и режим — перечни из ``knowledge`` (одна таблица знания), композитор и произведение — строки, которые
**проверяет код** каталогом (``rob_box_mcp_tools.engine.request_slots``): не подтвердилось — поле пустое.

Здесь — чистая часть: схема, промпт, валидатор и сама структура слотов. Сеть, каталог, кэш — в ``rob_box_mcp_tools``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Sequence, Tuple

from . import knowledge as kn
from .theme_queries import MAX_QUERY_CHARS, QueriesInvalid, clean_query

SUBMIT_TOOL = "submit_request_slots"
#: Режим заказа (ADR-0157 §2.1): пьеса, сет, песня. Режим решает грамматика (``asks_for_set``); слот LLM — для лога.
MODES = ("piece", "set", "song")
#: Поля-строки (проверяет каталог) и поля-перечни (проверяет схема) — в порядке строки лога.
TEXT_FIELDS = ("composer", "work")
ENUM_FIELDS: Mapping[str, Tuple[str, ...]] = {"instrument": tuple(kn.INSTRUMENTS), "style": tuple(kn.STYLES),
                                              "mood": tuple(kn.MOOD_ENERGY), "mode": MODES}
FIELDS = (*TEXT_FIELDS, *ENUM_FIELDS)
#: Откуда значение слота: грамматика и каталог (``code``), LLM с проверкой каталогом (``llm``), кэш разборов,
#: название по звучанию слов (``sound``: STT записал английское название кириллицей, #4016).
SOURCES = ("code", "llm", "cache", "sound")


@dataclass(frozen=True)
class RequestSlots:
    """Разобранный заказ одиночной пьесы. ``composer``/``work`` — написание каталога (LLM-путь) или слова человека
    (код-путь); ``query`` — строка, по которой код строит кандидатов произведения; ``work_confirmed`` — произведение
    подтверждено каталогом (названное играется само, свежесть его не вытесняет); ``source`` — кто разобрал,
    ``ms`` — сколько; ``unknown`` — слова, которых каталог не знает; ``rejected`` — что LLM дала, а каталог не
    подтвердил; ``doubt`` — слова названия, как услышаны, когда код не понял, что названо (непонятых больше понятых, по
    звучанию не нашлось, LLM не подтвердила, #4016): заказ не урезается молча до понятых слов, а переспрашивает."""

    query: str = ""
    composer: str = ""
    work: str = ""
    instrument: str = ""
    style: str = ""
    mood: str = ""
    mode: str = "piece"
    work_confirmed: bool = False
    source: str = "code"
    ms: float = 0.0
    unknown: Tuple[str, ...] = ()
    rejected: Tuple[str, ...] = field(default=())
    doubt: str = ""

    def describe(self) -> str:
        """Слоты для строки лога: ``composer=Wagner work=«Ride Of The Valkyries» instrument=— …``."""
        work = f"«{self.work}»{'✓' if self.work_confirmed else ''}" if self.work else "—"
        rest = " ".join(f"{name}={getattr(self, name) or '—'}" for name in ENUM_FIELDS)
        return f"composer={self.composer or '—'} work={work} {rest}"


def schema() -> Dict[str, Any]:
    props: Dict[str, Any] = {
        "composer": {"type": "string", "maxLength": MAX_QUERY_CHARS,
                     "description": "композитор или исполнитель латиницей, как в международных каталогах "
                                    "(Wagner, Bach, Mozart); пусто — не назван"},
        "work": {"type": "string", "maxLength": MAX_QUERY_CHARS,
                 "description": "оригинальное или международное название произведения без имени автора; русское "
                                "название переведи; пусто — назван только автор или ничего"},
    }
    for name, values in ENUM_FIELDS.items():
        props[name] = {"type": "string", "enum": ["", *values] if name != "mode" else list(values)}
    return {"type": "object", "properties": props, "required": list(FIELDS), "additionalProperties": False}


def tool() -> Dict[str, Any]:
    return {"type": "function", "function": {
        "name": SUBMIT_TOOL, "description": "Отдать слоты заказа музыки. Вызвать ровно один раз.",
        "parameters": schema()}}


def prompt(text: str, unknown: Sequence[str]) -> Tuple[str, str]:
    system = ("Ты разбираешь заказ музыки роботу на слоты. composer — композитор или исполнитель латиницей, как в "
              "международных каталогах (Wagner, Bach, Mozart). work — оригинальное или международное название "
              "произведения, без автора; русское переведи: «Полёт валькирий» → «Ride of the Valkyries», «Танец "
              "маленьких утят» → «Chicken Dance», «К Элизе» → «Fur Elise»; STT ошибается в гласных («волкирий» — "
              "«валькирий»). «что-нибудь из Моцарта» — только composer. instrument, style, mood — ключ из перечня, "
              "если назван, иначе пусто. mode: piece — одна пьеса, set — сет/диджей/вечеринка, song — спеть песню. "
              "Слова просьбы, эмоции и ругань — не слоты, их отбрось. Есть ли произведение в каталоге — проверит "
              f"код. Ответ — один вызов {SUBMIT_TOOL}, без текста.")
    user = f"Заказ: «{text}».\nНепонятные коду слова: " + (", ".join(unknown) or "—")
    return system, user


def validate(payload: Any) -> Dict[str, str]:
    """Слоты по схеме: строки — без лишних пробелов, не длиннее границы строки поиска; перечни — из ``knowledge``
    (пусто — не названо). Нет поля, не строка или значение вне перечня — :class:`QueriesInvalid` (ответ не
    принимается целиком, ADR-0148: невалидное — как без LLM)."""
    if not isinstance(payload, Mapping):
        raise QueriesInvalid("ответ не объект")
    out: Dict[str, str] = {}
    for name in FIELDS:
        value = payload.get(name, "")
        if not isinstance(value, str):
            raise QueriesInvalid(f"{name} не строка")
        if name in ENUM_FIELDS:
            value = value.strip().lower()
            if value and value not in ENUM_FIELDS[name]:
                raise QueriesInvalid(f"{name}={value!r} вне перечня")
        out[name] = clean_query(value) if name in TEXT_FIELDS else value
    out["mode"] = out["mode"] or "piece"
    return out


__all__ = ["ENUM_FIELDS", "FIELDS", "MODES", "RequestSlots", "SOURCES", "SUBMIT_TOOL", "TEXT_FIELDS", "prompt",
           "schema", "tool", "validate"]
