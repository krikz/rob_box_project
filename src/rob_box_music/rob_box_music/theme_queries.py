"""Поисковые запросы к архиву для части темы, которую прямой поиск не нашёл (#3493; ADR-0148, ADR-0155).

Общий механизм вместо ручных строк «слово → произведение»: LLM получает части темы без находок и предлагает
**поисковые строки** к англоязычному архиву (названия произведений, франшизы, композиторы, персонажи; для понятия —
конкретные произведения, которые к нему относятся) и вид части (``work`` — назвали произведение, ``concept`` —
понятие, ``not_theme`` — не про музыку: стиль, повод, оценка). Хуков LLM не выбирает и фраз не пишет: каждую строку
проверяет код поиском по каталогу (``rob_box_mcp_tools.engine.theme_links``), в сет идут только найденные записи;
принятое соответствие кешируется в реестре (``works.theme_links``).

Здесь — чистая часть: схема тула, промпт, валидатор. Сеть и библиотека — в ``rob_box_mcp_tools``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Sequence, Tuple

SUBMIT_TOOL = "submit_catalog_queries"
#: Строк поиска на одну часть темы: больше — ответ длиннее (латентность старта сета), а архив всё равно проверяет.
MAX_QUERIES = 6
MAX_QUERY_CHARS = 60
KINDS = ("work", "concept", "not_theme")


class QueriesInvalid(ValueError):
    """Ответ LLM не по схеме — предложения не принимаются целиком (ADR-0148: невалидное — как без LLM)."""


@dataclass(frozen=True)
class Suggestion:
    part: str
    kind: str
    queries: Tuple[str, ...]


def schema(parts: Sequence[str]) -> Dict[str, Any]:
    item = {
        "type": "object",
        "properties": {
            "part": {"type": "string", "enum": list(parts), "description": "часть темы — дословно из списка"},
            "kind": {"type": "string", "enum": list(KINDS),
                     "description": "work — назвали конкретное произведение/франшизу; concept — понятие (животные, "
                                    "космос); not_theme — не про музыку (стиль, повод, оценка, обрамление)"},
            "queries": {"type": "array", "maxItems": MAX_QUERIES,
                        "items": {"type": "string", "maxLength": MAX_QUERY_CHARS},
                        "description": "строки поиска по англоязычному архиву: оригинальные названия произведений, "
                                       "франшизы, композиторы; для понятия — названия конкретных произведений"},
        },
        "required": ["part", "kind", "queries"],
        "additionalProperties": False,
    }
    return {"type": "object", "properties": {"suggestions": {"type": "array", "items": item, "minItems": 1,
                                                             "maxItems": len(parts)}},
            "required": ["suggestions"], "additionalProperties": False}


def tool(parts: Sequence[str]) -> Dict[str, Any]:
    return {"type": "function", "function": {
        "name": SUBMIT_TOOL, "description": "Отдать строки поиска по архиву для частей темы. Вызвать ровно один раз.",
        "parameters": schema(parts)}}


def prompt(theme: str, parts: Sequence[str]) -> Tuple[str, str]:
    system = ("Ты библиотекарь мелодий робота-диджея. Архив — около 10 000 рингтонов с английскими названиями "
              "(фильмы, мультфильмы, игры, классика, поп). Для каждой части темы дай строки поиска, которыми её "
              "можно найти в таком архиве: оригинальное или международное название произведения (русское название "
              "переведи: «Танец маленьких утят» → «Chicken Dance», «Birdie Song»), названия его песен и частей. "
              "Имя автора или исполнителя — только для части-автора («Чайковский»): у названного произведения "
              "оно даст чужие вещи (Elton John → «Candle in the Wind» вместо «Короля Льва»). Для понятия — "
              "названия конкретных известных произведений про него. Только названия, без "
              f"слов вроде music, song, theme, remix. Есть ли запись в архиве — проверит код. Ответ — один вызов "
              f"{SUBMIT_TOOL}, без текста.")
    user = f"Тема сета: «{theme}».\nЧасти без находок:\n" + "\n".join(f"- {p}" for p in parts)
    return system, user


def _query(value: Any) -> str:
    text = " ".join(str(value or "").split())
    return text if 0 < len(text) <= MAX_QUERY_CHARS else ""


def validate(payload: Any, parts: Sequence[str]) -> Tuple[Suggestion, ...]:
    """Предложения по схеме: часть — из списка, вид — из :data:`KINDS`, строки — непустые, без повторов, не
    длиннее :data:`MAX_QUERY_CHARS`, не больше :data:`MAX_QUERIES`. Иначе :class:`QueriesInvalid`."""
    if not isinstance(payload, Mapping) or not isinstance(payload.get("suggestions"), list):
        raise QueriesInvalid("нет списка suggestions")
    out: Dict[str, Suggestion] = {}
    for item in payload["suggestions"]:
        if not isinstance(item, Mapping):
            raise QueriesInvalid("элемент suggestions не объект")
        part, kind, queries = item.get("part"), item.get("kind"), item.get("queries")
        if part not in parts:
            raise QueriesInvalid(f"часть {part!r} не из списка")
        if kind not in KINDS:
            raise QueriesInvalid(f"kind {kind!r} не из {KINDS}")
        if not isinstance(queries, list):
            raise QueriesInvalid("queries не список")
        clean = tuple(dict.fromkeys(q for q in map(_query, queries) if q))[:MAX_QUERIES]
        out[part] = Suggestion(part, kind, clean)
    return tuple(out.values())


__all__ = ["KINDS", "MAX_QUERIES", "MAX_QUERY_CHARS", "QueriesInvalid", "SUBMIT_TOOL", "Suggestion", "prompt",
           "schema", "tool", "validate"]
