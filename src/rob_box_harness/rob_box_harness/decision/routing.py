"""Конфиг маршрутизации decision layer: класс решения → цепочка провайдеров.

Формат YAML/mapping (issue #3084, комментарий Шифу про мульти-провайдер)::

    decision_routing:
      safety_critical:
        providers: [jev, deterministic]
        deadline_ms: 400
        min_confidence: 0.9
      latency_critical:
        providers: [laya, deterministic]
        deadline_ms: 150
      quality_critical:
        providers: [jev, laya, deterministic]
        deadline_ms: 800

``deterministic`` в конце цепочки — только маркер для читаемости:
детерминированный fallback вызывающего кода выполняется ВСЕГДА, если
ни один модельный провайдер не дал валидный уверенный ответ, и его нельзя
убрать конфигом. Стоять не последним ``deterministic`` не может.

Hard invariant: ``laya`` запрещена в ``safety_critical`` — self-hosted
модель без измеренной калибровки на наших данных не участвует в
safety-решениях (комментарий Шифу 5865519249). Нарушение — ``ValueError``
при загрузке, а не молчаливое исправление (ADR-0018, fail-loud).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .types import DecisionClass

DETERMINISTIC = "deterministic"

#: Провайдеры, которым запрещено участвовать в safety-critical решениях.
FORBIDDEN_FOR_SAFETY: frozenset[str] = frozenset({"laya"})

#: Верхняя граница дедлайна любого класса — защита от конфига «ждать 30 с».
MAX_DEADLINE_MS = 2000


@dataclass(frozen=True)
class ClassRoute:
    """Цепочка и пороги для одного класса решений."""

    providers: tuple[str, ...]
    deadline_ms: int
    min_confidence: float


#: Значения по умолчанию. Дедлайны консервативные и должны быть
#: пересмотрены по реальным замерам p95 (см. docs/research/jev/README.md,
#: «Что осталось»): сейчас замеров на нашем deployment path нет.
DEFAULT_ROUTES: Mapping[DecisionClass, ClassRoute] = {
    DecisionClass.SAFETY_CRITICAL: ClassRoute(("jev",), 400, 0.9),
    DecisionClass.LATENCY_CRITICAL: ClassRoute(("laya",), 150, 0.7),
    DecisionClass.QUALITY_CRITICAL: ClassRoute(("jev", "laya"), 800, 0.6),
}


@dataclass(frozen=True)
class RoutingConfig:
    routes: Mapping[DecisionClass, ClassRoute]

    def route(self, decision_class: DecisionClass) -> ClassRoute:
        return self.routes[decision_class]

    @classmethod
    def default(cls) -> "RoutingConfig":
        return cls(dict(DEFAULT_ROUTES))

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "RoutingConfig":
        """Разобрать mapping (из YAML). Незаданные классы берут дефолт."""
        section = raw.get("decision_routing", raw)
        if not isinstance(section, Mapping):
            raise ValueError("decision_routing must be a mapping")
        unknown = set(section) - {c.value for c in DecisionClass}
        if unknown:
            raise ValueError(f"unknown decision classes: {sorted(unknown)}")
        routes = dict(DEFAULT_ROUTES)
        for decision_class in DecisionClass:
            entry = section.get(decision_class.value)
            if entry is not None:
                routes[decision_class] = _parse_route(decision_class, entry)
        return cls(routes)


def _parse_route(decision_class: DecisionClass, entry: Any) -> ClassRoute:
    if not isinstance(entry, Mapping):
        raise ValueError(f"{decision_class.value}: must be a mapping")
    default = DEFAULT_ROUTES[decision_class]
    providers = _parse_providers(decision_class, entry.get("providers", list(default.providers)))
    deadline_ms = int(entry.get("deadline_ms", default.deadline_ms))
    if not 0 < deadline_ms <= MAX_DEADLINE_MS:
        raise ValueError(f"{decision_class.value}: deadline_ms must be in (0, {MAX_DEADLINE_MS}]")
    min_confidence = float(entry.get("min_confidence", default.min_confidence))
    if not 0.0 <= min_confidence <= 1.0:
        raise ValueError(f"{decision_class.value}: min_confidence must be in [0, 1]")
    return ClassRoute(providers, deadline_ms, min_confidence)


def _parse_providers(decision_class: DecisionClass, raw: Any) -> tuple[str, ...]:
    if not isinstance(raw, (list, tuple)) or not all(isinstance(p, str) for p in raw):
        raise ValueError(f"{decision_class.value}: providers must be a list of names")
    names = list(raw)
    if DETERMINISTIC in names:
        if names.index(DETERMINISTIC) != len(names) - 1 or names.count(DETERMINISTIC) > 1:
            raise ValueError(f"{decision_class.value}: '{DETERMINISTIC}' may only be last")
        names.pop()
    if len(set(names)) != len(names):
        raise ValueError(f"{decision_class.value}: duplicate providers")
    if decision_class is DecisionClass.SAFETY_CRITICAL:
        forbidden = FORBIDDEN_FOR_SAFETY.intersection(names)
        if forbidden:
            raise ValueError(f"safety_critical may not use providers {sorted(forbidden)}")
    return tuple(names)
