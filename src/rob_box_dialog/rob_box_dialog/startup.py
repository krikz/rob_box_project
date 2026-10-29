"""Стартовая фраза робота по реальной готовности узлов (#3913, ADR-0148: фразу выбирает код из фактов).

Было (v1, #1003): случайная из девяти «все системы в норме» через 12 с после старта, что бы ни поднялось. Стало:
готовность - факт из двух существующих источников, а фраза - ключ каталога ``phrases/ru.yaml`` по вердикту.

Источники (новых швов нет):

* узлы - отчёт монитора контекстного агрегатора ``/perception/context_update`` (``active_nodes``/``failed_nodes``/
  ``missing_nodes``; список ожидаемых узлов ведёт монитор, а не этот модуль);
* LLM - здоровье цепочки провайдеров (``ProviderState`` вида ``llm``, ADR-0150 §8).

Вердикт (:func:`assess`): ``ready`` - голос, остальные узлы и LLM на месте («готов к работе»); ``voice_only`` - голос
есть, остальные узлы отчёта не поднялись; ``partial`` - поднялось не всё; ``unknown`` - отчёта нет или он устарел
(честное «не знаю», не «всё в норме»). Явный текст оператора (``startup_greeting_text``) важнее. Чистый Python.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases

READY, VOICE_ONLY, PARTIAL, UNKNOWN = "ready", "voice_only", "partial", "unknown"
#: Ключ части для цепочки LLM (``startup.part.llm``).
LLM_PART = "llm"


@dataclass(frozen=True)
class NodeReport:
    """Отчёт монитора узлов: имена как в графе ROS (``/stt_node``)."""

    active: Tuple[str, ...] = ()
    failed: Tuple[str, ...] = ()
    missing: Tuple[str, ...] = ()

    @classmethod
    def from_lists(cls, active: Iterable[str], failed: Iterable[str], missing: Iterable[str]) -> "NodeReport":
        return cls(tuple(active), tuple(failed), tuple(missing))


@dataclass(frozen=True)
class Readiness:
    """Что известно о готовности: ``nodes`` - свежий отчёт или ``None``; ``llm_ok`` - есть ли пригодный провайдер LLM
    (``None`` - цепочка здоровья не публикует, не гадаем)."""

    nodes: Optional[NodeReport] = None
    llm_ok: Optional[bool] = None


@dataclass(frozen=True)
class Verdict:
    kind: str
    #: Ключи частей (``startup.part.*``), которые не поднялись, без повторов, в порядке отчёта.
    down: Tuple[str, ...] = ()


def llm_usable(states: Iterable[object]) -> Optional[bool]:
    """Есть ли пригодный провайдер среди состояний вида ``llm`` (``ProviderState``); состояний нет - ``None``."""
    health = [getattr(s, "health", None) for s in states if getattr(s, "kind", None) == "llm"]
    if not health:
        return None
    return any(h in kn.PROVIDER_USABLE_HEALTH for h in health)


def _part(node: str) -> str:
    return kn.STARTUP_NODE_PARTS.get(node, "other")


def _unique(items: Iterable[str]) -> Tuple[str, ...]:
    return tuple(dict.fromkeys(items))


def assess(readiness: Readiness) -> Verdict:
    """Вердикт готовности. Узел диалога в отчёте пропускается (фразу говорит он сам); «не поднялся» - любой узел из
    ``failed``/``missing`` отчёта."""
    report = readiness.nodes
    if report is None:
        return Verdict(UNKNOWN)
    own = set(kn.STARTUP_DIALOG_NODES)
    active = [n for n in report.active if n not in own]
    not_up = [n for n in (*report.failed, *report.missing) if n not in own]
    voice = set(kn.STARTUP_VOICE_NODES)
    down = [_part(n) for n in not_up]
    if readiness.llm_ok is False:
        down.append(LLM_PART)
    if not down:
        return Verdict(READY)
    voice_up = not any(n in voice for n in not_up)
    rest_up = any(n not in voice for n in active)
    return Verdict(VOICE_ONLY if voice_up and not rest_up else PARTIAL, _unique(down))


def enumerate_parts(parts: Sequence[str]) -> str:
    """«камера», «камера и лидар», «камера, лидар и карта и навигация» - слова частей из каталога."""
    words: List[str] = [phrases.text(f"startup.part.{p}") for p in parts]
    if len(words) <= 1:
        return "".join(words)
    return ", ".join(words[:-1]) + f" {phrases.text('startup.and')} " + words[-1]


def pick_startup_phrase(readiness: Readiness, override: Optional[str] = None,
                        rng: Optional[random.Random] = None) -> Tuple[str, Verdict]:
    """Стартовая фраза и вердикт, по которому она выбрана. Непустой ``override`` (без пробелов по краям) важнее
    вердикта: оператор сказал точный текст."""
    verdict = assess(readiness)
    if override and override.strip():
        return override.strip(), verdict
    text, _ = phrases.pick(f"startup.{verdict.kind}", rng=rng, down=enumerate_parts(verdict.down))
    return text, verdict


class ReadinessWatch:
    """Последний отчёт узлов с временем получения: старше ``stale_s`` - не факт (агрегатор мог упасть)."""

    def __init__(self, stale_s: float = kn.STARTUP_REPORT_STALE_S) -> None:
        self._stale_s = stale_s
        self._report: Optional[NodeReport] = None
        self._at = 0.0

    def observe(self, report: NodeReport, now: float) -> None:
        self._report, self._at = report, now

    def snapshot(self, now: float, llm_ok: Optional[bool] = None) -> Readiness:
        fresh = self._report is not None and now - self._at <= self._stale_s
        return Readiness(self._report if fresh else None, llm_ok)


__all__ = ["LLM_PART", "NodeReport", "PARTIAL", "READY", "Readiness", "ReadinessWatch", "UNKNOWN", "VOICE_ONLY",
           "Verdict", "assess", "enumerate_parts", "llm_usable", "pick_startup_phrase"]
