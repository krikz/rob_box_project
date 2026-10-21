"""Что делать с новым действием, когда ресурсы заняты (ADR-0150 §5.6.2, §5.6.4): :func:`compose` по таблице.

Кто решает (ADR-0148): (1) клетка ``knowledge.COMPOSITION`` «новое × занятое» — единственное место знания об
одновременности; (2) Tier-1 по закрытому списку ``knowledge.TIER1_POLICY`` (``words.policy_word``) переопределяет
клетки конфликта; (3) LLM — только ``when``: ``now`` — как Tier-1 «сейчас»; ``after_current`` — дефолт схемы команды,
неотличимый от молчания модели, поэтому решает таблица (явное «потом» человека ловит Tier-1). Текст ответа LLM
здесь не читается.

Правила переопределения:

* ``halt`` («стой») — строка ``halt`` таблицы целиком, какое бы действие ни пришло;
* ``stop``/``now`` (REPLACE на границе) — только клетки конфликта (не голый MERGE) и не непрерываемое занятое
  (``COMPOSITION_UNINTERRUPTIBLE``); граница — из Tier-1 для голоса (реплика, выступление), иначе из таблицы
  (такт, фраза трека);
* ``later`` (APPEND после блока) — любая клетка, кроме ресурса без события конца (``COMPOSITION_NO_END``);
* клетку-вопрос (REJECT с ``ask``, решение О9 — спросить человека) Tier-1 не трогает, кроме ``halt``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from rob_box_dialog import knowledge as kn

_VOICE = ("speech", "perform")


@dataclass(frozen=True)
class Cell:
    """Решение против одного занятого ресурса: политика, граница, вопрос и кто решил (table | tier1 | llm)."""

    busy: str
    policy: str
    boundary: Optional[str] = None
    ask: Optional[str] = None
    source: str = "table"

    @property
    def waits(self) -> bool:
        """Новое начинается после конца занятого блока (APPEND) или его границы (MERGE на сегменте, О7)."""
        return self.policy == "APPEND" or (self.policy == "MERGE" and self.boundary not in (None, "now"))


@dataclass(frozen=True)
class Decision:
    """Итог :func:`compose`: строка матрицы и клетки по каждому занятому ресурсу."""

    new: str
    cells: Tuple[Cell, ...] = ()

    @property
    def policy(self) -> str:
        """Самая строгая клетка (``knowledge.POLICY_ORDER``); ничего не занято — MERGE."""
        return next((p for p in kn.POLICY_ORDER if any(c.policy == p for c in self.cells)), "MERGE")

    @property
    def boundary(self) -> Optional[str]:
        return next((c.boundary for c in self.cells if c.policy == self.policy), None)

    @property
    def ask(self) -> Optional[str]:
        return next((c.ask for c in self.cells if c.ask), None)

    @property
    def waits_for(self) -> Tuple[str, ...]:
        """Занятые ресурсы, конца (или границы) которых новое ждёт."""
        return tuple(c.busy for c in self.cells if c.waits)

    @property
    def replaces(self) -> Tuple[str, ...]:
        """Занятые ресурсы, которые новое заменяет (на границе клетки)."""
        return tuple(c.busy for c in self.cells if c.policy == "REPLACE")


def kind_of(tool: str) -> str:
    """Строка матрицы для тула: особая (``COMPOSITION_KIND_BY_TOOL``) или по классу действия."""
    return kn.COMPOSITION_KIND_BY_TOOL.get(tool) or kn.COMPOSITION_KIND_BY_CLASS[kn.TOOL_CLASSES[tool]]


def compose(new: str, busy: Iterable[str], tier1: Optional[str] = None, when: str = "after_current") -> Decision:
    """Решение для нового действия строки ``new`` против занятых ресурсов ``busy`` (столбцы ``COMPOSITION_BUSY``).

    ``tier1`` — ключ ``TIER1_POLICY`` по словам человека (:func:`rob_box_dialog.words.policy_word`), ``when`` — поле
    команды LLM. Неизвестная строка или столбец — ``KeyError`` (громко: таблица — единственное знание)."""
    if new not in kn.COMPOSITION_NEW:
        raise KeyError(new)
    row = "halt" if tier1 == "halt" else new
    override, source = _override(tier1, when)
    cells = []
    for col in dict.fromkeys(busy):
        rule = kn.COMPOSITION[(row, col)]
        cell = Cell(col, rule.policy, rule.boundary, rule.ask, "tier1" if row != new else "table")
        cells.append(_apply(cell, override, source) if override is not None else cell)
    return Decision(new, tuple(cells))


def _override(tier1: Optional[str], when: str) -> Tuple[Optional[kn.PolicyWords], str]:
    if tier1 in ("stop", "now", "later"):
        return kn.TIER1_POLICY[tier1], "tier1"
    if tier1 is None and when == "now":
        return kn.TIER1_POLICY["now"], "llm"
    return None, "table"


def _apply(cell: Cell, words: kn.PolicyWords, source: str) -> Cell:
    if cell.ask or cell.busy in kn.COMPOSITION_NO_END:
        return cell
    if words.policy == "APPEND":
        return Cell(cell.busy, "APPEND", words.boundary, None, source)
    conflict = cell.policy != "MERGE" or cell.boundary is not None
    if not conflict or cell.busy in kn.COMPOSITION_UNINTERRUPTIBLE:
        return cell
    boundary = words.boundary if cell.busy in _VOICE else cell.boundary
    return Cell(cell.busy, words.policy, boundary, None, source)


__all__ = ["Cell", "Decision", "compose", "kind_of"]
