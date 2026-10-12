"""Выбор среди кандидатов, построенных кодом (ADR-0157 §2.2, §3.3; PR-3).

Контракт — как у вопросов спайка #3084 (``rob_box_harness.decision``: ``ChoiceQuestion`` = состояние + закрытый
перечень вариантов → ответ, ``confidence``, провайдер): :class:`ChoiceQuestion` несёт ``state`` (слоты заказа, что
играли недавно) и варианты с признаками, :class:`Choice` — ``id`` варианта, уверенность и имя провайдера. Пакет
чистый (``test_purity``: без сети и без ``httpx``, который тянет ``rob_box_harness.decision``), поэтому здесь —
тонкий локальный протокол с теми же полями; модельные провайдеры (``llm_harness``/``jev``, PR-7) — адаптер над
оркестратором спайка, который реализует :class:`Chooser` и подставляется в :class:`DeterministicChooser` без правки
вызывающего кода.

:class:`DeterministicChooser` — провайдер по умолчанию и всегда запасной: ``score = Σ CHOOSER_WEIGHTS·признак``
(``knowledge.CHOOSER_WEIGHTS``: 3·название + 2·композитор + 1·годность − 2·недавнее + 0.5·рейтинг); разрыв первого и
второго не меньше ``CHOOSER_CLEAR_GAP`` — модель не зовётся. Модель отвечает только ``id`` из перечня: ответ вне
перечня, исключение или пустота — детерминированный ответ (модель не может расширить список кандидатов).

:func:`dice` — последняя ступень цепочки Jev → LLM → код (решение Шифу 13.10, #3564): не argmax, а взвешенный бросок
по очкам кода, вероятность ∝ ``exp(score / CHOOSER_DICE_TEMPERATURE)`` (бросок — ``diversity.roulette``, ADR-0146).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Protocol, Sequence, Tuple

from .diversity import roulette
from .knowledge import (CHOOSER_CLEAR_GAP, CHOOSER_DEADLINE_MS, CHOOSER_DICE_TEMPERATURE, CHOOSER_WEIGHTS,
                        HOOK_FRESH_SETS, PLAY_COUNT_WINDOW_SEC)

DETERMINISTIC = "deterministic"
#: Провайдер ответа «кубика» кода (последняя ступень цепочки выбора, #3564).
DICE = "code"


@dataclass(frozen=True)
class Plays:
    """Сколько раз кандидата уже играли (за ``PLAY_COUNT_WINDOW_SEC``, версии одной мелодии — одна) и сколько минут
    назад в последний раз; ``None`` — не играли."""

    count: int = 0
    last_ago_min: Optional[int] = None


@dataclass(frozen=True)
class Option:
    """Вариант выбора: ``id`` кандидата, признаки (ключи ``CHOOSER_WEIGHTS``), подпись для модели/лога и сыгранное
    (``None`` — история не подключена: модели про повторы ничего не говорим)."""

    id: str
    features: Mapping[str, float]
    label: str = ""
    plays: Optional[Plays] = None


@dataclass(frozen=True)
class ChoiceQuestion:
    """Вопрос: ``text`` (что выбираем), ``state`` (слоты заказа, недавнее), закрытый перечень ``options``."""

    text: str
    options: Tuple[Option, ...]
    state: Mapping[str, Any] = field(default_factory=dict)

    def ids(self) -> Tuple[str, ...]:
        return tuple(o.id for o in self.options)


@dataclass(frozen=True)
class Choice:
    """Ответ: ``id`` варианта из перечня, уверенность 0..1, кто ответил."""

    id: str
    confidence: float
    provider: str


class Chooser(Protocol):
    def choose(self, question: ChoiceQuestion, *, deadline_ms: int = CHOOSER_DEADLINE_MS) -> Choice:
        ...


def recency(candidate: str, recent: Sequence[str], same: Optional[Mapping[str, Any]] = None,
            depth: int = HOOK_FRESH_SETS) -> float:
    """Признак ``recent``: 1.0 — сыграно последним, дальше по убыванию до ``depth``-го, не сыграно — 0. ``same`` —
    ключ мелодии (версии одной мелодии — одна мелодия: «mundiant_2» после «mundiant»)."""
    keys = same or {}
    key = keys.get(candidate)
    for age, played in enumerate(list(recent)[:depth]):
        if played == candidate or (key is not None and keys.get(played) == key):
            return (depth - age) / depth
    return 0.0


def play_stats(candidate: str, played: Sequence[Tuple[str, float]], now: float,
               same: Optional[Mapping[str, Any]] = None, window: float = PLAY_COUNT_WINDOW_SEC) -> Plays:
    """Счётчик сыгранного по истории ``(запись, ts)`` (из того же источника, что ``recent``) за ``window`` секунд."""
    keys = same or {}
    key = keys.get(candidate)
    stamps = [ts for name, ts in played
              if now - ts <= window and (name == candidate or (key is not None and keys.get(name) == key))]
    return Plays(len(stamps), max(0, int((now - max(stamps)) // 60)) if stamps else None)


def penalty(features: Mapping[str, float]) -> float:
    """Штрафные очки за повтор, которые ``score`` вычитает: ``-вес(recent)·recent`` (одна формула с оценкой)."""
    return -CHOOSER_WEIGHTS["recent"] * float(features.get("recent", 0.0))


def points(features: Mapping[str, float]) -> Dict[str, float]:
    """Вклад каждого признака в очках (``вес·признак``, ключи ``CHOOSER_WEIGHTS``); штраф за повтор - отрицательный."""
    return {name: w * float(features.get(name, 0.0)) for name, w in CHOOSER_WEIGHTS.items()}


def score(features: Mapping[str, float]) -> float:
    """Взвешенная сумма признаков (§3.3) - сумма :func:`points`."""
    return sum(points(features).values())


def dice_weights(question: "ChoiceQuestion", temperature: float = CHOOSER_DICE_TEMPERATURE) -> List[float]:
    """Веса «кубика» по очкам кода: ``exp((score - max) / T)`` (лучший — 1.0; сдвиг на максимум — без переполнения)."""
    scores = [score(o.features) for o in question.options]
    top = max(scores)
    return [math.exp((s - top) / temperature) for s in scores]


def dice(question: "ChoiceQuestion", rng: Any, temperature: float = CHOOSER_DICE_TEMPERATURE) -> "Choice":
    """Выбор кодом «с коэффициентом и кубиком» (решение Шифу 13.10, #3564): вероятность ∝ ``exp(score / T)``,
    бросок — общий ``diversity.roulette`` (ADR-0146); уверенность — вероятность выпавшего. Детерминирован при
    заданном ``rng`` (один ``rng.random()``)."""
    if not question.options:
        raise ValueError("ChoiceQuestion без вариантов")
    weights = dice_weights(question, temperature)
    picked = roulette(range(len(weights)), weights, rng)
    return Choice(question.options[picked].id, weights[picked] / sum(weights), DICE)


class DeterministicChooser:
    """Взвешенная сумма признаков; неоднозначный ответ (разрыв < ``CHOOSER_CLEAR_GAP``) отдаётся ``model``, если он
    подключён (PR-7), иначе — детерминированно."""

    name = DETERMINISTIC

    def __init__(self, model: Optional[Chooser] = None) -> None:
        self._model = model

    def rank(self, question: ChoiceQuestion) -> List[Tuple[float, Option]]:
        """Варианты по убыванию оценки; равные — в порядке перечня (кодом построенный ранг каталога)."""
        return sorted(((round(score(o.features), 6), o) for o in question.options), key=lambda so: -so[0])

    def choose(self, question: ChoiceQuestion, *, deadline_ms: int = CHOOSER_DEADLINE_MS) -> Choice:
        ranked = self.rank(question)
        if not ranked:
            raise ValueError("ChoiceQuestion без вариантов")
        gap = ranked[0][0] - ranked[1][0] if len(ranked) > 1 else CHOOSER_CLEAR_GAP
        own = Choice(ranked[0][1].id, min(1.0, 0.5 + gap / (2 * CHOOSER_CLEAR_GAP)), DETERMINISTIC)
        if gap >= CHOOSER_CLEAR_GAP or self._model is None:
            return own
        try:
            answer = self._model.choose(question, deadline_ms=deadline_ms)
        except Exception:  # noqa: BLE001 — модель не держит выбор: детерминированный ответ всегда есть
            return own
        return answer if answer is not None and answer.id in question.ids() else own

    def order(self, question: ChoiceQuestion) -> Tuple[Choice, List[str]]:
        """Выбор и очередь перебора: выбранный первым, остальные — по оценке (fail-open: не сложился — следующий)."""
        choice = self.choose(question)
        return choice, [choice.id] + [o.id for _s, o in self.rank(question) if o.id != choice.id]


__all__ = ["Choice", "ChoiceQuestion", "Chooser", "DETERMINISTIC", "DICE", "DeterministicChooser", "Option", "Plays",
           "dice", "dice_weights", "penalty", "play_stats", "points", "recency", "score"]
