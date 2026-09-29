"""Типы decision layer (issue #3084): вопросы, ответы, ошибки провайдеров.

Контракт повторяет примитивы System One (Jev): ``Choice`` — выбор из
закрытого набора вариантов, ``Score`` — упорядоченная шкала, ``Noul`` —
вероятность «да». Модуль pure-Python, без ROS и без сети — его
импортируют и провайдеры, и оркестратор, и офлайн-тесты.

Важно (docs/research/jev/awesome-jev-survey.md §«Primitive semantics»):
у ``Noul`` нет отдельного поля confidence — только вероятность «да».
«Не знаю / нужен human review» — решение НАШЕГО кода по порогам, а не
третий выход модели.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Sequence, Union


class DecisionClass(str, Enum):
    """Класс решения — определяет разрешённую цепочку провайдеров и дедлайн.

    * ``SAFETY_CRITICAL`` — acceptance / всё, что трогает физику робота.
      Self-hosted Laya сюда не допускается (см. ``FORBIDDEN_FOR_SAFETY``
      в :mod:`.routing`). Модель может только УЖЕСТОЧИТЬ решение.
    * ``LATENCY_CRITICAL`` — решения на горячем пути диалога.
    * ``QUALITY_CRITICAL`` — решения, где важнее качество, чем 100 мс.
    """

    SAFETY_CRITICAL = "safety_critical"
    LATENCY_CRITICAL = "latency_critical"
    QUALITY_CRITICAL = "quality_critical"


@dataclass(frozen=True)
class ChoiceQuestion:
    """Выбор ровно одного варианта из ``options`` (подготовленных нашим кодом)."""

    text: str
    options: tuple[str, ...]

    def __post_init__(self) -> None:
        _require_options(self.options, "ChoiceQuestion.options", minimum=2)


@dataclass(frozen=True)
class ScoreQuestion:
    """Оценка по упорядоченной шкале ``levels`` (от худшего к лучшему)."""

    text: str
    levels: tuple[str, ...]

    def __post_init__(self) -> None:
        _require_options(self.levels, "ScoreQuestion.levels", minimum=2)


@dataclass(frozen=True)
class NoulQuestion:
    """Вероятность ответа «да» на утверждение ``text``."""

    text: str


Question = Union[ChoiceQuestion, ScoreQuestion, NoulQuestion]
State = Union[str, Mapping[str, Any], Sequence[str]]


@dataclass(frozen=True)
class ChoiceAnswer:
    """Ответ на Choice/Score: выбранный вариант + распределение + confidence."""

    answer: str
    probabilities: Mapping[str, float]
    confidence: float


@dataclass(frozen=True)
class NoulAnswer:
    """Ответ на Noul: вероятность «да» в [0, 1]. Confidence нет — по контракту."""

    probability: float


Answer = Union[ChoiceAnswer, NoulAnswer]


@dataclass(frozen=True)
class Decision:
    """Результат одного провайдера (или детерминированного fallback'а).

    ``provider`` — имя провайдера, реально ответившего (``"jev"``,
    ``"laya"``, ``"deterministic"``). ``fallback_used`` — True, если
    ответ дал детерминированный путь вызывающего кода.
    """

    answers: Mapping[str, Answer]
    provider: str
    model: str | None = None
    latency_ms: float | None = None
    usage: Mapping[str, Any] = field(default_factory=dict)
    fallback_used: bool = False
    fallback_reason: str | None = None


# ---------------------------------------------------------------------------
# Ошибки провайдеров. Иерархия нужна оркестратору, чтобы разнести метрики
# jev_timeout / jev_error / jev_invalid (acceptance criteria issue #3084).
# ---------------------------------------------------------------------------


class DecisionProviderError(Exception):
    """Базовая ошибка провайдера. Никогда не пробрасывается из оркестратора."""

    outcome = "error"


class ProviderTimeout(DecisionProviderError):
    outcome = "timeout"


class ProviderUnavailable(DecisionProviderError):
    """DNS / соединение / 5xx / модель недоступна."""


class ProviderAuthError(DecisionProviderError):
    """401/403 — ключ не задан или отозван. Сам ключ в сообщение НЕ попадает."""


class ProviderRateLimited(DecisionProviderError):
    """429."""


class ProviderConfigError(DecisionProviderError):
    """Провайдер не сконфигурирован (нет ключа/base_url)."""


class ProviderInvalidResponse(DecisionProviderError):
    """Ответ не прошёл схему/валидацию: вариант вне набора, NaN, нет ответа."""

    outcome = "invalid"


# ---------------------------------------------------------------------------
# Валидация ответа против заданных вопросов.
# ---------------------------------------------------------------------------

#: Допуск на сумму вероятностей распределения Choice/Score.
PROBABILITY_SUM_TOLERANCE = 0.02


def validate_answers(
    questions: Mapping[str, Question],
    answers: Mapping[str, Answer],
) -> None:
    """Проверить, что ``answers`` строго соответствуют ``questions``.

    Raises:
        ProviderInvalidResponse: при любом расхождении — лишний/пропущенный
            id, вариант вне закрытого набора, вероятность вне [0, 1] или
            NaN, распределение не суммируется в 1.
    """
    if set(answers) != set(questions):
        raise ProviderInvalidResponse(f"answer ids {sorted(answers)} != question ids {sorted(questions)}")
    for qid, question in questions.items():
        answer = answers[qid]
        if isinstance(question, NoulQuestion):
            if not isinstance(answer, NoulAnswer):
                raise ProviderInvalidResponse(f"{qid}: expected noul answer")
            _require_probability(answer.probability, f"{qid}.probability")
            continue
        allowed = question.options if isinstance(question, ChoiceQuestion) else question.levels
        _validate_distribution(qid, answer, allowed)


def _validate_distribution(qid: str, answer: Answer, allowed: tuple[str, ...]) -> None:
    if not isinstance(answer, ChoiceAnswer):
        raise ProviderInvalidResponse(f"{qid}: expected choice/score answer")
    if answer.answer not in allowed:
        raise ProviderInvalidResponse(f"{qid}: answer {answer.answer!r} not in closed set")
    if set(answer.probabilities) - set(allowed):
        raise ProviderInvalidResponse(f"{qid}: probabilities contain unknown options")
    for option, p in answer.probabilities.items():
        _require_probability(p, f"{qid}.probabilities[{option}]")
    total = sum(answer.probabilities.values())
    if abs(total - 1.0) > PROBABILITY_SUM_TOLERANCE:
        raise ProviderInvalidResponse(f"{qid}: probabilities sum to {total:.3f}")
    _require_probability(answer.confidence, f"{qid}.confidence")


def _require_probability(value: Any, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProviderInvalidResponse(f"{name}: not a number")
    if math.isnan(value) or value < 0.0 or value > 1.0:
        raise ProviderInvalidResponse(f"{name}: {value!r} outside [0, 1]")


def _require_options(options: tuple[str, ...], name: str, *, minimum: int) -> None:
    if not isinstance(options, tuple):
        raise TypeError(f"{name} must be a tuple")
    if len(options) < minimum:
        raise ValueError(f"{name} needs at least {minimum} entries")
    if len(set(options)) != len(options):
        raise ValueError(f"{name} contains duplicates")
