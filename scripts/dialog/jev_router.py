"""Классификатор групп тулов на Jev для варианта C сухого стенда (карточка #4163).

Перед ходом Jev (System One, ``/v1/systemone``) получает реплику и закрытый список групп тулов с описаниями из файла
данных (``understanding_corpus/jev_groups.<язык>.json``) и отвечает одним выбором: ``choice`` + ``confidence``. Клиент -
``rob_box_harness.decision.systemone_http`` (тот же, что у выбора пьесы ``music_chooser: chain``, ADR-0157 PR-7, #3564),
здесь без оркестратора: стенд меряет сам ответ, а не цепочку и её circuit breaker.

Порог уверенности. В настройках ``music_chooser`` порога нет: ``ModelChooser`` строит маршрут с ``min_confidence`` 0.0
(``mcp_server/engine/chooser_model.py``, ``ClassRoute((name,), deadline_ms, 0.0)``). Общий порог decision layer для
решений на качество - ``DEFAULT_ROUTES[QUALITY_CRITICAL].min_confidence`` (0.6, ``decision/routing.py``): его и берём.

Описания групп - данные: сверяются с ``knowledge.TOOL_GROUPS`` при загрузке (лишняя или пропущенная группа - громкая
ошибка), слов из ``knowledge.py`` в файле нет. Ключ API в отчёт и логи не попадает (клиент читает его из env).
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

from rob_box_dialog import knowledge as kn
from rob_box_harness.decision import ChoiceQuestion, DecisionClass
from rob_box_harness.decision.routing import DEFAULT_ROUTES

import understanding_variants as uv

CORPUS_DIR = Path(__file__).resolve().parent / "understanding_corpus"
DEFAULT_SPEC = CORPUS_DIR / "jev_groups.ru.json"
QUESTION_ID = "route"
#: Порог по умолчанию - общий для решений на качество (0.6); в настройках ``music_chooser`` своего порога нет.
DEFAULT_THRESHOLD = DEFAULT_ROUTES[DecisionClass.QUALITY_CRITICAL].min_confidence
#: Верхняя страховка одного запроса, с: дедлайна пути хода у стенда нет, а время ответа Jev меряется и попадает в отчёт.
DEFAULT_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class GroupSpec:
    """Вопрос классификатору: язык, текст вопроса, описания групп и выбора «группа не нужна»."""

    language: str
    instructions: str
    groups: Mapping[str, str]
    none: str

    def question(self) -> ChoiceQuestion:
        options = (*self.groups, uv.NO_GROUP)
        return ChoiceQuestion(self.instructions, options, {**self.groups, uv.NO_GROUP: self.none})


def load_spec(path: Path = DEFAULT_SPEC) -> GroupSpec:
    """Файл описаний групп; набор групп обязан совпадать с ``knowledge.TOOL_GROUPS``."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    groups = dict(data["groups"])
    missing, extra = sorted(set(kn.TOOL_GROUPS) - set(groups)), sorted(set(groups) - set(kn.TOOL_GROUPS))
    if missing or extra:
        raise ValueError(f"{path}: группы расходятся с knowledge.TOOL_GROUPS: нет {missing}, лишние {extra}")
    return GroupSpec(str(data["language"]), str(data["instructions"]), groups, str(data["none"]))


@dataclass
class Route:
    """Выбор классификатора на одну реплику: группа (или ``none``), уверенность, принят ли по порогу, цена."""

    group: str = ""
    confidence: float = 0.0
    accepted: bool = False
    latency_ms: float = 0.0
    input_tokens: int = 0
    output_tokens: int = 0
    error: str = ""

    def as_pair(self) -> Optional[tuple]:
        """``(группа, принят)`` для среза C; сбой Jev - ``None`` (срез B)."""
        return None if self.error or not self.group else (self.group, self.accepted)


class JevRouter:
    """Классификатор Jev: один запрос на реплику, счётчики вызовов, сбоев и токенов для строки бюджета."""

    def __init__(self, provider: Any, spec: GroupSpec, threshold: float = DEFAULT_THRESHOLD,
                 max_calls: Optional[int] = None) -> None:
        self._provider = provider
        self._spec = spec
        self._question = spec.question()
        self.threshold = threshold
        self.max_calls = max_calls
        self.calls = 0
        self.errors = 0
        self.input_tokens = 0
        self.output_tokens = 0

    async def choose(self, text: str, music_playing: bool = False) -> Route:
        """Группа тулов для реплики. Сбой (таймаут, HTTP, разбор) - ``Route.error``: ход пойдёт со срезом B. Бюджет
        вызовов (``max_calls``) исчерпан - запроса нет, ``Route.error == "jev_budget"``."""
        if self.max_calls is not None and self.calls >= self.max_calls:
            return Route(error="jev_budget")
        state: Dict[str, Any] = {"реплика": text, "сейчас играет музыка": bool(music_playing)}
        self.calls += 1
        started = time.monotonic()
        try:
            decision = await self._provider.decide(state, {QUESTION_ID: self._question})
        except Exception as exc:  # noqa: BLE001 — любой сбой классификатора = срез B, а не падение хода
            self.errors += 1
            return Route(error=type(exc).__name__, latency_ms=(time.monotonic() - started) * 1000.0)
        answer = decision.answers[QUESTION_ID]
        usage = decision.usage or {}
        route = Route(answer.answer, float(answer.confidence), float(answer.confidence) >= self.threshold,
                      float(decision.latency_ms or (time.monotonic() - started) * 1000.0),
                      int(usage.get("input_tokens") or 0), int(usage.get("output_tokens") or 0))
        self.input_tokens += route.input_tokens
        self.output_tokens += route.output_tokens
        return route

    def stats(self) -> Dict[str, Any]:
        return {"calls": self.calls, "errors": self.errors, "input_tokens": self.input_tokens,
                "output_tokens": self.output_tokens, "threshold": self.threshold, "language": self._spec.language,
                "max_calls": self.max_calls}


__all__ = ["DEFAULT_SPEC", "DEFAULT_THRESHOLD", "DEFAULT_TIMEOUT_S", "GroupSpec", "JevRouter", "Route", "load_spec"]
