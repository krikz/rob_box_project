"""track_reasoner.py — фоновый ризонер трека за интерфейсом (ADR-0142 §7-§8, issue #3136).

Вход — :class:`TrackInput` (слова юзера дословно + снимок состояния +
якоря), выход — :class:`~core.track_spec.TrackSpec` или ``None``. Ризонер
возвращает ДАННЫЕ и ничего не исполняет (урок сабагента MusicSkill,
ADR-0142 §9): звук ставит только владелец плеера и только на границе.

Инварианты (ADR-0142 §8.2, по образцу ``DecisionOrchestrator``):

* дедлайн ОДИН на всю цепочку провайдеров;
* ретраев нет: следующий провайдер — это и есть ретрай;
* исключения провайдера наружу не выходят;
* fallback (превью / seeded-спека) остаётся у вызывающего всегда —
  ``spec=None`` значит «играет то, что уже играет».

Ответ провайдера разбирается двумя путями (ADR-0142 §8.5, ADR-0143:
MiniMax игнорирует ``tool_choice="required"``, у MiMo только ``auto``):
аргументы вызова ``submit_track_spec`` или ровно один JSON-блок в тексте.

ЧТО ЗДЕСЬ НЕТ (честно, следующий PR): реального облачного провайдера с
включённым thinking. Выбор модели и кошелька — открытый вопрос ADR-0142
§12 В1 (решает товарищ Шифу), а замеров латентности thinking в репо нет
(§8.4). Здесь только протокол :class:`ReasonerProvider`; в тестах —
провайдер-фейк. Circuit breaker (§8.2) — тоже следующий шаг: переиспользовать
``rob_box_harness.decision.health`` или нет, надо сверить по зависимостям.
Модуль пока никем в проде не вызывается (подключение в shadow — PR-6).
"""

from __future__ import annotations

import asyncio
import json
import logging
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Mapping, Optional, Protocol, Sequence, Tuple

from .track_spec import SpecAnchors, SpecError, TrackSpec, track_spec_schema, validate_spec

logger = logging.getLogger(__name__)

#: Имя «функции» структурного выхода (ничего не исполняет, ADR-0142 §8.5).
SUBMIT_TOOL_NAME = "submit_track_spec"

OUTCOME_OK = "ok"
OUTCOME_LATE = "late"
OUTCOME_INVALID = "invalid"
OUTCOME_ERROR = "error"
OUTCOME_NO_PROVIDER = "no_provider"

_FENCED_JSON = re.compile(r"```(?:json)?\s*(\{.*?\})\s*```", re.DOTALL)


@dataclass(frozen=True)
class TrackInput:
    """Неизменяемый снимок для ризонера (ADR-0142 §4). Без ссылок на живые объекты.

    ``request`` — слова юзера ДОСЛОВНО (не перефраз голосовой LLM); для
    DJ — строка плана. ``played`` — что уже звучало (``track_id``).
    """

    request: str
    anchors: SpecAnchors = field(default_factory=SpecAnchors)
    seed: int = 0
    persona: Optional[str] = None
    theme: Optional[str] = None
    played: Tuple[str, ...] = ()


@dataclass(frozen=True)
class ProviderReply:
    """Сырой ответ провайдера: аргументы ``submit_track_spec`` и/или текст."""

    tool_args: Optional[Mapping[str, Any]] = None
    text: str = ""
    thinking: bool = False


class ReasonerProvider(Protocol):
    """Провайдер ризонера: один ход рассуждения → :class:`ProviderReply`.

    Реализация обязана включать reasoning/thinking явно и сама не ретраит.
    """

    name: str

    async def propose(self, inp: TrackInput, schema: Mapping[str, Any]) -> ProviderReply:
        ...


@dataclass(frozen=True)
class ReasonerResult:
    """Итог ризонера. ``spec=None`` — оставить то, что играет (превью)."""

    outcome: str
    spec: Optional[TrackSpec] = None
    provider: Optional[str] = None
    detail: str = ""
    latency_s: float = 0.0
    thinking: bool = False


def extract_spec_payload(reply: ProviderReply) -> Optional[Dict[str, Any]]:
    """Достать сырую спеку: аргументы тула, иначе РОВНО один JSON-объект в тексте.

    Два и больше JSON-блоков — неоднозначно, ``None`` (не угадываем).
    """
    if isinstance(reply.tool_args, Mapping):
        args = dict(reply.tool_args)
        inner = args.get("spec")
        return dict(inner) if isinstance(inner, Mapping) and len(args) == 1 else args
    text = (reply.text or "").strip()
    blocks = _FENCED_JSON.findall(text)
    candidates = blocks if blocks else [text]
    if len(candidates) != 1:
        return None
    try:
        payload = json.loads(candidates[0])
    except (TypeError, ValueError):
        return None
    return payload if isinstance(payload, dict) else None


def _judge(reply: ProviderReply, inp: TrackInput) -> Tuple[str, Optional[TrackSpec], str]:
    payload = extract_spec_payload(reply)
    if payload is None:
        return OUTCOME_INVALID, None, "в ответе нет спеки (ни submit_track_spec, ни одного JSON-блока)"
    try:
        return OUTCOME_OK, validate_spec(payload, inp.anchors), ""
    except SpecError as exc:
        return OUTCOME_INVALID, None, str(exc)


async def reason_track(
    inp: TrackInput,
    providers: Sequence[ReasonerProvider],
    deadline_s: float,
    clock: Callable[[], float] = time.monotonic,
) -> ReasonerResult:
    """Пройти цепочку провайдеров в пределах ОДНОГО дедлайна.

    Первый валидный ответ — ``ok``. Невалидный ответ или исключение —
    следующий провайдер (без повтора того же). Кончилось время — ``late``.
    Исключения наружу не выходят.
    """
    start = clock()
    schema = track_spec_schema()
    last = ReasonerResult(outcome=OUTCOME_NO_PROVIDER, detail="цепочка провайдеров пуста")
    for provider in providers:
        name = getattr(provider, "name", type(provider).__name__)
        remaining = deadline_s - (clock() - start)
        if remaining <= 0:
            return ReasonerResult(OUTCOME_LATE, provider=name, detail="дедлайн исчерпан до вызова",
                                  latency_s=clock() - start)
        try:
            reply = await asyncio.wait_for(provider.propose(inp, schema), timeout=remaining)
        except asyncio.TimeoutError:
            return ReasonerResult(OUTCOME_LATE, provider=name, detail=f"не успел за {deadline_s:g} с",
                                  latency_s=clock() - start)
        except Exception as exc:  # noqa: BLE001 — провайдер не должен ронять владельца плеера
            last = ReasonerResult(OUTCOME_ERROR, provider=name, detail=f"{type(exc).__name__}: {exc}",
                                  latency_s=clock() - start)
            continue
        outcome, spec, detail = _judge(reply, inp)
        last = ReasonerResult(outcome, spec=spec, provider=name, detail=detail,
                              latency_s=clock() - start, thinking=bool(reply.thinking))
        if outcome == OUTCOME_OK:
            return last
    return last


class ReasonerJob:
    """Ризонер в фоновом потоке со своим event loop (ADR-0142 §8.3, вариант А).

    ``on_done(result)`` вызывается ровно один раз из фонового потока;
    исключение в нём логируется и не роняет поток. Пока джоб думает,
    играет превью — вызывающий ничего не ждёт.
    """

    def __init__(
        self,
        inp: TrackInput,
        providers: Sequence[ReasonerProvider],
        deadline_s: float,
        on_done: Callable[[ReasonerResult], None],
    ) -> None:
        self._inp = inp
        self._providers = tuple(providers)
        self._deadline_s = deadline_s
        self._on_done = on_done
        self._result: Optional[ReasonerResult] = None
        self._thread = threading.Thread(target=self._run, name="track-reasoner", daemon=True)

    @property
    def result(self) -> Optional[ReasonerResult]:
        return self._result

    def start(self) -> "ReasonerJob":
        self._thread.start()
        return self

    def join(self, timeout: Optional[float] = None) -> Optional[ReasonerResult]:
        self._thread.join(timeout)
        return self._result

    def _run(self) -> None:
        try:
            result = asyncio.run(reason_track(self._inp, self._providers, self._deadline_s))
        except Exception as exc:  # noqa: BLE001 — последний рубеж: поток не должен умереть молча
            result = ReasonerResult(OUTCOME_ERROR, detail=f"{type(exc).__name__}: {exc}")
        self._result = result
        logger.info(
            "[#3136] reasoner_outcome=%s provider=%s latency=%.2fs thinking=%s %s",
            result.outcome, result.provider, result.latency_s,
            "on" if result.thinking else "off", result.detail,
        )
        try:
            self._on_done(result)
        except Exception:  # noqa: BLE001
            logger.exception("[#3136] on_done ризонера упал")


__all__ = [
    "OUTCOME_ERROR",
    "OUTCOME_INVALID",
    "OUTCOME_LATE",
    "OUTCOME_NO_PROVIDER",
    "OUTCOME_OK",
    "ProviderReply",
    "ReasonerJob",
    "ReasonerProvider",
    "ReasonerResult",
    "SUBMIT_TOOL_NAME",
    "TrackInput",
    "extract_spec_payload",
    "reason_track",
]
