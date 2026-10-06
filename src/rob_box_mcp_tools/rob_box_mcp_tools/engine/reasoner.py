"""Фоновый ризонер плана сета в процессе плеера (ADR-0149 §4.5, §4.7, §4.8; ADR-0142 §8.2; PR-10).

Сет стартует сразу с seeded-плана — звук никогда не ждёт LLM. После старта :class:`SetReasoner` в своём
потоке один раз спрашивает LLM (MiniMax — решение Шифу В1; тот же провайдер и кошелёк, что у голоса:
``rob_box_harness.providers.build_minimax_provider``, ключ ``MINIMAX_API_KEY``) профиль сета по схеме
``rob_box_music.reasoner``. Пришёл до дедлайна и прошёл валидатор — :class:`SetPlanBox` подменяет план, и
со следующего ещё не сыгранного трека сет звучит по нему; иначе весь сет — seeded.

Инварианты (ADR-0142 §8.2): один жёсткий дедлайн; ретраев нет (``RetryPolicy(max_attempts=1)``, невалидное —
seeded, ADR-0148); исключения наружу не выходят; circuit breaker на провайдера — 3 ошибки/опоздания подряд,
и он пропускается 10 мин (§4.8), сет этого не замечает. ``enabled=False`` — ноль вызовов.

Метрики — в лог одной строкой на сет: ``plan_outcome=ok|late|invalid|error|circuit_open|disabled``,
``provider``, ``latency_ms`` и p50/p95 по провайдеру; у каждого стартовавшего трека — ``source=theme|pool|motif``
(A11: доля треков с хуком, найденным по словам темы) и ``plan=llm|seeded``.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Tuple

from rob_box_harness.decision.health import CircuitBreaker, DecisionMetrics
from rob_box_music import reasoner as rz
from rob_box_music.set_plan import SetPlan

_LOG = logging.getLogger(__name__)

PROVIDER = "minimax"
#: Дедлайн до ``nearly_finished`` трека 1 (§4.8: ≈ 45–60 с; форма 48 тактов при 128–138 BPM ≈ 83–90 с).
DEADLINE_S = 45.0
BREAKER_FAILURES = 3
BREAKER_RESET_S = 600.0

#: ``() -> LLMProvider`` (``complete(messages, tools=, settings=)``); строится в потоке ризонера.
ProviderFactory = Callable[[], Any]


def minimax_provider(timeout_s: float = DEADLINE_S) -> ProviderFactory:
    """Провайдер голосового стека без ретраев SDK и харнесса: следующий шанс — следующий сет."""
    def build() -> Any:
        from rob_box_harness.config import LLMConfig
        from rob_box_harness.providers import build_minimax_provider
        from rob_box_harness.providers.retry import RetryPolicy

        return build_minimax_provider(LLMConfig(provider=PROVIDER, timeout_s=timeout_s),
                                      retry=RetryPolicy(max_attempts=1))
    return build


def payload_of(response: Any) -> Any:
    """Аргументы ``submit_set_profile``; без вызова — весь текст ответа как JSON (без вырезания кусков)."""
    if getattr(response, "truncated_tool_args", False):
        raise rz.PlanInvalid("$", "аргументы вызова обрезаны")
    for call in getattr(response, "tool_calls", ()) or ():
        if call.name == rz.SUBMIT_TOOL:
            return dict(call.arguments)
    try:
        return json.loads(getattr(response, "content", "") or "")
    except ValueError:
        raise rz.PlanInvalid("$", "нет вызова submit_set_profile и ответ не JSON") from None


class SetReasoner:
    """Один вызов LLM на сет, в фоне. Живёт дольше сетов: breaker и метрики общие для всех сетов."""

    def __init__(self, provider: Optional[ProviderFactory] = None, *, enabled: bool = True,
                 deadline_s: float = DEADLINE_S, logger: Any = None,
                 breaker: Optional[CircuitBreaker] = None, clock: Callable[[], float] = time.monotonic,
                 spawn: Callable[[Callable[[], None]], None] = lambda fn: threading.Thread(
                     target=fn, name="rbx-set-reasoner", daemon=True).start()) -> None:
        self.enabled = enabled
        self._provider = provider or minimax_provider(deadline_s)
        self._deadline = float(deadline_s)
        self._log = logger or _LOG
        self._breaker = breaker or CircuitBreaker(failure_threshold=BREAKER_FAILURES, reset_after_s=BREAKER_RESET_S,
                                                  clock=clock)
        self.metrics = DecisionMetrics()
        self._clock = clock
        self._spawn = spawn
        self._loop: Optional[asyncio.AbstractEventLoop] = None  # один loop на процесс: клиент LLM живёт в нём
        self._client: Any = None
        self._loop_lock = threading.Lock()

    def request(self, set_id: str, theme: str, plan: SetPlan,
                on_ok: Callable[[rz.Refinement], None]) -> str:
        """Запустить ризонер сета. Возвращает сразу: ``started`` или исход без вызова (``disabled``…)."""
        if not self.enabled:
            return self._outcome(set_id, "disabled", 0.0, "reasoner выключен параметром")
        if not self._breaker.allow():
            return self._outcome(set_id, "circuit_open", 0.0, f"{PROVIDER} пропускается после ошибок подряд")
        self._spawn(lambda: self._run(set_id, theme, plan, on_ok))
        return "started"

    def _run(self, set_id: str, theme: str, plan: SetPlan, on_ok: Callable[[rz.Refinement], None]) -> None:
        start = self._clock()
        try:
            outcome, ref, detail = asyncio.run_coroutine_threadsafe(self._ask(theme, plan), self._event_loop()).result()
        except Exception as exc:  # noqa: BLE001 — ризонер не роняет процесс плеера
            outcome, ref, detail = "error", None, f"{type(exc).__name__}: {exc}"
        latency_s = self._clock() - start
        self._judge_provider(outcome, latency_s)
        self._outcome(set_id, outcome, latency_s, detail)
        if ref is not None:
            try:
                on_ok(ref)
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f"⚠️ [reasoner] {set_id} поправка не применена: {type(exc).__name__}: {exc}")

    def _event_loop(self) -> asyncio.AbstractEventLoop:
        """Свой поток с event loop на всё время процесса: HTTP-клиент не переживает чужой закрытый loop."""
        with self._loop_lock:
            if self._loop is None:
                self._loop = asyncio.new_event_loop()
                threading.Thread(target=self._loop.run_forever, name="rbx-reasoner-loop", daemon=True).start()
            return self._loop

    async def _ask(self, theme: str, plan: SetPlan) -> Tuple[str, Optional[rz.Refinement], str]:
        system, user = rz.prompt(theme, plan.profile)
        outcome, response, detail = await self._complete(system, user, rz.tool(plan.style, plan.profile),
                                                         self._deadline, 1024)
        if outcome != "ok":
            return outcome, None, detail
        try:
            return "ok", rz.validate(payload_of(response), plan.style, plan.profile), ""
        except rz.PlanInvalid as exc:
            return "invalid", None, f"plan_invalid{{{exc.path}}} {exc}"

    async def _complete(self, system: str, user: str, tool: Dict[str, Any], deadline_s: float,
                        max_tokens: int) -> Tuple[str, Any, str]:
        """Один вызов провайдера с жёстким дедлайном: ``("ok", ответ, "")`` или ``("late", None, причина)``."""
        from rob_box_llm.provider import LLMMessage, LLMSettings

        if self._client is None:
            self._client = self._provider()  # нет ключа — исключение, исход error; следующий сет попробует снова
        try:
            response = await asyncio.wait_for(self._client.complete(
                [LLMMessage("system", system), LLMMessage("user", user)], tools=[tool],
                settings=LLMSettings(tool_choice="auto", max_tokens=max_tokens)), timeout=deadline_s)
        except asyncio.TimeoutError:
            return "late", None, f"нет ответа за {deadline_s:g} с"
        return "ok", response, ""

    def ask(self, system: str, user: str, tool: Dict[str, Any], deadline_s: float) -> Tuple[str, Any, str]:
        """Синхронный вызов из фонового потока (реплика перехода, ``engine.dj_lines``): тот же провайдер, клиент,
        дедлайн-механика и circuit breaker, что у плана сета. Исключения наружу не выходят."""
        if not self.enabled:
            return "disabled", None, "reasoner выключен параметром"
        if not self._breaker.allow():
            return "circuit_open", None, f"{PROVIDER} пропускается после ошибок подряд"
        start = self._clock()
        try:
            outcome, response, detail = asyncio.run_coroutine_threadsafe(
                self._complete(system, user, tool, deadline_s, 256), self._event_loop()).result()
        except Exception as exc:  # noqa: BLE001 — реплика не роняет процесс плеера
            outcome, response, detail = "error", None, f"{type(exc).__name__}: {exc}"
        self._judge_provider(outcome, self._clock() - start)
        return outcome, response, detail

    def _judge_provider(self, outcome: str, latency_s: float) -> None:
        """Ошибка и опоздание — провайдер болен (кончились деньги, сеть); невалидный ответ — нет."""
        if outcome in ("error", "late"):
            self._breaker.record_failure()
        else:
            self._breaker.record_success()
            self.metrics.observe_latency(PROVIDER, latency_s * 1000.0)

    def _outcome(self, set_id: str, outcome: str, latency_s: float, detail: str) -> str:
        self.metrics.count(f"plan_outcome_{outcome}")
        lat = self.metrics.latency(PROVIDER)
        line = (f"🧠 [reasoner] {set_id} plan_outcome={outcome} provider={PROVIDER} latency_ms={latency_s * 1000:.0f}"
                f" p50_ms={_ms(lat.p50_ms)} p95_ms={_ms(lat.p95_ms)} n={lat.count} {detail}".rstrip())
        if outcome in ("ok", "disabled"):  # rclpy: severity привязана к месту вызова
            self._log.info(line)
        else:
            self._log.warning(line)
        return outcome


def _ms(value: Optional[float]) -> str:
    return "-" if value is None else f"{value:.0f}"


class SetPlanBox:
    """Текущий план сета: seeded до ответа LLM, затем — поправленный (один раз). Источник для ``plan_source``.
    ``lines`` — реплики диджея на переходах (``engine.dj_lines``, §12 В2); ``None`` — переходы молчат."""

    def __init__(self, plan: SetPlan, lookup: Callable[[Iterable[str]], Dict[str, str]], *,
                 lines: Any = None, logger: Any = None) -> None:
        self._lock = threading.Lock()
        self._plan = plan
        self._lookup = lookup
        self._melodies: Mapping[str, str] = lookup(plan.profile.hook_ids)
        self._refined: Optional[rz.Refinement] = None
        self._sources: Dict[str, Tuple[str, str]] = {}  # track_id → (хук: theme|pool|motif, план: llm|seeded)
        self._played: Dict[str, Tuple[str, str]] = {}
        self._lines = lines
        self._log = logger or _LOG

    def current(self) -> Tuple[SetPlan, Optional[Mapping[str, str]]]:
        with self._lock:
            return self._plan, self._melodies

    def compose_mark(self, track: Any, no: Optional[int] = None) -> Any:
        """``source=theme`` (A11) — только хук, найденный по словам темы (``profile.theme_hooks``), а не любой.
        Трек ``no`` скомпонован — реплика на его ``started`` готовится в фоне."""
        hook = getattr(getattr(track, "hook", None), "source", None)
        with self._lock:
            source = "theme" if hook in self._plan.profile.theme_hooks else "pool" if hook else "motif"
            self._sources[track.track_id] = (source, "llm" if self._refined else "seeded")
            plan = self._plan
        if self._lines is not None and no is not None:
            self._lines.prepare(track.track_id, no, plan, hook)
        return track

    def apply(self, ref: rz.Refinement) -> None:
        """Поправка LLM: план со следующего не сыгранного трека; темп сета прежний (``reasoner.apply``)."""
        with self._lock:
            plan = rz.apply(self._plan, ref)
        melodies = self._lookup(plan.profile.hook_ids)  # хуки плана (``reasoner.plan_hooks``), не только выбор LLM
        with self._lock:
            self._plan, self._melodies, self._refined = plan, melodies, ref
        self._log.info(f"🧠 [reasoner] {self._plan.set_id} план применён: row={ref.row} mode={ref.mode} "
                       f"hooks={list(plan.profile.hook_ids)} llm_hooks={list(ref.hook_ids)} найдено={len(melodies)} "
                       f"energy={list(ref.energy)} "
                       f"bpm={self._plan.bpm} (темп сета прежний)")

    def on_started(self, track_id: str) -> str:
        """Пометка ``source=…`` для лога ``started``; реплика диджея на этот трек — одна (I23), в фоне."""
        with self._lock:
            source, plan = self._sources.get(track_id, ("motif", "seeded"))
            self._played[track_id] = (source, plan)
            theme = sum(1 for s, _p in self._played.values() if s == "theme")
            note = f"source={source} plan={plan} A11={theme}/{len(self._played)}"
        if self._lines is not None:
            note = f"{note} {self._lines.on_started(track_id)}".rstrip()
        return note


__all__ = ["BREAKER_FAILURES", "BREAKER_RESET_S", "DEADLINE_S", "PROVIDER", "ProviderFactory", "SetPlanBox",
           "SetReasoner", "minimax_provider", "payload_of"]
