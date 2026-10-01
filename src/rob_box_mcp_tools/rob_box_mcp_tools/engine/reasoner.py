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
``provider``, ``latency_ms`` и p50/p95 по провайдеру; у каждого стартовавшего трека — ``source=theme|seeded``
(A11: доля треков по профилю темы от LLM).
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
                 deadline_s: float = DEADLINE_S, hype: bool = False, logger: Any = None,
                 breaker: Optional[CircuitBreaker] = None, clock: Callable[[], float] = time.monotonic,
                 spawn: Callable[[Callable[[], None]], None] = lambda fn: threading.Thread(
                     target=fn, name="rbx-set-reasoner", daemon=True).start()) -> None:
        self.enabled = enabled
        self.hype = hype
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
        from rob_box_llm.provider import LLMMessage, LLMSettings

        system, user = rz.prompt(theme, plan.profile, self.hype)
        if self._client is None:
            self._client = self._provider()  # нет ключа — исключение, исход error; следующий сет попробует снова
        try:
            response = await asyncio.wait_for(self._client.complete(
                [LLMMessage("system", system), LLMMessage("user", user)],
                tools=[rz.tool(plan.profile.genre, self.hype)],
                settings=LLMSettings(tool_choice="auto", max_tokens=1024)), timeout=self._deadline)
        except asyncio.TimeoutError:
            return "late", None, f"нет ответа за {self._deadline:g} с"
        try:
            return "ok", rz.validate(payload_of(response), plan.profile.genre, self.hype), ""
        except rz.PlanInvalid as exc:
            return "invalid", None, f"plan_invalid{{{exc.path}}} {exc}"

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
        (self._log.info if outcome in ("ok", "disabled") else self._log.warning)(line)
        return outcome


def _ms(value: Optional[float]) -> str:
    return "-" if value is None else f"{value:.0f}"


class SetPlanBox:
    """Текущий план сета: seeded до ответа LLM, затем — поправленный (один раз). Источник для ``plan_source``."""

    def __init__(self, plan: SetPlan, lookup: Callable[[Iterable[str]], Dict[str, str]], *,
                 speak: Optional[Callable[[str], None]] = None, logger: Any = None) -> None:
        self._lock = threading.Lock()
        self._plan = plan
        self._lookup = lookup
        self._melodies: Mapping[str, str] = lookup(plan.profile.hook_ids)
        self._refined: Optional[rz.Refinement] = None
        self._sources: Dict[str, str] = {}  # track_id → theme|seeded на момент компоновки
        self._played: Dict[str, str] = {}
        self._speak = speak
        self._log = logger or _LOG

    def current(self) -> Tuple[SetPlan, Optional[Mapping[str, str]]]:
        with self._lock:
            return self._plan, self._melodies

    def compose_mark(self, track: Any) -> Any:
        with self._lock:
            self._sources[track.track_id] = "theme" if self._refined else "seeded"
        return track

    def apply(self, ref: rz.Refinement) -> None:
        """Поправка LLM: план со следующего не сыгранного трека; темп сета прежний (``reasoner.apply``)."""
        melodies = self._lookup(ref.hook_ids)
        with self._lock:
            self._plan, self._melodies, self._refined = rz.apply(self._plan, ref), melodies, ref
        self._log.info(f"🧠 [reasoner] {self._plan.set_id} план применён: row={ref.row} mode={ref.mode} "
                       f"hooks={list(ref.hook_ids)} найдено={sorted(melodies)} energy={list(ref.energy)} "
                       f"bpm={self._plan.bpm} (темп сета прежний)")

    def on_started(self, track_id: str) -> str:
        """Пометка ``source=…`` для лога ``started``; первый трек по профилю LLM — выкрик, если включён."""
        with self._lock:
            source = self._sources.get(track_id, "seeded")
            first_theme = source == "theme" and "theme" not in self._played.values()
            self._played[track_id] = source
            line = self._refined.hype_line if self._refined else None
            theme = sum(1 for s in self._played.values() if s == "theme")
            note = f"source={source} A11={theme}/{len(self._played)}"
        if first_theme and line and self._speak is not None:
            threading.Thread(target=self._say, args=(line,), name="rbx-hype", daemon=True).start()
            note += " hype=on"
        return note

    def _say(self, line: str) -> None:
        try:
            self._speak(line)  # type: ignore[misc]
        except Exception as exc:  # noqa: BLE001 — выкрик не роняет поток и не трогает музыку
            self._log.warning(f"⚠️ [reasoner] выкрик не озвучен: {type(exc).__name__}: {exc}")


__all__ = ["BREAKER_FAILURES", "BREAKER_RESET_S", "DEADLINE_S", "PROVIDER", "ProviderFactory", "SetPlanBox",
           "SetReasoner", "minimax_provider", "payload_of"]
