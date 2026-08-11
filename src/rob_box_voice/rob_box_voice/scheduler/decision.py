"""Explicit contract between the high-level planner and low-level executor."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Awaitable, Callable, Mapping, Protocol, Sequence, runtime_checkable

from .event_bus import EventBus, EventEnvelope
from .task_scheduler import SchedulerTask, TaskScheduler, TaskStatus


class StepStatus(str, Enum):
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    SKIPPED = "SKIPPED"


@dataclass(frozen=True)
class PlanStep:
    """Validated low-level command emitted by a planner."""

    step_id: str
    action: str
    arguments: Mapping[str, Any] = field(default_factory=dict)
    channel: str = "voice"
    depends_on: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.step_id:
            raise ValueError("step_id must not be empty")
        if not self.action:
            raise ValueError("action must not be empty")


@dataclass(frozen=True)
class DecisionPlan:
    """Planner output. Dependencies must reference earlier steps only."""

    plan_id: str
    steps: tuple[PlanStep, ...]
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.plan_id:
            raise ValueError("plan_id must not be empty")
        known: set[str] = set()
        all_ids = {step.step_id for step in self.steps}
        if len(all_ids) != len(self.steps):
            raise ValueError("duplicate step_id in plan")
        for step in self.steps:
            for dependency in step.depends_on:
                if dependency not in all_ids:
                    raise ValueError(
                        f"unknown dependency {dependency!r} for step {step.step_id!r}"
                    )
                if dependency not in known:
                    raise ValueError(
                        f"dependency {dependency!r} must reference an earlier step"
                    )
            known.add(step.step_id)


@dataclass(frozen=True)
class StepExecution:
    step_id: str
    status: StepStatus
    result: Any = None
    error: str | None = None


@dataclass(frozen=True)
class PlanExecution:
    plan_id: str
    status: StepStatus
    steps: tuple[StepExecution, ...]


@runtime_checkable
class HighLevelPlanner(Protocol):
    async def plan(self, request: object) -> DecisionPlan:
        ...


@runtime_checkable
class LowLevelExecutor(Protocol):
    async def execute(self, step: PlanStep) -> StepExecution:
        ...


class DecisionCoordinator:
    """Runs planning once, then executes validated steps deterministically."""

    def __init__(
        self,
        planner: HighLevelPlanner,
        executor: LowLevelExecutor,
        *,
        event_bus: EventBus | None = None,
    ) -> None:
        self._planner = planner
        self._executor = executor
        self._event_bus = event_bus

    async def run(self, request: object) -> PlanExecution:
        await self._publish("decision.planning.started", {"request": request})
        plan = await self._planner.plan(request)
        if not isinstance(plan, DecisionPlan):
            raise TypeError("planner must return DecisionPlan")
        await self._publish(
            "decision.plan.created",
            {"plan_id": plan.plan_id, "step_count": len(plan.steps)},
            correlation_id=plan.plan_id,
        )

        executions: list[StepExecution] = []
        by_id: dict[str, StepExecution] = {}
        for step in plan.steps:
            blocked_by = [
                dependency
                for dependency in step.depends_on
                if by_id[dependency].status is not StepStatus.COMPLETED
            ]
            if blocked_by:
                execution = StepExecution(
                    step.step_id,
                    StepStatus.SKIPPED,
                    error=f"dependency failed: {', '.join(blocked_by)}",
                )
            else:
                try:
                    execution = await self._executor.execute(step)
                    if execution.step_id != step.step_id:
                        raise ValueError("executor returned mismatched step_id")
                except Exception as exc:  # executor boundary must isolate failures
                    execution = StepExecution(
                        step.step_id,
                        StepStatus.FAILED,
                        error=f"{type(exc).__name__}: {exc}",
                    )
            executions.append(execution)
            by_id[step.step_id] = execution
            await self._publish(
                f"decision.step.{execution.status.value.lower()}",
                {
                    "plan_id": plan.plan_id,
                    "step_id": step.step_id,
                    "error": execution.error,
                },
                correlation_id=plan.plan_id,
            )

        overall = (
            StepStatus.FAILED
            if any(item.status is StepStatus.FAILED for item in executions)
            else StepStatus.COMPLETED
        )
        result = PlanExecution(plan.plan_id, overall, tuple(executions))
        await self._publish(
            f"decision.plan.{overall.value.lower()}",
            {"plan_id": plan.plan_id},
            correlation_id=plan.plan_id,
        )
        return result

    async def _publish(
        self,
        topic: str,
        payload: object,
        *,
        correlation_id: str | None = None,
    ) -> None:
        if self._event_bus is not None:
            await self._event_bus.publish(
                EventEnvelope(topic, payload, correlation_id=correlation_id)
            )


TaskFactory = Callable[[PlanStep], Awaitable[SchedulerTask] | SchedulerTask]


class SchedulerStepExecutor:
    """Backward-compatible adapter from PlanStep to Phase 1 TaskScheduler."""

    def __init__(self, scheduler: TaskScheduler, task_factory: TaskFactory) -> None:
        self._scheduler = scheduler
        self._task_factory = task_factory

    async def execute(self, step: PlanStep) -> StepExecution:
        task = self._task_factory(step)
        if hasattr(task, "__await__"):
            task = await task  # type: ignore[misc, assignment]
        if not isinstance(task, SchedulerTask):
            raise TypeError("task_factory must return SchedulerTask")
        scheduled = self._scheduler.submit(task)
        await self._scheduler.wait_all()
        if scheduled.status is TaskStatus.COMPLETED:
            result = scheduled.result.payload if scheduled.result is not None else None
            return StepExecution(step.step_id, StepStatus.COMPLETED, result=result)
        return StepExecution(
            step.step_id,
            StepStatus.FAILED,
            error=scheduled.error or scheduled.status.value,
        )
