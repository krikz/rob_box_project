"""Phase 2 unit tests: explicit high-level planner / low-level executor contract."""
from __future__ import annotations

import asyncio

import pytest

from rob_box_voice.scheduler import (
    ChannelKind,
    DecisionCoordinator,
    DecisionPlan,
    EventBus,
    PlanExecution,
    PlanStep,
    SchedulerTask,
    StepExecution,
    StepStatus,
    TaskResult,
    TaskScheduler,
)


class RecordingPlanner:
    def __init__(self, decision_plan: DecisionPlan) -> None:
        self.decision_plan = decision_plan
        self.requests: list[object] = []

    async def plan(self, request: object) -> DecisionPlan:
        self.requests.append(request)
        return self.decision_plan


class RecordingExecutor:
    def __init__(self) -> None:
        self.steps: list[str] = []

    async def execute(self, step: PlanStep) -> StepExecution:
        self.steps.append(step.step_id)
        return StepExecution(
            step_id=step.step_id,
            status=StepStatus.COMPLETED,
            result={"action": step.action},
        )


@pytest.mark.asyncio
async def test_coordinator_validates_contract_and_executes_steps_in_order():
    plan = DecisionPlan(
        plan_id="p1",
        steps=(
            PlanStep(step_id="s1", action="speak", arguments={"text": "hi"}),
            PlanStep(step_id="s2", action="animate", arguments={"name": "happy"}),
        ),
    )
    planner = RecordingPlanner(plan)
    executor = RecordingExecutor()
    coordinator = DecisionCoordinator(planner=planner, executor=executor)

    execution = await coordinator.run({"utterance": "hello"})

    assert planner.requests == [{"utterance": "hello"}]
    assert executor.steps == ["s1", "s2"]
    assert isinstance(execution, PlanExecution)
    assert execution.plan_id == "p1"
    assert execution.status is StepStatus.COMPLETED
    assert [item.step_id for item in execution.steps] == ["s1", "s2"]


def test_plan_rejects_duplicate_step_ids_and_invalid_dependencies():
    with pytest.raises(ValueError, match="duplicate"):
        DecisionPlan(
            plan_id="p1",
            steps=(PlanStep("s", "a"), PlanStep("s", "b")),
        )
    with pytest.raises(ValueError, match="unknown dependency"):
        DecisionPlan(
            plan_id="p2",
            steps=(PlanStep("s", "a", depends_on=("missing",)),),
        )
    with pytest.raises(ValueError, match="earlier step"):
        DecisionPlan(
            plan_id="p3",
            steps=(
                PlanStep("s1", "a", depends_on=("s2",)),
                PlanStep("s2", "b"),
            ),
        )


@pytest.mark.asyncio
async def test_failure_skips_dependents_but_allows_independent_steps():
    plan = DecisionPlan(
        plan_id="p1",
        steps=(
            PlanStep("fail", "failing"),
            PlanStep("dependent", "must-not-run", depends_on=("fail",)),
            PlanStep("independent", "still-runs"),
        ),
    )

    class Executor:
        async def execute(self, step: PlanStep) -> StepExecution:
            if step.step_id == "fail":
                return StepExecution(step.step_id, StepStatus.FAILED, error="boom")
            return StepExecution(step.step_id, StepStatus.COMPLETED)

    result = await DecisionCoordinator(RecordingPlanner(plan), Executor()).run("request")

    assert [step.status for step in result.steps] == [
        StepStatus.FAILED,
        StepStatus.SKIPPED,
        StepStatus.COMPLETED,
    ]
    assert result.status is StepStatus.FAILED


@pytest.mark.asyncio
async def test_coordinator_publishes_lifecycle_events():
    bus = EventBus()
    subscription = bus.subscribe("decision.*", max_queue_size=10)
    plan = DecisionPlan("p1", (PlanStep("s1", "speak"),))

    await DecisionCoordinator(RecordingPlanner(plan), RecordingExecutor(), event_bus=bus).run("x")

    topics = [(await subscription.get()).topic for _ in range(4)]
    assert topics == [
        "decision.planning.started",
        "decision.plan.created",
        "decision.step.completed",
        "decision.plan.completed",
    ]
    subscription.close()
    await bus.close()


@pytest.mark.asyncio
async def test_scheduler_step_executor_adapts_plan_step_without_changing_scheduler_api():
    scheduler = TaskScheduler()
    scheduler.start()

    async def task_factory(step: PlanStep) -> SchedulerTask:
        async def run(task: SchedulerTask) -> TaskResult:
            return TaskResult(payload={"tool": task.tool, **task.args})

        return SchedulerTask(
            task_id=step.step_id,
            tool=step.action,
            channel=ChannelKind(step.channel),
            executor=run,
            args=dict(step.arguments),
        )

    from rob_box_voice.scheduler import SchedulerStepExecutor

    adapter = SchedulerStepExecutor(scheduler, task_factory)
    try:
        result = await adapter.execute(
            PlanStep("s1", "speak_text", {"text": "hi"}, channel="voice")
        )
        assert result.status is StepStatus.COMPLETED
        assert result.result == {"tool": "speak_text", "text": "hi"}
    finally:
        scheduler.shutdown()
