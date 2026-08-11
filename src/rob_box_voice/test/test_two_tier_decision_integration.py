"""Phase 2 integration tests for planner, EventBus and Phase 1 scheduler."""
from __future__ import annotations

import asyncio

import pytest

from rob_box_voice.scheduler import (
    ChannelKind,
    DecisionCoordinator,
    DecisionPlan,
    EventBus,
    PlanStep,
    SchedulerStepExecutor,
    SchedulerTask,
    StepStatus,
    TaskResult,
    TaskScheduler,
)


@pytest.mark.asyncio
async def test_two_tier_plan_runs_through_existing_scheduler_and_emits_events():
    scheduler = TaskScheduler()
    scheduler.start()
    event_bus = EventBus()
    events = event_bus.subscribe("decision.*", max_queue_size=16)
    effects: list[tuple[str, str]] = []

    class Planner:
        async def plan(self, request: object) -> DecisionPlan:
            assert request == "play and greet"
            return DecisionPlan(
                "turn-1",
                (
                    PlanStep("say", "speak_text", {"text": "hello"}, "voice"),
                    PlanStep("music", "execute_music_code", {"pattern": "p1"}, "music"),
                    PlanStep("anim", "play_animation", {"name": "happy"}, "anim", ("say",)),
                ),
            )

    def task_factory(step: PlanStep) -> SchedulerTask:
        async def execute(task: SchedulerTask) -> TaskResult:
            await asyncio.sleep(0)
            effects.append((task.channel.value, task.tool))
            return TaskResult(payload={"task_id": task.task_id})

        return SchedulerTask(
            task_id=step.step_id,
            tool=step.action,
            channel=ChannelKind(step.channel),
            executor=execute,
            args=dict(step.arguments),
        )

    coordinator = DecisionCoordinator(
        Planner(),
        SchedulerStepExecutor(scheduler, task_factory),
        event_bus=event_bus,
    )
    try:
        result = await coordinator.run("play and greet")

        assert result.status is StepStatus.COMPLETED
        assert [step.status for step in result.steps] == [StepStatus.COMPLETED] * 3
        assert effects == [
            ("voice", "speak_text"),
            ("music", "execute_music_code"),
            ("anim", "play_animation"),
        ]
        topics = [(await events.get()).topic for _ in range(6)]
        assert topics[0:2] == ["decision.planning.started", "decision.plan.created"]
        assert topics[-1] == "decision.plan.completed"
        assert topics.count("decision.step.completed") == 3
    finally:
        events.close()
        await event_bus.close()
        scheduler.shutdown()
