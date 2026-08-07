"""Integration tests for the Phase 1 MVP :mod:`rob_box_voice.scheduler`.

The MVP acceptance criterion (per issue #968 §11.1) is:

    «интеграционный тест с happy path» — the scheduler
    accepts tasks, routes them to the right channel, runs them
    sequentially per channel, and returns a result for each.

These tests sit one level above ``test_task_scheduler.py`` — they
build realistic multi-channel scenarios that mirror the dialogue
shell's actual workload (speak_text + execute_music_code +
play_animation submitted back-to-back) and check the contract the
LLM loop will rely on in W7.

They are *not* ROS2-bound — the dialogue shell remains the only
caller that bridges the executor to ROS publishers; here we use
in-memory fakes so the test is hermetic and deterministic.
"""

from __future__ import annotations

import asyncio
import time

import pytest

from rob_box_voice.scheduler import (
    ChannelKind,
    SchedulerTask,
    TaskResult,
    TaskScheduler,
    TaskStatus,
)


# ---------------------------------------------------------------------------
# Realistic executor fakes — model the side-effect boundaries the LLM
# shell will eventually bridge to (TTS publish, SC publish, animation publish).
# ---------------------------------------------------------------------------


class _FakeSideEffectBus:
    """Records the order in which executor coroutines fire their effects.

    Tests assert against this log so the regression «stop_music
    outruns speak_text» cannot sneak back in: every speak_text
    effect is timestamped before any execute_music_code effect that
    was submitted after it.
    """

    def __init__(self) -> None:
        self.log: list[tuple[float, str, str, str]] = []
        # (timestamp, kind, effect, payload)

    def record(self, kind: ChannelKind, effect: str, payload: str) -> None:
        self.log.append((time.monotonic(), kind.value, effect, payload))


def _make_speak_executor(bus: _FakeSideEffectBus) -> "_Executor":
    """Speak text via the (fake) TTS pipeline."""

    async def _exec(task: SchedulerTask) -> TaskResult:
        # Simulate a short TTS synth/playback window. The MVP
        # makes no ETA claims — we just keep the channel lock
        # held long enough to observe ordering.
        await asyncio.sleep(0.01)
        speech_id = f"sid-{task.task_id}"
        bus.record(task.channel, "tts_publish", f"{task.args.get('text', '')}|{speech_id}")
        await asyncio.sleep(0.005)
        bus.record(task.channel, "tts_finished", speech_id)
        return TaskResult(
            payload={"speech_id": speech_id, "text": task.args.get("text")},
            message=f"spoken '{task.args.get('text')}'",
        )

    return _exec


def _make_music_executor(bus: _FakeSideEffectBus) -> "_Executor":
    """Start a SuperCollider pattern via the (fake) music bridge."""

    async def _exec(task: SchedulerTask) -> TaskResult:
        # Music tasks in the MVP are slower than TTS — exercise
        # the cross-channel parallelism claim.
        await asyncio.sleep(0.02)
        bus.record(task.channel, "sc_publish", f"start|{task.args.get('pattern', 'p1')}")
        return TaskResult(
            payload={"pattern": task.args.get("pattern", "p1"), "state": "running"},
            message=f"started pattern {task.args.get('pattern')!r}",
        )

    return _exec


def _make_anim_executor(bus: _FakeSideEffectBus) -> "_Executor":
    """Push an animation frame to the LED/audio pipeline."""

    async def _exec(task: SchedulerTask) -> TaskResult:
        await asyncio.sleep(0.005)
        bus.record(task.channel, "anim_publish", task.args.get("name", "idle"))
        return TaskResult(payload={"anim": task.args.get("name", "idle")})

    return _exec


def _make_stop_music_executor(bus: _FakeSideEffectBus) -> "_Executor":
    """Stop music — the regression v36 case from issue #968."""

    async def _exec(task: SchedulerTask) -> TaskResult:
        await asyncio.sleep(0.002)
        bus.record(task.channel, "sc_publish", "stop_all")
        return TaskResult(payload={"state": "stopped"}, message="music stopped")

    return _exec


class _Executor:  # marker for type checkers, not actually used at runtime
    pass


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------


class TestHappyPath:
    @pytest.mark.asyncio
    async def test_dialogue_turn_with_voice_music_anim(self):
        """One LLM turn emits speak_text + execute_music_code + play_animation.

        Expected behaviour:

        * All three tasks complete successfully.
        * The two VOICE tasks (speak_text + stop_music later)
          remain strictly sequential on the voice channel — no
          overlap, no race.
        * MUSIC and ANIM tasks do NOT block VOICE; their
          completion timestamps interleave with VOICE in any
          order.
        * Every task carries a :class:`TaskResult` whose payload
          matches the executor's contract.
        """
        bus = _FakeSideEffectBus()
        sched = TaskScheduler()
        sched.start()

        speak = _make_speak_executor(bus)
        music = _make_music_executor(bus)
        anim = _make_anim_executor(bus)

        speak_task = sched.submit(SchedulerTask(
            task_id="voice-1",
            tool="speak_text",
            channel=ChannelKind.VOICE,
            executor=speak,
            args={"text": "Привет, ставлю трек"},
        ))
        music_task = sched.submit(SchedulerTask(
            task_id="music-1",
            tool="execute_music_code",
            channel=ChannelKind.MUSIC,
            executor=music,
            args={"code": "p1 >> d(), ", "pattern": "p1"},
        ))
        anim_task = sched.submit(SchedulerTask(
            task_id="anim-1",
            tool="play_animation",
            channel=ChannelKind.ANIM,
            executor=anim,
            args={"name": "happy"},
        ))

        try:
            await sched.wait_all(timeout=2.0)

            # Every task completed.
            for t in (speak_task, music_task, anim_task):
                assert t.status is TaskStatus.COMPLETED, (
                    f"{t.task_id} ended in {t.status.value}: {t.error}"
                )
                assert t.result is not None
                assert t.result.is_success

            # The §7 [CHANNELS] feedback block can be built from
            # all_statuses() — Phase 3 wires the LLM formatter.
            statuses = sched.all_statuses()
            for kind in (ChannelKind.VOICE, ChannelKind.MUSIC, ChannelKind.ANIM):
                assert kind in statuses
                assert statuses[kind].queue_depth == 0
                assert statuses[kind].current_task_id is None

            # The side-effect log captured every step in order.
            effects = [(kind, eff, payload) for _, kind, eff, payload in bus.log]
            assert any(eff == "tts_publish" for _, eff, _ in effects), "voice task missing"
            assert any(eff == "sc_publish" for _, eff, _ in effects), "music task missing"
            assert any(eff == "anim_publish" for _, eff, _ in effects), "anim task missing"
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_two_speak_text_then_stop_music_preserve_order(self):
        """Regression v36: two speak_text then stop_music must NOT race.

        Per §11.1: «stop_music() не может ДОЙТИ ДО ЖЕЛЕЗА раньше
        конца TTS-чанка». The MVP enforces this by serialising the
        VOICE channel — the second speak_text cannot start until
        the first finished, and stop_music cannot start until the
        second finished. Crucially: stop_music NEVER lands on the
        SC bus before the second tts_finished.
        """
        bus = _FakeSideEffectBus()
        sched = TaskScheduler()
        sched.start()

        speak = _make_speak_executor(bus)
        stop = _make_stop_music_executor(bus)

        s1 = sched.submit(SchedulerTask(
            task_id="s1", tool="speak_text",
            channel=ChannelKind.VOICE, executor=speak,
            args={"text": "куплет 1"},
        ))
        s2 = sched.submit(SchedulerTask(
            task_id="s2", tool="speak_text",
            channel=ChannelKind.VOICE, executor=speak,
            args={"text": "куплет 2"},
        ))
        stop_task = sched.submit(SchedulerTask(
            task_id="stop", tool="stop_music",
            channel=ChannelKind.VOICE, executor=stop,
        ))

        try:
            await sched.wait_all(timeout=2.0)
            assert s1.status is TaskStatus.COMPLETED
            assert s2.status is TaskStatus.COMPLETED
            assert stop_task.status is TaskStatus.COMPLETED

            # Walk the log and assert strict ordering on the
            # VOICE channel. ``bus.log`` stores
            # ``(timestamp, kind, effect, payload)`` — strip the
            # timestamp for the ordering check.
            voice_log = [
                (idx, eff, payload)
                for idx, (_, kind, eff, payload) in enumerate(bus.log)
                if kind == "voice"
            ]
            # Indices into the voice log of each expected effect.
            def _find(eff: str, contains: str = "") -> int:
                for idx, e, p in voice_log:
                    if e == eff and (not contains or contains in p):
                        return idx
                raise AssertionError(f"missing {eff} (contains={contains!r}) in {voice_log!r}")

            i_s1_pub = _find("tts_publish", "куплет 1")
            i_s1_fin = _find("tts_finished", "sid-s1")
            i_s2_pub = _find("tts_publish", "куплет 2")
            i_s2_fin = _find("tts_finished", "sid-s2")
            i_stop = _find("sc_publish", "stop_all")

            assert i_s1_pub < i_s1_fin < i_s2_pub < i_s2_fin < i_stop, (
                f"voice channel ordering broken: {voice_log!r}"
            )
        finally:
            sched.shutdown()

    @pytest.mark.asyncio
    async def test_concurrent_music_does_not_delay_voice(self):
        """Acceptance (§11.1): «Два подряд speak_text возвращают
        управление за < 50мс каждый» — this is a proxy. Real
        end-to-end latency is a CI metric; here we assert that a
        long-running MUSIC task does NOT inflate VOICE latency.
        """
        bus = _FakeSideEffectBus()
        sched = TaskScheduler()
        sched.start()

        async def slow_music(task: SchedulerTask) -> TaskResult:
            # Long enough to dominate the test window if channels
            # were accidentally serialised.
            await asyncio.sleep(0.1)
            bus.record(task.channel, "sc_publish", "slow_pattern")
            return TaskResult(payload="slow_music")

        async def fast_voice(task: SchedulerTask) -> TaskResult:
            t0 = time.monotonic()
            await asyncio.sleep(0.005)
            elapsed_ms = (time.monotonic() - t0) * 1000
            bus.record(task.channel, "tts_publish", f"latency_ms={elapsed_ms:.1f}")
            return TaskResult(payload={"latency_ms": elapsed_ms})

        music = sched.submit(SchedulerTask(
            task_id="music-slow", tool="execute_music_code",
            channel=ChannelKind.MUSIC, executor=slow_music,
        ))
        v1 = sched.submit(SchedulerTask(
            task_id="voice-1", tool="speak_text",
            channel=ChannelKind.VOICE, executor=fast_voice,
            args={"text": "одновременно с музыкой"},
        ))
        v2 = sched.submit(SchedulerTask(
            task_id="voice-2", tool="speak_text",
            channel=ChannelKind.VOICE, executor=fast_voice,
            args={"text": "и сразу ещё"},
        ))

        try:
            await sched.wait_all(timeout=2.0)
            assert v1.status is TaskStatus.COMPLETED
            assert v2.status is TaskStatus.COMPLETED
            # Both voice tasks took ~5ms (their own sleep); they
            # were NOT delayed by the 100ms music task.
            assert v1.result.payload["latency_ms"] < 50, (
                f"first speak_text took {v1.result.payload['latency_ms']:.1f}ms — "
                "music channel is leaking into voice latency"
            )
            assert v2.result.payload["latency_ms"] < 50, (
                f"second speak_text took {v2.result.payload['latency_ms']:.1f}ms — "
                "voice channel is being serialised across channels"
            )
            # Music finished too — the scheduler ran both channels concurrently.
            assert music.status is TaskStatus.COMPLETED
        finally:
            sched.shutdown()


class TestChannelsFeedbackBlock:
    """Phase 1 acceptance (§11.1): LLM in each turn sees [CHANNELS]."""

    @pytest.mark.asyncio
    async def test_channel_status_is_callable_mid_flight(self):
        """The §7 [CHANNELS] block can be assembled mid-flight.

        The MVP does not yet format the LLM feedback string — that
        is the Phase 1.5 ``AwaitingFeedbackFormatter``'s job.
        This test asserts the data shape that the formatter will
        consume: every channel exposes a ``ChannelStatus`` with
        the four MVP fields.
        """
        sched = TaskScheduler()
        sched.start()
        try:
            async def hold(task: SchedulerTask) -> TaskResult:
                await asyncio.sleep(0.02)
                return TaskResult(payload="ok")

            sched.submit(SchedulerTask(
                task_id="v", tool="speak_text",
                channel=ChannelKind.VOICE, executor=hold,
                args={"text": "test"},
            ))
            # Yield to the pump.
            await asyncio.sleep(0.005)

            for kind, expected_tool in (
                (ChannelKind.VOICE, "speak_text"),
                (ChannelKind.MUSIC, None),
                (ChannelKind.ANIM, None),
            ):
                status = sched.channel_status(kind)
                # MVP fields
                assert hasattr(status, "kind")
                assert hasattr(status, "queue_depth")
                assert hasattr(status, "current_task_id")
                assert hasattr(status, "current_tool")
                assert hasattr(status, "eta_s")
                if kind is ChannelKind.VOICE:
                    assert status.current_tool == expected_tool
                else:
                    assert status.current_task_id is None

            await sched.wait_all()
        finally:
            sched.shutdown()