"""Дедлайн хода (ADR-0150 §8, В5; PR-8 #3702): остаток, таймаут провайдера, отмена ожидания."""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.deadline import TIMEOUT_PHRASE, DeadlineExpired, TurnDeadline, latency_line


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_budget_is_knowledge_value():
    assert TurnDeadline().budget_s == kn.TURN_DEADLINE_S == 12


def test_left_and_expired_follow_clock():
    clock = Clock()
    d = TurnDeadline(clock=clock)
    assert d.left() == 12.0 and not d.expired()
    clock.t += 9.5
    assert d.left() == pytest.approx(2.5)
    clock.t += 5
    assert d.left() == 0.0 and d.expired()


def test_cap_is_min_of_provider_timeout_and_left():
    clock = Clock()
    d = TurnDeadline(clock=clock)
    assert d.cap(30.0) == 12.0
    assert d.cap(4.0) == 4.0
    clock.t += 10
    assert d.cap(4.0) == pytest.approx(2.0)
    assert d.cap(None) == pytest.approx(2.0)
    clock.t += 10
    assert d.cap(4.0) == 0.0


def test_wait_returns_result_in_time():
    async def main():
        return await TurnDeadline().wait(asyncio.sleep(0, result="ok"), "llm")

    assert asyncio.run(main()) == "ok"


def test_wait_cancels_and_raises_when_turn_budget_ends():
    cancelled = []

    async def slow():
        try:
            await asyncio.sleep(5)
        except asyncio.CancelledError:
            cancelled.append(True)
            raise

    async def main():
        with pytest.raises(DeadlineExpired) as err:
            await TurnDeadline(budget_s=0.05).wait(slow(), "llm:deepseek")
        return err.value

    err = asyncio.run(main())
    assert err.what == "llm:deepseek" and cancelled == [True]


def test_provider_timeout_before_deadline_is_plain_timeout():
    async def main():
        with pytest.raises(asyncio.TimeoutError):
            await TurnDeadline(budget_s=10).wait(asyncio.sleep(5), "llm", provider_timeout_s=0.05)

    asyncio.run(main())


def test_timeout_phrase_exists_in_catalog():
    assert phrases.text(TIMEOUT_PHRASE, 0)


def test_latency_line():
    assert latency_line("minimax", "llm", 12001.4, "timeout") == (
        "⏱ [provider_latency] provider=minimax kind=llm latency_ms=12001 status=timeout")
