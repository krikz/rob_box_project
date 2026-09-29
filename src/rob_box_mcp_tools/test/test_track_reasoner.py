"""Тесты ``core.track_reasoner`` (ADR-0142 §7-§8, issue #3136).

Реального облачного провайдера здесь нет (выбор модели — ADR-0142 §12 В1,
следующий PR). Проверяется контракт: один дедлайн на цепочку, без
ретраев, исключения не выходят наружу, спека — только после валидатора,
fallback (``spec=None``) — всегда.
"""

from __future__ import annotations

import asyncio
import json
import threading

import pytest

from rob_box_mcp_tools.core.track_reasoner import (
    OUTCOME_ERROR,
    OUTCOME_INVALID,
    OUTCOME_LATE,
    OUTCOME_NO_PROVIDER,
    OUTCOME_OK,
    ProviderReply,
    ReasonerJob,
    TrackInput,
    extract_spec_payload,
    reason_track,
)
from rob_box_mcp_tools.core.track_spec import SpecAnchors, seeded_spec, track_spec_schema


class FakeProvider:
    """Провайдер-фейк: отдаёт заготовленный ответ, по желанию — с задержкой/ошибкой."""

    def __init__(self, name, reply=None, delay=0.0, exc=None):
        self.name = name
        self.reply = reply
        self.delay = delay
        self.exc = exc
        self.calls = []

    async def propose(self, inp, schema):
        self.calls.append((inp, schema))
        if self.delay:
            await asyncio.sleep(self.delay)
        if self.exc is not None:
            raise self.exc
        return self.reply


def _spec_dict(**overrides):
    raw = seeded_spec(3).to_dict()
    raw.update(overrides)
    return raw


INP = TrackInput(request="сыграй 8-битного монстра", anchors=SpecAnchors(bpm=124), seed=3)


def _run(coro):
    return asyncio.run(coro)


# --- разбор ответа --------------------------------------------------------


def test_payload_from_tool_args():
    assert extract_spec_payload(ProviderReply(tool_args=_spec_dict())) == _spec_dict()


def test_payload_from_tool_args_wrapped_in_spec_key():
    assert extract_spec_payload(ProviderReply(tool_args={"spec": _spec_dict()})) == _spec_dict()


def test_payload_from_single_fenced_json_block():
    text = "Думаю, это чиптюн.\n```json\n" + json.dumps(_spec_dict()) + "\n```\nГотово."
    assert extract_spec_payload(ProviderReply(text=text)) == _spec_dict()


def test_payload_from_bare_json_text():
    assert extract_spec_payload(ProviderReply(text=json.dumps(_spec_dict()))) == _spec_dict()


@pytest.mark.parametrize("text", [
    "",
    "не знаю",
    "```json\n{\"a\": 1}\n```\n```json\n{\"b\": 2}\n```",  # два блока — неоднозначно
    "```json\n{broken\n```",
    "[1, 2]",
])
def test_no_payload(text):
    assert extract_spec_payload(ProviderReply(text=text)) is None


# --- цепочка провайдеров --------------------------------------------------


def test_ok_first_provider_passes_schema_and_input():
    p = FakeProvider("mimo", ProviderReply(tool_args=_spec_dict(), thinking=True))
    res = _run(reason_track(INP, [p], deadline_s=5))
    assert res.outcome == OUTCOME_OK
    assert res.spec == seeded_spec(3)
    assert res.provider == "mimo" and res.thinking is True
    assert p.calls[0][0] is INP
    assert p.calls[0][1] == track_spec_schema()


def test_invalid_spec_goes_to_next_provider_without_retry():
    bad = FakeProvider("a", ProviderReply(tool_args=_spec_dict(levels={"lead": 1.3})))
    good = FakeProvider("b", ProviderReply(tool_args=_spec_dict()))
    res = _run(reason_track(INP, [bad, good], deadline_s=5))
    assert res.outcome == OUTCOME_OK and res.provider == "b"
    assert len(bad.calls) == 1 and len(good.calls) == 1


def test_all_invalid_gives_invalid_with_path_and_no_spec():
    bad = FakeProvider("a", ProviderReply(tool_args=_spec_dict(levels={"lead": 1.3})))
    res = _run(reason_track(INP, [bad], deadline_s=5))
    assert res.outcome == OUTCOME_INVALID and res.spec is None
    assert "levels.lead" in res.detail


def test_anchor_violation_is_invalid():
    p = FakeProvider("a", ProviderReply(tool_args=_spec_dict(bpm=132)))
    res = _run(reason_track(INP, [p], deadline_s=5))
    assert res.outcome == OUTCOME_INVALID and "bpm" in res.detail


def test_text_without_spec_is_invalid():
    p = FakeProvider("a", ProviderReply(text="Включаю!"))
    res = _run(reason_track(INP, [p], deadline_s=5))
    assert res.outcome == OUTCOME_INVALID and res.spec is None


def test_provider_exception_does_not_escape_and_falls_through():
    boom = FakeProvider("a", exc=RuntimeError("402 insufficient balance"))
    good = FakeProvider("b", ProviderReply(tool_args=_spec_dict()))
    res = _run(reason_track(INP, [boom, good], deadline_s=5))
    assert res.outcome == OUTCOME_OK and res.provider == "b"


def test_only_exception_gives_error():
    boom = FakeProvider("a", exc=RuntimeError("402 insufficient balance"))
    res = _run(reason_track(INP, [boom], deadline_s=5))
    assert res.outcome == OUTCOME_ERROR and res.spec is None
    assert "402" in res.detail


def test_single_deadline_for_whole_chain():
    slow = FakeProvider("slow", ProviderReply(tool_args=_spec_dict()), delay=5)
    never = FakeProvider("never", ProviderReply(tool_args=_spec_dict()))
    res = _run(reason_track(INP, [slow, never], deadline_s=0.2))
    assert res.outcome == OUTCOME_LATE and res.spec is None and res.provider == "slow"
    assert never.calls == []  # дедлайн общий: после опоздания цепочка не продолжается
    assert res.latency_s < 2


def test_deadline_exhausted_before_next_provider():
    ticks = iter([0.0, 0.0, 1.0, 11.0, 11.0])
    boom = FakeProvider("a", exc=RuntimeError("x"))
    never = FakeProvider("b", ProviderReply(tool_args=_spec_dict()))
    res = _run(reason_track(INP, [boom, never], deadline_s=10, clock=lambda: next(ticks)))
    assert res.outcome == OUTCOME_LATE and never.calls == []


def test_empty_chain():
    res = _run(reason_track(INP, [], deadline_s=5))
    assert res.outcome == OUTCOME_NO_PROVIDER and res.spec is None


# --- фоновый джоб ---------------------------------------------------------


def test_job_runs_in_background_thread_and_calls_on_done_once():
    got = []
    main = threading.get_ident()
    p = FakeProvider("a", ProviderReply(tool_args=_spec_dict()), delay=0.05)
    job = ReasonerJob(INP, [p], deadline_s=5, on_done=lambda r: got.append((r, threading.get_ident())))
    assert job.result is None
    job.start()
    res = job.join(timeout=5)
    assert res is not None and res.outcome == OUTCOME_OK
    assert len(got) == 1 and got[0][0] is res and got[0][1] != main


def test_job_swallows_on_done_exception():
    p = FakeProvider("a", ProviderReply(tool_args=_spec_dict()))

    def bad_callback(_result):
        raise RuntimeError("callback boom")

    job = ReasonerJob(INP, [p], deadline_s=5, on_done=bad_callback).start()
    assert job.join(timeout=5).outcome == OUTCOME_OK


def test_job_late_keeps_preview():
    got = []
    p = FakeProvider("slow", ProviderReply(tool_args=_spec_dict()), delay=5)
    job = ReasonerJob(INP, [p], deadline_s=0.1, on_done=got.append).start()
    res = job.join(timeout=5)
    assert res.outcome == OUTCOME_LATE and res.spec is None and got == [res]
