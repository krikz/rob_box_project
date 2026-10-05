"""Фоновый ризонер плана сета (ADR-0149 PR-10, эпик #3312): дедлайн, breaker, выключатель, план со след. трека.

Провайдер — фейк с тем же ``complete(messages, tools=, settings=)``, что у MiniMax голосового стека; сет —
настоящие ``DjSetTool``/``SetSession``/``PlayerOwner`` на симуляторе клока из ``test_engine_session``.
"""

import asyncio
from types import SimpleNamespace

import pytest

from rob_box_llm.provider import LLMResponse, ToolCall
from rob_box_mcp_tools.engine.reasoner import SetPlanBox, SetReasoner, payload_of
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool
from rob_box_music import reasoner as rz
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

from .test_engine_session import Log, _rig, _started

pytestmark = pytest.mark.unit

GOOD = {"theme_row": "cyber", "mode": "phrygian", "hooks": ["axelf_3"], "energy": [2, 5, 5]}
PLAN = seeded_plan(seeded_profile("ночной город"), 5, set_id="s5")


class Provider:
    """Фейк провайдера; ``calls`` — сколько раз ходили в «облако»."""

    def __init__(self, response=None, *, delay=0.0, error=None):
        self.response, self.delay, self.error, self.calls, self.closed = response, delay, error, [], 0

    async def complete(self, messages, *, tools=(), settings=None):
        self.calls.append((list(messages), list(tools), settings))
        if self.delay:
            await asyncio.sleep(self.delay)
        if self.error:
            raise self.error
        return self.response

    async def aclose(self):
        self.closed += 1


def _tool_reply(args):
    return LLMResponse(tool_calls=(ToolCall("c1", rz.SUBMIT_TOOL, args),), finish_reason="tool_calls")


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _reasoner(provider, **kw):
    built = []

    def factory():
        built.append(1)
        if isinstance(provider, Exception):
            raise provider
        return provider

    log = Log()
    r = SetReasoner(factory, logger=log, spawn=lambda fn: fn(), **kw)
    return r, built, log


def _outcomes(log):
    return [m.split("plan_outcome=")[1].split()[0] for _lvl, m in log.lines if "plan_outcome=" in m]


def test_valid_tool_call_is_applied_and_logged_with_latency():
    provider = Provider(_tool_reply(GOOD))
    r, _built, log = _reasoner(provider)
    got = []
    assert r.request("s5", "ночной город", PLAN, got.append) == "started"
    assert got == [rz.validate(GOOD)]
    assert _outcomes(log) == ["ok"] and "provider=minimax" in log.lines[-1][1] and "p95_ms=" in log.lines[-1][1]
    (messages, tools, settings), = provider.calls
    assert tools == [rz.tool("club", False, PLAN.profile)] and settings.tool_choice == "auto"
    assert "«ночной город»" in messages[1].content
    assert r.metrics.latency("minimax").count == 1


def test_plain_json_answer_without_tool_call_is_accepted_whole():
    import json
    r, _b, log = _reasoner(Provider(LLMResponse(content=json.dumps(GOOD))))
    got = []
    r.request("s5", "т", PLAN, got.append)
    assert got and _outcomes(log) == ["ok"]


@pytest.mark.parametrize("response,path", [
    (_tool_reply({**GOOD, "bpm": 150}), "bpm"),  # LLM не меняет темп: поля нет в схеме
    (_tool_reply({**GOOD, "hooks": ["nope"]}), "hooks"),
    (LLMResponse(content="Вот профиль: ```json {}```"), "$"),  # кусок текста не вырезаем (ADR-0148)
    (LLMResponse(tool_calls=(ToolCall("c", rz.SUBMIT_TOOL, {}),), truncated_tool_args=True), "$"),
])
def test_invalid_answer_means_seeded_and_no_retry(response, path):
    provider = Provider(response)
    r, _b, log = _reasoner(provider)
    got = []
    r.request("s5", "т", PLAN, got.append)
    assert got == [] and _outcomes(log) == ["invalid"] and f"plan_invalid{{{path}}}" in log.lines[-1][1]
    assert len(provider.calls) == 1  # без синтетического ретрая


def test_deadline_gives_late_and_the_plan_stays_seeded():
    r, _b, log = _reasoner(Provider(_tool_reply(GOOD), delay=1.0), deadline_s=0.05)
    got = []
    r.request("s5", "т", PLAN, got.append)
    assert got == [] and _outcomes(log) == ["late"]


def test_provider_exceptions_never_escape():
    r, _b, log = _reasoner(RuntimeError("minimax: missing API key"))
    assert r.request("s5", "т", PLAN, lambda ref: None) == "started"
    r2, _b2, log2 = _reasoner(Provider(error=ConnectionError("402 insufficient balance")))
    r2.request("s5", "т", PLAN, lambda ref: None)
    assert _outcomes(log) == ["error"] and _outcomes(log2) == ["error"] and "missing API key" in log.lines[-1][1]


def test_breaker_opens_after_three_failures_and_skips_the_provider_for_ten_minutes():
    clock = Clock()
    provider = Provider(error=ConnectionError("down"))
    r, built, log = _reasoner(provider, clock=clock)
    for _ in range(3):
        r.request("s", "т", PLAN, lambda ref: None)
    assert r.request("s", "т", PLAN, lambda ref: None) == "circuit_open"
    assert len(provider.calls) == 3 and len(built) == 1  # клиент один; четвёртого похода в облако нет
    clock.t += 601
    provider.error, provider.response = None, _tool_reply(GOOD)
    assert r.request("s", "т", PLAN, lambda ref: None) == "started"  # пробный вызов после паузы
    assert _outcomes(log) == ["error", "error", "error", "circuit_open", "ok"]


def test_disabled_reasoner_makes_zero_calls():
    provider = Provider(_tool_reply(GOOD))
    r, built, log = _reasoner(provider, enabled=False)
    assert r.request("s5", "т", PLAN, lambda ref: None) == "disabled"
    assert built == [] and provider.calls == [] and _outcomes(log) == ["disabled"]


def test_payload_of_takes_only_the_submit_call():
    other = LLMResponse(tool_calls=(ToolCall("x", "speak_text", {"text": "hi"}),), content="")
    with pytest.raises(rz.PlanInvalid):
        payload_of(other)


def _dj(rig, reasoner, speak=None):
    node = SimpleNamespace(get_logger=lambda: rig.log)  # логи сессии и ризонера — в лог рига
    return DjSetTool(node, rig.owner, melodies=lambda ids: {}, finder=lambda theme: (), seed=lambda: 123456,
                     reasoner=reasoner, speak=speak)


def _queued(rig):
    return [e for e in rig.events if e["event"] == "queued"]


def _wait(cond, timeout=5.0):
    """``DjSetTool`` компонует N+1 в фоновом потоке сессии — ждём событие, а не спим наугад."""
    import time
    end = time.monotonic() + timeout
    while not cond():
        assert time.monotonic() < end, "не дождались"
        time.sleep(0.005)


def _notes(rig):
    return [m for _lvl, m in rig.log.lines if "[set v2]" in m and "started track_id=" in m]


def test_plan_from_llm_replaces_the_queued_next_track_and_keeps_one_tempo():
    rig = _rig()
    deferred = []
    reasoner = SetReasoner(lambda: Provider(_tool_reply(GOOD)), logger=rig.log, spawn=deferred.append)
    tool = _dj(rig, reasoner)
    assert tool.execute(action="start", theme="ночной город").data["reasoner"] == "started"
    rig.clock.run_until(rig.clock.beat + 2)  # трек 1 встал, трек 2 по seeded-плану уже в очереди
    _wait(lambda: len(_queued(rig)) == 1)
    deferred[0]()  # LLM ответила, пока играет трек 1
    _wait(lambda: len(_queued(rig)) == 2)  # N+1 перекомпонован по новому плану
    requeued = _queued(rig)
    first = _started(rig)[0]
    rig.clock.run_until(first["start_beat"] + first["form_beats"] + 1)
    _wait(lambda: len(_queued(rig)) == 3)
    rig.clock.run_until(first["start_beat"] + 2 * first["form_beats"] + 1)
    started = _started(rig)
    assert started[1]["track_id"] == requeued[-1]["track_id"] != requeued[0]["track_id"]
    assert {e["bpm"] for e in started} == {first["bpm"]}  # один темп на сет
    assert [c for c in rig.clock.calls if c[0] == "tempo"] == [("tempo", first["bpm"])]
    notes = _notes(rig)
    # мелодий нет — хук не из темы: A11 не растёт от того, что план пришёл от LLM (#3399)
    assert "source=motif plan=seeded A11=0/1" in notes[0] and "source=motif plan=llm A11=0/2" in notes[1]
    assert "plan=llm A11=0/3" in notes[2]


def test_late_llm_leaves_the_whole_set_seeded():
    rig = _rig()
    reasoner = SetReasoner(lambda: Provider(_tool_reply(GOOD), delay=1.0), deadline_s=0.01, logger=rig.log,
                           spawn=lambda fn: fn())
    _dj(rig, reasoner).execute(action="start", theme="ночной город")
    rig.clock.run_until(rig.clock.beat + 2)
    first = _started(rig)[0]
    _wait(lambda: len(_queued(rig)) == 1)
    rig.clock.run_until(first["start_beat"] + first["form_beats"] + 1)
    assert all("plan=seeded" in n for n in _notes(rig)) and len(_notes(rig)) == 2
    assert any("plan_outcome=late" in m for _l, m in rig.log.lines)


def test_hype_line_is_spoken_once_only_when_enabled():
    said = []
    plan = seeded_plan(seeded_profile("т"), 1, set_id="h")
    box = SetPlanBox(plan, lambda ids: {}, speak=said.append, logger=Log())
    box.apply(rz.validate({**GOOD, "hype_line": "Погнали!"}, hype=True))
    t2, t3 = SimpleNamespace(track_id="h:02"), SimpleNamespace(track_id="h:03")
    box.compose_mark(t2), box.compose_mark(t3)
    assert "hype=on" in box.on_started("h:02") and "hype" not in box.on_started("h:03")
    import time
    for _ in range(50):
        if said:
            break
        time.sleep(0.01)
    assert said == ["Погнали!"]
    quiet = []
    box2 = SetPlanBox(plan, lambda ids: {}, speak=quiet.append, logger=Log())
    box2.apply(rz.validate(GOOD))  # выкрик выключен — в ответе его нет
    box2.compose_mark(t2)
    assert "hype" not in box2.on_started("h:02") and quiet == []


def test_dj_set_without_reasoner_is_disabled_by_default():
    rig = _rig()
    assert _dj(rig, None).execute(action="start", theme="космос").data["reasoner"] == "disabled"
