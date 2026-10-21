"""Допуск действий по плану (ADR-0150 §5.6.4, PR-7b #3699): идти сразу, ждать конца занятого или честный отказ."""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.plan.runner import PlanRunner


def _runner():
    events, lines = [], []
    return PlanRunner(publish=events.append, log=lines.append), events, lines


def test_free_resources_go_at_once():
    runner, events, _ = _runner()
    admission = asyncio.run(runner.admit("request_music", "b1"))
    assert (admission.status, admission.event, events) == ("go", None, [])


def test_owner_queue_is_not_awaited_by_the_plan():
    """Реплика за репликой — очередь tts_node, звук за звуком — sound_node: запрос уходит владельцу сразу."""
    runner, events, _ = _runner()
    runner.state.on_speech("started", "s1", text_len=200)
    runner.state.on_sound("started", "r1")
    assert asyncio.run(runner.admit("speak_text", "b1")).status == "go"
    assert asyncio.run(runner.admit("play_sound", "b2")).status == "go"
    assert events == []


def test_later_waits_for_the_speech_end_then_goes():
    """(б) «расскажи анекдот, а потом включи что-нибудь»: музыка — после ``SpeechEvent finished`` анекдота."""
    runner, events, lines = _runner()
    runner.begin_turn("Робот, расскажи анекдот, а потом включи что-нибудь")
    runner.state.on_speech("started", "joke", text_len=28)

    async def scenario():
        admit = asyncio.ensure_future(runner.admit("request_music", "m1", turn_id="t"))
        await asyncio.sleep(0.05)
        assert not admit.done()  # ждёт конца речи
        runner.state.on_speech("finished", "joke")
        runner.changed()
        return await asyncio.wait_for(admit, 2.0)

    admission = asyncio.run(scenario())
    assert admission.status == "go" and admission.waited_s > 0
    (queued,) = events
    assert (queued.kind, queued.after_resource, queued.after_block, queued.turn_id) == ("queued", "speech", "joke", "t")
    assert any("PlanEvent queued" in ln for ln in lines) and any("дождался speech" in ln for ln in lines)


def test_wait_ceiling_gives_an_honest_timeout(monkeypatch):
    """Занятое не освободилось к потолку — действие не исполняется, исход ``timeout`` (а не тихий старт поверх)."""
    monkeypatch.setattr(kn, "PLAN_WAIT_MAX_S", 0.1)
    runner, _, lines = _runner()
    runner.begin_turn("робот, а потом включи музыку")
    runner.state.on_music(True, "trk")  # трек без конца — ETA неизвестна, потолок — максимум
    admission = asyncio.run(runner.admit("request_music", "m1"))
    assert admission.status == "timeout" and admission.event.after_resource == "track"
    assert any("не освободился" in ln for ln in lines)


def test_ceiling_formula():
    assert kn.plan_wait_ceiling_s(None) == kn.PLAN_WAIT_MAX_S
    assert kn.plan_wait_ceiling_s(5.0) == 5.0 + kn.PLAN_WAIT_SLACK_S
    assert kn.plan_wait_ceiling_s(10_000.0) == kn.PLAN_WAIT_MAX_S


def test_parallel_by_default_x11_and_halt_does_not_wait():
    runner, events, _ = _runner()
    runner.state.on_speech("started", "song", text_len=300)
    runner.begin_turn("Робот, направо")
    assert asyncio.run(runner.admit("move_direction", "g1")).status == "go"
    runner.begin_turn("Робот, стой")
    assert asyncio.run(runner.admit("stop_navigation", "g2")).status == "go"
    assert events == []


def test_new_turn_forgets_the_hint():
    runner, _, _ = _runner()
    runner.begin_turn("робот, потом включи музыку")
    assert runner.hint == "later"
    runner.begin_turn("")
    assert runner.hint is None


def test_publish_failure_does_not_break_the_action(caplog):
    def broken(_event):
        raise RuntimeError("no ros")

    runner = PlanRunner(publish=broken, log=lambda _: None)
    runner.begin_turn("робот, а потом включи музыку")
    runner.state.on_sound("started", "r1")

    async def scenario():
        admit = asyncio.ensure_future(runner.admit("request_music", "m1"))
        await asyncio.sleep(0.02)
        runner.state.on_sound("finished", "r1")
        runner.changed()
        return await asyncio.wait_for(admit, 2.0)

    assert asyncio.run(scenario()).status == "go"
    assert "не опубликован" in caplog.text


@pytest.mark.parametrize("tool", ["play_animation", "memory_save", "get_current_time"])
def test_resource_free_kinds_never_wait(tool):
    runner, _, _ = _runner()
    runner.begin_turn("робот, а потом сделай")
    runner.state.on_speech("started", "s1", text_len=100)
    assert asyncio.run(runner.admit(tool, "b")).status == "go"


def test_wait_is_extended_while_the_owner_still_reports_busy(monkeypatch):
    """Живой робот 21.10: ETA до синтеза короче речи — срок продлевается по свежей ETA, а не рвётся на первой оценке."""
    monkeypatch.setattr(kn, "PLAN_WAIT_MAX_S", 3.0)
    monkeypatch.setattr(kn, "PLAN_WAIT_SLACK_S", 0.2)
    runner, _, _ = _runner()
    runner.begin_turn("робот, а потом включи музыку")
    runner.state.on_speech("started", "s1", text_len=1)  # оценка ~0,1 с, потолок по ней ~0,3 с

    async def scenario():
        admit = asyncio.ensure_future(runner.admit("request_music", "m1"))
        for spoken in (0, 0, 0):  # владелец ещё говорит: progress каждые 0,25 с
            await asyncio.sleep(0.25)
            runner.state.on_speech("progress", "s1", text_len=1, spoken_chars=spoken, seconds=0.5)
            runner.changed()
        runner.state.on_speech("finished", "s1")
        runner.changed()
        return await asyncio.wait_for(admit, 3.0)

    admission = asyncio.run(scenario())
    assert admission.status == "go" and admission.waited_s > 0.5
