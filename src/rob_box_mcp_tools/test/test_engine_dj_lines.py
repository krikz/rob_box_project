"""Реплика диджея на каждом переходе (ADR-0149 §12 В2, решение Шифу 06.10; I23): факты из плана, LLM — раскраска,
отказ LLM — шаблон, одна фраза на трек, музыка не останавливается."""

from types import SimpleNamespace

import pytest

from rob_box_llm.provider import LLMResponse, ToolCall
from rob_box_mcp_tools.engine.dj_lines import TransitionLines, latin_fold
from rob_box_mcp_tools.engine.reasoner import SetPlanBox, SetReasoner
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool
from rob_box_music import dj_line as dl
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

from .test_engine_reasoner import Provider, _wait
from .test_engine_session import Log, _rig, _started

pytestmark = pytest.mark.unit

PLAN = seeded_plan(seeded_profile("Марио, Тетрис"), 1, set_id="h")
TITLES = {"tetris_2": "Тетрис", "supermar": "Super Mario Bros"}


def _titles(ids):
    return {i: TITLES[i] for i in ids if i in TITLES}


def _reply(line):
    return LLMResponse(tool_calls=(ToolCall("c1", dl.SUBMIT_TOOL, {"line": line}),), finish_reason="tool_calls")


def _ask(*results):
    """Фейк ``SetReasoner.ask``: исходы по очереди; ``asked`` — что спросили."""
    queue, asked = list(results), []

    def ask(system, user, tool, deadline_s):
        asked.append(user)
        return queue.pop(0)
    return ask, asked


def _lines(ask=None, said=None, log=None):
    said = [] if said is None else said
    return TransitionLines(said.append, titles=_titles, ask=ask, theme_names=lambda t: ["Марио", "Тетрис"],
                           fold=latin_fold, logger=log or Log(), spawn=lambda fn: fn(), grace_s=0.01), said


def _spoken_log(log):
    return [m for _l, m in log.lines if "[dj line]" in m and "text=" in m]


def test_llm_line_is_filled_with_facts_from_plan():
    ask, asked = _ask(("ok", _reply("Трек {no} из {total}: {hook} на танцполе!"), ""))
    log = Log()
    lines, said = _lines(ask, log=log)
    lines.prepare("h:02:B:x", 2, PLAN, "tetris_2")
    assert lines.on_started("h:02:B:x") == "dj_line=on"
    total = len(PLAN.tracks)
    assert said == [f"Трек 2 из {total}: Тетрис на танцполе!"]
    assert "«Тетрис»" in asked[0]  # факт — название мелодии трека, не тема
    assert "source=llm" in _spoken_log(log)[0] and "hook='Тетрис'" in _spoken_log(log)[0]


@pytest.mark.parametrize("outcome", [
    ("ok", _reply("Играет Марио, ура!"), ""),  # другая мелодия буквами — невалидно
    ("ok", _reply("Super Mario на подходе!"), ""),
    ("late", None, "нет ответа за 20 с"),
    ("error", None, "RuntimeError: нет денег"),
    ("circuit_open", None, "minimax пропускается"),
    ("ok", LLMResponse(content="просто текст"), ""),  # без вызова
])
def test_llm_failure_or_lie_falls_back_to_template_with_same_facts(outcome):
    ask, _ = _ask(outcome)
    log = Log()
    lines, said = _lines(ask, log=log)
    lines.prepare("h:02:B:x", 2, PLAN, "tetris_2")
    lines.on_started("h:02:B:x")
    assert len(said) == 1 and "Тетрис" in said[0] and "Марио" not in said[0] and "Mario" not in said[0]
    assert "source=template" in _spoken_log(log)[0]
    assert any("line_outcome=" in m and "line_outcome=ok" not in m for _l, m in log.lines)


def test_without_llm_every_track_still_gets_a_template_line():
    lines, said = _lines(ask=None)
    for no, hook in ((1, "supermar"), (2, "tetris_2"), (3, None)):
        lines.prepare(f"h:0{no}:A:x", no, PLAN, hook)
        lines.on_started(f"h:0{no}:A:x")
    assert len(said) == 3 and "Super Mario Bros" in said[0] and "Тетрис" in said[1]
    assert len(set(said)) == 3


def test_one_line_per_transition():
    lines, said = _lines()
    lines.prepare("h:02:B:x", 2, PLAN, "tetris_2")
    lines.on_started("h:02:B:x")
    assert lines.on_started("h:02:B:x") == ""  # повторный started того же трека — не вторая реплика
    assert lines.on_started("h:09:B:never-composed") == ""
    assert len(said) == 1


def test_last_track_line_says_final():
    lines, said = _lines()
    last = len(PLAN.tracks)
    lines.prepare("h:last", last, PLAN, "tetris_2")
    lines.on_started("h:last")
    assert ("Финал" in said[0] or "Последний" in said[0]) and "Тетрис" in said[0]


def test_speak_failure_does_not_escape():
    log = Log()
    lines = TransitionLines(lambda _t: (_ for _ in ()).throw(RuntimeError("tts")), titles=_titles,
                            logger=log, spawn=lambda fn: fn(), grace_s=0.01)
    lines.prepare("h:01", 1, PLAN, None)
    lines.on_started("h:01")
    assert any("не озвучена" in m for m in log.warnings())


def test_reasoner_ask_shares_breaker_and_is_off_when_disabled():
    assert SetReasoner(enabled=False).ask("s", "u", dl.tool(), 1.0)[0] == "disabled"
    provider = Provider(_reply("Погнали, {hook}!"))
    r = SetReasoner(lambda: provider, logger=Log())
    outcome, response, _ = r.ask("s", "u", dl.tool(), 1.0)
    assert outcome == "ok" and response.tool_calls[0].name == dl.SUBMIT_TOOL
    assert provider.calls[0][1] == [dl.tool()]
    failing = SetReasoner(lambda: Provider(error=RuntimeError("x")), logger=Log())
    assert [failing.ask("s", "u", dl.tool(), 1.0)[0] for _ in range(4)] == ["error"] * 3 + ["circuit_open"]


def test_plan_box_hands_composed_track_to_lines():
    lines, said = _lines()
    box = SetPlanBox(PLAN, lambda ids: {}, lines=lines, logger=Log())
    track = SimpleNamespace(track_id="h:01:A:x", hook=SimpleNamespace(source="tetris_2"))
    box.compose_mark(track, 1)
    assert "dj_line=on" in box.on_started("h:01:A:x") and "Тетрис" in said[0]


def test_dj_set_speaks_on_every_started_and_never_stops_music():
    rig = _rig()
    said = []
    node = SimpleNamespace(get_logger=lambda: rig.log)
    tool = DjSetTool(node, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 123456,
                     reasoner=SetReasoner(enabled=False), speak=said.append, titles=_titles)
    assert tool.execute(action="start", theme="космос").success
    rig.clock.run_until(rig.clock.beat + 2)
    _wait(lambda: len(said) == 1)
    first = _started(rig)[0]
    for k in (1, 2):
        _wait(lambda: len([e for e in rig.events if e["event"] == "queued"]) == k)
        rig.clock.run_until(first["start_beat"] + k * first["form_beats"] + 1)
    _wait(lambda: len(said) == 3)
    assert len(_started(rig)) == 3 and len(said) == 3  # одна реплика на каждый трек, включая первый
    assert not [e for e in rig.events if e["event"] in ("stopped", "finished", "rejected")]
    assert rig.owner.is_playing()  # реплика не ставит музыку на паузу и не глушит её
    assert "трек 1/" in _spoken_log(rig.log)[0]


def test_dj_set_lines_off_by_parameter():
    rig = _rig()
    said = []
    node = SimpleNamespace(get_logger=lambda: rig.log)
    tool = DjSetTool(node, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 1,
                     speak=said.append, titles=_titles, lines=False)
    tool.execute(action="start", theme="космос")
    rig.clock.run_until(rig.clock.beat + 2)
    assert _started(rig) and said == []
