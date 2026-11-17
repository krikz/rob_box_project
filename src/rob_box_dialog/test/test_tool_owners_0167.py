"""Набор тулов задают владельцы из профиля продукта (ADR-0167 §2.4, шаг Ш1в).

Порты владельцев - знание ядра (``owners.TOOL_OWNERS``): у каждого тула командой ровно один владелец. Какие владельцы у
продукта есть - профиль (``owners`` в ``assistant.yaml``, формат 4). Ядро без профиля - без тулов продукта. У робота
(все порты) срез тулов тот же, что до Ш1в; у продукта без движения и камеры их тулов модель не видит ни в одном срезе.
"""

from __future__ import annotations

import dataclasses

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import knowledge as kn
from rob_box_dialog import owners as ow
from rob_box_dialog import tools_view
from rob_box_dialog.agents import _VIEWS, agent_spec, tools_for
from rob_box_dialog.session import MusicStateRef, Session

#: Колонка (ADR-0160 §8 К3): речь, слух, память, время, поиск, музыка, звуки - без движения, карт, камеры и лиц.
STATION = ("speech", "hearing", "memory", "clock", "web", "music", "sound")


def _legacy_tools_for(agent: str):
    """Срез агента до Ш1в: вид каталога ∩ классы действий, доступные агенту (без владельцев)."""
    spec = agent_spec(agent)
    return tuple(
        t.name
        for t in _VIEWS[spec.tools_view]()
        if t.name in kn.TOOL_CLASSES and agent in kn.ACTION_CLASSES[kn.TOOL_CLASSES[t.name]].agents
    )


def _snap(agent: str, owners, playing: bool = False):
    snap = Session(agent, clock=lambda: 0.0, owners=owners).snapshot
    return dataclasses.replace(snap, music=MusicStateRef(state="playing")) if playing else snap


def test_every_tool_with_a_class_has_exactly_one_owner():
    owned = [t for tools in ow.TOOL_OWNERS.values() for t in tools]
    assert len(owned) == len(set(owned)), "тул у двух владельцев"
    assert set(owned) == set(kn.TOOL_CLASSES), "тул командой без владельца или владелец с тулом вне классов"
    assert set(owned) <= {e.name for e in TOOL_CATALOG}
    for tool in kn.SPEECH_CHANNEL_TOOLS:
        assert ow.owner_of(tool) is None, "транспорт речи - не тул владельца"


def test_core_without_profile_has_no_product_tools(neutral_core):
    assert ow.OWNERS_WITHOUT_PROFILE == ()
    for agent in kn.AGENTS:
        assert tools_for(agent_spec(agent)) == ()
        snap = Session(agent, clock=lambda: 0.0).snapshot
        assert snap.owners is None
        for text in ("поезжай на кухню", "включи музыку", "сколько заряда", "который час", ""):
            names = {e.name for e in tools_view.available_tools(snap, None, agent, text)}
            assert names <= set(kn.SPEECH_CHANNEL_TOOLS), (agent, text, names)


@pytest.mark.parametrize("agent", kn.AGENTS)
def test_robot_with_every_owner_sees_the_same_tools(agent):
    assert tools_for(agent_spec(agent), ow.OWNERS) == _legacy_tools_for(agent)


@pytest.mark.parametrize("agent", kn.AGENTS)
def test_station_without_motion_and_camera_never_sees_their_tools(agent):
    missing = {t for o in set(ow.OWNERS) - set(STATION) for t in ow.TOOL_OWNERS[o]}
    assert not set(tools_for(agent_spec(agent), STATION)) & missing
    for playing in (False, True):
        snap = _snap(agent, STATION, playing)
        for text in (
            "поезжай на кухню",
            "кто перед тобой",
            "сколько заряда",
            "покажи лица fff371fe",
            "запомни это место как кухня",
            "какие карты знаешь",
            "включи музыку",
        ):
            names = {e.name for e in tools_view.available_tools(snap, None, agent, text)}
            assert not names & missing, (agent, text, names & missing)
    music = tools_view.available_tools(_snap(agent, STATION), None, agent, "включи музыку")
    assert "request_music" in {e.name for e in music}


def test_widen_does_not_open_a_tool_of_an_absent_owner():
    assert tools_view.widen_for(["stop_music"], "personality", [], ow.OWNERS)
    assert tools_view.widen_for(["stop_music"], "personality", [], ("speech", "memory")) == ()


def test_unknown_owner_is_an_error_not_an_empty_slice():
    with pytest.raises(ValueError, match="незнакомые владельцы"):
        tools_for(agent_spec("personality"), ("speech", "camera"))
