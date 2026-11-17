"""«Посмотри налево» — взгляд лица, а не 1 м вперёд (#3779, ADR-0148): движение открывает только глагол движения."""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog.tools_view import available_tools, groups_for, motion_named, motion_refusal

#: Реплики с глаголом движения: тулы движения в срезе и исполняются.
MOVE = [
    "Робот, повернись налево", "поезжай вперёд на метр", "езжай на кухню", "двигайся назад", "отъедь немного назад",
    "подъедь ко мне", "развернись", "иди вперёд", "поехали на кухню", "приезжай сюда", "отвези меня к двери",
    "вернись на базу", "поедь направо", "подвинься вправо и двигайся дальше", "пойдём в комнату",
    "посмотри налево и поезжай туда",
]
#: Реплики с восприятием или голым направлением: движения нет, группа лица/взгляда есть (где названо «смотри»).
LOOK = [
    "Робот, посмотри налево", "глянь что там справа", "посмотри на меня", "взгляни вправо", "смотри вверх",
    "посмотри направо", "посмотри вперёд", "глянь назад", "посмотри на кухню", "посмотри туда",
    "посмотри на потолок", "повернись глазами налево", "поверни голову направо", "взгляни на меня",
    "посмотри налево, потом направо",
]
NO_VERB = ["налево", "что слева", "вперёд", "расскажи анекдот", "покажи кухню", "сколько у тебя заряда"]


def _snap():
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0)


def _names(text):
    return {t.name for t in available_tools(_snap(), text=text)}


@pytest.mark.parametrize("text", MOVE)
def test_motion_verb_opens_motion_tools(text):
    assert motion_named(text)
    assert {"move_direction", "navigate_to_waypoint"} <= _names(text)
    assert motion_refusal("move_direction", text) is None


@pytest.mark.parametrize("text", LOOK + NO_VERB)
def test_look_does_not_open_motion_tools(text):
    assert not motion_named(text)
    assert not set(kn.MOTION_TOOLS) & _names(text)
    assert motion_refusal("move_direction", text)


@pytest.mark.parametrize("text", LOOK)
def test_look_is_gaze_via_animation(text):
    if "глаз" in text or "голов" in text:
        return
    assert "animation" in groups_for(_snap(), text)
    assert "play_animation" in _names(text)


def test_guard_refuses_with_catalog_phrase_and_only_motion_tools():
    phrase = motion_refusal("navigate_to_waypoint", "посмотри налево")
    assert phrase and "не просил ехать" in phrase
    assert motion_refusal("play_animation", "посмотри налево") is None
    assert motion_refusal("stop_navigation", "посмотри налево") is None


def test_empty_utterance_is_not_judged():
    assert motion_refusal("move_direction", "") is None
