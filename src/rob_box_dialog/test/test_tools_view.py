"""Срез тулов хода (ADR-0150 §4.2, #3696): ≤ 12 схем на всех кадрах, голос — только явной просьбой (#3744)."""

import itertools
from types import SimpleNamespace

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG, llm_visible_tools
from rob_box_dialog import knowledge as kn
from rob_box_dialog.session import MusicStateRef, SceneFrame, SessionSnapshot
from rob_box_dialog.tools_view import available_tools, groups_for


def _snap(**kw):
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, **kw)


SNAPSHOTS = {
    "idle": _snap(),
    "music": _snap(music=MusicStateRef("playing", title="Still Dre")),
    "dj": _snap(music=MusicStateRef("playing"), scenes=(SceneFrame("DJ", "code"),)),
    "speaking": _snap(speaking=True),
}
#: Реплики, задевающие несколько групп сразу — худший случай размера среза.
PHRASES = [
    "", "привет", "расскажи анекдот", "спой песенку про стрижей и ежей", "включи что-нибудь из библиотеки",
    "включи голос надёжного мужика", "смени голос и спой песню", "поезжай на кухню и включи музыку",
    "сколько у тебя заряда", "построй карту комнаты и запомни меня", "сделай музыку громче голосом яндекса",
    "удали точку кухня и поезжай назад", "какое расписание мероприятия", "покажи анимацию и включи звук",
    "отмени задачу и включи трек погромче голос быстрее",
    " ".join(stem for g in kn.TOOL_GROUPS.values() for stem in g.stems),  # все группы разом
]


def _names(tools):
    return [t.name for t in tools]


@pytest.mark.parametrize("frame, text", list(itertools.product(SNAPSHOTS, PHRASES)))
def test_at_most_twelve_schemas_on_every_frame(frame, text):
    tools = available_tools(SNAPSHOTS[frame], text=text)
    assert 0 < len(tools) <= kn.TOOLS_VISIBLE_MAX
    assert len(set(_names(tools))) == len(tools)


@pytest.mark.parametrize("agent", kn.AGENTS)
def test_every_agent_slice_fits(agent):
    worst = PHRASES[-1]
    for snap in SNAPSHOTS.values():
        assert len(available_tools(snap, agent=agent, text=worst)) <= kn.TOOLS_VISIBLE_MAX


def test_always_visible_tools_are_core_and_at_most_six():
    assert len(kn.TOOLS_ALWAYS) <= 6
    assert set(kn.TOOLS_ALWAYS) <= set(_names(available_tools(SNAPSHOTS["idle"], text="расскажи анекдот")))


def test_groups_are_small_and_point_into_the_catalog():
    catalog = {t.name for t in TOOL_CATALOG}
    for name, group in kn.TOOL_GROUPS.items():
        assert 0 < len(group.tools) <= 6, name
        assert set(group.tools) <= catalog, name
    assert set(kn.TOOLS_ALWAYS) | set(kn.TOOLS_CODE_ONLY) <= catalog
    assert set(kn.TOOL_GROUPS_DEFAULT) <= set(kn.TOOL_GROUPS)


def test_every_llm_visible_tool_is_reachable_or_code_only():
    """Никакой тул не пропадает молча: либо всегда виден, либо в группе, либо явно «только кодом»."""
    reachable = set(kn.TOOLS_ALWAYS) | {t for g in kn.TOOL_GROUPS.values() for t in g.tools}
    missing = {t.name for t in llm_visible_tools()} - reachable - set(kn.TOOLS_CODE_ONLY)
    assert missing == set()


def test_sing_request_sees_generation_not_only_the_library():
    names = _names(available_tools(SNAPSHOTS["idle"], text="Робот, спой песенку про стрижей и ежей"))
    assert {"generate_music", "music_library", "request_music"} <= set(names)


@pytest.mark.parametrize("text, tool", [
    ("робот сделай так чтобы было повеселее как на вечеринке", "dj_set"),  # voice_bench router-miss-music
    ("робот отвези меня туда где мы вчера были", "navigate_to_waypoint"),  # voice_bench router-miss-nav
])
def test_voice_bench_router_misses_still_see_their_tools(text, tool):
    assert tool in _names(available_tools(SNAPSHOTS["idle"], text=text))


def test_no_music_tools_without_music_words_or_music_scene():
    names = set(_names(available_tools(SNAPSHOTS["idle"], text="расскажи анекдот")))
    assert not names & set(kn.TOOL_GROUPS["music"].tools)
    assert {"get_robot_status", "get_battery_level"} <= names  # группа по умолчанию


@pytest.mark.parametrize("frame", ["music", "dj"])
def test_music_scene_brings_music_tools(frame):
    names = set(_names(available_tools(SNAPSHOTS[frame], text="расскажи анекдот")))
    assert {"stop_music", "get_music_state", "dj_set", "request_music"} <= names


@pytest.mark.parametrize("text", ["расскажи сказку где каждый говорит своим тоном", "спой песню", "привет",
                                  "расскажи страшную историю про полицию"])
def test_voice_tools_only_on_explicit_voice_request(text):
    """#3744: в ходе «сказка» LLM сама вызвала set_voice — голос персистентный; без голосовой просьбы схем нет."""
    names = set(_names(available_tools(SNAPSHOTS["idle"], text=text)))
    assert not names & {"set_voice", "set_tts_provider"}, text


@pytest.mark.parametrize("text", ["смени голос на женский", "говори голосом алёны", "переключись на яндекс"])
def test_voice_tools_on_voice_request(text):
    assert "set_voice" in _names(available_tools(SNAPSHOTS["idle"], text=text))


def test_silence_and_no_tool_providers_give_nothing():
    silenced = _snap(scenes=(SceneFrame("Silence", "user"),))
    assert available_tools(silenced, text="спой") == ()
    assert available_tools(SNAPSHOTS["idle"], SimpleNamespace(tools=False), text="спой") == ()
    assert available_tools(SNAPSHOTS["idle"], SimpleNamespace(tools=True), text="спой")


def test_say_is_not_for_the_personality():
    operator = _names(available_tools(SNAPSHOTS["idle"], agent="operator", text="скажи вслух"))
    personality = _names(available_tools(SNAPSHOTS["idle"], text="скажи вслух"))
    assert "say" not in personality and "speak_text" in personality and operator


def test_group_order_is_priority_order():
    assert groups_for(SNAPSHOTS["idle"], "поезжай и спой") == ("music", "navigation")
    assert groups_for(SNAPSHOTS["idle"], "") == kn.TOOL_GROUPS_DEFAULT


def test_generic_verb_yields_to_a_named_group():
    """«включи голос надёжного мужика»: «включи» — общий глагол, голос назван прямо — set_voice в срезе."""
    names = _names(available_tools(SNAPSHOTS["idle"], text="Робот, включи голос надежного мужика"))
    assert "set_voice" in names
    assert groups_for(SNAPSHOTS["idle"], "включи голос надежного мужика") == ("voice", "music")


def test_continuation_keeps_the_previous_groups():
    """«я же просил про стрижей и ежей» сразу после «спой песенку про стрижей» — продолжение, срез прошлого хода."""
    text = "есть страничка робот я же просил про стрижей и ежей"
    assert "generate_music" not in _names(available_tools(SNAPSHOTS["idle"], text=text))
    assert "generate_music" in _names(available_tools(SNAPSHOTS["idle"], text=text, carry=("music",)))
    assert groups_for(SNAPSHOTS["idle"], "который час", carry=("music",)) == ("music",)
    assert groups_for(SNAPSHOTS["idle"], "поезжай на кухню", carry=("music",)) == ("navigation",)


def test_voice_group_is_never_carried_over():
    """Робот 21.10: после «какие у тебя голоса» ход «расскажи анекдот» видел set_voice — перенос группы прошлого хода.
    Голос — только по явной просьбе в этой же реплике (#3744)."""
    assert groups_for(SNAPSHOTS["idle"], "расскажи анекдот", carry=("voice",)) == kn.TOOL_GROUPS_DEFAULT
    names = _names(available_tools(SNAPSHOTS["idle"], text="расскажи анекдот", carry=("voice", "music")))
    assert "set_voice" not in names and "generate_music" in names
    assert set(kn.TOOL_GROUPS_EXPLICIT) <= set(kn.TOOL_GROUPS)


#: Реплики акта 1 марафона, прозвучавшие под играющий диджей-сет (run 39849541397, #3857): срез был
#: ``command, memory_save, memory_search, get_current_time, search_web, play_animation, request_music,
#: generate_music, music_library, dj_set, stop_music, get_music_state`` — тула, о котором спросили, в нём не было.
@pytest.mark.parametrize("frame", ["music", "dj"])
@pytest.mark.parametrize("text, tool", [
    ("робот сколько у тебя осталось заряда до утра дотянешь", "get_battery_level"),
    ("робот доложи статус все ли у тебя работает ничего не отвалилось", "get_robot_status"),
    ("робот что ты сейчас видишь вокруг себя опиши в двух словах", "get_perception_context"),
    ("робот какие звуки ты вообще умеешь издавать", "get_sound_info"),
    ("робот где ты", "get_current_pose"),
])
def test_named_group_is_not_crowded_out_by_the_music_scene(frame, text, tool):
    names = _names(available_tools(SNAPSHOTS[frame], text=text))
    assert tool in names
    assert "stop_music" in names  # музыка играет — остановить её можно и в этом ходе


def test_music_scene_still_fills_a_neutral_turn():
    assert groups_for(SNAPSHOTS["dj"], "расскажи анекдот") == ("music",)
    assert groups_for(SNAPSHOTS["music"], "сколько у тебя заряда") == ("status", "music")
