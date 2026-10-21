"""Срез тулов хода в v1 (ADR-0150 PR-6 #3696): нода задаёт ``AgentCore.set_visible_tools`` до вызова LLM."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_dialog import knowledge as kn
from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.core.tools_view_host import apply_tools_view


def _node(playing=False, dj=False, tools=True):
    logger = MagicMock()
    return SimpleNamespace(
        _core=MagicMock(), get_logger=lambda: logger,
        _dsm=SimpleNamespace(current_state=DialogueStateKind.DIALOGUE),
        _music_player_state=MusicPlayerState(state="playing" if playing else "idle", dj=dj, dj_info={}),
        _llm=SimpleNamespace(capabilities=SimpleNamespace(tools=tools)),
    )


def _view(node):
    names, label = node._core.set_visible_tools.call_args[0]
    return names, label


def test_sing_turn_sees_generation_and_at_most_twelve():
    node = _node()
    apply_tools_view(node, "[Speaker:unknown] спой песенку про стрижей и ежей")
    names, label = _view(node)
    assert "generate_music" in names and len(names) <= kn.TOOLS_VISIBLE_MAX
    assert label == "music"
    node.get_logger().info.assert_called_once()
    assert f"tools({len(names)})" in node.get_logger().info.call_args[0][0]


def test_story_turn_has_no_voice_switching_tools():
    """#3744: в ходе «сказка» LLM меняла голос насовсем — без голосовой просьбы set_voice в срезе нет."""
    node = _node()
    apply_tools_view(node, "расскажи сказку где каждый персонаж говорит своим тоном")
    names, _ = _view(node)
    assert "set_voice" not in names and "set_tts_provider" not in names


def test_dj_set_scene_brings_set_tools_and_next_turn_carries_groups():
    node = _node(playing=True, dj=True)
    apply_tools_view(node, "расскажи анекдот")
    assert "dj_set" in _view(node)[0]
    plain = _node()
    apply_tools_view(plain, "спой песню про кота")
    apply_tools_view(plain, "я же просил про кота")
    assert "generate_music" in _view(plain)[0]


def test_provider_without_tools_gets_an_empty_view():
    node = _node(tools=False)
    apply_tools_view(node, "спой")
    assert _view(node)[0] == []


def test_no_core_is_a_no_op():
    apply_tools_view(SimpleNamespace(_core=None), "спой")


def test_identity_turn_hides_register_speaker():
    """#3765: голос похож на другого — личность выясняет робот, LLM этого хода register_speaker не видит."""
    node = _node()
    apply_tools_view(node, "привет меня зовут Матвей")
    assert "register_speaker" in _view(node)[0]
    node._turn_hidden_tools = ("register_speaker",)
    apply_tools_view(node, "привет меня зовут Матвей")
    assert "register_speaker" not in _view(node)[0]
