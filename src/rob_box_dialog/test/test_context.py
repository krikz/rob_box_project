"""Контекст вызова LLM структурой: снимок хода из разделов-данных, токены вызова по блокам (ADR-0150 §7.3, A5)."""

import json

import pytest

from rob_box_core.token_estimate import estimate_message_tokens, estimate_tokens, estimate_tool_tokens
from rob_box_dialog import knowledge as kn
from rob_box_dialog.context import PROMPT_TOKENS_MARK, SNAPSHOT_TAG, ContextBlocks, build_context, measure


def test_sections_render_in_table_order_as_json_and_empty_ones_are_skipped():
    text = build_context({
        "music_state": {"playing": "yes", "track": 'Techno "rave" & <more>'},
        "speech_gender": "male",
        "providers": [],
        "recent_unaddressed": None,
        "position": {"x": 1.5, "y": 0.0, "theta": 0.0, "frame": "odom"},
    })
    lines = text.splitlines()
    assert lines[0] == f"<{SNAPSHOT_TAG}>" and lines[-1] == f"</{SNAPSHOT_TAG}>"
    assert [ln.split(">", 1)[0].strip() for ln in lines[1:-1]] == ["<speech_gender", "<position", "<music_state"]
    music = lines[3].removeprefix("  <music_state>").removesuffix("</music_state>")
    assert json.loads(music) == {"playing": "yes", "track": 'Techno "rave" & <more>'}
    assert '"y":0.0' in lines[2]  # число — данные, а не «пусто»


def test_multiline_string_section_is_a_block():
    text = build_context({"user_profile": "    <name>Саша</name>\n    <speaker_id>spk_1</speaker_id>"})
    assert "  <user_profile>\n    <name>Саша</name>\n    <speaker_id>spk_1</speaker_id>\n  </user_profile>" in text


def test_unknown_section_is_refused_loudly():
    with pytest.raises(ValueError, match="reminder"):
        build_context({"reminder": "ОБЯЗАТЕЛЬНО вызови register_speaker"})


def test_rules_are_not_sections():
    """Снимок — данные: правил для модели (``reminder``, ``*_rule``) в таблице разделов нет."""
    assert not [s for s in kn.CONTEXT_SECTIONS if "rule" in s or "reminder" in s]


def _msg(role, content, tool_calls=None):
    return {"role": role, "content": content, "tool_calls": tool_calls}


def test_measure_splits_a_request_by_structure():
    snapshot = build_context({"speech_gender": "male", "music_state": {"playing": "no", "dj": "off"}})
    current = _msg("user", snapshot + "\n\nкоторый час")
    call = _msg("assistant", "", [{"name": "get_current_time", "arguments": "{}"}])
    tool = {"type": "function", "function": {"name": "get_current_time", "parameters": {}}}
    messages = [_msg("system", "персона"), _msg("system", "факты о Саше"), _msg("user", "привет"),
                _msg("assistant", "Привет!"), current, call, _msg("tool", '{"formatted_time": "двенадцать"}')]

    blocks = measure(messages, [tool], snapshot)

    assert blocks.tokens["system"] == estimate_message_tokens(messages[0])
    assert blocks.tokens["facts"] == estimate_message_tokens(messages[1])
    assert blocks.tokens["window"] == estimate_message_tokens(messages[2]) + estimate_message_tokens(messages[3])
    assert blocks.tokens["snapshot"] == estimate_tokens(snapshot)
    assert blocks.tokens["turn"] == (estimate_message_tokens(current) - estimate_tokens(snapshot)
                                     + estimate_message_tokens(call) + estimate_message_tokens(messages[6]))
    assert blocks.tokens["tools"] == estimate_tool_tokens(tool)
    assert tuple(blocks.tokens) == kn.CONTEXT_BLOCKS and blocks.total == sum(blocks.tokens.values())


def test_measure_without_snapshot_in_the_turn_counts_it_as_turn():
    blocks = measure([_msg("system", "персона"), _msg("user", "привет")], (), snapshot="<system_context/>")
    assert blocks.tokens["snapshot"] == 0 and blocks.tokens["turn"] > 0


def test_over_budget_and_log_line():
    tokens = dict.fromkeys(kn.CONTEXT_BLOCKS, 10)
    tokens.update(system=kn.CONTEXT_BUDGET_TOKENS["system"] + 1, tools=kn.CONTEXT_BUDGET_TOKENS["tools"] + 1,
                  turn=99_999)  # у реплики хода бюджета нет
    blocks = ContextBlocks(tokens)
    assert blocks.over_budget() == ("system", "tools")
    line = blocks.log_line(prompt_tokens=8123, estimated=False, provider="deepseek", view="core+music")
    assert line == (f"{PROMPT_TOKENS_MARK}8123 estimated=False provider=deepseek view=core+music system=4001 facts=10 "
                    "window=10 snapshot=10 turn=99999 tools=4001 over=system,tools")
    assert ContextBlocks(dict.fromkeys(kn.CONTEXT_BLOCKS, 1)).log_line(
        prompt_tokens=6, estimated=True, provider="p", view="all").endswith(" over=-")
