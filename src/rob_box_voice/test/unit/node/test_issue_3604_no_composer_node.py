"""#3604: композитор назван, а его пьес нет - роутер говорит фразу кода дословно, ход LLM не стартует."""

from __future__ import annotations

import asyncio

from rob_box_llm.provider import ToolResult

from rob_box_voice.core.media_phrases import (composer_missing_error, composer_missing_phrase,
                                              composer_missing_text)
from rob_box_voice.core.named_play import NamedPlayStatus, outcome_text, run_named_play

from .test_issue_3176_play_named_node import _MelodyExecutor, _make_node, _stt, run_plans  # noqa: F401

_PHRASE = composer_missing_text("к элизе")
_HIT = {"name": "furelise", "title": "Fur Elise", "display_title": "Fur Elise",
        "match": {"matched": ["fur", "elise"], "unmatched": [], "coverage": 1.0, "ignored": []}}


class _NoComposerExecutor(_MelodyExecutor):
    async def execute(self, call):
        if call.name == "request_music":
            self.calls.append((call.name, dict(call.arguments)))
            return ToolResult(tool_call_id=call.id, content=composer_missing_error(_PHRASE), is_error=True)
        return await super().execute(call)


def test_router_says_the_code_phrase_verbatim_and_never_calls_the_llm(run_plans):  # noqa: F811
    n = _make_node()
    n._tool_executor = _NoComposerExecutor()
    _stt(n, "Робот, поставь к элизе")
    run_plans()
    n._dispatch_turn.assert_not_called()
    n._speak_direct.assert_called_once_with(_PHRASE)
    assert n._llm_skipped_counter["media_command"] == 1


def test_flow_returns_no_composer_status_with_the_phrase():
    async def tools(name, args):
        if name == "lookup_melody":
            return True, repr(_HIT)
        return False, composer_missing_error(_PHRASE)

    out = asyncio.new_event_loop().run_until_complete(run_named_play(tools, "к элизе"))
    assert out.status is NamedPlayStatus.NO_COMPOSER and outcome_text(out, "К элизе") == _PHRASE


def test_phrase_codec_roundtrip_and_other_errors_are_not_it():
    assert composer_missing_phrase(composer_missing_error(_PHRASE)) == _PHRASE
    assert composer_missing_phrase("мелодия не заиграла: not_found") == ""
