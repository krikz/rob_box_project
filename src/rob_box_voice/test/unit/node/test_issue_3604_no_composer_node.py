"""#3604: композитор назван, а его пьес нет - исход «нет композитора» несёт фразу кода дословно.

PR-15 (#3711, часть 15p): сквозная проверка роутера в ноде диалога v1 («ход LLM не стартует») удалена вместе с ней.
"""

from __future__ import annotations

import asyncio

from rob_box_voice.core.media_phrases import (composer_missing_error, composer_missing_phrase,
                                              composer_missing_text)
from rob_box_voice.core.named_play import NamedPlayStatus, outcome_text, run_named_play

_PHRASE = composer_missing_text("к элизе")
_HIT = {"name": "furelise", "title": "Fur Elise", "display_title": "Fur Elise",
        "match": {"matched": ["fur", "elise"], "unmatched": [], "coverage": 1.0, "ignored": []}}


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
