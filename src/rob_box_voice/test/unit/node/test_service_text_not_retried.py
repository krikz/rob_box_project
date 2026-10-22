"""Служебный текст в ответе хода (разметка вызова тула #2760, ``<system>``-шаблон #2175) не звучит и не зовёт LLM.

ADR-0150 PR-7 ч.7d-2 (#3698): ретраи в dialogue_node удалены. Такой текст не уходит в речь (``build_ssml_payload``
снял бы теги, и робот прочёл бы значения параметров — отказ tts_node их уже не узнал бы): ход без тулов получает
честную фразу кода :data:`EMPTY_REPLY_PHRASE`, второго вызова LLM нет.
"""

from __future__ import annotations

import json
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.agent_core import DialogResult
from rob_box_voice.core.speak_helpers import EMPTY_REPLY_PHRASE

from ._result_node import make_result_node, published_texts

MARKUP = '<invoke name="weather_lookup">\n<parameter name="city">Бишкек</parameter>\n</invoke>'
SYSTEM = "<system>\n[получатель ответа забыл указать антропоморфные атрибуты]\n</system>"


@pytest.mark.parametrize("spoken, leaked", [(MARKUP, "Бишкек"), (SYSTEM, "получатель")])
def test_service_text_is_not_spoken_and_not_retried(spoken, leaked):
    n = make_result_node()
    n._dispatch_turn = MagicMock()

    n._handle_result(DialogResult(spoken_text=spoken, tools_called=[], finish_reason="stop"),
                     user_input="какая погода в Бишкеке")

    n._dispatch_turn.assert_not_called()
    said = [json.loads(text)["ssml"] for text in published_texts(n)]
    assert said == [f"<speak>{EMPTY_REPLY_PHRASE}</speak>"], said
    assert leaked not in " ".join(said)
