"""Ответ хода без тулов уходит в речь как есть: babble-ретрая (#992 Bug D) и hard-mute планирования (#1882) нет.

ADR-0150 PR-7 ч.7d-1 (#3698): в режиме команд выступление — поле ``perform``, план модели в ``speech`` не кладётся;
повторный вызов LLM по тексту ответа не делается.
"""

from __future__ import annotations

import json
from unittest.mock import MagicMock

from rob_box_harness.core.agent_core import DialogResult

from ._result_node import make_result_node, published_texts


def _spoken(node) -> list[str]:
    return [json.loads(text)["ssml"] for text in published_texts(node)]


def _reply(spoken: str) -> DialogResult:
    return DialogResult(spoken_text=spoken, tools_called=[], finish_reason="stop")


def test_promise_opener_is_spoken_once_without_retry():
    n = make_result_node()
    n._dispatch_turn = MagicMock()

    n._handle_result(_reply("Зачитаю рэп про космос!"), user_input="зачитай рэп про космос")

    assert _spoken(n) == ["<speak>Зачитаю рэп про космос!</speak>"]
    n._dispatch_turn.assert_not_called()


def test_state_answer_to_performance_word_is_spoken_once():
    n = make_result_node()
    n._dispatch_turn = MagicMock()

    n._handle_result(_reply("Сейчас тишина — ничего не играет."), user_input="играет ли сейчас музыка")

    assert _spoken(n) == ["<speak>Сейчас тишина — ничего не играет.</speak>"]
    n._dispatch_turn.assert_not_called()


def test_reply_text_never_starts_a_second_llm_call():
    """ADR-0150 PR-7 ч.7d-4: гарды «заявил без тула» (#992 Bug E, #2549, #2559) и фраза запуска без тула (#3455)
    сняты — ни на какой текст ответа нода не диспатчит синтетический ход и не подменяет фразу."""
    for text, user in (("Точка сохранена.", "запомни эту точку как кухня"),
                       ("Сейчас перезапущу трек.", "что-то тихо"),
                       ("Включаю диджей-сет. Тема — космос.", "как дела")):
        n = make_result_node()
        n._dispatch_turn = MagicMock()

        n._handle_result(_reply(text), user_input=user)

        n._dispatch_turn.assert_not_called()
        assert len(_spoken(n)) == 1, (text, _spoken(n))
