"""Легаси-тесты механики ответа на вопрос о личности (#2809/#2888/#2906/#2914/#2925/#3024) после В17 (#3695).

С #3695 вопрос об имени звучит только после хода (``core/speaker_turn.plan_identity_question``) и не раньше
N-й фразы собеседника. Эти тесты проверяют не накопление, а то, что происходит ПОСЛЕ вопроса (чтение «да/нет»,
рост галереи, лицевая подсказка), поэтому накопление в них выключено (``min_phrases=0``), а тесты, зовущие
``_apply_speaker_identity`` напрямую, получают «конец хода» сразу после него — ход без действий с речью.
Накопление и запрет вопроса на команду проверяют ``test_speaker_turn_v17.py`` и
``rob_box_dialog/test/test_speaker_adapter.py``.
"""

from __future__ import annotations

from types import SimpleNamespace

from rob_box_dialog.speaker_adapter import AskPolicy
from rob_box_voice.core.speaker_turn import ask_planned, plan_identity_question
from rob_box_voice.dialogue_node import DialogueNode

PLAIN_TURN = SimpleNamespace(spoken_text="Понял.", tools_called=())


def no_accumulation(node) -> None:
    """Спрашивать на первой же фразе (как до В17) — для тестов механики ответа."""
    node._identity_ask = AskPolicy(min_phrases=0, min_s=0.0)


def finish_turn_after_identity(node) -> None:
    """``_apply_speaker_identity`` + сразу конец хода без действий (вопрос, если он положен)."""
    no_accumulation(node)

    async def apply_then_finish(*args, **kwargs):
        out = await DialogueNode._apply_speaker_identity(node, *args, **kwargs)
        node._identity_question_asked_in_turn = False
        ask_planned(node, plan_identity_question(node, PLAIN_TURN, other_question=False))
        return out

    node._apply_speaker_identity = apply_then_finish
