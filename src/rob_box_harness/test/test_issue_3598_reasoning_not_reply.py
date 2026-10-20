"""Issue #3598 — рассуждение модели в финальном тексте хода — не реплика.

Живой лог 13.10 08:38 UTC (Vision Pi): на «[TG] продолжай играть вагнера»
MiniMax вернула ``tools=[]`` и разбор от третьего лица с ``<music_state>``
и ``speak_text`` внутри; авто-TTS прочитал его шестью чанками, а в 08:31
такой же разбор («… RULE #LANG и §1 … через `speak_text` …») лёг в окно
истории ходом ``[3] assistant`` и задал тон следующему ходу.

Контракт: ядро помечает такой текст (``internal_text_marker``) и не пишет
его в историю. Словарь проверки — не рукописный: теги контекста этого хода
и имена тулов единого каталога.
"""

from __future__ import annotations

import asyncio

import pytest

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import DialogueStateMachine
from rob_box_harness.core.tool_loop.text_classify import quoted_turn_context

from .test_agent_core import _FakeLLMProvider, _FakeMemoryStore, _FakeToolProvider, _wake

#: Финальный текст хода 08:38:56 (``process_input returned: spoken=…``), дословно.
WAGNER_REASONING = (
    "[Spkr:Литвин] — TG-сообщение «продолжай играть вагнера». Это не директорский "
    "приказ, а гостевая реплика в чате, плюс запрос бессмысленный: (а) «продолжай» "
    "предполагает, что я что-то уже играю, но `<music_state> playing=\"no\" "
    "last_ended=\"finished\" last_ended_ago_s=\"22\"` — последний трек «без "
    "названия» закончился 22 секунды назад, я сейчас в полной тишине; (б) «вагнера» "
    "— Виктор или Рихард? … Скажу коротко через `speak_text`, что в фоне тишина и "
    "прошлый трек уже отзвенел — пусть уточнит, что именно хочет."
)

#: Финальный текст хода 08:31:56 («блядь на ситаре» → в историю ``[3] assistant``).
SITAR_REASONING = (
    "[Spkr:Литвин] — в чате проскочила обсценная реплика, это явно не директорская "
    "команда, а мат/эмоция гостя. Согласно RULE #LANG и §1 — не подыгрываю мату. "
    "Отвечать через `speak_text` тут тоже не нужно — это не voice-канал, а Telegram. "
    "Возвращаю \"done\" без инструмента."
)

#: Контекст хода из того же лога (сокращён до тегов, что собрал код).
TURN_CONTEXT = (
    "<system_context>\n  <user_profile>\n    <name>Литвин</name>\n  </user_profile>\n"
    "  <hardware>\n    <battery>unknown</battery>\n  </hardware>\n"
    "  <music_state playing=\"no\" last_ended=\"finished\" />\n</system_context>"
)


class TestQuotedTurnContext:
    def test_wagner_reasoning_quotes_music_state(self):
        assert quoted_turn_context(WAGNER_REASONING, TURN_CONTEXT) == "<music_state>"

    def test_sitar_reasoning_names_speak_text(self):
        assert quoted_turn_context(SITAR_REASONING, TURN_CONTEXT) == "speak_text"

    def test_tool_name_found_without_context(self):
        assert quoted_turn_context("Запускаю через request_music.", None) == "request_music"

    @pytest.mark.parametrize(
        "reply",
        [
            "Поставила «Полёт валькирий», Вагнер.",
            "Музыка сейчас не играет — что включить?",
            "Скажу так: всё в порядке, работаю.",
            "",
        ],
    )
    def test_human_replies_pass(self, reply):
        assert quoted_turn_context(reply, TURN_CONTEXT) is None

    def test_identifier_needs_word_boundary(self):
        """``speak_textual`` — не имя тула ``speak_text``."""
        assert quoted_turn_context("слово speak_textual", None) is None


def _core(llm):
    core = AgentCore(
        llm=llm,
        tools=_FakeToolProvider(),
        memory=_FakeMemoryStore(),
        dsm=DialogueStateMachine(),
        system_prompt="ПРОМПТ",
    )
    _wake(core)
    return core


def test_reasoning_turn_is_marked_and_not_persisted():
    core = _core(_FakeLLMProvider(response_text=WAGNER_REASONING))
    result = asyncio.run(
        core.process_input("[TG] продолжай играть вагнера", dynamic_system=TURN_CONTEXT)
    )
    assert result.internal_text_marker == "<music_state>"
    roles = [t.role for t in core._turn_window]
    assert roles == ["user"], f"рассуждение легло в историю: {core._turn_window!r}"


def test_plain_reply_is_persisted_unmarked():
    core = _core(_FakeLLMProvider(response_text="Сейчас тишина, что включить?"))
    result = asyncio.run(core.process_input("[TG] как дела", dynamic_system=TURN_CONTEXT))
    assert result.internal_text_marker is None
    assert [t.role for t in core._turn_window] == ["user", "assistant"]
