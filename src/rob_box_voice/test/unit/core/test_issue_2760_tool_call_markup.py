"""Issue #2760 — разметка вызова тула не должна звучать.

Живой прогон 35704637846 (акт 2, шаги n204/n206 — оба про сохранение
факта): MiniMax-M3 вернула вызов тулов ТЕКСТОМ при ``tools=[]``, и он
прошёл весь тракт::

    [dialogue_node] 📤 LLM OUTPUT: '<functioncalls>\\n<invoke name="registerspeaker">…'
    [tts_node] 🔊 TTS: batch=9b15b85e 1/2, text='<functioncalls>…'

Штатный путь лечения — восстановление намерения внутри цикла тулов
(``rob_box_harness.core.tool_loop.markup_recovery``). Этот guard —
последний рубеж: если разметка всё-таки доехала до выхода в TTS
(например, имя тула не опознано), она не звучит, а уходит в ретрай.

Корпус строк ЗДЕСЬ И в ``rob_box_harness/test/test_issue_2760_tool_call_markup.py``
намеренно одинаковый: регекс продублирован через границу пакетов (harness
не может импортировать voice), и расхождение поймается тем, что один из
двух наборов покраснеет.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.dialogue_guards import (
    build_tool_call_markup_retry_prompt,
    is_tool_call_markup,
)
from rob_box_voice.core.turn import GuardContext, ToolCallMarkupGuard, VerdictKind

from .test_turn import _reply, _state, _turn  # type: ignore[attr-defined]


LIVE_N206 = (
    '<function_calls>\n<invoke name="memory_save">\n'
    '<parameter name="fact">Болеет за Спартак с 98 года, всегда приносит пиццу</parameter>\n'
    '<parameter name="category">general</parameter>\n'
    '<parameter name="speaker_id">05ff0881-b2d5-47b7-a096-7234a8a21738</parameter>\n'
    "</invoke>\n</function_calls>"
)

#: Тот же текст после ``strip_markdown``: тот снимает ``_..._`` парами
#: через весь текст, и в лог робота уехало уже ``memorysave``/``speakerid``.
#: Именно поэтому детектор смотрит на ТЕГИ, а не на имена тулов.
LIVE_N206_FLATTENED = LIVE_N206.replace("_", "")

MINIMAX_NATIVE = (
    '<minimax:tool_call>\n<invoke name="search_web">\n'
    '<parameter name="query_list">["погода в Москве"]</parameter>\n'
    "</invoke>\n</minimax:tool_call>"
)


class TestDetector:
    @pytest.mark.parametrize(
        "text",
        [
            pytest.param(LIVE_N206, id="live_n206"),
            pytest.param(LIVE_N206_FLATTENED, id="live_n206_after_strip_markdown"),
            pytest.param(MINIMAX_NATIVE, id="minimax_native"),
            pytest.param('Привет! <invoke name="stop_music"></invoke>', id="mixed_with_speech"),
        ],
    )
    def test_markup_is_detected(self, text: str) -> None:
        assert is_tool_call_markup(text) is True

    @pytest.mark.parametrize(
        "text",
        [
            pytest.param("", id="empty"),
            pytest.param(None, id="none"),
            pytest.param("Записала: зелёный чай без сахара.", id="plain_speech"),
            pytest.param("Я вызвала memory_save и всё сохранила.", id="tool_name_in_prose"),
            pytest.param(
                '<speak><break time="300ms"/><prosody pitch="high">Привет</prosody></speak>',
                id="ssml_is_not_tool_markup",
            ),
        ],
    )
    def test_speech_is_not_markup(self, text) -> None:
        assert is_tool_call_markup(text) is False


class TestGuard:
    def test_fires_and_asks_for_a_real_tool_call(self) -> None:
        g = ToolCallMarkupGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken=LIVE_N206),
                turn=_turn(user_input="запомни про меня: болею за Спартак"),
                state=_state(),
            )
        )
        assert v is not None, "разметка обязана ловиться до выхода в TTS"
        assert v.kind is VerdictKind.RETRY
        assert v.guard_name == "tool_call_markup"
        assert v.prompt and "CRITICAL" in v.prompt

    def test_defers_on_normal_reply(self) -> None:
        g = ToolCallMarkupGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken="Запомнила: зелёный чай без сахара."),
                turn=_turn(user_input="запомни"),
                state=_state(),
            )
        )
        assert v is None

    def test_defers_when_speech_already_happened(self) -> None:
        """Если ``speak_text`` уже реально прозвучала — ретраить нечего."""
        g = ToolCallMarkupGuard()
        v = g.evaluate(
            GuardContext(
                reply=_reply(spoken=LIVE_N206, speak_text_real=1),
                turn=_turn(user_input="x"),
                state=_state(),
            )
        )
        assert v is None


class TestRetryPrompt:
    def test_names_the_mistake_and_the_remedy(self) -> None:
        p = build_tool_call_markup_retry_prompt("запомни про меня: болею за Спартак")
        assert "CRITICAL" in p
        assert "tool-calls" in p
        # Промпт обязан цитировать запрос юзера, иначе ретрай теряет задачу.
        assert "Спартак" in p
