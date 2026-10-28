"""Issue #2760 — разметка вызова тула не должна звучать (``rob_box_core.tool_call_markup``).

Живой прогон 35704637846 (акт 2, шаги n204/n206 — оба про сохранение факта):
MiniMax-M3 вернула вызов тулов ТЕКСТОМ при ``tools=[]``, и он прошёл весь тракт::

    [dialogue_node] 📤 LLM OUTPUT: '<functioncalls>\\n<invoke name="registerspeaker">…'
    [tts_node] 🔊 TTS: batch=9b15b85e 1/2, text='<functioncalls>…'

Штатный путь лечения — восстановление намерения внутри цикла тулов
(``rob_box_harness.core.tool_loop.markup_recovery``). Предикат ниже — последний рубеж: если разметка всё-таки доехала
до выхода в TTS (например, имя тула не опознано), её не озвучивает ``tts_node``.

Регекс одна копия (PR-15 часть 15-core, #3711): ``tts_node`` и harness (``text_classify``) импортируют его из
``rob_box_core``; до этого он дублировался через границу пакетов, и расхождение ловил одинаковый корпус двух тестов.
"""

from __future__ import annotations

import pytest

from rob_box_core.tool_call_markup import TOOL_CALL_MARKUP_RE, is_tool_call_markup


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


def test_harness_and_tts_node_share_the_one_regex() -> None:
    """Второй копии нет: harness реэкспортирует тот же объект, а не свой компилированный регекс."""
    from rob_box_harness.core.tool_loop import text_classify

    assert text_classify.TOOL_CALL_MARKUP_RE is TOOL_CALL_MARKUP_RE
    assert text_classify.is_tool_call_markup is is_tool_call_markup
