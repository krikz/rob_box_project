"""Issue #2760 — модель пишет вызов тула текстом вместо function-calling.

Живой прогон 35704637846 (акт 2, шаги n204/n206 — оба про сохранение
факта), модель MiniMax-M3, ``provider=minimax``, ``mode=stream``, 57 тулов
в запросе. Разметка уходила в TTS и оседала в истории.

Это известное поведение MiniMax: их модели вызывают тулы XML-ом
(``<minimax:tool_call><invoke name=…><parameter name=…>``), и официальный
``docs/tool_calling_guide.md`` прямо предлагает разбирать сырой вывод
самостоятельно, если под рукой нет парсера vLLM/SGLang. Здесь закреплены
оба диалекта: родной ``minimax:`` и тот, что приехал живьём.
"""

from __future__ import annotations

from typing import Any, Dict, List

import pytest

from rob_box_harness.core.tool_loop.markup_recovery import parse_tool_call_markup
from rob_box_harness.core.tool_loop.text_classify import is_tool_call_markup


def _tool(name: str, props: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return {
        "type": "function",
        "function": {
            "name": name,
            "description": "",
            "parameters": {"type": "object", "properties": props or {}},
        },
    }


TOOLS: List[Dict[str, Any]] = [
    _tool(
        "memory_save",
        {
            "fact": {"type": "string"},
            "category": {"type": "string"},
            "speaker_id": {"type": "string"},
        },
    ),
    _tool("register_speaker", {"name": {"type": "string"}}),
    _tool("set_volume", {"steps": {"type": "integer"}}),
    _tool("play_sound", {"loop": {"type": "boolean"}}),
    _tool("search_web", {"query_list": {"type": "array"}}),
]

# Дословно из лога робота (шаг n206).
LIVE_N206 = (
    '<function_calls>\n<invoke name="memory_save">\n'
    '<parameter name="fact">Болеет за Спартак с 98 года, всегда приносит пиццу</parameter>\n'
    '<parameter name="category">general</parameter>\n'
    '<parameter name="speaker_id">05ff0881-b2d5-47b7-a096-7234a8a21738</parameter>\n'
    "</invoke>\n</function_calls>"
)

# Тот же ответ после ``strip_markdown`` на стороне voice — подчёркивания
# схлопнулись парами через весь текст (в логе: `memorysave`, `speakerid`).
LIVE_N206_FLATTENED = LIVE_N206.replace("_", "")

# Родной диалект MiniMax из их tool_calling_guide.md.
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
            pytest.param("Привет! <invoke name=\"stop_music\"></invoke>", id="mixed_with_speech"),
        ],
    )
    def test_markup_is_detected(self, text: str) -> None:
        assert is_tool_call_markup(text) is True

    @pytest.mark.parametrize(
        "text",
        [
            pytest.param("", id="empty"),
            pytest.param("Записала: зелёный чай без сахара.", id="plain_speech"),
            pytest.param("Я вызвала memory_save и всё сохранила.", id="tool_name_in_prose"),
            # SSML — законная разметка речи, её трогать нельзя.
            pytest.param(
                '<speak><break time="300ms"/><prosody pitch="high">Привет</prosody></speak>',
                id="ssml_is_not_tool_markup",
            ),
        ],
    )
    def test_speech_is_not_markup(self, text: str) -> None:
        assert is_tool_call_markup(text) is False


class TestRecovery:
    def test_live_case_is_recovered(self) -> None:
        calls = parse_tool_call_markup(LIVE_N206, tools=TOOLS)
        assert len(calls) == 1
        name, args = calls[0]
        assert name == "memory_save"
        assert args["category"] == "general"
        assert args["speaker_id"] == "05ff0881-b2d5-47b7-a096-7234a8a21738"
        assert "Спартак" in args["fact"]

    def test_flattened_names_map_back_to_real_tools(self) -> None:
        """``memorysave`` после strip_markdown — всё ещё ``memory_save``."""
        calls = parse_tool_call_markup(LIVE_N206_FLATTENED, tools=TOOLS)
        assert [name for name, _ in calls] == ["memory_save"]

    def test_several_invokes_keep_order(self) -> None:
        text = (
            '<function_calls>\n<invoke name="register_speaker">\n'
            '<parameter name="name">Борис</parameter>\n</invoke>\n'
            '<invoke name="memory_save">\n'
            '<parameter name="fact">Друг Саши</parameter>\n</invoke>\n'
            "</function_calls>"
        )
        calls = parse_tool_call_markup(text, tools=TOOLS)
        assert [name for name, _ in calls] == ["register_speaker", "memory_save"]

    def test_minimax_native_dialect(self) -> None:
        calls = parse_tool_call_markup(MINIMAX_NATIVE, tools=TOOLS)
        assert len(calls) == 1
        name, args = calls[0]
        assert name == "search_web"
        # array по схеме → распарсенный список, а не строка с квадратными
        # скобками (иначе валидация тула отвергнет аргумент).
        assert args["query_list"] == ["погода в Москве"]

    @pytest.mark.parametrize(
        "tool,param,raw,expected",
        [
            pytest.param("set_volume", "steps", "3", 3, id="integer"),
            pytest.param("play_sound", "loop", "true", True, id="boolean_true"),
            pytest.param("play_sound", "loop", "False", False, id="boolean_false"),
            pytest.param("memory_save", "fact", "3", "3", id="string_stays_string"),
        ],
    )
    def test_types_follow_the_schema(
        self, tool: str, param: str, raw: str, expected: Any
    ) -> None:
        text = (
            f'<function_calls><invoke name="{tool}">'
            f'<parameter name="{param}">{raw}</parameter>'
            "</invoke></function_calls>"
        )
        calls = parse_tool_call_markup(text, tools=TOOLS)
        assert calls[0][1][param] == expected
        assert type(calls[0][1][param]) is type(expected)

    def test_unparsable_value_stays_a_string(self) -> None:
        """Мусор в числовом поле не превращаем в ``None``.

        Пусть валидация тула скажет о нём явно — это диагностичнее, чем
        молча подставленный пустой аргумент.
        """
        text = (
            '<function_calls><invoke name="set_volume">'
            '<parameter name="steps">погромче</parameter>'
            "</invoke></function_calls>"
        )
        assert parse_tool_call_markup(text, tools=TOOLS)[0][1]["steps"] == "погромче"

    def test_unknown_tool_recovers_nothing(self) -> None:
        """Имя вне предложенного набора — не исполняем НИЧЕГО.

        Восстановление не должно расширять права модели, а частично
        исполненный ход хуже честного ретрая: юзер услышал бы
        подтверждение половины действия.
        """
        text = (
            '<function_calls><invoke name="register_speaker">'
            '<parameter name="name">Борис</parameter></invoke>'
            '<invoke name="launch_missile"></invoke></function_calls>'
        )
        assert parse_tool_call_markup(text, tools=TOOLS) == ()

    @pytest.mark.parametrize(
        "text",
        [
            pytest.param("", id="empty"),
            pytest.param(None, id="none"),
            pytest.param("Записала: зелёный чай.", id="plain_speech"),
        ],
    )
    def test_nothing_to_recover(self, text: str | None) -> None:
        assert parse_tool_call_markup(text, tools=TOOLS) == ()

    def test_empty_tool_list_recovers_nothing(self) -> None:
        assert parse_tool_call_markup(LIVE_N206, tools=[]) == ()
