"""Issue #3598 — рассуждение модели не уходит в TTS и Telegram-дубль.

Живой лог 13.10 08:38:56 UTC: ``process_input returned: spoken='[Spkr:Литвин]
— TG-сообщение «продолжай играть вагнера». Это не директорский приказ … но
`<music_state> playing="no" …` … Скажу коротко через `speak_text` …'
tools=[]``, далее ``TTS batch: 6 chunks`` — первые чанки «— TG-сообщение
«продолжай играть вагнера».», «но <musicstate> playing="no" lastended=…».
Гард #1882 промахнулся: к нему текст пришёл после ``strip_markdown`` —
``musicstate``/``speaktext`` без подчёркиваний.

Решение: ядро помечает такой текст (``internal_text_marker``), оболочка его
не озвучивает; ход без тулов получает честную фразу кода
(``EMPTY_REPLY_PHRASE``), ход с тулами — фразу кода о действии или тишину.
"""

from __future__ import annotations

from rob_box_harness.core.agent_core import DialogResult
from rob_box_voice.core.speak_helpers import EMPTY_REPLY_PHRASE

from ._result_node import make_result_node as _make_node, published_texts as _published_texts

WAGNER_REASONING = (
    "[Spkr:Литвин] — TG-сообщение «продолжай играть вагнера». Это не директорский "
    "приказ, а гостевая реплика в чате, плюс запрос бессмысленный: (а) «продолжай» "
    "предполагает, что я что-то уже играю, но `<music_state> playing=\"no\" "
    "last_ended=\"finished\" last_ended_ago_s=\"22\"` — последний трек «без "
    "названия» закончился 22 секунды назад, я сейчас в полной тишине. Скажу коротко "
    "через `speak_text`, что в фоне тишина — пусть уточнит, что именно хочет."
)


def _result(spoken: str, marker, tools=()) -> DialogResult:
    return DialogResult(
        spoken_text=spoken,
        tools_called=list(tools),
        finish_reason="stop",
        internal_text_marker=marker,
    )


def test_reasoning_is_not_spoken_and_honest_phrase_instead():
    n = _make_node()
    n._handle_result(
        _result(WAGNER_REASONING, "<music_state>"),
        user_input="[TG] продолжай играть вагнера",
        raw_user_command="продолжай играть вагнера",
    )
    published = " ".join(_published_texts(n))
    for leaked in ("директорский", "music", "speak_text", "speaktext", "TG-сообщение", "Литвин"):
        assert leaked not in published, f"рассуждение ушло в TTS: {published!r}"
    assert EMPTY_REPLY_PHRASE in published
    warnings = [c.args[0] for c in n.get_logger().warning.call_args_list if c.args]
    assert any("[issue 3598]" in w and "<music_state>" in w for w in warnings), warnings


def test_tag_wrapped_reply_is_spoken_without_tags():
    """Живой лог 13.10 09:17: ``spoken='<Текст>…</Текст>'`` ушёл в TTS с тегами."""
    n = _make_node()
    n._handle_result(
        _result("<Текст>У РОББОКСа нет Вагнера в библиотеке.</Текст>", None),
        user_input="[TG] продолжай играть вагнера",
        raw_user_command="продолжай играть вагнера",
    )
    published = " ".join(_published_texts(n))
    assert "У РОББОКСа нет Вагнера" in published
    assert "Текст" not in published


def test_unmarked_reply_is_spoken_as_before():
    n = _make_node()
    n._handle_result(
        _result("Сейчас тишина. Что включить?", None),
        user_input="[TG] как дела",
        raw_user_command="как дела",
    )
    assert "Сейчас тишина" in " ".join(_published_texts(n))
