"""Ответ хода с тегами-обёртками звучит без тегов; обычный ответ звучит как есть.

Issue #3598 (рассуждение модели с ``<music_state>``/именами тулов) закрывала временная заплатка: ядро помечало
текст (``internal_text_marker``), оболочка его не озвучивала. ADR-0150 PR-7 ч.7d-3 (#3698) её сняла — в режиме команд
речь только поле ``speech``, ``content`` вне канала не озвучивается (§4.4).
"""

from __future__ import annotations

from rob_box_harness.core.agent_core import DialogResult

from ._result_node import make_result_node as _make_node, published_texts as _published_texts


def _result(spoken: str, tools=()) -> DialogResult:
    return DialogResult(spoken_text=spoken, tools_called=list(tools), finish_reason="stop")


def test_tag_wrapped_reply_is_spoken_without_tags():
    """Живой лог 13.10 09:17: ``spoken='<Текст>…</Текст>'`` ушёл в TTS с тегами."""
    n = _make_node()
    n._handle_result(
        _result("<Текст>У РОББОКСа нет Вагнера в библиотеке.</Текст>"),
        user_input="[TG] продолжай играть вагнера",
        raw_user_command="продолжай играть вагнера",
    )
    published = " ".join(_published_texts(n))
    assert "У РОББОКСа нет Вагнера" in published
    assert "Текст" not in published


def test_unmarked_reply_is_spoken_as_before():
    n = _make_node()
    n._handle_result(
        _result("Сейчас тишина. Что включить?"),
        user_input="[TG] как дела",
        raw_user_command="как дела",
    )
    assert "Сейчас тишина" in " ".join(_published_texts(n))
