"""test_issue_1343_empty_speak_text.py — Unit-тесты для бага #1343.

Баг: робот «принял» и замолчал — deepseek иногда возвращает
``speak_text({})`` / ``speak_text({"text": ""})`` (пустой текст).
Имя инструмента попадает в ``tools_called``, но реального вызова
(и реальной озвучки) нет — MCP валидация отклоняет пустой текст до
отправки запроса. Старый issue-988 guard (`"speak_text" in
tools_called`) считал это «LLM уже озвучила» и СКИПАЛ auto-TTS
финального текста → юзер слышал только акцепт и тишину.

Фикс (#1343): DialogCore считает ``speak_text_real_count`` — сколько
speak_text-вызовов пришли с НЕПУСТЫМ ``text``. Guard скипает auto-TTS
только когда реальных вызовов > 0.

Покрывает:
  - tools_called содержит speak_text, но 0 реальных вызовов →
    финальный текст ОЗВУЧИВАЕТСЯ (не тишина)
  - реальный вызов speak_text (real_count > 0) → auto-TTS скипается
    (без дубля, issue #988)
"""

from unittest.mock import MagicMock

from rob_box_harness.core.dialog_core import DialogResult
from rob_box_voice.dialogue_node import DialogueNode


def _make_node() -> DialogueNode:
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    n._response_pub = MagicMock()
    n.response_pub = MagicMock()
    n.animation_pub = MagicMock()
    n.sound_trigger_pub = MagicMock()
    n.state_pub = MagicMock()
    n.current_dialogue_id = None
    n.available_tools = []
    n.mcp_tools_available = False
    n.internet_available = True
    n.current_time_info = {}
    n._verbose_llm = False
    n._babble_retry_used = False
    n._memory = MagicMock()
    n._active_tg_chat_id = None
    return n


def _make_result(spoken: str = "", tools=None, real_count: int = 0) -> DialogResult:
    """Реальный DialogResult с полем speak_text_real_count (issue #1343)."""
    return DialogResult(
        spoken_text=spoken,
        tools_called=list(tools or []),
        speak_text_real_count=real_count,
        finish_reason="stop",
    )


def _published_texts(node: DialogueNode) -> list[str]:
    return [c.args[0].data for c in node._response_pub.publish.call_args_list]


class TestIssue1343EmptySpeakText:
    def test_phantom_speak_text_does_not_suppress_auto_tts(self):
        """tools_called содержит speak_text, но real_count=0 → TTS финального текста.

        Воспроизводит инцидент: LLM вернула speak_text({}) + непустой
        spoken → раньше тишина, теперь финальный текст озвучивается.
        """
        n = _make_node()
        result = _make_result(
            spoken="Это ошибка — пустой вызов. Выполню правильно.",
            tools=["memory_context", "speak_text"],
            real_count=0,
        )
        n._handle_result(result, user_input="это иван а теперь пожалуйста мне на денчика")

        published = _published_texts(n)
        assert published, "финальный текст должен быть озвучен (issue #1343)"
        joined = " | ".join(published)
        assert "Выполню правильно" in joined, f"текст не озвучен: {joined!r}"

    def test_real_speak_text_still_suppresses_auto_tts(self):
        """Реальный speak_text (real_count>0) → auto-TTS скипается (issue #988)."""
        n = _make_node()
        result = _make_result(
            spoken="done",
            tools=["speak_text"],
            real_count=1,
        )
        n._handle_result(result, user_input="робот спой песенку")

        published = _published_texts(n)
        assert published == [], f"auto-TTS должен быть скипнут: {published!r}"

    def test_no_speak_text_normal_auto_tts(self):
        """Никакого speak_text → обычная озвучка (регрессия #988)."""
        n = _make_node()
        result = _make_result(
            spoken="Привет!",
            tools=[],
            real_count=0,
        )
        n._handle_result(result, user_input="робот привет")

        published = _published_texts(n)
        assert published, "обычный текст должен озвучиваться"
        assert "Привет" in " | ".join(published)


if __name__ == "__main__":  # pragma: no cover
    import sys

    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
