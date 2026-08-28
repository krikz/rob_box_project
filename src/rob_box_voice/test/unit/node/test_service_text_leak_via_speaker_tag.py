"""test_service_text_leak_via_speaker_tag.py — regression for the 20.08 DJ leak.

Баг: LLM скопировала входной маршрутный маркер ``[Spkr:<имя>]`` в свой
финальный ответ. Служебный ``[CRITICAL]``-ретрай-промпт начинался с
``[Spkr:Эйджик]``, поэтому startswith-проверка служебного текста в
``_handle_result`` не срабатывала — и внутренняя инструкция целиком
уходила в TTS (робот «говорил лишнее»: «[CRITICAL] В прошлом цикле ты
НЕ вызвал ни один музыкальный тул...»).

Фикс: ``strip_speaker_tag`` снимает ведущий ``[Spkr:...]`` перед
проверкой служебных маркеров, поэтому внутренний текст больше не
озвучивается.

Покрывает:
  - ``[Spkr:Эйджик] [CRITICAL] ...`` → НЕ озвучивается (тихо)
  - обычный ``[CRITICAL] ...`` без тега → по-прежнему НЕ озвучивается
  - ``[Spkr:Эйджик] Привет...`` → озвучивается БЕЗ маркера спикера
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
    n.response_pub = n._response_pub
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


def _make_result(spoken: str = "", tools=None) -> DialogResult:
    return DialogResult(
        spoken_text=spoken,
        tools_called=list(tools or []),
        speak_text_real_count=0,
        finish_reason="stop",
    )


def _published_texts(node: DialogueNode) -> list:
    return [c.args[0].data for c in node._response_pub.publish.call_args_list]


_LEAK = (
    "[Spkr:Эйджик] [CRITICAL] В прошлом цикле ты НЕ вызвал ни один "
    "музыкальный тул, хотя пользователь ЯВНО попросил диджей/вечеринку."
)


class TestServiceTextLeakViaSpeakerTag:
    def test_service_text_with_speaker_tag_is_not_voiced(self):
        n = _make_node()
        result = _make_result(spoken=_LEAK)
        n._handle_result(result, user_input="[DJ_AUTO] ...")

        assert _published_texts(n) == [], (
            f"служебный [CRITICAL] после [Spkr:...] не должен озвучиваться: "
            f"{_published_texts(n)!r}"
        )

    def test_plain_service_text_still_suppressed(self):
        # Регрессия: без спикер-тега служебный текст подавлялся и раньше.
        n = _make_node()
        result = _make_result(spoken=_LEAK.removeprefix("[Spkr:Эйджик] "))
        n._handle_result(result, user_input="[DJ_AUTO] ...")

        assert _published_texts(n) == []

    def test_speaker_tag_stripped_from_normal_speech(self):
        n = _make_node()
        result = _make_result(spoken="[Spkr:Эйджик] Привет, как дела?")
        n._handle_result(result, user_input="привет")

        published = _published_texts(n)
        assert published, "обычная речь после тега должна озвучиваться"
        joined = " | ".join(published)
        assert "Привет, как дела?" in joined
        assert "[Spkr" not in joined, f"маркер спикера попал в TTS: {joined!r}"


if __name__ == "__main__":  # pragma: no cover
    import sys

    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
