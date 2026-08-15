"""
test_issue_1278_fallback.py — Unit-тесты для бага #1278.

Баг: когда ВСЕ LLM-провайдеры недоступны (HealthAwareFallbackLLM
поднимает ``ProviderError("health-aware-fallback: все провайдеры
unavailable ...")``), робот говорил «Принял.» / «Что-то я задумался»
вместо честной degraded-фразы («интернет недоступен, но я могу
выполнять базовые команды»). ``_generate_fallback_response`` не
вызывалась нигде.

Покрывает:
  - _is_llm_unavailable_error   (распознавание ProviderError /
    health-aware-fallback маркера в обёрнутом Exception)
  - _handle_result с error=ProviderError → публикует degraded-фразу,
    НЕ «Принял.»
  - _handle_result с обычной ошибкой → НЕ degraded (идёт в «Принял.»)
"""

import asyncio
from unittest.mock import MagicMock

import pytest

from rob_box_llm.errors import ProviderError
from rob_box_voice.dialogue_node import DialogueNode

# ─────────────────────────────────────────────────────────────────────────────
#  Fixture: минимальная DialogueNode без __init__
# ─────────────────────────────────────────────────────────────────────────────

_HEALTH_AWARE_MSG = (
    "health-aware-fallback: все провайдеры unavailable "
    "(TTL ещё не истёк); ждём повторной проверки"
)


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
    return n


def _make_result(error=None, spoken: str = "", tools=None):
    """Минимальный DialogResult-подобный объект."""
    return MagicMock(
        error=error,
        spoken_text=spoken,
        tools_called=tools or [],
        finish_reason="stop",
        raw_response=None,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  _is_llm_unavailable_error
# ─────────────────────────────────────────────────────────────────────────────


class TestIsLlmUnavailableError:
    def test_provider_error_health_aware_is_unavailable(self):
        n = _make_node()
        err = ProviderError(_HEALTH_AWARE_MSG)
        assert n._is_llm_unavailable_error(err) is True

    def test_wrapped_exception_with_marker_is_unavailable(self):
        # DialogCore оборачивает LLM-исключение в plain Exception с
        # traceback (4ba16f23) — тип ProviderError теряется, маркер
        # остаётся в сообщении.
        n = _make_node()
        wrapped = Exception(f"{_HEALTH_AWARE_MSG}\nTraceback (most recent call last):\n...")
        assert n._is_llm_unavailable_error(wrapped) is True

    def test_plain_provider_error_is_unavailable(self):
        # Любой ProviderError = LLM-провайдер отказал → degraded-режим.
        n = _make_node()
        assert n._is_llm_unavailable_error(ProviderError("deepseek: stream() exited unexpectedly")) is True

    def test_ordinary_error_is_not_unavailable(self):
        n = _make_node()
        assert n._is_llm_unavailable_error(ValueError("boom")) is False
        assert n._is_llm_unavailable_error(RuntimeError("no LLM client")) is False

    def test_none_is_not_unavailable(self):
        n = _make_node()
        assert n._is_llm_unavailable_error(None) is False


# ─────────────────────────────────────────────────────────────────────────────
#  _handle_result — degraded-фраза вместо «Принял.»
# ─────────────────────────────────────────────────────────────────────────────


class TestHandleResultDegradedFallback:
    def test_provider_error_publishes_degraded_phrase_not_prinyal(self):
        n = _make_node()
        result = _make_result(error=ProviderError(_HEALTH_AWARE_MSG))
        n._handle_result(result, user_input="робот как дела")

        # Публикуем degraded-фразу из _generate_fallback_response
        # (содержит «интернет недоступен»), а НЕ «Принял.».
        published = [c.args[0].data for c in n._response_pub.publish.call_args_list]
        assert published, "response_pub должен был опубликовать degraded-ответ"
        joined = " | ".join(published).lower()
        assert "интернет" in joined, f"degraded-фраза не содержит «интернет»: {joined!r}"
        assert "принял" not in joined, f"не должно быть «Принял.»: {joined!r}"

    def test_wrapped_exception_publishes_degraded_phrase(self):
        n = _make_node()
        wrapped = Exception(f"{_HEALTH_AWARE_MSG}\nTraceback...")
        result = _make_result(error=wrapped)
        n._handle_result(result, user_input="как дела")

        published = [c.args[0].data for c in n._response_pub.publish.call_args_list]
        joined = " | ".join(published).lower()
        assert "интернет" in joined
        assert "принял" not in joined

    def test_greeting_gets_fallback_greeting(self):
        n = _make_node()
        result = _make_result(error=ProviderError(_HEALTH_AWARE_MSG))
        n._handle_result(result, user_input="привет робот")

        published = [c.args[0].data for c in n._response_pub.publish.call_args_list]
        joined = " | ".join(published).lower()
        assert "привет" in joined and "интернет" in joined

    def test_ordinary_error_still_falls_to_prinyal(self):
        # Обычная ошибка (не ProviderError) — поведение как раньше:
        # пустой spoken → empty-response fallback → «Принял.».
        n = _make_node()
        result = _make_result(error=ValueError("boom"))

        async def _run():
            n._handle_result(result, user_input="робот как дела")

        asyncio.run(_run())

        published = [c.args[0].data for c in n._response_pub.publish.call_args_list]
        joined = " | ".join(published).lower()
        assert "принял" in joined, f"ожидался «Принял.» для обычной ошибки: {joined!r}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
