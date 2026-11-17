"""MiniMax-M3.1-Flash: thinking выключить нельзя — запрос обязан это учесть.

Живая проба 20.10 (Sound Pi, ключ робота, провайдер собран как в проде)::

    400: invalid params, model "MiniMax-M3.1-Flash" requires adaptive
    thinking; thinking.type="disabled" (including reasoning.effort=none)
    is not allowed (2013)

Голосовой ход шлёт ``thinking={"type": "disabled"}`` (DEFAULT_THINKING_POLICY),
так что с одной лишь заменой имени модели КАЖДЫЙ ход падал бы в фолбек.
Знание «эта модель не умеет без thinking» живёт в одном месте —
``rob_box_llm.providers.minimax`` — и превращает «выключи» в самую малую
глубину рассуждения (``reasoning_effort``).

Двойник API здесь отвечает так же, как MiniMax: 400 на ``disabled`` для
Flash. Правило в двойнике записано отдельно от кода провайдера, чтобы тест не
сверял код сам с собой.

Решение Шифу 20.10: по умолчанию остаётся MiniMax-M3 (``DEFAULT_MODEL``), Flash
— запас на будущее. Поэтому Flash-тесты называют модель явно, а отдельный блок
ниже фиксирует, что экземпляр по умолчанию (M3) шлёт ровно то, что слал до
адаптера под Flash.
"""

from __future__ import annotations

import asyncio
from types import SimpleNamespace
from typing import Any, AsyncIterator

import pytest
from openai import APIStatusError

from rob_box_llm.provider import LLMMessage, LLMSettings
from rob_box_llm.providers.deepseek import _thinking_label
from rob_box_llm.providers.minimax import (
    DEFAULT_THINKING_POLICY,
    LOWEST_REASONING_EFFORT,
    MiniMaxProvider,
    apply_model_thinking,
    model_always_thinks,
)

FLASH = "MiniMax-M3.1-Flash"
M3 = "MiniMax-M3"


class _Rejected(APIStatusError):
    def __init__(self, body: dict[str, Any]) -> None:
        self.status_code = 400
        self.body = body
        self.request = None
        Exception.__init__(self, f"400: {body}")


def _api_rejects(kwargs: dict[str, Any]) -> None:
    """Правило API (live 20.10): Flash отвечает 400 на thinking.type=disabled."""
    thinking = (kwargs.get("extra_body") or {}).get("thinking")
    if "flash" in kwargs["model"].lower() and isinstance(thinking, dict) and thinking.get("type") == "disabled":
        raise _Rejected({"type": "bad_request_error", "message": "requires adaptive thinking (2013)"})


def _reply(content: str = "ок") -> Any:
    message = SimpleNamespace(content=content, tool_calls=None)
    return SimpleNamespace(choices=[SimpleNamespace(message=message, finish_reason="stop")], usage=None, base_resp={})


def _event(content: str | None = None, finish: str | None = None) -> Any:
    delta = SimpleNamespace(content=content, tool_calls=None)
    return SimpleNamespace(choices=[SimpleNamespace(delta=delta, finish_reason=finish)], usage=None, base_resp={})


class _Completions:
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        _api_rejects(kwargs)
        if kwargs.get("stream"):
            return self._stream()
        return _reply()

    async def _stream(self) -> AsyncIterator[Any]:
        yield _event("ок")
        yield _event(finish="stop")


def _client() -> tuple[Any, _Completions]:
    completions = _Completions()
    return SimpleNamespace(chat=SimpleNamespace(completions=completions), is_closed=False), completions


def _provider(model: str = FLASH, **kwargs: Any) -> tuple[MiniMaxProvider, _Completions]:
    client, completions = _client()
    return MiniMaxProvider(api_key="sk-test", client=client, model=model, **kwargs), completions  # type: ignore[arg-type]


def _default_provider() -> tuple[MiniMaxProvider, _Completions]:
    """Экземпляр без имени модели — как собирает прод (``catalog`` → ``DEFAULT_MODEL``)."""
    client, completions = _client()
    return MiniMaxProvider(api_key="sk-test", client=client), completions  # type: ignore[arg-type]


async def _drain(provider: MiniMaxProvider, settings: LLMSettings | None) -> str:
    messages = [LLMMessage(role="user", content="привет")]
    return "".join([chunk.content_delta async for chunk in provider.stream(messages, settings=settings)])


def _ask(provider: MiniMaxProvider, settings: LLMSettings | None, *, stream: bool) -> str:
    if stream:
        return asyncio.run(_drain(provider, settings))
    return asyncio.run(provider.complete([LLMMessage(role="user", content="привет")], settings=settings)).content


# ── имя модели живёт в одном месте; по умолчанию — M3 ───────────────────


def test_default_model_is_m3_and_flash_is_opt_in() -> None:
    assert MiniMaxProvider.DEFAULT_MODEL == M3
    assert _default_provider()[0]._default_model == M3


# ── M3 по умолчанию: на проде не меняется ни бита ───────────────────────


@pytest.mark.parametrize("stream", [False, True])
def test_default_instance_sends_thinking_disabled_as_before(stream: bool) -> None:
    provider, completions = _default_provider()

    _ask(provider, LLMSettings(temperature=0.7, max_tokens=3000), stream=stream)

    sent = completions.calls[0]
    assert sent["model"] == M3
    assert sent["extra_body"] == {"thinking": {"type": "disabled"}}, "M3 получает disabled, глубины нет"
    assert "reasoning_effort" not in sent["extra_body"]


def test_default_instance_keeps_image_input_and_sends_thinking_disabled_with_a_picture() -> None:
    from rob_box_llm.provider import ImagePart, TextPart

    provider, completions = _default_provider()
    jpeg = bytes([0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10]) + b"JFIF"
    content = (TextPart("Что на картинке?"), ImagePart(source=jpeg, media_type="image/jpeg"))

    assert provider.capabilities_for(None).image_input is True
    assert provider.capabilities_for(M3).image_input is True
    asyncio.run(provider.complete([LLMMessage(role="user", content=content)]))

    sent = completions.calls[0]
    parts = sent["messages"][0]["content"]
    assert [part["type"] for part in parts] == ["text", "image_url"]
    assert sent["extra_body"] == {"thinking": {"type": "disabled"}}


# ── Flash (опционально): «thinking выключен» на модели, которая не умеет ─


@pytest.mark.parametrize("stream", [False, True])
def test_voice_turn_on_flash_is_not_rejected_by_the_api(stream: bool) -> None:
    provider, completions = _provider()

    text = _ask(provider, LLMSettings(temperature=0.7, max_tokens=3000), stream=stream)

    assert text == "ок"
    assert completions.calls[0]["model"] == FLASH


@pytest.mark.parametrize("stream", [False, True])
def test_voice_turn_asks_for_the_lowest_depth_instead_of_disabled(stream: bool) -> None:
    provider, completions = _provider()

    _ask(provider, LLMSettings(max_tokens=3000), stream=stream)

    assert completions.calls[0]["extra_body"] == {"reasoning_effort": LOWEST_REASONING_EFFORT}


@pytest.mark.parametrize("model", [FLASH, "MiniMax-M3.1-Flash-Preview", "minimax-m3.1-flash"])
def test_every_spelling_of_the_always_thinking_model_is_adapted(model: str) -> None:
    provider, completions = _provider(model=model)

    _ask(provider, None, stream=False)

    assert completions.calls[0]["extra_body"] == {"reasoning_effort": LOWEST_REASONING_EFFORT}


def test_turn_allowed_to_think_sends_neither_thinking_nor_depth() -> None:
    """``extra={"thinking": None}`` (#3220) — дефолт модели; глубину тоже не навязываем."""
    provider, completions = _provider()

    _ask(provider, LLMSettings(extra={"thinking": None}), stream=False)

    assert "extra_body" not in completions.calls[0], completions.calls[0].get("extra_body")


def test_caller_depth_wins_over_the_lowest() -> None:
    provider, completions = _provider()

    _ask(provider, LLMSettings(extra={"reasoning_effort": "high"}), stream=False)

    assert completions.calls[0]["extra_body"] == {"reasoning_effort": "high"}


def test_model_chosen_per_call_decides_not_the_instance_default() -> None:
    """Экземпляр на Flash, вызов явно просит M3 — M3 получает прежнее ``disabled``."""
    provider, completions = _provider()

    _ask(provider, LLMSettings(model=M3), stream=False)

    assert completions.calls[0]["extra_body"] == {"thinking": DEFAULT_THINKING_POLICY}


def test_default_instance_asked_for_flash_per_call_gets_the_depth_not_disabled() -> None:
    """Обратное: экземпляр по умолчанию (M3), вызов явно просит Flash — API не получит ``disabled``."""
    provider, completions = _default_provider()

    text = _ask(provider, LLMSettings(model=FLASH), stream=False)

    assert text == "ок"
    assert completions.calls[0]["extra_body"] == {"reasoning_effort": LOWEST_REASONING_EFFORT}


def test_model_that_can_disable_thinking_keeps_the_old_request() -> None:
    provider, completions = _provider(model=M3)  # явная M3 — то же, что и по умолчанию

    _ask(provider, LLMSettings(max_tokens=3000), stream=False)

    assert completions.calls[0]["extra_body"] == {"thinking": {"type": "disabled"}}


# ── чистая функция ──────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("model", "extra", "expected"),
    [
        (FLASH, {"thinking": {"type": "disabled"}}, {"reasoning_effort": "low"}),
        (FLASH, {"thinking": {"type": "disabled"}, "top_k": 5}, {"top_k": 5, "reasoning_effort": "low"}),
        (FLASH, {"thinking": {"type": "adaptive"}}, {"thinking": {"type": "adaptive"}}),
        (FLASH, {}, {}),
        (M3, {"thinking": {"type": "disabled"}}, {"thinking": {"type": "disabled"}}),
        (None, {"thinking": {"type": "disabled"}}, {"thinking": {"type": "disabled"}}),
    ],
)
def test_apply_model_thinking(model: str | None, extra: dict[str, Any], expected: dict[str, Any]) -> None:
    before = dict(extra)

    assert apply_model_thinking(model, extra) == expected
    assert extra == before, "входной extra не должен меняться"


def test_model_always_thinks_is_an_explicit_list_not_a_substring() -> None:
    assert model_always_thinks(FLASH) and model_always_thinks("MiniMax-M3.1-Flash-Preview")
    assert not model_always_thinks(M3)
    assert not model_always_thinks("MiniMax-M3.1-Future")
    assert not model_always_thinks(None)


# ── image_input — по списку, проверенному живой пробой ──────────────────


@pytest.mark.parametrize("model", [FLASH, "MiniMax-M3.1-Flash-Preview", M3, "MiniMax-M2-vision", "minimax-vision-x"])
def test_models_probed_with_a_picture_accept_images(model: str) -> None:
    provider, _ = _provider()

    assert provider.capabilities_for(model).image_input is True


@pytest.mark.parametrize("model", ["MiniMax-M3-chat", "MiniMax-M3.1-Future", "MiniMax-M2.7", "some-other-model"])
def test_models_nobody_probed_do_not_get_image_input_from_a_name_match(model: str) -> None:
    """Раньше ``"minimax-m3" in name`` ручалось за любое ``MiniMax-M3*``."""
    provider, _ = _provider()

    assert provider.capabilities_for(model).image_input is False


def test_capabilities_without_a_model_follow_the_instance_default() -> None:
    assert _provider()[0].capabilities_for(None).image_input is True
    assert _provider(model="MiniMax-M2.7")[0].capabilities_for(None).image_input is False


# ── строка трассы называет глубину ──────────────────────────────────────


@pytest.mark.parametrize(
    ("extra_body", "label"),
    [
        ({"reasoning_effort": "low"}, "reasoning_effort=low"),
        ({"thinking": {"type": "disabled"}}, '{"type": "disabled"}'),
        ({"thinking": {"type": "adaptive"}, "reasoning_effort": "high"}, '{"type": "adaptive"} reasoning_effort=high'),
        ({}, "model-default"),
    ],
)
def test_request_trace_names_the_depth(extra_body: dict[str, Any], label: str) -> None:
    assert _thinking_label({"extra_body": extra_body}) == label
