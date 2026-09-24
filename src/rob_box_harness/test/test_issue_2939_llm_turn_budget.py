"""Issue #2939 — зависший запрос к MiniMax = 3 минуты тишины без фолбека.

Живой лог (E2E акт 2b, run 35948844238, n707)::

    06:00:31  LLM REQUEST START provider=minimax mode=stream
    06:02:01  openai._base_client - Retrying request to /chat/completions
    06:03:32  openai._base_client - Retrying request to /chat/completions
    06:03:35  Cancel: new STT input  (barge-in — единственное, что прервало ход)

Что доказано кодом + логом:

* интервал ~90 с — это ``timeout_s or 90.0`` из ``catalog.build_provider``
  (одно число на ВСЕ фазы httpx), а два «Retrying request» — скрытые
  ретраи самого SDK (``AsyncOpenAI`` по умолчанию ``max_retries=2``):
  3 попытки × 90 с = 270 с на один ход, фолбек-цепочка
  ``minimax → deepseek`` не получает исключения, пока SDK не сдастся;
* first-chunk-гуард #2718 (10 с) в этом ходе не сработал — иначе был бы
  ``stream.peek: no first byte`` через 10 с после старта.

Почему гуард не сработал — ГИПОТЕЗА (на роботе не воспроизведено):
``asyncio.wait_for`` после таймаута отменяет внутреннюю корутину и
ЖДЁТ, пока она признает отмену. Если отмена застряла в транспорте SDK
(робот: Python 3.10 + пакеты из apt), ``wait_for`` ждёт вместе с ней.
Тест ниже моделирует ровно это: внутренний ``__anext__`` проглатывает
первую отмену и продолжает «ретраить».
"""

from __future__ import annotations

import asyncio
import time
from typing import Any, AsyncIterator

import pytest

from rob_box_harness.health import HealthAwareFallbackLLM, HealthCache
from rob_box_harness.providers.catalog import build_provider
from rob_box_harness.providers.minimax import (
    HarnessMiniMaxProvider,
    RetryPolicy,
)
from rob_box_llm.errors import TimeoutError as LLMTimeoutError
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMProvider, ProviderCapabilities


def _msgs() -> list[LLMMessage]:
    return [LLMMessage(role="user", content="как меня зовут")]


class _StubbornInner:
    """Upstream-провайдер, чей первый ``__anext__`` не отпускает отмену.

    Моделирует SDK, который после нашей отмены продолжает ждать ответа
    сервера (скрытый ретрай): отмену проглатывает один раз и висит ещё
    ``linger_s`` секунд. ``linger_s=None`` — обычный зависший сервер,
    отмену признаёт сразу.
    """

    def __init__(self, linger_s: float | None) -> None:
        self.linger_s = linger_s
        self.calls = 0

    def capabilities_for(self, model: str | None) -> ProviderCapabilities:
        return ProviderCapabilities(text=True, streaming_text=True, tools=True, streaming_tools=True)

    @property
    def capabilities(self) -> ProviderCapabilities:
        return self.capabilities_for(None)

    def stream(self, messages: Any, *, tools: Any = (), settings: Any = None) -> AsyncIterator[LLMChunk]:
        self.calls += 1
        linger = self.linger_s

        async def _gen() -> AsyncIterator[LLMChunk]:
            try:
                await asyncio.sleep(3600)
            except asyncio.CancelledError:
                if linger is None:
                    raise
                # SDK «не заметил» отмену и ретраит дальше.
                await asyncio.sleep(linger)
            yield LLMChunk(content_delta="поздно", finish_reason="stop")

        return _gen()

    async def aclose(self) -> None:
        return None


def _hung_minimax(*, first_chunk_s: float, linger_s: float | None, attempts: int = 3) -> tuple[HarnessMiniMaxProvider, _StubbornInner]:
    p = HarnessMiniMaxProvider(
        api_key="sk-test",
        first_chunk_timeout_s=first_chunk_s,
        retry=RetryPolicy(max_attempts=attempts, backoff_base=0.01, backoff_jitter=0.0),
    )
    inner = _StubbornInner(linger_s)
    p._inner = inner  # type: ignore[assignment]
    return p, inner


class _Answering(LLMProvider):  # type: ignore[misc]
    """Живой запасной провайдер (deepseek в проде)."""

    name = "deepseek"

    @property
    def capabilities(self) -> ProviderCapabilities:
        return ProviderCapabilities(text=True, streaming_text=True, tools=True, streaming_tools=True)

    async def complete(self, messages: Any, *, tools: Any = (), settings: Any = None) -> Any:
        raise AssertionError("dialogue_node стримит")

    async def stream(self, messages: Any, *, tools: Any = (), settings: Any = None) -> AsyncIterator[LLMChunk]:
        yield LLMChunk(content_delta="Ты Саша.", finish_reason=None)
        yield LLMChunk(content_delta="", finish_reason="stop")

    async def aclose(self) -> None:
        return None


# ---------------------------------------------------------------------------
# 1. Клиент SDK: без скрытых ретраев, короткий connect
# ---------------------------------------------------------------------------


def test_catalog_minimax_client_has_no_hidden_sdk_retries(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ретраи — только harness-овые (видимые в логе и ограниченные).

    Скрытые ретраи SDK умножали 90-секундный таймаут на 3 и не давали
    фолбек-цепочке шанса: исключение приходило через 270 с.
    """
    monkeypatch.setenv("MINIMAX_API_KEY", "sk-test")
    provider = build_provider("minimax")
    client = provider._inner._client
    assert client.max_retries == 0, (
        f"AsyncOpenAI.max_retries={client.max_retries}: SDK молча ретраит "
        "зависший запрос (живой лог #2939: два «Retrying request» по 90 с)"
    )
    assert client.timeout.connect is not None and client.timeout.connect <= 5.0, (
        f"connect={client.timeout.connect}: мёртвый хост должен выясняться за секунды"
    )


# ---------------------------------------------------------------------------
# 2. First-chunk-гуард не ждёт, пока SDK признает отмену
# ---------------------------------------------------------------------------


def test_first_chunk_guard_fires_even_if_cancel_is_not_acknowledged() -> None:
    p, _inner = _hung_minimax(first_chunk_s=0.2, linger_s=3.0, attempts=1)

    async def _consume() -> None:
        async for _ in p.stream(_msgs()):
            pass

    t0 = time.monotonic()
    with pytest.raises(LLMTimeoutError):
        asyncio.run(_consume())
    elapsed = time.monotonic() - t0
    assert elapsed < 1.0, (
        f"гуард 0.2 с отработал за {elapsed:.2f} с — ждал, пока SDK признает "
        "отмену (живой #2939: гуард 10 с не сработал вовсе, ход висел 3 мин)"
    )


# ---------------------------------------------------------------------------
# 3. Не дождались первого чанка → сразу следующий провайдер цепочки
# ---------------------------------------------------------------------------


def test_hung_minimax_falls_back_to_next_provider_within_one_guard() -> None:
    minimax, inner = _hung_minimax(first_chunk_s=0.2, linger_s=None, attempts=3)
    chain = HealthAwareFallbackLLM([minimax, _Answering()], cache=HealthCache())

    async def _consume() -> str:
        parts = []
        async for chunk in chain.stream(_msgs()):
            parts.append(chunk.content_delta)
        return "".join(parts)

    t0 = time.monotonic()
    text = asyncio.run(_consume())
    elapsed = time.monotonic() - t0

    assert text == "Ты Саша."
    assert inner.calls == 1, (
        f"minimax попробован {inner.calls} раз: молчащий провайдер ретраить "
        "бессмысленно, бюджет хода уходит на ожидание вместо фолбека"
    )
    assert elapsed < 0.6, f"фолбек через {elapsed:.2f} с, ожидали ~один гуард (0.2 с)"
