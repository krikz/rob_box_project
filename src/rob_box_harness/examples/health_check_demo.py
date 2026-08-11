"""Demo: LLM health-check перед первым запросом (issue #1082).

Воспроизводит acceptance-сценарий из issue #1082:

* MiniMax с исчерпанной квотой (429 Token Plan usage limit 2056) —
  провайдер помечается ``unavailable`` и ПЕРВЫЙ же запрос идёт на
  deepseek (без retry-цикла, без 15-19с потерь).
* DeepSeek balance API (/user/balance) с ФЕЙКОВЫМ балансом через
  mock-транспорт httpx — health-check срабатывает ДО первого запроса.
* TTL-кэш: повторные запросы не дёргают balance API.
* После TTL expiry провайдер снова первый в цепочке (квота могла
  пополниться).

Запуск (raw-вывод для доказательств в PR):

    cd src/rob_box_harness
    python -m examples.health_check_demo
    # или
    python examples/health_check_demo.py

Никаких реальных API-ключей не нужно: провайдеры заменены фейковыми
двойниками, HTTP-транспорт — ``httpx.MockTransport``.
"""

from __future__ import annotations

import asyncio
import logging
import sys
import time
from pathlib import Path
from typing import Any, AsyncIterator, Iterable, Mapping

import httpx

# Репозиторий-корень (src/) на path — чтобы работал запуск из любого каталога.
_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from rob_box_harness.health import (  # noqa: E402
    DEFAULT_HEALTH_TTL_S,
    TRANSIENT_TTL_S,
    HealthAwareFallbackLLM,
    HealthCache,
    ProviderStatus,
    check_deepseek_balance,
    is_quota_exhausted,
)
from rob_box_llm.errors import RateLimitError  # noqa: E402
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings  # noqa: E402

logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s %(name)s: %(message)s",
    stream=sys.stdout,
)
log = logging.getLogger("demo.health")


# ---------------------------------------------------------------------------
# Фейковые провайдеры (LLMProvider-контракт, счётчики вызовов)
# ---------------------------------------------------------------------------


class FakeProvider:
    """LLMProvider-двойник: считает вызовы, умеет падать с заданной ошибкой."""

    def __init__(self, name: str, fail: BaseException | None = None) -> None:
        self.name = name
        self.fail = fail
        self.calls = 0
        self.closed = False

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        self.calls += 1
        if self.fail is not None:
            raise self.fail
        return LLMResponse(content=f"ответ от {self.name}")

    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> AsyncIterator[LLMChunk]:
        self.stream_calls = getattr(self, "stream_calls", 0) + 1
        if self.fail is not None:
            raise self.fail
        yield LLMChunk(content_delta=f"ответ от {self.name}", finish_reason="stop")

    async def aclose(self) -> None:
        self.closed = True


def msg(text: str = "сыграй Моцарта") -> list[LLMMessage]:
    return [LLMMessage(role="user", content=text)]


# ---------------------------------------------------------------------------
# Фейковый balance API DeepSeek (httpx.MockTransport)
# ---------------------------------------------------------------------------


def fake_balance_transport(balance: float, *, fail: bool = False):
    """httpx-транспорт, отвечающий как DeepSeek GET /user/balance."""

    def handler(request: httpx.Request) -> httpx.Response:
        assert request.url.path.endswith("/user/balance"), request.url
        if fail:
            raise httpx.ConnectError("connection refused (api down)", request=request)
        return httpx.Response(
            200,
            json={
                "is_available": True,
                "balance_infos": [
                    {"currency": "CNY", "total_balance": f"{balance:.2f}"},
                ],
            },
            request=request,
        )

    return httpx.AsyncClient(transport=httpx.MockTransport(handler))


def make_deepseek_balance_checker(balance: float, *, fail: bool = False):
    """Возвращает checker для balance_checkers, использующий mock-транспорт."""
    import rob_box_harness.health as health_mod

    async def checker() -> float | None:
        client = fake_balance_transport(balance, fail=fail)
        try:
            # Мокаем httpx.AsyncClient в модуле health на время вызова.
            original = health_mod.httpx.AsyncClient
            health_mod.httpx.AsyncClient = lambda *a, **k: client
            try:
                return await check_deepseek_balance(
                    "https://api.deepseek.com", "sk-fake-key"
                )
            finally:
                health_mod.httpx.AsyncClient = original
        finally:
            await client.aclose()

    return checker


# ---------------------------------------------------------------------------
# Демо-сценарии
# ---------------------------------------------------------------------------


async def demo_dead_minimax_first_request_goes_to_deepseek() -> None:
    """Acceptance: мёртвый MiniMax (quota 2056) — первый запрос на deepseek.

    До фичи: minimax retry 1/3 → 2/3 (RateLimitError, ~15-19с потерь) →
    потом fallback. После фичи: единственная попытка minimax, сразу
    помечается unavailable, следующий запрос вообще его не трогает.
    """
    print("\n" + "=" * 78)
    print("СЦЕНАРИЙ 1: MiniMax мёртв (429 Token Plan usage limit 2056)")
    print("=" * 78)

    quota_error = RateLimitError(
        "429: rate_limit_error Token Plan usage limit reached (2056)",
        provider="minimax",
    )
    minimax = FakeProvider("minimax", fail=quota_error)
    deepseek = FakeProvider("deepseek")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM([minimax, deepseek], cache=cache)

    t0 = time.monotonic()
    response = await wrapper.complete(msg())
    dt = time.monotonic() - t0

    print(f"\n--- результат ---")
    print(f"первый LLM-запрос ответил: {response.content!r}")
    print(f"вызовов minimax:           {minimax.calls}  (было бы 2-3 retry до фичи)")
    print(f"вызовов deepseek:          {deepseek.calls}")
    print(f"время ответа:              {dt*1000:.0f} мс (до фичи: ~15-19 с)")
    print(f"cache[minimax]:            {cache.status('minimax').value} "
          f"(reason={cache.get('minimax').reason!r})")
    assert response.content == "ответ от deepseek"
    assert minimax.calls == 1, "ровно одна попытка, без retry-цикла"
    assert cache.is_unavailable("minimax")

    # Второй запрос — minimax вообще не трогается (TTL свежий).
    t0 = time.monotonic()
    response2 = await wrapper.complete(msg("сыграй ещё"))
    dt2 = time.monotonic() - t0
    print(f"\nвторой запрос (в пределах TTL): {response2.content!r}")
    print(f"вызовов minimax теперь:    {minimax.calls}  (не увеличился!)")
    print(f"время ответа:              {dt2*1000:.0f} мс")
    assert minimax.calls == 1

    print("\n✅ Сценарий 1 пройден: первый же запрос ушёл на deepseek, "
          "минимумакс не ретраился.")


async def demo_fake_balance_zero_blocks_deepseek_before_first_call() -> None:
    """Acceptance: health-check с фейковым балансом (mock API).

    DeepSeek balance=0 → провайдер помечается unavailable ДО первого
    запроса; первый запрос идёт на MiniMax.
    """
    print("\n" + "=" * 78)
    print("СЦЕНАРИЙ 2: фейковый баланс DeepSeek = 0 → блок ДО первого запроса")
    print("=" * 78)

    deepseek = FakeProvider("deepseek")
    minimax = FakeProvider("minimax")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM(
        [deepseek, minimax],
        cache=cache,
        balance_checkers={"deepseek": make_deepseek_balance_checker(0.0)},
    )

    response = await wrapper.complete(msg())

    print(f"\n--- результат ---")
    print(f"первый LLM-запрос ответил: {response.content!r}")
    print(f"вызовов deepseek:          {deepseek.calls}  (0 = заблокирован ДО запроса)")
    print(f"вызовов minimax:           {minimax.calls}")
    print(f"cache[deepseek]:           {cache.status('deepseek').value} "
          f"(reason={cache.get('deepseek').reason!r})")
    assert deepseek.calls == 0
    assert cache.is_unavailable("deepseek")

    print("\n✅ Сценарий 2 пройден: balance=0 распознан health-check'ом, "
          "запрос на мёртвого не ушёл.")


async def demo_fake_balance_healthy_and_cached() -> None:
    """Фейковый баланс > 0 → провайдер healthy, balance API дёргается 1 раз."""
    print("\n" + "=" * 78)
    print("СЦЕНАРИЙ 3: фейковый баланс DeepSeek = 110 → healthy, TTL-кэш")
    print("=" * 78)

    deepseek = FakeProvider("deepseek")
    minimax = FakeProvider("minimax")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM(
        [deepseek, minimax],
        cache=cache,
        balance_checkers={"deepseek": make_deepseek_balance_checker(110.0)},
    )

    for i in range(3):
        response = await wrapper.complete(msg(f"запрос {i+1}"))
        print(f"запрос {i+1}: {response.content!r}")

    print(f"cache[deepseek]: {cache.status('deepseek').value} "
          f"(balance={cache.get('deepseek').balance})")
    assert cache.status("deepseek") == ProviderStatus.HEALTHY
    # balance API дёргался один раз: первый запрос → HEALTHY, дальше TTL-кэш.
    print("\n✅ Сценарий 3 пройден: balance>0 → healthy; TTL-кэш не даёт "
          "дёргать API на каждый запрос.")


async def demo_ttl_expiry_puts_provider_back_first() -> None:
    """Acceptance: после TTL expiry MiniMax снова первый в цепочке."""
    print("\n" + "=" * 78)
    print("СЦЕНАРИЙ 4: TTL expiry → провайдер снова проверяется")
    print("=" * 78)

    class Clock:
        def __init__(self) -> None:
            self.now = 1000.0

        def __call__(self) -> float:
            return self.now

    clock = Clock()
    minimax = FakeProvider("minimax")
    deepseek = FakeProvider("deepseek")
    cache = HealthCache(clock=clock)
    cache.mark_unavailable("minimax", reason="quota 2056")

    wrapper = HealthAwareFallbackLLM([minimax, deepseek], cache=cache)

    r1 = await wrapper.complete(msg("в пределах TTL"))
    print(f"в пределах TTL:   {r1.content!r}  (minimax.calls={minimax.calls})")
    assert minimax.calls == 0

    clock.now += DEFAULT_HEALTH_TTL_S + 1  # TTL истёк
    r2 = await wrapper.complete(msg("после TTL"))
    print(f"после TTL expiry: {r2.content!r}  (minimax.calls={minimax.calls})")
    assert minimax.calls == 1, "после expiry minimax снова первый"

    print("\n✅ Сценарий 4 пройден: TTL expiry → повторная проверка, "
          "quota могла пополниться.")


async def demo_balance_api_down_keeps_provider_healthy() -> None:
    """Edge case: balance API упал → провайдер НЕ блокируется."""
    print("\n" + "=" * 78)
    print("СЦЕНАРИЙ 5: balance API недоступен → провайдер считается рабочим")
    print("=" * 78)

    deepseek = FakeProvider("deepseek")
    minimax = FakeProvider("minimax")
    cache = HealthCache()
    wrapper = HealthAwareFallbackLLM(
        [deepseek, minimax],
        cache=cache,
        balance_checkers={"deepseek": make_deepseek_balance_checker(0.0, fail=True)},
    )

    response = await wrapper.complete(msg())
    print(f"первый запрос ответил: {response.content!r}")
    print(f"вызовов deepseek:      {deepseek.calls}  (не заблокирован)")
    print(f"cache[deepseek]:       {cache.status('deepseek').value}")
    assert deepseek.calls == 1
    assert not cache.is_unavailable("deepseek")

    print("\n✅ Сценарий 5 пройден: сломанный health-check не блокирует "
          "провайдера (edge case #1082).")


async def main() -> None:
    print("rob_box_harness.health — демо (issue #1082)")
    print(f"  DEFAULT_HEALTH_TTL_S={DEFAULT_HEALTH_TTL_S}, TRANSIENT_TTL_S={TRANSIENT_TTL_S}")
    print(f"  QUOTA_EXHAUSTED_HINTS={tuple(h for h in __import__('rob_box_harness.health', fromlist=['QUOTA_EXHAUSTED_HINTS']).QUOTA_EXHAUSTED_HINTS)}")
    print(f"  is_quota_exhausted('429 ... 2056') = {is_quota_exhausted(RateLimitError('429: rate_limit_error Token Plan usage limit reached (2056)'))}")

    await demo_dead_minimax_first_request_goes_to_deepseek()
    await demo_fake_balance_zero_blocks_deepseek_before_first_call()
    await demo_fake_balance_healthy_and_cached()
    await demo_ttl_expiry_puts_provider_back_first()
    await demo_balance_api_down_keeps_provider_healthy()

    print("\n" + "=" * 78)
    print("ВСЕ ДЕМО-СЦЕНАРИИ ПРОЙДЕНЫ ✅")
    print("=" * 78)


if __name__ == "__main__":
    asyncio.run(main())
