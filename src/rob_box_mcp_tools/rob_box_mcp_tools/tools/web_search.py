#!/usr/bin/env python3
"""web_search.py — MCP tool ``search_web``: поиск в интернете для свежих данных (issues #1101, #4026).

LLM получает фактуру, которой нет в её обучающей выборке: погода, новости, цены, спорт, местные факты.

Бэкенды по порядку (ADR-0148: какой отвечает и что значит ответ — решает код, а не модель):

1. ``ddgs`` — DuckDuckGo. Ответом считаются только непустые строки. Пустая выдача (``DDGSException("No results found.")``
   или ``[]``) — не вердикт: ddgs бросает то же самое, когда запрос ничего не нашёл и когда источник заблокировал
   запрос или отдал анти-бот страницу с пустым разбором (n907/n908 23.10). Поэтому пусто и любой другой сбой (сеть,
   rate-limit, пакет не установлен) одинаково отправляют дальше, к DeepSeek.
2. ``deepseek`` — ``POST https://api.deepseek.com/v1/tools/search`` (``Authorization: Bearer $DEEPSEEK_API_KEY``,
   тело ``{query, count}``, ответ ``results[]`` с ``title / url / snippet / summary``). Модели отдаётся ``summary``
   (он написан для модели), ``snippet`` — запасной. Нет ключа — ступень пропускается со строкой в логе. Только
   ответ этого бэкенда - 200 с пустым ``results[]`` - честное «0 результатов» (пустые выдачи не тарифицируются).
3. Подтвердить не удалось (DeepSeek недоступен, а ddgs промолчал или сломался) — ``success=False``,
   ``data.status="rejected"``, ``data.reason="search_unavailable"``: «пусто у одного источника» не выдаётся
   LLM за «в сети нет» (n907/n908: 4 вызова LLM впустую и «поиск ничего не выдал»).

Какой бэкенд ответил — в логе (``backend=``) и в ``data.backend``.

Тул синхронный: HTTP внутри ``execute()``, без ROS-топиков.

Usage (from LLM tool-call):
    search_web(query="погода в Батайске сегодня", max_results=3)
    → {"query": "...", "results": [{"title":"...","body":"...","url":"..."}], "backend": "ddgs"}
"""

from __future__ import annotations

import json
import os
from typing import TYPE_CHECKING, Any, List, Mapping, Optional, Tuple

from ..base import MCPTool, MCPToolParameter, MCPToolResult

if TYPE_CHECKING:
    from ..engine.gen.kie_suno import Transport

#: Имена бэкендов в логе и в ``data.backend``.
BACKEND_DDGS = "ddgs"
BACKEND_DEEPSEEK = "deepseek"
#: Причина отказа, когда не ответил ни один бэкенд (поле ``reason`` результата: по нему диалог выбирает фразу).
SEARCH_UNAVAILABLE = "search_unavailable"
DEEPSEEK_SEARCH_URL = "https://api.deepseek.com/v1/tools/search"
DEEPSEEK_KEY_ENV = "DEEPSEEK_API_KEY"
#: API: запрос 1-128 символов. Тул отдаёт ответ в пределах дедлайна класса ``query`` (15 с, ``rob_box_dialog.knowledge.
#: ACTION_CLASSES``): DDGS ждёт до ``DDGS_TIMEOUT_S``, поэтому DeepSeek (замер 26.10: 0,7 с) получает не больше 4 с;
#: сумма держит запас в 1 с (тест фиксирует её числами - диалог отсюда не импортируется).
DEEPSEEK_QUERY_MAX = 128
DDGS_TIMEOUT_S = 10
DEEPSEEK_TIMEOUT_S = 4.0
#: Честная причина по HTTP-коду DeepSeek (остальные - просто код).
_DEEPSEEK_HTTP_WHY = {401: "ключ отвергнут", 402: "на балансе пусто", 429: "лимит запросов"}


class _BackendDown(Exception):
    """Бэкенд не дал ответа (сеть, лимит, нет ключа, не установлен, пустая выдача ddgs) - это не «ничего не найдено»."""


class SearchWebTool(MCPTool):
    """Поиск в интернете: DuckDuckGo, запасной - DeepSeek (``/v1/tools/search``)."""

    _MAX_RESULTS: int = 10
    _SNIPPET_CHARS: int = 280
    #: ``summary`` DeepSeek написан для модели и длиннее сниппета DDG; обрезка - потолок на токены.
    _SUMMARY_CHARS: int = 500

    def __init__(
        self,
        node,
        *,
        transport: Optional["Transport"] = None,
        environ: Optional[Mapping[str, str]] = None,
    ) -> None:
        super().__init__(node)
        self._ddgs_available: bool = False  # True after first successful import
        self._ddgs_cls: type | None = None  # cached DDGS class reference
        self._transport = transport  # HTTP DeepSeek; ``None`` - боевой urllib (см. _http), тесты подставляют свой
        self._environ = os.environ if environ is None else environ
        self.log_info("🔍 SearchWebTool registered (ddgs import deferred)")

    # ── Tool metadata ────────────────────────────────────────────────────

    @property
    def name(self) -> str:
        return "search_web"

    @property
    def description(self) -> str:
        return (
            "Поиск в интернете через DuckDuckGo. Возвращает до N сниппетов "
            "(title, text, url). Используй ОБЯЗАТЕЛЬНО для любых вопросов, "
            "требующих свежих данных, которых нет в твоей тренировочной "
            "выборке: "
            "погода («погода в Батайске сегодня»), "
            "новости («что случилось в мире сегодня»), "
            "курсы и цены («курс доллара сейчас», «сколько стоит iPhone 16»), "
            "спорт («счёт матча Спартак-Зенит»), "
            "локальная информация («работает ли метро в Москве»), "
            "факты и даты («когда день города в Ростове»). "
            # issue #1810: the old wording («не для музыкального ресёрча») pushed the
            # music skill away from the one tool that can find a melody it does
            # not know, so it substituted a generic scale instead. Notes and
            # artist style are web search; samples are picked by the arranger.
            "ТАКЖЕ для музыки: ноты незнакомой мелодии («<название> ноты», "
            "«<name> melody notes MIDI») и стиль/темп артиста или группы. "
            "НЕ используй для: личных фактов о собеседнике "
            "(memory_search/memory_context)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query",
                type="string",
                description=(
                    "Поисковый запрос на русском или английском. "
                    "Будь конкретным: укажи город, дату, тему."
                ),
                required=True,
            ),
            MCPToolParameter(
                name="max_results",
                type="integer",
                description="Сколько результатов вернуть (1-10, по умолчанию 5).",
                required=False,
            ),
        ]

    @property
    def execution_type(self):
        from ..base import ToolExecutionType

        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    # ── DuckDuckGo ───────────────────────────────────────────────────────

    def _ddgs_text(self, DDGS_cls, query: str, max_results: int) -> list:
        """Сырая выдача DDGS; исключения библиотеки пробрасываются как есть.

        ``DDGSException("No results found.")`` тут НЕ превращается в ``[]`` (так было до #4026): ddgs бросает его и
        когда запрос ничего не нашёл, и когда ни один движок не вернул строк из-за блока или анти-бот страницы без
        ошибки разбора. Отличить одно от другого изнутри нельзя, поэтому вердикт «ничего нет» ddgs не выносит.
        """
        with DDGS_cls(timeout=DDGS_TIMEOUT_S) as ddgs:
            return list(
                ddgs.text(
                    query,
                    max_results=max_results,
                    region="wt-wt",
                    backend="duckduckgo",
                )
            )

    def _ddgs_class(self):
        """Класс DDGS или ``None``, если пакета нет.

        Issue #1101 — lazy ddgs import with self-healing. Module-level import
        sometimes fails at mcp_server startup (race with pip install, stale
        .pyc, or sys.path ordering). Retry on EVERY call until successful —
        Python caches successful imports internally, so the retry cost is
        negligible (dict lookup). Once imported, reuse the cached class.
        """
        if self._ddgs_available is True:
            return self._ddgs_cls
        for pkg in ("ddgs", "duckduckgo_search"):
            try:
                mod = __import__(pkg, fromlist=["DDGS"])
            except ImportError:
                continue
            self._ddgs_cls = mod.DDGS
            self._ddgs_available = True
            return self._ddgs_cls
        self.log_error(
            "[search_web] ddgs import FAILED: "
            "both 'ddgs' and 'duckduckgo_search' unavailable"
        )
        return None

    def _search_ddgs(self, query: str, max_results: int) -> List[dict]:
        ddgs_cls = self._ddgs_class()
        if ddgs_cls is None:
            raise _BackendDown("пакет ddgs не установлен")
        try:
            raw = self._ddgs_text(ddgs_cls, query, max_results)
        except Exception as exc:  # noqa: BLE001 — сеть, rate-limit, смена разметки, «No results found»: вердикта нет
            raise _BackendDown(f"{type(exc).__name__}: {str(exc)[:200]}") from exc
        rows = [
            row for row in (
                _row(r.get("title"), r.get("body"), r.get("href"), self._SNIPPET_CHARS) for r in raw
            ) if row
        ]
        if not rows:
            raise _BackendDown("выдача пуста — от блока источника не отличить, вердикта нет")
        return rows

    # ── DeepSeek ─────────────────────────────────────────────────────────

    def _http(self) -> "Transport":
        """Транспорт HTTP. Боевой - тот же ``urllib_transport``, что у клиента KIE (одна реализация): импорт ленивый,
        чтобы ``tools/__init__`` не тянул ``rob_box_music`` на импорте (``music.py`` там под ``try`` по той же причине)."""
        if self._transport is None:
            from ..engine.gen.kie_suno import urllib_transport

            self._transport = urllib_transport
        return self._transport

    def _search_deepseek(self, query: str, max_results: int) -> List[dict]:
        key = str(self._environ.get(DEEPSEEK_KEY_ENV) or "").strip()
        if not key:
            raise _BackendDown(f"нет ключа {DEEPSEEK_KEY_ENV} — ступень пропущена")
        body = json.dumps({"query": query[:DEEPSEEK_QUERY_MAX], "count": max_results}, ensure_ascii=False)
        headers = {"Authorization": f"Bearer {key}", "Content-Type": "application/json"}
        try:
            status, raw = self._http()("POST", DEEPSEEK_SEARCH_URL, headers, body.encode("utf-8"), DEEPSEEK_TIMEOUT_S)
        except Exception as exc:  # noqa: BLE001 — сеть: бэкенд не ответил
            raise _BackendDown(f"{type(exc).__name__}: {str(exc)[:200]}") from exc
        if status != 200:
            why = _DEEPSEEK_HTTP_WHY.get(status)
            raise _BackendDown(f"HTTP {status}" + (f" ({why})" if why else ""))
        results = _deepseek_results(raw)
        return [
            row for row in (
                _row(r.get("title"), r.get("summary") or r.get("snippet"), r.get("url"), self._SUMMARY_CHARS,
                     date=r.get("date"))
                for r in results if isinstance(r, dict)
            ) if row
        ]

    # ── Execution ────────────────────────────────────────────────────────

    def execute(
        self,
        query: str,
        max_results: int = 5,
    ) -> MCPToolResult:
        query = (query or "").strip()
        if not query:
            return MCPToolResult(
                success=False,
                error="empty_query",
                message="Пустой поисковый запрос — уточни, что искать.",
            )
        max_results = max(1, min(int(max_results or 5), self._MAX_RESULTS))

        failures: List[Tuple[str, str]] = []
        for backend, search in ((BACKEND_DDGS, self._search_ddgs), (BACKEND_DEEPSEEK, self._search_deepseek)):
            try:
                rows = search(query, max_results)
            except _BackendDown as down:
                failures.append((backend, str(down)))
                self.log_warning(f"[search_web] query={query[:60]!r} backend={backend} не дал ответа: {down}")
                continue
            return self._answer(query, backend, rows, tried=[name for name, _ in failures] + [backend])
        return self._unavailable(query, failures)

    def _answer(self, query: str, backend: str, rows: List[dict], tried: List[str]) -> MCPToolResult:
        """Бэкенд ответил: пустая выдача (до сюда доходит только пустой ответ DeepSeek) - честное «ничего не нашёл
        по этой строке», иначе строки для LLM."""
        payload = {"query": query, "results": rows, "backend": backend}
        if not rows:
            self.log_info(f"[search_web] query={query[:60]!r} → 0 results backend={backend} tried={tried}")
            return MCPToolResult(
                success=True,
                data={**payload, "tried": tried},
                # 07.10: пустая выдача по одной строке («"танец маленьких утят" ноты RTTTL OR melody») стала
                # у робота «в сети нет». Код называет, что именно искали.
                message=(
                    f"Поиск ({backend}) по строке {query!r} вернул 0 результатов — "
                    "это про эту строку, а не про то, что такого нет в сети. "
                    "Попробуй переформулировать или уточнить."
                ),
            )
        self.log_info(
            f"[search_web] query={query[:60]!r} → {len(rows)} results backend={backend}"
        )
        return MCPToolResult(
            success=True,
            data=payload,
            message=json.dumps(payload, ensure_ascii=False),
        )

    def _unavailable(self, query: str, failures: List[Tuple[str, str]]) -> MCPToolResult:
        """Не ответил ни один бэкенд: отказ с причиной, а не «0 результатов» (ADR-0148, #4026)."""
        detail = "; ".join(f"{backend}: {why}" for backend, why in failures)
        self.log_error(f"[search_web] query={query[:60]!r} {SEARCH_UNAVAILABLE}: {detail}")
        return MCPToolResult(
            success=False,
            error=f"{SEARCH_UNAVAILABLE}: {detail}",
            message=(
                "Поиск в интернете не дал надёжного ответа: источники недоступны или промолчали, подтвердить, что "
                "ничего нет, не удалось — это сбой, а не «ничего не найдено»."
            ),
            data={
                "status": "rejected",
                "reason": SEARCH_UNAVAILABLE,
                "query": query,
                "results": [],
                "tried": [backend for backend, _ in failures],
            },
        )


def _row(title: Any, text: Any, url: Any, limit: int, date: Any = None) -> Optional[dict]:
    """Строка выдачи для LLM: ``{title, body, url[, date]}``; без текста строки нет."""
    body = str(text or "").strip()
    if not body:
        return None
    if len(body) > limit:
        body = body[:limit].rstrip() + "..."
    row = {"title": str(title or "").strip(), "body": body, "url": str(url or "").strip()}
    if date:
        row["date"] = str(date)
    return row


def _deepseek_results(raw: bytes) -> list:
    """``results[]`` ответа DeepSeek; тело не JSON или без ``results`` - бэкенд не ответил."""
    try:
        results = json.loads(raw).get("results")
    except (ValueError, AttributeError) as exc:
        raise _BackendDown(f"ответ не разобрать: {type(exc).__name__}") from exc
    if not isinstance(results, list):
        raise _BackendDown("в ответе нет results[]")
    return results
