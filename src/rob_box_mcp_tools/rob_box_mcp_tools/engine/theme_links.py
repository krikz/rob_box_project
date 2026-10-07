"""Часть темы, которую прямой поиск не нашёл → проверенные каталогом произведения (#3493; ADR-0148, ADR-0155 §3.4).

Общий путь вместо строк «слово → произведение» в коде («танец маленьких утят», «животные», «король лев»):

1. прямой поиск (``engine.search.theme_search``) — как был; нашёл всё — сюда не заходим;
2. часть без находок ищется в хранилище связей реестра (``rob_box_music.works.theme_links``, та же SQLite, что
   RTTTL-библиотека): семена, ручные связи Шифу и прежние проверенные ответы LLM — без похода в LLM;
3. связи нет — LLM (тот же провайдер и circuit breaker, что у ризонера сета) предлагает строки поиска по
   англоязычному архиву (``rob_box_music.theme_queries``); **каждую строку проверяет код** поиском по каталогу:
   берутся только записи, в опознавательных полях которых есть все слова строки; строка, совпавшая со слишком
   многими записями («dance», «love»), — не название, отбрасывается. Вердикт пишется в хранилище;
4. LLM недоступна или не уложилась в :data:`ASK_WAIT_S` — сет решает без неё (как до #3493), фраза — в журнал
   непонятого (``missed``); опоздавший ответ дописывается в хранилище в фоне — следующий заказ найдёт сразу.

Хуков LLM не выбирает и фраз не пишет; «нашлось/не нашлось» — из результата поиска по каталогу.
"""

from __future__ import annotations

import logging
import sqlite3
import threading
from contextlib import closing
from dataclasses import replace
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from rob_box_music import theme_queries as tq
from rob_box_music.rtttl import THEME_HOOKS
from rob_box_music.works import ThemeLink, get_theme_link, put_theme_link, theme_phrase

from ..core.rtttl_library import db_path_of
from .search import THEME_LIST_HOOKS, ThemeHits, _pool, by_part, ordered, round_robin, terms

_LOG = logging.getLogger(__name__)

#: Сколько старт сета ждёт LLM — только у фразы, которой нет в хранилище (повтор — без LLM, живьём 07.10 трек 1
#: через 3.4 с после STT). Замер на роботе 07.10 13:20 UTC (hotpatch): ответ на 3 части темы — 5.9 с, на одну —
#: 3.5 с (с проверкой каталогом); с дедлайном 3 с оба опоздали, сет отказал/сыграл пул. Ожидание 8 с — меньше
#: прежних 17 с до трека 1 (#3491); опоздавший ответ всё равно проверяется и кешируется.
ASK_WAIT_S = 8.0
#: Дедлайн самого вызова LLM в фоне (после :data:`ASK_WAIT_S` ответ уже идёт только в хранилище).
ASK_DEADLINE_S = 20.0
ASK_MAX_TOKENS = 600
#: Строка поиска, совпавшая с таким числом записей и больше, — не название произведения («dance» — 50, «love» — 41;
#: «mario» — 25, «harry potter» — 12 — названия).
GENERIC_MATCHES = 40
#: Записей на одну проверенную строку (версии одной мелодии — одна строка).
PER_QUERY = 4

#: ``(system, user, tool, deadline_s, max_tokens) -> (outcome, response, detail)`` — ``SetReasoner.ask``.
Ask = Callable[..., Tuple[str, Any, str]]


def verify(library: Any, query: str) -> Tuple[str, ...]:
    """Записи каталога по строке поиска: все слова строки в опознавательных полях записи (``found_min=1.0``);
    слишком общая строка (:data:`GENERIC_MATCHES`) — пусто."""
    query_terms = terms(library, query)
    pool = _pool(library, query, query_terms, 1.0) if query_terms else []
    return ordered(pool, query, PER_QUERY).names if 0 < len(pool) < GENERIC_MATCHES else ()


def _payload(response: Any) -> Any:
    if getattr(response, "truncated_tool_args", False):
        raise tq.QueriesInvalid("аргументы вызова обрезаны")
    for call in getattr(response, "tool_calls", ()) or ():
        if call.name == tq.SUBMIT_TOOL:
            return dict(call.arguments)
    raise tq.QueriesInvalid(f"нет вызова {tq.SUBMIT_TOOL}")


class ThemeLinks:
    """Расширение находок темы по хранилищу связей и проверенным предложениям LLM."""

    def __init__(self, db_path: Callable[[Any], Optional[str]] = db_path_of, *,
                 ask: Optional[Ask] = None, wait_s: float = ASK_WAIT_S, logger: Any = None,
                 now: Callable[[], datetime] = lambda: datetime.now(timezone.utc).replace(tzinfo=None),
                 spawn: Callable[[Callable[[], None]], None] = lambda fn: threading.Thread(
                     target=fn, name="rbx-theme-links", daemon=True).start()) -> None:
        self._db_path = db_path
        self._ask = ask
        self._wait_s = wait_s
        self._log = logger or _LOG
        self._now = now
        self._spawn = spawn
        self._lock = threading.Lock()  # одна запись в хранилище за раз (фоновый опоздавший ответ и старт сета)

    def expand(self, library: Any, theme: str, hits: ThemeHits) -> ThemeHits:
        """Находки темы с частями, найденными через хранилище связей или проверенные предложения LLM."""
        unresolved = list(hits.missing) or ([] if hits.names else [theme])
        path = self._db_path(library)
        if not theme.strip() or not unresolved or not path:
            return hits
        found: Dict[str, Tuple[str, ...]] = {}
        kinds: Dict[str, str] = {}
        ask_parts = []
        for part in unresolved:
            with self._lock, closing(sqlite3.connect(path)) as conn:
                link = get_theme_link(conn, part, self._now())
            if link is None:
                ask_parts.append(part)
                continue
            kinds[part] = link.kind
            found[part] = self._names(library, link.queries)
            self._log.info(f"🔎 [theme-links] «{part}»: связь {link.source}/{link.status} {list(link.queries)} → "
                           f"{list(found[part])}")
        if ask_parts:
            answered = self._ask_llm(library, path, theme, ask_parts)
            for part in ask_parts:
                if part in answered:
                    kinds[part], found[part] = answered[part]
        return self._merge(hits, unresolved, found, kinds)

    def _names(self, library: Any, queries: Sequence[str]) -> Tuple[str, ...]:
        return tuple(round_robin([verify(library, q) for q in queries], THEME_HOOKS))

    def _ask_llm(self, library: Any, path: str, theme: str,
                 parts: List[str]) -> Mapping[str, Tuple[str, Tuple[str, ...]]]:
        """Спросить LLM и проверить ответ каталогом; ждём не дольше ``wait_s``, опоздавший ответ — в хранилище."""
        if self._ask is None:
            self._note_missed(path, parts, "LLM выключена")
            return {}
        box: Dict[str, Any] = {}
        done = threading.Event()

        def run() -> None:
            try:
                box["answer"] = self._query(library, path, theme, parts)
            except Exception as exc:  # noqa: BLE001 — поиск темы не роняет сет
                box["answer"] = {}
                self._log.warning(f"⚠️ [theme-links] {parts}: {type(exc).__name__}: {exc}")
            done.set()

        self._spawn(run)
        if not done.wait(self._wait_s):
            self._log.warning(f"⚠️ [theme-links] {parts}: LLM не ответила за {self._wait_s:g} с — сет без неё, "
                              "ответ допишется в хранилище")
            self._note_missed(path, parts, "LLM опоздала")
            return {}
        return box["answer"]

    def _query(self, library: Any, path: str, theme: str,
               parts: List[str]) -> Mapping[str, Tuple[str, Tuple[str, ...]]]:
        system, user = tq.prompt(theme, parts)
        outcome, response, detail = self._ask(system, user, tq.tool(parts), ASK_DEADLINE_S, ASK_MAX_TOKENS)
        if outcome != "ok":
            self._note_missed(path, parts, f"LLM {outcome} {detail}".strip())
            return {}
        try:
            suggestions = tq.validate(_payload(response), parts)
        except tq.QueriesInvalid as exc:
            self._note_missed(path, parts, f"ответ LLM не по схеме: {exc}")
            return {}
        out = {}
        for s in suggestions:
            checked = {q: verify(library, q) for q in s.queries}
            ok = tuple(q for q, names in checked.items() if names)
            names = tuple(round_robin([checked[q] for q in ok], THEME_HOOKS))
            status = "not_theme" if s.kind == "not_theme" else "found" if ok else "not_found"
            link = ThemeLink(theme_phrase(s.part), status, "llm", s.kind, ok, names,
                             tuple(q for q in s.queries if q not in ok))
            with self._lock, closing(sqlite3.connect(path)) as conn:
                put_theme_link(conn, link, self._now())
            self._log.info(f"🔎 [theme-links] «{s.part}»: LLM {s.kind} {list(s.queries)} → каталог подтвердил "
                           f"{list(ok)} → {list(names)}; отброшено {list(link.rejected)}")
            out[s.part] = (s.kind, names)
        return out

    def _note_missed(self, path: str, parts: Sequence[str], why: str) -> None:
        with self._lock, closing(sqlite3.connect(path)) as conn:
            for part in parts:
                put_theme_link(conn, ThemeLink(theme_phrase(part) or part, "missed", "miss"), self._now())
        self._log.info(f"🔎 [theme-links] не понято {list(parts)} ({why}) — в журнал")

    @staticmethod
    def _merge(hits: ThemeHits, unresolved: List[str], found: Mapping[str, Tuple[str, ...]],
               kinds: Mapping[str, str]) -> ThemeHits:
        """Находки по частям в порядке названного; часть ``not_theme`` — не тема (не в «не найдено»)."""
        if not any(found.values()) and not kinds:
            return hits
        lists = list(hits.parts) or ([hits.names] if hits.names else [])
        lists += [found[p] for p in unresolved if found.get(p)]
        missing = tuple(p for p in unresolved if not found.get(p) and kinds.get(p) != "not_theme")
        names = tuple(round_robin(lists, THEME_LIST_HOOKS))
        named = hits.named or any(kinds.get(p) == "work" for p in missing)
        return replace(hits, names=names, missing=missing, parts=by_part(lists, names) if len(lists) > 1 else (),
                       named=named)


__all__ = ["ASK_DEADLINE_S", "ASK_WAIT_S", "GENERIC_MATCHES", "PER_QUERY", "ThemeLinks", "verify"]
