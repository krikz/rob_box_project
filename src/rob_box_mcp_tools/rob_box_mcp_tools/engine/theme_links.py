"""Часть темы, которую прямой поиск не нашёл → проверенные каталогом произведения (#3493; ADR-0148, ADR-0155 §3.4).

Общий путь вместо строк «слово → произведение» в коде («танец маленьких утят», «животные», «король лев»):

1. прямой поиск (``engine.search.theme_search``) — как был; нашёл всё — сюда не заходим;
2. часть без находок ищется в хранилище связей реестра (``rob_box_music.works.theme_links``, та же SQLite, что
   RTTTL-библиотека): семена, ручные связи Шифу и прежние проверенные ответы LLM — без похода в LLM;
3. связи нет — LLM (тот же провайдер и circuit breaker, что у ризонера сета) предлагает строки поиска по
   англоязычному архиву (``rob_box_music.theme_queries``); **каждую строку проверяет код** поиском по каталогу:
   берутся только записи, в опознавательных полях которых есть все слова строки; строка, совпавшая со слишком
   многими записями («dance», «love»), — не название, отбрасывается. Вердикт пишется в хранилище;
4. LLM недоступна или не уложилась в ``knowledge.THEME_ASK_WAIT_S`` — сет решает без неё (как до #3493), фраза — в
   журнал непонятого (``missed``); опоздавший ответ дописывается в хранилище в фоне — следующий заказ найдёт сразу.
   Одиночный трек (``knowledge.THEME_ASK_WAIT_TRACK_S`` = 0) не ждёт вовсе: LLM спрашивается только фоном.

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

from rob_box_music import knowledge as kn
from rob_box_music import theme_queries as tq
from rob_box_music.rtttl import THEME_HOOKS
from rob_box_music.works import ThemeLink, get_theme_link, put_theme_link, theme_phrase

from ..core.rtttl_library import db_path_of
from .search import THEME_LIST_HOOKS, ThemeHits, _pool, by_part, ordered, round_robin, terms

_LOG = logging.getLogger(__name__)

#: Дедлайн самого вызова LLM в фоне (после ожидания ``knowledge.THEME_ASK_WAIT_S`` ответ идёт только в хранилище).
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


def tool_payload(response: Any, name: str = tq.SUBMIT_TOOL) -> Any:
    """Аргументы вызова тула ``name`` из ответа ``Ask``; обрезаны или вызова нет — ``QueriesInvalid``."""
    if getattr(response, "truncated_tool_args", False):
        raise tq.QueriesInvalid("аргументы вызова обрезаны")
    for call in getattr(response, "tool_calls", ()) or ():
        if call.name == name:
            return dict(call.arguments)
    raise tq.QueriesInvalid(f"нет вызова {name}")


def in_time(spawn: Callable[[Callable[[], None]], None], wait_s: float, run: Callable[[], Any]) -> Tuple[bool, Any]:
    """``(успел, результат)``: ``run`` — в своём потоке (``spawn``), ждём не дольше ``wait_s``; опоздал — ``(False,
    None)``, а ``run`` доработает в фоне (опоздавший ответ LLM дописывается в хранилище/кэш сам)."""
    box: Dict[str, Any] = {}
    done = threading.Event()

    def task() -> None:
        try:
            box["value"] = run()
        finally:
            done.set()

    spawn(task)
    return (True, box.get("value")) if done.wait(wait_s) else (False, None)


class ThemeLinks:
    """Расширение находок темы по хранилищу связей и проверенным предложениям LLM."""

    def __init__(self, db_path: Callable[[Any], Optional[str]] = db_path_of, *,
                 ask: Optional[Ask] = None, wait_s: float = kn.THEME_ASK_WAIT_S, logger: Any = None,
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

    def expand(self, library: Any, theme: str, hits: ThemeHits, ask: bool = True,
               wait_s: Optional[float] = None) -> ThemeHits:
        """Находки темы с частями, найденными через хранилище связей или проверенные предложения LLM; ``ask=False`` —
        LLM не спрашивается и не ждётся (одиночная пьеса, #3608: до 8 с на пути звука), в журнал «не понято» не пишется.
        ``wait_s`` — сколько ждать LLM в этом вызове (нет — ``self._wait_s``); ``0`` — не ждать вовсе: LLM
        спрашивается фоном, ответ допишется в хранилище (одиночный трек, #3960 п.2)."""
        unresolved = list(hits.missing) or ([] if hits.names else [theme])
        path = self._db_path(library)
        if not theme.strip() or not unresolved or not path:
            return hits
        found: Dict[str, Tuple[str, ...]] = {}
        kinds: Dict[str, str] = {}
        queries: Dict[str, Tuple[str, ...]] = {}  # проверенные строки связи — ими же ищутся партитуры (#3512)
        ask_parts = []
        for part in unresolved:
            with self._lock, closing(sqlite3.connect(path)) as conn:
                link = get_theme_link(conn, part, self._now())
            if link is None:
                ask_parts.append(part)
                continue
            kinds[part] = link.kind
            queries[part] = link.queries
            found[part] = self._names(library, link.queries)
            self._log.info(f"🔎 [theme-links] «{part}»: связь {link.source}/{link.status} {list(link.queries)} → "
                           f"{list(found[part])}")
        if ask_parts and ask:
            answered = self._ask_llm(library, path, theme, ask_parts, wait_s)
            for part in ask_parts:
                if part in answered:
                    kinds[part], found[part], queries[part] = answered[part]
        merged = self._merge(hits, unresolved, found, kinds)
        if hits.query is not None and any(queries.values()):
            merged = replace(merged, query=hits.query.with_links(queries))
        return merged

    def _names(self, library: Any, queries: Sequence[str]) -> Tuple[str, ...]:
        return tuple(round_robin([verify(library, q) for q in queries], THEME_HOOKS))

    def _ask_llm(self, library: Any, path: str, theme: str, parts: List[str],
                 wait: Optional[float]) -> Mapping[str, Tuple[str, Tuple[str, ...], Tuple[str, ...]]]:
        """Спросить LLM и проверить ответ каталогом; ждём не дольше ``wait`` (нет — ``self._wait_s``), опоздавший ответ
        — в хранилище."""
        wait_s = self._wait_s if wait is None else wait
        if self._ask is None:
            self._note_missed(path, parts, "LLM выключена")
            return {}

        def run() -> Mapping[str, Tuple[str, Tuple[str, ...], Tuple[str, ...]]]:
            try:
                return self._query(library, path, theme, parts)
            except Exception as exc:  # noqa: BLE001 — поиск темы не роняет сет
                self._log.warning(f"⚠️ [theme-links] {parts}: {type(exc).__name__}: {exc}")
                return {}

        done, answer = in_time(self._spawn, wait_s, run)
        if not done and wait_s <= 0:  # фон: заказ не ждёт, вердикт (или «не понято») запишет сам вызов LLM
            self._log.info(f"🔎 [theme-links] {parts}: LLM спрошена в фоне — заказ её не ждёт, ответ "
                           "допишется в хранилище")
            return {}
        if not done:
            self._log.warning(f"⚠️ [theme-links] {parts}: LLM не ответила за {wait_s:g} с — сет без неё, "
                              "ответ допишется в хранилище")
            self._note_missed(path, parts, "LLM опоздала")
            return {}
        return answer

    def _query(self, library: Any, path: str, theme: str,
               parts: List[str]) -> Mapping[str, Tuple[str, Tuple[str, ...], Tuple[str, ...]]]:
        system, user = tq.prompt(theme, parts)
        outcome, response, detail = self._ask(system, user, tq.tool(parts), ASK_DEADLINE_S, ASK_MAX_TOKENS)
        if outcome != "ok":
            self._note_missed(path, parts, f"LLM {outcome} {detail}".strip())
            return {}
        try:
            suggestions = tq.validate(tool_payload(response), parts)
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
            out[s.part] = (s.kind, names, ok)
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


__all__ = ["ASK_DEADLINE_S", "GENERIC_MATCHES", "PER_QUERY", "Ask", "ThemeLinks", "in_time",
           "tool_payload", "verify"]
