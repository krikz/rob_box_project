"""Фраза заказа одиночной пьесы → слоты, проверенные каталогом (#3609; ADR-0157 §2.1, ADR-0148).

Тот же путь, что у тем сетов (``engine.theme_links``): прямой разбор кодом → только непонятое — LLM по узкой схеме
(``rob_box_music.request_slots``) → **каждое поле проверяет код каталогом** → проверенное кешируется.

1. **Код первым.** Название берёт грамматика заказа (``MediaCommand``: инструмент, стиль, настроение — её слоты),
   слова названия — словарь поиска (``search.terms``: семена реестра, алиасы, слова архива RTTTL, исправление
   безударной гласной #3602). Все слова известны — LLM не зовётся (``source=code``): «сыграй к Элизе».
2. **Непонятые слова** (нет написания в архиве: «полет», «ебанем», «утят») — в строку поиска не идут (на них
   ``theme_search`` тратил 1.8 с на сегментацию, #3608); есть кэш разборов этой фразы — он (``source=cache``);
   иначе LLM, ждём не дольше :data:`knowledge.REQUEST_SLOTS_WAIT_MS` (провайдер — LLM-ступени выбора, ``llm_harness``).
3. **Проверка каталогом**: композитор — поле «композитор» индекса партитур или ``artist`` записей RTTTL; произведение
   — записи RTTTL со всеми словами названия (``theme_links.verify``) или партитуры с полным совпадением названия;
   инструмент, стиль, настроение, режим — перечни ``knowledge`` (схема). Не подтвердилось — поле пустое, в лог
   «отброшено». Подтверждённое произведение играется само: свежесть его не вытесняет (``piece.levels``).
4. LLM не ответила вовремя — код-разбор (известные слова), честная строка лога; опоздавший ответ проверяется и
   ложится в кэш в фоне — следующий такой заказ без LLM.
5. **На слух** (#4016, живой прогон 24.10: «раннинг ап дэд хилл» → «хилл» → «King Of The Hill»): непонятых слов
   названия больше понятых — до LLM код ищет запись архива, чьё название звучит этими словами (``search.heard_title``,
   ``source=sound``): «Running Up That Hill». Не нашлось, а LLM не подтвердила ни произведение, ни автора — заказ не
   урезается молча до понятых слов: ``RequestSlots.doubt`` и фраза кода :func:`doubt_text` («не уверен, что это за
   песня — «…»?»).

Одна строка лога на заказ: ``[request-slots] «…» source=code|llm|cache|sound ms=… <слоты> → «запрос» (или
``не уверен: «…»``); llm=a/n cache=b p50/p95``. Решение «что играть» — дальше, у ``PieceChain`` (кандидаты по
``RequestSlots.query``).
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from collections import OrderedDict
from dataclasses import replace
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from rob_box_harness.decision.health import CircuitBreaker, DecisionMetrics
from rob_box_music import knowledge as kn
from rob_box_music import request_slots as rs
from rob_box_music.theme_queries import QueriesInvalid
from rob_box_music.works import WorkRef, WorkWord, theme_phrase

from .piece import work_word
from .search import Heard, Term, heard_title, terms
from .theme_links import Ask, in_time, tool_payload, verify

_LOG = logging.getLogger(__name__)
#: Слова фразы: ``текст -> Term`` (``search.terms`` по словарю архива).
Terms = Callable[[str], Sequence[Term]]
#: Фраз в кэше разборов (в памяти процесса; после рестарта пуст).
CACHE_SIZE = 256
ASK_MAX_TOKENS = 200
BREAKER_FAILURES = 3
BREAKER_RESET_S = 300.0
#: Слово имени композитора короче — не фамилия («j», «s», «von»).
_NAME_MIN = 4


def unknown(words: Sequence[Term]) -> List[Term]:
    """Слова, которых каталог не знает: у русского слова нет написания в архиве (латиница — сама себе написание)."""
    return [t for t in words if not t.alts]


class Catalog:
    """Проверка слотов LLM каталогом: архив RTTTL (``library``) и индекс партитур (``scores``)."""

    def __init__(self, library: Callable[[], Any], scores: Any) -> None:
        self._library = library
        self._scores = scores

    def composer(self, name: str) -> str:
        """Фамилия композитора, если каталог её знает (поле «композитор» партитур или ``artist`` RTTTL), иначе «»."""
        for word in reversed([w for w in name.lower().replace(".", " ").split() if len(w) >= _NAME_MIN]):
            spelled = WorkWord((word,), (word,))
            if (self._scores is not None and self._scores.composer_role(spelled)) or any(
                    spelled.named_in(r.get("artist")) for r in self._library().search(word, limit=50,
                                                                                      include_rtttl=True)):
                return word.capitalize()
        return ""

    def heard(self, text: str) -> Heard:
        """Название заказа на слух (``search.heard_title``, #4016): слова названия, сколько из них архив знает, и
        запись, чьё название звучит ими, когда непонятых больше."""
        return heard_title(self._library(), text)

    def work(self, title: str, composer: str) -> bool:
        """Произведение есть в каталоге: запись RTTTL со всеми словами названия или партитура с полным совпадением."""
        if verify(self._library(), title):
            return True
        title_words = tuple(work_word(t) for t in terms(None, title))
        ref = WorkRef((WorkWord((composer.lower(),), (composer.lower(),)),) if composer else (), title_words)
        scores = self._scores.work_candidates(ref) if self._scores is not None and title_words else ()
        return any(c.title_match >= kn.TITLE_EXACT for c in scores)


class RequestSlotParser:
    """Код-разбор → кэш → LLM с проверкой каталогом; метрики (доля LLM, кэш, p50/p95) — в строку лога."""

    def __init__(self, words: Terms, catalog: Catalog, *, ask: Optional[Ask] = None,
                 wait_s: float = kn.REQUEST_SLOTS_WAIT_MS / 1000.0, log: Callable[[], Any] = lambda: _LOG,
                 clock: Callable[[], float] = time.perf_counter,
                 spawn: Callable[[Callable[[], None]], None] = lambda fn: threading.Thread(
                     target=fn, name="rbx-request-slots", daemon=True).start()) -> None:
        self._words = words
        self._catalog = catalog
        self._ask = ask
        self._wait_s = wait_s
        self._log = log
        self._clock = clock
        self._spawn = spawn
        self._cache: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()
        self._lock = threading.Lock()
        self.metrics = DecisionMetrics()

    def parse(self, text: str, work: str, command: Any = None) -> rs.RequestSlots:
        """Слоты заказа: ``work`` — название от грамматики (без глагола и тональности), ``text`` — слова человека,
        ``command`` — ``MediaCommand`` (инструмент, стиль, настроение грамматики). Непонятых слов названия больше
        понятых (#4016) — сначала название по звучанию (``Catalog.heard``, без LLM); не нашлось, а LLM не подтвердила
        ни произведение, ни автора — ``doubt``: слова, как услышаны, вместо поиска по одним понятым."""
        started = self._clock()
        words = list(self._words(work))
        missing = unknown(words)
        code = self._code(work, words, missing, command)
        heard = self._catalog.heard(work)
        if heard.record is not None:
            slots, source = _sounded(code, heard), "sound"
        else:
            slots, source = self._resolved(text, work, code, words, missing)
            if heard.truncated and not (slots.work_confirmed or slots.composer):
                slots = replace(slots, doubt=heard.spoken)
        slots = replace(slots, source=source, ms=(self._clock() - started) * 1000.0)
        self._note(work, slots, source)
        return slots

    def _resolved(self, text: str, work: str, code: rs.RequestSlots, words: Sequence[Term],
                  missing: Sequence[Term]) -> Tuple[rs.RequestSlots, str]:
        """Код-разбор, а с непонятыми словами — кэш разборов или LLM (``(слоты, источник)``)."""
        if not missing:
            return code, "code"
        key = theme_phrase(work)
        answer, source = self._cached(key), "cache"
        if answer is None and self._ask is not None:
            answer, source = self._llm(text, key, [t.word for t in missing])
        return (self._merge(code, answer, words), source) if answer is not None else (code, "code")

    @staticmethod
    def _code(work: str, words: Sequence[Term], missing: Sequence[Term], command: Any) -> rs.RequestSlots:
        """Код-разбор: известные слова — строка поиска (непонятые в ней не нужны: каталог их не знает), слоты
        грамматики как есть. Известных нет — фраза целиком (как до #3609)."""
        known = [t.word for t in words if t not in missing]
        query = " ".join(known) if missing and known else work
        return rs.RequestSlots(query, work=work, instrument=getattr(command, "instrument", "") or "",
                               style=getattr(command, "style", "") or "", mood=getattr(command, "mood", "") or "",
                               unknown=tuple(t.word for t in missing))

    def _cached(self, key: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            answer = self._cache.get(key)
            if answer is not None:
                self._cache.move_to_end(key)
            return answer

    def _store(self, key: str, answer: Dict[str, Any]) -> None:
        with self._lock:
            self._cache[key] = answer
            self._cache.move_to_end(key)
            while len(self._cache) > CACHE_SIZE:
                self._cache.popitem(last=False)

    def _llm(self, text: str, key: str, missing: List[str]) -> Tuple[Optional[Dict[str, Any]], str]:
        """Спросить LLM и проверить ответ каталогом; ждём не дольше ``wait_s``, опоздавший ответ — в кэш."""
        def run() -> Optional[Dict[str, Any]]:
            started = self._clock()
            try:
                answer = self._checked(text, missing)
            except Exception as exc:  # noqa: BLE001 — разбор не роняет заказ: остаётся код-разбор
                self._log().warning(f"⚠️ [request-slots] «{key}»: {type(exc).__name__}: {exc}")
                return None
            self.metrics.observe_latency("llm", (self._clock() - started) * 1000.0)
            if answer is not None:
                self._store(key, answer)
            return answer

        self.metrics.count("llm")
        done, answer = in_time(self._spawn, self._wait_s, run)
        if not done:
            self.metrics.count("late")
            self._log().warning(f"⚠️ [request-slots] «{key}»: LLM не ответила за {self._wait_s:g} с — код-разбор, "
                                "ответ допишется в кэш")
            return None, "code"
        return answer, "llm"

    def _checked(self, text: str, missing: List[str]) -> Optional[Dict[str, Any]]:
        """Ответ LLM по схеме и проверка каталогом: ``{поле: значение, work_confirmed, rejected}`` или ``None``."""
        system, user = rs.prompt(text, missing)
        outcome, response, detail = self._ask(system, user, rs.tool(), kn.REQUEST_SLOTS_CALL_MS / 1000.0,
                                              ASK_MAX_TOKENS)
        if outcome != "ok":
            self._log().info(f"🧩 [request-slots] LLM {outcome} {detail}".rstrip())
            return None
        try:
            fields = rs.validate(tool_payload(response, rs.SUBMIT_TOOL))
        except QueriesInvalid as exc:
            self._log().info(f"🧩 [request-slots] ответ LLM не по схеме: {exc}")
            return None
        rejected = []
        composer = self._catalog.composer(fields["composer"]) if fields["composer"] else ""
        if fields["composer"] and not composer:
            rejected.append(f"composer={fields['composer']}")
        confirmed = bool(fields["work"]) and self._catalog.work(fields["work"], composer)
        if fields["work"] and not confirmed:
            rejected.append(f"work={fields['work']}")
        return {**fields, "composer": composer, "work": fields["work"] if confirmed else "",
                "work_confirmed": confirmed, "rejected": tuple(rejected)}

    def _merge(self, code: rs.RequestSlots, answer: Mapping[str, Any], words: Sequence[Term]) -> rs.RequestSlots:
        """Слоты грамматики важнее (инструмент «на органе» назван словами); пустые — из проверенного ответа LLM.
        Строка поиска — подтверждённые композитор и произведение; неподтверждённое — известные слова кода."""
        composer, work = answer["composer"], answer["work"]
        known = " ".join(t.word for t in words if t.word not in code.unknown)
        query = " ".join(p for p in (composer, work or known) if p) or code.query
        return replace(code, query=query, composer=composer, work=work or code.work,
                       work_confirmed=bool(answer["work_confirmed"]),
                       instrument=code.instrument or answer["instrument"], style=code.style or answer["style"],
                       mood=code.mood or answer["mood"], mode=answer["mode"], rejected=answer["rejected"])

    def _note(self, work: str, slots: rs.RequestSlots, source: str) -> None:
        self.metrics.count("parse")
        self.metrics.count(f"source_{slots.source}")
        self.metrics.observe_latency("parse", slots.ms)
        counts, lat, llm = self.metrics.counters(), self.metrics.latency("parse"), self.metrics.latency("llm")
        tail = (f"llm={counts.get('llm', 0)}/{counts.get('parse', 0)} late={counts.get('late', 0)} "
                f"cache={counts.get('source_cache', 0)} p50={_ms(lat.p50_ms)} p95={_ms(lat.p95_ms)} "
                f"llm_p50={_ms(llm.p50_ms)} llm_p95={_ms(llm.p95_ms)}")
        target = f"не уверен: «{slots.doubt}»" if slots.doubt else f"«{slots.query}»"
        self._log().info(f"🧩 [request-slots] «{work}» source={slots.source} ms={slots.ms:.0f} {slots.describe()} "
                         f"непонятые={list(slots.unknown)} отброшено={list(slots.rejected)} → {target}; "
                         f"{tail}")


def _ms(value: Optional[float]) -> str:
    return "-" if value is None else f"{value:.0f}"


def _sounded(code: rs.RequestSlots, heard: Heard) -> rs.RequestSlots:
    """Слоты по названию, найденному на слух (#4016): название записи — произведение, подтверждённое каталогом
    (запись и есть каталог); совпал исполнитель («роллинг стоунз») — автор, строка поиска — его имя."""
    if heard.field == "artist":
        return replace(code, query=heard.name, composer=heard.name)
    return replace(code, query=heard.name, work=heard.name, work_confirmed=True)


def doubt_text(heard: str) -> str:
    """Фраза кода, когда непонятно, что названо (``RequestSlots.doubt``, #4016): слова — как услышаны."""
    return kn.REQUEST_SLOTS_DOUBT_TEXT.format(heard=heard)


class HarnessAsk:
    """``Ask`` (контракт ``SetReasoner.ask``: ``(system, user, tool, deadline_s, max_tokens) -> (outcome, ответ,
    detail)``) над провайдером LLM-ступени выбора (``llm_harness``: DeepSeek, ``json_object``) — ответ по схеме тула
    без ``<think>`` MiniMax. Свой circuit breaker: 3 ошибки/опоздания подряд — 5 мин без LLM."""

    def __init__(self, provider: Any, breaker: Optional[CircuitBreaker] = None) -> None:
        self.provider = provider
        self._breaker = breaker or CircuitBreaker(failure_threshold=BREAKER_FAILURES, reset_after_s=BREAKER_RESET_S)

    def __call__(self, system: str, user: str, tool: Mapping[str, Any], deadline_s: float,
                 max_tokens: int = ASK_MAX_TOKENS) -> Tuple[str, Any, str]:
        if not self._breaker.allow():
            return "circuit_open", None, "llm_harness пропускается после ошибок подряд"
        fn = tool["function"]
        try:
            args = _run(asyncio.wait_for(self.provider.complete_json(system, user, fn["parameters"]), deadline_s))
        except asyncio.TimeoutError:
            self._breaker.record_failure()
            return "late", None, f"нет ответа за {deadline_s:g} с"
        except Exception as exc:  # noqa: BLE001 — провайдер не роняет заказ
            self._breaker.record_failure()
            return "error", None, f"{type(exc).__name__}: {exc}"
        self._breaker.record_success()
        call = SimpleNamespace(name=fn["name"], arguments=dict(args))
        return "ok", SimpleNamespace(tool_calls=(call,), truncated_tool_args=False), ""


def _run(coro: Any) -> Any:
    from .chooser_model import run_sync

    return run_sync(coro)


def harness_ask(env: Optional[Mapping[str, str]] = None, log: Callable[[], Any] = lambda: _LOG) -> Optional[Ask]:
    """``Ask`` разбора слотов из env LLM-ступени выбора (``LLM_HARNESS_*``, по умолчанию DeepSeek); нет ключа —
    ``None`` (только код-разбор) и одна строка в лог."""
    from .chooser_model import build_provider

    provider = build_provider("llm_harness", env, timeout_s=kn.REQUEST_SLOTS_CALL_MS / 1000.0)
    if provider is None:
        log().warning("🧩 [request-slots] LLM выключена: нет ключа/конфига llm_harness — только код-разбор")
        return None
    log().info(f"🧩 [request-slots] LLM: llm_harness {getattr(provider, 'model', '')}, ждём "
               f"{kn.REQUEST_SLOTS_WAIT_MS} мс")
    return HarnessAsk(provider)


__all__ = ["CACHE_SIZE", "Catalog", "HarnessAsk", "RequestSlotParser", "Terms", "doubt_text", "harness_ask", "unknown"]
