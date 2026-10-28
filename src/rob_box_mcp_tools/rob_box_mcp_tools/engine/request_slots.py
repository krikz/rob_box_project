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
   названия больше понятых, а понятое само ничего не называет (не «что-то похожее на **марио**», не «просто **Баха**»)
   — код ищет запись архива, чьё название звучит этими словами (``search.heard_title``): «Running Up That Hill».
   Находка — догадка, не подтверждение: кэш и LLM спрашиваются; назвали другое — решают они, молчат или опоздали —
   играет находка (``source=sound``, без ``work_confirmed``). Не нашлось, а LLM не подтвердила ни произведение, ни
   автора — заказ не урезается молча до понятых слов: ``RequestSlots.doubt`` и фраза кода :func:`doubt_text` («не
   уверен, что это за песня — «…»?»).

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

from ..core.translit_ru import strip_version_tail
from .piece import work_word
from .search import Heard, Term, heard_title, terms, title_key, warm_heard
from .theme_links import Ask, later, tool_payload, verify

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

    def heard(self, text: str, said: str = "") -> Heard:
        """Название заказа на слух (``search.heard_title``, #4016): слова названия ``text``, сколько из них архив
        знает, и запись, чьё название звучит ими (с жанровым словом перед названием в словах человека ``said``), когда
        непонятых больше."""
        return heard_title(self._library(), text, said)

    def warm(self) -> int:
        """Указатель звучаний архива для разбора на слух — заранее (``search.warm_heard``)."""
        return warm_heard(self._library())

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
        понятых (#4016; пояснение после названия — «дарта вейдера», «только быстрее» — не в счёт; понятое слово само
        называет мелодию или автора — «что-то похожее на марио» — тоже) — название по звучанию (``Catalog.heard``).
        Находка на слух — догадка кода, а не подтверждение (ревью #4124, круг 2: «уи вил рок ю» → «Rock With You»):
        кэш разборов и LLM спрашиваются как обычно; назвали произведение или автора — решают они (проверенные каталогом),
        молчат или не успели — играет находка, ``work_confirmed`` у неё нет. Не нашлось, а LLM не подтвердила ни
        произведение, ни автора — ``doubt``: слова, как услышаны, вместо поиска по одним понятым."""
        started = self._clock()
        words = list(self._words(work))
        missing = unknown(words)
        code = self._code(work, words, missing, command)
        pending = self._answer(text, work, [t.word for t in missing])  # LLM думает, пока код слушает (ревью #4124)
        heard = self._catalog.heard(work, text)
        if not missing and (heard.record is not None or heard.truncated):  # «ап», «си» ``terms`` отбросил как короткие
            pending = self._answer(text, work, list(heard.words))
        answer, source = pending()
        merged = self._merge(code, answer, words) if answer is not None else code
        slots, source = _settle(code, merged, heard, answer, source)
        slots = replace(slots, source=source, ms=(self._clock() - started) * 1000.0)
        self._note(work, slots, source)
        return slots

    def warm(self) -> float:
        """Прогрев разбора на слух (#4016): указатель звучаний архива строится при старте в фоне
        (``RequestMusicTool.warm``), а не в первом заказе с непонятым названием (там 0.15–1 с); мс — в лог."""
        started = self._clock()
        count = self._catalog.warm()
        ms = (self._clock() - started) * 1000.0
        self._log().info(f"🧩 [request-slots] прогрев: указатель звучаний ({count}) {ms:.0f} мс")
        return ms

    def _answer(self, text: str, work: str, missing: List[str]) -> Callable[[], Tuple[Optional[Dict[str, Any]], str]]:
        """Проверенный каталогом разбор непонятых слов ``missing``: кэш разборов этой фразы или LLM (вопрос уходит
        сразу, в фоне). Вернувшаяся функция — ``(ответ, источник)``; непонятых нет, LLM нет, не успела или ответ не по
        схеме — ``(None, "code")``."""
        answer = self._cached(theme_phrase(work)) if missing else None
        if answer is not None:
            return lambda: (answer, "cache")
        if not missing or self._ask is None:
            return lambda: (None, "code")
        return self._llm(text, theme_phrase(work), missing)

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

    def _llm(self, text: str, key: str, missing: List[str]) -> Callable[[], Tuple[Optional[Dict[str, Any]], str]]:
        """Спросить LLM (сразу, в фоне) и проверить ответ каталогом; вернувшаяся функция ждёт ответа не дольше
        ``wait_s`` от вопроса — ``(ответ, "llm")``, опоздал или ошибка — ``(None, "code")``; опоздавший ответ — в кэш."""
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
        asked, waiter = self._clock(), later(self._spawn, run)

        def wait() -> Tuple[Optional[Dict[str, Any]], str]:
            done, answer = waiter(max(0.0, self._wait_s - (self._clock() - asked)))
            if not done:
                self.metrics.count("late")
                self._log().warning(f"⚠️ [request-slots] «{key}»: LLM не ответила за {self._wait_s:g} с — код-разбор, "
                                    "ответ допишется в кэш")
                return None, "code"
            return (answer, "llm") if answer is not None else (None, "code")

        return wait

    def _checked(self, text: str, missing: List[str]) -> Optional[Dict[str, Any]]:
        """Ответ LLM по схеме и проверка каталогом: ``{поле: значение, work_confirmed, rejected, said}`` или ``None``
        (``said`` — название от LLM как есть: сверка с находкой на слух, :func:`_agrees`)."""
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
                "work_confirmed": confirmed, "rejected": tuple(rejected), "said": fields["work"]}

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
    """Слоты по названию, найденному на слух (#4016): название записи — произведение и строка поиска; совпал
    исполнитель («роллинг стоунз») — автор, строка поиска — его имя. ``work_confirmed`` нет: запись в каталоге есть, но
    что человек назвал именно её — догадка по звучанию (ревью #4124, круг 2), её не подтвердили ни LLM, ни кэш."""
    if heard.field == "artist":
        return replace(code, query=heard.name, composer=heard.name)
    return replace(code, query=heard.name, work=heard.name)


def _settle(code: rs.RequestSlots, merged: rs.RequestSlots, heard: Heard, answer: Optional[Mapping[str, Any]],
            source: str) -> Tuple[rs.RequestSlots, str]:
    """Слоты заказа (``(слоты, источник)``) из код-разбора ``code``, разбора LLM/кэша, проверенного каталогом
    (``merged``, поля ответа — ``answer``), и находки на слух ``heard`` (ревью #4124, круг 2): LLM назвала то же
    произведение (:func:`_agrees`) — находка подтверждена; LLM молчит о произведении и авторе или не успела — догадка
    (``source=sound``); LLM назвала другое — решает её разбор; не нашлось, урезано и ничего не подтверждено — переспрос
    (``doubt``)."""
    if heard.record is not None and _agrees(heard, answer):
        return replace(merged, query=heard.name, work=heard.name, work_confirmed=True), source
    if heard.record is not None and not _names(answer):
        return _sounded(code, heard), "sound"
    if heard.truncated and not (merged.work_confirmed or merged.composer):
        return replace(merged, doubt=heard.spoken), source
    return merged, source


def _agrees(heard: Heard, answer: Optional[Mapping[str, Any]]) -> bool:
    """LLM (или кэш) назвала то же произведение, что нашлось на слух: все слова названия находки (без хвоста версии)
    — в её названии («Running Up That Hill» в «Running Up That Hill (A Deal with God)»: полного названия LLM в архиве
    может не быть — запись находки в нём есть). Автор LLM записи не отменяет: «Kate Bush» при кавере Within
    Temptation на develop кончался отказом «Пьес автора «Bush hill» у меня не нашлось»."""
    if answer is None or heard.field != "title":
        return False
    found = set(title_key(strip_version_tail(heard.name)).split())
    return bool(found) and found <= set(title_key(answer.get("said") or answer["work"]).split())


def _names(answer: Optional[Mapping[str, Any]]) -> bool:
    """LLM (или кэш её разборов) назвала произведение или автора — подтверждённых каталогом или отброшенных: тогда
    решает её разбор, а не находка на слух («тико тико» — «Tico Tico no Fuba», которого в архиве нет, а не «Tick Tick»).
    Молчит о них (только инструмент, стиль, режим) — находке не возражает."""
    return answer is not None and bool(answer["work"] or answer["composer"] or answer["rejected"])


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
