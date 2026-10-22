"""Сет из генерации (ADR-0158 §2.7, решения Шифу В2/В4/В5; PR-8): кэш сначала, недостающее — генерацией по ходу.

Звук, стыки и события — ``SetSession`` поверх ``PlayerOwner`` (тот же сет, что у синтеза): здесь только источник
треков ``(track_no, deck) -> AudioTrack | Track`` (шов ``session.TrackSource``) и заказы генерации.

* План: треки кэша жанра (``source.set_cache``: инструментал, свежие первыми) закрывают сколько могут; недостающее
  заказывается по одной задаче за раз (2 варианта на задачу), не больше ``GEN_SET_MAX_TASKS`` и в пределах лимита сета
  (``engine.gen.budget``: ``set_spent`` в ``generate_into_cache``). Заказ — при подготовке трека, то есть на
  ``started`` предыдущего (ADR §2.7 «заказ N+1 на started(N)»).
* Первая задача — ``GEN_SET_FIRST_MODEL`` (V6_MINI), если кэш не закрыл первый трек (В2); остальные — ``GEN_SET_MODEL``.
* Варианты одной задачи соседями не ставятся (§1.4: подряд — «вариация одного трека»); не готово ничего другого к
  сроку — второй вариант той же задачи (§2.7), затем повтор сыгранного, но не тишина.
* Мост (В4): кэш пуст — первым треком сета играет синтез ближайшим сильным стилем (``GEN_BRIDGE_STYLES``), пока
  пишется первая генерация (сет продлевает мост проходами формы). Фразу моста строит код из фактов; LLM только
  окрашивает её подстановками с дедлайном (:func:`bridge_line`), опоздала или соврала — голая фраза кода.
* Тема (#3775, решение Шифу 21.10): из N треков сета K берутся из кэша по теме (``source.theme_cache``) и играют
  сразу, N − K заказываются; K = 0 — пока пишется первый трек темы, тихо (``SET_BACKGROUND_GAIN``) играет фон — трек
  кэша стиля сета — и фраза кода «Готовлю сет на тему …» (:func:`background_text`); кэша стиля нет — мост синтезом.
* Провал генерации (В3): заказов больше нет, сет доигрывает то, что есть; фраза причины — от кода. Исключение (В17):
  Suno отклонил кавер произведения (``rejected``) после всех попыток ``COVER_TRIES`` — выходит только это
  произведение, его слоты берут другие произведения темы, заказы идут дальше.
* Стили по трекам (решение Шифу 14.10): «в стилях клуб, джангл и джаз» — стиль каждого трека по кругу
  (``GenSet.genres``); путь каждого трека — чойсер (``SourceRouter.route_slots``: кэш → генерация → синтез), заказ —
  для ближайшего трека, чьему стилю не хватает готового; тема сета — в теги заказа (``source.theme_tags``).

Стрим до ``success`` (замер 13.10, ``f5aa62a5…``, V6 × 120 с): ``stream_audio_url`` на стадии ``text`` (32 с) отдал весь
файл за 6 с, ``success`` — на 50 с; после ``success`` тот же URL отдаёт те же байты (не пусто). Выигрыш 12–18 с нужен
только первому треку сета, а его закрывает мост — здесь стрим не используется (см. описание PR).
"""

from __future__ import annotations

import logging
import math
import subprocess
import threading
import time
from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Set, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.dj_line import tracks_word
from rob_box_music.render.audio import xfade_beats
from rob_box_music.source import (CACHE, COVER_SLOT, GENERATE, REASON_WORDS, CacheTrack, CoverWork, SourceAsk,
                                  cover_rejected_text, cover_slot, gen_kind, slot_style, theme_fit, theme_title)

from ..renardo_adapter import ALIGN_LEAD_BEATS
from ..session import NEARLY_LEAD_BEATS
from .audio import AudioUnavailable, audio_track
from .budget import Budget
from .pipeline import generate_into_cache
from .provider import GEN_TIMEOUT_S, GenJob, GenUnavailable

_LOG = logging.getLogger(__name__)

#: Сколько ждать фразу LLM поверх моста (В4): опоздала — звучит фраза кода.
BRIDGE_LINE_DEADLINE_S = 4.0
#: Запас до ``nearly_finished`` играющего файла: ждать генерацию дольше — стык не успеет встать.
WAIT_MARGIN_S = 8.0
#: Код отказа пути (``tools_v2.SOURCE_DECLINED``; равенство держит тест).
DECLINED = "music_source"
SUBMIT_BRIDGE = "submit_bridge_line"
BRIDGE_FIELDS = ("next", "bridge", "eta")
BRIDGE_LINE_MAX = 140


@dataclass(frozen=True)
class SetItem:
    """Трек сета из кэша: ``group`` — задача (варианты одной задачи — одна группа), ``origin`` — cache/generated."""

    id: str
    group: str
    title: str
    origin: str = "cache"
    genre: str = ""


def pick_item(ready: Sequence[SetItem], used: Set[str], prev_group: str) -> Tuple[Optional[SetItem], str]:
    """Следующий трек из готовых: не сосед по задаче — ``ok`` (из задачи, у которой несыгранных больше: её второй
    вариант не останется последним без разделителя); иначе вариант той же задачи — ``sibling``; нет — None."""
    fresh = [item for item in ready if item.id not in used]
    left = {group: sum(item.group == group for item in fresh) for group in {item.group for item in fresh}}
    other = sorted((item for item in fresh if item.group != prev_group), key=lambda item: -left[item.group])
    if other:
        return other[0], "ok"
    return (fresh[0], "sibling") if fresh else (None, "")


def supply_of(ready: Sequence[SetItem], used: Set[str], inflight: int) -> int:
    """Сколько треков подряд закрывают готовые и заказанные без соседей-вариантов: у одной задачи из двух
    несыгранных вариантов подряд годится один (между ними нужен трек другой задачи)."""
    fresh = [item for item in ready if item.id not in used]
    groups = len({item.group for item in fresh}) + inflight
    return min(len(fresh) + 2 * inflight, max(0, 2 * groups - 1))


def tasks_needed(slots_left: int, supply: int, ordered: int) -> int:
    """Сколько ещё задач заказать: недостающие треки по 2 варианта на задачу, не больше ``GEN_SET_MAX_TASKS``."""
    missing = max(0, slots_left - supply)
    return min((missing + 1) // 2, max(0, kn.GEN_SET_MAX_TASKS - ordered))


def task_model(ordered: int, bridge: bool) -> str:
    """Первая задача при мосте (кэш не закрыл первый трек) — быстрая модель, остальные — качество (В2)."""
    return kn.GEN_SET_FIRST_MODEL if ordered == 0 and bridge else kn.GEN_SET_MODEL


def genre_name(genre: str) -> str:
    genre = slot_style(genre)
    return kn.GEN_STYLES[genre].name if genre in kn.GEN_STYLES else genre


def bridge_style(genre: str) -> str:
    """Стиль синтеза моста: ближайший сильный стиль жанра (таблица ``GEN_BRIDGE_STYLES``), иначе стиль по умолчанию."""
    return kn.GEN_BRIDGE_STYLES.get(genre, kn.DEFAULT_STYLE)


def bridge_facts(genre: str, model: str) -> Dict[str, str]:
    """Факты фразы моста — от кода: что пишется, что играет сейчас, через сколько секунд."""
    return {"next": genre_name(genre), "bridge": kn.GEN_STYLES[bridge_style(genre)].name,
            "eta": str(int(round(kn.GEN_MODELS[model].typical_s / 10.0) * 10))}


def bridge_template(facts: Dict[str, str]) -> str:
    return "Пока я дописываю {next} — вот вам {bridge}, секунд {eta}.".format(**facts)


def bridge_tool() -> Dict[str, Any]:
    line = {"type": "string", "maxLength": BRIDGE_LINE_MAX,
            "description": "фраза по-русски; жанр, стиль и секунды — только подстановками {next}, {bridge}, {eta}"}
    return {"type": "function", "function": {
        "name": SUBMIT_BRIDGE, "description": "Отдать одну фразу ожидания. Вызвать ровно один раз.",
        "parameters": {"type": "object", "properties": {"line": line}, "required": ["line"],
                       "additionalProperties": False}}}


def validate_bridge(template: Any, facts: Dict[str, str]) -> Optional[str]:
    """Шаблон LLM → фраза или None: все три подстановки, других скобок и цифр нет, длина в пределе."""
    if not isinstance(template, str) or any("{" + f + "}" not in template for f in BRIDGE_FIELDS):
        return None
    bare = template
    for field in BRIDGE_FIELDS:
        bare = bare.replace("{" + field + "}", "")
    if any(ch.isdigit() or ch in "{}" for ch in bare):
        return None
    line = template.format(**facts)
    return line if len(line) <= BRIDGE_LINE_MAX else None


def bridge_line(facts: Dict[str, str], ask: Optional[Callable[..., Tuple[str, Any, str]]],
                deadline_s: float = BRIDGE_LINE_DEADLINE_S, log: Any = _LOG) -> str:
    """Фраза моста: окраска LLM (``SetReasoner.ask``) подстановками, иначе шаблон кода (ADR-0148)."""
    if ask is None:
        return bridge_template(facts)
    system = ("Ты робот-диджей. Пока сочиняется новый трек, играет другой. Скажи одну весёлую фразу ожидания, "
              f"по-русски, не длиннее {BRIDGE_LINE_MAX} символов. Жанр, который сочиняешь, стиль, который играет, и "
              "секунды НЕ пиши словами и цифрами — только подстановками {next}, {bridge}, {eta} (все три). Ответ — "
              f"один вызов {SUBMIT_BRIDGE}.")
    user = f"{{next}} = «{facts['next']}»; {{bridge}} = «{facts['bridge']}»; {{eta}} = {facts['eta']} секунд."
    outcome, response, detail = ask(system, user, bridge_tool(), deadline_s)
    calls = [c for c in (getattr(response, "tool_calls", ()) or ()) if c.name == SUBMIT_BRIDGE]
    line = validate_bridge(dict(calls[0].arguments).get("line"), facts) if outcome == "ok" and calls else None
    log.info(f"🗣️ [gen set] фраза моста: outcome={outcome} {'llm' if line else 'code'} {detail}".rstrip())
    return line or bridge_template(facts)


def set_head(genres: Any) -> str:
    """«Рок-сет» одним стилем; «Сет «клуб, драм-н-бейс, джаз»» — стили по трекам."""
    names = [genre_name(g) for g in ((genres,) if isinstance(genres, str) else dict.fromkeys(genres))]
    if len(names) == 1:
        return f"{names[0][:1].upper()}{names[0][1:]}-сет"
    return f"Сет «{', '.join(names)}»"


def set_notes(unknown: Sequence[str], dropped: Sequence[Tuple[str, str]]) -> str:
    """Что из просьбы не сыграет — от кода: непонятые слова стилей и стили без пути (``(стиль, причина)``)."""
    parts = []
    if unknown:
        listed = ", ".join(f"«{w}»" for w in unknown)
        parts.append(f"{'Стиль' if len(unknown) == 1 else 'Стили'} {listed} не знаю.")
    for genre, reason in dropped:
        parts.append(f"Без «{genre_name(genre)}»: {REASON_WORDS.get(reason, reason) if reason else 'нечем играть'}.")
    return " ".join(parts)


def synth_set_note(reason: str, style: str) -> str:
    """Синтез-сет — запас (В13): почему не кэш и не генерация; генерации нет совсем — «»."""
    from rob_box_music.source import QUIET_REASONS

    if reason in QUIET_REASONS:
        return ""
    return f"Сгенерировать сет сейчас не могу: {REASON_WORDS.get(reason, reason)} — играю своё: {genre_name(style)}."


def background_text(theme: str) -> str:
    """Фраза запуска сета темы, когда по теме в кэше ничего и играет тихий фон — от кода (Шифу 21.10, #3775)."""
    return f"Готовлю сет на тему «{theme}», пока немного фона."


def theme_why(tracks: int, themed: int, ordered: int, gen_reason: str) -> str:
    """Почему столько заказано (строка лога ``[gen set]``): кэша по теме хватило, не было, дозаказ, денег нет."""
    if themed >= tracks:
        return "по теме хватило кэша — без заказа"
    if ordered < tracks - themed:
        return f"заказать все недостающие нельзя: {gen_reason or 'путь трека — кэш стиля/синтез'}"
    return "по теме в кэше ничего — сет заказом" if not themed else "дозаказ недостающих, что есть — сразу"


def themed_count(tags: str, routes: Sequence[Any]) -> int:
    """K — сколько треков сета взято из кэша по теме (``source.theme_fit`` ≥ ``THEME_FIT_MIN``)."""
    return sum(r.kind == CACHE and r.track is not None and theme_fit(tags, r.track) >= kn.THEME_FIT_MIN
               for r in routes)


def theme_note(tags: str, routes: Sequence[Any], themed: int, background: Optional[CacheTrack]) -> str:
    """Часть строки ``[gen set] слоты``: K по теме из кэша, заказано треков (задач), фон, почему (#3775)."""
    ordered = sum(r.kind == GENERATE for r in routes)
    head = (f"по теме из кэша={themed if tags else '—'} заказ={ordered} (задач ≈{math.ceil(ordered / 2)}) "
            f"фон={f'да {background.id}' if background else 'нет'}")
    if not tags:
        return head
    reason = next((r.gen_reason for r in routes if r.gen_reason), "")
    return f"{head} — {theme_why(len(routes), themed, ordered, reason)}"


def launch_text(genre: Any, tracks: int, cached: int, gen_reason: str) -> str:
    """Фраза запуска сета без моста — от кода: сколько из сочинённого раньше, допишет ли остальное (G3)."""
    head = set_head(genre)
    if cached >= tracks:
        return f"{head} из сочинённого раньше: {tracks} {tracks_word(tracks)}."
    if not gen_reason:
        return f"{head} на {tracks} {tracks_word(tracks)}: {cached} из сочинённого раньше, остальные допишу по ходу."
    words = REASON_WORDS.get(gen_reason, gen_reason)
    return f"{head} из сочинённого раньше: только {cached} {tracks_word(cached)} — дописать не могу: {words}."


def failed_text(genre: str, reason: str) -> str:
    return f"Дописать {genre_name(genre)} не вышло: {REASON_WORDS.get(reason, reason)} — доиграю из того, что есть."


def slot_plan(keys: Sequence[str], genres: Sequence[str], unique: Sequence[str], slots: int) -> List[str]:
    """Ключ слота каждого трека-файла: ключи путей (стиль или кавер стиля, В14), иначе стили, иначе стили по кругу."""
    if keys and len(keys) == slots:
        return list(keys)
    if len(genres) == slots:
        return list(genres)
    return [unique[i % len(unique)] for i in range(max(1, slots))]


def respread(plan: Sequence[str], start: int, dead: Set[str], alive: Callable[[str], List[str]]) -> List[str]:
    """Слоты ``plan[start:]`` с ключом из ``dead`` (произведение вышло из сета, В17) → ключ другого произведения того же
    стиля (``alive(стиль)``, порядок темы): реже всех в плане, не сосед; других нет — инструментал стиля слота."""
    out = list(plan)
    for i in range(start, len(out)):
        if out[i] not in dead:
            continue
        keys = alive(slot_style(out[i]))
        near = {out[j] for j in (i - 1, i + 1) if 0 <= j < len(out)}
        pool = [k for k in keys if k not in near] or keys
        out[i] = min(pool, key=lambda k: (out.count(k), keys.index(k))) if pool else slot_style(out[i])
    return out


class SetCovers:
    """Каверы произведений темы в треках сета (В14, В16): ``works`` — произведение по ключу слота
    ``cover|<стиль>|<пьеса>`` (план решил, какое произведение у какого трека), ``make(work, ask, set_spent)`` — тот же
    ``CoverMaker.make`` с попытками после отказа Suno (``cover.cover_tries``, В17); ``pieces`` — все произведения темы
    по порядку: ими закрываются слоты произведения, которое Suno не взял (:meth:`drop`)."""

    def __init__(self, works: Optional[Mapping[str, CoverWork]] = None,
                 make: Optional[Callable[[CoverWork, SourceAsk, float], Any]] = None,
                 pieces: Sequence[CoverWork] = ()) -> None:
        self._works, self._make = dict(works or {}), make
        self._pieces = tuple(dict.fromkeys(tuple(pieces) + tuple(self._works.values())))
        self._dead: Set[CoverWork] = set()

    def drop(self, key: str, plan: Sequence[str], start: int) -> Tuple[CoverWork, List[str], bool]:
        """Произведение слота ``key`` выходит из сета: ``(произведение, план, остались ли каверы впереди)`` — его слоты
        с ``start`` во всех стилях берут живые произведения (:func:`respread`)."""
        work = self._works.get(key, CoverWork(key, ""))
        self._dead.add(work)
        dead = {k for k in plan if k == key or self._works.get(k) == work}
        out = respread(plan, start, dead, self.alive)
        return work, out, any(k.startswith(COVER_SLOT) for k in out[start:])

    def alive(self, style: str) -> List[str]:
        """Ключи слотов живых произведений темы в стиле ``style`` (порядок темы)."""
        keys = []
        for work in (w for w in self._pieces if w not in self._dead):
            key = cover_slot(style, work.title)
            self._works.setdefault(key, work)
            keys.append(key)
        return keys

    def task(self, key: str, ask: SourceAsk, spent: float) -> Tuple[List["SetItem"], str, float]:
        """Задача кавера произведения слота в его стиле → ``(варианты, task_id, кредиты)``."""
        work = self._works.get(key)
        if work is None or self._make is None:
            raise GenUnavailable("no_material", f"кавер {key}: материала нет")
        result = self._make(work, replace(ask, genre=slot_style(key)), spent)
        if result.reason:
            raise GenUnavailable(result.reason, f"кавер «{work.title}»")
        title = f"{work.spoken or work.title} (кавер)"
        return ([SetItem(t, result.task_id, title, "generated", key) for t in result.track_ids], result.task_id,
                result.credits)


def _say(speak: Optional[Callable[[str], Any]], text: str) -> None:
    if speak is not None:
        speak(text)


def _thread(fn: Callable[[], None]) -> None:
    threading.Thread(target=fn, name="rbx-gen-set", daemon=True).start()


class GenSet:
    """Источник треков одного сета из генерации. ``bridge`` — ``deck -> Track`` синтеза-моста (первый трек) или
    None; ``gen_reason`` — почему генерировать нельзя («» — можно); ``speak`` — фраза провала генерации."""

    def __init__(self, *, provider: Any, library: Any, ask: SourceAsk, genre: str, theme: str, set_id: str, bpm: int,
                 tracks: int, cache: Sequence[CacheTrack], bridge: Optional[Callable[[str], Any]] = None,
                 gen_reason: str = "", speak: Optional[Callable[[str], Any]] = None, log: Any = _LOG,
                 spawn: Callable[[Callable[[], None]], None] = _thread, run: Callable[..., Any] = subprocess.run,
                 clock: Callable[[], float] = time.monotonic,
                 publish: Optional[Callable[[str, Dict[str, Any]], Any]] = None, genres: Sequence[str] = (),
                 theme_tags: str = "", keys: Sequence[str] = (), covers: Optional["SetCovers"] = None) -> None:
        self._provider, self._library, self._ask, self._theme = provider, library, ask, theme
        self._publish = publish  # ``PlayerOwner.update_dj``: название играющего трека — в снимок («что играет»)
        self.set_id, self.bpm, self.slots = set_id, int(bpm), int(tracks)
        self.genres = tuple(dict.fromkeys(genres)) or (genre,)
        self.genre, self._tags = self.genres[0], theme_tags
        self._plan = slot_plan(keys, genres, self.genres, self.slots)
        self._covers = covers or SetCovers()  # каверы материала темы (В14)
        self._bridge, self.gen_reason, self._speak, self._log = bridge, gen_reason, speak, log
        self._spawn, self._run, self._clock = spawn, run, clock
        self._ready: List[SetItem] = [SetItem(t.id, t.group or t.id, t.title, genre=self._genre_of(t)) for t in cache]
        self._used: Set[str] = set()
        self._prev, self._taken, self._inflight, self.ordered, self.spent = "", 0, 0, 0, 0.0
        self._inflight_genre = ""
        self._playing_s: Optional[float] = None  # длина играющего файла; None — играет мост (продлевается)
        self._items: Dict[str, Tuple[SetItem, str]] = {}  # track_id деки → (трек, как выбран)
        self.task_ids: List[str] = []
        self._cond = threading.Condition()

    @property
    def bridged(self) -> bool:
        return self._bridge is not None

    def _genre_of(self, track: CacheTrack) -> str:
        """Ключ слота трека кэша: кавер — кавер своего произведения и стиля; стиль свой, иначе стиль той же семьи
        синтеза (гранж — рок), иначе первый."""
        if track.kind == "cover" and cover_slot(track.style_key, track.work_id) in self._plan:
            return cover_slot(track.style_key, track.work_id)
        if track.style_key in self.genres:
            return track.style_key
        family = kn.GEN_STYLES.get(track.style_key)
        return next((g for g in self.genres if family is not None and family.synth is not None
                     and kn.GEN_STYLES[g].synth == family.synth), self.genres[0])

    def _slot_genre(self) -> str:
        return self._plan[min(self._taken, len(self._plan) - 1)]

    def next_track(self, track_no: int, deck: str) -> Any:
        """Шов ``TrackSource``: мост (трек 1, кэш пуст) или файл кэша; заказ недостающего — здесь же."""
        self.order_more()
        if track_no == 1 and self._bridge is not None:
            return self._bridge(deck)
        for _ in range(3):  # файл не декодировался — следующий трек, а не отказ сета
            item, how = self._take(self._wait_s())
            if item is None:
                raise RuntimeError(f"сет {self.set_id}: нечего ставить треком {track_no}")
            try:
                track = self._audio(item, how, track_no, deck)
            except AudioUnavailable as exc:
                self._log.warning(f"⚠️ [gen set] {self.set_id} трек {item.id} не звучит: {exc}")
                continue
            self.order_more()
            return track
        raise RuntimeError(f"сет {self.set_id}: файлы треков не декодируются")

    def on_started(self, track_id: str) -> str:
        """Пометка строки ``started``: какой трек кэша и как выбран; название — в снимок плеера."""
        item, how = self._items.get(track_id, (None, "bridge"))
        if item is None:
            return "gen=bridge"
        if self._publish is not None:
            self._publish(track_id, {"name": item.title, "source": "generated", "library_id": item.id})
        return f"gen={item.id} {item.origin} {how} «{item.title}»"

    def _wait_s(self) -> float:
        if self._playing_s is None:  # играет мост: он продлевается проходами — ждать генерацию до таймаута
            return GEN_TIMEOUT_S
        lead_s = NEARLY_LEAD_BEATS * 60.0 / self.bpm
        return max(1.0, self._playing_s - lead_s - WAIT_MARGIN_S)

    def _take(self, wait_s: float) -> Tuple[Optional[SetItem], str]:
        """Трек стиля этого слота; пишется его задача — ждать до срока; иначе вариант-сосед, трек другого стиля
        сета (``other``), повтор сыгранного — но не тишина."""
        end = self._clock() + wait_s
        with self._cond:
            while True:
                genre = self._slot_genre()  # план меняется, когда произведение выходит из сета (В17)
                item, how = pick_item([i for i in self._ready if i.genre == genre], self._used, self._prev)
                left = end - self._clock()
                if how == "ok" or not (self._inflight and self._inflight_genre == genre) or left <= 0:
                    break
                self._cond.wait(left)
            if item is None:
                item, how = pick_item(self._ready, self._used, self._prev)
                how = "other" if item is not None else how
            if item is None:
                item, how = self._repeat()
            if item is not None:
                self._used.add(item.id)
                self._prev, self._taken = item.group, self._taken + 1
            return item, how

    def _repeat(self) -> Tuple[Optional[SetItem], str]:
        """Нового нет и не будет к сроку: повтор сыгранного (не сосед), но не тишина."""
        played = [item for item in self._ready if item.group != self._prev] or list(self._ready)
        if not played:
            return None, ""
        self._used.discard(played[0].id)
        self._ready.append(self._ready.pop(self._ready.index(played[0])))  # повтор — в конец очереди
        return played[0], "repeat"

    def _audio(self, item: SetItem, how: str, track_no: int, deck: str) -> Any:
        info = self._library.get_track_info(item.id)
        if not info.get("success"):
            raise AudioUnavailable("no_row", str(info.get("error")))
        track_id = f"{self.set_id}:{track_no:02d}:{deck}"
        track = replace(audio_track(info, track_id, run=self._run), bpm_nominal=self.bpm)
        self._library.increment_play_count(item.id)
        self._playing_s = track.duration_s
        self._items[track_id] = (item, how)
        self._log.info(f"🎧 [gen set] {self.set_id} трек {track_no}: {item.genre} {item.origin} {item.id} "
                       f"«{item.title}» ({how}, задача {item.group}) {track.duration_s:.0f} с")
        return track

    def _short_genre(self) -> Optional[str]:
        """Стиль ближайшего трека, которому не хватает готового (одним стилем — с учётом вариантов-соседей)."""
        upcoming = self._plan[self._taken:]
        for genre in dict.fromkeys(upcoming):
            ready = [i for i in self._ready if i.genre == genre]
            supply = (supply_of(ready, self._used, 0) if len(set(self._plan)) == 1
                      else sum(i.id not in self._used for i in ready))
            if tasks_needed(upcoming.count(genre), supply, self.ordered):
                return genre
        return None

    def order_more(self) -> None:
        """Заказать следующую задачу, если недостающих треков больше, чем готовых и заказанных (одна за раз)."""
        with self._cond:
            genre = None if self.gen_reason or self._inflight else self._short_genre()
            if genre is None:
                return
            cover = genre.startswith(COVER_SLOT)
            model = kn.COVER_MODEL if cover else task_model(self.ordered, self._bridge is not None)
            self._inflight, self._inflight_genre, self.ordered = 1, genre, self.ordered + 1
        self._spawn(lambda: self._generate(model, genre))

    def _job(self, model: str, genre: str) -> GenJob:
        tags = ", ".join(t for t in (kn.GEN_STYLES[genre].tags, self._tags) if t)
        return GenJob(kind="instrumental", style_key=genre, title=theme_title(self._theme) or f"RoBBox {genre} set",
                      style_tags=tags, duration_s=kn.GEN_SET_DURATION_S, model=model)

    def _task(self, model: str, genre: str) -> Tuple[List[SetItem], str, float]:
        """Задача инструментала стиля (теги стиля и темы) → ``(варианты, task_id, кредиты)``."""
        result = generate_into_cache(self._provider, self._library, Budget(self._library), self._job(model, genre),
                                     theme=self._theme, set_spent=self.spent)
        items = [SetItem(str(s["id"]), result.task_id, str(s.get("title") or ""), "generated", genre)
                 for s in result.saved if s.get("success")]
        return items, result.task_id, result.credits

    def _generate(self, model: str, genre: str = "") -> None:
        started, items, reason, genre = self._clock(), [], "", genre or self.genre
        try:
            items, task_id, spent = (self._covers.task(genre, self._ask, self.spent) if genre.startswith(COVER_SLOT)
                                     else self._task(model, genre))
            self.task_ids.append(task_id)
            self._log.info(f"🎛️ [gen set] {self.set_id} задача {task_id} {genre} {model}: {len(items)} вар., "
                           f"{spent:g} кр., {self._clock() - started:.0f} с")
        except GenUnavailable as exc:
            reason, spent = exc.reason, 0.0
        except Exception as exc:  # noqa: BLE001 — сбой заказа не роняет сет: он доигрывает то, что есть
            reason, spent = "provider_down", 0.0
            self._log.error(f"❌ [gen set] {self.set_id}: {type(exc).__name__}: {exc}")
        with self._cond:
            self._ready.extend(items)
            self.spent, self._inflight = self.spent + spent, 0
            failed = self.gen_reason = reason or ("" if items else "provider_down")
            if failed == "rejected" and genre.startswith(COVER_SLOT):  # В17: без этой пьесы, сет заказывает дальше
                failed = self._drop_piece(genre)
            self._cond.notify_all()
        if failed:
            self._failed(model, genre)

    def _drop_piece(self, key: str) -> str:
        """Под замком: Suno не взял ни одну мелодию произведения слота ``key`` (В17) — его слоты впереди берут другие
        произведения темы (``SetCovers.drop``), фраза кода называет пьесу, следующий заказ — сразу (до ``notify``:
        ждущий трек видит новый заказ своего слота). Отказ кредитов не тратит — задача не в счёт ``GEN_SET_MAX_TASKS``."""
        before = self._plan[self._taken:]
        work, self._plan, others = self._covers.drop(key, self._plan, self._taken)
        self.gen_reason, self.ordered = "", self.ordered - 1
        self._log.warning(f"⚠️ [gen set] {self.set_id} «{work.title}»: Suno не взял ни одну мелодию — слоты {before} "
                          f"→ {self._plan[self._taken:]}")
        _say(self._speak, cover_rejected_text(work, others))
        self.order_more()  # Condition — на RLock: заказ из-под того же замка
        return ""

    def _failed(self, model: str, genre: str) -> None:
        self._log.warning(f"⚠️ [gen set] {self.set_id} генерация {genre} {model} не вышла: {self.gen_reason}")
        _say(self._speak, failed_text(genre, self.gen_reason))


#: Сколько мост ждёт результат генерации (кавер, песня: TTS + загрузка + Suno) до финала без него.
HOLD_WAIT_S = GEN_TIMEOUT_S + 120.0
#: Запас до доли входа результата: cue исполняет программу за ``ALIGN_LEAD_BEATS`` до неё, плюс доля на поток клока.
CUT_IN_CUE_BEATS = ALIGN_LEAD_BEATS + 1.0
_BAR = 4.0  # доли такта (``model.BEATS_PER_BAR``)


def cut_in_beat(position: Optional[Tuple[float, float, float]], cue: float = CUT_IN_CUE_BEATS) -> Optional[float]:
    """Доля входа готового результата поверх моста: ближайший такт, до которого успевает cue, — а не граница прохода
    формы моста (13.10: песня после клуба вставала до ~2 мин после готовности; 14.10: после моста-файла — 52 с).
    ``position`` — ``PlayerOwner.position`` моста: ``(доля клока, доля старта, форма)``; ``None`` — мост не играет.
    Штатный ``nearly_finished`` моста, попавший между постановкой и входом, видит поставленный результат и ничего не
    продлевает (``SetSession._on_nearly_finished``)."""
    if position is None:
        return None
    now = position[0]
    return (math.floor((now + cue) / _BAR) + 1) * _BAR


def cut_in_leave(position: Tuple[float, float, float], at: float, file: bool, bpm: int) -> Tuple[float, float]:
    """``(leave_at, fade)`` входа на доле ``at``: мост уходит через кроссфейд. Мост-файл: его нота звучит всю ``sus`` и
    после снятия плеера — её гасит огибающая деки (``fade`` долей, ``render.audio.deck_fade_args``), а плеер снимается
    до начала следующего прохода (новая нота прохода звучала бы полностью поверх результата); вход ровно на границе
    прохода — нота уже гаснет своим хвостом, огибающая деки не нужна."""
    xfade = xfade_beats(bpm)
    if not file:
        return at + xfade, 0.0
    _, start, form = position
    boundary = start + math.ceil((at - start) / form) * form
    return min(at + xfade, boundary), (xfade if boundary > at else 0.0)


class BridgeHold:
    """Мост держит звук, пока генерация не вернёт результат или провал (правило координатора 13.10: живой прогон —
    мост кавера доиграл форму за 22 с, кавер встал через 64 с, между ними 44 с тишины). Одна реализация для кавера
    пьесы (``PieceCover``) и песни голосом робота (``SongOrder``): ``SetSession`` из двух треков — мост (его форма
    повторяется, пока ждём) и результат, который входит кроссфейдом ``GEN_XFADE_S``; провал или таймаут — финал:
    мост ещё одним проходом и конец, без тишины посреди. ``model`` — ``Track`` синтеза или ``AudioTrack`` кэша моста
    (финал собирается из него на другой деке); ``program`` — уже собранная программа моста (дека A)."""

    def __init__(self, owner: Any, model: Any, program: Any, library: Callable[[], Any], *,
                 facts: Optional[Dict[str, Any]] = None, logger: Any = _LOG,
                 on_live: Callable[[], None] = lambda: None, run: Callable[..., Any] = subprocess.run,
                 wait_s: float = HOLD_WAIT_S, submit: Optional[Callable[[Callable[[], None]], None]] = None) -> None:
        from ..session import SetSession

        self._owner, self._model, self._program, self._library = owner, model, program, library
        self._facts, self._log, self.on_live, self._run, self._wait = dict(facts or {}), logger, on_live, run, wait_s
        self._result: Optional[Tuple[str, str]] = None  # (id трека кэша, название)
        self._event = threading.Event()
        self.bridge_id = program.track_id
        self.live_id = ""
        self._submit = submit or _thread
        self.session = SetSession(owner, self._source, set_id=f"hold:{self.bridge_id}", bpm=program.bpm,
                                  logger=logger, tracks=2, fields=self._fields, on_track_started=self._started,
                                  submit=self._then_cut_in, entry_lead=False)

    def start(self) -> Dict[str, Any]:
        return self.session.start()

    def on_deck(self) -> bool:
        """На деке мост или результат этого ожидания (звучит или ждёт ``started``)."""
        now = self._owner.now_playing()
        ids = {self.bridge_id, self.live_id} - {""}
        return self.session.active and bool(ids & {now.get("track_id"), now.get("pending")})

    def deliver(self, library_id: str, title: str = "") -> bool:
        """Результат готов: войдёт кроссфейдом на ближайшем такте. ``False`` — деку занял другой заказ."""
        return self._settle((library_id, title))

    def fail(self) -> bool:
        """Генерация не вышла: мост доиграет финалом. ``False`` — моста на деке уже нет."""
        return self._settle(None)

    def _settle(self, result: Optional[Tuple[str, str]]) -> bool:
        ours = self.on_deck()
        if not ours:
            self.session.detach()
        self._result = result if ours else None
        self._event.set()
        return ours

    def _then_cut_in(self, prepare: Callable[[], None]) -> None:
        """Подготовка результата (``SetSession._compose_next`` ждёт ``deliver``/``fail``) — и сразу вход по такту."""
        def run() -> None:
            prepare()
            self._cut_in()

        self._submit(run)

    def _cut_in(self) -> None:
        """Результат в очереди владельца: входит на ближайшем такте кроссфейдом ``GEN_XFADE_S`` (огибающая файла).
        Синтез-мост уходит в конце кроссфейда; мост-файл гаснет огибающей своей деки (:func:`cut_in_leave`)."""
        from rob_box_music.model import AudioTrack

        if self._result is None or not self.live_id:
            return
        position = self._owner.position(self.bridge_id)
        at = cut_in_beat(position)
        if at is None:
            return
        leave, fade = cut_in_leave(position, at, isinstance(self._model, AudioTrack), self._program.bpm)
        if self._owner.advance(at, dj=self._fields(2), leave_at=leave, once=True, fade_beats=fade):
            self._log.info(f"🎚️ [bridge] {self.bridge_id}: результат {self._result[0]} входит на доле {at:g} "
                           f"кроссфейдом {kn.GEN_XFADE_S:g} с, не ждёт конца формы моста"
                           + (f"; мост-файл гаснет за {fade:g} долей" if fade else ""))

    def _fields(self, track_no: int) -> Dict[str, Any]:
        if track_no == 1 or self._result is None:
            return {**self._facts, "enabled": False}
        return {"enabled": False, "title": self._result[1], "source": "generated", "library_id": self._result[0]}

    def _started(self, track_id: str) -> str:
        if track_id == self.live_id and self._result is not None:
            self.on_live()
            return f"hold=live {self._result[0]}"
        return "hold=bridge" if track_id == self.bridge_id else "hold=finale"

    def _source(self, track_no: int, deck: str) -> Any:
        if track_no == 1:
            return self._program
        if not self._event.wait(self._wait):
            self._log.warning(f"⚠️ [bridge] {self.bridge_id}: результата нет за {self._wait:.0f} с — финал моста")
        track_id = f"hold:{self.bridge_id}:02:{deck}"
        if self._result is not None:
            try:
                track = replace(audio_track(self._library().get_track_info(self._result[0]), track_id, run=self._run),
                                bpm_nominal=int(self._program.bpm))
            except (AudioUnavailable, KeyError) as exc:  # файл не звучит — финал моста, а не вечный мост
                self._log.warning(f"⚠️ [bridge] {self.bridge_id}: результат {self._result[0]} не звучит: {exc}")
                self._result = None
                return self._finale(track_id, deck)
            self._library().increment_play_count(self._result[0])
            self.live_id = track_id
            return track
        return self._finale(track_id, deck)

    def _finale(self, track_id: str, deck: str) -> Any:
        """Мост ещё одним проходом на другой деке — и конец (провал, таймаут, другой заказ)."""
        from rob_box_music.model import AudioTrack
        from rob_box_music.render.renardo import render

        model = replace(self._model, track_id=track_id)
        return model if isinstance(model, AudioTrack) else render(model, deck)


#: Мост сета: ``style -> (deck -> Track, bpm)`` — трек 1 синтезом (собирает ``DjSetTool``).
Bridge = Callable[[str], Tuple[Callable[[str], Any], int]]


class GenSetMaker:
    """Сет из генерации для ``dj_set`` (ADR-0158 PR-8): путь сета решает чойсер (``SourceRouter.route(for_set=True)``),
    сет — ``SetSession`` с источником :class:`GenSet`. ``library`` — ленивый доступ к кэшу (поднимается позже движка),
    ``balance`` — последний известный баланс KIE (``CoverMaker.balance``, сеть заказ не ждёт); ``cover`` —
    ``CoverMaker`` пьесы: им же исполняются каверы материала темы в треках сета (В14, одна реализация)."""

    def __init__(self, provider: Any, library: Callable[[], Any], chooser: Any, *,
                 balance: Callable[[], Optional[float]] = lambda: None, log: Callable[[], Any] = lambda: _LOG,
                 cover: Any = None) -> None:
        from ..source_route import SourceRouter

        self._provider, self._library, self._balance, self._log, self._cover = provider, library, balance, log, cover
        self.router = SourceRouter(chooser, library, log=log, sets=self)

    def ready(self) -> bool:
        return bool(getattr(self._provider, "configured", False)) and self._library() is not None

    def cover_ready(self) -> bool:
        """Каверы в треках сета можно заказывать: исполнитель кавера подключён и готов (ключ, кэш)."""
        return self._cover is not None and self._cover.ready()

    def balance(self) -> Optional[float]:
        return self._balance()

    def plan(self, theme: str, style: str, heard_text: Optional[str], length: int, *, named: bool = False,
             styles: Sequence[str] = (), minutes: Optional[int] = None,
             covers: Sequence[Tuple[CoverWork, Any]] = ()) -> "SetRoutes":
        """Стили по трекам, длина и путь каждого трека — кодом (ADR-0148, решение Шифу 14.10): :func:`slot_genres`,
        :func:`slot_count`, путь — ``SourceRouter.route_slots``; ``covers`` — наш узнаваемый материал темы
        ``(произведение, трек с мелодией)`` для каверов (В14, ``DjSetTool.theme_covers``)."""
        from rob_box_music.source import source_ask, theme_tags
        from rob_box_voice.core.set_length_words import heard_set_seconds

        genres, unknown = slot_genres(heard_text, styles, style)
        seconds = heard_set_seconds(heard_text or "") or (float(minutes) * 60 if minutes else None)
        slots = tuple(genres[i % len(genres)] for i in range(slot_count(len(genres), length, named, seconds)))
        ask = source_ask(heard_text or theme)
        asks = [replace(ask, genre=g, vocal=False, robot_voice=False) for g in slots]
        tags = theme_tags(theme)
        routes = self.router.route_slots(asks, tags, [w for w, _t in covers], seconds=seconds, minimum=len(genres))
        slots, asks = slots[:len(routes)], asks[:len(routes)]
        themed = themed_count(tags, routes)
        background = self._background_track(asks, routes, tags, themed, bool(covers))
        self._log().info(f"🎛️ [gen set] слоты: тема={theme or '—'!r} стили={list(genres)} треков={len(slots)} "
                         f"{theme_note(tags, routes, themed, background)} "
                         f"время={f'{seconds:.0f} с' if seconds else '—'} тема-теги={tags or '—'} "
                         f"непонято={list(unknown)} каверы={[w.material for w, _t in covers]}")
        return SetRoutes(slots, tuple(routes), unknown, tags, tuple(covers), background)

    def start(self, plan: "SetRoutes", owner: Any, *, theme: str, persona: Optional[str], set_id: str,
              bridge: Bridge, line_ask: Any = None, speak: Optional[Callable[[str], Any]] = None,
              replace_set: Callable[[], None] = lambda: None) -> Dict[str, Any]:
        """Запустить сет по путям треков ``plan`` (кэш и генерация): ``{ok, …, source_message, session}`` — фраза от
        кода. Треков с путём нет — отказ с фразой кода, идущий сет не трогается (синтез-запас решает ``dj_set``);
        иначе ``replace_set`` снимает идущий сет до старта нового."""
        from rob_box_music.source import decline_text

        played = plan.played()
        if not played:
            return {"ok": False, "reason": DECLINED, "route": plan.routes[0].id, "message": decline_text(plan.routes[0])}
        replace_set()
        genres, reason, cache = [plan.genres[i] for i in played], plan.gen_reason(), plan.cache()
        first = plan.routes[played[0]]
        logger = self._log()
        made, wait = self._waiting(plan, first.kind != CACHE and not reason, bridge, genres[0], set_id, logger)
        keys = plan.keys()
        gen = GenSet(provider=self._provider, library=self._library(), ask=first.ask, genre=genres[0], theme=theme,
                     set_id=set_id, bpm=made[1] if made else kn.AUDIO_DEFAULT_BPM, tracks=len(played), cache=cache,
                     bridge=made[0] if made else None, gen_reason=reason, speak=speak, log=logger,
                     publish=getattr(owner, "update_dj", None), genres=genres, theme_tags=plan.theme_tags,
                     keys=[keys[i] for i in played],
                     covers=SetCovers(plan.covers(), self._set_cover(plan), plan.pieces()))
        line = _LineBox(bridge_facts(genres[0], task_model(0, True)), line_ask, logger) if wait == "да" else None
        session = _gen_session(owner, gen, theme, persona, logger)
        result = session.start()
        if wait == "фон":
            head = background_text(theme or plan.theme_tags)
        else:
            head = line.text() if line else launch_text(genres, gen.slots, min(len(cache), gen.slots), reason)
        message = start_message(head, plan)
        logger.info(f"🎛️ [gen set] {set_id} {[r.id for r in plan.routes]}: {genres} {gen.slots} "
                    f"{tracks_word(gen.slots)}, кэш {len(cache)}, мост={wait} gen={reason or 'ok'} "
                    f"→ {result.get('ok')}")
        return {**result, "theme": theme, "style": genres[0], "styles": list(gen.genres), "source": "generated",
                "tracks": gen.slots, "cached": min(len(cache), gen.slots), "route": first.id,
                "routes": [r.id for r in plan.routes], "source_message": message,
                "session": session if result.get("ok") else None}

    def _background_track(self, asks: Sequence[SourceAsk], routes: Sequence[Any], tags: str, themed: int,
                          covers: bool) -> Optional[CacheTrack]:
        """Фон сета темы: по теме в кэше ничего (K = 0), каверов нет, первый трек — заказ → трек кэша стиля сета
        (``SourceRouter.style_cache``); иначе ``None``."""
        if not tags or themed or covers or not routes or routes[0].kind != GENERATE:
            return None
        return self.router.style_cache(asks[0])

    def _waiting(self, plan: "SetRoutes", waits: bool, bridge: Bridge, genre: str, set_id: str,
                 log: Any) -> Tuple[Optional[Tuple[Callable[[str], Any], int]], str]:
        """Что играет, пока пишется первый трек (кэш его не закрыл, ``waits``): тихий фон кэша стиля (сет темы,
        K = 0) — «фон», иначе мост синтезом — «да»; ждать нечего — ``(None, "—")``."""
        if not waits:
            return None, "—"
        quiet = self._background(plan.background, set_id, log) if plan.background is not None else None
        return (quiet, "фон") if quiet else (bridge(bridge_style(genre)), "да")

    def _background(self, track: CacheTrack, set_id: str, log: Any) -> Optional[Tuple[Callable[[str], Any], int]]:
        """Фон сета темы (K = 0): файл кэша стиля тише (``SET_BACKGROUND_GAIN``) и не длиннее ``SET_BACKGROUND_S`` за
        проход, темп — ``AUDIO_DEFAULT_BPM``; декодируется сразу — не вышло — ``None`` (мост синтезом)."""
        try:
            info = self._library().get_track_info(track.id)
            if not info.get("success"):
                raise AudioUnavailable("no_row", str(info.get("error")))
            audio = audio_track(info, f"{set_id}:bg")
        except AudioUnavailable as exc:
            log.warning(f"⚠️ [gen set] {set_id} фон {track.id} не звучит: {exc} — мост синтезом")
            return None
        bpm = kn.AUDIO_DEFAULT_BPM
        quiet = replace(audio, bpm_nominal=bpm, gain=kn.SET_BACKGROUND_GAIN,
                        duration_s=min(audio.duration_s, kn.SET_BACKGROUND_S))
        log.info(f"🎧 [gen set] {set_id} фон: {track.id} «{track.title}» {quiet.duration_s:.0f} с за проход, "
                 f"громкость {kn.SET_BACKGROUND_GAIN:g}")
        return (lambda deck: replace(quiet, track_id=f"{set_id}:00:{deck}")), bpm

    def _set_cover(self, plan: "SetRoutes") -> Optional[Callable[[CoverWork, SourceAsk, float], Any]]:
        """Исполнитель кавера трека сета: тот же ``CoverMaker.make`` с мелодиями материала темы, оба варианта; отказ
        Suno — следующая попытка ``COVER_TRIES`` (``cover.cover_tries``, В17)."""
        from .cover import cover_tries

        if self._cover is None:
            return None
        sources = {work: srcs for work, srcs in plan.theme_covers}  # у двух пьес может быть одна партитура
        return lambda work, ask, spent: cover_tries(self._cover.make, sources[work], work, ask, set_spent=spent,
                                                    early=False, log=self._log())


def slot_genres(heard_text: Optional[str], styles: Sequence[str], style: str) -> Tuple[Tuple[str, ...], Tuple[str, ...]]:
    """``(стили по порядку, непонятые слова)``: слова человека (``source.set_genres``) важнее ``styles`` LLM, те —
    стиля сета (``dj_set``: слова реплики, ключ LLM, строка темы); стиль не из ``GEN_STYLES`` — ``DEFAULT_STYLE``."""
    from rob_box_music.source import set_genres

    heard, unknown = set_genres(heard_text or "")
    llm = tuple(g for g in dict.fromkeys(styles) if g in kn.GEN_STYLES)
    return heard or llm or ((style if style in kn.GEN_STYLES else kn.DEFAULT_STYLE),), unknown


def slot_count(styles: int, length: int, named: bool, seconds: Optional[float]) -> int:
    """Треков сета (запас просьб): время («10 минут») — ``MAX_TRACKS``, сколько из них играет, решает набор по
    плановой длине пути (``route_slots(seconds=…)``, ``source.route_seconds``); число, названное человеком, — как
    есть; иначе ``length`` (по умолчанию сета), не меньше числа стилей. Зажато в 1..``MAX_TRACKS``."""
    from rob_box_music.set_plan import MAX_TRACKS

    if seconds:
        n = MAX_TRACKS
    else:
        n = length if named else max(styles, length)
    return min(max(1, n), MAX_TRACKS)


def start_message(head: str, plan: "SetRoutes") -> str:
    """Фраза запуска сета: голова (мост или ``launch_text``) и что из просьбы не сыграет (:func:`set_notes`)."""
    return " ".join(p for p in (head, set_notes(plan.unknown, plan.dropped())) if p)


def _gen_session(owner: Any, gen: GenSet, theme: str, persona: Optional[str], logger: Any) -> Any:
    from ..session import SetSession

    return SetSession(owner, gen.next_track, set_id=gen.set_id, bpm=gen.bpm, logger=logger,
                      tracks=gen.slots + gen.bridged, on_track_started=gen.on_started,
                      dj={"theme": theme or genre_name(gen.genre), "persona": persona, "style": gen.genre,
                          "styles": list(gen.genres), "source": "generated"})


@dataclass(frozen=True)
class SetRoutes:
    """План сета из кэша и генерации: стиль и путь (``source.Route``) каждого трека, непонятые слова стилей, теги
    темы для заказа."""

    genres: Tuple[str, ...]
    routes: Tuple[Any, ...]
    unknown: Tuple[str, ...] = ()
    theme_tags: str = ""
    theme_covers: Tuple[Tuple[CoverWork, Any], ...] = ()  # материал темы (В14, В17): (произведение, мелодии попыток)
    background: Optional[CacheTrack] = None  # фон сета темы, пока пишется первый трек (K = 0, #3775)

    def keys(self) -> List[str]:
        """Ключ слота каждого трека: кавер (новый или из кэша) — ``cover|<стиль>|<пьеса>``, иначе стиль трека."""
        def key(genre: str, route: Any) -> str:
            if gen_kind(route.id) == "cover" and route.work is not None:
                return cover_slot(genre, route.work.title)
            if route.kind == CACHE and getattr(route.track, "kind", "") == "cover":
                return cover_slot(route.track.style_key, route.track.work_id)
            return genre

        return [key(g, r) for g, r in zip(self.genres, self.routes)]

    def covers(self) -> Dict[str, CoverWork]:
        """Произведение кавера по ключу слота (задачи генерации каверов сета)."""
        return {key: route.work for key, route in zip(self.keys(), self.routes)
                if gen_kind(route.id) == "cover" and route.work is not None}

    def pieces(self) -> List[CoverWork]:
        """Произведения темы по порядку (В17: ими закрываются слоты произведения, которое Suno не взял)."""
        return [work for work, _sources in self.theme_covers]

    def cache(self) -> List[CacheTrack]:
        """Треки кэша, выбранные путями треков (в сет — именно они)."""
        return [r.track for r in self.routes if r.kind == CACHE and r.track is not None]

    def played(self) -> List[int]:
        """Треки, которые сыграет сет: путь — кэш или генерация (синтез и отказ — не в этом сете)."""
        return [i for i, r in enumerate(self.routes) if r.kind in (CACHE, GENERATE)]

    def gen_reason(self) -> str:
        """Почему генерации нет («» — хоть один трек генерируется)."""
        if any(r.kind == GENERATE for r in self.routes):
            return ""
        return next((r.gen_reason for r in self.routes if r.gen_reason), "")

    def dropped(self) -> List[Tuple[str, str]]:
        """Стили, которых в сете не будет, с причиной (стиль, у которого есть трек, не выпал)."""
        played = {self.genres[i] for i in self.played()}
        out: Dict[str, str] = {}
        for genre, route in zip(self.genres, self.routes):
            if genre not in played:
                out.setdefault(genre, route.gen_reason)
        return list(out.items())

    def synth_style(self) -> Optional[str]:
        """Стиль синтеза-запаса: первый стиль сета, который синтез играет; нет — ``None``."""
        from rob_box_music.source import synth_fit, synth_style

        return next((synth_style(g) for g in dict.fromkeys(self.genres) if synth_fit(g) >= kn.SYNTH_FIT_MIN), None)


class _LineBox:
    """Окраска фразы моста LLM в фоне — параллельно со стартом сета; ``text`` ждёт её не дольше дедлайна."""

    def __init__(self, facts: Dict[str, str], ask: Any, log: Any) -> None:
        self._facts, self._line = facts, ""
        self._done = threading.Event()
        if ask is None:
            self._line = bridge_template(facts)
            self._done.set()
            return

        def run() -> None:
            try:
                self._line = bridge_line(facts, ask, log=log)
            finally:
                self._done.set()

        _thread(run)

    def text(self) -> str:
        self._done.wait(BRIDGE_LINE_DEADLINE_S + 1.0)
        return self._line or bridge_template(self._facts)


__all__ = ["BRIDGE_LINE_DEADLINE_S", "Bridge", "GenSet", "GenSetMaker", "SetCovers", "SetItem", "SetRoutes", "bridge_facts", "bridge_line",
           "background_text", "theme_why", "bridge_style", "bridge_template", "failed_text", "genre_name", "launch_text", "pick_item", "respread", "set_head", "set_notes",
           "supply_of", "synth_set_note", "task_model", "tasks_needed", "validate_bridge"]
