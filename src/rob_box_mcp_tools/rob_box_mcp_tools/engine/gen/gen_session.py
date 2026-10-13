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
* Провал генерации (В3): заказов больше нет, сет доигрывает то, что есть; фраза причины — от кода.

Стрим до ``success`` (замер 13.10, ``f5aa62a5…``, V6 × 120 с): ``stream_audio_url`` на стадии ``text`` (32 с) отдал весь
файл за 6 с, ``success`` — на 50 с; после ``success`` тот же URL отдаёт те же байты (не пусто). Выигрыш 12–18 с нужен
только первому треку сета, а его закрывает мост — здесь стрим не используется (см. описание PR).
"""

from __future__ import annotations

import logging
import subprocess
import threading
import time
from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.dj_line import tracks_word
from rob_box_music.source import REASON_WORDS, CacheTrack, SourceAsk

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


def launch_text(genre: str, tracks: int, cached: int, gen_reason: str) -> str:
    """Фраза запуска сета без моста — от кода: сколько из сочинённого раньше, допишет ли остальное (G3)."""
    head = f"{genre_name(genre)[:1].upper()}{genre_name(genre)[1:]}-сет"
    if cached >= tracks:
        return f"{head} из сочинённого раньше: {tracks} {tracks_word(tracks)}."
    if not gen_reason:
        return f"{head} на {tracks} {tracks_word(tracks)}: {cached} из сочинённого раньше, остальные допишу по ходу."
    words = REASON_WORDS.get(gen_reason, gen_reason)
    return f"{head} из сочинённого раньше: только {cached} {tracks_word(cached)} — дописать не могу: {words}."


def failed_text(genre: str, reason: str) -> str:
    return f"Дописать {genre_name(genre)} не вышло: {REASON_WORDS.get(reason, reason)} — доиграю из того, что есть."


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
                 publish: Optional[Callable[[str, Dict[str, Any]], Any]] = None) -> None:
        self._provider, self._library, self._ask, self.genre, self._theme = provider, library, ask, genre, theme
        self._publish = publish  # ``PlayerOwner.update_dj``: название играющего трека — в снимок («что играет»)
        self.set_id, self.bpm, self.slots = set_id, int(bpm), int(tracks)
        self._bridge, self.gen_reason, self._speak, self._log = bridge, gen_reason, speak, log
        self._spawn, self._run, self._clock = spawn, run, clock
        self._ready: List[SetItem] = [SetItem(t.id, t.group or t.id, t.title) for t in cache]
        self._used: Set[str] = set()
        self._prev, self._taken, self._inflight, self.ordered, self.spent = "", 0, 0, 0, 0.0
        self._playing_s: Optional[float] = None  # длина играющего файла; None — играет мост (продлевается)
        self._items: Dict[str, Tuple[SetItem, str]] = {}  # track_id деки → (трек, как выбран)
        self.task_ids: List[str] = []
        self._cond = threading.Condition()

    @property
    def bridged(self) -> bool:
        return self._bridge is not None

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
        end = self._clock() + wait_s
        with self._cond:
            while True:
                item, how = pick_item(self._ready, self._used, self._prev)
                left = end - self._clock()
                if how == "ok" or not self._inflight or left <= 0:
                    break
                self._cond.wait(left)
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
        self._log.info(f"🎧 [gen set] {self.set_id} трек {track_no}: {item.origin} {item.id} «{item.title}» "
                       f"({how}, задача {item.group}) {track.duration_s:.0f} с")
        return track

    def order_more(self) -> None:
        """Заказать следующую задачу, если недостающих треков больше, чем готовых и заказанных (одна за раз)."""
        with self._cond:
            supply = supply_of(self._ready, self._used, self._inflight)
            if self.gen_reason or self._inflight or not tasks_needed(self.slots - self._taken, supply, self.ordered):
                return
            model = task_model(self.ordered, self._bridge is not None)
            self._inflight, self.ordered = 1, self.ordered + 1
        self._spawn(lambda: self._generate(model))

    def _job(self, model: str) -> GenJob:
        name = kn.GEN_STYLES[self.genre]
        return GenJob(kind="instrumental", style_key=self.genre, title=f"RoBBox {self.genre} set",
                      style_tags=name.tags, duration_s=kn.GEN_SET_DURATION_S, model=model)

    def _generate(self, model: str) -> None:
        started, items, reason = self._clock(), [], ""
        try:
            result = generate_into_cache(self._provider, self._library, Budget(self._library), self._job(model),
                                         theme=self._theme, set_spent=self.spent)
            items = [SetItem(str(s["id"]), result.task_id, str(s.get("title") or ""), "generated")
                     for s in result.saved if s.get("success")]
            self.task_ids.append(result.task_id)
            self._log.info(f"🎛️ [gen set] {self.set_id} задача {result.task_id} {model}: {len(items)} вар., "
                           f"{result.credits:g} кр., {self._clock() - started:.0f} с")
            spent = result.credits
        except GenUnavailable as exc:
            reason, spent = exc.reason, 0.0
        except Exception as exc:  # noqa: BLE001 — сбой заказа не роняет сет: он доигрывает то, что есть
            reason, spent = "provider_down", 0.0
            self._log.error(f"❌ [gen set] {self.set_id}: {type(exc).__name__}: {exc}")
        with self._cond:
            self._ready.extend(items)
            self.spent, self._inflight = self.spent + spent, 0
            self.gen_reason = reason or ("" if items else "provider_down")
            self._cond.notify_all()
        if self.gen_reason:
            self._failed(model)

    def _failed(self, model: str) -> None:
        self._log.warning(f"⚠️ [gen set] {self.set_id} генерация {model} не вышла: {self.gen_reason}")
        if self._speak is not None:
            self._speak(failed_text(self.genre, self.gen_reason))


#: Сколько мост ждёт результат генерации (кавер, песня: TTS + загрузка + Suno) до финала без него.
HOLD_WAIT_S = GEN_TIMEOUT_S + 120.0


class BridgeHold:
    """Мост держит звук, пока генерация не вернёт результат или провал (правило координатора 13.10: живой прогон —
    мост кавера доиграл форму за 22 с, кавер встал через 64 с, между ними 44 с тишины). Одна реализация для кавера
    пьесы (``PieceCover``) и песни голосом робота (``SongOrder``): ``SetSession`` из двух треков — мост (его форма
    повторяется, пока ждём) и результат, который входит кроссфейдом ``GEN_XFADE_BARS``; провал или таймаут — финал:
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
        extra = {"submit": submit} if submit is not None else {}
        self.session = SetSession(owner, self._source, set_id=f"hold:{self.bridge_id}", bpm=program.bpm,
                                  logger=logger, tracks=2, fields=self._fields, on_track_started=self._started,
                                  **extra)

    def start(self) -> Dict[str, Any]:
        return self.session.start()

    def on_deck(self) -> bool:
        """На деке мост или результат этого ожидания (звучит или ждёт ``started``)."""
        now = self._owner.now_playing()
        ids = {self.bridge_id, self.live_id} - {""}
        return self.session.active and bool(ids & {now.get("track_id"), now.get("pending")})

    def deliver(self, library_id: str, title: str = "") -> bool:
        """Результат готов: войдёт кроссфейдом на конце прохода моста. ``False`` — деку занял другой заказ."""
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
    ``balance`` — последний известный баланс KIE (``CoverMaker.balance``, сеть заказ не ждёт)."""

    def __init__(self, provider: Any, library: Callable[[], Any], chooser: Any, *,
                 balance: Callable[[], Optional[float]] = lambda: None, log: Callable[[], Any] = lambda: _LOG) -> None:
        from ..source_route import SourceRouter

        self._provider, self._library, self._balance, self._log = provider, library, balance, log
        self.router = SourceRouter(chooser, library, log=log, sets=self)

    def ready(self) -> bool:
        return bool(getattr(self._provider, "configured", False)) and self._library() is not None

    def balance(self) -> Optional[float]:
        return self._balance()

    def route(self, theme: str, style: str, heard_text: Optional[str]) -> Any:
        """Путь сета или None — сет синтезом, как раньше: жанр синтез играет («у нас норм играет»), генерацию не
        просили. Жанр — из слов реплики, иначе стиль сета, если он есть в ``GEN_STYLES``."""
        from rob_box_music.source import SYNTH, source_ask, synth_fit

        ask = source_ask(heard_text or theme)
        genre = ask.genre or (style if style in kn.GEN_STYLES else "")
        if not genre or (synth_fit(genre) >= kn.SYNTH_FIT_MIN and not (ask.generated or ask.fresh)):
            return None
        route = self.router.route(replace(ask, genre=genre, vocal=False, robot_voice=False), for_set=True)
        return None if route.kind == SYNTH else route

    def start(self, route: Any, owner: Any, *, theme: str, persona: Optional[str], length: int, set_id: str,
              bridge: Bridge, line_ask: Any = None, speak: Optional[Callable[[str], Any]] = None,
              replace_set: Callable[[], None] = lambda: None) -> Dict[str, Any]:
        """Запустить сет пути ``route`` (кэш или генерация): ``{ok, …, source_message, session}`` — фраза от кода.
        Играть нечем (отказ пути, кэш жанра пуст и генерации нет) — отказ с фразой кода, идущий сет не трогается;
        иначе ``replace_set`` снимает идущий сет до старта нового."""
        from rob_box_music.source import DECLINE, decline_text, set_cache

        from ..session import SetSession

        ask, reason = route.ask, route.gen_reason
        cache = set_cache(ask, self.router.context(ask, for_set=True))
        if route.kind == DECLINE or (reason and not cache):
            return {"ok": False, "reason": DECLINED, "route": route.id, "message": decline_text(route)}
        replace_set()
        tracks = length if not reason else min(length, len(cache))
        style = bridge_style(ask.genre)
        made = bridge(style) if not cache and not reason else None
        logger = self._log()
        gen = GenSet(provider=self._provider, library=self._library(), ask=ask, genre=ask.genre, theme=theme,
                     set_id=set_id, bpm=made[1] if made else kn.AUDIO_DEFAULT_BPM, tracks=tracks, cache=cache,
                     bridge=made[0] if made else None, gen_reason=reason, speak=speak, log=logger,
                     publish=getattr(owner, "update_dj", None))
        line = _LineBox(bridge_facts(ask.genre, task_model(0, True)), line_ask, logger) if made else None
        session = SetSession(owner, gen.next_track, set_id=set_id, bpm=gen.bpm, logger=logger, tracks=tracks + bool(made),
                             dj={"theme": theme or genre_name(ask.genre), "persona": persona, "style": ask.genre,
                                 "source": "generated"}, on_track_started=gen.on_started)
        result = session.start()
        message = line.text() if line else launch_text(ask.genre, tracks, min(len(cache), tracks), reason)
        logger.info(f"🎛️ [gen set] {set_id} {route.id}: {ask.genre} {tracks} {tracks_word(tracks)}, кэш {len(cache)}, "
                    f"мост={style if made else '—'} gen={reason or 'ok'} → {result.get('ok')}")
        return {**result, "theme": theme, "style": ask.genre, "source": "generated", "tracks": tracks,
                "cached": min(len(cache), tracks), "route": route.id, "source_message": message,
                "session": session if result.get("ok") else None}


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


__all__ = ["BRIDGE_LINE_DEADLINE_S", "Bridge", "GenSet", "GenSetMaker", "SetItem", "bridge_facts", "bridge_line", "bridge_style",
           "bridge_template", "failed_text", "genre_name", "launch_text", "pick_item", "supply_of", "task_model", "tasks_needed",
           "validate_bridge"]
