"""Реплика диджея на каждом переходе сета (ADR-0149 §12 В2 — решение Шифу 06.10; I23; ADR-0148).

На компоновке трека N (``SetPlanBox.compose_mark``: трек 1 — на старте сета, N+1 — когда он готовится в очередь)
:class:`TransitionLines` в своём потоке собирает факты трека из плана (``rob_box_music.dj_line.LineFacts``: номер и
длина сета, тема, человеческое название мелодии-хука из RTTTL-библиотеки, фаза дуги энергии) и спрашивает LLM
раскраску (тот же провайдер, клиент и circuit breaker, что у ризонера плана — ``SetReasoner.ask``). На ``started``
трека — ровно одна фраза (I23): готова и прошла валидатор фраза LLM — она, иначе шаблон кода с теми же фактами.
Музыку реплика не трогает: фраза уходит в ``speak_text`` (тот же путь в TTS, что у LLM) поверх сета, без паузы и
приглушения; ``speak_text`` в потоке, переход и клок его не ждут.

Те же факты играющего трека (человеческое название мелодии, следующие мелодии по плану, части темы без мелодий)
уходят в снимок плеера (``PlayerOwner.update_dj`` → ``dj`` в ``/voice/music/state`` → ``<music_state>``) и в
``dj_set.status`` (``get_music_state``): на «что играет / когда будет X» отвечает код по фактам, а не догадка LLM
по теме (06.10: «Марио реально играет» при хуках tetris_2/spacecha).
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

from rob_box_music import dj_line as dl
from rob_box_music.set_plan import SetPlan

_LOG = logging.getLogger(__name__)

#: Сколько поток реплики на ``started`` ждёт фразу LLM (трек 1: LLM спрошена за миг до старта; дальше — за трек).
GRACE_S = 3.0
#: Дедлайн одного вызова LLM за фразу: опоздала — шаблон.
LINE_DEADLINE_S = 20.0
#: Сколько следующих мелодий плана показывать в снимке и в ответе «что дальше».
UPCOMING = 3

#: ``(system, user, tool, deadline_s) -> (outcome, response, detail)`` — ``SetReasoner.ask``.
Ask = Callable[[str, str, Dict[str, Any], float], Tuple[str, Any, str]]
#: Человеческие названия мелодий по id: ``ids -> {id: название}``.
Titles = Callable[[Iterable[str]], Dict[str, str]]


def _thread(fn: Callable[[], None]) -> None:
    threading.Thread(target=fn, name="rbx-dj-line", daemon=True).start()


@dataclass
class _Entry:
    no: int
    seed: int
    plan: SetPlan
    hook_id: Optional[str]
    facts: Optional[dl.LineFacts] = None
    llm: Optional[str] = None
    known: threading.Event = field(default_factory=threading.Event)  # факты собраны
    ready: threading.Event = field(default_factory=threading.Event)  # и LLM ответила (или нет)


def line_payload(response: Any) -> Any:
    """Строка из вызова ``submit_dj_line``; без вызова — ``LineInvalid``."""
    for call in getattr(response, "tool_calls", ()) or ():
        if call.name == dl.SUBMIT_TOOL:
            return dl.payload_line(dict(call.arguments))
    raise dl.LineInvalid("no_call", f"нет вызова {dl.SUBMIT_TOOL}")


class TransitionLines:
    """Реплики и факты одного сета. ``ask=None`` — LLM не спрашивается, звучат шаблоны; ``speak=None`` — переходы
    молчат (``music_v2_dj_lines: false``), но факты трека всё равно уходят в снимок (``publish``) и в :meth:`now`."""

    def __init__(self, speak: Optional[Callable[[str], None]], *, titles: Titles, ask: Optional[Ask] = None,
                 missing: Iterable[str] = (), persona: Optional[str] = None,
                 publish: Optional[Callable[[str, Dict[str, Any]], Any]] = None,
                 fold: dl.Fold = dl.plain_fold, logger: Any = None, spawn: Callable[[Callable[[], None]], None] = _thread,
                 grace_s: float = GRACE_S, deadline_s: float = LINE_DEADLINE_S) -> None:
        self._speak = speak
        self._publish = publish
        self._missing = tuple(missing)  # части темы-перечисления без мелодий (``ThemeHits.missing``)
        self._titles = titles
        self._ask = ask
        self._persona = persona
        self._fold = fold
        self._log = logger or _LOG
        self._spawn = spawn
        self._grace = float(grace_s)
        self._deadline = float(deadline_s)
        self._lock = threading.Lock()
        self._entries: Dict[str, _Entry] = {}
        self._spoken: set = set()
        self._last: Optional[str] = None
        self._title_map: Dict[str, str] = {}
        self._played: List[str] = []  # хуки сыгранных треков сета
        self._now: Dict[str, Any] = {}

    def prepare(self, track_id: str, no: int, plan: SetPlan, hook_id: Optional[str]) -> None:
        """Компоновка трека ``no``: факты и фраза LLM — в фоне (не на пути звука)."""
        entry = _Entry(no, plan.seed + no, plan, hook_id)
        with self._lock:
            self._entries[track_id] = entry
        self._spawn(lambda: self._prepare(track_id, entry, plan, hook_id))

    def _prepare(self, track_id: str, entry: _Entry, plan: SetPlan, hook_id: Optional[str]) -> None:
        try:
            entry.facts = self._facts(entry.no, plan, hook_id)
            entry.known.set()
            entry.llm = self._colour(track_id, entry.facts)
        except Exception as exc:  # noqa: BLE001 — сбой фактов/LLM: на started прозвучит шаблон
            self._log.warning(f"⚠️ [dj line] {track_id} факты/LLM: {type(exc).__name__}: {exc}")
        finally:
            entry.known.set()
            entry.ready.set()

    def _facts(self, no: int, plan: SetPlan, hook_id: Optional[str]) -> dl.LineFacts:
        new = [h for h in plan.profile.hook_ids if h not in self._title_map]
        if new:  # названия мелодий плана — один раз на хук (поправка LLM может добавить хуки)
            found = self._titles(new)
            self._title_map.update({h: found.get(h, "") for h in new})
        hook = (self._titles([hook_id]).get(hook_id) or None) if hook_id else None
        names = tuple(dict.fromkeys((*(t for t in self._title_map.values() if t), *self._missing)))
        energies = [plan.track(n).energy for n in range(1, max(no, len(plan.tracks)) + 1)]
        return dl.facts_for(no, len(plan.tracks), plan.profile.theme, hook, energies, names)

    def _colour(self, track_id: str, facts: dl.LineFacts) -> Optional[str]:
        """Фраза LLM по фактам или ``None``; исход — в лог одной строкой."""
        if self._ask is None:
            return None
        system, user = dl.prompt(facts, self._persona)
        outcome, response, detail = self._ask(system, user, dl.tool(), self._deadline)
        line = None
        if outcome == "ok":
            try:
                line = dl.validate_line(line_payload(response), facts, self._fold)
            except dl.LineInvalid as exc:
                outcome, detail = "invalid", f"line_invalid{{{exc.reason}}} {exc}"
        (self._log.info if outcome == "ok" else self._log.warning)(
            f"🗣️ [dj line] {track_id} line_outcome={outcome} {detail}".rstrip())
        return line

    def on_started(self, track_id: str) -> str:
        """``started`` трека: одна фраза (I23) — в потоке, переход её не ждёт. Пометка для лога ``started``."""
        with self._lock:
            entry = self._entries.get(track_id)
            if entry is None or track_id in self._spoken:
                return ""
            self._spoken.add(track_id)
            self._entries = {k: e for k, e in self._entries.items() if e.no > entry.no}  # старое не нужно
        self._spawn(lambda: self._say(track_id, entry))
        return "dj_line=on" if self._speak is not None else ""

    def _say(self, track_id: str, entry: _Entry) -> None:
        entry.known.wait(self._grace)
        self._share(track_id, entry)
        if self._speak is None:
            return
        entry.ready.wait(self._grace)
        facts = entry.facts or dl.LineFacts(entry.no, 0, "", None, 0)  # факты не успели — только номер трека
        with self._lock:
            if entry.llm:
                source, text = "llm", entry.llm
            else:
                source, text = "template", dl.template_line(facts, entry.seed, self._last)
            self._last = text
        total = facts.tracks or "?"
        self._log.info(f"🗣️ [dj line] {track_id} трек {entry.no}/{total} phase={facts.phase} source={source} "
                       f"hook={facts.hook!r} text={text!r}")
        try:
            self._speak(text)
        except Exception as exc:  # noqa: BLE001 — реплика не роняет поток и не трогает музыку
            self._log.warning(f"⚠️ [dj line] {track_id} не озвучена: {type(exc).__name__}: {exc}")

    def _share(self, track_id: str, entry: _Entry) -> None:
        """Факты играющего трека — в снимок (``dj``) и в :meth:`now`: мелодия, следующие по плану, не найденное."""
        facts = entry.facts
        with self._lock:
            if entry.hook_id:
                self._played.append(entry.hook_id)
            fields: Dict[str, Any] = {"track_no": entry.no, "tracks": len(entry.plan.tracks),
                                      "melody": (facts.hook if facts else None) or "",
                                      "next_melodies": self._upcoming(entry), "not_found": list(self._missing)}
            if fields["melody"]:
                fields["title"] = f"«{fields['melody']}» · трек {entry.no} из {fields['tracks']}"
            self._now = fields
        if self._publish is not None:
            try:
                self._publish(track_id, fields)
            except Exception as exc:  # noqa: BLE001 — снимок не роняет поток реплики
                self._log.warning(f"⚠️ [dj line] {track_id} факты не в снимке: {type(exc).__name__}: {exc}")
        self._log.info(f"🎧 [dj facts] {track_id} melody={fields['melody']!r} next={fields['next_melodies']} "
                       f"not_found={fields['not_found']}")

    def _upcoming(self, entry: _Entry) -> List[str]:
        """Следующие мелодии по плану — только у хуков, найденных по теме: они идут по порядку профиля
        (``compose._hook_queue``); пул по хешу выбирает сид — заранее не известен, честно пусто."""
        profile = entry.plan.profile
        left = len(entry.plan.tracks) - entry.no
        if not profile.theme_hooks or left <= 0:
            return []
        ids = [h for h in profile.hook_ids if h not in self._played]
        titles = list(dict.fromkeys(t for t in (self._title_map.get(h) for h in ids) if t))
        return titles[:min(UPCOMING, left)]

    def now(self) -> Dict[str, Any]:
        """Факты трека, который сейчас играет (последний ``started``); пусто — ещё не известны."""
        with self._lock:
            return dict(self._now)


def latin_fold(text: str) -> str:
    """Свёртка для сверки названий: ``Тетрис`` и ``Tetris`` — один ключ (транслит + звуковой ключ поиска)."""
    from ..core.translit_ru import transliterate_ru
    from .search import sound_key

    return sound_key(transliterate_ru(dl.plain_fold(text)))


def library_titles(library_factory: Callable[[], Any]) -> Titles:
    """``{id: название}`` для голоса — ``human_track_title`` (русский алиас записи или title без хвостов)."""
    cache: Dict[str, str] = {}

    def titles(ids: Iterable[str]) -> Dict[str, str]:
        from ..core.rtttl_library import human_track_title

        out: Dict[str, str] = {}
        lib = None
        for melody_id in ids:
            if melody_id not in cache:
                lib = lib or library_factory()
                record = lib.get(melody_id) or {}
                same = str(record.get("name", "")).lower() == melody_id.lower()
                cache[melody_id] = human_track_title(lib, record) if same else ""
            if cache[melody_id]:
                out[melody_id] = cache[melody_id]
        return out

    return titles


__all__ = ["Ask", "GRACE_S", "LINE_DEADLINE_S", "Titles", "TransitionLines", "latin_fold", "library_titles",
           "line_payload"]
