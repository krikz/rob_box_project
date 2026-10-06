"""Реплика диджея на каждом переходе сета (ADR-0149 §12 В2 — решение Шифу 06.10; I23; ADR-0148).

На компоновке трека N (``SetPlanBox.compose_mark``: трек 1 — на старте сета, N+1 — когда он готовится в очередь)
:class:`TransitionLines` в своём потоке собирает факты трека из плана (``rob_box_music.dj_line.LineFacts``: номер и
длина сета, тема, человеческое название мелодии-хука из RTTTL-библиотеки, фаза дуги энергии) и спрашивает LLM
раскраску (тот же провайдер, клиент и circuit breaker, что у ризонера плана — ``SetReasoner.ask``). На ``started``
трека — ровно одна фраза (I23): готова и прошла валидатор фраза LLM — она, иначе шаблон кода с теми же фактами.
Музыку реплика не трогает: фраза уходит в ``speak_text`` (тот же путь в TTS, что у LLM) поверх сета, без паузы и
приглушения; ``speak_text`` в потоке, переход и клок его не ждут.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, Optional, Tuple

from rob_box_music import dj_line as dl
from rob_box_music.set_plan import SetPlan

_LOG = logging.getLogger(__name__)

#: Сколько поток реплики на ``started`` ждёт фразу LLM (трек 1: LLM спрошена за миг до старта; дальше — за трек).
GRACE_S = 3.0
#: Дедлайн одного вызова LLM за фразу: опоздала — шаблон.
LINE_DEADLINE_S = 20.0

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
    facts: Optional[dl.LineFacts] = None
    llm: Optional[str] = None
    ready: threading.Event = field(default_factory=threading.Event)  # факты собраны и LLM ответила (или нет)


def line_payload(response: Any) -> Any:
    """Строка из вызова ``submit_dj_line``; без вызова — ``LineInvalid``."""
    for call in getattr(response, "tool_calls", ()) or ():
        if call.name == dl.SUBMIT_TOOL:
            return dl.payload_line(dict(call.arguments))
    raise dl.LineInvalid("no_call", f"нет вызова {dl.SUBMIT_TOOL}")


class TransitionLines:
    """Реплики одного сета. ``ask=None`` — LLM не спрашивается, звучат шаблоны (фраза всё равно есть)."""

    def __init__(self, speak: Callable[[str], None], *, titles: Titles, ask: Optional[Ask] = None,
                 theme_names: Callable[[str], Iterable[str]] = lambda _theme: (), persona: Optional[str] = None,
                 fold: dl.Fold = dl.plain_fold, logger: Any = None, spawn: Callable[[Callable[[], None]], None] = _thread,
                 grace_s: float = GRACE_S, deadline_s: float = LINE_DEADLINE_S) -> None:
        self._speak = speak
        self._titles = titles
        self._ask = ask
        self._theme_names = theme_names
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
        self._names: Optional[Tuple[str, ...]] = None

    def prepare(self, track_id: str, no: int, plan: SetPlan, hook_id: Optional[str]) -> None:
        """Компоновка трека ``no``: факты и фраза LLM — в фоне (не на пути звука)."""
        entry = _Entry(no, plan.seed + no)
        with self._lock:
            self._entries[track_id] = entry
        self._spawn(lambda: self._prepare(track_id, entry, plan, hook_id))

    def _prepare(self, track_id: str, entry: _Entry, plan: SetPlan, hook_id: Optional[str]) -> None:
        try:
            entry.facts = self._facts(entry.no, plan, hook_id)
            entry.llm = self._colour(track_id, entry.facts)
        except Exception as exc:  # noqa: BLE001 — сбой фактов/LLM: на started прозвучит шаблон
            self._log.warning(f"⚠️ [dj line] {track_id} факты/LLM: {type(exc).__name__}: {exc}")
        finally:
            entry.ready.set()

    def _facts(self, no: int, plan: SetPlan, hook_id: Optional[str]) -> dl.LineFacts:
        theme = plan.profile.theme
        if self._names is None:  # названия сета (мелодии плана и части темы) — один раз на сет
            titles = self._titles(plan.profile.hook_ids)
            self._names = tuple(dict.fromkeys((*titles.values(), *self._theme_names(theme))))
        hook = self._titles([hook_id]).get(hook_id) if hook_id else None
        energies = [plan.track(n).energy for n in range(1, max(no, len(plan.tracks)) + 1)]
        return dl.facts_for(no, len(plan.tracks), theme, hook, energies, self._names)

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
        return "dj_line=on"

    def _say(self, track_id: str, entry: _Entry) -> None:
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
