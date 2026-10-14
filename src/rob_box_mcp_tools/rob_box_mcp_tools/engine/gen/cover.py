"""Кавер пьесы по названию (ADR-0158 §2.9, решение Шифу В7): наш рендер мелодии → KIE upload → Suno cover → кэш.

Проба 13.10 (``b7.py`` координатора, «звучит шикарно»): мелодия RTTTL «Ride Of The Valkyries» простым синтезом
(пила + синус, 20.8 с) → ``upload-and-cover-audio`` V6, ``audio_weight`` 0.65 → 2 × 89 с за ≈ 43 с, 12 кредитов.
Здесь тот же рендер, но из модели трека, который робот и так собирает для пьесы (``model.Track``: лид песни из
RTTTL или партитуры) — одна мелодия у моста-синтеза и у кавера. Сеть/деньги/кэш — ``pipeline.generate_into_cache``
(лимиты ``budget`` до заказа, трата в ``gen_ledger`` по ``complete``). Стиль Suno — из таблиц ``knowledge``
(``COVER_STYLES`` инструмента, ``GEN_STYLES`` жанра), не от LLM.

Синхронный :meth:`CoverMaker.make` — для потока; :meth:`CoverMaker.start` запускает его в фоне и зовёт ``done``
с результатом. Решения «кавер или нет» здесь нет — это чойсер пути (``rob_box_music.source``).
"""

from __future__ import annotations

import array
import logging
import math
import tempfile
import threading
import time
import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, List, Optional, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.source import CoverWork, SourceAsk, cover_style_key

from .budget import Budget
from .keys import sha256_file
from .pipeline import generate_into_cache
from .provider import GenJob, GenUnavailable

_LOG = logging.getLogger(__name__)
#: Баланс KIE на пути заказа не спрашивается синхронно (сеть): берётся последний известный, старше — обновить в фоне.
BALANCE_TTL_S = 3600.0
#: Нота короче этого (с) не рендерится: щелчок, а не звук.
MIN_NOTE_S = 0.03


def melody_notes(track: Any) -> List[Tuple[float, float, int]]:
    """``(начало, длина, midi)`` лида трека в секундах от первой ноты; лида нет — хук. Пусто — мелодии нет."""
    part = (getattr(track, "parts", None) or {}).get("lead")
    pitches = tuple(getattr(part, "pitches", None) or ())
    if not pitches and getattr(track, "hook", None) is not None:
        pitches = tuple(track.hook.notes)
    if not pitches:
        return []
    beat_s = 60.0 / float(track.bpm)
    first = min(p.beat for p in pitches)
    ordered = sorted(pitches, key=lambda p: p.beat)
    return [((p.beat - first) * beat_s, p.dur_beats * beat_s, int(p.midi)) for p in ordered]


def source_notes(notes: List[Tuple[float, float, int]], min_s: float = kn.COVER_SOURCE_MIN_S,
                 max_s: float = kn.COVER_SOURCE_MAX_S) -> List[Tuple[float, float, int]]:
    """Мелодия длиной ``min_s..max_s``: короткая — повторяется (проба: мотив трижды), длинная — до ноты на ``max_s``."""
    if not notes:
        return []
    span = max(start + dur for start, dur, _ in notes)
    if span <= 0:
        return []
    out: List[Tuple[float, float, int]] = []
    offset = 0.0
    while offset < min_s:
        out.extend((offset + s, d, m) for s, d, m in notes if offset + s + d <= max_s)
        offset += span
    return out


def render_wav(notes: List[Tuple[float, float, int]], path: str, rate: int = kn.COVER_SOURCE_RATE_HZ) -> float:
    """Ноты → моно wav 16 бит (пила 0.6 + синус 0.4, атака/спад 10/30 мс — как у пробы); длина, секунд."""
    total = max(s + d for s, d, _ in notes)
    buf = array.array("f", bytes(4 * int(total * rate + 1)))
    for start, dur, midi in notes:
        if dur < MIN_NOTE_S:
            continue
        freq, first, count = 440.0 * 2 ** ((midi - 69) / 12.0), int(start * rate), int(dur * rate)
        for k in range(count):
            t = k / rate
            env = min(1.0, t / 0.01) * max(0.0, 1.0 - max(0.0, t - (dur - 0.03)) / 0.03)
            phase = (freq * t) % 1.0
            buf[first + k] += env * 0.35 * ((2 * phase - 1) * 0.6 + math.sin(2 * math.pi * freq * t) * 0.4)
    pcm = array.array("h", (int(max(-1.0, min(1.0, x)) * 32000) for x in buf))
    with wave.open(path, "wb") as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(rate)
        out.writeframes(pcm.tobytes())
    return len(buf) / float(rate)


def cover_job(work: CoverWork, ask: SourceAsk, audio_path: str) -> GenJob:
    """Заказ кавера: теги инструмента (``COVER_STYLES``) и/или жанра (``GEN_STYLES``), без них — оркестр."""
    style = cover_style_key(work, ask)
    tags = ", ".join(t for t in (kn.COVER_STYLES.get(work.instrument, ""),
                                 kn.GEN_STYLES[style].tags if style in kn.GEN_STYLES else "") if t)
    acoustic = style not in kn.COVER_ELECTRONIC_STYLES
    negative = ", ".join(t for t in (kn.COVER_NEGATIVE_TAGS, kn.COVER_ACOUSTIC_NEGATIVE_TAGS if acoustic else "") if t)
    return GenJob(kind="cover", style_key=style, title=f"{work.title} (RoBBox cover)", style_tags=tags,
                  negative_tags=negative, duration_s=kn.COVER_DURATION_S, audio_path=audio_path,
                  model=kn.COVER_MODEL, audio_weight=kn.COVER_AUDIO_WEIGHT)


@dataclass(frozen=True)
class CoverResult:
    """Итог кавера: ``track_id`` первого варианта в кэше (второй — «в запас»; ``track_ids`` — оба, сету) или
    ``reason`` провала."""

    track_id: str = ""
    reason: str = ""
    task_id: str = ""
    credits: float = 0.0
    seconds: float = 0.0
    source_s: float = 0.0
    track_ids: Tuple[str, ...] = ()


class CoverMaker:
    """Кавер под ключом ``KIE_API``: ``provider`` — ``MusicGenProvider`` (боевой — ``KieSunoProvider``), ``library`` —
    ленивый доступ к кэшу ``GeneratedMusicLibrary`` (его поднимает ``mcp_server`` позже движка)."""

    def __init__(self, provider: Any, library: Callable[[], Any], *, log: Callable[[], Any] = lambda: _LOG,
                 clock: Callable[[], float] = time.monotonic, workdir: Optional[str] = None) -> None:
        self._provider = provider
        self._library = library
        self._log = log
        self._clock = clock
        self._workdir = workdir
        self._balance: Optional[float] = None
        self._balance_at: Optional[float] = None
        self._balance_lock = threading.Lock()

    def ready(self) -> bool:
        """Можно ли вообще заказывать: ключ задан и кэш поднят (иначе кавер-кандидата нет — ``not_wired``)."""
        return bool(getattr(self._provider, "configured", False)) and self._library() is not None

    def balance(self) -> Optional[float]:
        """Последний известный баланс; старше ``BALANCE_TTL_S`` или неизвестен — обновить в фоне (заказ не ждёт)."""
        with self._balance_lock:
            stale = self._balance_at is None or self._clock() - self._balance_at >= BALANCE_TTL_S
            if stale:
                self._balance_at = self._clock()  # одно обновление за раз
        if stale and self.ready():
            threading.Thread(target=self.refresh_balance, name="rbx-gen-balance", daemon=True).start()
        return self._balance

    def refresh_balance(self) -> Optional[float]:
        value = self._provider.balance()
        with self._balance_lock:
            self._balance, self._balance_at = value, self._clock()
        self._log().info(f"🎼 [cover] баланс KIE: {value if value is not None else 'неизвестен'}")
        return value

    def make(self, track: Any, work: CoverWork, ask: SourceAsk, set_spent: Optional[float] = None,
             early: bool = True) -> CoverResult:
        """Рендер → заказ → ожидание → кэш (оба варианта). Ошибка — ``CoverResult(reason=…)``, звука нет.
        Кавер трека тематического сета (В14): ``set_spent`` — потрачено сетом (лимит сета до заказа), ``early=False``
        — ждать оба варианта (второй закрывает следующий трек сета)."""
        started = self._clock()
        notes = source_notes(melody_notes(track))
        if not notes:
            return CoverResult(reason="rejected")
        workdir = Path(self._workdir or tempfile.gettempdir())
        workdir.mkdir(parents=True, exist_ok=True)
        wav = str(workdir / f"cover_src_{int(time.time())}.wav")
        source_s = render_wav(notes, wav)
        library = self._library()
        try:
            result = generate_into_cache(self._provider, library, Budget(library), cover_job(work, ask, wav),
                                         work_id=work.title, instrument=work.instrument,
                                         source_audio_sha=sha256_file(wav), set_spent=set_spent, early=early,
                                         log=self._log)
        except GenUnavailable as exc:
            self._log().warning(f"⚠️ [cover] «{work.title}» не сгенерирован: {exc}")
            return CoverResult(reason=exc.reason, source_s=source_s, seconds=self._clock() - started)
        finally:
            Path(wav).unlink(missing_ok=True)
        saved = [s for s in result.saved if s.get("success")]
        self.refresh_balance()
        seconds = self._clock() - started
        self._log().info(f"🎼 [cover] «{work.title}» task_id={result.task_id} кредиты={result.credits:g} "
                         f"исходник={source_s:.1f} с вариантов={len(saved)} за {seconds:.1f} с")
        if not saved:
            return CoverResult(reason="provider_down", task_id=result.task_id, credits=result.credits,
                               seconds=seconds, source_s=source_s)
        return CoverResult(str(saved[0]["id"]), "", result.task_id, result.credits, seconds, source_s,
                           tuple(str(s["id"]) for s in saved))

    def start(self, track: Any, work: CoverWork, ask: SourceAsk,
              done: Callable[[CoverResult], None]) -> threading.Thread:
        """:meth:`make` в фоне; ``done`` — с результатом (в том же потоке)."""
        def run() -> None:
            try:
                result = self.make(track, work, ask)
            except Exception as exc:  # noqa: BLE001 — сбой не роняет ноду, но и не молчит
                self._log().error(f"❌ [cover] «{work.title}»: {type(exc).__name__}: {exc}")
                result = CoverResult(reason="provider_down")
            done(result)

        thread = threading.Thread(target=run, name="rbx-cover", daemon=True)
        thread.start()
        return thread


__all__ = ["BALANCE_TTL_S", "CoverMaker", "CoverResult", "cover_job", "melody_notes", "render_wav", "source_notes"]
