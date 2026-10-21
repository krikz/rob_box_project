"""Одиночный инструментал «Робот, сгенерируй рок» (ADR-0158 §2.1, §2.2, §2.7; решения Шифу В2, В3, В4).

Живой прогон 14.10 02:27 UTC (develop ``362a6d3f9``): «Робот, сгенерируй рок» → ``кандидаты=[только cache:…]`` →
«Сгенерировать сейчас не могу: генерация у меня ещё не подключена» — кавер, песня и сет были подключены, одиночный
инструментал — нет. Здесь его исполнитель; путь выбирает чойсер (``rob_box_music.source``), мост и вход результата —
``engine.instrumental_order`` (тот же ``BridgeHold``, что у кавера и песни).

Модель — из выбранного пути (``generate:instrumental:<модель>``, ``DEFAULT_GEN_MODEL`` = V6): мост держит звук,
поэтому одиночному треку — качество; V6_MINI — только первому треку сета, когда его закрывает мост (В2, §2.7).
Сеть, деньги и кэш — ``pipeline.generate_into_cache`` (лимиты ``budget`` до заказа, трата в ``gen_ledger`` по
``complete``). Теги Suno — из ``GEN_STYLES``, не от LLM. Фразы ожидания, готовности и провала — от кода из фактов
результата (ADR-0148).
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from rob_box_music import knowledge as kn
from rob_box_music.source import REASON_WORDS, theme_tags, theme_title

from .budget import Budget
from .pipeline import generate_into_cache
from .provider import GenJob, GenUnavailable
from .song import DEAD_FOR_S

_LOG = logging.getLogger(__name__)

#: Занято: инструментал уже пишется (один за раз — стенд и деньги общие).
BUSY_TEXT = "Новый трек я уже сочиняю, подожди немного."


def genre_key(genre: str) -> str:
    """Жанр заказа — ключ ``GEN_STYLES``; не назван — стиль по умолчанию."""
    return genre if genre in kn.GEN_STYLES else kn.DEFAULT_STYLE


def instrumental_job(genre: str, model: str, theme: str = "") -> GenJob:
    """Заказ: теги стиля из ``GEN_STYLES`` и тема человека (``source.theme_tags``) в ``style``, тема — в названии
    (#3775: тема доходила только до ключа кэша, Suno получал обычный «RoBBox club»)."""
    key = genre_key(genre)
    tags = ", ".join(t for t in (kn.GEN_STYLES[key].tags, theme_tags(theme)) if t)
    return GenJob(kind="instrumental", style_key=key, title=theme_title(theme) or f"RoBBox {key}", style_tags=tags,
                  model=model)


@dataclass(frozen=True)
class InstrumentalResult:
    """Итог заказа — из ответа провайдера и кэша: по нему строится фраза. ``reason`` — провал (звука нет)."""

    track_id: str = ""
    title: str = ""
    duration_s: float = 0.0
    task_id: str = ""
    credits: float = 0.0
    reason: str = ""
    theme: str = ""


def _about(theme: str) -> str:
    return f" на тему «{theme}»" if theme else ""


def wait_text(genre: str, model: str, bridge: str, theme: str = "") -> str:
    """Фраза моста (В4): что пишется (и на какую тему) и сколько ждать — от кода; ``bridge`` — что играет сейчас
    («» — ничего)."""
    eta = int(round(kn.GEN_MODELS[model].typical_s / 10.0) * 10)
    tail = f" Пока играет {bridge}." if bridge else ""
    return f"Сочиняю новый трек{_about(theme)}, {kn.GEN_STYLES[genre_key(genre)].name} — секунд {eta}.{tail}"


def ready_text(result: InstrumentalResult, genre: str) -> str:
    """Фраза готовности — из фактов результата: название, тема заказа (если её нет в названии), стиль, длина."""
    from rob_box_voice.core.media_phrases import duration_text

    theme = result.theme if result.theme.lower() not in result.title.lower() else ""
    return (f"А вот новый трек «{result.title}»{_about(theme)}, {kn.GEN_STYLES[genre_key(genre)].name}, "
            f"{duration_text(result.duration_s)}.")


def failed_text(reason: str, bridged: bool) -> str:
    """Провал (В3): причина словами — одна таблица ``REASON_WORDS``; мост доигрывает — так и сказано."""
    tail = " — доиграю то, что играет" if bridged else ""
    return f"Сгенерировать не вышло: {REASON_WORDS.get(reason, reason)}{tail}."


class InstrumentalMaker:
    """Инструментал под ключом ``KIE_API``: ``provider`` — ``MusicGenProvider``, ``library`` — ленивый доступ к кэшу,
    ``balance`` — последний известный баланс KIE (``CoverMaker.balance``: заказ сеть не ждёт)."""

    def __init__(self, provider: Any, library: Callable[[], Any], *,
                 balance: Callable[[], Optional[float]] = lambda: None, log: Callable[[], Any] = lambda: _LOG,
                 clock: Callable[[], float] = time.monotonic,
                 spawn: Optional[Callable[[Callable[[], None]], None]] = None) -> None:
        self._provider, self._library, self._balance, self._log, self._clock = provider, library, balance, log, clock
        self._spawn = spawn or (lambda fn: threading.Thread(target=fn, name="rbx-gen-instr", daemon=True).start())
        self._lock = threading.Lock()
        self._busy, self._turn, self._dead_until = False, None, 0.0

    # ── что видит чойсер пути ──
    def ready(self) -> bool:
        return bool(getattr(self._provider, "configured", False)) and self._library() is not None

    def balance(self) -> Optional[float]:
        return self._balance()

    def provider_ok(self) -> bool:
        return self._clock() >= self._dead_until

    def busy(self) -> bool:
        return self._busy

    def ordered_in(self, turn: Optional[str]) -> bool:
        """Идущий заказ сделал этот же ход (LLM зовёт ``request_music`` и ``generate_music`` подряд)."""
        return bool(self._busy and turn and turn == self._turn)

    # ── заказ ──
    def make(self, genre: str, model: str, theme: str = "") -> InstrumentalResult:
        """Заказ → ожидание → кэш (оба варианта); играет первый. Ошибка — ``reason``, звука нет."""
        started, library = self._clock(), self._library()
        try:
            made = generate_into_cache(self._provider, library, Budget(library), instrumental_job(genre, model, theme),
                                       theme=theme, early=True, log=self._log)
        except GenUnavailable as exc:
            if exc.reason == "provider_down":
                self._dead_until = self._clock() + DEAD_FOR_S
            self._log().warning(f"⚠️ [gen instr] {genre_key(genre)} {model} не сгенерирован: {exc}")
            return InstrumentalResult(reason=exc.reason, theme=theme)
        saved = next((s for s in made.saved if s.get("success")), None)
        seconds = self._clock() - started
        self._log().info(f"🎛️ [gen instr] {genre_key(genre)} {model} тема={theme or '—'!r} task_id={made.task_id} "
                         f"кредиты={made.credits:g} track={saved['id'] if saved else '—'} за {seconds:.0f} с")
        if saved is None:
            return InstrumentalResult(task_id=made.task_id, credits=made.credits, reason="no_track", theme=theme)
        info = library.get_track_info(str(saved["id"]))
        title = str(info.get("title") or saved.get("title") or "")
        return InstrumentalResult(str(saved["id"]), title, float(info.get("duration_ms") or 0) / 1000.0,
                                  made.task_id, made.credits, theme=theme)

    def launch(self, genre: str, model: str, theme: str, turn: Optional[str],
               done: Callable[[InstrumentalResult], None]) -> bool:
        """:meth:`make` в фоне, ``done`` — с итогом; ``False`` — уже пишется другой."""
        with self._lock:
            if self._busy:
                return False
            self._busy, self._turn = True, turn

        def run() -> None:
            try:
                result = self.make(genre, model, theme)
            except Exception as exc:  # noqa: BLE001 — сбой не роняет ноду, но и не молчит
                self._log().error(f"❌ [gen instr] {type(exc).__name__}: {exc}")
                result = InstrumentalResult(reason="internal")
            finally:
                self._busy = False
            done(result)

        self._spawn(run)
        return True


__all__ = ["BUSY_TEXT", "InstrumentalMaker", "InstrumentalResult", "failed_text", "genre_key", "instrumental_job",
           "ready_text", "wait_text"]
