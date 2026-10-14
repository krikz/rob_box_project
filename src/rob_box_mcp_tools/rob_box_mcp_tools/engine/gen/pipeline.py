"""Заказ → ожидание → скачивание → кэш → учёт (ADR-0158 §2.1, §2.5, §2.6).

Синхронный, ждущий: вызывать из потока (``asyncio.to_thread``), как прежний
клиент MiniMax. Выбор «генерировать или нет» здесь НЕ принимается — это
чойсер (PR-3); сюда доходит уже решённый заказ.

``early=True`` (кавер, песня, одиночный инструментал — их ждёт мост): как только у задачи появился
``stream_audio_url`` (или ``audio_url`` первого варианта), он скачивается целиком и проверяется на полноту
(``stream.py``); проверенный вариант кладётся в кэш с готовым wav сервера и возвращается сразу — мост
кроссфейдит в него, не дожидаясь ``success``. Остальное (второй вариант, финальный ``audio_url`` и длина,
факт кредитов) дописывает фоновый финишёр после ``success``; уже отданный вариант заново не играется.
Стрим неполный / ошибка — причина в лог и обычное ожидание ``success`` (честный фолбэк).
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from . import stream
from .budget import Budget
from .keys import pool_key, request_key, sha256_bytes
from .library import GeneratedMusicLibrary
from .provider import GEN_TIMEOUT_S, GenJob, GenStatus, GenTicket, GenTrack, GenUnavailable, MusicGenProvider, \
    wait_complete
from .quota import enforce_quota

_LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class GenResult:
    task_id: str
    request_key: str
    credits: float           # факт из ответа провайдера (creditsConsumed)
    credits_estimated: bool  # True — провайдер не прислал, взята цена из GEN_MODELS
    saved: List[dict]        # результаты ``save_track`` по вариантам
    early: bool = False      # True — отдан ранний звук; остальные варианты дописывает финишёр
    finisher: Optional[threading.Thread] = field(default=None, compare=False, repr=False)


@dataclass
class _Order:
    """Всё, что нужно для записи вариантов заказа в кэш (одно на ранний путь и финишёр)."""

    provider: MusicGenProvider
    library: GeneratedMusicLibrary
    job: GenJob
    ticket: GenTicket
    rkey: str
    spent: float
    extra: Dict[str, str]
    log: Callable[[], Any]

    def save(self, track: GenTrack, audio: bytes, duration_s: float) -> dict:
        job = self.job
        instrumental = job.kind in ("instrumental", "cover")
        return self.library.save_track(
            audio, prompt=job.style_tags or job.title, title=track.title or job.title, lyrics=job.lyrics,
            model=job.model, duration_ms=int(duration_s * 1000), is_instrumental=instrumental, genre=job.style_key,
            kind=job.kind, style_key=job.style_key, request_key=self.rkey,
            pool_key=pool_key(job.kind, job.style_key, instrumental), gen_provider=self.provider.name,
            task_id=self.ticket.task_id, variant=track.variant, credits=self.spent, meta_source="live", **self.extra,
        )

    def record_spend(self) -> None:
        self.library.ledger.record_spend(self.ticket.task_id, self.spent, kind=self.job.kind, request_key=self.rkey,
                                         model=self.job.model, gen_provider=self.provider.name)

    def save_all(self, status: GenStatus, done: Dict[int, str]) -> List[dict]:
        """Варианты ``complete``: новые — в кэш; ``done`` (вариант → id раннего) — финальный файл и длина."""
        saved = []
        for track in status.tracks:
            audio = self.provider.download(track.audio_url)  # медиа KIE живут 14 дней — качаем сразу
            if track.variant in done:
                same = self.library.get_track_info(done[track.variant]).get("file_sha256") == sha256_bytes(audio)
                self.library.replace_audio(done[track.variant], audio, duration_ms=int(track.duration_s * 1000),
                                           title=track.title)
                self.log().info(f"🎼 [gen] {self.ticket.task_id} вариант {track.variant}: финальный audio_url "
                                f"{'= стрим' if same else '≠ стрим, файл заменён'}, {track.duration_s:.1f} с")
                continue
            saved.append(self.save(track, audio, track.duration_s))
        # квота диска (PR-9): новое не трогаем, уходят худшие и давно не игранные заказы
        enforce_quota(self.library, protect=[*done.values(), *(str(s["id"]) for s in saved if s.get("id"))])
        return saved


def generate_into_cache(
    provider: MusicGenProvider,
    library: GeneratedMusicLibrary,
    budget: Budget,
    job: GenJob,
    *,
    theme: str = "",
    work_id: str = "",
    instrument: str = "",
    tts_provider: str = "",
    voice: str = "",
    source_audio_sha: str = "",
    set_spent: Optional[float] = None,
    timeout_s: float = GEN_TIMEOUT_S,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
    early: bool = False,
    log: Callable[[], Any] = lambda: _LOG,
    spawn: Optional[Callable[[Callable[[], None]], Optional[threading.Thread]]] = None,
) -> GenResult:
    """Сгенерировать оба варианта и положить в кэш.

    Ошибки — ``GenUnavailable(reason)`` без частичного «успеха»: лимит
    (``budget_exhausted``) проверяется ДО заказа; трата пишется в
    ``gen_ledger`` сразу по ``complete`` (или по раннему звуку — оценкой, факт по ``complete``) — до
    скачивания, чтобы сбой сети на скачивании не стёр факт потраченных кредитов.
    """
    cost = provider.cost(job)
    budget.check(cost, set_spent=set_spent)

    ticket = provider.submit(job)
    started = clock()
    rkey = request_key(
        kind=job.kind, style_key=job.style_key, theme=theme, work_id=work_id,
        instrument=instrument, lyrics=job.lyrics, tts_provider=tts_provider, voice=voice,
        gen_provider=provider.name, model=job.model, duration_s=job.duration_s,
        source_audio_sha=source_audio_sha,
    )
    extra = dict(tts_provider=tts_provider, voice=voice, work_id=work_id, instrument=instrument,
                 source_audio_sha=source_audio_sha)
    order = _Order(provider, library, job, ticket, rkey, cost, extra, log)
    probe = _EarlyProbe(order) if early else None
    status = wait_complete(provider, ticket, timeout_s=timeout_s, sleep=sleep, clock=clock,
                           stop=probe.try_status if probe else None)
    if status.stage != "complete" and probe is not None and probe.got is not None:
        return _deliver_early(order, probe.got, status, timeout_s - (clock() - started), sleep, clock, spawn)

    estimated = status.credits is None
    order.spent = cost if estimated else float(status.credits)
    order.record_spend()
    saved = order.save_all(status, {})
    return GenResult(ticket.task_id, rkey, order.spent, estimated, saved)


class _EarlyProbe:
    """``stop``-хук ``wait_complete``: одна попытка раннего звука на стадию (``text`` — стрим, ``first`` —
    ``audio_url`` первого варианта), URL не повторяется."""

    def __init__(self, order: _Order) -> None:
        self.order: _Order = order
        self.got: Optional[stream.EarlyAudio] = None
        self.tried: set = set()
        self.stages: set = set()

    def try_status(self, status: GenStatus) -> bool:
        if status.stage not in stream.EARLY_STAGES or status.stage in self.stages:
            return False
        found = stream.candidate(status.tracks, self.tried)
        if found is None:
            return False
        track, url = found
        self.tried.add(url)
        self.stages.add(status.stage)
        task = self.order.ticket.task_id
        try:
            minimum = stream.minimum_seconds(self.order.job, track)
            blob = self.order.provider.download(url, timeout_s=stream.STREAM_DOWNLOAD_TIMEOUT_S)
            wav, seconds = stream.verify(blob, minimum)
        except (stream.StreamIncomplete, GenUnavailable) as exc:
            self.order.log().warning(f"⚠️ [gen] {task} ранний звук ({status.stage}) не принят: {exc} — жду success")
            return False
        self.got = stream.EarlyAudio(track, url, blob, wav, seconds)
        self.order.log().info(f"🎼 [gen] {task} ранний звук на стадии {status.stage}: вариант {track.variant}, "
                              f"{seconds:.1f} с (≥ {minimum:.1f}), {len(blob)} байт")
        return True


def _deliver_early(order: _Order, got: stream.EarlyAudio, status: GenStatus, remaining_s: float,
                   sleep: Callable[[float], None], clock: Callable[[], float],
                   spawn: Optional[Callable[[Callable[[], None]], Optional[threading.Thread]]]) -> GenResult:
    """Ранний вариант — в кэш с готовым wav; трата — оценкой (факт допишет финишёр); финишёр — в фон."""
    estimated = status.credits is None
    if not estimated:
        order.spent = float(status.credits)
    order.record_spend()
    saved = order.save(got.track, got.blob, got.seconds)
    if not saved.get("success"):
        stream.discard(got)
        raise GenUnavailable("provider_down", f"ранний звук не записан в кэш: {saved.get('error')}", order.ticket)
    stream.adopt_wav(got.wav_path, str(saved["path"]))
    done = {got.track.variant: str(saved["id"])}

    def finish() -> None:
        try:
            final = wait_complete(order.provider, order.ticket, timeout_s=max(1.0, remaining_s), sleep=sleep,
                                  clock=clock)
            if final.credits is not None and float(final.credits) != order.spent:
                order.library.ledger.settle(order.ticket.task_id, float(final.credits))
            more = order.save_all(final, done)
            order.log().info(f"🎼 [gen] {order.ticket.task_id} финал кэша: +{len(more)} вар.")
        except Exception as exc:  # noqa: BLE001 — финишёр не роняет ноду; ранний вариант уже в кэше
            order.log().warning(f"⚠️ [gen] {order.ticket.task_id} финал кэша не дописан: {exc}")

    runner = spawn or _thread
    return GenResult(order.ticket.task_id, order.rkey, order.spent, estimated, [saved], early=True,
                     finisher=runner(finish))


def _thread(fn: Callable[[], None]) -> threading.Thread:
    thread = threading.Thread(target=fn, name="rbx-gen-finish", daemon=True)
    thread.start()
    return thread
