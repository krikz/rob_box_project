"""Заказ → ожидание → скачивание → кэш → учёт (ADR-0158 §2.1, §2.5, §2.6).

Синхронный, ждущий: вызывать из потока (``asyncio.to_thread``), как прежний
клиент MiniMax. Выбор «генерировать или нет» здесь НЕ принимается — это
чойсер (PR-3); сюда доходит уже решённый заказ.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, List, Optional

from .budget import Budget
from .keys import pool_key, request_key
from .library import GeneratedMusicLibrary
from .provider import GEN_TIMEOUT_S, GenJob, MusicGenProvider, wait_complete


@dataclass(frozen=True)
class GenResult:
    task_id: str
    request_key: str
    credits: float           # факт из ответа провайдера (creditsConsumed)
    credits_estimated: bool  # True — провайдер не прислал, взята цена из GEN_MODELS
    saved: List[dict]        # результаты ``save_track`` по вариантам


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
) -> GenResult:
    """Сгенерировать оба варианта и положить в кэш.

    Ошибки — ``GenUnavailable(reason)`` без частичного «успеха»: лимит
    (``budget_exhausted``) проверяется ДО заказа; трата пишется в
    ``gen_ledger`` сразу по ``complete`` — до скачивания, чтобы сбой сети на
    скачивании не стёр факт потраченных кредитов.
    """
    cost = provider.cost(job)
    budget.check(cost, set_spent=set_spent)

    ticket = provider.submit(job)
    status = wait_complete(provider, ticket, timeout_s=timeout_s, sleep=sleep, clock=clock)

    instrumental = job.kind in ("instrumental", "cover")
    rkey = request_key(
        kind=job.kind, style_key=job.style_key, theme=theme, work_id=work_id,
        instrument=instrument, lyrics=job.lyrics, tts_provider=tts_provider, voice=voice,
        gen_provider=provider.name, model=job.model, duration_s=job.duration_s,
        source_audio_sha=source_audio_sha,
    )
    estimated = status.credits is None
    spent = cost if estimated else float(status.credits)
    library.ledger.record_spend(
        ticket.task_id, spent, kind=job.kind, request_key=rkey, model=job.model,
        gen_provider=provider.name,
    )

    saved = []
    for track in status.tracks:
        audio = provider.download(track.audio_url)  # медиа KIE живут 14 дней — качаем сразу
        saved.append(library.save_track(
            audio,
            prompt=job.style_tags or job.title,
            title=track.title or job.title,
            lyrics=job.lyrics,
            model=job.model,
            duration_ms=int(track.duration_s * 1000),
            is_instrumental=instrumental,
            genre=job.style_key,
            kind=job.kind,
            style_key=job.style_key,
            request_key=rkey,
            pool_key=pool_key(job.kind, job.style_key, instrumental),
            gen_provider=provider.name,
            task_id=ticket.task_id,
            variant=track.variant,
            credits=spent,
            tts_provider=tts_provider,
            voice=voice,
            work_id=work_id,
            instrument=instrument,
            source_audio_sha=source_audio_sha,
            meta_source="live",
        ))
    return GenResult(ticket.task_id, rkey, spent, estimated, saved)
