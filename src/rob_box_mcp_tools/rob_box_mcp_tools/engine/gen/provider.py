"""Интерфейс провайдера генерации музыки (ADR-0158 §2.1).

Без тихой деградации: нет ключа / денег / 429 / 455 / сети — типизированная
``GenUnavailable(reason)``, а не пустой результат. Первый провайдер —
``kie_suno.KieSunoProvider``; второй встаёт за тот же ``MusicGenProvider``.

Модуль ROS-free: чистый Python, тестируется на фейковом транспорте.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Literal, Optional, Protocol, Tuple

#: Причины недоступности генерации (ADR-0158 §2.1, §3.5). Перечень закрытый.
REASONS: Tuple[str, ...] = (
    "no_key",             # нет ключа / 401
    "no_credits",         # KIE code=402
    "rate_limited",       # 429
    "provider_down",      # 455 / 5xx / сеть / таймаут запроса
    "rejected",           # 400 / 422 / state=fail — контент, параметры
    "timeout",            # нет complete за GEN_TIMEOUT_S
    "budget_exhausted",   # наш лимит (budget.py)
)

#: Сколько ждать ``complete`` (ADR-0158 §2.1).
GEN_TIMEOUT_S = 180.0

JobKind = Literal["instrumental", "song", "robot_song", "cover"]
Stage = Literal["queued", "text", "first", "complete", "failed"]


class GenUnavailable(Exception):
    """Генерация не состоялась по одной из ``REASONS``. Звука нет."""

    def __init__(self, reason: str, detail: str = "", ticket: "Optional[GenTicket]" = None) -> None:
        if reason not in REASONS:
            raise ValueError(f"неизвестная причина GenUnavailable: {reason!r}")
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail
        #: Заказ, который уже ушёл провайдеру (есть при ``timeout``: результат
        #: может доиграть позже, кредиты потрачены).
        self.ticket = ticket


@dataclass(frozen=True)
class ModelInfo:
    """Цена и скорость модели генерации (замеры 13.10, ADR-0158 §1.3)."""

    credits_per_task: float      # кредитов за задачу (= 2 варианта трека)
    typical_s: float             # типичное время до success, секунд


#: Таблица моделей Suno через KIE. Единственное место цены/скорости (PR-3
#: перенесёт в ``rob_box_music.knowledge`` вместе с ``GEN_STYLES``).
GEN_MODELS = {
    "V6": ModelInfo(credits_per_task=12.0, typical_s=51.0),
    "V6_MINI": ModelInfo(credits_per_task=12.0, typical_s=30.0),
}
DEFAULT_MODEL = "V6"


@dataclass(frozen=True)
class GenJob:
    """Заказ на генерацию. Теги Suno приходят из таблицы стилей, не от LLM."""

    kind: JobKind
    style_key: str                 # ключ стиля (кэш, учёт)
    title: str
    style_tags: str = ""           # EN-теги Suno для style/tags (из таблицы)
    negative_tags: str = ""
    duration_s: int = 150          # 60..360
    lyrics: str = ""               # robot_song
    audio_path: str = ""           # robot_song: вокал TTS; cover: наш рендер
    model: str = DEFAULT_MODEL
    audio_weight: Optional[float] = None
    vocal_gender: str = ""         # "m" / "f"


@dataclass(frozen=True)
class GenTicket:
    task_id: str
    provider: str
    job: GenJob
    submitted_at: float
    cost: float                    # ожидаемые кредиты (из GEN_MODELS)


@dataclass(frozen=True)
class GenTrack:
    variant: int                   # 0 / 1 — порядок в ответе
    track_id: str                  # id трека у провайдера
    audio_url: str
    stream_url: str
    duration_s: float
    title: str = ""
    tags: str = ""


@dataclass(frozen=True)
class GenStatus:
    stage: Stage
    tracks: Tuple[GenTrack, ...] = ()
    credits: Optional[float] = None      # creditsConsumed из ответа (факт)
    fail_msg: str = ""
    cost_time_s: Optional[float] = None
    raw: dict = field(default_factory=dict, compare=False, repr=False)


class MusicGenProvider(Protocol):
    name: str

    def submit(self, job: GenJob) -> GenTicket:
        """Создать задачу. ``GenUnavailable`` — сразу, без звука."""

    def poll(self, ticket: GenTicket) -> GenStatus: ...

    def balance(self) -> Optional[float]:
        """Остаток кредитов; ``None`` — неизвестно (нет ключа, сеть)."""

    def cost(self, job: GenJob) -> float: ...

    def download(self, url: str) -> bytes:
        """Скачать результат (медиа KIE живут 14 дней — качать сразу)."""


def poll_interval_s(elapsed_s: float) -> float:
    """Шаг опроса: 2 с первые 30 с, дальше 4 с (ADR-0158 §2.1)."""
    return 2.0 if elapsed_s < 30.0 else 4.0


def wait_complete(
    provider: MusicGenProvider,
    ticket: GenTicket,
    *,
    timeout_s: float = GEN_TIMEOUT_S,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> GenStatus:
    """Ждать ``complete``. ``failed`` → ``rejected``; нет за ``timeout_s`` → ``timeout``."""
    start = clock()
    while True:
        status = provider.poll(ticket)
        if status.stage == "complete":
            return status
        if status.stage == "failed":
            raise GenUnavailable("rejected", status.fail_msg or "state=fail", ticket)
        elapsed = clock() - start
        if elapsed >= timeout_s:
            raise GenUnavailable("timeout", f"нет complete за {timeout_s:.0f} с", ticket)
        sleep(min(poll_interval_s(elapsed), max(0.0, timeout_s - elapsed)))
