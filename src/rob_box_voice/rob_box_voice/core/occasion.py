"""Occasion Gate — единая точка решения «можно ли заговорить без слова пользователя».

Issue #2536, ADR-0101 §3.1, ADR-0102 §5. PR-A: каркас + регрессионные тесты.
Полная миграция wake-gate / startup / dj_tick / meeting — PR-B…F (не здесь).

Консолидирует:
- стаб-фильтр (vision event_type='person' + source_camera='unknown'/'stub' → REFUSE)
- одноразовые маркеры (startup — только один раз за uptime)
- user-initiated bypass (wake_word — без задержки, байт-в-байт эквивалентно)
- per-source кулдаун (настраивается через ``source_cooldowns``)
- глобальный дебаунс (любые-два повода ближе N секунд → DEFER)

Чистая функция от ``(state, occasion)``: тестируется без ROS2.

Зависимости: НИЧЕГО из ``rob_box_perception``. perception — отдельный сервис
(Main Pi / vision-hailo), которого НЕТ в voice-assistant образе; cross-service
связь voice↔perception — только по топикам, не по Python-импорту (ADR-0103 §3.1 п.6).
Первый черновик PR-A импортировал ``rob_box_perception.core.event_detector``
на уровне модуля → dialogue_node падал на старте voice-образа
(ModuleNotFoundError). Гейт ведёт собственный ``_last_any_at`` (см. __init__).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class VerdictKind(str, Enum):
    """Решение шва «можно ли заговорить»."""

    ALLOW = "allow"
    DEFER = "defer"
    REFUSE = "refuse"


@dataclass(frozen=True)
class Occasion:
    """Семантическое описание «что даёт право заговорить».

    Attributes
    ----------
    kind:
        ``"wake_word"`` / ``"dj_tick"`` / ``"startup"`` / ``"meeting"`` /
        ``"unclear_acknowledgement"`` / ``"inactivity_acknowledgement"``.
        Конкретные kind'ы не валидируются на уровне типа — это «семантика
        по соглашению», чтобы добавлять новые поводы без правки кода.
    payload:
        Произвольный контекст (``event_type``, ``source_camera`` и т.п.).
    is_user_initiated:
        True для wake_word / TG-маркера (минует кулдаун); False для
        автономных поводов (dj_tick, meeting, startup, …).
    """

    kind: str
    payload: dict = field(default_factory=dict)
    is_user_initiated: bool = False


@dataclass
class Verdict:
    """Решение шва «можно ли заговорить».

    Attributes
    ----------
    kind:
        ALLOW / DEFER / REFUSE.
    reason:
        Человекочитаемое объяснение (для логов и метрик).
    retry_after_s:
        Для DEFER — через сколько секунд повторить.
    """

    kind: VerdictKind
    reason: str
    retry_after_s: Optional[float] = None


class OccasionGate:
    """Единая точка решения «можно ли заговорить без слова пользователя».

    Алгоритм (см. ADR-0101 §3.1):

    1. **Стаб-фильтр** — для ``kind == "meeting"`` + ``event_type ==
       "person"`` + ``source_camera in {"unknown", "stub"}`` →
       ``REFUSE`` (защита от детерминированного 2-sec таймера
       ``StubHEFLoader``).
    2. **Одноразовые маркеры** — ``startup`` после ``mark_consumed``
       → ``DEFER("one-shot already consumed")``.
    3. **user-initiated bypass** — wake_word / TG → ``ALLOW("user-initiated")``
       минует ВСЕ нижестоящие шаги.
    4. **Per-source кулдаун** — если ``source_cooldowns[kind]`` задан
       и ``(now - last_fire_at[kind]) < cooldown`` → ``DEFER`` с
       ``retry_after_s ≈ cooldown - (now - last)``.
    5. **Глобальный дебаунс** — если ``(now - last_any_at) <
       global_debounce_s`` → ``DEFER("global debounce")``.
    6. **ALLOW** + (опционально) ``mark_consumed``.

    Parameters
    ----------
    global_debounce_s:
        Минимальный интервал между ЛЮБЫМИ двумя поводами.
    source_cooldowns:
        Словарь ``{kind: cooldown_seconds}``. Незаданные kind'ы —
        без per-source ограничения (только глобальный дебаунс).
    stub_event_type:
        event_type, при котором срабатывает стаб-фильтр (по умолчанию
        ``"person"`` — vision-stub детектирует лица).
    stub_source_cameras:
        frozenset имён stub-камер, при которых стаб-фильтр REFUSE'ит.
    one_shot_kinds:
        frozenset kind'ов, которые разрешены только один раз за uptime.
    """

    def __init__(
        self,
        *,
        global_debounce_s: float = 2.0,
        source_cooldowns: Optional[dict[str, float]] = None,
        stub_event_type: str = "person",
        stub_source_cameras: frozenset[str] = frozenset({"unknown", "stub"}),
        one_shot_kinds: frozenset[str] = frozenset({"startup"}),
    ) -> None:
        self._global_debounce_s = float(global_debounce_s)
        self._source_cooldowns: dict[str, float] = dict(source_cooldowns or {})
        self._stub_event_type = stub_event_type
        self._stub_source_cameras = frozenset(stub_source_cameras)
        self._one_shot_kinds = frozenset(one_shot_kinds)

        # Bookkeeping. Два независимых счётчика:
        #   self._last_fire_at[kind] — per-source (для source_cooldowns);
        #   self._last_any_at — «глобальный last any» (для глобального дебаунса).
        self._last_fire_at: dict[str, float] = {}
        self._last_any_at: float = 0.0
        self._consumed_one_shot: set[str] = set()

    # ------------------------------------------------------------------ may_speak

    def may_speak(
        self, occasion: Occasion, now: Optional[float] = None
    ) -> Verdict:
        """Решить, можно ли инициировать ход для данного повода.

        Parameters
        ----------
        occasion:
            Повод (см. :class:`Occasion`).
        now:
            Монотонное время для воспроизводимости тестов. ``None`` →
            ``time.monotonic()`` (production).

        Returns
        -------
        Verdict
            :class:`Verdict` с ``kind`` ∈ {ALLOW, DEFER, REFUSE}.
        """
        # 1. Стаб-фильтр (только для meeting/person — vision-only).
        if occasion.kind == "meeting":
            payload = occasion.payload or {}
            event_type = payload.get("event_type")
            source_camera = payload.get("source_camera")
            if (
                event_type == self._stub_event_type
                and source_camera in self._stub_source_cameras
            ):
                return Verdict(
                    VerdictKind.REFUSE,
                    f"stub event: event_type={event_type!r} "
                    f"source_camera={source_camera!r}",
                )

        # 2. Одноразовые маркеры (startup и др.).
        if occasion.kind in self._one_shot_kinds:
            if occasion.kind in self._consumed_one_shot:
                return Verdict(VerdictKind.DEFER, "one-shot already consumed")

        # 3. user-initiated bypass — wake_word / TG-маркер.
        #    Сохраняет байт-в-байт поведение _on_stt → has_wake_word.
        if occasion.is_user_initiated:
            return Verdict(VerdictKind.ALLOW, "user-initiated")

        # 4. Per-source кулдаун.
        now_val = now if now is not None else time.monotonic()
        cooldown = self._source_cooldowns.get(occasion.kind, 0.0)
        last = self._last_fire_at.get(occasion.kind)
        if last is not None and cooldown > 0.0 and (now_val - last) < cooldown:
            return Verdict(
                VerdictKind.DEFER,
                f"source cooldown ({cooldown}s)",
                retry_after_s=cooldown - (now_val - last),
            )

        # 5. Глобальный дебаунс (любые-два повода ближе N).
        if (now_val - self._last_any_at) < self._global_debounce_s:
            return Verdict(
                VerdictKind.DEFER,
                f"global debounce ({self._global_debounce_s}s)",
                retry_after_s=self._global_debounce_s
                - (now_val - self._last_any_at),
            )

        # 6. ALLOW (bookkeeping — mark_consumed — отдельным вызовом после
        #    успешной отправки хода, чтобы DEFER не обновлял last_fire_at).
        return Verdict(VerdictKind.ALLOW, "ok")

    # ------------------------------------------------------------- mark_consumed

    def mark_consumed(
        self, occasion: Occasion, now: Optional[float] = None
    ) -> None:
        """Зафиксировать, что по данному поводу успешно отправлен ход.

        Обновляет:
        - ``last_fire_at[kind]`` — для per-source кулдауна.
        - ``last_any_at`` — для глобального дебаунса.
        - ``consumed_one_shot`` — для одноразовых kind'ов (startup).
        """
        now_val = now if now is not None else time.monotonic()
        self._last_fire_at[occasion.kind] = now_val
        self._last_any_at = now_val
        if occasion.kind in self._one_shot_kinds:
            self._consumed_one_shot.add(occasion.kind)

    # -------------------------------------------------------------- diagnostics

    def stats(self) -> dict:
        """Диагностический снимок состояния gate (для логов / метрик)."""
        return {
            "last_fire_at": dict(self._last_fire_at),
            "last_any_at": self._last_any_at,
            "consumed_one_shot": sorted(self._consumed_one_shot),
        }


__all__ = [
    "Occasion",
    "Verdict",
    "VerdictKind",
    "OccasionGate",
]