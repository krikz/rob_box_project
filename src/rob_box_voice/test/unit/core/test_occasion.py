"""Unit-тесты :mod:`rob_box_voice.core.occasion` (PR-A, ADR-0101 §3.1).

Покрывают acceptance-критерии §6.2-6.6:

- §6.2 — per-source кулдаун.
- §6.3 — глобальный дебаунс (любые-два повода ближе N).
- §6.4 — стаб-фильтр (vision-only meeting/person + unknown/stub камера).
- §6.5 — startup one-shot (после ``mark_consumed`` второй вызов → DEFER).
- §6.6 — user_initiated bypass (минует ВСЕ кулдауны).

Чистые unit-тесты: без ROS2, без rclpy. ``now`` всегда подаётся явно
(монотонные секунды) — без ``time.monotonic()``.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.occasion import (
    Occasion,
    OccasionGate,
    Verdict,
    VerdictKind,
)


# ---------------------------------------------------------------------------
# §6.2 — per-source кулдаун
# ---------------------------------------------------------------------------


class TestPerSourceCooldown:
    """ALLOW → (в окне кулдауна) → DEFER(retry_after_s ≈ cooldown - elapsed)."""

    def test_first_meeting_allowed(self) -> None:
        gate = OccasionGate(source_cooldowns={"meeting": 10.0})
        v = gate.may_speak(Occasion(kind="meeting"), now=100.0)
        assert v.kind == VerdictKind.ALLOW
        assert v.retry_after_s is None

    def test_second_meeting_within_cooldown_deferred(self) -> None:
        gate = OccasionGate(source_cooldowns={"meeting": 10.0})
        gate.mark_consumed(Occasion(kind="meeting"), now=100.0)
        # +1 секунда → 9 секунд до конца кулдауна.
        v = gate.may_speak(Occasion(kind="meeting"), now=101.0)
        assert v.kind == VerdictKind.DEFER
        assert v.retry_after_s == pytest.approx(9.0)
        assert "source cooldown" in v.reason

    def test_second_meeting_after_cooldown_allowed(self) -> None:
        gate = OccasionGate(source_cooldowns={"meeting": 10.0})
        gate.mark_consumed(Occasion(kind="meeting"), now=100.0)
        # +10 секунд ровно → cooldown исчерпан (строгое < → ALLOW).
        v = gate.may_speak(Occasion(kind="meeting"), now=110.0)
        assert v.kind == VerdictKind.ALLOW

    def test_per_source_cooldowns_isolated_per_kind(self) -> None:
        """Кулдаун на meeting НЕ блокирует dj_tick и наоборот."""
        gate = OccasionGate(
            source_cooldowns={"meeting": 100.0, "dj_tick": 50.0},
            global_debounce_s=0.0,  # отключаем глобальный дебаунс
        )
        gate.mark_consumed(Occasion(kind="meeting"), now=100.0)
        # dj_tick идёт в окне meeting-кулдауна — должен быть ALLOW
        # (per-source независимы; глобальный дебаунс выключен).
        v = gate.may_speak(Occasion(kind="dj_tick"), now=101.0)
        assert v.kind == VerdictKind.ALLOW


# ---------------------------------------------------------------------------
# §6.3 — глобальный дебаунс
# ---------------------------------------------------------------------------


class TestGlobalDebounce:
    """Любые-два повода ближе N секунд → DEFER('global debounce')."""

    def test_second_any_kind_within_window_deferred(self) -> None:
        gate = OccasionGate(global_debounce_s=2.0)
        # Первый повод (любой) — ALLOW.
        v1 = gate.may_speak(Occasion(kind="meeting"), now=100.0)
        assert v1.kind == VerdictKind.ALLOW
        gate.mark_consumed(Occasion(kind="meeting"), now=100.0)
        # Через 0.5 секунды dj_tick → DEFER.
        v2 = gate.may_speak(Occasion(kind="dj_tick"), now=100.5)
        assert v2.kind == VerdictKind.DEFER
        assert "global debounce" in v2.reason
        # 2.0 - 0.5 = 1.5.
        assert v2.retry_after_s == pytest.approx(1.5)

    def test_window_passed_allows_next(self) -> None:
        gate = OccasionGate(global_debounce_s=2.0)
        gate.may_speak(Occasion(kind="meeting"), now=100.0)
        gate.mark_consumed(Occasion(kind="meeting"), now=100.0)
        # +3 секунды → глобальный дебаунс исчерпан.
        v = gate.may_speak(Occasion(kind="dj_tick"), now=103.0)
        assert v.kind == VerdictKind.ALLOW


# ---------------------------------------------------------------------------
# §6.4 — стаб-фильтр
# ---------------------------------------------------------------------------


class TestStubFilterRefuses:
    """``kind=meeting`` + ``event_type=person`` + ``source_camera ∈
    {unknown, stub}`` → REFUSE даже до user-initiated bypass и кулдаунов."""

    @pytest.mark.parametrize("bad_camera", ["unknown", "stub"])
    def test_refuse_stub_person_camera(self, bad_camera: str) -> None:
        gate = OccasionGate()
        v = gate.may_speak(
            Occasion(
                kind="meeting",
                payload={"event_type": "person", "source_camera": bad_camera},
            ),
            now=100.0,
        )
        assert v.kind == VerdictKind.REFUSE
        assert "stub event" in v.reason
        assert bad_camera in v.reason

    def test_allows_real_person_camera(self) -> None:
        """Реальная камера (не stub) — REFUSE НЕ срабатывает."""
        gate = OccasionGate(global_debounce_s=10.0)
        v = gate.may_speak(
            Occasion(
                kind="meeting",
                payload={"event_type": "person", "source_camera": "main_camera"},
            ),
            now=100.0,
        )
        assert v.kind == VerdictKind.ALLOW

    def test_stub_filter_only_for_meeting_kind(self) -> None:
        """Для kind != 'meeting' стаб-фильтр НЕ срабатывает (vision-only)."""
        gate = OccasionGate()
        v = gate.may_speak(
            Occasion(
                kind="dj_tick",
                payload={"event_type": "person", "source_camera": "unknown"},
            ),
            now=100.0,
        )
        assert v.kind == VerdictKind.ALLOW

    def test_stub_filter_ignores_other_event_types(self) -> None:
        """event_type != 'person' — стаб-фильтр пропускает."""
        gate = OccasionGate()
        v = gate.may_speak(
            Occasion(
                kind="meeting",
                payload={"event_type": "gesture", "source_camera": "unknown"},
            ),
            now=100.0,
        )
        assert v.kind == VerdictKind.ALLOW


# ---------------------------------------------------------------------------
# §6.5 — startup one-shot
# ---------------------------------------------------------------------------


class TestStartupOneShot:
    """``startup`` — только один раз за uptime (после ``mark_consumed``)."""

    def test_first_startup_allowed(self) -> None:
        gate = OccasionGate()
        v = gate.may_speak(Occasion(kind="startup"), now=100.0)
        assert v.kind == VerdictKind.ALLOW

    def test_second_startup_before_consume_allowed(self) -> None:
        """До ``mark_consumed`` второй вызов НЕ блокируется (one-shot ещё не «съеден»)."""
        gate = OccasionGate()
        v1 = gate.may_speak(Occasion(kind="startup"), now=100.0)
        assert v1.kind == VerdictKind.ALLOW
        # Сразу же, без mark_consumed — глобальный дебаунс сработает
        # (если окно не исчерпано), но НЕ one-shot. У нас окно = 2s.
        # Проверяем именно one-shot: возьмём now=110 (после дебаунса).
        v2 = gate.may_speak(Occasion(kind="startup"), now=110.0)
        assert v2.kind == VerdictKind.ALLOW  # всё ещё не consumed

    def test_startup_after_consume_deferred(self) -> None:
        gate = OccasionGate(global_debounce_s=0.0)
        gate.may_speak(Occasion(kind="startup"), now=100.0)
        gate.mark_consumed(Occasion(kind="startup"), now=100.0)
        v = gate.may_speak(Occasion(kind="startup"), now=200.0)
        assert v.kind == VerdictKind.DEFER
        assert v.reason == "one-shot already consumed"

    def test_other_kinds_not_one_shot(self) -> None:
        """Один и тот же kind (не startup) можно повторять сколько угодно."""
        gate = OccasionGate(global_debounce_s=0.0)
        gate.may_speak(Occasion(kind="dj_tick"), now=100.0)
        gate.mark_consumed(Occasion(kind="dj_tick"), now=100.0)
        gate.may_speak(Occasion(kind="dj_tick"), now=200.0)
        gate.mark_consumed(Occasion(kind="dj_tick"), now=200.0)
        # Третий вызов — всё ещё ALLOW (не one-shot).
        v = gate.may_speak(Occasion(kind="dj_tick"), now=300.0)
        assert v.kind == VerdictKind.ALLOW


# ---------------------------------------------------------------------------
# §6.6 — user_initiated bypass
# ---------------------------------------------------------------------------


class TestUserInitiatedBypassesCooldown:
    """``is_user_initiated=True`` минует ВСЕ нижестоящие шаги (кроме стаб-фильтра)."""

    def test_user_initiated_meeting_ignores_per_source_cooldown(self) -> None:
        gate = OccasionGate(source_cooldowns={"meeting": 100.0})
        gate.mark_consumed(Occasion(kind="meeting"), now=100.0)
        # Сразу же (в окне кулдауна 100s) — user_initiated → ALLOW.
        v = gate.may_speak(
            Occasion(kind="meeting", is_user_initiated=True), now=101.0
        )
        assert v.kind == VerdictKind.ALLOW
        assert v.reason == "user-initiated"

    def test_user_initiated_ignores_global_debounce(self) -> None:
        gate = OccasionGate(global_debounce_s=100.0)
        gate.mark_consumed(Occasion(kind="meeting"), now=100.0)
        # В окне глобального дебаунса — user_initiated → ALLOW.
        v = gate.may_speak(
            Occasion(kind="wake_word", is_user_initiated=True), now=101.0
        )
        assert v.kind == VerdictKind.ALLOW
        assert v.reason == "user-initiated"

    def test_user_initiated_wake_word_first_call(self) -> None:
        """wake_word без history → ALLOW (базовый случай байт-в-байт)."""
        gate = OccasionGate()
        v = gate.may_speak(
            Occasion(kind="wake_word", is_user_initiated=True), now=100.0
        )
        assert v.kind == VerdictKind.ALLOW

    def test_user_initiated_still_respects_stub_filter(self) -> None:
        """Стаб-фильтр (шаг 1) срабатывает ДО bypass (шаг 3) — это правильно:
        wake_word не использует stub payload, но если кто-то поставит
        ``is_user_initiated=True`` на meeting/person/stub, gate всё равно
        откажет.
        """
        gate = OccasionGate()
        v = gate.may_speak(
            Occasion(
                kind="meeting",
                is_user_initiated=True,
                payload={"event_type": "person", "source_camera": "stub"},
            ),
            now=100.0,
        )
        # Шаг 1 (стаб-фильтр) срабатывает ДО шага 3 (bypass) — REFUSE.
        assert v.kind == VerdictKind.REFUSE


# ---------------------------------------------------------------------------
# Базовые свойства типов и фабрика (smoke)
# ---------------------------------------------------------------------------


class TestTypeShape:
    """Замороженный dataclass Occasion, enum VerdictKind."""

    def test_occasion_is_frozen(self) -> None:
        occ = Occasion(kind="wake_word")
        with pytest.raises((AttributeError, TypeError)):
            occ.kind = "meeting"  # type: ignore[misc]

    def test_occasion_default_payload_is_independent(self) -> None:
        """У двух Occasion с пустым payload должны быть РАЗНЫЕ dict'ы
        (default_factory, не shared mutable). Frozen dataclass защищает
        от присваивания атрибутов; dict-мутация внутри payload — нет
        (это обычный dict, по дизайну dataclass field).
        """
        a = Occasion(kind="wake_word")
        b = Occasion(kind="wake_word")
        assert a.payload is not b.payload
        c = Occasion(kind="wake_word", payload={"k": 1})
        assert c.payload is not a.payload
        # Frozen dataclass: попытка присвоить новый payload → FrozenInstanceError.
        from dataclasses import FrozenInstanceError
        with pytest.raises(FrozenInstanceError):
            a.payload = {"k": 2}  # type: ignore[misc]

    def test_verdict_kind_string_enum(self) -> None:
        # Используется в логах / метриках — строковое значение должно
        # совпадать с именем.
        assert VerdictKind.ALLOW.value == "allow"
        assert VerdictKind.DEFER.value == "defer"
        assert VerdictKind.REFUSE.value == "refuse"

    def test_stats_snapshot(self) -> None:
        gate = OccasionGate()
        gate.mark_consumed(Occasion(kind="dj_tick"), now=100.0)
        gate.mark_consumed(Occasion(kind="startup"), now=101.0)
        s = gate.stats()
        assert s["last_any_at"] == 101.0
        assert s["last_fire_at"] == {"dj_tick": 100.0, "startup": 101.0}
        assert s["consumed_one_shot"] == ["startup"]
        # EventDetector оживлён: после ``mark_consumed`` для обоих kind'ов
        # он содержит timestamp'ы. NB: EventDetector использует
        # ``time.time()`` (wall-clock), а gate — ``time.monotonic()`` /
        # переданный ``now``, поэтому сравниваем только KEYS, не значения.
        last_reaction = s["detector_event_last_reaction"]
        assert set(last_reaction.keys()) == {"dj_tick", "startup"}
        assert all(isinstance(v, float) for v in last_reaction.values())


# ---------------------------------------------------------------------------
# §6.9 — параметризация: чистый gate без лишних кулдаунов
# ---------------------------------------------------------------------------


class TestEmptyGateAllowsAll:
    """Gate с дефолтами (нет source_cooldowns, global_debounce_s=0) — все ALLOW."""

    def test_empty_gate_allows_distinct_kinds(self) -> None:
        gate = OccasionGate(global_debounce_s=0.0)
        for kind in ("dj_tick", "meeting", "startup"):
            assert gate.may_speak(Occasion(kind=kind), now=100.0).kind == VerdictKind.ALLOW
            gate.mark_consumed(Occasion(kind=kind), now=100.0)