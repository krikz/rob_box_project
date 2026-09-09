"""Unit-тесты на гонку acquire/release в LocalAvatarArbiterClient.

ADR-0051 §2.2 (issue #1999) — единственный владелец floor-ов это
:class:`rob_box_supervisor.core.locks.LockManager`. Quest-WS в
production должен звать avatar_arbiter через ROS 2 service, но пока
это не подключено, :class:`LocalAvatarArbiterClient` играет его роль
в одном процессе.

Раньше (до #1999) у Quest было ДВА источника истины о floor-ах —
``SupervisorFloorTracker`` и FSM в ``ModeManager``. Тесты на гонку
двух клиентов периодически показывали два разных вердикта (FSM
«conflict», tracker «granted» — оба правы по своим контрактам).
Теперь единственный владелец — avatar_arbiter (на production) или
его ``LocalAvatarArbiterClient`` (в тестах), и тест ниже фиксирует:
- ровно один клиент выигрывает гонку за floor;
- идемпотентность для одного и того же session_id;
- release сбрасывает состояние;
- два разных ресурса (teleop vs voice) — независимы;
- кэш :class:`AvatarStateFloorCache` пушится после каждой мутации.

Контракт гонки:
- 2 клиента одновременно зовут ``try_acquire_floor`` с РАЗНЫМИ
  ``session_id``;
- один получает ``granted=True``, другой — ``granted=False /
  reason=held_by_other``;
- ни в каком из N запусков не должно быть обоих ``granted=True``.
"""

from __future__ import annotations

from rob_box_quest.core.avatar_arbiter import LocalAvatarArbiterClient
from rob_box_quest.core.floor import (
    AvatarFloorSnapshot,
    AvatarStateFloorCache,
    FloorHolder,
    make_server_client_id,
)


# === Race: two clients compete for teleop_floor =====================


def test_two_clients_compete_only_one_wins():
    """Race: 2 клиента с разными session_id — только один granted.

    Гонка моделируется последовательными вызовами ``try_acquire_floor``
    с разными session_id (без threading — клиент однопоточный).
    Контракт ADR-0051 §2.2: ``_floor_holder`` после двух acquire —
    ровно один из session_id, второй получает ``granted=False``.

    issue #2190 (voice-vr 05): ``_floor_holder`` теперь хранит
    ``server_client_id`` (``"quest:<uuid>"``), а не голый session_id.
    Это единый формат во всех точках (gate/heartbeat/STATE_UPDATE).
    """
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    # Клиент A выигрывает.
    res_a = arbiter.try_acquire_floor(session_id="sessionA", client_id="questA")
    assert res_a.granted is True
    assert res_a.held_by == make_server_client_id("sessionA")
    assert res_a.reason in ("granted", "already_held")

    # Клиент B — отказ, держит A.
    res_b = arbiter.try_acquire_floor(session_id="sessionB", client_id="questB")
    assert res_b.granted is False
    assert res_b.held_by == make_server_client_id("sessionA")
    assert res_b.reason == "held_by_other"

    # Кэш зеркалит фактическое состояние — ровно один holder.
    assert arbiter.floor_holder == make_server_client_id("sessionA")
    assert cache.holder == make_server_client_id("sessionA")
    assert cache.is_held_by(make_server_client_id("sessionA")) is True
    assert cache.is_held_by(make_server_client_id("sessionB")) is False


def test_re_acquire_same_session_is_idempotent():
    """Re-acquire той же сессии — no-op, granted=True, reason=already_held.

    Идемпотентность важна: ws_server может позвать try_acquire_floor
    на HELLO повторно (например, после re-SUBSCRIBE) и не должен
    уронить сессию с FLOOR_HELD.
    """
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    first = arbiter.try_acquire_floor(session_id="sessionA", client_id="questA")
    second = arbiter.try_acquire_floor(session_id="sessionA", client_id="questA")

    assert first.granted is True and first.reason == "granted"
    assert second.granted is True and second.reason == "already_held"
    # Floor не ушёл в чужие руки.
    assert arbiter.floor_holder == make_server_client_id("sessionA")


def test_release_then_other_can_acquire():
    """После release другая сессия может занять floor — round-trip clean.

    Это ключевой сценарий для Quest-телеоп: op отпустил PTT, новый
    op подключился → floor уходит новому без ручной переинициализации.
    """
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    arbiter.try_acquire_floor(session_id="sessionA", client_id="questA")
    assert arbiter.release_floor("sessionA") is True
    assert arbiter.floor_holder is None
    # Кэш пуст.
    assert cache.holder is None

    # Теперь B может занять.
    res = arbiter.try_acquire_floor(session_id="sessionB", client_id="questB")
    assert res.granted is True
    assert arbiter.floor_holder == make_server_client_id("sessionB")
    assert cache.holder == make_server_client_id("sessionB")


def test_release_other_session_is_no_op():
    """release чужой сессии — False, состояние не меняется.

    Защита от «случайных» release от не-своего клиента: например,
    ws_server одного session_id не должен освобождать floor у
    другого session_id. Это часто всплывало в регрессе при двух
    WS-сессиях одновременно.
    """
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    arbiter.try_acquire_floor(session_id="sessionA", client_id="questA")
    # B пытается освободить A — no-op.
    assert arbiter.release_floor("sessionB") is False
    # Floor всё ещё у A.
    assert arbiter.floor_holder == make_server_client_id("sessionA")
    assert cache.holder == make_server_client_id("sessionA")


def test_force_release_for_session_releases_both_floors():
    """``force_release_for(session_id)`` сбрасывает ОБА floor-а если их держала эта сессия.

    Используется при watchdog-trip / WS-disconnect: одна сессия
    могла держать и teleop, и voice; при отвале нужно освободить оба.
    """
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    arbiter.try_acquire_floor(session_id="sessionA", client_id="questA")
    arbiter.try_acquire_voice(session_id="sessionA", client_id="questA")
    assert arbiter.floor_holder == make_server_client_id("sessionA")
    assert arbiter.voice_holder == "sessionA"

    released = arbiter.force_release_for("sessionA")
    assert released is True
    assert arbiter.floor_holder is None
    assert arbiter.voice_holder is None
    # Кэш чист.
    assert cache.holder is None
    assert cache.voice_holder is None


def test_force_release_for_other_session_no_op():
    """``force_release_for`` чужой сессии — False, ничего не сбрасывается."""
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    arbiter.try_acquire_floor(session_id="sessionA", client_id="questA")
    released = arbiter.force_release_for("sessionB")
    assert released is False
    assert arbiter.floor_holder == make_server_client_id("sessionA")


# === Voice floor independent of teleop ======================================


def test_voice_and_teleop_independent():
    """Voice и teleop floor-ы живут раздельно — voice может держать B
    пока teleop у A (типичный сценарий mixed: op на телеопе, телеграм
    на голосе).

    До #1999 локальный ``VoiceFloor`` мог конфликтовать с
    ``SupervisorFloorTracker`` по разным session_id в одном тесте —
    был регресс «одна сессия держит оба» vs «только teleop». Сейчас
    avatar_arbiter владеет обоими, и независимость — часть контракта.
    """
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    # A держит teleop, B держит voice.
    arbiter.try_acquire_floor(session_id="sessionA", client_id="questA")
    res_voice = arbiter.try_acquire_voice(session_id="sessionB", client_id="telegram")

    assert res_voice.granted is True
    assert arbiter.floor_holder == make_server_client_id("sessionA")
    assert arbiter.voice_holder == "sessionB"
    assert cache.holder == make_server_client_id("sessionA")
    assert cache.voice_holder == "sessionB"


def test_two_clients_compete_voice_only_one_wins():
    """Voice race: 2 клиента — только один granted.

    Аналогично teleop — для голоса тоже должен быть ровно один
    держатель. До #1999 у Quest был локальный ``VoiceFloor`` mutex
    с race-регрессом «два клиента granted одновременно».
    """
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    res_a = arbiter.try_acquire_voice(session_id="sessionA", client_id="questA")
    res_b = arbiter.try_acquire_voice(session_id="sessionB", client_id="telegram")

    assert res_a.granted is True
    assert res_b.granted is False
    assert res_b.reason == "held_by_other"
    # busy_holder присутствует на DENIED для UI label.
    assert res_b.busy_holder is not None
    assert isinstance(res_b.busy_holder, FloorHolder)
    assert res_b.busy_holder.session_id == "sessionA"


# === Cache mirror — единственный источник истины =============================


def test_cache_mirrors_after_each_mutation():
    """Каждая мутация пушит свежий AvatarFloorSnapshot в кэш.

    Это критичный инвариант ADR-0051 §2.2: ``AvatarStateFloorCache``
    читают ws_server и UI для гейта teleop_twist / voice_ptt_start.
    Если бы acquire не пушил в кэш — UI висел бы на «floor занят»
    до явного release.
    """
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    arbiter.try_acquire_floor(session_id="sessionA", client_id="questA")
    assert cache.holder == make_server_client_id("sessionA")

    arbiter.release_floor("sessionA")
    assert cache.holder is None

    arbiter.try_acquire_voice(session_id="sessionB", client_id="telegram")
    assert cache.voice_holder == "sessionB"
    # teleop остался пустым.
    assert cache.holder is None


def test_direct_cache_update_also_works():
    """Кэш можно обновлять напрямую из /avatar/state (production path).

    На production avatar_arbiter публикует /avatar/state через ROS 2,
    ws_server декодирует msgpack → ``AvatarFloorSnapshot`` → ``update()``.
    LocalAvatarArbiterClient тоже пушит через ``_push_to_cache``, и оба
    пути должны быть валидны.
    """
    cache = AvatarStateFloorCache()
    cache.update(
        AvatarFloorSnapshot(
            teleop_holder="sessionA",
            voice_holder="sessionB",
            avatar_mode="mixed",
            schema_version=2,
        )
    )
    assert cache.holder == "sessionA"
    assert cache.voice_holder == "sessionB"
    assert cache.avatar_mode == "mixed"


# === FLOOR_HELD rate-limit ==================================================


def test_floor_held_error_rate_limited_per_session():
    """ERROR{FLOOR_HELD} rate-limit — 1 Hz per session.

    На 30 Hz teleop_twist без rate-limit ws_server слал бы по 30
    ошибок/сек каждому клиенту. Анти-spam окно — 1 с.
    """
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    # Подменяем часы, чтобы не зависеть от реального времени.
    fake_now = [0.0]

    def now_fn() -> float:
        return fake_now[0]

    arbiter._now_fn = now_fn  # type: ignore[attr-defined]

    # Сразу — можно слать.
    assert arbiter.should_send_floor_held_error("sessionA") is True
    # Через 0.5 с — ещё нельзя.
    fake_now[0] = 0.5
    assert arbiter.should_send_floor_held_error("sessionA") is False
    # Через 1.1 с — снова можно.
    fake_now[0] = 1.1
    assert arbiter.should_send_floor_held_error("sessionA") is True


def test_floor_held_rate_limit_per_session_independent():
    """Rate-limit на sessionA не блокирует sessionB."""
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    arbiter.should_send_floor_held_error("sessionA")
    # Сразу после — sessionB ещё не получал ошибок.
    assert arbiter.should_send_floor_held_error("sessionB") is True


def test_reset_floor_held_rate_limit():
    """``reset_floor_held_rate_limit`` сбрасывает окно — после release
    ws_server может сразу слать следующую FLOOR_HELD, если клиент
    снова попытался занять floor."""
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    arbiter.should_send_floor_held_error("sessionA")
    assert arbiter.should_send_floor_held_error("sessionA") is False

    arbiter.reset_floor_held_rate_limit("sessionA")
    assert arbiter.should_send_floor_held_error("sessionA") is True


# === Invalid session_id =====================================================


def test_empty_session_id_returns_invalid():
    """try_acquire с пустым session_id — denied, reason=invalid_session_id."""
    cache = AvatarStateFloorCache()
    arbiter = LocalAvatarArbiterClient(cache=cache)

    res = arbiter.try_acquire_floor(session_id="", client_id="questA")
    assert res.granted is False
    assert res.reason == "invalid_session_id"

    res_voice = arbiter.try_acquire_voice(session_id="", client_id="questA")
    assert res_voice.granted is False
    assert res_voice.reason == "invalid_session_id"