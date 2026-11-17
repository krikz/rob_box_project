#!/usr/bin/env python3
"""Tests for ``rob_box_telegram.avatar_card`` (AV-24, issue #1916).

Что покрываем
-------------

* ``format_avatar_card``: режимы, держатели, длительность, stale-state.
* ``AvatarCardStore``: register / clear / record_count / on_state_update.
* throttling 2 с (``fake_clock`` через ``now_fn`` в конструкторе).
* dedup-loss-уведомлений: повторное состояние с тем же held_by → не шлём.
* утечка listener'ов: 100 регистраций в разных чатах — record_count == 100,
  но ``on_state_update`` обрабатывает все чаты **одним вызовом**.
* ``build_floor_keyboard``: варианты teleop/voice у нас / у другого / свободно.

Тесты НЕ требуют ``python-telegram-bot`` или ``rclpy`` — модуль чистый.
"""

from __future__ import annotations

import unittest

from rob_box_telegram.avatar_card import (
    EDIT_THROTTLE_S,
    STALE_THRESHOLD_S,
    AvatarCardStore,
    _detect_floor_loss,
    build_floor_keyboard,
    format_avatar_card,
    is_held_by_other,
    is_held_by_us,
)
from rob_box_telegram.supervisor_client import AvatarState


class FakeClock:
    """Подмена ``time.monotonic`` для тестов throttling/stale."""

    def __init__(self, start: float = 1000.0) -> None:
        self._now = start

    def __call__(self) -> float:
        return self._now

    def advance(self, dt: float) -> None:
        self._now += dt


# ─── format_avatar_card ──────────────────────────────────────────────────


class TestFormatAvatarCard(unittest.TestCase):
    """Формат карточки: режим, держатели, длительность, stale."""

    def test_off_state_no_holders(self) -> None:
        st = AvatarState(mode="off", teleop_floor=None, voice_floor=None)
        text = format_avatar_card(st, now_s=1000.0, last_seen_s=1000.0)
        self.assertIn("свободно", text)
        self.assertIn("Руль", text)
        self.assertIn("Голос", text)
        # off → «робот свободен»
        self.assertIn("робот свободен", text)

    def test_avatar_present_shows_mode(self) -> None:
        st = AvatarState(mode="avatar_present", teleop_floor="quest", voice_floor="quest", since_ms=0)
        text = format_avatar_card(st, now_s=1000.0, last_seen_s=1000.0)
        self.assertIn("оператор в очках", text)
        self.assertIn("оператор в очках телеопит", text)

    def test_telegram_active_holds_teleop(self) -> None:
        st = AvatarState(
            mode="telegram_active",
            teleop_floor="telegram",
            voice_floor=None,
            since_ms=180_000,  # 3 мин
        )
        text = format_avatar_card(st, now_s=1000.0, last_seen_s=1000.0)
        self.assertIn("3 мин", text)
        self.assertIn("Telegram", text)
        # Руль «у нас», голос свободен
        self.assertIn("Голос", text)

    def test_unknown_mode_falls_back_to_raw(self) -> None:
        """Неизвестный режим супервизора не должен крашить — пишем как есть."""
        st = AvatarState(mode="experimental_mode", teleop_floor=None, voice_floor=None)
        text = format_avatar_card(st, now_s=1000.0, last_seen_s=1000.0)
        self.assertIn("experimental_mode", text)

    def test_unknown_client_id_displayed_as_is(self) -> None:
        st = AvatarState(mode="mixed", teleop_floor="quest", voice_floor="some_new_client")
        text = format_avatar_card(st, now_s=1000.0, last_seen_s=1000.0)
        # known: quest → «оператор в очках»
        self.assertIn("оператор в очках", text)
        # unknown: some_new_client — отображаем как есть
        self.assertIn("some_new_client", text)

    def test_stale_state_shows_unknown(self) -> None:
        """Если /avatar/state старше STALE_THRESHOLD_S — «неизвестно»."""
        st = AvatarState(mode="avatar_present", teleop_floor="quest")
        text = format_avatar_card(st, now_s=1000.0 + STALE_THRESHOLD_S + 1, last_seen_s=1000.0)
        self.assertIn("неизвестно", text)
        # НЕ показываем фактическое состояние как «актуальное»
        self.assertNotIn("оператор в очках телеопит", text)

    def test_fresh_state_does_not_show_unknown(self) -> None:
        st = AvatarState(mode="avatar_present", teleop_floor="quest")
        text = format_avatar_card(st, now_s=1000.0 + 1.0, last_seen_s=1000.0)
        self.assertNotIn("неизвестно", text)

    def test_since_ms_zero_shown_as_zero(self) -> None:
        """since_ms=0 → «уже 0 с» (а не отрицательное)."""
        st = AvatarState(mode="mixed", teleop_floor="telegram", since_ms=0)
        text = format_avatar_card(st, now_s=1000.0, last_seen_s=1000.0)
        self.assertIn("0 с", text)

    def test_idempotent_for_same_state(self) -> None:
        """Один и тот же state → один и тот же текст (для dedup-throttling'а)."""
        st = AvatarState(mode="off", teleop_floor=None, voice_floor=None)
        a = format_avatar_card(st, now_s=1000.0, last_seen_s=1000.0)
        b = format_avatar_card(st, now_s=1000.0, last_seen_s=1000.0)
        self.assertEqual(a, b)


# ─── AvatarCardStore: register/clear/record_count ────────────────────────


class TestAvatarCardStoreBasic(unittest.TestCase):
    def test_register_and_count(self) -> None:
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        st = AvatarState(mode="off")
        store.register(chat_id=1, message_id=10, text="x", state=st)
        store.register(chat_id=2, message_id=20, text="y", state=st)
        self.assertEqual(store.record_count(), 2)

    def test_clear_removes(self) -> None:
        store = AvatarCardStore(now_fn=FakeClock())
        st = AvatarState(mode="off")
        store.register(chat_id=1, message_id=10, text="x", state=st)
        store.clear(chat_id=1)
        self.assertEqual(store.record_count(), 0)

    def test_clear_unknown_chat_no_error(self) -> None:
        store = AvatarCardStore(now_fn=FakeClock())
        store.clear(chat_id=999)  # не должно кидать

    def test_register_replaces_existing(self) -> None:
        """Повторный /avatar в том же чате — старая карточка заменяется."""
        store = AvatarCardStore(now_fn=FakeClock())
        store.register(chat_id=1, message_id=10, text="old", state=AvatarState(mode="off"))
        store.register(chat_id=1, message_id=20, text="new", state=AvatarState(mode="off"))
        rec = store.get(chat_id=1)
        self.assertEqual(rec.message_id, 20)
        self.assertEqual(rec.last_text, "new")


# ─── AvatarCardStore: throttling ────────────────────────────────────────


class TestAvatarCardStoreThrottling(unittest.TestCase):
    def test_edit_immediate_after_register(self) -> None:
        """Первое изменение после register'а → edit сразу (last_edit_s=0)."""
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        st_off = AvatarState(mode="off")
        st_on = AvatarState(mode="avatar_present", teleop_floor="quest")
        store.register(chat_id=1, message_id=10, text="<off>", state=st_off)

        actions = store.on_state_update(st_on, last_seen_s=clock())
        self.assertIn(1, actions)
        self.assertIsNotNone(actions[1].edit_text)

    def test_edit_suppressed_within_throttle_window(self) -> None:
        """Два изменения подряд в течение EDIT_THROTTLE_S → второе подавлено."""
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        st_a = AvatarState(mode="avatar_present", teleop_floor="quest", voice_floor=None)
        st_b = AvatarState(mode="mixed", teleop_floor="telegram", voice_floor="quest")
        store.register(chat_id=1, message_id=10, text="<initial>", state=st_a)

        # Первое изменение → edit
        first = store.on_state_update(st_b, last_seen_s=clock())
        self.assertIn(1, first)
        store.apply_edit(1, first[1].edit_text, st_b)

        # Сразу же (через 0.5 с) — должно подавиться
        clock.advance(0.5)
        second = store.on_state_update(st_b, last_seen_s=clock())
        self.assertEqual(second, {})

    def test_edit_allowed_after_throttle_window(self) -> None:
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        st_a = AvatarState(mode="avatar_present", teleop_floor="quest")
        st_b = AvatarState(mode="mixed", teleop_floor="telegram", voice_floor="quest")
        st_c = AvatarState(mode="telegram_active", teleop_floor="telegram", voice_floor=None)
        store.register(chat_id=1, message_id=10, text="<initial>", state=st_a)
        first = store.on_state_update(st_b, last_seen_s=clock())
        store.apply_edit(1, first[1].edit_text, st_b)

        # Перематываем время и присылаем ДРУГОЕ состояние — текст
        # изменится, throttling пропустит.
        clock.advance(EDIT_THROTTLE_S + 0.1)
        third = store.on_state_update(st_c, last_seen_s=clock())
        self.assertIn(1, third)
        self.assertIsNotNone(third[1].edit_text)


# ─── AvatarCardStore: dedup ──────────────────────────────────────────────


class TestAvatarCardStoreDedup(unittest.TestCase):
    def test_same_text_no_edit(self) -> None:
        """Если текст не изменился — никакого edit (даже после throttle)."""
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        st = AvatarState(mode="off")
        text = format_avatar_card(st, now_s=clock(), last_seen_s=clock())
        store.register(chat_id=1, message_id=10, text=text, state=st)

        # Продвигаем время, шлём то же состояние — текст тот же.
        clock.advance(EDIT_THROTTLE_S + 1.0)
        actions = store.on_state_update(st, last_seen_s=clock())
        self.assertEqual(actions, {})

    def test_not_modified_does_not_advance_throttle(self) -> None:
        """Telegram ответил 400 → мы «синхронизированы», но last_edit_s
        НЕ двигаем (это было бы бесплатное обновление)."""
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        st = AvatarState(mode="off")
        store.register(chat_id=1, message_id=10, text="<initial>", state=AvatarState(mode="avatar_present"))
        # Сразу после register'а делаем успешный edit
        store.apply_edit(1, "new text", st)
        last_edit_before = store.get(1).last_edit_s

        # Применяем "not modified" — last_edit_s не должен двинуться
        store.apply_edit_not_modified(1)
        self.assertEqual(store.get(1).last_edit_s, last_edit_before)


# ─── AvatarCardStore: loss-уведомления (dedup) ───────────────────────────


class TestAvatarCardStoreFloorLoss(unittest.TestCase):
    def test_loss_notification_once(self) -> None:
        """У нас забрали floor → ровно одно уведомление, пока не вернётся."""
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        prev = AvatarState(mode="telegram_active", teleop_floor="telegram")
        cur = AvatarState(mode="mixed", teleop_floor="quest", voice_floor="quest")
        store.register(chat_id=1, message_id=10, text="<initial>", state=prev)

        # Первый update после потери — должно быть уведомление
        actions = store.on_state_update(cur, last_seen_s=clock())
        self.assertIn(1, actions)
        self.assertEqual(actions[1].notify_floor_lost, "quest")

        # ack (мы «отправили» уведомление в Telegram)
        store.ack_loss_notified(1, "quest")
        clock.advance(EDIT_THROTTLE_S + 1)

        # Второй такой же update — уведомления быть не должно
        actions = store.on_state_update(cur, last_seen_s=clock())
        # либо {} если текст тот же, либо edit без notify_floor_lost
        for a in actions.values():
            self.assertIsNone(a.notify_floor_lost)

    def test_no_loss_when_floor_was_free(self) -> None:
        """floor был None → None, уведомления о потере нет."""
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        prev = AvatarState(mode="off", teleop_floor=None)
        cur = AvatarState(mode="mixed", teleop_floor="quest")
        store.register(chat_id=1, message_id=10, text="<init>", state=prev)
        actions = store.on_state_update(cur, last_seen_s=clock())
        if 1 in actions:
            self.assertIsNone(actions[1].notify_floor_lost)

    def test_loss_marker_cleared_when_floor_returns(self) -> None:
        """После возврата floor'а к нам → маркер сбрасывается → следующая потеря снова уведомит."""
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        prev = AvatarState(mode="telegram_active", teleop_floor="telegram")
        cur = AvatarState(mode="mixed", teleop_floor="quest")
        store.register(chat_id=1, message_id=10, text="<init>", state=prev)
        actions = store.on_state_update(cur, last_seen_s=clock())
        store.ack_loss_notified(1, "quest")

        # Возврат: floor снова у нас
        clock.advance(EDIT_THROTTLE_S + 1)
        back = AvatarState(mode="telegram_active", teleop_floor="telegram")
        store.clear_loss_marker_if_held(1, current_holder="telegram")
        actions = store.on_state_update(back, last_seen_s=clock())
        if 1 in actions:
            store.apply_edit(1, actions[1].edit_text or "", back)

        # Снова потеря → уведомление должно быть снова
        clock.advance(EDIT_THROTTLE_S + 1)
        actions = store.on_state_update(cur, last_seen_s=clock())
        self.assertIn(1, actions)
        self.assertEqual(actions[1].notify_floor_lost, "quest")


# ─── AvatarCardStore: leak guard (один listener → много чатов) ──────────


class TestAvatarCardStoreNoLeak(unittest.TestCase):
    def test_many_chats_one_update(self) -> None:
        """100 /avatar в разных чатах → record_count==100, один on_state_update
        обрабатывает все. Это и есть контракт «не N listener'ов»."""
        clock = FakeClock()
        store = AvatarCardStore(now_fn=clock)
        st_off = AvatarState(mode="off")
        st_on = AvatarState(mode="avatar_present", teleop_floor="quest")
        for chat_id in range(100):
            store.register(chat_id=chat_id, message_id=chat_id * 10, text="<init>", state=st_off)
        self.assertEqual(store.record_count(), 100)

        # Один update должен покрыть все чаты
        actions = store.on_state_update(st_on, last_seen_s=clock())
        self.assertEqual(len(actions), 100)


# ─── build_floor_keyboard ────────────────────────────────────────────────


class TestBuildFloorKeyboard(unittest.TestCase):
    def test_teleop_free_voice_free(self) -> None:
        kb = build_floor_keyboard(None, None)["rows"]
        labels = [btn["text"] for row in kb for btn in row]
        self.assertIn("🛞 Взять руль", labels)
        self.assertIn("🎤 Взять голос", labels)
        self.assertIn("🔄 Обновить", labels)

    def test_teleop_held_by_other(self) -> None:
        kb = build_floor_keyboard("quest", None)["rows"]
        labels = [btn["text"] for row in kb for btn in row]
        self.assertIn("🛞 Руль у оператор в очках", labels)
        # callback_data для кнопки с held_by != us — это «take» (попробуем взять)
        cbs = [btn["callback_data"] for row in kb for btn in row]
        self.assertIn("floor:take:teleop", cbs)

    def test_teleop_held_by_us(self) -> None:
        kb = build_floor_keyboard("telegram", None)["rows"]
        cbs = [btn["callback_data"] for row in kb for btn in row]
        self.assertIn("floor:release:teleop", cbs)

    def test_callback_data_format(self) -> None:
        """callback_data строго 'floor:{take|release}:{teleop|voice}'."""
        kb = build_floor_keyboard(None, None)["rows"]
        for row in kb:
            for btn in row:
                cb = btn["callback_data"]
                if cb.startswith("floor:"):
                    parts = cb.split(":")
                    self.assertEqual(len(parts), 3)
                    self.assertIn(parts[1], ("take", "release"))
                    self.assertIn(parts[2], ("teleop", "voice"))


# ─── is_held_by_* helpers ────────────────────────────────────────────────


class TestFloorHelpers(unittest.TestCase):
    def test_is_held_by_us(self) -> None:
        self.assertTrue(is_held_by_us("telegram", client_id="telegram"))
        self.assertFalse(is_held_by_us("quest", client_id="telegram"))
        self.assertFalse(is_held_by_us(None, client_id="telegram"))

    def test_is_held_by_other(self) -> None:
        self.assertTrue(is_held_by_other("quest", client_id="telegram"))
        self.assertFalse(is_held_by_other("telegram", client_id="telegram"))
        self.assertFalse(is_held_by_other(None, client_id="telegram"))


# ─── _detect_floor_loss: corner cases ───────────────────────────────────


class TestDetectFloorLoss(unittest.TestCase):
    def test_none_previous(self) -> None:
        self.assertIsNone(_detect_floor_loss(None, AvatarState(mode="mixed", teleop_floor="quest")))

    def test_other_lost_our_floor(self) -> None:
        prev = AvatarState(mode="telegram_active", teleop_floor="telegram")
        cur = AvatarState(mode="mixed", teleop_floor="quest")
        self.assertEqual(_detect_floor_loss(prev, cur), "quest")

    def test_floor_released_no_notification(self) -> None:
        """Floor освободился (другого держателя нет) — уведомления нет."""
        prev = AvatarState(mode="telegram_active", teleop_floor="telegram")
        cur = AvatarState(mode="off", teleop_floor=None)
        self.assertIsNone(_detect_floor_loss(prev, cur))

    def test_quest_to_quest_no_notification(self) -> None:
        prev = AvatarState(mode="avatar_present", teleop_floor="quest")
        cur = AvatarState(mode="avatar_present", teleop_floor="quest")
        self.assertIsNone(_detect_floor_loss(prev, cur))


if __name__ == "__main__":
    unittest.main()
