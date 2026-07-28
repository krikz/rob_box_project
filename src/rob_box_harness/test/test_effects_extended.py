"""Unit tests for the extended :class:`Effect` union (P1.4 + ADR-0001 §2.4.4).

These tests exercise the *new* effect types that the ADR names but
that :mod:`rob_box_harness.effects` did not yet ship:

* ``SendReplyEffect`` — send a chat reply to a channel (Telegram).
* ``SpeakEffect`` — synthesize TTS via the speech pipeline.
* ``PlaySoundEffect`` — trigger a sound effect node.
* ``SetLEDEffect`` — drive the LED matrix.
* ``MoveEffect`` — publish a velocity command.

The tests use the existing :class:`RecordingBus` to assert the *shape*
and *serialisation* of each effect, plus a tiny bus per kind to assert
the routing contract (only the matching bus consumes the effect).

Per the host decision H.3 (P1.4 scope), only ``TelegramBus`` ships as
a concrete fan-out destination; TTS/Sound/LED/Move stay on
``NoopBus`` until P2. The tests reflect that — ``TelegramBus`` is the
one bus with real routing, the others simply verify their effects are
serialisable and identifiable.
"""

from __future__ import annotations

import json
from dataclasses import dataclass

import pytest

from rob_box_harness.effects import (
    CompositeBus,
    EffectContext,
    LogEffect,
    NoopBus,
    PlaySoundEffect,
    RecordingBus,
    SendReplyEffect,
    SetLEDEffect,
    SideEffectBus,
    SpeakEffect,
    MoveEffect,
    TelegramBus,
    to_dict,
    from_dict,
    effect_kind,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


@dataclass
class _Channel:
    """Tiny stand-in for a Telegram bot sink.

    The bus only needs ``send_message(chat_id, text)``; we capture
    every call so tests can assert the exact channel/text pair.
    """

    sent: list[tuple[str, str, str | None]]

    async def send_message(self, chat_id: str, text: str, reply_markup: str | None = None) -> None:
        self.sent.append((chat_id, text, reply_markup))


# ---------------------------------------------------------------------------
# Effect identity + serialisation
# ---------------------------------------------------------------------------


class TestEffectIdentity:
    def test_send_reply_effect_kind(self) -> None:
        eff = SendReplyEffect(channel="123", text="hello")
        assert effect_kind(eff) == "send_reply"

    def test_speak_effect_kind(self) -> None:
        eff = SpeakEffect(text="hello", ssml=None, voice=None)
        assert effect_kind(eff) == "speak"

    def test_play_sound_effect_kind(self) -> None:
        eff = PlaySoundEffect(name="beep")
        assert effect_kind(eff) == "play_sound"

    def test_set_led_effect_kind(self) -> None:
        eff = SetLEDEffect(pattern="blink", color="red", duration_ms=500)
        assert effect_kind(eff) == "set_led"

    def test_move_effect_kind(self) -> None:
        eff = MoveEffect(linear=0.1, angular=0.0)
        assert effect_kind(eff) == "move"


class TestEffectSerialisation:
    """``to_dict`` / ``from_dict`` is the snapshot round-trip contract."""

    def test_send_reply_roundtrip(self) -> None:
        eff = SendReplyEffect(channel="chat-9", text="Привет", reply_markup="main")
        restored = from_dict(to_dict(eff))
        assert isinstance(restored, SendReplyEffect)
        assert restored == eff

    def test_speak_roundtrip(self) -> None:
        eff = SpeakEffect(text="текст", ssml="<speak>текст</speak>", voice="ru-female")
        restored = from_dict(to_dict(eff))
        assert restored == eff

    def test_play_sound_roundtrip(self) -> None:
        eff = PlaySoundEffect(name="alert")
        restored = from_dict(to_dict(eff))
        assert restored == eff

    def test_set_led_roundtrip(self) -> None:
        eff = SetLEDEffect(pattern="pulse", color="blue", duration_ms=250)
        restored = from_dict(to_dict(eff))
        assert restored == eff

    def test_move_roundtrip(self) -> None:
        eff = MoveEffect(linear=0.5, angular=-0.25)
        restored = from_dict(to_dict(eff))
        assert restored == eff

    def test_unknown_kind_raises(self) -> None:
        with pytest.raises(ValueError, match="unknown effect kind"):
            from_dict({"kind": "made_up", "payload": {}})

    def test_serialised_payload_is_json_safe(self) -> None:
        eff = SendReplyEffect(channel="c", text="t", reply_markup="r")
        # reply_markup is the only field that might be a complex object;
        # here we pass a plain string, so json.dumps should succeed.
        json.dumps(to_dict(eff))


# ---------------------------------------------------------------------------
# TelegramBus routing
# ---------------------------------------------------------------------------


class TestTelegramBus:
    def _bus(self) -> tuple[TelegramBus, _Channel]:
        channel = _Channel(sent=[])
        return TelegramBus(channel=channel), channel  # type: ignore[arg-type]

    def test_routes_send_reply(self) -> None:
        """Same intent as the async version, expressed sync to avoid
        depending on pytest-asyncio's auto-mode (the project uses
        ``asyncio.run`` explicitly throughout the harness test suite).
        """
        bus, channel = self._bus()
        self._run(bus.dispatch(SendReplyEffect(channel="42", text="hello")))
        assert channel.sent == [("42", "hello", None)]

    def _run(self, coro):  # noqa: ANN001 - sync helper
        import asyncio
        return asyncio.run(coro)

    def test_send_reply_dispatch_sync(self) -> None:
        bus, channel = self._bus()
        self._run(bus.dispatch(SendReplyEffect(channel="42", text="hi", reply_markup="main")))
        assert channel.sent == [("42", "hi", "main")]

    def test_ignores_non_send_reply(self) -> None:
        bus, channel = self._bus()
        # SpeakEffect goes elsewhere; TelegramBus must not see it.
        self._run(bus.dispatch(SpeakEffect(text="x")))
        self._run(bus.dispatch(PlaySoundEffect(name="y")))
        self._run(bus.dispatch(SetLEDEffect(pattern="z", color="r", duration_ms=1)))
        self._run(bus.dispatch(MoveEffect(linear=0.0, angular=0.0)))
        self._run(bus.dispatch(LogEffect(message="ignored too")))
        assert channel.sent == []


# ---------------------------------------------------------------------------
# CompositeBus fan-out (TelegramBus + RecordingBus)
# ---------------------------------------------------------------------------


class TestCompositeBusWithTelegram:
    """H.3 says only TelegramBus ships in P1.4. ``CompositeBus`` lets us
    combine TelegramBus (real channel) with RecordingBus (audit log)."""

    def _run(self, coro):  # noqa: ANN001
        import asyncio
        return asyncio.run(coro)

    def test_send_reply_routes_to_both(self) -> None:
        channel = _Channel(sent=[])
        telegram = TelegramBus(channel=channel)  # type: ignore[arg-type]
        recording = RecordingBus()
        bus: SideEffectBus = CompositeBus([telegram, recording])

        self._run(bus.dispatch(SendReplyEffect(channel="7", text="Привет")))

        assert channel.sent == [("7", "Привет", None)]
        assert len(recording.effects) == 1
        assert isinstance(recording.effects[0], SendReplyEffect)

    def test_speak_only_routes_to_recording_in_p1_4(self) -> None:
        """Per H.3, TTSBus is deferred to P2; Speak must NOT raise, just
        land on RecordingBus when the harness wires that up."""
        telegram = TelegramBus(channel=_Channel(sent=[]))  # type: ignore[arg-type]
        recording = RecordingBus()
        bus: SideEffectBus = CompositeBus([telegram, recording])

        self._run(bus.dispatch(SpeakEffect(text="hi")))

        # TelegramBus ignores it; RecordingBus catches it.
        assert len(recording.effects) == 1
        assert isinstance(recording.effects[0], SpeakEffect)


# ---------------------------------------------------------------------------
# Apply() smoke — every new effect has a no-op apply() so it survives
# NoopBus dispatch (H.3: bus without a concrete impl).
# ---------------------------------------------------------------------------


class TestEffectApplyNoop:
    def _run(self, coro):  # noqa: ANN001
        import asyncio
        return asyncio.run(coro)

    def test_every_effect_apply_is_callable(self) -> None:
        ctx = EffectContext(harness="noop")
        for eff in (
            SendReplyEffect(channel="c", text="t"),
            SpeakEffect(text="t"),
            PlaySoundEffect(name="n"),
            SetLEDEffect(pattern="p", color="r", duration_ms=1),
            MoveEffect(linear=0.0, angular=0.0),
        ):
            # Should not raise; apply() may log but does not require external services.
            self._run(eff.apply(ctx))

    def test_noop_bus_accepts_new_effects(self) -> None:
        bus = NoopBus()
        self._run(bus.dispatch(SendReplyEffect(channel="c", text="t")))
        # NoopBus counts invocations but does not raise on unknown types.
        assert bus.count == 1