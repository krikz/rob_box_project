"""test_build_envelope.py — wire-contract sanity for ``build_parsed_envelope``.

Issue #1997 DoD #2 implicitly requires the wire format to be
predictable (downstream consumers parse the envelope by hand — the
:class:`ReflexLayer` reconstructs a command-like object from the
payload via :func:`ReflexLayer._envelope_to_command`).

These tests pin the topic, the payload keys, and the field types
without spinning up an asyncio loop. That keeps the contract under
test even if the bus / layer refactor in the future.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict

import pytest

from rob_box_voice.command_node import CommandNode
from rob_box_voice.scheduler import EventEnvelope, ReflexLayer


@dataclass
class _Intent:
    value: str


@dataclass
class _Cmd:
    intent_value: str
    text: str
    entities: Dict[str, Any]
    confidence: float

    @property
    def intent(self) -> _Intent:
        return _Intent(self.intent_value)


def _cmd(
    intent: str = "navigate",
    text: str = "поехали",
    entities: Dict[str, Any] | None = None,
    confidence: float = 0.85,
) -> _Cmd:
    return _Cmd(
        intent_value=intent,
        text=text,
        entities=entities if entities is not None else {"destination": "kitchen"},
        confidence=confidence,
    )


class TestBuildParsedEnvelope:
    """The wire contract is defined in ADR-0054."""

    def test_topic_is_subscribe_topic(self) -> None:
        """The envelope MUST go to ``/command/parsed`` so ReflexLayer
        picks it up. Anything else silently bypasses the layer."""

        env = CommandNode.build_parsed_envelope(_cmd())  # type: ignore[arg-type]
        assert env.topic == ReflexLayer.SUBSCRIBE_TOPIC
        assert env.topic == "/command/parsed"

    def test_returns_event_envelope_instance(self) -> None:
        env = CommandNode.build_parsed_envelope(_cmd())  # type: ignore[arg-type]
        assert isinstance(env, EventEnvelope)

    def test_payload_carries_intent_text_entities_confidence(self) -> None:
        """All four wire fields must be present and in the right types.

        The :class:`ReflexLayer` only reads these four (see
        :func:`ReflexLayer._envelope_to_command`); if any is renamed
        or dropped the consumer drops the envelope with a warning.
        """

        env = CommandNode.build_parsed_envelope(_cmd(  # type: ignore[arg-type]
            intent="stop",
            text="стоп",
            entities={"x": 1},
            confidence=0.42,
        ))
        p = env.payload
        assert p["intent"] == "stop"
        assert p["text"] == "стоп"
        assert p["entities"] == {"x": 1}
        assert p["confidence"] == pytest.approx(0.42)

    def test_entities_are_defensively_copied(self) -> None:
        """The bus may hand the same payload to multiple subscribers;
        mutating the source entities dict on the caller side must not
        affect the published payload (or vice-versa)."""

        entities = {"direction": "left"}
        cmd = _cmd(intent="navigate", text="налево", entities=entities)
        env = CommandNode.build_parsed_envelope(cmd)  # type: ignore[arg-type]

        entities["direction"] = "right"
        assert env.payload["entities"] == {"direction": "left"}, (
            "build_parsed_envelope must defensively copy entities"
        )

        env.payload["entities"]["direction"] = "back"
        assert cmd.entities["direction"] == "right", (
            "mutating the published payload must not leak back to the command"
        )

    def test_confidence_is_coerced_to_float(self) -> None:
        """The parser sometimes passes numpy float64; the wire format
        must be plain float so JSON serialisation round-trips."""

        # ``int`` should be coerced — reflex layer reads it as float.
        env = CommandNode.build_parsed_envelope(  # type: ignore[arg-type]
            _cmd(confidence=1)  # type: ignore[arg-type]
        )
        assert isinstance(env.payload["confidence"], float)

    def test_text_is_coerced_to_str(self) -> None:
        """The parser can pass numpy str_ — wire is plain str."""

        class _StrLike(str):
            pass

        env = CommandNode.build_parsed_envelope(  # type: ignore[arg-type]
            _cmd(text=_StrLike("привет"))
        )
        assert isinstance(env.payload["text"], str)
        assert env.payload["text"] == "привет"

    def test_unknown_intent_still_emits_envelope(self) -> None:
        """UNKNOWN commands reach dialogue_node (existing path) but
        must ALSO produce an envelope — the layer treats them as
        NOOP and just records a metric (reflex §8.10.3).
        """

        env = CommandNode.build_parsed_envelope(  # type: ignore[arg-type]
            _cmd(intent="unknown", text="привет как дела")
        )
        assert env.payload["intent"] == "unknown"
        assert env.topic == ReflexLayer.SUBSCRIBE_TOPIC

    def test_none_entities_is_coerced_to_empty_dict(self) -> None:
        """The parser occasionally produces ``entities=None`` for
        commands with no extracted slots (a STOP with no direction).
        The wire contract says "always a mapping" so the bus
        subscriber does not drop the envelope.

        Regression: the previous implementation called
        ``dict(None)`` which raised ``TypeError`` and silently killed
        the bus publish for those commands — STOP, FOLLOW without a
        speaker, etc.
        """

        env = CommandNode.build_parsed_envelope(  # type: ignore[arg-type]
            _Cmd(
                intent_value="stop",
                text="стоп",
                entities=None,  # type: ignore[arg-type]
                confidence=0.9,
            )
        )
        assert env.payload["entities"] == {}
        assert env.payload["intent"] == "stop"

    @pytest.mark.parametrize(
        "intent",
        ["navigate", "stop", "follow", "status", "map", "vision", "unknown"],
    )
    def test_all_supported_intents_build_envelope(self, intent: str) -> None:
        """Every intent the parser emits must produce a valid envelope.
        Regression guard for #1997 — adding a new intent to the parser
        without extending the layer's classification matrix used to
        silently drop the envelope."""

        env = CommandNode.build_parsed_envelope(_cmd(intent=intent))  # type: ignore[arg-type]
        assert env.topic == ReflexLayer.SUBSCRIBE_TOPIC
        assert env.payload["intent"] == intent
