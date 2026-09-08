"""Sanity tests for ``rob_box_core._bridge_protocol_data``.

These guard the catalog from silent regressions: every entry must have
the fields the TS generator relies on, the union types must close over
real values, and the ``voice_set_nack.voice_id`` field must stay optional
(AV-27 contract — see meta-quest-api.md §11.2). They do NOT check that
the wire contract matches the server implementation — that is the job of
the conformance test in ``voice-vr 02``.
"""

from __future__ import annotations

import re
import unittest

from rob_box_core import _bridge_protocol_data as bpd


# Allowed payload-value shapes. The TS generator recognises:
#   * "str" / "int" / "float" / "bool" / "unknown"
#   * "list[T]" / "Record<string, T>"
#   * Trailing "?" marks a field optional.
#   * {"type": "literal", "values": [...], "optional": bool}
#   * Any string-literal (self-reference) — used for the ``cmd``/``type``
#     discriminant field, which always equals the entry's ``name`` (the
#     generator turns it into a TS literal type ``"admin_logs"`` etc).
_ATOMIC = re.compile(r"^(str|int|float|bool|unknown)$")
_OPTIONAL_ATOMIC = re.compile(r"^(str|int|float|bool|unknown)\?$")
_COMPOSITE = re.compile(r"^(list\[.+\]|Record<string,\s*.+>)$")
_OPTIONAL_COMPOSITE = re.compile(r"^(list\[.+\]|Record<string,\s*.+>)\?$")


def _is_valid_payload_value(value: object) -> bool:
    if isinstance(value, str):
        if _ATOMIC.match(value) or _OPTIONAL_ATOMIC.match(value):
            return True
        if _COMPOSITE.match(value) or _OPTIONAL_COMPOSITE.match(value):
            return True
        # Treat any other bare string as a literal-type self-reference
        # (the discriminant). The generator only emits these for the
        # ``cmd``/``type`` field of an entry whose name matches.
        return bool(re.match(r"^[a-zA-Z_][a-zA-Z0-9_]*$", value))
    if isinstance(value, dict):
        # Literal-union: {"type": "literal", "values": [...], "optional": bool}
        if value.get("type") == "literal":
            vals = value.get("values")
            if not isinstance(vals, list) or not all(isinstance(v, str) for v in vals):
                return False
            if "optional" in value and not isinstance(value["optional"], bool):
                return False
            return True
        # Inline-object schema: {field: type, ...} (e.g. linear/angular
        # on teleop_twist). Each value must itself be a valid payload type.
        for k, v in value.items():
            if not isinstance(k, str) or not k:
                return False
            if not _is_valid_payload_value(v):
                return False
        return True
    return False


# Known intentional cmd/event name collisions: the same wire-string is
# used as both a client command and a server event (or vice versa). The
# generator renders them in separate ``JsonCmd`` / ``JsonEvent`` unions
# and they remain distinct at the TS level, but at the catalog level
# the names match because the wire protocol uses the same string for
# both directions. See meta-quest-api.md §7 (ping/pong in both
# directions) and §6/§5 (stream_list exists as both cmd and event).
KNOWN_CROSS_COLLISIONS: frozenset[str] = frozenset({"ping", "stream_list"})


class CatalogShapeTest(unittest.TestCase):
    """Static structural checks on the catalog itself."""

    def test_required_top_level_fields(self) -> None:
        for entry in bpd.COMMANDS:
            self.assertIn("name", entry)
            self.assertIn("subprotocol", entry)
            self.assertIn("payload", entry)
            self.assertIn(entry["subprotocol"], ("v1", "v2"))
        for entry in bpd.EVENTS:
            self.assertIn("name", entry)
            self.assertIn("subprotocol", entry)
            self.assertIn("payload", entry)
        for entry in bpd.CONTROL_FRAMES:
            self.assertIn("name", entry)
            self.assertIn("direction", entry)
            self.assertIn("payload", entry)
        for entry in bpd.STREAMS:
            self.assertIn("topic", entry)
            self.assertIn("topic_id", entry)
            self.assertIn("kind", entry)
        for entry in bpd.ERRORS:
            self.assertIn("code", entry)

    def test_no_duplicate_names(self) -> None:
        # Cross-set collisions are allowed for entries listed in
        # KNOWN_CROSS_COLLISIONS (e.g. ``ping`` is both a cmd and an event).
        cmd_names = {c["name"] for c in bpd.COMMANDS}
        evt_names = {e["name"] for e in bpd.EVENTS}
        control_names = {c["name"] for c in bpd.CONTROL_FRAMES}
        unexpected = (cmd_names & evt_names) - KNOWN_CROSS_COLLISIONS
        self.assertEqual(unexpected, set(),
                         msg=f"unexpected cmd/event name collision: {unexpected}")
        # ``ping`` is intentionally a Cmd AND an Event.
        self.assertIn("ping", cmd_names)
        self.assertIn("ping", evt_names)
        self.assertEqual(control_names & cmd_names, set())
        self.assertEqual(control_names & evt_names, set())

    def test_payload_shapes_are_valid(self) -> None:
        for entry in bpd.COMMANDS:
            for fname, ftype in entry["payload"].items():
                self.assertTrue(_is_valid_payload_value(ftype),
                                msg=f"{entry['name']}.{fname}: bad payload type {ftype!r}")
        for entry in bpd.EVENTS:
            for fname, ftype in entry["payload"].items():
                self.assertTrue(_is_valid_payload_value(ftype),
                                msg=f"{entry['name']}.{fname}: bad payload type {ftype!r}")
        for entry in bpd.CONTROL_FRAMES:
            for fname, ftype in entry["payload"].items():
                self.assertTrue(_is_valid_payload_value(ftype),
                                msg=f"{entry['name']}.{fname}: bad payload type {ftype!r}")

    def test_cmd_discriminant_is_payload_cmd_field(self) -> None:
        # Every JSON_CMD must carry ``cmd: "<name>"`` as the discriminant.
        for entry in bpd.COMMANDS:
            self.assertEqual(entry["payload"].get("cmd"), entry["name"],
                             msg=f"{entry['name']}: payload.cmd mismatch")

    def test_event_discriminant_is_payload_type_field(self) -> None:
        for entry in bpd.EVENTS:
            self.assertEqual(entry["payload"].get("type"), entry["name"],
                             msg=f"{entry['name']}: payload.type mismatch")

    def test_topic_ids_unique(self) -> None:
        ids = [s["topic_id"] for s in bpd.STREAMS]
        self.assertEqual(len(ids), len(set(ids)),
                         msg="duplicate topic_id in STREAMS — codec-level IDs must be unique")

    def test_error_codes_unique(self) -> None:
        codes = [e["code"] for e in bpd.ERRORS]
        self.assertEqual(len(codes), len(set(codes)),
                         msg="duplicate code in ERRORS — was the FLOOR_HELD/MODE_CONFLICT dup "
                             "from session.py accidentally re-added? See voice-vr 07 §FLOOR_HELD.")

    def test_modes_and_floors_match_adr_0028(self) -> None:
        self.assertEqual(bpd.MODES,
                         ("off", "telegram_active", "avatar_present", "mixed",
                          "teleop_only", "voice_only"))
        self.assertEqual(bpd.FLOORS, ("teleop", "voice"))

    def test_voice_presets_include_av28_styles(self) -> None:
        # AV-28 §P7 — стили речи из voice_presets.yaml.
        for preset in ("technical", "street", "caveman", "business",
                       "philosopher", "lenin"):
            self.assertIn(preset, bpd.VOICE_PRESETS,
                          msg=f"AV-28 §P7: voice preset {preset!r} must be in VOICE_PRESETS")

    def test_voice_set_nack_voice_id_optional(self) -> None:
        # AV-27: voice_id may be absent (UI may not know which voice the
        # server complained about). Locked in by the conformance test —
        # see meta-quest-api.md §11.2 + tars1_text analog.
        nack = next(e for e in bpd.EVENTS if e["name"] == "voice_set_nack")
        self.assertEqual(nack["payload"]["voice_id"], "str?",
                         msg="voice_set_nack.voice_id must stay optional (AV-27 contract)")

    def test_subprotocols_order_is_v2_then_v1(self) -> None:
        # aiohttp picks the first match from the client's offered list.
        # v2 MUST be first so v2-capable clients win negotiation.
        self.assertEqual(bpd.SUBPROTOCOLS, ("robbox-quest-v2", "robbox-quest-v1"))

    def test_v2_supervisor_commands_tagged(self) -> None:
        # Sanity: every supervisor_* command is subprotocol="v2".
        for entry in bpd.COMMANDS:
            if entry["name"].startswith("supervisor_"):
                self.assertEqual(entry["subprotocol"], "v2",
                                 msg=f"{entry['name']}: supervisor commands must be v2-only")


if __name__ == "__main__":
    unittest.main()