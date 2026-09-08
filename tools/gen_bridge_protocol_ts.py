#!/usr/bin/env python3
"""Generate the TS mirror of the bridge-protocol catalog.

Why this exists
---------------
The WebXR client's JSON_CMD / JSON_EVENT contracts used to be hand-written
in ``src/rob_box_quest/webxr_client/src/wire/messages.ts`` (321 lines as
of voice-vr 08), which meant the client and server drifted silently
(server announced ``voice_pipeline``, ``voice_listen_start/stop``,
``ping`` that the client never declared). ``voice-vr 07`` extracted the
catalog into ``rob_box_core._bridge_protocol_data``; this script is
the matching TS generator (ADR-0080 §2.2).

What it emits
-------------
``src/rob_box_quest/webxr_client/src/wire/protocol_generated.ts`` with
``DO NOT EDIT`` warning header. The file contains:

* ``JsonCmd`` discriminated union over every entry in ``COMMANDS``.
* ``JsonEvent`` discriminated union over every entry in ``EVENTS``.
* One ``interface XxxCmd`` / ``XxxEvent`` per entry.
* Discriminant-only string-literal types (``CommandName`` /
  ``EventName``) for narrowing without relying on the inline literal
  form, since the wire protocol occasionally sends payloads from
  older/different schemas that still parse against ``[k: string]:
  unknown`` fallback variants in hand-written code.
* Module-level constants (``ERROR_CODES``, ``MODES``, ``FLOORS``,
  ``VOICE_PRESETS``, ``VOICE_LANGUAGES``, ``QUALITY_LEVELS``).

Hand-written parts of ``messages.ts`` keep importing from this file
under the bangs:

    import type {
      JsonCmd,
      JsonEvent,
      TeleopTwistCmd,
      VoicePipelineCmd,
      // ...
    } from "./protocol_generated";

Usage
-----
    python tools/gen_bridge_protocol_ts.py            # rewrite the file
    python tools/gen_bridge_protocol_ts.py --check    # exit 1 if stale

CI uses ``--check`` (see ``.github/workflows/G-Bridge-Protocol-Drift.yml``)
to fail the build when the catalog and the committed TS mirror disagree.

Why we hand-roll this and don't use ``json-schema-to-typescript``
---------------------------------------------------------------
The catalog uses a tiny, opinionated type-language (str/int/float/bool,
list[X], Record<string,X>, optional ``?``, and ``{"type":"literal",
"values":[…]}``) that lines up 1:1 with what the LLM-facing harness
already consumes (``_tool_catalog_data``). Pulling in a heavy
json-schema-codegen dependency for one file would be a worse fit than
40 lines of string munging, and the drift detector becomes trivially
testable from CI without the npm install chain.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
import textwrap

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
CATALOG_FILE = REPO_ROOT / "src" / "rob_box_core" / "rob_box_core" / "_bridge_protocol_data.py"
OUT_FILE = (
    REPO_ROOT
    / "src" / "rob_box_quest" / "webxr_client" / "src" / "wire"
    / "protocol_generated.ts"
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _load_catalog_module():
    """Import the catalog module without triggering __init__.py side-effects.

    The catalog is a pure data module (no ROS2, no aiohttp), so this is the
    only place we pull in ``rob_box_core``. Running it on a CI box without
    a ROS2 toolchain works because ``rob_box_core/setup.py`` has no
    non-stdlib install_requires.
    """
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "_bridge_protocol_data_for_gen", CATALOG_FILE
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load catalog from {CATALOG_FILE}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# Allowed payload-value shapes. The catalog test
# (``test_bridge_protocol_data._is_valid_payload_value``) enforces the
# same grammar; we re-check it here only to give a clearer error message
# at codegen time.
_ATOMIC = re.compile(r"^(str|int|float|bool|unknown)$")
_OPTIONAL_ATOMIC = re.compile(r"^(str|int|float|bool|unknown)\?$")
_COMPOSITE = re.compile(r"^(list\[.+\]|Record<string,\s*.+>)$")
_OPTIONAL_COMPOSITE = re.compile(r"^(list\[.+\]|Record<string,\s*.+>)\?$")
_LITERAL_REF = re.compile(r"^[a-zA-Z_][a-zA-Z0-9_]*$")


def _is_valid_payload(value: object) -> bool:
    """Mirror of the runtime check in ``test_bridge_protocol_data``.

    Returns True iff ``value`` is a payload value the generator knows how
    to render. The strict version of this check lives in the test
    module; we keep a loose copy here so the generator fails fast with
    a file:line when the catalog gets a new shape.
    """
    if isinstance(value, str):
        if _ATOMIC.match(value) or _OPTIONAL_ATOMIC.match(value):
            return True
        if _COMPOSITE.match(value) or _OPTIONAL_COMPOSITE.match(value):
            return True
        return bool(_LITERAL_REF.match(value))
    if isinstance(value, dict):
        if value.get("type") == "literal":
            vals = value.get("values")
            if not isinstance(vals, list) or not all(isinstance(v, str) for v in vals):
                return False
            if "optional" in value and not isinstance(value["optional"], bool):
                return False
            return True
        # Inline-object schema (e.g. linear/angular on teleop_twist).
        if not value:
            return False
        return all(
            isinstance(k, str) and k and _is_valid_payload(v)
            for k, v in value.items()
        )
    return False


def _ts_type(value: object, indent: int = 0) -> str:
    """Render one payload value as TS.

    ``indent`` controls leading whitespace for multi-line inline objects
    (linear/angular on teleop_twist). Single-line atomic types always
    return a single token regardless.
    """
    pad = " " * indent
    if isinstance(value, str):
        if _ATOMIC.match(value):
            return {"str": "string", "int": "number", "float": "number",
                    "bool": "boolean", "unknown": "unknown"}[value]
        if _OPTIONAL_ATOMIC.match(value):
            return _ts_type(value.rstrip("?"))
        if _COMPOSITE.match(value):
            if value.startswith("list["):
                inner = value[len("list["):-1]
                return f"Array<{_ts_type(inner)}>"
            # Record<string, X> — strip prefix, strip trailing ``>``.
            inner = value[len("Record<string,"):-1].strip()
            return f"Record<string, {_ts_type(inner)}>"
        if _OPTIONAL_COMPOSITE.match(value):
            return _ts_type(value.rstrip("?"))
        # Literal-type self-reference (e.g. ``cmd: "admin_logs"``).
        return f'"{value}"'
    if isinstance(value, dict):
        if value.get("type") == "literal":
            values = value["values"]
            optional = bool(value.get("optional"))
            rendered = " | ".join(f'"{v}"' for v in values)
            return rendered  # The trailing ``?`` is added by the caller
        # Inline-object schema.
        if not value:
            return "Record<string, never>"
        lines = ["{"]
        for k, v in value.items():
            opt_marker = ""
            rendered_v = _ts_type(v)
            # If the value is a literal dict flagged optional, OR the
            # trailing ``?`` is already on the atomic/composite string,
            # honour it.
            if isinstance(v, dict) and v.get("type") == "literal" and v.get("optional"):
                opt_marker = "?"
            elif isinstance(v, str) and v.endswith("?"):
                opt_marker = ""
                rendered_v = _ts_type(v[:-1])
            sep = "," if not opt_marker else ","
            lines.append(f"{pad}  {k}{opt_marker}: {rendered_v}{sep}")
        lines.append(f"{pad}}}")
        return "\n".join(lines)
    raise ValueError(f"unhandled payload value: {value!r}")


def _render_interface(name: str, entry: dict) -> str:
    """Emit one ``interface XxxCmd`` (or ``XxxEvent``) body.

    The discriminant field (``cmd`` or ``type``) is rendered as a
    literal-string union element so the generated ``JsonCmd`` /
    ``JsonEvent`` discriminates by it.
    """
    payload = entry["payload"]
    description = entry.get("description", "")
    if not _is_valid_payload(payload):
        raise ValueError(
            f"{name}: payload does not match the generator grammar: {payload!r}"
        )

    lines: list[str] = []
    if description:
        # First line as ``/** … */`` header; multi-line gets one line per row.
        for row in description.splitlines():
            lines.append(f"  /** {row.strip()} */")
    lines.append(f"  export interface {name} {{")
    for fname, ftype in payload.items():
        if isinstance(ftype, dict) and ftype.get("type") == "literal":
            values = ftype["values"]
            optional = bool(ftype.get("optional"))
            rendered = " | ".join(f'"{v}"' for v in values)
            opt = "?" if optional else ""
            lines.append(f"    {fname}{opt}: {rendered};")
        elif isinstance(ftype, str):
            optional = ftype.endswith("?")
            inner = ftype[:-1] if optional else ftype
            if _COMPOSITE.match(inner) or _OPTIONAL_COMPOSITE.match(inner):
                # Inline-object composite (rare; left as Record<string, X>
                # fallback when composite references a named type).
                rendered = _ts_type(inner)
            else:
                rendered = _ts_type(inner)
            opt = "?" if optional else ""
            lines.append(f"    {fname}{opt}: {rendered};")
        elif isinstance(ftype, dict):
            # Inline-object schema (linear/angular on teleop_twist, etc.).
            rendered = _ts_type(ftype, indent=4)
            lines.append(f"    {fname}: {rendered};")
        else:
            raise ValueError(f"{name}.{fname}: bad payload value {ftype!r}")
    lines.append("  }")
    return "\n".join(lines)


def _render_const_array(name: str, values: tuple[str, ...]) -> str:
    """Emit ``export const <NAME> = […] as const;`` for catalog tuples."""
    inner = ", ".join(f'"{v}"' for v in values)
    return f"  export const {name} = [{inner}] as const;"


def _render_const_array_dict(name: str, values: tuple[dict, ...], fields: tuple[str, ...]) -> str:
    """Emit a const array of dict literals — for ERRORS / STREAMS."""
    rows = []
    for row in values:
        bits = []
        for f in fields:
            v = row.get(f)
            if v is None:
                continue
            if isinstance(v, str):
                bits.append(f'{f}: "{v}"')
            elif isinstance(v, bool):
                bits.append(f"{f}: {'true' if v else 'false'}")
            elif isinstance(v, int):
                bits.append(f"{f}: {v}")
            else:
                raise ValueError(f"{name}: unsupported field {f}={v!r}")
        rows.append(f"    {{ {', '.join(bits)} }}")
    body = ",\n".join(rows) if rows else ""
    return f"  export const {name} = [\n{body}\n  ] as const;"


def _pascal(name: str) -> str:
    """Convert snake_case to PascalCase for TS interface names."""
    return "".join(part.capitalize() for part in name.split("_"))


# ---------------------------------------------------------------------------
# Code generation
# ---------------------------------------------------------------------------


HEADER = """\
// ⚠️  GENERATED FILE — DO NOT EDIT BY HAND.
//
// Source of truth:
//   src/rob_box_core/rob_box_core/_bridge_protocol_data.py
//
// Regenerate with:
//   python tools/gen_bridge_protocol_ts.py
//
// Detect drift in CI with:
//   python tools/gen_bridge_protocol_ts.py --check
// (.github/workflows/G-Bridge-Protocol-Drift.yml)
//
// Hand-written parts of ``messages.ts`` import from this file; this file
// must NEVER be edited to fix a server contract bug — instead fix the
// catalog and re-run the generator (ADR-0080 §2.2).
"""


def generate(catalog) -> str:
    chunks: list[str] = [HEADER.rstrip()]

    # -- COMMAND interfaces -----------------------------------------------
    chunks.append("")
    chunks.append("// JSON_CMD — client → server (meta-quest-api.md §5)")
    cmd_names: list[str] = []
    for entry in catalog.COMMANDS:
        iface_name = _pascal(entry["name"]) + "Cmd"
        cmd_names.append(entry["name"])
        chunks.append(_render_interface(iface_name, entry))
    chunks.append("")

    # -- EVENT interfaces -------------------------------------------------
    chunks.append("// JSON_EVENT — server → client (meta-quest-api.md §6)")
    evt_names: list[str] = []
    for entry in catalog.EVENTS:
        iface_name = _pascal(entry["name"]) + "Event"
        evt_names.append(entry["name"])
        chunks.append(_render_interface(iface_name, entry))
    chunks.append("")

    # -- Discriminated unions --------------------------------------------
    chunks.append("/** Discriminated union — every JSON_CMD the server accepts. */")
    cmd_union = " | ".join(_pascal(n) + "Cmd" for n in cmd_names)
    chunks.append(f"export type JsonCmdGenerated = {cmd_union};")
    chunks.append("")
    chunks.append("/** Discriminated union — every JSON_EVENT the server emits. */")
    evt_union = " | ".join(_pascal(n) + "Event" for n in evt_names)
    chunks.append(f"export type JsonEventGenerated = {evt_union};")
    chunks.append("")

    # -- CommandName / EventName literals ---------------------------------
    chunks.append("/** All ``cmd`` discriminant values the server knows. */")
    chunks.append(
        "export type CommandName = "
        + " | ".join(f'"{n}"' for n in cmd_names)
        + ";"
    )
    chunks.append("")
    chunks.append("/** All ``type`` discriminant values the server emits. */")
    chunks.append(
        "export type EventName = "
        + " | ".join(f'"{n}"' for n in evt_names)
        + ";"
    )
    chunks.append("")

    # -- Const arrays ----------------------------------------------------
    chunks.append("// Catalog tuples — single source of truth for whitelists.")
    chunks.append(_render_const_array("MODES", catalog.MODES))
    chunks.append(_render_const_array("FLOORS", catalog.FLOORS))
    chunks.append(_render_const_array("VOICE_PRESETS", catalog.VOICE_PRESETS))
    chunks.append(_render_const_array("VOICE_LANGUAGES", catalog.VOICE_LANGUAGES))
    chunks.append(_render_const_array("QUALITY_LEVELS", catalog.QUALITY_LEVELS))
    chunks.append(_render_const_array(
        "SUBPROTOCOLS_OFFERED", tuple(s.replace("robbox-quest-", "") for s in catalog.SUBPROTOCOLS)
    ))
    chunks.append(
        _render_const_array_dict(
            "ERROR_CODES",
            tuple({"code": e["code"]} for e in catalog.ERRORS),
            ("code",),
        )
    )
    chunks.append(
        _render_const_array_dict(
            "STREAMS",
            catalog.STREAMS,
            ("topic", "topic_id", "kind", "default_quality"),
        )
    )
    chunks.append("")

    return "\n".join(chunks).rstrip() + "\n"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument(
        "--check",
        action="store_true",
        help="Exit 1 if the on-disk file differs from what would be generated."
             " Used by CI; does not touch the file.",
    )
    parser.add_argument(
        "--out",
        type=pathlib.Path,
        default=OUT_FILE,
        help=f"Output path (default: {OUT_FILE.relative_to(REPO_ROOT)}).",
    )
    args = parser.parse_args()

    catalog = _load_catalog_module()
    rendered = generate(catalog)

    if args.check:
        if not args.out.exists():
            print(f"FAIL: {args.out} does not exist — run the generator first.",
                  file=sys.stderr)
            return 1
        on_disk = args.out.read_text(encoding="utf-8")
        if on_disk != rendered:
            print(
                "FAIL: bridge-protocol TS mirror is stale vs the catalog.\n"
                f"  catalog : {CATALOG_FILE.relative_to(REPO_ROOT)}\n"
                f"  mirror  : {args.out.relative_to(REPO_ROOT)}\n"
                "Fix : run `python tools/gen_bridge_protocol_ts.py` and commit the diff.",
                file=sys.stderr,
            )
            # Print first 60 diff lines for fast eyeballing in CI logs.
            import difflib
            diff = list(difflib.unified_diff(
                on_disk.splitlines(),
                rendered.splitlines(),
                fromfile="on-disk",
                tofile="would-be",
                lineterm="",
            ))[:60]
            print("\n".join(diff), file=sys.stderr)
            return 1
        print("OK: protocol_generated.ts matches the catalog.")
        return 0

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(rendered, encoding="utf-8")
    print(f"Wrote {args.out.relative_to(REPO_ROOT)} ({len(rendered)} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
