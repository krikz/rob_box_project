#!/usr/bin/env python3
"""Generate the TS mirror of the bridge-protocol catalog.

Why this exists
---------------
The WebXR client's JSON_CMD / JSON_EVENT contracts used to be hand-written
in ``src/rob_box_quest/webxr_client/src/wire/messages.ts`` (321 lines as
of voice-vr 08), which meant the client and server drifted silently
(server announced ``voice_pipeline``, ``voice_listen_start/stop``,
``ping`` that the client never declared).

In voice-vr 07 the catalog was extracted into
``rob_box_core.bridge_protocol`` (frozen dataclasses over the wire
contract). In voice-vr 08 the **payload grammar** (typed JSON shape of
each ``cmd`` / ``type`` discriminant) was moved from a parallel
``_bridge_protocol_data.py`` onto :class:`CommandSpec.payload` and
:class:`EventSpec.payload`. This script reads those payloads and emits
``src/rob_box_quest/webxr_client/src/wire/protocol_generated.ts`` with
a ``DO NOT EDIT`` warning.

What it emits
------------
``src/rob_box_quest/webxr_client/src/wire/protocol_generated.ts``:

* ``JsonCmdGenerated`` discriminated union over every ``COMMANDS`` entry.
* ``JsonEventGenerated`` discriminated union over every ``EVENTS`` entry.
* One ``interface XxxCmd`` / ``XxxEvent`` per entry, derived from
  ``spec.payload``.
* Discriminant-only string-literal types (``CommandName`` /
  ``EventName``).
* Module-level constants (``ERROR_CODES``, ``MODES``, ``FLOORS``,
  ``VOICE_PRESETS``, ``VOICE_LANGUAGES``, ``QUALITY_LEVELS``,
  ``SUBPROTOCOLS_OFFERED``).

Hand-written parts of ``messages.ts`` keep importing from this file
under the bangs:

    import type {
      JsonCmdGenerated,
      JsonEventGenerated,
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
The catalog uses a tiny, opinionated type-language (``str`` / ``int`` /
``float`` / ``bool``, literal-dict, optional ``?`` via suffix or
``"optional": True``) that lines up 1:1 with what the runtime tests
already enforce. Pulling in a heavy json-schema-codegen dependency for
one file would be a worse fit than ~50 lines of string munging, and
the drift detector becomes trivially testable from CI without the
npm install chain.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
CATALOG_FILE = REPO_ROOT / "src" / "rob_box_core" / "rob_box_core" / "bridge_protocol.py"
OUT_FILE = (
    REPO_ROOT
    / "src" / "rob_box_quest" / "webxr_client" / "src" / "wire"
    / "protocol_generated.ts"
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _load_catalog_module():
    """Import the catalog module without triggering ``__init__.py`` side-effects.

    The catalog is a pure data module (no ROS2, no aiohttp), so this is the
    only place we pull in ``rob_box_core``. Running it on a CI box without
    a ROS2 toolchain works because ``rob_box_core/setup.py`` has no
    non-stdlib install_requires.
    """
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "rob_box_core.bridge_protocol", str(CATALOG_FILE)
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load catalog from {CATALOG_FILE}")
    module = importlib.util.module_from_spec(spec)
    # Bridge-protocol declares its classes under a submodule of
    # ``rob_box_core``; registering parents avoids
    # ``AttributeError: 'NoneType' object has no attribute '__dict__'``
    # in dataclasses._is_type.
    sys.modules["rob_box_core.bridge_protocol"] = module
    parent_name = "rob_box_core"
    if parent_name not in sys.modules:
        parent_mod = type(sys)(parent_name)
        sys.modules[parent_name] = parent_mod
    setattr(sys.modules[parent_name], "bridge_protocol", module)
    spec.loader.exec_module(module)
    return module


# Allowed payload-value shapes. Mirror of the runtime check
# ``bridge_protocol._p`` contract — atomic strings (with ``?`` suffix
# for optional), composite strings (``list[T]`` / ``Record<string, T>``),
# literal-dict (``{"type":"literal",...}``), inline-object.
_ATOMIC = re.compile(r"^(str|int|float|bool|unknown)$")
_OPTIONAL_ATOMIC = re.compile(r"^(str|int|float|bool|unknown)\?$")
_COMPOSITE = re.compile(r"^list\[.+\]$")
_OPTIONAL_COMPOSITE = re.compile(r"^list\[.+\]\?$")
_RECORD = re.compile(r"^Record<string,\s*.+>$")
_OPTIONAL_RECORD = re.compile(r"^Record<string,\s*.+>\?$")


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
            inner = value[len("list["):-1]
            return f"Array<{_ts_type(inner)}>"
        if _OPTIONAL_COMPOSITE.match(value):
            return _ts_type(value.rstrip("?"))
        if _RECORD.match(value):
            inner = value[len("Record<string,"):-1].strip()
            return f"Record<string, {_ts_type(inner)}>"
        if _OPTIONAL_RECORD.match(value):
            return _ts_type(value.rstrip("?"))
        # Plain string literal (discriminant value, e.g. ``cmd: "ping"``).
        return f'"{value}"'
    if isinstance(value, dict):
        if value.get("type") == "literal":
            values = value["values"]
            return " | ".join(f'"{v}"' for v in values)
        # Inline-object schema.
        if not value:
            return "Record<string, never>"
        lines = ["{"]
        for k, v in value.items():
            opt_marker = ""
            rendered_v = _ts_type(v)
            if isinstance(v, dict) and v.get("type") == "literal" and v.get("optional"):
                opt_marker = "?"
            lines.append(f"{pad}  {k}{opt_marker}: {rendered_v};")
        lines.append(f"{pad}}}")
        return "\n".join(lines)
    raise ValueError(f"unhandled payload value: {value!r}")


def _render_interface(name: str, payload: dict) -> str:
    """Emit one ``interface XxxCmd`` (or ``XxxEvent``) body from a payload dict.

    The payload value is read directly off ``spec.payload``; the optional
    suffix (``?`` on atomic / composite / ``"optional": True`` on literal
    dict) drives the ``?`` on the field name.
    """
    lines: list[str] = []
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
//   src/rob_box_core/rob_box_core/bridge_protocol.py
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
    for spec in catalog.COMMANDS:
        iface_name = _pascal(spec.name) + "Cmd"
        cmd_names.append(spec.name)
        chunks.append(_render_interface(iface_name, dict(spec.payload)))
    chunks.append("")

    # -- EVENT interfaces -------------------------------------------------
    chunks.append("// JSON_EVENT — server → client (meta-quest-api.md §6)")
    evt_names: list[str] = []
    for spec in catalog.EVENTS:
        iface_name = _pascal(spec.name) + "Event"
        evt_names.append(spec.name)
        chunks.append(_render_interface(iface_name, dict(spec.payload)))
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
    chunks.append(_render_const_array("VOICE_PRESETS", catalog.VOICE_PRESET_IDS))
    chunks.append(_render_const_array("VOICE_LANGUAGES", catalog.VOICE_LANGUAGES))
    chunks.append(_render_const_array(
        "SUBPROTOCOLS_OFFERED",
        ("v1", "v2"),  # mirror of meta-quest-api.md §11.1; канон держит
                       # ``FrameTypeId``-имена, но ADR-0028 §4.5 явно
                       # «v1|v2», этого достаточно для picker'а.
    ))
    chunks.append("")
    chunks.append(
        "  export const ERROR_CODES = ["
        + ", ".join('"' + e.code + '"' for e in catalog.ERROR_SPECS)
        + "] as const;"
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