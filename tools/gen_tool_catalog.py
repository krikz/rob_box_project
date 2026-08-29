#!/usr/bin/env python3
"""Generate the shared dialogue-tool catalog from the ``MCPTool`` classes.

Why this exists
---------------
The LLM-facing tool catalog used to be written **twice**: once as
``MCPTool`` subclasses in ``rob_box_mcp_tools/tools/*.py`` (which own
``execute()`` and are what the MCP server actually runs), and once as
hand-maintained ``ToolSpec`` manifests in
``rob_box_harness/core/tool_registry.py`` (which is what the LLM actually
saw). The two drifted badly — parameter *names* disagreed on
``navigate_to_waypoint`` and ``move_direction``, 13 registered tools were
missing from the LLM catalog entirely, and 29 of 38 shared descriptions
had degraded to one-line stubs on the harness side.

Now there is one declaration — the tool class — and this script derives
the pure-Python catalog that ``rob_box_harness`` (and anything else that
must not import ROS2) consumes.

Why AST instead of importing the classes
----------------------------------------
``rob_box_mcp_tools.tools`` cannot be imported without a ROS2 runtime
(``navigation.py`` imports ``rclpy.action`` at module scope, and most
tools build publishers in ``__init__``). Parsing the source keeps this
script runnable in plain CI, on a laptop, and inside the drift test.

Usage
-----
    python tools/gen_tool_catalog.py            # rewrite the data module
    python tools/gen_tool_catalog.py --check    # exit 1 if it is stale
"""

from __future__ import annotations

import argparse
import ast
import pathlib
import pprint
import sys
from typing import Any

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
TOOLS_DIR = REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "tools"
OUT_FILE = REPO_ROOT / "src" / "rob_box_core" / "rob_box_core" / "_tool_catalog_data.py"

DYNAMIC = object()


class ToolSourceError(RuntimeError):
    """A tool class could not be read statically — the generator must not guess."""


# ---------------------------------------------------------------------------
# AST helpers
# ---------------------------------------------------------------------------


def _literal(node: ast.AST) -> Any:
    """Evaluate a literal node, or return :data:`DYNAMIC` if it is computed."""
    try:
        return ast.literal_eval(node)
    except (ValueError, TypeError, SyntaxError, MemoryError, RecursionError):
        return DYNAMIC


def _returned_literal(fn: ast.AST) -> Any:
    """Return the literal a single-``return`` property yields."""
    for node in ast.walk(fn):
        if isinstance(node, ast.Return) and node.value is not None:
            return _literal(node.value)
    return None


def _param_from_call(call: ast.Call) -> dict[str, Any]:
    """Turn one ``MCPToolParameter(...)`` call into a plain dict."""
    kwargs = {kw.arg: kw.value for kw in call.keywords if kw.arg}
    param: dict[str, Any] = {}
    for key in ("name", "type", "description", "required", "enum", "default", "enum_strict"):
        if key in kwargs:
            value = _literal(kwargs[key])
            if value is DYNAMIC:
                # ``enum=list(KNOWN_ANIMATIONS)`` and friends: resolve the
                # module-level constant the call wraps rather than guessing.
                value = _resolve_constant(kwargs[key])
            param[key] = value
    param.setdefault("required", True)
    for nested, key in (("items", "items"), ("properties", "properties")):
        if nested in kwargs:
            param[key] = _nested_schema(kwargs[nested])
    return param


def _nested_schema(node: ast.AST) -> Any:
    """Recurse into ``items=MCPToolParameter(...)`` / ``properties={...}``."""
    if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "MCPToolParameter":
        return _param_from_call(node)
    if isinstance(node, ast.Dict):
        out = {}
        for key, value in zip(node.keys, node.values):
            key_literal = _literal(key) if key is not None else None
            out[key_literal] = _nested_schema(value)
        return out
    return None


#: Module-level constants the generator is allowed to resolve, filled in by
#: :func:`_collect_module_constants` per source file.
_MODULE_CONSTANTS: dict[str, Any] = {}

#: ``{"ClassName": {"ATTR": <ast node>}}`` for the file being parsed. Kept as
#: AST nodes so that ``DIRECTIONS = {"вперёд": {..., math.pi / 2}}`` — whose
#: *values* are not literals — can still answer ``.keys()``.
_CLASS_CONSTANTS: dict[str, dict[str, ast.AST]] = {}

#: The class whose ``parameters`` property is being read, so ``self.ATTR``
#: resolves against the right class body.
_CURRENT_CLASS: str = ""


def _class_attr_node(name: str, owner: str = "") -> ast.AST | None:
    return _CLASS_CONSTANTS.get(owner or _CURRENT_CLASS, {}).get(name)


def _resolve_constant(node: ast.AST) -> Any:
    """Resolve the handful of constant shapes tool schemas actually use.

    Supported: ``NAME``, ``self.ATTR``, ``OtherClass.ATTR``, ``list(...)`` /
    ``tuple(...)`` / ``sorted(...)`` around any of those, and ``.keys()`` on a
    dict literal (used by ``move_direction``, whose dict *values* contain
    ``math.pi`` and so are not literal-evaluable).
    """
    # NAME
    if isinstance(node, ast.Name):
        if node.id in _MODULE_CONSTANTS:
            return _MODULE_CONSTANTS[node.id]
        attr_node = _class_attr_node(node.id)
        if attr_node is not None:
            return _resolve_constant(attr_node)
        return DYNAMIC

    # self.ATTR / SomeClass.ATTR
    if isinstance(node, ast.Attribute):
        owner = getattr(node.value, "id", "")
        attr_node = _class_attr_node(node.attr, "" if owner == "self" else owner)
        if attr_node is not None:
            return _resolve_constant(attr_node)
        return DYNAMIC

    if isinstance(node, ast.Call):
        func = node.func
        # list(X) / tuple(X) / sorted(X)
        if getattr(func, "id", None) in ("list", "tuple", "sorted") and len(node.args) == 1:
            inner = _resolve_constant(node.args[0])
            if inner is not DYNAMIC and inner is not None:
                values = list(inner)
                return sorted(values) if func.id == "sorted" else values
            return DYNAMIC
        # X.keys() — read the keys straight off the dict literal.
        if isinstance(func, ast.Attribute) and func.attr == "keys":
            target = func.value
            resolved_node = target
            if isinstance(target, (ast.Name, ast.Attribute)):
                owner = getattr(getattr(target, "value", None), "id", "")
                found = _class_attr_node(
                    getattr(target, "attr", getattr(target, "id", "")),
                    "" if owner in ("self", "") else owner,
                )
                if found is not None:
                    resolved_node = found
            if isinstance(resolved_node, ast.Dict):
                keys = [_literal(k) for k in resolved_node.keys if k is not None]
                if all(isinstance(k, str) for k in keys):
                    return keys
            inner = _resolve_constant(target)
            if isinstance(inner, dict):
                return list(inner)
        return DYNAMIC

    literal = _literal(node)
    return literal if literal is not DYNAMIC else DYNAMIC


def _collect_class_constants(tree: ast.Module) -> dict[str, dict[str, ast.AST]]:
    """Collect ``ClassName.ATTR = <expr>`` assignments in each class body."""
    out: dict[str, dict[str, ast.AST]] = {}
    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        attrs: dict[str, ast.AST] = {}
        for node in cls.body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name):
                        attrs[target.id] = node.value
            elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                if node.value is not None:
                    attrs[node.target.id] = node.value
        out[cls.name] = attrs
    return out


#: Package-level modules whose literal constants tool schemas may reference.
#: Kept as an explicit list rather than "resolve any import" so the generator
#: never silently guesses at a value it cannot actually see.
SHARED_CONSTANT_MODULES = (TOOLS_DIR.parent / "animations.py",)


def _collect_shared_constants() -> dict[str, Any]:
    """Constants that tools in different files legitimately share.

    ``KNOWN_ANIMATIONS`` lives in ``rob_box_mcp_tools.animations`` precisely
    so ``speak_text`` and ``play_animation`` cannot disagree; the generator
    has to follow that import to render either enum.
    """
    shared: dict[str, Any] = {}
    for module in SHARED_CONSTANT_MODULES:
        if not module.exists():
            continue
        shared.update(_collect_module_constants(ast.parse(module.read_text(encoding="utf-8"))))
    return shared


def _collect_module_constants(tree: ast.Module) -> dict[str, Any]:
    """Collect top-level ``NAME = <literal>`` assignments."""
    constants: dict[str, Any] = {}
    for node in tree.body:
        targets = []
        if isinstance(node, ast.Assign):
            targets = [t for t in node.targets if isinstance(t, ast.Name)]
            value = node.value
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            targets = [node.target]
            value = node.value
        else:
            continue
        if value is None:
            continue
        literal = _literal(value)
        if literal is not DYNAMIC:
            for target in targets:
                constants[target.id] = literal
    return constants


# ---------------------------------------------------------------------------
# Dynamic descriptions
# ---------------------------------------------------------------------------


def _vibe_preset_description() -> str:
    """Rebuild ``set_vibe_preset``'s runtime-computed description.

    The tool formats its description from ``MusicManager.VIBE_PRESETS`` so
    the preset list can never go stale at runtime. The generator mirrors
    that formatting from the same literal, which means a preset added to
    ``music.py`` shows up as a catalog diff in CI instead of silently
    diverging from what the LLM is told.
    """
    tree = ast.parse((TOOLS_DIR / "music.py").read_text(encoding="utf-8"))
    presets = None
    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign) and getattr(node.target, "id", "") == "VIBE_PRESETS":
            presets = _literal(node.value)
        elif isinstance(node, ast.Assign) and any(
            getattr(t, "id", "") == "VIBE_PRESETS" for t in node.targets
        ):
            presets = _literal(node.value)
    if not isinstance(presets, dict):
        raise ToolSourceError("could not read MusicManager.VIBE_PRESETS from music.py")
    presets_desc = ", ".join(
        f"{name} (scale={p['scale']}, bpm={p['bpm']})" for name, p in presets.items()
    )
    return (
        "Применить вайб-пресет для быстрой настройки музыкального контекста. "
        "Устанавливает скейл, BPM и тонику в Renardo одной командой. "
        f"Доступные пресеты: {presets_desc}. "
        "Устанавливает: Clock.bpm, Scale.default, Root.default (целое число полутонов от C)."
    )


#: Tools whose ``description`` property is computed at runtime. Each entry
#: rebuilds the exact same string from the exact same source literal.
DYNAMIC_DESCRIPTIONS = {
    "set_vibe_preset": _vibe_preset_description,
}


def _sound_pack_triggers() -> list[str]:
    """Read ``play_sound``'s enum from the sound pack the tool itself loads.

    ``PlaySoundTool`` fills ``_available_sounds`` at construction time from
    ``sound_pack/sound_catalog.json``. The generator reads the same file, so
    adding a sound to the pack shows up as a catalog diff instead of leaving
    the LLM with a stale list.
    """
    import json

    catalog = json.loads(
        (REPO_ROOT / "sound_pack" / "sound_catalog.json").read_text(encoding="utf-8")
    )
    # Mirrors PlaySoundTool._load_sounds_from_catalog exactly: the enum is the
    # ``trigger`` names, NOT the ``.mp3`` filenames the dict is keyed by.
    triggers = sorted(
        info["trigger"]
        for filename, info in catalog.get("sounds", {}).items()
        if filename.endswith(".mp3") and isinstance(info, dict) and "trigger" in info
    )
    if not triggers:
        raise ToolSourceError("sound_pack/sound_catalog.json yielded no triggers")
    return triggers


#: ``(tool_name, param_name)`` → resolver, for enums built from runtime data
#: rather than from a literal in the tool module.
DYNAMIC_ENUMS = {
    ("play_sound", "sound"): _sound_pack_triggers,
}


# ---------------------------------------------------------------------------
# Schema assembly — mirrors MCPTool.to_openai_tool_format()
# ---------------------------------------------------------------------------


def _json_schema(param: dict[str, Any]) -> dict[str, Any]:
    """Mirror :meth:`MCPToolParameter.to_json_schema`."""
    schema: dict[str, Any] = {
        "type": param.get("type"),
        "description": param.get("description", ""),
    }
    if param.get("enum") is not None:
        schema["enum"] = list(param["enum"])
    if param.get("default") is not None:
        schema["default"] = param["default"]
    if param.get("type") == "object" and param.get("properties"):
        schema["properties"] = {
            name: _json_schema(sub) for name, sub in param["properties"].items()
        }
        schema["required"] = [
            name for name, sub in param["properties"].items() if sub.get("required")
        ]
        schema["additionalProperties"] = False
    if param.get("type") == "array" and param.get("items"):
        schema["items"] = _json_schema(param["items"])
    return schema


def _parameters_schema(params: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {p["name"]: _json_schema(p) for p in params},
        "required": [p["name"] for p in params if p.get("required")],
        "additionalProperties": False,
    }


# ---------------------------------------------------------------------------
# Extraction
# ---------------------------------------------------------------------------


def extract_tools() -> list[dict[str, Any]]:
    """Read every ``MCPTool`` subclass under ``tools/`` into catalog entries."""
    global _MODULE_CONSTANTS, _CLASS_CONSTANTS, _CURRENT_CLASS, _PARAM_FACTORIES
    entries: list[dict[str, Any]] = []
    shared_constants = _collect_shared_constants()

    for source in sorted(TOOLS_DIR.glob("*.py")):
        if source.name == "__init__.py":
            continue
        tree = ast.parse(source.read_text(encoding="utf-8"))
        _MODULE_CONSTANTS = {**shared_constants, **_collect_module_constants(tree)}
        _CLASS_CONSTANTS = _collect_class_constants(tree)
        _PARAM_FACTORIES = _collect_param_factories(tree)

        for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
            bases = {getattr(b, "id", getattr(b, "attr", "")) for b in cls.bases}
            if "MCPTool" not in bases:
                continue
            _CURRENT_CLASS = cls.name

            entry: dict[str, Any] = {
                "llm_visible": True,
                "read_only": False,
                "destructive": True,
                "idempotent": False,
                "execution_type": "medium",
            }
            params: list[dict[str, Any]] | None = None
            signature: dict[str, Any] | None = None

            for fn in [n for n in cls.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
                if fn.name == "name":
                    entry["name"] = _returned_literal(fn)
                elif fn.name == "description":
                    entry["description"] = _returned_literal(fn)
                elif fn.name == "parameters":
                    params = _extract_parameters(fn, cls.name, source.name)
                elif fn.name in ("read_only", "destructive", "idempotent", "llm_visible"):
                    value = _returned_literal(fn)
                    if isinstance(value, bool):
                        entry[fn.name] = value
                elif fn.name == "execution_type":
                    for n in ast.walk(fn):
                        if isinstance(n, ast.Attribute) and getattr(n.value, "id", "") == "ToolExecutionType":
                            entry["execution_type"] = n.attr.lower()
                elif fn.name == "execute":
                    signature = _signature(fn)

            name = entry.get("name")
            if not isinstance(name, str) or not name:
                continue
            if params is None:
                raise ToolSourceError(f"{cls.name} ({source.name}) has no `parameters` property")
            if signature is None:
                raise ToolSourceError(f"{cls.name} ({source.name}) has no `execute` method")

            description = entry.get("description")
            if description is DYNAMIC or not isinstance(description, str):
                resolver = DYNAMIC_DESCRIPTIONS.get(name)
                if resolver is None:
                    raise ToolSourceError(
                        f"{name} has a computed description and no resolver in "
                        "DYNAMIC_DESCRIPTIONS — add one so the catalog stays exact"
                    )
                description = resolver()
            entry["description"] = description.strip()

            for param in params:
                if param.get("enum") is DYNAMIC:
                    resolver = DYNAMIC_ENUMS.get((name, param.get("name")))
                    if resolver is None:
                        raise ToolSourceError(
                            f"{name}.{param.get('name')} has a computed enum and no "
                            "resolver in DYNAMIC_ENUMS — add one so the LLM is told "
                            "the same values the tool validates against"
                        )
                    param["enum"] = resolver()

            entry["parameters"] = _parameters_schema(params)
            entry["signature"] = signature
            entries.append(entry)

    entries.sort(key=lambda e: e["name"])
    return entries


#: Module-level ``def _foo_param(...) -> MCPToolParameter`` factories in the
#: file being parsed. ``minimax_music.py`` shares parameters between tools
#: this way, so the generator has to follow the call to see them.
_PARAM_FACTORIES: dict[str, ast.FunctionDef] = {}


def _collect_param_factories(tree: ast.Module) -> dict[str, ast.FunctionDef]:
    factories: dict[str, ast.FunctionDef] = {}
    for node in tree.body:
        if not isinstance(node, ast.FunctionDef):
            continue
        returns_param = any(
            isinstance(n, ast.Return)
            and isinstance(n.value, ast.Call)
            and getattr(n.value.func, "id", None) == "MCPToolParameter"
            for n in ast.walk(node)
        )
        if returns_param:
            factories[node.name] = node
    return factories


def _extract_parameters(fn: ast.AST, cls_name: str, filename: str) -> list[dict[str, Any]]:
    """Read the list a ``parameters`` property returns.

    Every element must resolve to a concrete ``MCPToolParameter`` — either
    written inline or produced by a module-level factory. Anything else
    raises: a schema the generator only *partly* understands is worse than
    no catalog at all, because the missing parameters would silently vanish
    from what the LLM is told (this is exactly how ``gen_*``'s mandatory
    ``track_id`` went missing while ``execute()`` still required it).
    """
    returned = None
    for node in ast.walk(fn):
        if isinstance(node, ast.Return) and node.value is not None:
            returned = node.value
            break
    if not isinstance(returned, (ast.List, ast.Tuple)):
        raise ToolSourceError(
            f"{cls_name} ({filename}): `parameters` must return a list literal"
        )

    params: list[dict[str, Any]] = []
    for element in returned.elts:
        params.append(_resolve_param_element(element, cls_name, filename))
    return params


def _resolve_param_element(node: ast.AST, cls_name: str, filename: str) -> dict[str, Any]:
    if isinstance(node, ast.Call):
        callee = getattr(node.func, "id", None)
        if callee == "MCPToolParameter":
            return _param_from_call(node)
        factory = _PARAM_FACTORIES.get(callee or "")
        if factory is not None:
            return _param_from_factory(factory, node)
    raise ToolSourceError(
        f"{cls_name} ({filename}): cannot resolve a `parameters` entry "
        f"({ast.dump(node)[:120]}…) — add a factory or inline the parameter"
    )


def _param_from_factory(factory: ast.FunctionDef, call: ast.Call) -> dict[str, Any]:
    """Resolve ``_prompt_param(required=False)`` style helpers.

    Keyword arguments at the call site override the factory's defaults, so
    ``_prompt_param(required=False)`` yields an optional parameter while a
    bare ``_prompt_param()`` yields the mandatory one.
    """
    inner = next(
        n
        for n in ast.walk(factory)
        if isinstance(n, ast.Call) and getattr(n.func, "id", None) == "MCPToolParameter"
    )
    param = _param_from_call(inner)

    # Factory defaults, then call-site overrides, for any field the factory
    # forwards from its own signature (`required=required`).
    forwarded = {
        kw.arg: kw.value.id
        for kw in inner.keywords
        if kw.arg and isinstance(kw.value, ast.Name)
    }
    arg_names = [a.arg for a in factory.args.args]
    defaults = dict(
        zip(arg_names[len(arg_names) - len(factory.args.defaults):], factory.args.defaults)
    )
    supplied = {kw.arg: kw.value for kw in call.keywords if kw.arg}
    for positional_name, value in zip(arg_names, call.args):
        supplied[positional_name] = value

    for field, source_arg in forwarded.items():
        node = supplied.get(source_arg, defaults.get(source_arg))
        if node is not None:
            resolved = _literal(node)
            if resolved is not DYNAMIC:
                param[field] = resolved
    return param


def _is_nested_param(fn: ast.AST, target: ast.Call) -> bool:
    """True when *target* sits inside another ``MCPToolParameter(...)`` call."""
    for node in ast.walk(fn):
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "MCPToolParameter":
            if node is target:
                continue
            for descendant in ast.walk(node):
                if descendant is target:
                    return True
    return False


def _signature(fn: ast.AST) -> dict[str, Any]:
    """Record ``execute()``'s accepted arguments — the real runtime contract."""
    args = fn.args
    positional = [a.arg for a in args.args][1:]  # drop self
    defaults = len(args.defaults)
    required = positional[: len(positional) - defaults] if defaults else list(positional)
    return {
        "params": positional + [a.arg for a in args.kwonlyargs],
        "required": required,
        "accepts_kwargs": args.kwarg is not None,
    }


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

HEADER = '''"""Dialogue tool catalog — GENERATED, do not edit by hand.

Source of truth: the ``MCPTool`` subclasses in
``src/rob_box_mcp_tools/rob_box_mcp_tools/tools/*.py``.

Regenerate with::

    python tools/gen_tool_catalog.py

``test_tool_catalog_is_current`` fails the build when this file drifts from
the tool classes, which is what keeps the LLM-facing catalog and the
executable tools from disagreeing (they did, for a long time — see the
generator's module docstring).

Consumers: ``rob_box_core.tool_catalog`` (typed access) and, through it,
``rob_box_harness.core.tool_registry`` — neither may import ROS2, which is
why this is checked-in data rather than an import-time reflection.
"""

from __future__ import annotations

from typing import Any

#: One entry per ``MCPTool`` subclass, sorted by name. ``signature`` mirrors
#: what ``execute()`` accepts so the catalog can be verified against the
#: code that runs it.
TOOL_CATALOG_DATA: tuple[dict[str, Any], ...] = '''


def render(entries: list[dict[str, Any]]) -> str:
    body = pprint.pformat(tuple(entries), indent=4, width=88, sort_dicts=False)
    return HEADER + body + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="exit 1 when the generated file is stale instead of rewriting it",
    )
    opts = parser.parse_args()

    rendered = render(extract_tools())

    if opts.check:
        current = OUT_FILE.read_text(encoding="utf-8") if OUT_FILE.exists() else ""
        if current != rendered:
            print(
                f"{OUT_FILE.relative_to(REPO_ROOT)} is stale — "
                "run `python tools/gen_tool_catalog.py`",
                file=sys.stderr,
            )
            return 1
        print("tool catalog is up to date")
        return 0

    OUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUT_FILE.write_text(rendered, encoding="utf-8")
    print(f"wrote {OUT_FILE.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
