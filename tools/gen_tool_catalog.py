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

#: ``{"NAME": <ast.List/ast.Tuple node>}`` for top-level ``NAME = [...]``
#: assignments in the file being parsed. Kept as raw AST (not literal-
#: evaluated) because the *elements* are ``MCPToolParameter(...)`` calls,
#: not literals — this is what lets ``ComposeMusicTool.parameters`` and
#: ``PreviewArrangementTool.parameters`` (ADR-0132 PR-5) both
#: ``return _ARRANGEMENT_PARAMETERS`` and have the generator read the same
#: list once instead of requiring two copies of the schema inline.
_MODULE_LIST_CONSTANTS: dict[str, ast.AST] = {}

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


def _collect_module_list_constants(tree: ast.Module) -> dict[str, ast.AST]:
    """Collect top-level ``NAME = [...]`` / ``NAME: T = [...]`` assignments.

    Unlike :func:`_collect_module_constants`, the value is kept as a raw
    ``ast.List``/``ast.Tuple`` node rather than literal-evaluated — a
    parameter list's elements are ``MCPToolParameter(...)`` calls, which
    ``ast.literal_eval`` cannot handle. See ``_ARRANGEMENT_PARAMETERS`` in
    ``tools/music.py`` (ADR-0132 PR-5).
    """
    out: dict[str, ast.AST] = {}
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
        if isinstance(value, (ast.List, ast.Tuple)):
            for target in targets:
                out[target.id] = value
    return out


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
        elif isinstance(node, ast.Assign) and any(getattr(t, "id", "") == "VIBE_PRESETS" for t in node.targets):
            presets = _literal(node.value)
    if not isinstance(presets, dict):
        raise ToolSourceError("could not read MusicManager.VIBE_PRESETS from music.py")
    presets_desc = ", ".join(f"{name} (scale={p['scale']}, bpm={p['bpm']})" for name, p in presets.items())
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

    catalog = json.loads((REPO_ROOT / "sound_pack" / "sound_catalog.json").read_text(encoding="utf-8"))
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


def _load_arranger():
    """Import ``rob_box_mcp_tools.core.arranger`` without importing the package.

    The arranger is deliberately ROS-free, so it can be loaded directly from
    its file — importing the package would drag in ``rclpy``, which the
    generator must run without.
    """
    import importlib.util

    path = REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "core" / "arranger.py"
    spec = importlib.util.spec_from_file_location("_arranger_for_catalog", path)
    if spec is None or spec.loader is None:
        raise ToolSourceError(f"cannot load arranger module from {path}")
    module = importlib.util.module_from_spec(spec)
    # Register before exec: ``dataclasses`` resolves field annotations through
    # ``sys.modules[cls.__module__]``, which is None for an unregistered
    # dynamically loaded module.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _composition_forms() -> list[str]:
    """``compose_music.form`` enum — the arranger's own form table.

    Read from the arranger rather than duplicated here, so adding a form
    shows up as a catalog diff instead of leaving the LLM with a stale list.
    """
    forms = sorted(_load_arranger().FORMS)
    if not forms:
        raise ToolSourceError("arranger.FORMS is empty")
    return forms


def _composition_roots() -> list[str]:
    """``compose_music.root`` enum — the arranger's accepted tonics."""
    roots = list(_load_arranger().VALID_ROOTS)
    if not roots:
        raise ToolSourceError("arranger.VALID_ROOTS is empty")
    return roots


def _composition_scales() -> list[str]:
    """``compose_music.scale`` enum — the arranger's scale table (ADR-0132 PR-2:
    an unknown scale is now an error, so the LLM must see the same list)."""
    scales = list(_load_arranger().SCALE_INTERVALS)
    if not scales:
        raise ToolSourceError("arranger.SCALE_INTERVALS is empty")
    return scales


def _groove_loops() -> list[str]:
    """``compose_music.groove_loop`` enum — the loop catalog data file (#2841).

    Mirrors ``sorted(sample_loops.loop_catalog())``: pack-0 and pack-1 names
    together (the pack-1 flag gates playback, not the schema).
    """
    import json

    path = REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "data" / "sample_loops.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    names = sorted(list(raw.get("pack0", {})) + list(raw.get("pack1", {})))
    if not names:
        raise ToolSourceError(f"{path} yielded no loops")
    return names


def _drum_styles() -> list[str]:
    """``compose_music.drum_style`` enum — ``harmonize.DRUM_STYLES`` (#2841).

    Read by AST, not import: ``harmonize`` imports the arranger relatively,
    so it cannot be loaded as a standalone file the way the arranger is.
    """
    import ast

    path = REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "core" / "harmonize.py"
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        target = node.target if isinstance(node, ast.AnnAssign) else (
            node.targets[0] if isinstance(node, ast.Assign) else None
        )
        if isinstance(target, ast.Name) and target.id == "DRUM_STYLES":
            return list(ast.literal_eval(node.value))
    raise ToolSourceError(f"DRUM_STYLES not found in {path}")


def _harmonize_constant(name: str) -> Any:
    """Top-level constant of ``core/harmonize.py``, read by AST (ADR-0132 PR-4).

    Like :func:`_drum_styles`, but the value may reference other literal
    top-level constants by name (``KNOB_VALUES`` is written with ``AUTO``),
    so those names are substituted before ``literal_eval``.
    """
    import ast

    path = REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "core" / "harmonize.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    known = _collect_module_constants(tree)
    for node in tree.body:
        target = node.target if isinstance(node, ast.AnnAssign) else (
            node.targets[0] if isinstance(node, ast.Assign) else None
        )
        if isinstance(target, ast.Name) and target.id == name and node.value is not None:
            value = _SubstituteNames(known).visit(node.value)
            return ast.literal_eval(ast.fix_missing_locations(value))
    raise ToolSourceError(f"{name} not found in {path}")


class _SubstituteNames(ast.NodeTransformer):
    """Replace ``Name`` nodes by the literal value of a known constant."""

    def __init__(self, known: dict[str, Any]) -> None:
        self._known = known

    def visit_Name(self, node: ast.Name) -> ast.AST:  # noqa: N802 — ast API
        if node.id in self._known:
            return ast.copy_location(ast.Constant(self._known[node.id]), node)
        return node


def _harmonize_knob(knob: str):
    """``compose_music.<knob>`` enum — ``harmonize.KNOB_VALUES[knob]`` (ADR-0132 PR-4)."""

    def resolve() -> list[str]:
        values = list(_harmonize_constant("KNOB_VALUES")[knob])
        if not values:
            raise ToolSourceError(f"harmonize.KNOB_VALUES[{knob!r}] is empty")
        return values

    return resolve


def _on_off_auto() -> list[str]:
    """``compose_music.counter``/``theme_octaves`` enum — ``arranger.ON_OFF_AUTO``."""
    return list(_load_arranger().ON_OFF_AUTO)


def _lead_octave_choices() -> list[str]:
    """``compose_music.lead_octave`` enum — mirrors ``compose_knobs.lead_octave_choices``.

    ``compose_knobs`` imports ``harmonize`` relatively and cannot be loaded as
    a file, so the two-line rule is repeated here; ``test_compose_music_knobs``
    pins the catalog enum to the tool's own one.
    """
    lo, hi = _harmonize_constant("LEAD_OCTAVE_RANGE")
    words = list(_harmonize_constant("LEAD_OCTAVE_WORDS"))
    return words + [f"{n:+d}" if n else "0" for n in range(lo, hi + 1)]


#: ``(tool_name, param_name)`` → resolver, for enums built from runtime data
#: rather than from a literal in the tool module.
DYNAMIC_ENUMS = {
    ("play_sound", "sound"): _sound_pack_triggers,
    ("compose_music", "form"): _composition_forms,
    ("compose_music", "root"): _composition_roots,
    ("compose_music", "scale"): _composition_scales,
    ("compose_music", "groove_loop"): _groove_loops,
    ("compose_music", "drum_style"): _drum_styles,
    # ADR-0132 PR-4: ручки аранжировщика — значения из ядра, не копия.
    ("compose_music", "key_detection"): _harmonize_knob("key_detection"),
    ("compose_music", "harmonic_rhythm"): _harmonize_knob("harmonic_rhythm"),
    ("compose_music", "density"): _harmonize_knob("density"),
    ("compose_music", "bass_style"): _harmonize_knob("bass_style"),
    ("compose_music", "bass_approach"): _harmonize_knob("bass_approach"),
    ("compose_music", "pad_style"): _harmonize_knob("pad_style"),
    ("compose_music", "lead_outliers"): _harmonize_knob("lead_outliers"),
    ("compose_music", "counter"): _on_off_auto,
    ("compose_music", "theme_octaves"): _on_off_auto,
    ("compose_music", "lead_octave"): _lead_octave_choices,
}
# ADR-0132 PR-5: preview_arrangement shares ``_ARRANGEMENT_PARAMETERS`` with
# compose_music (see ``tools/music.py``) — same computed enums, same
# resolvers, so the two tools can never drift apart on what values a knob
# accepts.
DYNAMIC_ENUMS.update(
    {("preview_arrangement", param): resolver for (tool, param), resolver in list(DYNAMIC_ENUMS.items()) if tool == "compose_music"}
)


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
        schema["properties"] = {name: _json_schema(sub) for name, sub in param["properties"].items()}
        schema["required"] = [name for name, sub in param["properties"].items() if sub.get("required")]
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


#: Доменные скиллы: имя скилла -> имена инструментов из этого же каталога.
#:
#: ЕДИНСТВЕННОЕ объявление принадлежности. Скилл ссылается на инструменты
#: по имени и НЕ переобъявляет их контракт (описание, схему параметров) —
#: именно второе объявление контракта убило предыдущую попытку скиллов
#: (Compositor, e96b912d: navigate_to_waypoint рекламировал ``name`` при
#: ``waypoint`` в execute(), 13 инструментов не доехали до LLM вообще).
#:
#: Инструмент может входить в несколько скиллов: ``stop_music`` нужен и
#: композитору, и диджею, и плееру. Описание при этом одно — оно берётся
#: из каталога, поэтому разойтись не может по построению.
#:
#: ``core`` предъявляется ВСЕГДА, независимо от активного скилла.
SKILL_TOOLS: dict[str, tuple[str, ...]] = {
    "core": (
        "speak_text",
        "get_robot_status",
        "get_battery_level",
        "get_current_time",
        "get_perception_context",
        "listen_for_response",
    ),
    "composer": (
        "compose_music",
        "preview_arrangement",
        "execute_music_code",
        "set_vibe_preset",
        "search_samples",
        "lookup_melody",
        "search_melody",
        "search_web",
        "get_music_state",
        "stop_music",
    ),
    "dj": (
        "set_dj_mode",
        "get_music_state",
        "stop_music",
    ),
    "player": (
        "gen_list_library",
        "gen_play_from_library",
        "gen_search_library",
        "gen_get_track_info",
        "gen_save_to_library",
        "gen_delete_from_library",
        "stop_music",
    ),
    "renardo-library": (
        "save_track",
        "list_tracks",
        "load_track",
        "delete_track",
    ),
    "navigation": (
        "navigate_to_waypoint",
        "navigate_to_coordinates",
        "move_direction",
        "stop_navigation",
        "list_waypoints",
        "save_waypoint",
        "delete_waypoint",
        "clear_waypoints",
        "get_current_pose",
    ),
    "mapping": (
        "start_mapping",
        "continue_mapping",
        "finish_mapping",
        "optimize_map",
        "load_map",
    ),
    "voice-tts": (
        "set_voice",
        "set_volume",
        "set_pitch",
        "set_speed",
        "list_tts_voices",
        "set_tts_provider",
        "estimate_tts_duration",
    ),
    "memory": (
        "memory_save",
        "memory_search",
        "memory_context",
        "register_speaker",
    ),
    "expression": (
        "play_animation",
        "play_sound",
        "get_sound_info",
    ),
    "knowledge": (
        "search_web",
        "faq_search",
    ),
    "scheduler": ("task_delta",),
}


def _assign_skills(entries: list[dict[str, Any]]) -> None:
    """Проставить ``skill`` каждой записи и проверить обе стороны связи.

    Падаем на:

    * скилл ссылается на инструмент, которого нет в каталоге (опечатка,
      переименование, удалённый инструмент) — иначе LLM получит скилл,
      обещающий несуществующую функцию;
    * llm_visible инструмент не попал ни в один скилл — при включённом
      сужении каталога (Move B) такой инструмент стал бы невидимым молча.

    Инструменты, скрытые от LLM (``llm_visible=False``), от второй
    проверки освобождены: их всё равно никто не предъявляет.
    """
    by_name = {entry["name"]: entry for entry in entries}

    unknown: list[str] = []
    for skill, tool_names in sorted(SKILL_TOOLS.items()):
        for tool_name in tool_names:
            if tool_name not in by_name:
                unknown.append(f"{skill} -> {tool_name}")
    if unknown:
        raise SystemExit(
            "SKILL_TOOLS ссылается на инструменты, которых нет в каталоге:"
            + "\n  "
            + "\n  ".join(unknown)
            + "\nПроверь имя в tools/gen_tool_catalog.py::SKILL_TOOLS."
        )

    for entry in entries:
        entry["skill"] = tuple(
            skill for skill, tool_names in sorted(SKILL_TOOLS.items()) if entry["name"] in tool_names
        )

    orphans = sorted(entry["name"] for entry in entries if entry.get("llm_visible", True) and not entry["skill"])
    if orphans:
        raise SystemExit(
            f"{len(orphans)} llm_visible инструмент(ов) не отнесены "
            + "ни к одному скиллу:\n  "
            + "\n  ".join(orphans)
            + "\nДобавь их в tools/gen_tool_catalog.py::SKILL_TOOLS."
        )


def extract_tools() -> list[dict[str, Any]]:
    """Read every ``MCPTool`` subclass under ``tools/`` into catalog entries."""
    global _MODULE_CONSTANTS, _MODULE_LIST_CONSTANTS, _CLASS_CONSTANTS, _CURRENT_CLASS, _PARAM_FACTORIES
    entries: list[dict[str, Any]] = []
    shared_constants = _collect_shared_constants()

    for source in sorted(TOOLS_DIR.glob("*.py")):
        if source.name == "__init__.py":
            continue
        tree = ast.parse(source.read_text(encoding="utf-8"))
        _MODULE_CONSTANTS = {**shared_constants, **_collect_module_constants(tree)}
        _MODULE_LIST_CONSTANTS = _collect_module_list_constants(tree)
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
                "starts_music": False,
                "satisfies_user_music": False,
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
                elif fn.name in (
                    "read_only",
                    "destructive",
                    "idempotent",
                    "llm_visible",
                    "starts_music",
                    "satisfies_user_music",
                ):
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
    # Назначение скиллов — часть извлечения, а не отдельный шаг в main():
    # иначе его можно забыть, и ``test_tool_catalog_is_current`` начнёт
    # сравнивать каталог со скиллами против каталога без них.
    _assign_skills(entries)
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
    # ``return _ARRANGEMENT_PARAMETERS`` — a bare name referring to a
    # module-level ``NAME = [MCPToolParameter(...), ...]`` (ADR-0132 PR-5:
    # compose_music/preview_arrangement share one schema list by identity,
    # not by copy-pasted source).
    if isinstance(returned, ast.Name):
        resolved = _MODULE_LIST_CONSTANTS.get(returned.id)
        if resolved is not None:
            returned = resolved
    if not isinstance(returned, (ast.List, ast.Tuple)):
        raise ToolSourceError(f"{cls_name} ({filename}): `parameters` must return a list literal")

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
        n for n in ast.walk(factory) if isinstance(n, ast.Call) and getattr(n.func, "id", None) == "MCPToolParameter"
    )
    param = _param_from_call(inner)

    # Factory defaults, then call-site overrides, for any field the factory
    # forwards from its own signature (`required=required`).
    forwarded = {kw.arg: kw.value.id for kw in inner.keywords if kw.arg and isinstance(kw.value, ast.Name)}
    arg_names = [a.arg for a in factory.args.args]
    defaults = dict(zip(arg_names[len(arg_names) - len(factory.args.defaults) :], factory.args.defaults))
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
                f"{OUT_FILE.relative_to(REPO_ROOT)} is stale — " "run `python tools/gen_tool_catalog.py`",
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
