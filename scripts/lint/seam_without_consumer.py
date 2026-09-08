#!/usr/bin/env python3
"""Seam-without-consumer guard (ADR-0021 policy, issue #2118).

За сутки 2026-09-07/08 один и тот же дефект прошёл мимо зелёных тестов три
раза подряд (issues #1992, #2113, #2116): один конец шва написан и
покрыт тестами, другой конец — нет, и ничего в CI об этом не сигналит.
Docs: docs/plans/2026-09-05-operator-agent-architecture-handoff.md §4.1.

Два узких инварианта (НЕ общий детектор мёртвого кода):

1. ROS-топик без потребителя. Собираем по AST литералы топиков во всех
   ``create_publisher(...)``/``create_subscription(...)`` в продовом коде
   под ``src/``. Топик с паблишером и без единого подписчика в репозитории
   (и наоборот) — потенциальный шов без потребителя.

2. Метод-шов без вызывающего. Приватный ``_publish_*``/``_on_*`` метод, на
   который нет ни одной ссылки (вызов или передача как callback) в
   непроверочном коде, но есть ссылки в тестах — сильный признак
   неподключённого шва (ровно сигнатура issue #2116).

Конвенция подключения — как у ``scripts/lint/cc_budget.py`` (ADR-0021):

* ``seam_baseline.json`` грандфазерит текущие находки (обе категории) —
  скрипт не роняет CI на первом прогоне. Новый шов (не в baseline, не в
  allow-list) — FAIL.
* ``seam_allowlist.json`` — ручной, куррируемый список легальных внешних
  потребителей/источников (WS-мост к шлему, Telegram, дашборд), каждая
  запись обязана нести причину. Пустая причина — ошибка загрузки
  (fail-loud, ADR-0018), а не молчаливый пропуск.

Ограничения (см. секцию "не удалось разрешить" в выводе — печатается
всегда, никогда не гейтит CI):

* Топик, построенный не литералом (f-строка с интерполяцией, вызов
  функции, ``get_parameter().value``) — резолвится best-effort через
  константы уровня модуля/класса; если не вышло — попадает в
  «не удалось разрешить», а не тихо считается отсутствующим.
* Метод, вызываемый через ``getattr(self, "...")`` по динамическому имени,
  инвизибл для этого сканера (см. docstring ниже про constraints).

Usage:
  python scripts/lint/seam_without_consumer.py                   # check (default: src/)
  python scripts/lint/seam_without_consumer.py <path> [<path>...]
  python scripts/lint/seam_without_consumer.py --update-baseline
"""

from __future__ import annotations

import argparse
import ast
import json
import subprocess
import sys
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BASELINE_FILE = REPO_ROOT / "scripts" / "lint" / "seam_baseline.json"
ALLOWLIST_FILE = REPO_ROOT / "scripts" / "lint" / "seam_allowlist.json"

DEFAULT_TARGETS: tuple[Path, ...] = (REPO_ROOT / "src",)

_SKIP_DIR_NAMES = {
    "__pycache__",
    ".git",
    "build",
    "install",
    "log",
    "node_modules",
    ".venv",
    "venv",
}

_TOPIC_CALL_NAMES = {"create_publisher": "pub", "create_subscription": "sub"}
_SEAM_PREFIXES = ("_publish_", "_on_")

# Msg-type resolution: как и для топиков, лучшее, что мы можем сделать
# статически — нормализовать выражение к строковому идентификатору типа
# (``"String"``, ``"TeleopHeartbeat"``). Если оба конца шва дают одинаковый
# идентификатор — типы совпадают; если разные — ``topic_type_mismatch``
# (issue #2188 / voice-vr 03). ROS 2 в рантайме такое соединение не
# поднимет: ``create_publisher(String, ...)`` и
# ``create_subscription(TeleopHeartbeat, ...)`` — два разных IDL-класса.
#
# Граница применимости: «не резолвится» (f-строка, вызов функции,
# неразрешимый атрибут) попадает в ``unresolved_msg_types`` — НЕ в FAIL.
# Если хоть один конец шва неизвестен, мы не имеем права утверждать ни
# «совпадают», ни «не совпадают», и молчаливо считать «совпадают» —
# ровно тот класс silent-fail, от которого этот сторож и появился
# (ADR-0021 §R3). Поэтому новый FAIL выдаётся ТОЛЬКО когда ОБА конца
# шва резолвятся в конкретные разные идентификаторы.


# ---------------------------------------------------------------------------
# File classification / discovery
# ---------------------------------------------------------------------------


def _is_test_path(path: Path) -> bool:
    """True if ``path`` is test code by this repo's convention.

    Convention observed across the tree: package tests live under a
    ``test/`` or ``tests/`` directory (``src/*/test/unit/...``), and/or the
    file itself is named ``test_*.py`` / ``*_test.py``.
    """
    if any(part in ("test", "tests") for part in path.parts):
        return True
    name = path.name
    return name.startswith("test_") or name.endswith("_test.py")


def _expand_targets(targets: list[Path]) -> list[Path]:
    """Resolve files/dirs to a flat, sorted list of .py files (prod + test)."""
    resolved: list[Path] = []
    for target in targets:
        if target.is_file():
            resolved.append(target)
            continue
        if not target.is_dir():
            print(f"seam_without_consumer: no such path: {target}")
            sys.exit(2)
        for path in sorted(target.rglob("*.py")):
            if any(part in _SKIP_DIR_NAMES for part in path.parts):
                continue
            resolved.append(path)
    return resolved


def _rel(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def _git_head() -> str:
    try:
        head = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
        return head.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _parse(path: Path) -> ast.Module:
    # utf-8-sig: tolerate a BOM (files authored on Windows), same as cc_budget.py.
    return ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))


# ---------------------------------------------------------------------------
# Best-effort literal resolution (module constants / class constants / f-strings)
# ---------------------------------------------------------------------------


def _literal_str(node: ast.AST) -> str | None:
    """A plain string literal, or a fully-literal f-string (no interpolation)."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.JoinedStr):
        parts: list[str] = []
        for value in node.values:
            if isinstance(value, ast.Constant) and isinstance(value.value, str):
                parts.append(value.value)
            else:
                return None
        return "".join(parts)
    return None


def _module_consts(tree: ast.Module) -> dict[str, str | None]:
    """``NAME = "literal"`` (or ``NAME: str = "literal"``) at module top level."""
    consts: dict[str, str | None] = {}
    for node in tree.body:
        targets: list[ast.expr] = []
        value: ast.expr | None = None
        if isinstance(node, ast.Assign):
            targets = node.targets
            value = node.value
        elif isinstance(node, ast.AnnAssign) and node.value is not None:
            targets = [node.target]
            value = node.value
        else:
            continue
        lit = _literal_str(value) if value is not None else None
        for target in targets:
            if isinstance(target, ast.Name):
                if target.id in consts and consts[target.id] != lit:
                    consts[target.id] = None  # ambiguous: reassigned differently
                else:
                    consts[target.id] = lit
    return consts


def _unwrap_str_call(node: ast.expr) -> ast.expr:
    """``str(x)`` -> ``x``; anything else is returned unchanged."""
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "str" and len(node.args) == 1:
        return node.args[0]
    return node


def _declared_params(class_node: ast.ClassDef) -> dict[str, str | None]:
    """``self.declare_parameter("name", <literal-default>)`` across the class.

    rclpy's declare-then-get-parameter idiom is the other extremely common
    way this repo names a topic (``self.declare_parameter("avatar_request_topic",
    "/avatar/tts/request")`` ... later ``self.get_parameter("avatar_request_topic").value``).
    Without tracking it, every topic wired this way — which is most of
    ``quest_node.py`` and ``tts_node.py`` — would incorrectly show up as
    "could not resolve" (or worse, as a false pub/sub mismatch).
    """
    declared: dict[str, str | None] = {}
    for node in class_node.body:
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for sub in ast.walk(node):
            if not isinstance(sub, ast.Call):
                continue
            func = sub.func
            if not (
                isinstance(func, ast.Attribute)
                and func.attr == "declare_parameter"
                and isinstance(func.value, ast.Name)
                and func.value.id == "self"
            ):
                continue
            if not sub.args:
                continue
            name_arg = sub.args[0]
            if not (isinstance(name_arg, ast.Constant) and isinstance(name_arg.value, str)):
                continue
            default = _literal_str(sub.args[1]) if len(sub.args) >= 2 else None
            name = name_arg.value
            if name in declared and declared[name] != default:
                declared[name] = None
            elif name not in declared:
                declared[name] = default

    # Второй идиом объявления параметров: список кортежей в
    # ``super().__init__(..., parameters=[("input_topic", "led_matrix/data")])``
    # (и то же самое в ``declare_parameters(namespace=..., parameters=[...])``).
    #
    # Без него сторож даёт ЛОЖНОЕ срабатывание там, где две стороны шва
    # объявляют топик по-разному. Реальный случай: led_matrix_compositor
    # пишет паблишер литералом ``self.output_topic = "led_matrix/data"`` —
    # резолвится; led_matrix_driver объявляет подписку этим идиомом — не
    # резолвился, и паблишер выглядел как шов без потребителя, хотя
    # потребитель есть (led_matrix_driver.py:27).
    for sub in ast.walk(class_node):
        if not isinstance(sub, ast.Call):
            continue
        for kw in sub.keywords:
            if kw.arg != "parameters" or not isinstance(kw.value, (ast.List, ast.Tuple)):
                continue
            for elt in kw.value.elts:
                if not isinstance(elt, (ast.Tuple, ast.List)) or len(elt.elts) < 2:
                    continue
                name_node, default_node = elt.elts[0], elt.elts[1]
                if not (
                    isinstance(name_node, ast.Constant)
                    and isinstance(name_node.value, str)
                ):
                    continue
                default = _literal_str(default_node)
                name = name_node.value
                if name in declared and declared[name] != default:
                    declared[name] = None
                elif name not in declared:
                    declared[name] = default
    return declared


def _get_parameter_value(node: ast.expr, declared_params: dict[str, str | None]) -> str | None:
    """Match ``self.get_parameter("name").value`` (optionally ``str(...)``-wrapped)."""
    node = _unwrap_str_call(node)
    if not (isinstance(node, ast.Attribute) and node.attr == "value"):
        return None
    call = node.value
    if not (
        isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "get_parameter"
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "self"
        and call.args
    ):
        return None
    name_arg = call.args[0]
    if isinstance(name_arg, ast.Constant) and isinstance(name_arg.value, str):
        return declared_params.get(name_arg.value)
    return None


def _class_consts(
    class_node: ast.ClassDef, module_consts: dict[str, str | None] | None = None
) -> dict[str, str | None]:
    """Class-body literals + unambiguous ``self.attr = <literal-ish>`` in any method.

    "literal-ish" covers a plain string/f-string literal, the
    declare_parameter/get_parameter idiom above, and (for class-body assigns
    only) a bare reference to a same-named module-level constant — this repo
    has a re-export idiom (``TOPIC_STATE = "/avatar/state"`` at module scope,
    then ``TOPIC_STATE = TOPIC_STATE`` inside the class body so it's reachable
    as ``self.TOPIC_STATE``). All of these resolve to the same flat
    ``attr -> str | None`` map so callers don't need to know which style
    produced the value.
    """
    consts: dict[str, str | None] = {}
    declared_params = _declared_params(class_node)
    module_consts = module_consts or {}

    def _record(name: str, lit: str | None) -> None:
        if name in consts and consts[name] != lit:
            consts[name] = None
        elif name not in consts:
            consts[name] = lit

    for node in class_node.body:
        if isinstance(node, ast.Assign):
            lit = _literal_str(node.value)
            if lit is None and isinstance(node.value, ast.Name):
                lit = module_consts.get(node.value.id)
            for target in node.targets:
                if isinstance(target, ast.Name):
                    _record(target.id, lit)
        elif isinstance(node, ast.AnnAssign) and node.value is not None:
            if isinstance(node.target, ast.Name):
                lit = _literal_str(node.value)
                if lit is None and isinstance(node.value, ast.Name):
                    lit = module_consts.get(node.value.id)
                _record(node.target.id, lit)

    for node in class_node.body:
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for sub in ast.walk(node):
            if isinstance(sub, ast.Assign):
                targets = sub.targets
                value = sub.value
            elif isinstance(sub, ast.AnnAssign) and sub.value is not None:
                targets = [sub.target]
                value = sub.value
            else:
                continue
            lit = _literal_str(value)
            if lit is None:
                lit = _get_parameter_value(value, declared_params)
            for target in targets:
                if (
                    isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "self"
                ):
                    _record(target.attr, lit)
    return consts


def _module_dotted_name(path: Path) -> str | None:
    """Best-effort importable dotted name for an ament_python package module.

    ``src/<pkg>/<pkg>/tools/dialogue.py`` -> ``pkg.tools.dialogue`` (the
    ``<pkg>/<pkg>`` duplication is the standard ROS2 ament_python layout:
    the outer dir is the ROS package, the inner one is the Python module
    root). Anything that doesn't match this shape (bare scripts under
    ``scripts/``, non-doubled dirs) returns ``None`` — such files simply
    aren't resolvable as import targets by this heuristic, and any
    ``from X import Y`` pointing at them stays in the unresolved bucket
    rather than guessing wrong.
    """
    rel = _rel(path)
    parts = Path(rel).parts
    if len(parts) < 3 or parts[0] != "src" or parts[1] != parts[2]:
        return None
    module_parts = list(parts[2:])
    if module_parts[-1] == "__init__.py":
        module_parts = module_parts[:-1]
    elif module_parts[-1].endswith(".py"):
        module_parts[-1] = module_parts[-1][: -len(".py")]
    else:
        return None
    if not module_parts:
        return None
    return ".".join(module_parts)


def _collect_imports(tree: ast.Module) -> dict[str, tuple[str, str]]:
    """Top-level + lazy ``from module import name [as alias]`` -> alias -> (module, name).

    Также собираем bare ``import std_msgs.msg`` (то есть без ``from``) —
    имя ``std_msgs.msg`` становится доступно как ``std_msgs``-корень в
    :func:`_msg_type_id` для dotted-msg-типов вроде
    ``std_msgs.msg.String``. Без этого пункта полные dotted-имена
    типов не резолвились бы (issue #2188).

    Резолвим lazy-импорты внутри функций (``from std_msgs.msg import
    String`` глубоко в :py:meth:`run` — частый паттерн в rob_box_mcp_tools),
    потому что они несут тот же идентификатор типа, что и top-level
    импорты, и без них msg-type резолвер показывал бы десятки
    нерезолвов на ``String`` в MCP-тулзах (issue #2188).

    Only absolute (``level == 0``) imports are tracked — relative imports
    are rare enough in this repo that resolving them isn't worth the
    complexity; both fall through to "could not resolve" rather than
    silently guessing.
    """
    imports: dict[str, tuple[str, str]] = {}

    def _record_from(node: ast.ImportFrom) -> None:
        if not (node.module and node.level == 0):
            return
        for alias in node.names:
            local = alias.asname or alias.name
            imports[local] = (node.module, alias.name)

    def _record_import(node: ast.Import) -> None:
        for alias in node.names:
            local = alias.asname or alias.name
            # ``import std_msgs.msg`` binds ``std_msgs.msg`` к ``std_msgs``
            # (Python: ``import a.b`` → ``a`` is the bound name). Берём
            # только корневой компонент — он нам нужен для матча
            # ``std_msgs.msg.String`` в :func:`_msg_type_id`.
            root = alias.name.split(".", 1)[0]
            imports[local] = (root, alias.name)

    for node in tree.body:
        if isinstance(node, ast.ImportFrom):
            _record_from(node)
        elif isinstance(node, ast.Import):
            _record_import(node)
    # Lazy imports inside functions. We rely on the last assignment wins
    # semantics for the same alias (matches Python's runtime behaviour:
    # whichever import runs last binds the name).
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            _record_from(node)
        elif isinstance(node, ast.Import):
            _record_import(node)
    return imports


def _param_defaults(func_node: ast.FunctionDef | ast.AsyncFunctionDef) -> dict[str, str | None]:
    """Literal default values of a function's parameters.

    ROS wiring code in this repo commonly takes the topic as a keyword
    parameter with the real topic string as its default (``def __init__(self,
    node, *, panel_url_topic: str = "/avatar/tars/panel_url"): ...
    node.create_publisher(String, panel_url_topic, 10)``), so a bare
    parameter reference resolves through its default when the body itself
    doesn't reassign it. This assumes callers don't override the default —
    true for every call site in this repo today; if that ever changes for a
    given topic, the resolver will start reporting it as unresolved (a
    conservative, not silently-wrong, failure mode) once the parameter is
    reassigned locally, since local reassignment takes precedence below.
    """
    args = func_node.args
    result: dict[str, str | None] = {}
    positional = list(args.posonlyargs) + list(args.args)
    defaults = list(args.defaults)
    offset = len(positional) - len(defaults)
    for i, default in enumerate(defaults):
        result[positional[offset + i].arg] = _literal_str(default)
    for kwarg, default in zip(args.kwonlyargs, args.kw_defaults):
        if default is not None:
            result[kwarg.arg] = _literal_str(default)
    return result


def _local_consts(
    func_node: ast.AST, declared_params: dict[str, str | None]
) -> dict[str, str | None]:
    """``name = "literal"`` local assigns + literal param defaults in a function.

    Also chases the declare_parameter/get_parameter idiom for a plain local
    variable (``battery_topic = str(self.get_parameter("battery_json_topic").value)``)
    the same way ``_class_consts`` does for ``self.attr`` — this is the
    quest_node.py shape for the /device/snapshot topic.
    """
    consts: dict[str, str | None] = {}
    if isinstance(func_node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        consts.update(_param_defaults(func_node))
    for node in ast.walk(func_node):
        if not isinstance(node, ast.Assign):
            continue
        lit = _literal_str(node.value)
        if lit is None:
            lit = _get_parameter_value(node.value, declared_params)
        for target in node.targets:
            if isinstance(target, ast.Name):
                if target.id in consts and consts[target.id] != lit:
                    consts[target.id] = None
                elif target.id not in consts:
                    consts[target.id] = lit
    return consts


def _resolve(
    node: ast.AST,
    module_consts: dict[str, str | None],
    class_consts: dict[str, str | None] | None,
    local_consts: dict[str, str | None],
    import_map: dict[str, tuple[str, str]] | None = None,
    all_module_consts: dict[str, dict[str, str | None]] | None = None,
) -> str | None:
    """Best-effort literal resolution of a topic expression.

    Resolution order for a bare ``Name``: local var in the enclosing
    function, then module-level constant in the same file, then (if the
    name was imported via ``from <module> import <name>``) the constant in
    that other module — this is what lets a topic constant defined once in
    a shared module (e.g. ``rob_box_core.avatar_command.AVATAR_COMMAND_TOPIC``)
    resolve identically at every publisher/subscriber call site.
    """
    lit = _literal_str(node)
    if lit is not None:
        return lit
    if isinstance(node, ast.JoinedStr):
        parts: list[str] = []
        for value in node.values:
            if isinstance(value, ast.Constant) and isinstance(value.value, str):
                parts.append(value.value)
            elif isinstance(value, ast.FormattedValue):
                inner = _resolve(value.value, module_consts, class_consts, local_consts, import_map, all_module_consts)
                if inner is None:
                    return None
                parts.append(inner)
            else:
                return None
        return "".join(parts)
    if isinstance(node, ast.Name):
        if node.id in local_consts:
            return local_consts[node.id]
        if node.id in module_consts:
            return module_consts[node.id]
        if import_map and all_module_consts and node.id in import_map:
            mod, orig = import_map[node.id]
            return all_module_consts.get(mod, {}).get(orig)
        return None
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
        if class_consts is not None:
            return class_consts.get(node.attr)
        return None
    return None


def _src(node: ast.AST) -> str:
    try:
        return ast.unparse(node)
    except Exception:  # pragma: no cover - defensive, unparse is stable on 3.9+
        return "<unparseable>"


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass
class TopicHit:
    topic: str
    file: str
    line: int
    kind: str  # "pub" | "sub"
    # ``msg_type_id`` — нормализованный строковый идентификатор типа
    # сообщения (``"String"``, ``"std_msgs.msg.String"``,
    # ``"rob_box_supervisor_msgs.msg.TeleopHeartbeat"``), если получилось
    # резолвнуть AST-узел статически; ``None`` — если нет (тогда
    # сравнение типов пропускается, попадает в
    # ``scan.unresolved_msg_types``). Голое короткое имя
    # (``"String"``, ``"TeleopHeartbeat"``) — нормальная форма для
    # сравнения: импорт алиаса не должен скрывать факт расхождения.
    msg_type_id: str | None = None


@dataclass
class UnresolvedTopic:
    file: str
    line: int
    kind: str
    expr_src: str


@dataclass
class UnresolvedMsgType:
    """``create_publisher(Subscribed, ...)``/``create_subscription(...)``
    call site, где 1-й позиционный аргумент (msg-type) не удалось
    нормализовать к идентификатору типа — f-строка, вызов функции,
    нерезолвимый атрибут. Никогда не гейтит CI; печатается в секции
    «не удалось разрешить» для последующей диагностики вручную.
    """

    file: str
    line: int
    kind: str
    expr_src: str


@dataclass
class SeamDef:
    file: str
    qualname: str  # "ClassName.method_name"
    method_name: str
    line: int


@dataclass
class SeamScan:
    pubs: dict[str, list[TopicHit]] = field(default_factory=dict)
    subs: dict[str, list[TopicHit]] = field(default_factory=dict)
    unresolved: list[UnresolvedTopic] = field(default_factory=list)
    # ``unresolved_msg_types`` отдельно от ``unresolved`` (топиков),
    # потому что msg-type может быть нерезолвим даже при известном
    # имени топика (и наоборот) — это два независимых канала отчёта.
    unresolved_msg_types: list[UnresolvedMsgType] = field(default_factory=list)
    seam_defs: list[SeamDef] = field(default_factory=list)
    # method_name -> {"prod": count, "test": count}
    seam_usage: dict[str, dict[str, int]] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# AST walking
# ---------------------------------------------------------------------------


def _topic_call_kind(call: ast.Call) -> str | None:
    func = call.func
    if isinstance(func, ast.Attribute) and func.attr in _TOPIC_CALL_NAMES:
        return _TOPIC_CALL_NAMES[func.attr]
    return None


def _topic_arg(call: ast.Call) -> ast.expr | None:
    for kw in call.keywords:
        if kw.arg == "topic":
            return kw.value
    if len(call.args) >= 2:
        return call.args[1]
    if len(call.args) == 1:
        return call.args[0]
    return None


def _msg_type_arg(call: ast.Call) -> ast.expr | None:
    """1-й позиционный аргумент ``create_publisher/submission(...)``
    (msg-type: ``String``, ``TeleopHeartbeat``, ``std_msgs.msg.String``).

    Если когда-нибудь ROS2 примет keyword-аргумент для типа
    (``msg_type=...``) — добавим сюда; пока все call-sites в репо
    передают его первым позиционным аргументом.
    """
    return call.args[0] if call.args else None


def _msg_type_id(
    node: ast.expr,
    module_consts: dict[str, str | None],
    class_consts: dict[str, str | None] | None,
    local_consts: dict[str, str | None],
    import_map: dict[str, tuple[str, str]] | None,
    all_module_consts: dict[str, dict[str, str | None]] | None,
) -> str | None:
    """Нормализовать выражение msg-типа к каноническому строковому
    идентификатору для сравнения между концами шва.

    Возвращает:
      * ``"String"`` — для ``create_publisher(String, ...)`` (импорт
        алиаса ``from std_msgs.msg import String as RosString``
        резолвится через ``import_map`` и сворачивается к тому же
        короткому имени — иначе alias мог бы маскировать расхождение);
      * ``"std_msgs.msg.String"`` — для атрибутной формы
        ``std_msgs.msg.String``;
      * ``"rob_box_supervisor_msgs.msg.TeleopHeartbeat"`` — то же
        для длинного IDL-имени;
      * ``None`` — для f-строк, вызовов функций, нерезолвимых
        атрибутов; сравнение типов в этом случае пропускается,
        запись попадает в ``unresolved_msg_types``.

    Важно: ``None`` НЕ означает «совпадают» — это явный отказ от
    ответа. Если хоть один конец шва ``None``, mismatch не
    выдаётся (иначе мы бы получили silent-fail ровно того класса,
    который этот сторож ловит — issue #2188 / ADR-0021 §R3).
    """
    # ``str(SomeName)``-обёртка — некоторые ноды пишут
    # ``create_subscription(str(self._heartbeat_msg_type), ...)``
    # на всякий случай. Идемпотентно, как у топиков.
    node = _unwrap_str_call(node)
    if isinstance(node, ast.Name):
        local = local_consts.get(node.id)
        if isinstance(local, str) and local:
            # Локальная переменная, инициализированная строкой —
            # редкая форма (msg-type как строка?), трактуем как opaque
            # идентификатор. На практике не встречается, но и не
            # мешает — резолв как opaque.
            return local
        if class_consts is not None:
            cls = class_consts.get(node.id)
            if isinstance(cls, str) and cls:
                return cls
        if node.id in module_consts:
            mc = module_consts[node.id]
            if isinstance(mc, str) and mc:
                return mc
        # import: ``from std_msgs.msg import String`` / ``... as RosString``
        if import_map and all_module_consts and node.id in import_map:
            mod, orig = import_map[node.id]
            # Короткий канонический ID = оригинальное имя импорта
            # (alias тут НЕ нормализуем: ``from X import Y as Z`` —
            # ``Y`` это имя типа; ``Z`` — локальный алиас. Если оба
            # конца используют один и тот же тип под разными алиасами,
            # orig-имя совпадёт; если типы разные — orig-имена
            # разойдутся).
            return orig
        return None
    if isinstance(node, ast.Attribute):
        # Полное dotted имя типа: ``pkg.msg.String``.
        # Только резолвим, если ``node.value`` — это ``Name`` с
        # известным imported модулем (например ``std_msgs.msg.String``
        # где ``std_msgs`` — это ``import std_msgs.msg``). Это НЕ
        # покрывает ``self.some_attr`` (метод-возвращаемый IDL-тип) —
        # тот резолвится вручную через ``_try_import_*`` в проде и
        # намеренно остаётся нерезолвимым здесь: «не знаю» лучше,
        # чем «совпадают», иначе мы рискуем проглядеть тот самый
        # silent-fail, ради которого этот сторож и существует.
        if isinstance(node.value, ast.Name) and node.value.id in (import_map or {}):
            return node.attr
        return None
    return None


def _walk_topics(
    node: ast.AST,
    rel: str,
    module_consts: dict[str, str | None],
    class_consts: dict[str, str | None] | None,
    local_consts: dict[str, str | None],
    import_map: dict[str, tuple[str, str]],
    all_module_consts: dict[str, dict[str, str | None]],
    declared_params: dict[str, str | None],
    scan: SeamScan,
) -> None:
    """Recursive walk collecting create_publisher/create_subscription topics.

    Tracks class scope (for ``self.ATTR`` resolution) and, per function,
    local-variable literal assigns (for ``topic = "..."; create_publisher(...,
    topic, ...)`` style). Test files are excluded by the caller (topics are a
    property of the real ROS graph; test fakes don't wire anything real).
    """
    for child in ast.iter_child_nodes(node):
        if isinstance(child, ast.ClassDef):
            new_class_consts = _class_consts(child, module_consts)
            new_declared_params = _declared_params(child)
            _walk_topics(
                child, rel, module_consts, new_class_consts, local_consts,
                import_map, all_module_consts, new_declared_params, scan,
            )
            continue
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
            new_local_consts = {**local_consts, **_local_consts(child, declared_params)}
            _walk_topics(
                child, rel, module_consts, class_consts, new_local_consts,
                import_map, all_module_consts, declared_params, scan,
            )
            continue
        if isinstance(child, ast.Call):
            kind = _topic_call_kind(child)
            if kind is not None:
                arg = _topic_arg(child)
                msg_type_expr = _msg_type_arg(child)
                msg_type_id: str | None = None
                if msg_type_expr is not None:
                    msg_type_id = _msg_type_id(
                        msg_type_expr,
                        module_consts,
                        class_consts,
                        local_consts,
                        import_map,
                        all_module_consts,
                    )
                    if msg_type_id is None:
                        scan.unresolved_msg_types.append(
                            UnresolvedMsgType(
                                rel, child.lineno, kind, _src(msg_type_expr)
                            )
                        )
                if arg is None:
                    scan.unresolved.append(
                        UnresolvedTopic(rel, child.lineno, kind, _src(child))
                    )
                else:
                    resolved = _resolve(arg, module_consts, class_consts, local_consts, import_map, all_module_consts)
                    if resolved is None:
                        scan.unresolved.append(
                            UnresolvedTopic(rel, child.lineno, kind, _src(arg))
                        )
                    else:
                        hit = TopicHit(
                            resolved,
                            rel,
                            child.lineno,
                            kind,
                            msg_type_id=msg_type_id,
                        )
                        bucket = scan.pubs if kind == "pub" else scan.subs
                        bucket.setdefault(resolved, []).append(hit)
        _walk_topics(
            child, rel, module_consts, class_consts, local_consts,
            import_map, all_module_consts, declared_params, scan,
        )


def _collect_seam_defs(tree: ast.Module, rel: str, scan: SeamScan) -> None:
    def _visit(body: list[ast.stmt], owner: str) -> None:
        for node in body:
            if isinstance(node, ast.ClassDef):
                _visit(node.body, node.name)
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                if owner and any(node.name.startswith(p) for p in _SEAM_PREFIXES):
                    scan.seam_defs.append(
                        SeamDef(rel, f"{owner}.{node.name}", node.name, node.lineno)
                    )
                _visit(node.body, "")  # nested defs: not a class method seam
            elif isinstance(node, (ast.If, ast.Try)):
                nested = list(node.body) + list(getattr(node, "orelse", []))
                _visit(nested, owner)

    _visit(tree.body, "")


def _collect_seam_usage(tree: ast.Module, is_test: bool, seam_names: set[str], scan: SeamScan) -> None:
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and any(node.name.startswith(p) for p in _SEAM_PREFIXES):
            continue  # the definition itself is not a "use"
        if isinstance(node, ast.Attribute) and node.attr in seam_names:
            bucket = scan.seam_usage.setdefault(node.attr, {"prod": 0, "test": 0})
            bucket["test" if is_test else "prod"] += 1


def scan_files(files: list[Path]) -> SeamScan:
    scan = SeamScan()
    trees: list[tuple[Path, str, ast.Module, bool]] = []
    for path in files:
        rel = _rel(path)
        try:
            tree = _parse(path)
        except SyntaxError as exc:
            print(f"seam_without_consumer: skip {rel}: {exc}")
            continue
        trees.append((path, rel, tree, _is_test_path(path)))

    # Pass 1 (prod files only): per-file module constants + import maps, and
    # a cross-file dotted-module-name -> constants index so a topic constant
    # imported from a shared module (rob_box_core.avatar_command, etc.)
    # resolves the same way at every call site that imports it.
    per_file_module_consts: dict[str, dict[str, str | None]] = {}
    per_file_imports: dict[str, dict[str, tuple[str, str]]] = {}
    all_module_consts: dict[str, dict[str, str | None]] = {}
    for path, rel, tree, is_test in trees:
        if is_test:
            continue
        module_consts = _module_consts(tree)
        per_file_module_consts[rel] = module_consts
        per_file_imports[rel] = _collect_imports(tree)
        dotted = _module_dotted_name(path)
        if dotted:
            all_module_consts[dotted] = module_consts

    # Pass 2 (prod files only): walk for topics + seam defs, now with full
    # cross-file constant knowledge available.
    for path, rel, tree, is_test in trees:
        if is_test:
            continue  # topics: only the real (prod) ROS graph counts
        _walk_topics(
            tree,
            rel,
            per_file_module_consts[rel],
            None,
            {},
            per_file_imports[rel],
            all_module_consts,
            {},
            scan,
        )
        _collect_seam_defs(tree, rel, scan)

    seam_names = {d.method_name for d in scan.seam_defs}
    for path, rel, tree, is_test in trees:
        _collect_seam_usage(tree, is_test, seam_names, scan)

    return scan


# ---------------------------------------------------------------------------
# Allow-list
# ---------------------------------------------------------------------------


def _load_allowlist() -> dict[str, dict[str, str]]:
    if not ALLOWLIST_FILE.exists():
        return {"publishers_without_local_subscriber": {}, "subscribers_without_local_publisher": {}}
    data = json.loads(ALLOWLIST_FILE.read_text(encoding="utf-8"))
    for category in ("publishers_without_local_subscriber", "subscribers_without_local_publisher"):
        entries = data.get(category, {})
        for topic, reason in entries.items():
            if not isinstance(reason, str) or not reason.strip():
                print(
                    f"seam_without_consumer: allow-list entry {category}/{topic} has no "
                    "explanation — every allow-list line must say who the consumer/producer "
                    "is and where (issue #2118). Fix scripts/lint/seam_allowlist.json."
                )
                sys.exit(2)
    return data


# ---------------------------------------------------------------------------
# Baseline
# ---------------------------------------------------------------------------


def _load_baseline() -> dict:
    if not BASELINE_FILE.exists():
        print(f"seam_without_consumer: missing baseline {BASELINE_FILE}; run --update-baseline")
        sys.exit(2)
    baseline = json.loads(BASELINE_FILE.read_text(encoding="utf-8"))
    # Совместимость со старым форматом baseline (до issue #2188 / voice-vr 03):
    # секций topic_type_mismatch / topic_type_mismatch_best_effort тогда не
    # было. Если ключа нет — добавляем как пустой список (новые находки
    # сразу начнут гейтить CI, если такие появятся). Это лучше, чем ронять
    # загрузку и путать разработчика.
    baseline.setdefault("topic_type_mismatch", [])
    baseline.setdefault("topic_type_mismatch_best_effort", [])
    return baseline


def _empty_baseline() -> dict:
    return {
        "publishers_without_local_subscriber": [],
        "subscribers_without_local_publisher": [],
        "method_seams_without_caller": [],
        "topic_type_mismatch": [],
        "topic_type_mismatch_best_effort": [],
    }


def _collect_topic_type_mismatches(scan: SeamScan) -> list[str]:
    """Список ключей ``"<topic>|<pub_type> != <sub_type>"`` для топиков,
    у которых ОБА конца шва (pub и sub) резолвнулись к разным
    msg-типам.

    Топики, где хоть один конец нерезолвим (msg_type_id == None),
    пропускаются — мы не имеем права утверждать ни «совпадают», ни
    «различаются». Это сознательно (см. граница применимости
    в шапке файла).
    """
    out: list[str] = []
    for topic in sorted(scan.pubs.keys() & scan.subs.keys()):
        pub_types = {h.msg_type_id for h in scan.pubs[topic] if h.msg_type_id}
        sub_types = {h.msg_type_id for h in scan.subs[topic] if h.msg_type_id}
        if not pub_types or not sub_types:
            continue
        # Сейчас по дизайну один pub-тип и один sub-тип на топик в
        # репо (если где-то их несколько — конфликт уже на уровне
        # нескольких pub'ов в одной ноде, что само по себе баг).
        # Берём первые непустые.
        pub_t = next(iter(pub_types))
        sub_t = next(iter(sub_types))
        if pub_t != sub_t:
            out.append(f"{topic}|{pub_t} != {sub_t}")
    return out


def _collect_best_effort_mismatches(scan: SeamScan) -> list[str]:
    """Топики, где одна сторона шва резолвима, а другая — нет.

    Тип-в-стиле ``self._heartbeat_msg_type = self._try_import_xxx()``
    (issue #2188 / quest_node.py → arbiter_node.py) — sub-тип не
    резолвится статически, pub-тип резолвится (``String``). Этого
    достаточно чтобы СИЛЬНО подозревать mismatch, но не достаточно
    чтобы гейтить CI (sub-тип мог быть и корректным — например,
    ``self._heartbeat_msg_type = String`` в ленивом импорте).

    Поэтому — отдельная best-effort секция: видна Шифу в выводе,
    подсвечивает точку, но НЕ считается FAIL (то же поведение,
    что и «не удалось разрешить» для топиков).
    """
    out: list[str] = []
    for topic in sorted(scan.pubs.keys() & scan.subs.keys()):
        pub_types = {h.msg_type_id for h in scan.pubs[topic] if h.msg_type_id}
        sub_types = {h.msg_type_id for h in scan.subs[topic] if h.msg_type_id}
        if pub_types and not sub_types:
            pub_t = next(iter(pub_types))
            out.append(f"{topic}|pub={pub_t} sub=? (unresolved msg-type)")
        elif sub_types and not pub_types:
            sub_t = next(iter(sub_types))
            out.append(f"{topic}|pub=? (unresolved msg-type) sub={sub_t}")
    return out


def cmd_update_baseline(files: list[Path], base_sha: str) -> int:
    scan = scan_files(files)
    allowlist = _load_allowlist()
    allow_pub = set(allowlist.get("publishers_without_local_subscriber", {}))
    allow_sub = set(allowlist.get("subscribers_without_local_publisher", {}))

    pub_only = sorted(t for t in scan.pubs if t not in scan.subs and t not in allow_pub)
    sub_only = sorted(t for t in scan.subs if t not in scan.pubs and t not in allow_sub)

    seam_only = sorted(
        d.qualname
        for d in scan.seam_defs
        if scan.seam_usage.get(d.method_name, {}).get("prod", 0) == 0
        and scan.seam_usage.get(d.method_name, {}).get("test", 0) > 0
    )

    baseline = {
        "version": 1,
        "created": date.today().isoformat(),
        "base_sha": base_sha or _git_head(),
        "publishers_without_local_subscriber": pub_only,
        "subscribers_without_local_publisher": sub_only,
        "method_seams_without_caller": seam_only,
        "topic_type_mismatch": _collect_topic_type_mismatches(scan),
        "topic_type_mismatch_best_effort": _collect_best_effort_mismatches(scan),
    }
    BASELINE_FILE.write_text(json.dumps(baseline, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"seam_without_consumer: baseline written to {_rel(BASELINE_FILE)}")
    print(f"seam_without_consumer: base_sha={baseline['base_sha']} created={baseline['created']}")
    return 0


# ---------------------------------------------------------------------------
# Check
# ---------------------------------------------------------------------------


def _report_topic_category(
    title: str,
    found: list[str],
    allowlist_entries: dict[str, str],
    baseline_entries: list[str],
    hits_by_topic: dict[str, list[TopicHit]],
) -> int:
    print(f"\n== {title} ==")
    if not found and not allowlist_entries:
        print("  (none)")
        return 0
    new_violations = 0
    baseline_set = set(baseline_entries)
    for topic in found:
        sites = hits_by_topic.get(topic, [])
        where = "; ".join(f"{h.file}:{h.line}" for h in sites)
        if topic in baseline_set:
            print(f"  [ok ] {topic}  ({where})  — grandfathered, see seam_baseline.json")
        else:
            new_violations += 1
            print(f"  [FAIL] {topic}  ({where})  — new seam without consumer")
    for topic, reason in sorted(allowlist_entries.items()):
        sites = hits_by_topic.get(topic, [])
        where = "; ".join(f"{h.file}:{h.line}" for h in sites) or "not currently emitted"
        print(f"  [allow] {topic}  ({where})  — {reason}")
    return new_violations


def cmd_check(files: list[Path], baseline: dict) -> int:
    scan = scan_files(files)
    allowlist = _load_allowlist()
    allow_pub = allowlist.get("publishers_without_local_subscriber", {})
    allow_sub = allowlist.get("subscribers_without_local_publisher", {})

    pub_only = sorted(t for t in scan.pubs if t not in scan.subs and t not in allow_pub)
    sub_only = sorted(t for t in scan.subs if t not in scan.pubs and t not in allow_sub)

    violations = 0
    violations += _report_topic_category(
        "ROS topics: publisher exists, no local subscriber",
        pub_only,
        allow_pub,
        baseline.get("publishers_without_local_subscriber", []),
        scan.pubs,
    )
    violations += _report_topic_category(
        "ROS topics: subscriber exists, no local publisher",
        sub_only,
        allow_sub,
        baseline.get("subscribers_without_local_publisher", []),
        scan.subs,
    )

    print("\n== Method seams: _publish_*/_on_* with test-only callers ==")
    seam_baseline = set(baseline.get("method_seams_without_caller", []))
    seam_hits = [
        d
        for d in scan.seam_defs
        if scan.seam_usage.get(d.method_name, {}).get("prod", 0) == 0
        and scan.seam_usage.get(d.method_name, {}).get("test", 0) > 0
    ]
    if not seam_hits:
        print("  (none)")
    for d in sorted(seam_hits, key=lambda d: d.qualname):
        usage = scan.seam_usage.get(d.method_name, {"prod": 0, "test": 0})
        loc = f"{d.file}:{d.line}"
        if d.qualname in seam_baseline:
            print(
                f"  [ok ] {d.qualname}  ({loc})  test-refs={usage['test']} prod-refs=0"
                "  — grandfathered, see seam_baseline.json"
            )
        else:
            violations += 1
            print(
                f"  [FAIL] {d.qualname}  ({loc})  test-refs={usage['test']} prod-refs=0"
                "  — seam covered by tests only, no production caller"
            )

    print("\n== ROS topics: pub/sub msg-type mismatch (issue #2188 / voice-vr 03) ==")
    mismatches = _collect_topic_type_mismatches(scan)
    type_baseline = set(baseline.get("topic_type_mismatch", []))
    if not mismatches:
        print("  (none)")
    for entry in mismatches:
        topic = entry.split("|", 1)[0]
        pub_sites = scan.pubs.get(topic, [])
        sub_sites = scan.subs.get(topic, [])
        pub_where = "; ".join(f"{h.file}:{h.line}" for h in pub_sites)
        sub_where = "; ".join(f"{h.file}:{h.line}" for h in sub_sites)
        if entry in type_baseline:
            print(
                f"  [ok ] {entry}  pub={pub_where}  sub={sub_where}  "
                "— grandfathered, see seam_baseline.json"
            )
        else:
            violations += 1
            print(
                f"  [FAIL] {entry}  pub={pub_where}  sub={sub_where}  "
                "— ROS 2 will not connect pub and sub with different msg types"
            )

    best_effort = _collect_best_effort_mismatches(scan)
    best_effort_baseline = set(baseline.get("topic_type_mismatch_best_effort", []))
    if best_effort:
        print(
            "\n== Best-effort mismatch suspects (informational — "
            "msg-type resolved on one side only, never gates CI) =="
        )
        for entry in best_effort:
            topic = entry.split("|", 1)[0]
            pub_sites = scan.pubs.get(topic, [])
            sub_sites = scan.subs.get(topic, [])
            pub_where = "; ".join(f"{h.file}:{h.line}" for h in pub_sites)
            sub_where = "; ".join(f"{h.file}:{h.line}" for h in sub_sites)
            status = (
                "— grandfathered, see seam_baseline.json"
                if entry in best_effort_baseline
                else "— NEW suspect, recommend manual check"
            )
            print(f"  [??] {entry}  pub={pub_where}  sub={sub_where}  {status}")

    print("\n== Could not resolve (informational — never gates CI) ==")
    if not scan.unresolved and not scan.unresolved_msg_types:
        print("  (none)")
    for u in scan.unresolved:
        kind_name = "create_publisher" if u.kind == "pub" else "create_subscription"
        print(f"  [??] {u.file}:{u.line}  {kind_name}(...) topic expr = {u.expr_src}")
    for u in scan.unresolved_msg_types:
        kind_name = "create_publisher" if u.kind == "pub" else "create_subscription"
        print(f"  [??] {u.file}:{u.line}  {kind_name}(...) msg-type expr = {u.expr_src}")

    print()
    if violations:
        print(f"seam_without_consumer: FAIL — {violations} new seam(s) without consumer.")
    else:
        print("seam_without_consumer: OK — no new seams without consumer.")
    return 1 if violations else 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _force_utf8_stdio() -> None:
    """Best-effort UTF-8 stdout/stderr — allow-list reasons are Cyrillic.

    CI (ubuntu-latest) already defaults to UTF-8; this only matters for local
    Windows runs where the console codepage would otherwise raise
    ``UnicodeEncodeError`` on the em-dashes/Cyrillic in report lines.
    """
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8")
            except Exception:
                pass


def main(argv: list[str] | None = None) -> int:
    _force_utf8_stdio()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "paths",
        nargs="*",
        type=Path,
        default=list(DEFAULT_TARGETS),
        help="Python files or directories to scan (default: src/)",
    )
    parser.add_argument(
        "--update-baseline",
        action="store_true",
        help="Snapshot current seams-without-consumer (minus allow-list) into the baseline file",
    )
    parser.add_argument("--base-sha", default="", help="Override base commit SHA in baseline")
    args = parser.parse_args(argv)

    raw_targets = [path if path.is_absolute() else REPO_ROOT / path for path in args.paths]
    files = _expand_targets(raw_targets)
    if not files:
        print("seam_without_consumer: no Python files found under the given targets")
        return 2

    if args.update_baseline:
        return cmd_update_baseline(files, args.base_sha)
    return cmd_check(files, _load_baseline())


if __name__ == "__main__":
    raise SystemExit(main())
