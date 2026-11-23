"""Guard: прототип B (tag_relocalization) не стартует сам (#4346, #4694).

Draft PR #4568: «поза по тегу» стабильно ошибается на ~4.4 м зеркально (tb1/tb2).
B остаётся выключенным до структурного фикса (ADR-0148). Проверяется поведение
launch/compose/entry points, а не тексты: на роботе и в default-пути B не поднимается.
Docker и ROS не нужны.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
DOCKER = REPO_ROOT / "docker"
SRC = REPO_ROOT / "src"

# Имена, по которым узнаём прототип B (нода / сервис / модуль).
B_MARKERS = (
    "tag_relocalization",
    "tag_relocalization_node",
    "tagreloc",
    "SIM_RELOC",
)

# Compose-сервис B, если появится, имеет право жить только за профилем reloc.
B_SERVICE_NAMES = frozenset({"tagreloc"})
B_ALLOWED_PROFILES = frozenset({"reloc"})


class _Loader(yaml.SafeLoader):
    """SafeLoader + теги слияния compose (``!override`` / ``!reset``)."""


def _passthrough(loader, node):
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node)
    if isinstance(node, yaml.MappingNode):
        return loader.construct_mapping(node)
    return loader.construct_scalar(node)


_Loader.add_constructor("!override", _passthrough)
_Loader.add_constructor("!reset", _passthrough)


def _load_yaml(path: Path) -> dict:
    return yaml.load(path.read_text(encoding="utf-8"), Loader=_Loader)  # noqa: S506 — SafeLoader


def _compose_files() -> list[Path]:
    return sorted(
        p
        for p in DOCKER.rglob("*")
        if p.is_file() and p.suffix in {".yaml", ".yml"} and "compose" in p.name.lower()
    )


def _launch_files() -> list[Path]:
    out: list[Path] = []
    for root in (DOCKER, SRC):
        out.extend(p for p in root.rglob("*.launch.py") if p.is_file())
    return sorted(out)


def _stringify(value) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if isinstance(value, (list, tuple)):
        return " ".join(_stringify(v) for v in value)
    if isinstance(value, dict):
        return " ".join(f"{k} {_stringify(v)}" for k, v in value.items())
    return str(value)


def _default_env(value: str) -> str:
    """``${VAR:-default}`` → ``default`` — как compose при пустом окружении."""
    return re.sub(r"\$\{[A-Za-z_][A-Za-z0-9_]*:-([^}]*)\}", r"\1", value)


def test_tagreloc_not_in_default_compose_services() -> None:
    """Сервис tagreloc, если есть, только за profile reloc; default up его не берёт."""
    for path in _compose_files():
        data = _load_yaml(path)
        services = data.get("services") or {}
        if not isinstance(services, dict):
            continue
        for name, svc in services.items():
            if not isinstance(svc, dict):
                continue
            blob = " ".join(
                (
                    name,
                    _stringify(svc.get("command")),
                    _stringify(svc.get("entrypoint")),
                    _stringify(svc.get("environment")),
                )
            )
            mentions_b = name in B_SERVICE_NAMES or any(m in blob for m in B_MARKERS)
            if not mentions_b:
                continue
            profiles = set(svc.get("profiles") or [])
            assert profiles == B_ALLOWED_PROFILES, (
                f"{path.relative_to(REPO_ROOT)} service {name!r}: "
                f"прототип B должен быть только за profiles={sorted(B_ALLOWED_PROFILES)}, "
                f"сейчас {sorted(profiles)}"
            )
            # Явный enabled:=true допустим только внутри profile reloc (стенд).
            # Default compose-путь без профиля B поднимать не должен — это и ловит profiles.
            env = svc.get("environment") or {}
            if isinstance(env, list):
                env_map = dict(item.partition("=")[::2] for item in env if isinstance(item, str))
            else:
                env_map = {k: str(v) for k, v in env.items()}
            for key, raw in env_map.items():
                if key == "SIM_RELOC" or key.endswith("SIM_RELOC"):
                    assert _default_env(raw).lower() in {"", "false", "0"}, (
                        f"{path.relative_to(REPO_ROOT)} {name}.{key} default={raw!r} — B включён"
                    )


def test_tag_relocalization_not_in_launch_files() -> None:
    """Ни один launch.py не поднимает tag_relocalization (робот / sim default)."""
    pattern = re.compile(r"tag_relocalization|tagreloc", re.IGNORECASE)
    offenders: list[str] = []
    for path in _launch_files():
        text = path.read_text(encoding="utf-8")
        if pattern.search(text):
            offenders.append(str(path.relative_to(REPO_ROOT)))
    assert not offenders, (
        "прототип B не должен быть в launch (зеркальная поза ~4.4 м, #4346): " + ", ".join(offenders)
    )


def test_tag_relocalization_not_in_perception_console_scripts() -> None:
    """setup.py perception не регистрирует автозапуск B как console_script."""
    setup_py = SRC / "rob_box_perception" / "setup.py"
    assert setup_py.is_file(), setup_py
    tree = ast.parse(setup_py.read_text(encoding="utf-8"), filename=str(setup_py))
    scripts: list[str] = []

    class _Visitor(ast.NodeVisitor):
        def visit_Call(self, node: ast.Call) -> None:
            for kw in node.keywords:
                if kw.arg != "entry_points" or not isinstance(kw.value, ast.Dict):
                    continue
                for key, val in zip(kw.value.keys, kw.value.values):
                    if not (isinstance(key, ast.Constant) and key.value == "console_scripts"):
                        continue
                    if isinstance(val, ast.List):
                        for elt in val.elts:
                            if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                                scripts.append(elt.value)
            self.generic_visit(node)

    _Visitor().visit(tree)
    joined = "\n".join(scripts)
    for marker in ("tag_relocalization", "tagreloc"):
        assert marker not in joined, f"console_scripts тянет B: {joined!r}"


def test_tag_relocalization_node_defaults_enabled_false_if_present() -> None:
    """Если нода B уже в дереве — параметр enabled по умолчанию false, без авто-start."""
    node = SRC / "rob_box_perception" / "rob_box_perception" / "tag_relocalization_node.py"
    if not node.is_file():
        return
    text = node.read_text(encoding="utf-8")
    # declare_parameter("enabled", False) — единственный допустимый default.
    match = re.search(
        r'declare_parameter\(\s*["\']enabled["\']\s*,\s*([^)]+)\)',
        text,
    )
    assert match, f"{node.name}: нет declare_parameter('enabled', ...)"
    default = match.group(1).strip()
    assert default in {"False", "false"}, (
        f"{node.name}: enabled default={default!r}, нужен False (прототип B выключен)"
    )
