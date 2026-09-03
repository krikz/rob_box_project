"""The dialogue's own state must outlive its container.

Live on vision 29.08: `sqlite_db_path` was `~/.rob_box/voice.db`, which
inside the voice-assistant container is `/root/.rob_box/voice.db`. Nothing
is mounted there, so the file was created at 15:29 together with the
container and held 69 turns, none older — every restart threw the whole
conversation away while `/data/voice_memory.db` (5288 turns, on the
volume) sat there being a different store that the LLM never reads.

These are static config checks: no ROS 2, no container, no robot.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml


def _repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "docker").is_dir() and (parent / "src").is_dir():
            return parent
    return start.parents[5]


REPO_ROOT = _repo_root(Path(__file__).resolve())
COMPOSE = REPO_ROOT / "docker" / "vision" / "docker-compose.yaml"

# Every config that sets the parameter — a fix in one and not the others is
# how it drifted in the first place.
CONFIGS = (
    REPO_ROOT / "docker" / "vision" / "config" / "voice_assistant" / "dialogue_node.yaml",
    REPO_ROOT / "src" / "rob_box_voice" / "config" / "dialogue_node.yaml",
)

# Per ADR-0004 (issue #1004) every node reads its own `<node>.yaml`.
PACKAGE_CONFIG_DIR = REPO_ROOT / "src" / "rob_box_voice" / "config"
DEPLOYED_CONFIG_DIR = (
    REPO_ROOT / "docker" / "vision" / "config" / "voice_assistant"
)

# `docker-compose.yaml` mounts `./data/voice` here for the voice-assistant.
MOUNTED = "/data/"

# VoiceMemory (mcp_server) owns this file. The harness must NOT share it:
# both declare `waypoints` and `faq_items` with incompatible schemas
# (harness: `waypoints.name` PRIMARY KEY; VoiceMemory: `map_id NOT NULL`
# plus a foreign key to `maps`). `CREATE TABLE IF NOT EXISTS` would accept
# the existing table silently and every insert would then fail at runtime.
VOICE_MEMORY_DB = "/data/voice_memory.db"


def _param(config: Path, name: str):
    with config.open(encoding="utf-8") as fh:
        loaded = yaml.safe_load(fh)
    return loaded["dialogue_node"]["ros__parameters"][name]


@pytest.mark.parametrize("config", CONFIGS, ids=lambda p: p.name)
def test_dialogue_db_lives_on_a_mounted_volume(config: Path) -> None:
    path = _param(config, "sqlite_db_path")
    assert path.startswith(MOUNTED), (
        f"{config.name}: sqlite_db_path={path!r} is not under {MOUNTED} — "
        "the container has no volume there, so the dialogue history dies "
        "with the container"
    )


@pytest.mark.parametrize("config", CONFIGS, ids=lambda p: p.name)
def test_dialogue_db_is_not_the_voice_memory_file(config: Path) -> None:
    path = _param(config, "sqlite_db_path")
    assert path != VOICE_MEMORY_DB, (
        f"{config.name}: the harness store cannot share {VOICE_MEMORY_DB} "
        "with VoiceMemory — `waypoints` and `faq_items` have incompatible "
        "schemas in the two, and CREATE TABLE IF NOT EXISTS hides it until "
        "the first insert fails"
    )


@pytest.mark.parametrize("config", CONFIGS, ids=lambda p: p.name)
def test_history_excluded_tools_names_a_live_tool(config: Path) -> None:
    """The filter must name tools that still exist.

    It defaulted to `handle_navigation` / `handle_music` — Compositor skill
    facades deleted after the party regression — so it excluded nothing at
    all while looking configured.
    """
    from rob_box_core.tool_catalog import tool_names

    known = set(tool_names())
    excluded = _param(config, "history_excluded_tools")
    assert excluded, f"{config.name}: empty history_excluded_tools"
    unknown = [name for name in excluded if name not in known]
    assert not unknown, (
        f"{config.name}: history_excluded_tools names tools that do not "
        f"exist: {unknown}. The filter silently excludes nothing."
    )


def test_compose_mounts_the_data_volume() -> None:
    """`/data` has to actually be a volume, or the checks above prove nothing."""
    content = COMPOSE.read_text(encoding="utf-8")
    assert ":/data\n" in content, (
        "docker-compose.yaml no longer mounts anything at /data — "
        "sqlite_db_path=/data/... would be container-local again"
    )


# ---------------------------------------------------------------------------
# ADR-0004: one YAML per node, and no monolith to drift against it
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "config_dir",
    (PACKAGE_CONFIG_DIR, DEPLOYED_CONFIG_DIR),
    ids=("package", "deployed"),
)
def test_no_multi_node_yaml_in_the_config_dir(config_dir: Path) -> None:
    """No config file may configure more than one node.

    ADR-0004 / issue #1004 replaced the monolithic `voice_assistant.yaml`
    with per-node files, because nested `<node>:` sections in a shared file
    become dotted parameters (`dialogue_node.llm_provider`) that
    `get_parameter("llm_provider")` never finds — the node runs on defaults
    and looks configured.

    The deployment dropped it and three files say so in as many words
    (`docker/vision/README.md`, `start_voice_assistant.sh`,
    `voice_assistant_headless.launch.py`). The package kept shipping it
    anyway, with a `wake_words` list eight spellings short of the live one —
    the same drift that issue #1252 produced and #1734 paid for — while
    `src/rob_box_voice/README.md` still told operators to edit it.
    """
    offenders = {}
    for path in sorted(config_dir.glob("*.yaml")):
        with path.open(encoding="utf-8") as fh:
            loaded = yaml.safe_load(fh) or {}
        if not isinstance(loaded, dict):
            continue
        nodes = [
            key
            for key, value in loaded.items()
            if isinstance(value, dict) and "ros__parameters" in value
        ]
        if len(nodes) > 1:
            offenders[path.name] = nodes
    assert not offenders, (
        "config files declaring more than one node (ADR-0004 forbids the "
        f"monolith — split them per node): {offenders}"
    )


def test_every_package_config_matches_a_node_name() -> None:
    """`<node>.yaml` must configure the node it is named after.

    A file whose only section is a *different* node is loaded for nobody:
    the launch file passes `config_dir/<node>.yaml` to that node alone.
    """
    mismatched = {}
    for path in sorted(PACKAGE_CONFIG_DIR.glob("*_node.yaml")):
        with path.open(encoding="utf-8") as fh:
            loaded = yaml.safe_load(fh) or {}
        nodes = [
            key.lstrip("/")
            for key, value in loaded.items()
            if isinstance(value, dict) and "ros__parameters" in value
        ]
        if nodes and path.stem not in nodes:
            mismatched[path.name] = nodes
    assert not mismatched, (
        f"config files that configure a node they are not named for: {mismatched}"
    )


# ---------------------------------------------------------------------------
# wake_words: one declaration
# ---------------------------------------------------------------------------

E2E_CONFIG = (
    REPO_ROOT / "docker" / "vision" / "test" / "config" / "voice_assistant_test.yaml"
)
ALL_CONFIG_DIRS = (PACKAGE_CONFIG_DIR, DEPLOYED_CONFIG_DIR)


def _node_params(path: Path):
    with path.open(encoding="utf-8") as fh:
        loaded = yaml.safe_load(fh) or {}
    for key, value in loaded.items():
        if isinstance(value, dict) and "ros__parameters" in value:
            yield key.lstrip("/"), (value["ros__parameters"] or {})


def test_wake_words_are_declared_exactly_once() -> None:
    """No YAML may carry its own copy of the wake-word list.

    There were seven declarations and three different lists:
    `dialogue_node.py` and `stt_node.py` defaulted to 13 spellings, four
    YAMLs carried 21, the e2e config carried the same stale 13 — so the
    test rig could not hear eight spellings the robot could. The comment
    at `dialogue_node.py` on the barge-in topic already names this exact
    failure: «Дублирование параметра — ровно тот класс ошибки, который уже
    случился с wake_words (issue #1252, два YAML, разъехались) и который и
    породил issue #1734».

    `rob_box_voice.core.dialogue_text.DEFAULT_WAKE_WORDS` is the one list,
    and its order matters — `strip_wake_word` alternates leftmost-first, so
    «роб» must come after «роб бокс».
    """
    offenders = {}
    for config_dir in ALL_CONFIG_DIRS:
        for path in sorted(config_dir.glob("*.yaml")):
            for node, params in _node_params(path):
                if "wake_words" in params:
                    offenders[f"{path.name}:{node}"] = params["wake_words"]
    for node, params in _node_params(E2E_CONFIG):
        if "wake_words" in params:
            offenders[f"{E2E_CONFIG.name}:{node}"] = params["wake_words"]
    assert not offenders, (
        "wake_words re-declared in YAML — the list lives in "
        "rob_box_voice.core.dialogue_text.DEFAULT_WAKE_WORDS and nowhere "
        f"else: {sorted(offenders)}"
    )


def test_node_defaults_use_the_canonical_wake_words() -> None:
    """Both nodes must default to the shared list, not a copy of it."""
    from rob_box_voice.core.dialogue_text import DEFAULT_WAKE_WORDS

    assert DEFAULT_WAKE_WORDS, "the canonical list is empty"
    for module in ("dialogue_node.py", "stt_node.py"):
        source = (
            REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / module
        ).read_text(encoding="utf-8")
        assert 'declare_parameter("wake_words", list(DEFAULT_WAKE_WORDS))' in source, (
            f"{module} declares wake_words with its own literal instead of "
            "DEFAULT_WAKE_WORDS"
        )
