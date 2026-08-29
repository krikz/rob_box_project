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
    REPO_ROOT / "src" / "rob_box_voice" / "config" / "voice_assistant.yaml",
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
