"""Tests for the config loader.

Covers:

* ``HarnessConfig.from_dict`` validation: missing sections, bad types.
* ``load_config`` from a YAML file (real PyYAML).
* ``load_config`` from a chain of YAML files (later wins).
* ENV placeholder interpolation (``${VAR}``).
* Secrets end-to-end via the ``secrets`` map.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rob_box_harness.config import HarnessConfig, load_config
from rob_box_harness.errors import ConfigError


def test_from_dict_minimal() -> None:
    """A minimal dict with ``harness.kind`` produces a valid config."""
    config = HarnessConfig.from_dict({"harness": {"kind": "dialog"}})
    assert config.harness == "dialog"
    assert config.name == "unnamed_harness"
    assert config.state == {}
    assert config.llm is None
    assert config.logging.level == "INFO"


def test_from_dict_full() -> None:
    """A full config round-trips through ``from_dict``."""
    raw = {
        "harness": {"kind": "dialog", "name": "main", "state": {"a": 1}},
        "llm": {"provider": "deepseek", "model": "ds-1", "fallback": ["mimo"]},
        "tools": {"provider": "fake", "endpoint": "/x"},
        "memory": {"backend": "in_memory"},
        "effects": {"bus": "noop"},
        "transport": {"kind": "fake"},
        "logging": {"level": "DEBUG"},
    }
    config = HarnessConfig.from_dict(raw)
    assert config.harness == "dialog"
    assert config.name == "main"
    assert config.state["a"] == 1
    assert config.llm is not None and config.llm.provider == "deepseek"
    assert config.llm is not None and config.llm.fallback == ("mimo",)
    assert config.tools is not None and config.tools.endpoint == "/x"
    assert config.memory is not None and config.memory.backend == "in_memory"
    assert config.effects is not None and config.effects.bus == "noop"
    assert config.transport is not None and config.transport.kind == "fake"


def test_from_dict_missing_kind_raises() -> None:
    """``harness.kind`` is mandatory and must be a non-empty string."""
    with pytest.raises(ConfigError, match="harness.kind"):
        HarnessConfig.from_dict({"harness": {"name": "x"}})


def test_from_dict_top_level_must_be_mapping() -> None:
    """A non-mapping top level raises ``ConfigError``."""
    with pytest.raises(ConfigError, match="top-level"):
        HarnessConfig.from_dict(["not a mapping"])  # type: ignore[arg-type]


def test_from_dict_unknown_section_warns_via_validation() -> None:
    """Unknown sections are silently ignored (forward-compatible)."""
    # Future harness versions may add new top-level sections; today's
    # config must not crash on them.
    config = HarnessConfig.from_dict(
        {"harness": {"kind": "x"}, "future_section": {"y": 1}}
    )
    assert config.harness == "x"


def test_load_config_from_yaml(tmp_path: Path) -> None:
    """``load_config`` reads a YAML file and parses it."""
    yaml_path = tmp_path / "harness.yaml"
    yaml_path.write_text(
        "harness:\n"
        "  kind: dialog\n"
        "  name: from_yaml\n"
        "llm:\n"
        "  provider: deepseek\n"
        "  model: ds-1\n"
    )
    config = load_config(yaml_path)
    assert config.harness == "dialog"
    assert config.name == "from_yaml"
    assert config.llm is not None and config.llm.model == "ds-1"


def test_load_config_layers(tmp_path: Path) -> None:
    """Two layered YAML files merge with the latter winning."""
    base = tmp_path / "base.yaml"
    base.write_text(
        "harness:\n  kind: dialog\n  name: from_base\n"
        "llm:\n  provider: deepseek\n  model: ds-1\n"
    )
    override = tmp_path / "override.yaml"
    override.write_text(
        "harness:\n  name: from_override\n"
        "llm:\n  model: ds-2\n"
    )
    config = load_config([base, override])
    assert config.name == "from_override"
    assert config.llm is not None and config.llm.model == "ds-2"
    assert config.llm is not None and config.llm.provider == "deepseek"


def test_load_config_env_interpolation(tmp_path: Path) -> None:
    """``${ENV_VAR}`` placeholders are resolved via ``secrets``."""
    yaml_path = tmp_path / "harness.yaml"
    yaml_path.write_text(
        "harness:\n  kind: dialog\n"
        "llm:\n  provider: deepseek\n  api_key: ${SECRET_TOKEN}\n"
    )
    config = load_config(yaml_path, secrets={"SECRET_TOKEN": "shh"})
    assert config.llm is not None
    assert config.llm.api_key == "shh"


def test_load_config_missing_env_raises(tmp_path: Path) -> None:
    """An unresolved placeholder raises ``ConfigError``."""
    yaml_path = tmp_path / "harness.yaml"
    yaml_path.write_text(
        "harness:\n  kind: dialog\n"
        "llm:\n  provider: deepseek\n  api_key: ${MISSING_VAR}\n"
    )
    with pytest.raises(ConfigError, match="MISSING_VAR"):
        load_config(yaml_path, secrets={})


def test_load_config_missing_file(tmp_path: Path) -> None:
    """A non-existent path raises ``ConfigError``."""
    with pytest.raises(ConfigError, match="not found"):
        load_config(tmp_path / "nope.yaml")


def test_load_config_top_level_must_be_mapping(tmp_path: Path) -> None:
    """A YAML file whose top level is a list raises ``ConfigError``."""
    yaml_path = tmp_path / "harness.yaml"
    yaml_path.write_text("- not\n- a\n- mapping\n")
    with pytest.raises(ConfigError, match="top-level"):
        load_config(yaml_path)


def test_load_config_none_falls_back_to_env() -> None:
    """``load_config(None)`` without ``ROB_BOX_CONFIG_PATH`` returns a default."""
    import os

    os.environ.pop("ROB_BOX_CONFIG_PATH", None)
    config = load_config(None)
    assert config.harness == "dialog"
