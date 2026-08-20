"""YAML↔declare_parameter consistency (issue #1004 regression guard).

Issue #1004 root cause: per-node YAML keys were nested under
``/**: ros__parameters: <node>:``, so ROS 2 created dotted parameter names
(``dialogue_node.llm_provider``) while the node code read unprefixed names
(``get_parameter("llm_provider")``). rclpy silently drops undeclared keys,
so the whole YAML was dead: user edits were ignored without any error.

The fix (ADR-0004) split the config into per-node files with the standard
ROS 2 layout (``<node>: ros__parameters: <flat keys>``). These tests guard
the invariant: every key present in a node's YAML must be declared by that
node's ``declare_parameter(...)`` call — otherwise the key is silently
ignored again.

Static test: parses ``declare_parameter("name", ...)`` calls from the node
sources (no ROS runtime needed).
"""

import re
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
SRC_CONFIG = REPO_ROOT / "src/rob_box_voice/config"
DOCKER_CONFIG = REPO_ROOT / "docker/vision/config/voice_assistant"
NODE_SRC = REPO_ROOT / "src/rob_box_voice/rob_box_voice"

# node name → (source file, yaml files to check)
NODES = {
    "audio_node": ("audio_node.py", ["audio_node.yaml"]),
    "stt_node": ("stt_node.py", ["stt_node.yaml"]),
    "tts_node": ("tts_node.py", ["tts_node.yaml"]),
    "dialogue_node": ("dialogue_node.py", ["dialogue_node.yaml"]),
    "sound_node": ("sound_node.py", ["sound_node.yaml"]),
    "led_node": ("led_node.py", ["led_node.yaml"]),
    "command_node": ("command_node.py", ["command_node.yaml"]),
    "speaker_id_node": ("speaker_id_node.py", ["speaker_id_node.yaml"]),
}

# dialogue_node declares per-provider params dynamically in a loop:
# for pname in ("minimax", "deepseek", "mimo", "qwen"): declare(f"{pname}.{key}")
_DYNAMIC_PROVIDER_KEYS = ("api_key", "base_url", "model",
                          "temperature", "max_tokens", "timeout_s")
_DYNAMIC_PROVIDERS = ("minimax", "deepseek", "mimo", "qwen")


def _declared_params(src_path: Path) -> set:
    src = src_path.read_text(encoding="utf-8")
    names = set()
    for m in re.finditer(r"declare_parameter\(\s*(['\"])([^'\"]+)\1", src):
        names.add(m.group(2))
    if src_path.name == "dialogue_node.py":
        for pname in _DYNAMIC_PROVIDERS:
            for key in _DYNAMIC_PROVIDER_KEYS:
                names.add(f"{pname}.{key}")
    return names


def _yaml_keys(node_name: str, cfg: dict) -> set:
    """Expand nested dict params to dotted names (1 level, e.g. colors.idle)."""
    params = cfg.get(node_name, {}).get("ros__parameters", {})
    keys = set()
    for k, v in params.items():
        if isinstance(v, dict):
            for kk in v:
                keys.add(f"{k}.{kk}")
        else:
            keys.add(k)
    return keys


def _load_config(path: Path, node_name: str) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert "/**" not in data, (
        f"{path.name}: запрещён старый формат '/**' — он создаёт dotted-имена, "
        f"которые ноды не читают (issue #1004)"
    )
    return data


@pytest.mark.parametrize("node_name", sorted(NODES))
def test_src_yaml_keys_declared(node_name: str):
    src_file, yaml_files = NODES[node_name]
    declared = _declared_params(NODE_SRC / src_file)
    for yf in yaml_files:
        path = SRC_CONFIG / yf
        if not path.exists():
            continue
        cfg = _load_config(path, node_name)
        ykeys = _yaml_keys(node_name, cfg)
        undeclared = sorted(ykeys - declared)
        assert not undeclared, (
            f"{yf}: YAML-ключи {undeclared} не объявлены в {src_file} — "
            f"rclpy их молча игнорирует (issue #1004). Добавьте declare_parameter "
            f"или удалите ключ из YAML."
        )


@pytest.mark.parametrize("node_name", sorted(NODES))
def test_docker_yaml_keys_declared(node_name: str):
    """Docker operator config (mounted /config/voice_assistant) — same rule."""
    src_file, _ = NODES[node_name]
    declared = _declared_params(NODE_SRC / src_file)
    path = DOCKER_CONFIG / f"{node_name}.yaml"
    if not path.exists():
        pytest.skip(f"{path} not found")
    cfg = _load_config(path, node_name)
    ykeys = _yaml_keys(node_name, cfg)
    undeclared = sorted(ykeys - declared)
    assert not undeclared, (
        f"{path.name}: YAML-ключи {undeclared} не объявлены в {src_file} — "
        f"rclpy их молча игнорирует (issue #1004)."
    )


def test_legacy_voice_assistant_yaml_is_standard_format():
    """src/rob_box_voice/config/voice_assistant.yaml — сводка в стандартном
    формате (никаких /** и вложенных имён нод внутри ros__parameters)."""
    path = SRC_CONFIG / "voice_assistant.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert "/**" not in data
    for node_name, (src_file, _) in NODES.items():
        assert node_name in data, (
            f"voice_assistant.yaml: нет секции '{node_name}'"
        )
        declared = _declared_params(NODE_SRC / src_file)
        ykeys = _yaml_keys(node_name, data)
        undeclared = sorted(ykeys - declared)
        assert not undeclared, (
            f"voice_assistant.yaml '{node_name}': ключи {undeclared} не "
            f"объявлены в {src_file} (issue #1004)."
        )


def test_dialogue_configs_route_to_minimax():
    """Both dialogue configs (src + docker) keep MiniMax as primary LLM."""
    for path in (SRC_CONFIG / "dialogue_node.yaml",
                 DOCKER_CONFIG / "dialogue_node.yaml"):
        if not path.exists():
            pytest.skip(f"{path} not found")
        cfg = _load_config(path, "dialogue_node")
        llm_providers = cfg["dialogue_node"]["ros__parameters"]["llm_providers"]
        assert llm_providers.startswith("minimax"), (
            f"{path.name}: llm_providers={llm_providers!r} — MiniMax должен "
            f"быть primary (issue #1004)"
        )


def test_tts_configs_route_to_minimax():
    """Both tts configs (src + docker) keep MiniMax as TTS provider."""
    for path in (SRC_CONFIG / "tts_node.yaml", DOCKER_CONFIG / "tts_node.yaml"):
        if not path.exists():
            pytest.skip(f"{path} not found")
        cfg = _load_config(path, "tts_node")
        provider = cfg["tts_node"]["ros__parameters"]["provider"]
        assert provider == "minimax", (
            f"{path.name}: provider={provider!r} — должен быть minimax "
            f"(issue #1004)"
        )
