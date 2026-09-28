"""Guard: на роботе ``ROB_BOX_MUSIC_ALIGN_CLOCK`` включён по умолчанию (issue #3112).

Фикс фазы клока Renardo (``Clock.set_time`` перед треком) в коде по умолчанию
ВЫКЛ (``tools/music.py:music_align_clock_enabled``) до live-подтверждения.
Владелец 28.09 решил включить его на роботе через compose-окружение
сервиса ``voice-assistant``, где работает ``mcp_server`` (он исполняет
``compose_music``), с возможностью выключить из ``docker/vision/.env``.

Цепочка, которую сторожит этот тест:
compose ``environment`` → ENTRYPOINT/``ros_with_namespace.sh`` (``exec "$@"``)
→ ``start_voice_assistant.sh`` (``exec ros2 launch``) → ``Node(mcp_server)``
без ``env=`` (launch_ros наследует окружение процесса launch).
"""
from __future__ import annotations

import os
import re
from pathlib import Path


def _repo_root(start: Path) -> Path:
    """Корень репо в dev и в CI (test_ws), как в test_issue_3114_scsynth_block_size."""
    override = os.environ.get("ROB_BOX_REPO_ROOT")
    if override and (Path(override) / "docker").is_dir():
        return Path(override).resolve()
    for parent in [start, *start.parents]:
        if (parent / "src").is_dir() and (parent / "docker").is_dir():
            return parent
    raise RuntimeError(f"repo root not found for {start!s}; set ROB_BOX_REPO_ROOT")


REPO = _repo_root(Path(__file__).resolve())
COMPOSE = REPO / "docker/vision/docker-compose.yaml"
DOT_ENV = REPO / "docker/vision/.env"
LAUNCH = REPO / "docker/vision/config/voice_assistant/voice_assistant_headless.launch.py"
START = REPO / "docker/vision/scripts/voice_assistant/start_voice_assistant.sh"
MUSIC = REPO / "src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py"

ENV = "ROB_BOX_MUSIC_ALIGN_CLOCK"


def _service_block(text: str, name: str) -> str:
    m = re.search(rf"^  {re.escape(name)}:\n(.*?)(?=^  [A-Za-z0-9_-]+:\n|\Z)", text, re.M | re.S)
    assert m, f"сервис {name} не найден в docker-compose.yaml"
    return m.group(1)


def test_compose_voice_assistant_enables_align_clock_by_default() -> None:
    block = _service_block(COMPOSE.read_text(encoding="utf-8"), "voice-assistant")
    line = f"- {ENV}=${{{ENV}:-1}}"
    assert line in block, f"нет `{line}` в environment сервиса voice-assistant"


def test_env_name_matches_code() -> None:
    # Имя переменной в compose должно совпадать с тем, что читает mcp_server.
    assert f'_ALIGN_CLOCK_ENV = "{ENV}"' in MUSIC.read_text(encoding="utf-8")


def test_dot_env_does_not_silently_disable() -> None:
    # Коммитнутый docker/vision/.env не должен перебивать дефолт на 0
    # (откат делается там осознанно — тогда этот тест надо поправить).
    text = DOT_ENV.read_text(encoding="utf-8")
    m = re.search(rf"^\s*{ENV}\s*=\s*(\S*)", text, re.M)
    assert m is None or m.group(1).strip("\"'") in {"1", "true", "yes", "on"}, m.group(0)


def test_mcp_server_node_inherits_env() -> None:
    text = LAUNCH.read_text(encoding="utf-8")
    m = re.search(r"mcp_server\s*=\s*Node\((.*?)\n\s*\)\n", text, re.S)
    assert m, "Node(mcp_server) не найден в headless launch"
    assert "executable='mcp_server'" in m.group(1)
    # env= у launch_ros заменяет окружение целиком — флаг бы потерялся.
    assert not re.search(r"\benv\s*=", m.group(1)), m.group(1)


def test_start_script_does_not_clear_env() -> None:
    text = START.read_text(encoding="utf-8")
    assert "env -i" not in text
    assert not re.search(rf"^\s*unset\s+.*\b{ENV}\b", text, re.M)
    assert "exec ros2 launch /config/voice_assistant/voice_assistant_headless.launch.py" in text
