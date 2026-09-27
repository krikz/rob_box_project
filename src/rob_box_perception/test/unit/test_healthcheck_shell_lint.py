"""Shell-lint регресс для docker/**/healthcheck*.sh (issue #2703, #2704).

Два антипаттерна, которые ЛОМАЛИ healthcheck молча (fallback на
pgrep-only / вечный FAIL по чужой причине), без единого красного
теста до этого PR:

1. ``command -v ros2`` до ``source /opt/ros/.../setup.bash``: ros2 CLI
   не в базовом PATH образа, поэтому такая проверка ВСЕГДА уходит в
   fallback ``exit 0`` ("pgrep-only") — issue #2704, баг из
   ``healthcheck_face_frame.sh`` (фикс #2608 не перенесли из
   ``healthcheck_frame.sh``).
2. Вызов ``ros2 <topic|node|service|action> ...`` без ``--no-daemon``:
   демон ros2cli на Vision Pi падает под rmw_zenoh
   (``xmlrpc.client.Fault: !rclpy.ok()``, issue #2703 п.1) — без
   ``--no-daemon`` любой такой вызов фейлится НЕЗАВИСИМО от состояния
   ноды.

На момент этого PR ни один healthcheck-скрипт вообще не вызывает ros2
CLI (heartbeat-файл вместо ``ros2 topic echo`` — см. issue #2703/#2704
"что сделать"). Эти тесты — guard против регрессии, если кто-то в
будущем снова добавит ros2cli-вызов в healthcheck.

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_healthcheck_shell_lint.py -v -o addopts=""
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import List

import pytest


# unit -> test -> rob_box_perception -> src -> REPO_ROOT (тот же паттерн,
# что test_vision_face_env_namespace.py:31).
_HERE = Path(__file__).resolve()
REPO_ROOT = _HERE.parents[4]

_SOURCE_SETUP_BASH_RE = re.compile(r'\bsource\b.*setup\.bash\b')
_COMMAND_V_ROS2_RE = re.compile(r'\bcommand\s+-v\s+ros2\b')
# Реальный вызов ros2 CLI (не "command -v ros2", не комментарий/слово
# "ros2cli" в прозе): "ros2 <subcommand> ...".
_ROS2_CLI_CALL_RE = re.compile(
    r'(?<![\w./-])ros2\s+(topic|node|service|action|param|daemon|launch|run)\b'
)


def _discover_healthcheck_scripts() -> List[Path]:
    docker_dir = REPO_ROOT / 'docker'
    assert docker_dir.is_dir(), f'docker/ не найден по {docker_dir}'
    return sorted(docker_dir.rglob('healthcheck*.sh'))


def _non_comment_lines(text: str) -> List[tuple[int, str]]:
    """(1-based номер строки, строка) для строк, которые не ЧИСТО комментарии.

    ``# foo`` целиком — пропускаем. Строка с кодом и инлайн-``#``-хвостом
    (например ``foo  # ros2 topic echo ...``) — здесь не отличаем инлайн-
    комментарий от кода нарочно, чтобы не плодить false negatives: если
    паттерн matched только внутри инлайн-комментария, это либо документация
    (не проблема — просто описывает механизм словами, не командной строкой
    с реальным ros2-вызовом), либо реальный код (проблема). Регэкспы ниже
    достаточно специфичны (требуют ``ros2 <subcommand>`` или
    ``command -v ros2``), чтобы не ловить прозу вроде "ros2cli демон".
    """
    lines = []
    for i, raw_line in enumerate(text.splitlines(), start=1):
        stripped = raw_line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        lines.append((i, raw_line))
    return lines


@pytest.fixture(params=_discover_healthcheck_scripts(), ids=lambda p: p.name)
def healthcheck_script(request) -> Path:
    return request.param


def test_at_least_one_healthcheck_script_discovered():
    """Guard: если glob сломается (например, переименуют каталог), тест не
    должен молча стать 'нет тестов -> зелёный CI'.
    """
    scripts = _discover_healthcheck_scripts()
    assert len(scripts) >= 1, (
        f'Не найдено ни одного docker/**/healthcheck*.sh под {REPO_ROOT}. '
        f'Либо здоровые healthcheck-скрипты удалили, либо discovery сломан.'
    )


def test_source_setup_bash_before_command_v_ros2(healthcheck_script: Path):
    """issue #2704: `command -v ros2` НЕ должен идти раньше `source ... setup.bash`.

    ros2 CLI не в базовом PATH образа — если `command -v ros2` проверяется
    ДО того, как workspace сурснут, проверка ВСЕГДА фейлится и скрипт
    падает в fallback (или наоборот — в зависимости от логики branch,
    но в любом случае решение принимается по неверной информации).
    """
    text = healthcheck_script.read_text(encoding='utf-8')
    lines = _non_comment_lines(text)

    source_line_no = None
    command_v_line_no = None
    for line_no, line in lines:
        if source_line_no is None and _SOURCE_SETUP_BASH_RE.search(line):
            source_line_no = line_no
        if command_v_line_no is None and _COMMAND_V_ROS2_RE.search(line):
            command_v_line_no = line_no

    if command_v_line_no is None:
        pytest.skip(
            f'{healthcheck_script.name}: не вызывает `command -v ros2` — '
            f'проверка неприменима (см. issue #2703: heartbeat-файл вместо '
            f'ros2 CLI).'
        )

    assert source_line_no is not None, (
        f'{healthcheck_script.name}:{command_v_line_no}: `command -v ros2` '
        f'вызывается, но нигде не найден `source .../setup.bash` — ros2 CLI '
        f'не в базовом PATH образа, проверка обречена на fallback (issue #2704).'
    )
    assert source_line_no < command_v_line_no, (
        f'{healthcheck_script.name}: `command -v ros2` на строке '
        f'{command_v_line_no} стоит РАНЬШЕ `source .../setup.bash` на строке '
        f'{source_line_no}. Это регрессия issue #2704 (баг из '
        f'healthcheck_face_frame.sh: command -v ros2 до source -> ros2 CLI '
        f'не в PATH -> вечный fallback на pgrep-only, exit 0, независимо от '
        f'реального состояния ноды).'
    )


def test_ros2_cli_calls_use_no_daemon(healthcheck_script: Path):
    """issue #2703 п.1: любой вызов `ros2 topic/node/...` — только с `--no-daemon`.

    Демон ros2cli на Vision Pi падает под rmw_zenoh
    (`xmlrpc.client.Fault: !rclpy.ok()`). Без `--no-daemon` любой такой
    вызов фейлится по ПРИЧИНЕ, НЕ СВЯЗАННОЙ с состоянием проверяемой ноды —
    healthcheck красится "по чужой вине".
    """
    text = healthcheck_script.read_text(encoding='utf-8')
    lines = _non_comment_lines(text)

    offending = []
    for line_no, line in lines:
        if _ROS2_CLI_CALL_RE.search(line) and '--no-daemon' not in line:
            offending.append((line_no, line.strip()))

    assert not offending, (
        f'{healthcheck_script.name}: вызовы ros2 CLI без --no-daemon: '
        + '; '.join(f'{n}: {c!r}' for n, c in offending)
        + '. Демон ros2cli на Vision Pi падает под rmw_zenoh (issue #2703 '
        + 'п.1) — без --no-daemon healthcheck красится по чужой причине.'
    )
