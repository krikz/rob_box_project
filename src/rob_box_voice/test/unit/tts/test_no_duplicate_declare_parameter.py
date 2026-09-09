"""Regression test: TTSNode must not declare ROS parameters twice.

Issue #976: after the merge of ``feature/harness-p0-foundation`` the robot's
``tts_node`` crashed on every start with::

    rclpy.exceptions.ParameterAlreadyDeclaredException:
        ('Parameter(s) already declared', ['chunk_max_chars_yandex'])

Root cause: two ``declare_parameter`` blocks for the same chunking params
(``ac0bf908`` issue #933 + ``386176e4`` issue #976) ended up in the same
``TTSNode.__init__``. rclpy raises ``ParameterAlreadyDeclaredException`` on the
second declaration even when the default values match.

This test is deliberately *static*: it greps the node source for
``declare_parameter`` calls and asserts every parameter name is declared
exactly once. It runs without rclpy / ROS2 (pure stdlib), so it guards the
regression on any developer host and in CI.

Prevention rule (from issue #976): before committing changes to
``tts_node.py``, run from the repo root::

    grep -n 'declare_parameter' rob_box_voice/tts_node.py

and make sure no parameter name appears more than once.
"""

from __future__ import annotations

import re
from collections import Counter
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[3]
_NODE_SRC = _PKG_ROOT / 'rob_box_voice' / 'tts_node.py'

# Parameters that were duplicated in the #976 regression.
_CHUNK_PARAMS = {
    'chunk_max_chars_yandex',
    'chunk_max_chars_silero',
    'chunk_max_chars_minimax',
    'chunk_max_retries',
    'chunk_min_chars',
}


def _declared_parameter_names() -> list[str]:
    """Extract every parameter name passed to declare_parameter in tts_node.py.

    Handles both single-line calls::

        self.declare_parameter("chunk_max_retries", DEFAULT_MAX_RETRIES)

    and multi-line calls::

        self.declare_parameter(
            "chunk_max_chars_yandex", CHUNK_LIMITS["yandex_grpc_v3"]
        )
    """
    src = _NODE_SRC.read_text(encoding='utf-8')
    return re.findall(
        r'declare_parameter\s*\(\s*["\']([^"\']+)["\']',
        src,
    )


def test_tts_node_source_exists() -> None:
    """Guard: the node source file exists at the expected location."""
    assert _NODE_SRC.exists(), f'tts_node.py not found at {_NODE_SRC}'


def test_no_duplicate_declare_parameter() -> None:
    """Every ROS parameter must be declared exactly once (issue #976)."""
    names = _declared_parameter_names()
    assert names, 'no declare_parameter calls found - source changed shape?'

    counts = Counter(names)
    duplicates = {name: n for name, n in counts.items() if n > 1}
    assert not duplicates, (
        'Duplicate declare_parameter in tts_node.py - rclpy raises '
        f'ParameterAlreadyDeclaredException on node init: {duplicates}'
    )


def test_chunk_params_declared_once() -> None:
    """The exact params from the #976 traceback are single-declared."""
    counts = Counter(_declared_parameter_names())
    for param in sorted(_CHUNK_PARAMS):
        assert counts[param] == 1, (
            f'{param} declared {counts.get(param, 0)} times '
            '(must be exactly 1, see issue #976)'
        )


def test_chunk_params_have_defaults_in_chunk_limits() -> None:
    """Guard: the chunk params still exist in the node source at all."""
    names = set(_declared_parameter_names())
    assert _CHUNK_PARAMS <= names, (
        f'missing chunk params: {sorted(_CHUNK_PARAMS - names)}'
    )
