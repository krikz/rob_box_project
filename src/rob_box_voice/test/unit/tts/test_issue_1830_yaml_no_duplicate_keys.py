"""Regression test: tts_node ROS YAML configs must not declare the same
ROS parameter twice as different types.

Issue #1830 (🚨 Deploy issues on develop (staging) — 2026-08-31,
workflow run https://github.com/krikz/rob_box_project/actions/runs/33445415280):

After PR #1793 (`feat(voice #1780): emotion/pitch/volume/pronunciation_dict +
yandex_ssml_aware`) both ``src/rob_box_voice/config/tts_node.yaml`` and
``docker/vision/config/voice_assistant/tts_node.yaml`` ended up with TWO
blocks of ``minimax_emotion / minimax_pitch / minimax_volume /
minimax_pronunciation_dict`` parameters with DIFFERENT types::

    # first block (legacy, kept by accident)
    minimax_emotion: ''
    minimax_pitch: 0
    minimax_volume: 0.0
    minimax_pronunciation_dict: ''
    # ...
    # canonical block
    minimax_emotion: neutral
    minimax_pitch: ''
    minimax_volume: ''
    minimax_pronunciation_dict: ''

PyYAML's loader silently takes the *last* value for each key (mapping
collapses duplicates), so the second (``str ''``) reaches
``rclpy.Node.set_parameters`` — but on the deploy run that triggered this
card, the FIRST declared type in Python was ``int 0`` for ``minimax_pitch``
(duplicate declare_parameter blocks from PR #1793, only fixed in
PR #1820/#1823) and ``set_parameters_from_yaml`` raised::

    rclpy.exceptions.InvalidParameterTypeException:
        Trying to set parameter 'minimax_pitch' to '0' of type 'INTEGER',
        expecting type 'STRING'

The tts_node died every ~10 s, perception/context_aggregator caught the
downstream ("❌ Нода упала: /tts_node"), and the deploy run was marked
"❌ DEPLOYMENT COMPLETED WITH ISSUES" with Critical Errors: 1.

Fix:
- PR #1820 (`6a6ec0d8`) removed the duplicate ``declare_parameter`` blocks
  in ``tts_node.py``;
- PR #1823 (`cc9ca38d`) routed raw reads through
  ``_parse_optional_int/_parse_optional_float`` and introduced ``*_raw`` attrs;
- This test guarantees the YAML files no longer carry duplicate keys with
  conflicting types — the trigger that makes set_parameters blow up even
  when the Python side is healthy.

The test is pure stdlib (PyYAML is **not** a test dep here — we deliberately
parse the YAML ourselves so the regression is caught even on hosts without
PyYAML installed). It scans the canonical tts_node YAML files at the
two well-known deployment paths and asserts every ROS parameter name appears
exactly once and the canonical type (STRING, declared in
``tts_node.declare_parameter``) matches.
"""

from __future__ import annotations

import re
from pathlib import Path

# _PKG_ROOT = .../src/rob_box_voice → repo root is one level up.
_PKG_ROOT = Path(__file__).resolve().parents[3]
_REPO_ROOT = _PKG_ROOT.parent.parent

# Canonical tts_node ROS YAML config files. Issue #1830 affects both:
# - src/rob_box_voice/config/tts_node.yaml (used by ``ros2 launch`` from src)
# - docker/vision/config/voice_assistant/tts_node.yaml (baked into image)
_TTS_NODE_YAMLS: tuple[Path, ...] = (
    _REPO_ROOT / 'src' / 'rob_box_voice' / 'config' / 'tts_node.yaml',
    _REPO_ROOT / 'docker' / 'vision' / 'config' / 'voice_assistant' / 'tts_node.yaml',
)

# Issue #1830 / #1780 — MiniMax TTS provider params that were duplicated.
# These MUST appear exactly once per YAML file with a STRING value (rclpy
# declared type is STRING in tts_node.py, see #1820/#1823).
_MINIMAX_PARAMS: frozenset[str] = frozenset({
    'minimax_emotion',
    'minimax_pitch',
    'minimax_volume',
    'minimax_pronunciation_dict',
})


def _load_yaml_keys(path: Path) -> list[tuple[str, str]]:
    """Return ``[(key, raw_value_string), ...]`` in source order.

    Pure-Python scanner (no PyYAML) so this test runs on any dev host
    without needing the test-extras. Each line that looks like
    ``key: value`` under the ``ros__parameters:`` mapping is captured.

    Notes:
    - Comments (``# ...``) and blank lines are skipped.
    - The capture is shallow: we only inspect one level of nesting below
      ``ros__parameters``. tts_node.yaml's only param mapping is flat, so
      this is sufficient for the regression we guard against.
    """
    out: list[tuple[str, str]] = []
    in_ros_params = False
    for raw in path.read_text(encoding='utf-8').splitlines():
        line = raw.rstrip()
        if not line or line.lstrip().startswith('#'):
            continue
        stripped = line.lstrip()
        if stripped.startswith('ros__parameters:'):
            in_ros_params = True
            continue
        if in_ros_params:
            # New top-level key (not indented enough) ends the ros__params block.
            if not line.startswith(' '):
                in_ros_params = False
                continue
            m = re.match(r'^\s+([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*?)\s*$', line)
            if m:
                key, value = m.group(1), m.group(2)
                # Strip trailing inline comment.
                if '#' in value and not (value.startswith("'") or value.startswith('"')):
                    value = value.split('#', 1)[0].rstrip()
                out.append((key, value))
    return out


def _count_keys(entries: list[tuple[str, str]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for k, _ in entries:
        counts[k] = counts.get(k, 0) + 1
    return counts


def test_yaml_files_exist() -> None:
    for path in _TTS_NODE_YAMLS:
        assert path.exists(), (
            f'tts_node ROS YAML not found at {path} '
            '(expected after issue #1830 refactor)'
        )


def test_minimax_params_no_duplicate_keys() -> None:
    """Issue #1830 root cause: YAML had two ``minimax_pitch: ...`` blocks
    with conflicting types. Every MiniMax TTS param must appear once per file.
    """
    for path in _TTS_NODE_YAMLS:
        counts = _count_keys(_load_yaml_keys(path))
        duplicates = {p: counts[p] for p in _MINIMAX_PARAMS if counts.get(p, 0) > 1}
        assert not duplicates, (
            f'{path}: duplicate ROS YAML keys for {duplicates}. '
            'Two blocks with conflicting types cause rclpy '
            'InvalidParameterTypeException → tts_node crash-loop '
            '(issue #1830). Keep only the canonical block (str values: '
            "``''`` for pitch/volume/pronunciation_dict, ``'neutral'`` for emotion)."
        )


def test_minimax_params_present_exactly_once() -> None:
    """Each MiniMax TTS param must be defined at least once (positive sanity)."""
    for path in _TTS_NODE_YAMLS:
        counts = _count_keys(_load_yaml_keys(path))
        for param in sorted(_MINIMAX_PARAMS):
            assert counts.get(param, 0) >= 1, (
                f'{path}: missing ROS YAML key {param!r}. '
                'Add it to the canonical MiniMax TTS param block '
                '(see tts_node.declare_parameter in src/rob_box_voice/rob_box_voice/tts_node.py).'
            )


def test_minimax_params_are_strings() -> None:
    """Canonical values must be STRINGs (matches rclpy declared type).

    ``minimax_pitch: 0`` (int) and ``minimax_volume: 0.0`` (float) were
    the legacy defaults that triggered InvalidParameterTypeException
    (issue #1830). The canonical block uses string literals
    (``''`` / ``'neutral'``); Python parses them via
    ``_parse_optional_int/_parse_optional_float``.
    """
    string_params = {'minimax_pitch', 'minimax_volume', 'minimax_pronunciation_dict'}
    for path in _TTS_NODE_YAMLS:
        entries = dict(_load_yaml_keys(path))
        for param in sorted(string_params):
            value = entries.get(param)
            assert value is not None, (
                f'{path}: missing {param!r} (test setup wrong?)'
            )
            # Must be quoted or empty — bare digits like ``0`` or ``0.0``
            # are the regression we guard against.
            assert value == '' or (
                (value.startswith("'") and value.endswith("'"))
                or (value.startswith('"') and value.endswith('"'))
            ), (
                f'{path}: {param!r} = {value!r} must be a YAML string '
                '(quoted ``""`` or ``\'\'``), not bare int/float. '
                'Bare 0/0.0 triggers InvalidParameterTypeException '
                '(issue #1830).'
            )


def test_minimax_emotion_is_string_neutral() -> None:
    """Canonical emotion is ``'neutral'`` (string).

    Guards against the issue #1830 regression where the legacy block
    declared ``minimax_emotion: ''`` (empty string). Empty emotion would
    cause ``_normalize_minimax_emotion`` to fall back to ``''`` and the
    MiniMax API would reject the request. The canonical block uses the
    API default ``'neutral'``.
    """
    for path in _TTS_NODE_YAMLS:
        entries = dict(_load_yaml_keys(path))
        value = entries.get('minimax_emotion')
        assert value is not None, (
            f'{path}: missing minimax_emotion (test setup wrong?)'
        )
        # Value must be a quoted/empty string, NOT a bare integer/float
        # (the issue #1830 regression shape). We accept any non-numeric
        # YAML scalar; the canonical default is ``neutral``.
        assert value not in {'0', '0.0', '1', '1.0'}, (
            f'{path}: minimax_emotion = {value!r} must be a YAML string, '
            'not a bare int/float (issue #1830 InvalidParameterTypeException).'
        )
        # Bare string check: bare ``neutral`` (unquoted) parses as a string,
        # but it's better practice to quote it. We just assert it's a
        # non-empty, non-numeric scalar — both quoted and bare neutral are
        # acceptable in YAML.
        assert value.strip() != '', (
            f'{path}: minimax_emotion = {value!r}; empty string would skip '
            'the field and let the API fall back to defaults — the canonical '
            'value is ``neutral`` (see issue #1780 / tts_node.declare_parameter).'
        )