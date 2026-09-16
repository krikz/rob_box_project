"""Regression tests for vision-face ENV-namespace consistency (ADR-0113).

Issue #2655 (F-1 from t_4b487ef8 component review 2026-09-15):

    docker-compose.yaml for ``vision-face`` exported
    ``HAILO_ENABLED=${FACE_HAILO_ENABLED:-false}``,
    but ``start_vision_face.sh`` read ``HAILO_ENABLED`` (no prefix).
    Result: ``FACE_HAILO_ENABLED=true`` in operator's ``.env`` never reached
    the node — vision-face always started in stub-mode, even when the
    operator explicitly opted into real inference (silently-broken
    integration, masks a stub, violates ADR-0018 capability-honest).

These tests guard against the namespace misalignment coming back. They
parse the compose YAML (text scan, not a real loader — keep test
dependencies minimal) and the bash entrypoint, then assert that the
``HAILO_ENABLED`` / ``HEF_PATH`` ENV names that the compose file maps
into the container match the names the bash script reads. If they
diverge again, this test fails BEFORE deploy.

Pure text scan, no docker runtime required.
"""

from __future__ import annotations

import re
from pathlib import Path

# Test is in src/rob_box_perception/test/unit/; REPO_ROOT is 5 levels up:
# unit -> test -> perception -> src -> REPO_ROOT.
REPO_ROOT = Path(__file__).resolve().parents[4]
COMPOSE = REPO_ROOT / 'docker' / 'vision' / 'docker-compose.yaml'
START_FACE = (
    REPO_ROOT
    / 'docker'
    / 'vision'
    / 'scripts'
    / 'vision-hailo'
    / 'start_vision_face.sh'
)
START_HAILO = (
    REPO_ROOT
    / 'docker'
    / 'vision'
    / 'scripts'
    / 'vision-hailo'
    / 'start_vision_hailo.sh'
)


def _vision_face_env_block():
    """Return the text of the ``vision-face:`` service block in compose.

    The block starts at the ``vision-face:`` line and runs through the
    next sibling key (line whose leading indentation is shallower than
    ``vision-face:``'s own indent). We slice on string index rather than
    depending on PyYAML so the test has zero deps.

    Handles both top-level (``vision-face:``) and indented
    (``  vision-face:`` under ``services:``) keys.
    """
    text = COMPOSE.read_text()
    lines = text.splitlines()
    start = None
    start_indent = None
    for i, line in enumerate(lines):
        stripped = line.lstrip()
        if stripped.startswith('vision-face:') and stripped.endswith(':'):
            start = i
            start_indent = len(line) - len(stripped)
            break
    assert start is not None, 'vision-face: service not found in compose'
    # start_indent is now an int (we just found a line).
    assert start_indent is not None

    # Walk forward; the block ends when we hit a line at the SAME or
    # SHALLOWER indent that is not a list-dash continuation.
    block = []
    for line in lines[start + 1:]:
        if not line.strip():
            block.append(line)
            continue
        indent = len(line) - len(line.lstrip())
        if indent <= start_indent and not line.lstrip().startswith('-'):
            break
        block.append(line)
    return '\n'.join(block)


def _compose_env_exports_for_service(service_block):
    """Parse ``- KEY=${SOURCE:-default}`` lines from a compose service block.

    Returns ``{KEY: SOURCE_OR_LITERAL}``. If SOURCE is absent (literal
    default), the value is the literal default string.
    """
    out = {}
    for raw in service_block.splitlines():
        # Compose scalar form: "- KEY=${SRC:-default}"
        m = re.match(
            r'^\s*-\s*([A-Z_][A-Z0-9_]*)=\$\{([A-Z_][A-Z0-9_]*):-(.*?)\}\s*$',
            raw,
        )
        if m:
            out[m.group(1)] = m.group(2)
            continue
        # Literal (no substitution).
        m = re.match(r'^\s*-\s*([A-Z_][A-Z0-9_]*)=(\S*)\s*$', raw)
        if m:
            out[m.group(1)] = m.group(2)
    return out


def _bash_reads(script):
    """Return the set of ENV names a bash script reads via ``${NAME:-...}``.

    Only top-level (non-quoted, non-heredoc) substitutions. We use a
    conservative regex — false positives don't hurt (we only assert that
    the compose-provided names are read).
    """
    text = script.read_text()
    names = set()
    for raw in text.splitlines():
        # Skip pure-comment lines.
        stripped = raw.lstrip()
        if stripped.startswith('#'):
            continue
        # Match ``${NAME:-default}`` (read with default) and ``${NAME}`` (read).
        for m in re.finditer(r'\$\{([A-Z_][A-Z0-9_]*)(:-[^}]*)?\}', raw):
            names.add(m.group(1))
    return names


def test_vision_face_compose_does_not_use_face_namespace():
    """Regression for F-1 (issue #2655).

    ``FACE_HAILO_ENABLED`` / ``FACE_HEF_PATH`` are NOT used by any
    consumer (start_vision_face.sh reads plain ``HAILO_ENABLED`` /
    ``HEF_PATH``). Compose must not invent a ``FACE_*`` namespace that
    the rest of the pipeline ignores.

    We only forbid USE as an ENV substitution (``${FACE_*...}``), not as
    a substring — ``VISION_HAILO_TAG`` in image tags is unrelated.
    """
    block = _vision_face_env_block()
    assert re.search(r'\$\{FACE_HAILO[A-Z0-9_]*', block) is None, (
        'vision-face compose uses ${FACE_HAILO_*} ENV substitution — but '
        'start_vision_face.sh reads HAILO_ENABLED (no prefix). '
        'Reintroducing silently-broken integration (ADR-0113).'
    )
    assert re.search(r'\$\{FACE_HEF[A-Z0-9_]*', block) is None, (
        'vision-face compose uses ${FACE_HEF_*} ENV substitution — but '
        'start_vision_face.sh reads HEF_PATH (no prefix).'
    )


def test_vision_face_compose_maps_hailo_enabled_and_hef_path():
    """Compose MUST map ``HAILO_ENABLED`` / ``HEF_PATH`` for vision-face.

    No ``FACE_`` indirection. Otherwise the bash script never receives
    the operator's intent.
    """
    block = _vision_face_env_block()
    exports = _compose_env_exports_for_service(block)
    assert 'HAILO_ENABLED' in exports, (
        'vision-face compose does not export HAILO_ENABLED — start script '
        'cannot activate real inference.'
    )
    assert 'HEF_PATH' in exports, (
        'vision-face compose does not export HEF_PATH — start script '
        'cannot load HEF model.'
    )


def test_vision_face_and_vision_hailo_use_same_env_names():
    """ADR-0113 §2.2: все vision-* сервисы используют ОБЩИЕ имена.

    ``HAILO_ENABLED`` / ``HEF_PATH``. Namespace-префиксы запрещены.
    """
    hailo_text = START_HAILO.read_text()
    face_text = START_FACE.read_text()

    # Both scripts must read HAILO_ENABLED and HEF_PATH (the canonical
    # contract).
    for name in ('HAILO_ENABLED', 'HEF_PATH'):
        assert name in hailo_text, (
            f'{name} not present in start_vision_hailo.sh — baseline '
            'contract broken.'
        )
        assert name in face_text, (
            f'{name} not present in start_vision_face.sh — divergence '
            'from vision-hailo baseline (ADR-0113 §2.2).'
        )


def test_face_namespace_not_introduced_anywhere_in_compose():
    """Last-line defence: scan the entire compose file for any ENV substitution.

    ``${FACE_HAILO...}`` / ``${FACE_HEF...}`` substitution — that is,
    real USE of the broken namespace in environment mappings. Comments
    explaining the fix are allowed.

    Even one occurrence means someone re-added the broken namespace.
    """
    text = COMPOSE.read_text()
    # Match ``${FACE_HAILO_ENABLED:-...}`` etc. — actual ENV mapping.
    assert re.search(r'\$\{FACE_HAILO[A-Z0-9_]*', text) is None, (
        'FACE_HAILO namespace re-introduced as an ENV substitution in '
        'compose — re-breaks F-1 fix (ADR-0113).'
    )
    assert re.search(r'\$\{FACE_HEF[A-Z0-9_]*', text) is None, (
        'FACE_HEF namespace re-introduced as an ENV substitution in '
        'compose — re-breaks F-1 fix (ADR-0113).'
    )


def test_vision_face_compose_hailo_enabled_no_substitution():
    """Operator must be able to flip real inference with a plain ``HAILO_ENABLED=true``.

    Compose's ``${HAILO_ENABLED:-false}`` is the exact form that makes
    this work.
    """
    block = _vision_face_env_block()
    exports = _compose_env_exports_for_service(block)
    src = exports.get('HAILO_ENABLED', '')
    assert src == 'HAILO_ENABLED', (
        f'HAILO_ENABLED should be sourced directly from HAILO_ENABLED, '
        f'got {src!r}. Operator expects plain HAILO_ENABLED=true to '
        'flip on real inference (matches vision-hailo).'
    )
