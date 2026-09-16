"""Regression tests for vision-face ENV wiring (ADR-0120 §8).

History:

* #2660 replaced ``HAILO_ENABLED=${FACE_HAILO_ENABLED:-false}`` with the
  shared ``HAILO_ENABLED=${HAILO_ENABLED:-false}`` / ``HEF_PATH=${HEF_PATH:-}``,
  believing the ``FACE_*`` flag never reached the node. It did — the prefix
  was only on the host variable.
* As a result vision-face received vision-hailo's host ``HEF_PATH``
  (``yolov8n.hef``) instead of RetinaFace (robot, 2026-09-16).

Contract guarded here:

1. vision-face reads its OWN host variables (``FACE_HAILO_ENABLED`` /
   ``FACE_HEF_PATH``) and never vision-hailo's ``HAILO_ENABLED`` / ``HEF_PATH``.
2. Inside the container the names stay ``HAILO_ENABLED`` / ``HEF_PATH`` —
   those are what ``start_vision_face.sh`` reads.
3. Compose defaults are empty, so ``config/hailo_models.yaml`` decides.
4. ``start_vision_face.sh`` applies its own defaults only AFTER the YAML block;
   otherwise an empty ENV becomes an exported ``false`` that beats the YAML.

Pure text scan, no docker runtime required.
"""

from __future__ import annotations

import re
from pathlib import Path

# unit -> test -> perception -> src -> REPO_ROOT.
REPO_ROOT = Path(__file__).resolve().parents[4]
COMPOSE = REPO_ROOT / 'docker' / 'vision' / 'docker-compose.yaml'
SCRIPTS = REPO_ROOT / 'docker' / 'vision' / 'scripts' / 'vision-hailo'
START_FACE = SCRIPTS / 'start_vision_face.sh'
HAILO_MODELS_YAML = REPO_ROOT / 'docker' / 'vision' / 'config' / 'hailo_models.yaml'


def _service_block(name):
    """Return the text of a top-level compose service block (no PyYAML)."""
    lines = COMPOSE.read_text(encoding='utf-8').splitlines()
    start = start_indent = None
    for i, line in enumerate(lines):
        stripped = line.lstrip()
        if stripped == f'{name}:':
            start = i
            start_indent = len(line) - len(stripped)
            break
    assert start is not None, f'{name}: service not found in compose'

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


def _env_exports(block):
    """Parse ``- KEY=${SOURCE:-default}`` lines → ``{KEY: (SOURCE, default)}``."""
    out = {}
    for raw in block.splitlines():
        m = re.match(
            r'^\s*-\s*([A-Z_][A-Z0-9_]*)=\$\{([A-Z_][A-Z0-9_]*):-(.*?)\}\s*$',
            raw,
        )
        if m:
            out[m.group(1)] = (m.group(2), m.group(3))
    return out


def test_vision_face_reads_its_own_host_variables():
    exports = _env_exports(_service_block('vision-face'))
    assert exports.get('HAILO_ENABLED', (None,))[0] == 'FACE_HAILO_ENABLED'
    assert exports.get('HEF_PATH', (None,))[0] == 'FACE_HEF_PATH'


def test_vision_face_does_not_share_vision_hailo_host_variables():
    """The regression: vision-face got vision-hailo's yolov8n.hef."""
    face = _env_exports(_service_block('vision-face'))
    hailo = _env_exports(_service_block('vision-hailo'))
    for key in ('HAILO_ENABLED', 'HEF_PATH'):
        assert face[key][0] != hailo[key][0], (
            f'vision-face and vision-hailo take {key} from the same host '
            f'variable {face[key][0]!r} — the face node would load the '
            'person-detection model (ADR-0120 §8).'
        )


def test_vision_face_compose_defaults_are_empty():
    """Empty defaults let hailo_models.yaml (RetinaFace) decide."""
    exports = _env_exports(_service_block('vision-face'))
    assert exports['HAILO_ENABLED'][1] == ''
    assert exports['HEF_PATH'][1] == ''


def test_yaml_points_face_node_at_retinaface():
    text = HAILO_MODELS_YAML.read_text(encoding='utf-8')
    after = text.split('\nvision_face_node:', 1)[1]
    # the section ends at the next top-level key
    section = re.split(r'\n(?=[A-Za-z_])', after, maxsplit=1)[0]
    assert re.search(r'^\s*hailo_enabled:\s*true\b', section, re.M)
    assert re.search(r'^\s*hef_path:\s*\S*retinaface\S*\.hef', section, re.M)


def test_start_script_reads_container_names():
    text = START_FACE.read_text(encoding='utf-8')
    for name in ('HAILO_ENABLED', 'HEF_PATH'):
        assert re.search(r'\$\{' + name + r'(:-[^}]*)?\}', text), name
    assert 'FACE_' not in text


def test_start_script_defaults_come_after_yaml():
    """An early ``${HAILO_ENABLED:-false}`` exports "false" and beats the YAML."""
    text = START_FACE.read_text(encoding='utf-8')
    yaml_block = text.index('<<\'PY\'')
    for name in ('HAILO_ENABLED', 'HEF_PATH'):
        m = re.search(r'^' + name + r'="\$\{' + name + r':-', text, re.M)
        assert m, f'{name} default not found in start_vision_face.sh'
        assert m.start() > yaml_block, (
            f'{name} default is applied before the YAML block — an empty '
            'ENV from compose would override hailo_models.yaml.'
        )
