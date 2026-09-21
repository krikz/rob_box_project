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
    # The host-only PREFIXED names must never leak into the script itself —
    # start_vision_face.sh only ever sees the generic in-container names;
    # the FACE_ prefix on FACE_HAILO_ENABLED/FACE_HEF_PATH exists solely to
    # disambiguate host .env entries between vision-hailo and vision-face in
    # docker-compose.yaml (ADR-0120 §8). This does NOT forbid a literal
    # 'FACE_' substring appearing for keys whose *generic* container name
    # already starts with face_ (face_store_root, face_privacy_mode,
    # face_identify_threshold, ADR-0123) — those have no vision-hailo
    # equivalent to collide with, so host and container names coincide.
    for host_only in ('FACE_HAILO_ENABLED', 'FACE_HEF_PATH'):
        # Word-boundary match, not substring: 'ARCFACE_HEF_PATH' (ADR-0123)
        # legitimately contains the substring 'FACE_HEF_PATH' without being
        # the host-only variable this test forbids.
        assert not re.search(r'(?<![A-Za-z])' + host_only + r'\b', text), host_only


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


# ---------------------------------------------------------------------------
# ArcFace / FaceStore plumbing (ADR-0123, issue #2599 PR-B)
# ---------------------------------------------------------------------------
# The face-embedding model is a SECOND HEF/inference, independent of
# RetinaFace detection above: it can degrade on its own (no recognition,
# detection still works). It has its own ADR-0120 §8 style host variables
# (FACE_ARCFACE_ENABLED / FACE_ARCFACE_HEF_PATH -> generic ARCFACE_ENABLED /
# ARCFACE_HEF_PATH inside the container), plus FaceStore settings
# (face_store_root, face_privacy_mode, face_identify_threshold) whose host
# and container names coincide because nothing else in the compose file
# reads them.

def test_vision_face_reads_its_own_arcface_host_variables():
    exports = _env_exports(_service_block('vision-face'))
    assert exports.get('ARCFACE_ENABLED', (None,))[0] == 'FACE_ARCFACE_ENABLED'
    assert exports.get('ARCFACE_HEF_PATH', (None,))[0] == 'FACE_ARCFACE_HEF_PATH'


def test_vision_face_arcface_compose_defaults_are_empty():
    """Empty defaults let hailo_models.yaml (vision_face_node) decide."""
    exports = _env_exports(_service_block('vision-face'))
    assert exports['ARCFACE_ENABLED'][1] == ''
    assert exports['ARCFACE_HEF_PATH'][1] == ''


def test_yaml_enables_arcface_with_512dim_embedding():
    text = HAILO_MODELS_YAML.read_text(encoding='utf-8')

    face_after = text.split('\nvision_face_node:', 1)[1]
    face_section = re.split(r'\n(?=[A-Za-z_])', face_after, maxsplit=1)[0]
    assert re.search(r'^\s*arcface_enabled:\s*true\b', face_section, re.M)
    assert re.search(
        r'^\s*arcface_hef_path:\s*\S*arcface_mobilefacenet\S*\.hef',
        face_section,
        re.M,
    )

    models_after = text.split('\n  arcface:', 1)[1]
    arcface_block = re.split(r'\n(?=  [A-Za-z_])', models_after, maxsplit=1)[0]
    assert re.search(r'^\s*enabled:\s*true\b', arcface_block, re.M)
    assert re.search(r'^\s*embedding_dim:\s*512\b', arcface_block, re.M), (
        'embedding_dim must be 512 (measured on the real device, fc1 output '
        'tensor); 128 was a stale model-zoo-page value (issue #2599 PR-B)'
    )


def test_arcface_hef_path_consistent_across_yaml_env_compose_and_script():
    """A grep for the arcface path must tell one coherent story everywhere."""
    path = '/opt/rob_box/models/arcface_mobilefacenet.hef'
    yaml_text = HAILO_MODELS_YAML.read_text(encoding='utf-8')
    env_text = (REPO_ROOT / 'docker' / 'vision' / '.env').read_text(encoding='utf-8')
    manifest_text = (
        REPO_ROOT
        / 'docker' / 'vision' / 'scripts' / 'resource_pack' / 'manifest.yaml'
    ).read_text(encoding='utf-8')
    script_text = START_FACE.read_text(encoding='utf-8')

    assert path in yaml_text
    assert path in env_text
    assert path in manifest_text
    assert 'ARCFACE_HEF_PATH' in script_text


def test_vision_face_mounts_persistent_face_store():
    """ADR-0123 §4: '/data/faces/' must survive container recreation."""
    block = _service_block('vision-face')
    assert re.search(r'-\s*\./data/faces:/data/faces\b', block), (
        'vision-face does not bind-mount a persistent host directory to '
        '/data/faces — the face gallery would be lost on container '
        'recreation (ADR-0123 §4).'
    )


def test_face_privacy_mode_is_wired_end_to_end():
    """face_privacy_mode: workshop must reach yaml, compose and the script."""
    text = HAILO_MODELS_YAML.read_text(encoding='utf-8')
    after = text.split('\nvision_face_node:', 1)[1]
    section = re.split(r'\n(?=[A-Za-z_])', after, maxsplit=1)[0]
    assert re.search(r'^\s*face_privacy_mode:\s*workshop\b', section, re.M)
    assert re.search(r'^\s*face_store_root:\s*/data/faces\b', section, re.M)
    assert re.search(r'^\s*face_identify_threshold:\s*[\d.]+\b', section, re.M)

    exports = _env_exports(_service_block('vision-face'))
    for key in ('FACE_STORE_ROOT', 'FACE_PRIVACY_MODE', 'FACE_IDENTIFY_THRESHOLD'):
        assert exports.get(key, (None,))[0] == key, (
            f'{key}: expected the compose host variable and the container '
            'variable to share the same name (no other service reads it, '
            'so ADR-0120 §8 namespacing is not needed here)'
        )

    script_text = START_FACE.read_text(encoding='utf-8')
    for name in ('FACE_STORE_ROOT', 'FACE_PRIVACY_MODE', 'FACE_IDENTIFY_THRESHOLD'):
        assert name in script_text, name
