"""Rover LED layout of the face: the one place that knows which panel is which board and where it sits.

Two kinds of knowledge, kept apart:

* **Boards** — logical groups of the ros2leds compositor and their sizes, the LED count of the strip. They are
  *read* from the compositor's own YAML (the file the compositor on the robot reads), not copied into code:
  :data:`GROUP_SIZE`, :data:`N_LEDS`.
* **The face on the boards** — which face panel goes to which group (:data:`GROUP_OF`) and where it lies on the
  canvas of cross-panel effects (:data:`CANVAS_AT`). The compositor does not know this; it lives here.

The YAML is looked up in this order (:func:`compositor_yaml_path`): ``$LED_MATRIX_COMPOSITOR_CONFIG``; the robot's
volume ``/config/led_matrix/led_matrix_compositor.yaml`` (what the compositor reads, volume over install); the
installed ``led_matrix_compositor`` share; the repository copy ``docker/vision/config/led_matrix/`` above this file
(offline tests, CI). No file — import fails with the list of places tried: the face does not guess the boards.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple

import yaml

CONFIG_ENV = 'LED_MATRIX_COMPOSITOR_CONFIG'
VOLUME_YAML = Path('/config/led_matrix/led_matrix_compositor.yaml')
REPO_YAML = Path('docker') / 'vision' / 'config' / 'led_matrix' / 'led_matrix_compositor.yaml'

#: Panel of the face (drawn for a person facing the robot) -> logical group of
#: the ros2leds compositor. The robot's left headlight (wheel_front_left) is on
#: the viewer's RIGHT, so eye_r goes there. Rear lights are drawn as seen from
#: behind: rear_l is the robot's left board. Checked on the robot by the side
#: test frame (``test_pattern.py``); the test pins this table.
GROUP_OF: Mapping[str, str] = {
    'eye_l': 'wheel_front_right',
    'eye_r': 'wheel_front_left',
    'mouth': 'main_display',
    'rear_l': 'wheel_rear_left',
    'rear_r': 'wheel_rear_right',
}
#: Panel -> (x, y) of its top-left LED on the face *canvas*: the one coordinate system of effects that cross
#: panels (a rainbow wave, the katana slash, ``face/effects.py``). The robot unrolled as seen from outside,
#: walking round it from the front to its left side and on to the back: ``eye_l | mouth | eye_r`` (front),
#: then ``rear_l | rear_r`` (back, seen from behind). Gaps are the dark space between boards in LEDs, by eye
#: (not measured on the robot); the mouth hangs 2 rows below the top of the headlights.
CANVAS_AT: Mapping[str, Tuple[int, int]] = {
    'eye_l': (0, 0),
    'mouth': (12, 2),
    'eye_r': (41, 0),
    'rear_l': (61, 0),
    'rear_r': (73, 0),
}


def _share_yaml() -> Optional[Path]:
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:
        return None
    try:
        return Path(get_package_share_directory('led_matrix_compositor')) / 'config' / 'led_matrix_compositor.yaml'
    except PackageNotFoundError:
        return None


def compositor_yaml_path() -> Path:
    """The compositor YAML the boards are read from (see the module doc for the order).

    An explicit ``$LED_MATRIX_COMPOSITOR_CONFIG`` that does not exist is an error, not a reason to look elsewhere."""
    env = os.environ.get(CONFIG_ENV)
    if env:
        if not Path(env).is_file():
            raise FileNotFoundError(f'{CONFIG_ENV}={env}: no such file')
        return Path(env)
    share = _share_yaml()
    candidates = [VOLUME_YAML] + ([share] if share is not None else [])
    candidates += [parent / REPO_YAML for parent in Path(__file__).resolve().parents]
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError('led_matrix_compositor.yaml not found; tried: ' + ', '.join(map(str, candidates)))


def read_boards(path: Path) -> Tuple[Dict[str, Tuple[int, int]], int]:
    """Compositor YAML -> ({group: (width, height)}, LEDs in the strip): the compositor's own arithmetic."""
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    cfg = raw['led_matrix_compositor']['ros__parameters']
    panels = cfg['physical_panels']
    sizes = {}
    for g in cfg['logical_groups']:
        cols, rows = g['arrangement']
        first = panels[g['physical_indices'][0]]
        sizes[g['name']] = (int(first['width']) * cols, int(first['height']) * rows)
    return sizes, sum(int(p['width']) * int(p['height']) for p in panels)


COMPOSITOR_YAML = compositor_yaml_path()
_sizes, N_LEDS = read_boards(COMPOSITOR_YAML)
_missing = sorted(set(GROUP_OF.values()) - set(_sizes))
if _missing:
    raise ValueError(f'{COMPOSITOR_YAML}: no logical group(s) {_missing} for the face panels')
#: (width, height) of every logical group of the face, as the compositor expects them.
GROUP_SIZE: Mapping[str, Tuple[int, int]] = {g: _sizes[g] for g in sorted(set(GROUP_OF.values()))}
#: Panel -> numpy shape (rows, columns) of its array in a ``Frame``.
PANEL_SHAPE: Mapping[str, Tuple[int, int]] = {p: GROUP_SIZE[g][::-1] for p, g in GROUP_OF.items()}

__all__ = ['CANVAS_AT', 'COMPOSITOR_YAML', 'GROUP_OF', 'GROUP_SIZE', 'N_LEDS', 'PANEL_SHAPE', 'compositor_yaml_path',
           'read_boards']
