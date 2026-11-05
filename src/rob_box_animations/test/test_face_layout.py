"""Rover layout of the face (``face/layout.py``): boards are read from the compositor YAML, not kept in code."""
from __future__ import annotations

import importlib
from pathlib import Path

import pytest
import yaml
from rob_box_animations.face import layout as L
from rob_box_animations.face.face import Frame

#: the repository copy (CI runs this package from test_ws/src/, the checkout is above it)
REPO_YAML = next(p / L.REPO_YAML for p in Path(__file__).resolve().parents if (p / L.REPO_YAML).is_file())
DRIVER_YAML = REPO_YAML.with_name('led_matrix_driver.yaml')


def test_offline_the_layout_comes_from_the_repo_copy_of_the_compositor_yaml():
    if L.VOLUME_YAML.is_file():
        pytest.skip('robot volume present: the compositor file wins')
    assert L.COMPOSITOR_YAML.resolve() == REPO_YAML.resolve()


def test_boards_are_the_compositor_groups():
    sizes, n = L.read_boards(REPO_YAML)
    assert sizes == {'main_display': (25, 5), 'wheel_front_left': (8, 8), 'wheel_front_right': (8, 8),
                     'wheel_rear_left': (8, 8), 'wheel_rear_right': (8, 8)}
    assert dict(L.GROUP_SIZE) == sizes and L.N_LEDS == n == 381


def test_strip_length_matches_the_driver():
    driver = yaml.safe_load(DRIVER_YAML.read_text(encoding='utf-8'))
    assert next(iter(driver.values()))['ros__parameters']['num_leds'] == L.N_LEDS


def test_frame_panels_take_their_size_from_the_layout():
    for panel, arr in Frame().panels().items():
        w, h = L.GROUP_SIZE[L.GROUP_OF[panel]]
        assert arr.shape == (h, w, 3), panel


def test_explicit_config_that_is_missing_fails_loudly(monkeypatch, tmp_path):
    monkeypatch.setenv(L.CONFIG_ENV, str(tmp_path / 'nope.yaml'))
    with pytest.raises(FileNotFoundError, match=L.CONFIG_ENV):
        L.compositor_yaml_path()


def test_config_without_a_face_group_fails_at_import(monkeypatch, tmp_path):
    raw = yaml.safe_load(REPO_YAML.read_text(encoding='utf-8'))
    groups = raw['led_matrix_compositor']['ros__parameters']['logical_groups']
    raw['led_matrix_compositor']['ros__parameters']['logical_groups'] = [
        g for g in groups if g['name'] != 'wheel_rear_right']
    bad = tmp_path / 'led_matrix_compositor.yaml'
    bad.write_text(yaml.safe_dump(raw), encoding='utf-8')
    monkeypatch.setenv(L.CONFIG_ENV, str(bad))
    try:
        with pytest.raises(ValueError, match='wheel_rear_right'):
            importlib.reload(L)
    finally:
        monkeypatch.delenv(L.CONFIG_ENV)
        importlib.reload(L)
