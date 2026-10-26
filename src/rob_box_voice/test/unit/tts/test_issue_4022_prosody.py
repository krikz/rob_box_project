"""#4022 — скорость и тон голоса «на лету»: звучащий голос меняется, а не только параметр.

Живой прогон 23.10 (акт 4, n409–n411): ``set_speed`` писал ``yandex_speed`` при звучащем MiniMax, а MiniMax читает
``minimax_speed``, который нода на лету не принимала; ``set_pitch`` писал ``pitch_shift``, слышный только при
``chipmunk_mode`` (в проде выключен). Здесь — поведение таблицы ``tts_prosody`` и применение её в ``tts_node``:
принятый параметр доезжает до ``TTSSettings`` MiniMax.
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_prosody as pm  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402

# ── таблица: провайдер → ось ─────────────────────────────────────────────────


@pytest.mark.parametrize(
    "provider, param",
    [
        ("minimax", "minimax_speed"),
        ("yandex", "yandex_speed"),
        ("silero", None),
        ("", None),
        ("unknown", None),
    ],
)
def test_speed_axis_is_the_active_providers_own_param(provider, param):
    axis = pm.speed_axis(provider)
    assert (axis.param if axis else None) == param


@pytest.mark.parametrize(
    "provider, chipmunk, applies",
    [
        ("minimax", False, True),  # голосовой параметр тона MiniMax
        ("yandex", False, False),  # без chipmunk_mode тона у Yandex нет
        ("silero", False, False),
        ("yandex", True, True),  # chipmunk_mode передискретизирует любой провайдер
        ("silero", True, True),
    ],
)
def test_pitch_is_offered_only_where_it_is_audible(provider, chipmunk, applies):
    axis = pm.pitch_axis(provider, chipmunk)
    assert (axis is not None) is applies
    assert axis is None or axis.param == "pitch_shift"


def test_live_axes_cover_every_param_the_tools_can_write():
    written = {axis.param for axis in pm.SPEED_AXES.values()} | {pm.PITCH_AXIS.param}
    assert (
        set(pm.LIVE_AXES) == written == {"minimax_speed", "yandex_speed", "pitch_shift"}
    )


# ── шаг по оси ───────────────────────────────────────────────────────────────


def test_step_moves_by_a_fifth_and_stays_inside_the_axis():
    axis = pm.speed_axis("minimax")
    assert pm.target_value(axis, +1, 1.0) == 1.2
    assert pm.target_value(axis, -1, 1.0) == 0.8
    assert pm.target_value(axis, -1, 0.8) == 0.6  # без хвоста 0.6000000000000001
    assert pm.target_value(axis, +1, 1.9) == 2.0
    assert pm.target_value(axis, +1, 2.0) == 2.0
    assert pm.target_value(axis, -1, 0.6) == 0.5
    assert pm.target_value(axis, -1, 0.5) == 0.5
    assert pm.target_value(axis, 0, 1.6) == axis.normal == 1.0


# ── тон: коэффициент → полутона MiniMax ──────────────────────────────────────


@pytest.mark.parametrize(
    "ratio, semitones",
    [
        (1.0, 0),
        (1.2, 3),
        (1.4, 6),
        (2.0, 12),
        (0.8, -4),
        (0.6, -9),
        (0.5, -12),
        (3.0, 12),
        (0.1, -12),
        (0.0, 0),
        (-1.0, 0),
    ],
)
def test_pitch_ratio_maps_onto_minimax_semitones_within_the_api_range(ratio, semitones):
    assert pm.pitch_semitones(ratio) == semitones


def test_minimax_pitch_is_omitted_until_someone_asks_for_a_shift():
    assert (
        pm.minimax_pitch(None, 1.0, False) is None
    )  # поведение до #4022: поле не уходит в API


def test_minimax_pitch_adds_the_live_shift_to_the_configured_one():
    assert pm.minimax_pitch(None, 1.2, False) == 3
    assert pm.minimax_pitch(2, 1.2, False) == 5
    assert pm.minimax_pitch(2, 1.0, False) == 2  # конфиг без живого сдвига — как и было
    assert pm.minimax_pitch(10, 1.4, False) == 12  # предел API


def test_chipmunk_already_used_the_ratio_so_minimax_does_not_apply_it_twice():
    assert pm.minimax_pitch(None, 1.4, True) is None
    assert pm.minimax_pitch(2, 1.4, True) == 2


@pytest.mark.parametrize(
    "count, text",
    [
        (3, "+3 полутона"),
        (-4, "-4 полутона"),
        (1, "+1 полутон"),
        (5, "+5 полутонов"),
        (12, "+12 полутонов"),
        (11, "+11 полутонов"),
        (0, "+0 полутонов"),
    ],
)
def test_semitones_are_named_in_russian(count, text):
    assert pm.semitones_ru(count) == text


# ── нода принимает параметры на лету ─────────────────────────────────────────


def _param(name, value):
    p = MagicMock()
    p.name = name
    p.value = value
    return p


def _node(**attrs):
    node = object.__new__(TTSNode)
    node.logger = MagicMock()
    node.get_logger = lambda: node.logger
    node.minimax_speed = 1.0
    node.yandex_speed = 1.0
    node.pitch_shift = 1.0
    node.chipmunk_mode = False
    # то, что читает _minimax_settings
    node.minimax_voice = "Russian_ReliableMan"
    node.minimax_model = "speech-02-hd"
    node.minimax_language = "ru"
    node.minimax_sample_rate = 32000
    node.minimax_format = None
    node.minimax_emotion = "neutral"
    node.minimax_pitch_raw = ""
    node.minimax_volume_raw = ""
    node.minimax_pronunciation_dict_raw = ""
    for name, value in attrs.items():
        setattr(node, name, value)
    return node


@pytest.mark.parametrize(
    "name, value", [("minimax_speed", 0.8), ("yandex_speed", 1.4), ("pitch_shift", 1.2)]
)
def test_every_live_axis_param_reaches_the_node_attribute(name, value):
    node = _node()
    result = node.parameters_callback([_param(name, value)])
    assert result.successful is True
    assert getattr(node, name) == value


def test_minimax_speed_set_live_is_what_the_next_minimax_synthesis_uses():
    """Корень #4022: MiniMax читает ``minimax_speed``, и нода обязана принимать его на лету."""
    node = _node()
    node.parameters_callback([_param("minimax_speed", 0.8)])
    settings = TTSNode._minimax_settings(node, None, None, None)
    assert settings.speed == 0.8


def test_yandex_speed_does_not_leak_into_minimax_synthesis():
    node = _node()
    node.parameters_callback([_param("yandex_speed", 0.8)])
    assert TTSNode._minimax_settings(node, None, None, None).speed == 1.0


def test_ssml_rate_of_the_replica_still_beats_the_node_speed():
    node = _node(minimax_speed=0.8)
    assert TTSNode._minimax_settings(node, {"rate": 1.5}, None, None).speed == 1.5


def test_pitch_shift_set_live_reaches_minimax_voice_setting_without_chipmunk():
    node = _node()
    node.parameters_callback([_param("pitch_shift", 1.2)])
    assert TTSNode._minimax_settings(node, None, None, None).pitch == 3


def test_pitch_shift_is_a_noop_for_minimax_until_set_and_after_it_returns_to_normal():
    node = _node()
    assert TTSNode._minimax_settings(node, None, None, None).pitch is None
    node.parameters_callback([_param("pitch_shift", 0.8)])
    assert TTSNode._minimax_settings(node, None, None, None).pitch == -4
    node.parameters_callback([_param("pitch_shift", 1.0)])
    assert TTSNode._minimax_settings(node, None, None, None).pitch is None


def test_pitch_shift_with_chipmunk_is_left_to_the_resampler():
    node = _node(chipmunk_mode=True, pitch_shift=1.4)
    assert TTSNode._minimax_settings(node, None, None, None).pitch is None


def test_configured_minimax_pitch_still_applies_and_live_shift_adds_to_it():
    node = _node(minimax_pitch_raw="2")
    assert TTSNode._minimax_settings(node, None, None, None).pitch == 2
    node.parameters_callback([_param("pitch_shift", 1.2)])
    assert TTSNode._minimax_settings(node, None, None, None).pitch == 5
