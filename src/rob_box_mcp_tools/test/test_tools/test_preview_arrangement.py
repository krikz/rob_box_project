"""``preview_arrangement`` (ADR-0132 PR-5): партитура без проигрывания.

Контракт из ADR §3.2/§3.5:
* те же параметры, что у ``compose_music`` (переиспользует
  ``ComposeMusicTool._build_arrangement`` — не копия логики);
* НЕ вызывает ``MusicManager.execute_code`` и не трогает музыкальное
  состояние (``set_form_deadline``/``clear_form_deadline``/
  ``publish_music_state``/``_last_flat``/``last_score`` компоузера);
* для одних и тех же параметров текст партитуры и текст ошибки
  валидации совпадают байт-в-байт с тем, что вернул бы ``compose_music``.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, Mock, patch

import pytest

for _mod in [
    "rclpy", "rclpy.node", "rclpy.action", "rclpy.qos", "std_msgs",
    "std_msgs.msg", "geometry_msgs", "geometry_msgs.msg", "nav2_msgs",
    "nav2_msgs.action", "action_msgs", "action_msgs.srv", "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.music import (  # noqa: E402
    ComposeMusicTool,
    PreviewArrangementTool,
)

from .test_music import _make_manager  # noqa: E402

_ARR = dict(lead_synth="blip", bass_synth="moogbass", pad_synth="strings")


@pytest.fixture(scope="module")
def rtttl_library(tmp_path_factory):
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary

    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("preview") / "lib.db"))


def _tools(mock_node, rtttl_library):
    """Одна пара (compose, preview) на одном реальном ``MusicManager``.

    ``exec`` патчится, как и в остальных тестах ``execute_code`` — сам
    рантайм Renardo/SuperCollider недоступен в юнит-тестах, но санация
    кода (``renardo_sanitizer.sanitize_renando``) прогоняется по-честному
    для обоих тулов на одном и том же коде.
    """
    mgr = _make_manager(sc_running=True, renardo_available=True)
    compose = ComposeMusicTool(mock_node, mgr, rtttl_library)
    preview = PreviewArrangementTool(mock_node, mgr, rtttl_library)
    return compose, preview, mgr


# ---------------------------------------------------------------------------
# Read-only: не исполняет код, не трогает состояние
# ---------------------------------------------------------------------------


def test_preview_does_not_execute_code(mock_node, rtttl_library):
    _compose, preview, mgr = _tools(mock_node, rtttl_library)
    mgr.execute_code = Mock(side_effect=AssertionError("preview must not execute code"))
    with patch("builtins.exec", side_effect=AssertionError("preview must not exec")):
        result = preview.execute(name="tetris", **_ARR)
    assert result.success is True
    mgr.execute_code.assert_not_called()


def test_preview_does_not_touch_music_manager_state(mock_node, rtttl_library):
    _compose, preview, mgr = _tools(mock_node, rtttl_library)
    for attr in ("set_form_deadline", "clear_form_deadline", "set_form_cycle_end"):
        setattr(mgr, attr, Mock(side_effect=AssertionError(f"{attr} must not be called by preview")))
    mock_node.publish_music_state = Mock(
        side_effect=AssertionError("publish_music_state must not be called by preview")
    )
    result = preview.execute(name="tetris", **_ARR)
    assert result.success is True
    mgr.set_form_deadline.assert_not_called()
    mgr.clear_form_deadline.assert_not_called()
    mgr.set_form_cycle_end.assert_not_called()
    mock_node.publish_music_state.assert_not_called()


def test_preview_does_not_change_compose_music_last_state(mock_node, rtttl_library):
    """``preview_arrangement`` не пишет в ``last_score``/``_last_flat``
    ВНУТРЕННЕГО ``ComposeMusicTool`` — только строит и возвращает текст."""
    _compose, preview, _mgr = _tools(mock_node, rtttl_library)
    assert preview._composer.last_score is None
    preview.execute(name="tetris", **_ARR)
    assert preview._composer.last_score is None
    assert preview._composer._last_flat == {}


def test_preview_is_declared_read_only_and_does_not_start_music(mock_node, rtttl_library):
    _compose, preview, _mgr = _tools(mock_node, rtttl_library)
    assert preview.read_only is True
    assert preview.destructive is False
    assert preview.starts_music is False  # дефолт MCPTool — не переопределён


# ---------------------------------------------------------------------------
# Партитура/ошибки совпадают с compose_music
# ---------------------------------------------------------------------------


def test_preview_score_text_matches_compose_for_same_params(mock_node, rtttl_library):
    compose, preview, mgr = _tools(mock_node, rtttl_library)
    with patch("builtins.exec"):
        compose_result = compose.execute(name="tetris", **_ARR)
    preview_result = preview.execute(name="tetris", **_ARR)

    assert compose_result.success is True
    assert preview_result.success is True
    assert preview_result.data["score"] == compose_result.data["score"]
    assert preview_result.data["title"] == compose_result.data["title"]


def test_preview_score_text_matches_compose_with_knobs_and_name(mock_node, rtttl_library):
    compose, preview, mgr = _tools(mock_node, rtttl_library)
    kwargs = dict(
        name="tetris", bass_style="root_fifth", pad_style="sustain",
        counter="off", theme_octaves="off", **_ARR,
    )
    with patch("builtins.exec"):
        compose_result = compose.execute(**kwargs)
    preview_result = preview.execute(**kwargs)

    assert compose_result.success is True
    assert preview_result.success is True
    assert preview_result.data["score"] == compose_result.data["score"]


@pytest.mark.parametrize(
    "bad_kwargs",
    [
        dict(form="not-a-form", **_ARR),
        dict(root="H", **_ARR),
        dict(scale="not-a-scale", **_ARR),
        dict(bpm=999, **_ARR),
        dict(swing=1.5, **_ARR),
    ],
)
def test_preview_validation_errors_match_compose(mock_node, rtttl_library, bad_kwargs):
    compose, preview, _mgr = _tools(mock_node, rtttl_library)
    compose_result = compose.execute(**bad_kwargs)
    preview_result = preview.execute(**bad_kwargs)

    assert compose_result.success is False
    assert preview_result.success is False
    assert preview_result.error == compose_result.error


def test_preview_knob_only_with_name_error_matches_compose(mock_node, rtttl_library):
    """Ручка, действующая только с ``name=`` (ADR-0132 PR-4), без него —
    ошибка, одинаковая у обоих тулов."""
    compose, preview, _mgr = _tools(mock_node, rtttl_library)
    kwargs = dict(bass_style="root", **_ARR)
    compose_result = compose.execute(**kwargs)
    preview_result = preview.execute(**kwargs)

    assert compose_result.success is False
    assert preview_result.success is False
    assert preview_result.error == compose_result.error


def test_preview_missing_arrangement_fields_error_matches_compose(mock_node, rtttl_library):
    compose, preview, _mgr = _tools(mock_node, rtttl_library)
    compose_result = compose.execute(name="tetris")
    preview_result = preview.execute(name="tetris")

    assert compose_result.success is False
    assert preview_result.success is False
    assert preview_result.error == compose_result.error


def test_preview_unknown_melody_error_matches_compose(mock_node):
    """Библиотека пуста (``Mock()``, ``.get()`` возвращает ``Mock`` без
    записи) — оба тула честно отказывают одинаковым текстом, а не находят
    что-то по слабому совпадению (в отличие от реальной RTTTL-библиотеки,
    см. issue #2896)."""
    empty_library = Mock()
    empty_library.get.return_value = None
    mgr = _make_manager(sc_running=True, renardo_available=True)
    compose = ComposeMusicTool(mock_node, mgr, empty_library)
    preview = PreviewArrangementTool(mock_node, mgr, empty_library)

    kwargs = dict(name="definitely not a real melody title xyz123", **_ARR)
    compose_result = compose.execute(**kwargs)
    preview_result = preview.execute(**kwargs)

    assert compose_result.success is False
    assert preview_result.success is False
    assert preview_result.error == compose_result.error
