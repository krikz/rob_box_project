"""test_compose_music_tweak_inherits.py — подстройка играющего трека
наследует недостающие ручки от последнего трека (issue #2950).

Живой лог 24.09.2026: юзер пожаловался «бас гудит» на только что сыгранную
«in the hall of the mountain king»; модель вызвала
``compose_music(name=<та же тема>, levels='bass=0.3,...')`` без тембров —
тул честно ответил «не хватает lead_synth, bass_synth, pad_synth»
(:meth:`ComposeMusicTool._missing_arrangement_fields`, ADR-0132 §3.3:
тембры при ``name=`` не выводятся автоматически). Модель на втором вызове
ВЫДУМАЛА новые тембры, чтобы пройти валидацию — звучание сменилось
целиком ради одной ручки.

Здесь проверяется: ``compose_music(name=X)`` (и ``preview_arrangement``)
для ТОЙ ЖЕ мелодии, что играл последний успешный трек, наследует
недостающие ручки (тембры, форма, bpm, root/scale, drum_style,
counter/theme_octaves и т.д. — :data:`_TRACK_INHERIT_FIELDS`) из того
трека; явно переданное вызовом всегда побеждает; другая мелодия ведёт
себя как раньше (честная ошибка); партитура называет, что унаследовано.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, Mock, patch

for _mod in [
    "rclpy", "rclpy.node", "rclpy.action", "rclpy.qos", "std_msgs",
    "std_msgs.msg", "geometry_msgs", "geometry_msgs.msg", "nav2_msgs",
    "nav2_msgs.action", "action_msgs", "action_msgs.srv", "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.core import arranger as arranger_mod  # noqa: E402
from rob_box_mcp_tools.core import rtttl_compose  # noqa: E402
from rob_box_mcp_tools.tools.music import (  # noqa: E402
    ComposeMusicTool,
    PreviewArrangementTool,
)

_THEME_RTTTL = "theme:d=4,o=5,b=120:8c,8d,8e,8f,8g,8a,8b,2c6"
_OTHER_RTTTL = "other:d=4,o=5,b=100:8e,8f,8g,8a"

_ARR = dict(lead_synth="blip", bass_synth="dub", pad_synth="warmpad")
_FULL = dict(
    root="D", scale="dorian", drum_style="march", bass_style="root",
    counter="on", theme_octaves="on", **_ARR,
)


def _theme_rec(name="theme"):
    return {"name": name, "title": "Theme", "rtttl": _THEME_RTTTL}


def _other_rec():
    return {"name": "other", "title": "Other Song", "rtttl": _OTHER_RTTTL}


def _tool(mock_node, resolve=None):
    """Один tool + один (Mock) manager — состояние наследования (issue
    #2950) живёт на ``manager.last_track_arrangement``, поэтому оба
    вызова, которые должны его делить, обязаны переиспользовать один и
    тот же manager (как в проде — один ``MusicManager`` на процесс)."""
    mgr = Mock()
    mgr.execute_code = Mock(return_value={"success": True})
    mgr.last_track_arrangement = None
    # preview_arrangement санирует код напрямую (без прогона через
    # execute_code) и спрашивает known_synth_names() — None отключает
    # проверку известных синтов (см. renardo_sanitizer._validate_synth_names),
    # как и для реального MusicManager без загруженного рантайма.
    mgr.known_synth_names = Mock(return_value=None)
    mgr._max_amp = 0.7
    tool = ComposeMusicTool(mock_node, mgr, Mock())
    if resolve is not None:
        tool._resolve_melody = resolve
    tool._melody_alternatives = lambda _n, _t: []
    return tool, mgr


def _spec_kwargs(mock_node, tool, **kwargs):
    """Сыграть и вернуть (результат, kwargs, ушедшие в spec_from_flat)."""
    real_prep = rtttl_compose.melody_to_compose_params
    real_spec = arranger_mod.spec_from_flat
    with patch(
        "rob_box_mcp_tools.tools.music.melody_to_compose_params", wraps=real_prep
    ), patch(
        "rob_box_mcp_tools.tools.music.spec_from_flat", wraps=real_spec
    ) as spec_spy:
        result = tool.execute(**kwargs)
    spec_kwargs = spec_spy.call_args.kwargs if spec_spy.call_args else None
    return result, spec_kwargs


class TestTweakInheritsFromLastTrack:
    def test_only_levels_inherits_synths_form_and_knobs(self, mock_node):
        tool, mgr = _tool(mock_node, resolve=lambda _n, _v: _theme_rec())

        baseline, base_kwargs = _spec_kwargs(mock_node, tool, name="theme", **_FULL)
        assert baseline.success is True, baseline.error

        tweak, tweak_kwargs = _spec_kwargs(
            mock_node, tool, name="theme", levels="bass=0.3,pad=0.8,lead=1.0",
        )
        assert tweak.success is True, tweak.error
        mgr.execute_code.assert_called()

        # Тембры, root/scale, drum_style — унаследованы, не выдуманы заново.
        for field in ("lead_synth", "bass_synth", "pad_synth", "root", "scale"):
            assert tweak_kwargs[field] == base_kwargs[field], field

        # Только ручка levels реально изменилась.
        assert tool.last_score["decisions"].get("levels")
        assert "lead×1" in tool.last_score["decisions"]["levels"]
        assert "bass×0.3" in tool.last_score["decisions"]["levels"]

        # Партитура называет, что унаследовано.
        assert "унаследовано от текущего трека:" in tool.last_score["text"]
        assert "lead_synth=blip" in tool.last_score["text"]

    def test_explicit_value_wins_over_inherited(self, mock_node):
        tool, _mgr = _tool(mock_node, resolve=lambda _n, _v: _theme_rec())

        _base, base_kwargs = _spec_kwargs(mock_node, tool, name="theme", **_FULL)

        tweak, tweak_kwargs = _spec_kwargs(
            mock_node, tool, name="theme", lead_synth="flute",
            levels="bass=0.3",
        )
        assert tweak.success is True, tweak.error
        # Явная ручка вызова — сильнее унаследованной.
        assert tweak_kwargs["lead_synth"] == "flute"
        assert tweak_kwargs["lead_synth"] != base_kwargs["lead_synth"]
        # Остальное по-прежнему унаследовано.
        assert tweak_kwargs["bass_synth"] == base_kwargs["bass_synth"]
        assert tweak_kwargs["pad_synth"] == base_kwargs["pad_synth"]
        # Партитура не приписывает наследованию ручку, которую вызов
        # задал сам явно (lead_synth).
        inherited_line = next(
            line for line in tool.last_score["text"].splitlines()
            if "унаследовано" in line
        )
        assert "lead_synth" not in inherited_line
        assert "bass_synth" in inherited_line

    def test_different_melody_gets_the_honest_missing_fields_error(self, mock_node):
        records = {"theme": _theme_rec(), "other": _other_rec()}
        tool, mgr = _tool(mock_node, resolve=lambda n, _v: records.get(n))

        played = tool.execute(name="theme", **_FULL)
        assert played.success is True, played.error

        result = tool.execute(name="other", levels="bass=0.3")
        assert result.success is False
        assert "не хватает" in result.error
        assert "lead_synth" in result.error and "bass_synth" in result.error

    def test_no_previous_track_still_errors_as_before(self, mock_node):
        """Первый вызов в сеансе — унаследовать нечего, поведение прежнее."""
        tool, _mgr = _tool(mock_node, resolve=lambda _n, _v: _theme_rec())
        result = tool.execute(name="theme", levels="bass=0.3")
        assert result.success is False
        assert "не хватает" in result.error

    def test_preview_arrangement_inherits_from_the_real_last_track(self, mock_node):
        """``preview_arrangement`` держит СВОЙ внутренний ``ComposeMusicTool``
        (см. ``PreviewArrangementTool.__init__``), но делит ``manager`` с
        реальным ``compose_music`` — наследование обязано быть видно и там."""
        tool, mgr = _tool(mock_node, resolve=lambda _n, _v: _theme_rec())
        played = tool.execute(name="theme", **_FULL)
        assert played.success is True, played.error

        rtttl_library = Mock()
        rtttl_library.get.return_value = _theme_rec()
        preview = PreviewArrangementTool(mock_node, mgr, rtttl_library)
        result = preview.execute(name="theme", levels="bass=0.3")
        assert result.success is True, result.error
        assert "унаследовано от текущего трека:" in result.data["score"]
        assert "lead_synth=blip" in result.data["score"]

    def test_preview_does_not_mutate_last_track_state(self, mock_node):
        """Превью read-only (ADR-0132 PR-5) — не переписывает наследуемый
        снимок; следующий compose_music всё ещё наследует от РЕАЛЬНОГО
        последнего сыгранного трека, не от превью."""
        tool, mgr = _tool(mock_node, resolve=lambda _n, _v: _theme_rec())
        played = tool.execute(name="theme", **_FULL)
        assert played.success is True, played.error
        snapshot_before = dict(mgr.last_track_arrangement)

        rtttl_library = Mock()
        rtttl_library.get.return_value = _theme_rec()
        preview = PreviewArrangementTool(mock_node, mgr, rtttl_library)
        preview_result = preview.execute(
            name="theme", lead_synth="flute", bass_synth="fuzz", pad_synth="ambi",
        )
        assert preview_result.success is True, preview_result.error
        assert mgr.last_track_arrangement == snapshot_before

        tweak, tweak_kwargs = _spec_kwargs(
            mock_node, tool, name="theme", levels="bass=0.3",
        )
        assert tweak.success is True, tweak.error
        assert tweak_kwargs["lead_synth"] == "blip"  # не "flute" из превью
