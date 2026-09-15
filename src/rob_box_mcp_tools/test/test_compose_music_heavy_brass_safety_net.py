"""Safety net для тяжёлых брасс-лидов (live 2026-09-15, Григ).

Тест ``ComposeMusicTool._heavy_brass_safety_net``: статический метод,
который для ``name=`` (известная RTTTL-тема) + ``lead_synth`` ∈
``HEAVY_BRASS_LEAD_SYNTHS`` (``{"imperialbrass"}``) + дефолтные
``counter_synth``/``theme_octaves`` отключает контрмелодию и удвоение
октавой, чтобы из SynthDef'а с длинным envelope release не получалась
«эхо-каша» из 3-х голосов.

Защитный сброс НЕ перетирает явный выбор модели: если модель сама
передала ``counter_synth='something'`` или ``theme_octaves=False``,
возвращаются исходные значения.

См. https://github.com/krikz/rob_box_project/issues/2569 (live log
``/memories/repo/dj-imperialbrass-echo-2026-09-15.md``).
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock

import pytest

# ``rob_box_mcp_tools.tools.music`` импортирует ``rclpy`` через цепочку
# ``tools/__init__.py`` → ``navigation`` — мокаем ROS2 как в
# ``test_compose_music_arranger_sync.py``, чтобы не тянуть рантайм.
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "std_msgs",
    "std_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.srv",
    "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.music import ComposeMusicTool  # noqa: E402


# ----------------------------------------------------------------------
# Простые проверки дефолтного поведения
# ----------------------------------------------------------------------


def test_safety_net_off_when_no_name() -> None:
    """Без ``name`` (свободная композиция) safety net не трогает значения."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name=None,
        lead_synth="imperialbrass",
        counter_synth=None,
        theme_octaves=True,
    )
    assert counter is None
    assert octaves is True
    assert did_override is False


def test_safety_net_off_when_lead_synth_not_heavy_brass() -> None:
    """``lead_synth='brass'`` (нормальный брасс-соло) — safety net не сработает."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth="brass",
        counter_synth=None,
        theme_octaves=True,
    )
    assert counter is None
    assert octaves is True
    assert did_override is False


def test_safety_net_off_when_lead_synth_is_piano() -> None:
    """``lead_synth='pianovel'`` — вообще не брасс, safety net не сработает."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth="pianovel",
        counter_synth=None,
        theme_octaves=True,
    )
    assert counter is None
    assert octaves is True
    assert did_override is False


# ----------------------------------------------------------------------
# Главный кейс: imperialbrass + MountainKing + дефолты → отключаем
# ----------------------------------------------------------------------


def test_safety_net_disables_counter_for_imperialbrass_default() -> None:
    """Главный репрос live: imperialbrass + name + дефолтный counter_synth."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth="imperialbrass",
        counter_synth=None,
        theme_octaves=True,
    )
    assert counter == "none"
    assert octaves is False
    assert did_override is True


def test_safety_net_disables_counter_when_empty_string() -> None:
    """``counter_synth=''`` (пустая строка) трактуется как дефолт → отключаем."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth="imperialbrass",
        counter_synth="",
        theme_octaves=True,
    )
    assert counter == "none"
    assert did_override is True


def test_safety_net_disables_octaves_for_imperialbrass_default() -> None:
    """``theme_octaves=True`` (дефолт) → отключаем при imperialbrass + name."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth="imperialbrass",
        counter_synth=None,
        theme_octaves=True,
    )
    assert octaves is False
    assert did_override is True


# ----------------------------------------------------------------------
# Уважение к явному выбору модели
# ----------------------------------------------------------------------


def test_safety_net_respects_explicit_counter_synth() -> None:
    """Модель явно попросила ``counter_synth='strings'`` — НЕ отключаем."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth="imperialbrass",
        counter_synth="strings",
        theme_octaves=True,
    )
    assert counter == "strings"
    assert did_override is True  # octaves всё равно override


def test_safety_net_respects_explicit_theme_octaves_false() -> None:
    """Модель явно попросила ``theme_octaves=False`` — counter всё равно override."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth="imperialbrass",
        counter_synth=None,
        theme_octaves=False,
    )
    assert counter == "none"
    assert octaves is False
    assert did_override is True


def test_safety_net_respects_both_explicit_overrides() -> None:
    """Оба явные — safety net ничего не делает."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth="imperialbrass",
        counter_synth="strings",
        theme_octaves=False,
    )
    assert counter == "strings"
    assert octaves is False
    assert did_override is False


# ----------------------------------------------------------------------
# Регистр и пробелы — устойчивость
# ----------------------------------------------------------------------


@pytest.mark.parametrize(
    "lead_synth_variant",
    ["imperialbrass", "ImperialBrass", "IMPERIALBRASS", "  imperialbrass  "],
)
def test_safety_net_handles_lead_synth_case_and_whitespace(
    lead_synth_variant: str,
) -> None:
    """Регистр/пробелы в lead_synth — safety net должен сработать."""
    counter, octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth=lead_synth_variant,
        counter_synth=None,
        theme_octaves=True,
    )
    assert counter == "none"
    assert octaves is False
    assert did_override is True


# ----------------------------------------------------------------------
# Контракт: HEAVY_BRASS_LEAD_SYNTHS — frozenset, не dict/list
# ----------------------------------------------------------------------


def test_heavy_brass_set_is_immutable_frozenset() -> None:
    """``HEAVY_BRASS_LEAD_SYNTHS`` обязан быть ``frozenset`` (мутирующий set
    мог бы рассинхронизировать hot-path)."""
    assert isinstance(
        ComposeMusicTool.HEAVY_BRASS_LEAD_SYNTHS, frozenset
    )
    assert "imperialbrass" in ComposeMusicTool.HEAVY_BRASS_LEAD_SYNTHS