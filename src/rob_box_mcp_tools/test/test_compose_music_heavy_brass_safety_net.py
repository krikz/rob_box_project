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
    assert counter == "off"
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
    assert counter == "off"
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
    assert counter == "off"
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
    assert counter == "off"
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


# ----------------------------------------------------------------------
# Issue #2836: literal 'none' — не название синта, безопас-нет больше её
# не подставляет; explicit 'none'/'off' от самой модели тоже распознаются
# как «дефолт» (нечего перетирать, результат — отключённый counter — уже
# запрошен).
# ----------------------------------------------------------------------


def test_safety_net_never_returns_the_literal_string_none() -> None:
    """Возвращаемое значение при срабатывании — не строка 'none'.

    'none' не является именем SynthDef в scsynth: до фикса #2836 это
    значение доходило до аранжировщика как настоящий синт и рендерилось
    ``d3 >> none([...])``, что ``renardo_sanitizer`` отклонял («Синта
    'none' не существует»).
    """
    counter, _octaves, _did_override = (
        ComposeMusicTool._heavy_brass_safety_net(
            name="MountainKing",
            lead_synth="imperialbrass",
            counter_synth=None,
            theme_octaves=True,
        )
    )
    assert counter != "none"


@pytest.mark.parametrize(
    "explicit_disable", ["none", "None", "NONE", "off", " null "]
)
def test_safety_net_treats_model_disable_words_as_default(
    explicit_disable: str,
) -> None:
    """Модель сама пишет counter_synth='none'/'off'/'null' — тоже «дефолт».

    Явное слово-отключение и «не задано вовсе» просят один и тот же
    результат (второго голоса нет), поэтому безопас-нет считает их
    равнозначными, а не «явным реальным синтом, который нельзя трогать».
    """
    counter, _octaves, did_override = ComposeMusicTool._heavy_brass_safety_net(
        name="MountainKing",
        lead_synth="imperialbrass",
        counter_synth=explicit_disable,
        theme_octaves=True,
    )
    assert counter != "none"
    assert did_override is True


# ----------------------------------------------------------------------
# Интеграционный репрод: safety-net → spec_from_flat → render →
# renardo_sanitizer, весь путь compose_music для тяжёлого брасс-лида.
# ----------------------------------------------------------------------


def test_imperialbrass_default_end_to_end_has_no_none_synth_and_passes_sanitizer() -> None:  # noqa: E501
    """Live-репрод issue #2836: 'in the hall of the mountain king' +
    imperialbrass + дефолтные counter_synth/theme_octaves.

    До фикса: safety net подставлял counter_synth='none' → аранжировщик
    строил ``d3 >> none([...])`` → renardo_sanitizer отклонял («Синта
    'none' не существует», см. raw-лог в issue). После фикса: код вообще
    не содержит вызова ``none(``, второй голос просто не добавляется, и
    полный pipeline очистки проходит без ошибок.
    """
    from rob_box_mcp_tools.core import renardo_sanitizer
    from rob_box_mcp_tools.core.arranger import render, spec_from_flat
    from rob_box_mcp_tools.core.harmonize import harmonize

    # Плотная тема — частые атаки шестнадцатыми, как в "In the Hall of the
    # Mountain King" (быстрый безостановочный бег темы вверх-вниз).
    notes = [(72 + (i % 5), 0.25) for i in range(32)]
    harmony = harmonize(notes, bpm=160, root="A", scale="minor")
    assert harmony.dense is True, (
        "тема должна быть плотной — иначе counter и так no-op"
    )

    lead_synth = "imperialbrass"
    counter_synth, theme_octaves, did_override = (
        ComposeMusicTool._heavy_brass_safety_net(
            name="in the hall of the mountain king",
            lead_synth=lead_synth,
            counter_synth=None,
            theme_octaves=True,
        )
    )
    assert did_override is True

    spec = spec_from_flat(
        harmony=harmony,
        bpm=harmony.bpm,
        root=harmony.root,
        scale=harmony.scale,
        form="arc",
        lead_synth=lead_synth,
        bass_synth="dub",
        pad_synth="warmpad",
        counter_synth=counter_synth,
        theme_octaves=theme_octaves,
    )
    code = render(spec)

    assert "none(" not in code, code
    assert not any(
        line.startswith("d3 >>") for line in code.splitlines()
    ), "counter должен быть отключён, а не звучать синтом 'none'"

    known_synths = frozenset({"imperialbrass", "dub", "warmpad"})
    result = renardo_sanitizer.sanitize_renando(
        code, max_amp=0.85, known_synths=known_synths
    )
    assert result.quality_errors == (), result.quality_errors
    assert result.security_error is None


def test_explicit_counter_synth_none_drops_the_counter_layer() -> None:
    """Модель сама пишет counter_synth='none' (без safety net) — тоже
    отключает второй голос, а не рендерит его синтом 'none'."""
    from rob_box_mcp_tools.core.arranger import render, spec_from_flat
    from rob_box_mcp_tools.core.harmonize import harmonize

    notes = [(72 + (i % 5), 0.25) for i in range(32)]
    harmony = harmonize(notes, bpm=120, root="C", scale="major")
    assert harmony.dense is True

    code = render(spec_from_flat(
        harmony=harmony, bpm=harmony.bpm, root=harmony.root,
        scale=harmony.scale, form="arc", lead_synth="pluck",
        bass_synth="dub", pad_synth="warmpad", counter_synth="none",
    ))
    assert "none(" not in code
    assert not any(line.startswith("d3 >>") for line in code.splitlines())


def test_explicit_counter_synth_off_drops_the_counter_layer() -> None:
    """``counter_synth='OFF'`` (другое слово, любой регистр) — тот же
    эффект."""
    from rob_box_mcp_tools.core.arranger import render, spec_from_flat
    from rob_box_mcp_tools.core.harmonize import harmonize

    notes = [(72 + (i % 5), 0.25) for i in range(32)]
    harmony = harmonize(notes, bpm=120, root="C", scale="major")
    assert harmony.dense is True

    code = render(spec_from_flat(
        harmony=harmony, bpm=harmony.bpm, root=harmony.root,
        scale=harmony.scale, form="arc", lead_synth="pluck",
        bass_synth="dub", pad_synth="warmpad", counter_synth="OFF",
    ))
    assert "none(" not in code
    assert "off(" not in code
    assert not any(line.startswith("d3 >>") for line in code.splitlines())


def test_explicit_real_counter_synth_is_still_respected_end_to_end() -> None:
    """Реальный контрастный тембр по-прежнему доходит до кода целиком."""
    from rob_box_mcp_tools.core.arranger import render, spec_from_flat
    from rob_box_mcp_tools.core.harmonize import harmonize

    notes = [(72 + (i % 5), 0.25) for i in range(32)]
    harmony = harmonize(notes, bpm=120, root="C", scale="major")
    assert harmony.dense is True

    code = render(spec_from_flat(
        harmony=harmony, bpm=harmony.bpm, root=harmony.root,
        scale=harmony.scale, form="arc", lead_synth="imperialbrass",
        bass_synth="dub", pad_synth="warmpad", counter_synth="strings",
    ))
    counter = next(
        line for line in code.splitlines() if line.startswith("d3 >>")
    )
    assert "strings(" in counter
