"""Нота файла деки не снимается ``makeSound`` на тихой голове файла (живьём 14.10: мост-файл молчал весь проход).

Каждая нота Renardo кончается узлом ``makeSound`` (``renardo_lib/.../sceffects/makeSound.scd``):
``DetectSilence.ar(osc, amp: 0.0001, time: 0.1, doneAction: 14)`` — 0.1 с тише −80 dBFS освобождают всю группу ноты.
Мост-файл (``render_audio(xfade_bars=4, once=False)``) нарастает с нуля за кроссфейд (``atk`` = 16 с), а голова
трека генерации — щелчок и ~0.1 с на −70 dBFS. Под почти нулевым краем это «тишина»: группа ноты моста ушла через
128 мс после ``/n_go`` (``/notify`` на scsynth робота), следующая нота — через проход (132 с), мост молчал до входа
результата. Нота файла деки (``fdeck > 0``) несёт неслышный DC-пол на время ``sus`` и под огибающей деки.
"""

from __future__ import annotations

import math
import re

from rob_box_voice.core.renardo_synthdef_patches import LOOP_ALIVE_FLOOR, LOOP_SYNTHDEF, MAKESOUND_SILENCE_AMP

SR = 16000  # scsynth робота: -S 16000
SILENCE_S = 0.1  # makeSound: DetectSilence time


def _head() -> list:
    """Голова трека кэша ``08e68ee0`` (пики окон 20 мс с робота): щелчок 40 мс, ~0.1 с у −70 dBFS, потом музыка."""
    peaks = [0.131, 0.086, 0.0036, 0.00037, 0.00024, 0.00021, 0.0042, 0.146, 0.204, 0.148]
    return [p for p in peaks for _ in range(SR // 50)]


def _note_out(floor: float, atk_s: float = 16.0) -> list:
    """Выход ноты ``loop`` (моно-сумма × 0.5, Pan2 центр) под краем ``\\wel`` — и пол, если он есть."""
    pan = 0.5 * math.sqrt(0.5)
    return [x * pan * math.sin(math.pi / 2 * min(1.0, i / SR / atk_s)) + floor for i, x in enumerate(_head())]


def _freed(out: list) -> bool:
    """``DetectSilence``: ``SILENCE_S`` подряд ниже порога после первого звучащего отсчёта — группа освобождена."""
    run, armed = 0, False
    for x in out:
        if abs(x) > MAKESOUND_SILENCE_AMP:
            run, armed = 0, True
        elif armed:
            run += 1
            if run >= SILENCE_S * SR:
                return True
    return False


def test_quiet_head_under_crossfade_edge_frees_the_note_without_a_floor():
    assert _freed(_note_out(0.0))  # так было: группа моста снята на голове файла


def test_floor_keeps_a_deck_file_note_alive():
    assert not _freed(_note_out(LOOP_ALIVE_FLOOR))
    assert LOOP_ALIVE_FLOOR >= 2 * MAKESOUND_SILENCE_AMP  # запас на ``volume`` ноты
    assert 20 * math.log10(LOOP_ALIVE_FLOOR) < -60  # пол неслышен


def test_floor_only_for_deck_files_for_sus_and_under_the_deck_fade():
    """Пол — только у файла деки (синтез с ``fdeck=0`` прежний), только на ``sus`` (группа уходит, когда нота
    отзвучала) и под огибающей деки (погашенный мост уходит со своей группой); обоим каналам — после ``Pan2``."""
    alive = re.search(r"alive = (.*);", LOOP_SYNTHDEF)
    assert alive, "нет пола ноты файла деки"
    assert alive.group(1) == f"EnvGen.ar(Env([1, 1, 0], [sus, 0])) * deckGain * (fdeck > 0) * {LOOP_ALIVE_FLOOR}"
    body = LOOP_SYNTHDEF
    assert body.index("osc = Pan2.ar(osc, pan);") < body.index("osc = osc + alive;") < body.index("ReplaceOut")
