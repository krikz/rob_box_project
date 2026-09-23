"""Свойства синтов, важные для аранжировки (ADR-0132 PR-6).

Зачем
=====

До PR-6 в ``tools/music.py`` жил ``_heavy_brass_safety_net``: зашитый
список ``{"imperialbrass"}``, который при ``name=`` МОЛЧА выключал второй
голос и октавное удвоение темы (live 15.09, Григ: три голоса imperialbrass
с хвостом звучали эхом). Это хак под имя синта: модель не видела, почему
её ``counter``/``theme_octaves`` не сыграли, а у других синтов с тем же
устройством огибающей защиты не было вовсе.

Теперь решение — у модели (правило composer.txt «долгий релиз → ≤2 голоса
на теме»), а тул честно показывает факт: таблица ниже описывает синт по
его SynthDef-исходнику, партитура по таблице пишет предупреждение, если на
теме звучат 3 голоса и среди них есть синт с долгим хвостом. Ничего не
выключается.

Откуда значения
===============

Только из исходников, которые лежат в репо:

* ``docker/vision/voice_assistant/custom_synthdefs/*.scd`` — свои синты;
* ``src/rob_box_voice/rob_box_voice/core/renardo_synthdef_patches.py`` —
  исправленные исходники Renardo ``brass``/``organ``/``tb303``, которые
  ``fix_brass_scd.py`` записывает поверх пакета в образе.

Остальные синты палитры ``foxdot_init.sc::startupSynths`` (``strings``,
``pianovel``, ``blip``, …) — это ``.scd`` установленного пакета
``renardo_lib``, в репо их нет, и классифицировать их «по памяти» мы не
стали: :func:`traits_of` для них даёт ``None``, предупреждения нет.

Классы хвоста (``tail``):

* ``held`` — ``Env.adsr(...)`` без аргумента ``gate``: у ``EnvGen`` gate по
  умолчанию 1, фаза release внутри SynthDef не наступает — огибающая
  держит уровень sustain, пока Renardo не освободит узел. Нота звучит
  дольше своей длительности, соседние ноты накладываются.
* ``fixed`` — ``Env.linen(atk, sus, rel)`` с фиксированным ``rel`` ≥ 1 с:
  хвост не зависит от длины ноты.
* ``short`` — ``Env.perc(atk, sus)`` или ``Env.asr`` с закрывающимся gate и
  коротким ``rel``: звук кончается вместе с нотой.

``long_release`` = ``tail != "short"``. Число секунд в ``tail_note`` есть
только там, где оно известно: у ``fixed`` — из исходника; у imperialbrass —
оценка на слух живого прогона 15.09 (коммит b830ad33e), не измерение.
Для остальных ``held`` длительность хвоста зависит от освобождения узла в
Renardo и в репо не измерена — число не пишем.

На слух подтверждён только imperialbrass. ``supersawlead``,
``strangerbrass``, ``strangerarp``, ``marchstrings`` отнесены к ``held`` по
устройству огибающей (та же форма ``Env.adsr`` без gate) — это вывод из
исходника, не прослушивание.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional

__all__ = [
    "SynthTraits",
    "SYNTH_TRAITS",
    "THEME_VOICE_LIMIT",
    "traits_of",
    "theme_tail_warning",
]

_CUSTOM = "docker/vision/voice_assistant/custom_synthdefs/"
_PATCHES = "rob_box_voice/core/renardo_synthdef_patches.py"

#: Сколько голосов темы одновременно допустимо, если среди них есть синт с
#: долгим хвостом (composer.txt: «не больше 2 голосов ОДНОВРЕМЕННО на теме»).
THEME_VOICE_LIMIT = 2


@dataclass(frozen=True)
class SynthTraits:
    """Свойства одного SynthDef для аранжировки.

    Attributes:
        tail: класс хвоста — ``held`` | ``fixed`` | ``short`` (см. модуль).
        register: для какой партии синт написан — ``lead`` | ``bass`` | ``pad``.
        tail_note: длина хвоста текстом (``"≈1.5 с"``), ``""`` — не измерена.
        source: файл и огибающая, из которых взяты значения.
    """

    tail: str
    register: str
    tail_note: str
    source: str

    @property
    def long_release(self) -> bool:
        """Хвост дольше ноты — стэк голосов на теме накладывается эхом."""
        return self.tail != "short"


#: Таблица свойств синтов. Ключ — имя SynthDef в нижнем регистре.
SYNTH_TRAITS: Mapping[str, SynthTraits] = {
    "imperialbrass": SynthTraits(
        "held", "lead", "≈1.5 с",
        _CUSTOM + "imperialbrass.scd: Env.adsr(atk, 0.16, 0.76, sus*0.45) без gate; "
        "≈1.5 с — на слух, live 15.09 (b830ad33e)",
    ),
    "supersawlead": SynthTraits(
        "held", "lead", "",
        _CUSTOM + "supersawlead.scd: Env.adsr(atk, 0.12, 0.75, sus*0.6) без gate",
    ),
    "strangerbrass": SynthTraits(
        "held", "lead", "",
        _CUSTOM + "strangerbrass.scd: Env.adsr(atk, 0.1, 0.72, sus*0.4) без gate",
    ),
    "strangerarp": SynthTraits(
        "held", "lead", "",
        _CUSTOM + "strangerarp.scd: Env.adsr(atk, 0.08, 0.68, sus*0.55) без gate",
    ),
    "marchstrings": SynthTraits(
        "held", "pad", "",
        _CUSTOM + "marchstrings.scd: Env.adsr(atk, 0.18, 0.66, sus*0.45) без gate, sus=1.2",
    ),
    "warmpad": SynthTraits(
        "fixed", "pad", "1.2 с",
        _CUSTOM + "warmpad.scd: Env.linen(atk, sus, 1.2), sus=4",
    ),
    "strangerpulsepad": SynthTraits(
        "fixed", "pad", "1.6 с",
        _CUSTOM + "strangerpulsepad.scd: Env.linen(atk, sus, 1.6), sus=6",
    ),
    "retrobass": SynthTraits(
        "short", "bass", "",
        _CUSTOM + "retrobass.scd: Env.perc(atk, sus)",
    ),
    "brass": SynthTraits(
        "short", "lead", "",
        _PATCHES + "::BRASS_SYNTHDEF: Env.perc(atk, sus)",
    ),
    "organ": SynthTraits(
        "short", "lead", "",
        _PATCHES + "::ORGAN_SYNTHDEF: Env.asr(atk, amp, rel=0.18) с gate на sus",
    ),
    "tb303": SynthTraits(
        "short", "bass", "",
        _PATCHES + "::TB303_SYNTHDEF: Env.perc(atk, sus+dec)",
    ),
}


def traits_of(synth: Optional[str]) -> Optional[SynthTraits]:
    """Свойства синта или ``None``, если исходника в репо нет."""
    if not synth:
        return None
    return SYNTH_TRAITS.get(synth.strip().lower())


def _voice_names(counter_synth: Optional[str], octave_doubled: bool) -> List[str]:
    voices = ["лид"]
    if counter_synth:
        voices.append("второй голос")
    if octave_doubled:
        voices.append("октава")
    return voices


def theme_tail_warning(
    lead_synth: Optional[str],
    counter_synth: Optional[str],
    octave_doubled: bool,
) -> Optional[str]:
    """Предупреждение «слишком много голосов с хвостом на теме» или ``None``.

    Голоса темы: лид, второй голос (``counter_synth``), октавное удвоение
    лида. Если их больше :data:`THEME_VOICE_LIMIT` и хотя бы у одного синта
    долгий хвост — модель получает конкретную ручку, а не тихую замену.
    Формулировка совпадает с правилом composer.txt «долгий релиз → ≤2
    голоса».
    """
    voices = _voice_names(counter_synth, octave_doubled)
    if len(voices) <= THEME_VOICE_LIMIT:
        return None
    long_synths: Dict[str, SynthTraits] = {}
    for synth in (lead_synth, counter_synth):
        traits = traits_of(synth)
        if traits is not None and traits.long_release:
            long_synths.setdefault(str(synth).strip().lower(), traits)
    if not long_synths:
        return None
    names = "/".join(long_synths)
    notes = [t.tail_note for t in long_synths.values() if t.tail_note]
    tail = f" {notes[0]}" if notes else ""
    return (
        f"{names}: долгий релиз{tail} + {' + '.join(voices[1:])} = "
        f"{len(voices)} голоса с хвостом → counter=off или theme_octaves=off"
    )
