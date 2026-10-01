"""Микс трека v2: уровни ролей по модели громкости, сайдчейн-огибающая, тембры темы, бочка жанра (PR-3c).

ADR-0149 §3.8, §3.10 п.1, §4.7. Все числа — таблицы ``knowledge``; здесь только правила.

* **Уровни ролей.** ``Part.level_db`` — dB RMS роли в шкале модели громкости (перенос ``core/club_loudness``:
  ``knowledge.LANE_DB_AT_UNIT``). Цель роли — ``knowledge.ROLE_LEVEL_DB``; ``amp`` плеера выводится из уровня
  и синта (:func:`level_amp`), поэтому смена тембра не меняет баланс. Синт, который не дотягивает до цели
  на потолке ``amp``, получает в модели свой честный максимум, а не цель.
* **Сайдчейн «S».** Огибающая по 16-м от шагов триггера (:func:`duck_envelope`): мгновенная атака на ударе,
  подъём ``knowledge.SIDECHAIN_SHAPE`` — без ступеньки «на всю ноту» (v1: 0.25/0.75, ``club_arranger.pump_weights``).
  Рендер умножает на неё ``amplify`` ролей ``Mix.duck_roles``.
* **Тембры.** Семья тембров темы (``knowledge.THEME_TIMBRE``) → синт роли по сиду трека (:func:`timbres`).
* **Бочка.** Сэмпл жанра из ``knowledge.KICK_SOUNDS`` (:func:`kick_sound`).
* **Стерео (PR-9, §3.9).** Ширина ролей — ``knowledge.ROLE_STEREO`` → ``Mix.stereo``; бочка и бас в центре.
  Ударные — сторона меняется на каждом ударе (:func:`alternate_pan`, перенос ``core/club_stereo.pan_steps``);
  тональная роль с двумя голосами — уровень делится на голоса (:func:`voice_amp`), громкость роли та же.
"""

from __future__ import annotations

import math
import random
from dataclasses import replace
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..model import SAMPLE_ROLES, STEPS_PER_BAR, Mix, Part, Stereo


def layer_db(unit_db: float, exponent: float, amp: float) -> float:
    """dB RMS слоя на ``amp`` (модель громкости: ``unit + 20·p·log10(amp)``); общая со старым ``club_loudness``."""
    return unit_db + 20.0 * exponent * math.log10(amp)


def _unit(role: str, part: Part) -> Tuple[float, float]:
    """(dB при ``amp`` 1.0, показатель) партии: синт тональной роли, рисунок и сэмпл ударной. Сэмпл DJ_Dave (PR-3d) —
    средний уровень файла (``SampleInfo.mean_db``): синт ``loop`` даёт файл ≈ без усиления (``Mix`` стерео × 0.5);
    это допущение, не замер — живой проход идёт через выравниватель мастера."""
    if role in SAMPLE_ROLES:
        return kn.SAMPLE_CATALOG[part.synth_or_sample].mean_db, 1.0
    option = part.synth_or_sample if role in kn.TONAL_ROLES else kn.DRUM_LOUDNESS_KEY[role]
    db = kn.LANE_DB_AT_UNIT[role][option]
    if role == "kick":
        db += next((k.loudness_offset_db for k in kn.KICK_SOUNDS.values() if k.sample == part.sample), 0.0)
    return db, kn.AMP_EXPONENT.get(option, 1.0)


def level_amp(role: str, part: Part) -> float:
    """``amp`` плеера, при котором роль звучит на ``part.level_db`` (≤ ``knowledge.MAX_LAYER_AMP``)."""
    unit, exponent = _unit(role, part)
    return min(kn.MAX_LAYER_AMP, 10.0 ** ((part.level_db - unit) / (20.0 * exponent)))


def voice_amp(role: str, part: Part, voices: int) -> float:
    """``amp`` одного из ``voices`` декоррелированных голосов: их мощности складываются (+10·lg N дБ)."""
    return level_amp(role, replace(part, level_db=part.level_db - 10.0 * math.log10(voices)))


def alternate_pan(hits: Sequence[bool], width: float, first: int = 1) -> List[float]:
    """``pan`` по шагам на два прохода рисунка: сторона меняется на КАЖДОМ ударе (пропуск сторону не занимает).

    Два прохода — чтобы при нечётном числе ударов за проход сумма по ударам была 0. Список ``pan`` Renardo
    индексирует номером события, а пропуски — тоже события, поэтому длина = 2 × длина рисунка.
    """
    side, out = -first, []  # флип ДО записи удара
    for _ in range(2):
        for hit in hits:
            if hit:
                side = -side
            out.append(side * width)
    return out


def _level(role: str, part: Part) -> float:
    """Цель роли, но не громче того, что синт даёт на потолке ``amp``."""
    unit, exponent = _unit(role, part)
    return round(min(kn.ROLE_LEVEL_DB[role], layer_db(unit, exponent, kn.MAX_LAYER_AMP)), 2)


def duck_envelope(trigger: Sequence[int], depth: float) -> Tuple[float, ...]:
    """16 усилений такта: на шаге триггера ``1 − depth·(1 − форма[0])``, дальше подъём по форме до 1.0."""
    if not trigger or depth <= 0:
        return (1.0,) * STEPS_PER_BAR
    shape = kn.SIDECHAIN_SHAPE
    out = []
    for step in range(STEPS_PER_BAR):
        since = min((step - t) % STEPS_PER_BAR for t in trigger)
        gain = shape[since] if since < len(shape) else 1.0
        out.append(round(1.0 - depth * (1.0 - gain), 3))
    return tuple(out)


def timbres(theme_row: Optional[str], rng: random.Random) -> Dict[str, str]:
    """Синт тональных ролей из семьи тембров темы; выбор внутри семьи — ``rng`` (сид трека)."""
    family = kn.TIMBRES[kn.THEME_TIMBRE.get(theme_row or "", kn.DEFAULT_TIMBRE)]
    return {role: rng.choice(family[role]) for role in kn.TONAL_ROLES}


def kick_sound(genre: str) -> kn.KickSound:
    return kn.KICK_SOUNDS[kn.GENRE_KICK[genre]]


def mix_parts(parts: Mapping[str, Part], trigger: Sequence[int]) -> Tuple[Dict[str, Part], Mix]:
    """Партии с уровнями ролей и ``Mix`` трека: уровни, ширина ролей, сайдчейн от ``trigger``."""
    leveled = {role: replace(part, level_db=_level(role, part)) for role, part in parts.items()}
    ducked = frozenset(r for r in kn.DUCK_ROLES if r in leveled)
    stereo = {r: Stereo(**kn.ROLE_STEREO[r]) for r in leveled if r in kn.ROLE_STEREO}
    mix = Mix({r: p.level_db for r, p in leveled.items()}, stereo,
              kn.DUCK_DEPTH if ducked else 0.0, duck_roles=ducked, duck_trigger=tuple(sorted(set(trigger))))
    return leveled, mix


__all__ = ["alternate_pan", "duck_envelope", "kick_sound", "layer_db", "level_amp", "mix_parts", "timbres", "voice_amp"]
