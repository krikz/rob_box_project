"""Микс трека v2: уровни ролей по модели громкости, сайдчейн-огибающая, тембры темы, бочка жанра (PR-3c).

ADR-0149 §3.8, §3.10 п.1, §4.7. Все числа — таблицы ``knowledge``; здесь только правила.

* **Уровни ролей.** ``Part.level_db`` — dB RMS роли в шкале модели громкости (перенос ``core/club_loudness``:
  ``knowledge.LANE_DB_AT_UNIT``). Цель роли — ``knowledge.ROLE_LEVEL_DB``; ``amp`` плеера выводится из уровня
  и синта (:func:`level_amp`), поэтому смена тембра не меняет баланс. Синт, который не дотягивает до цели
  на потолке ``amp``, получает в модели свой честный максимум, а не цель.
* **Сайдчейн «S».** Огибающая по 16-м от шагов триггера (:func:`duck_envelope`): мгновенная атака на ударе,
  подъём ``knowledge.SIDECHAIN_SHAPE`` — без ступеньки «на всю ноту» (v1: 0.25/0.75, ``club_arranger.pump_weights``).
  Рендер умножает на неё ``amplify`` ролей ``Mix.duck_roles``.
* **Вид секции (DJ_Dave, PR-7).** Энергия секции выбирает вид ``knowledge.LOOKS`` (:func:`look`): рисунок бочки и
  глубину сайдчейна сразу — build ↔ drop одним переключением; триггер огибающей — бочка вида (``Mix.duck``).
* **LPF-свип (PR-7, ADR-0149 §3.12).** ``knowledge.SECTION_LPF`` на басе и нотах одним «слайдером»
  (:func:`lpf_sweeps`): build открывается к дропу, дроп открыт, брейк прикрыт, хвост блэнда закрывается.
* **Мастер-шина (PR-7, §3.10).** :func:`set_master` — ``trim`` по энергии трека сета и профиль выравнивателя.
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
from ..model import SAMPLE_ROLES, STEPS_PER_BAR, Duck, Form, Mix, Part, Stereo, Sweep


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


def look(energy: int) -> kn.Look:
    """Вид секции энергии ``energy`` (0..10): первый порог ``knowledge.LOOKS`` не выше энергии."""
    return next(v for threshold, v in kn.LOOKS if energy >= threshold)


def kick_steps(pattern: str) -> Tuple[int, ...]:
    return tuple(i for i, ch in enumerate(pattern) if ch == "X")


def lpf_sweeps(form: Form, roles: Sequence[str]) -> Dict[str, Tuple[Sweep, ...]]:
    """Свип роли по секциям: ``SECTION_LPF`` на ``LPF_ROLES``, в хвосте блэнда — на всём, кроме бочки."""
    out: Dict[str, Tuple[Sweep, ...]] = {}
    for role in roles:
        if role == "kick":
            continue
        sweeps = tuple(kn.SECTION_LPF.get(sec.name, (kn.LPF_OPEN, kn.LPF_OPEN))
                       if role in kn.LPF_ROLES or sec.name in kn.LPF_TAIL_SECTIONS else (kn.LPF_OPEN, kn.LPF_OPEN)
                       for sec in form.sections)
        if any(hz != kn.LPF_OPEN for sweep in sweeps for hz in sweep):
            out[role] = sweeps
    return out


def set_master(energy: int) -> Dict[str, float]:
    """Ручки мастер-шины трека сета энергии ``energy``: ``trim`` после динамики и профиль выравнивателя сета."""
    return {"trim": kn.ENERGY_TRIM_DB[energy], **kn.SET_LEVELER}


def mix_parts(parts: Mapping[str, Part], form: Form) -> Tuple[Dict[str, Part], Mix]:
    """Партии с уровнями ролей и ``Mix`` трека: уровни, ширина ролей, сайдчейн и свип по видам секций формы.

    Песня (``form.song``, PR-11) — без клубного вида: ни сайдчейна, ни LPF-свипа; энергия куплетов (``SONG_VERSES``)
    — только состав ролей. ``trim`` у песни — дефолт 0: она играет вне сета (``Program.master`` пуст)."""
    leveled = {role: replace(part, level_db=_level(role, part)) for role, part in parts.items()}
    ducked = frozenset() if form.song else frozenset(r for r in kn.DUCK_ROLES if r in leveled)
    stereo = {r: Stereo(**kn.ROLE_STEREO[r]) for r in leveled if r in kn.ROLE_STEREO}
    duck = tuple(Duck(look(sec.energy).duck_depth, kick_steps(look(sec.energy).kick)) for sec in form.sections)
    mix = Mix({r: p.level_db for r, p in leveled.items()}, stereo, duck if ducked else (), duck_roles=ducked,
              lpf={} if form.song else lpf_sweeps(form, sorted(leveled)))
    return leveled, mix


__all__ = ["alternate_pan", "duck_envelope", "kick_sound", "kick_steps", "layer_db", "level_amp", "look", "lpf_sweeps",
           "mix_parts", "set_master", "timbres", "voice_amp"]
