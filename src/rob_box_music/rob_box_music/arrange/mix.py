"""Микс трека v2: уровни ролей по модели громкости, сайдчейн-огибающая, тембры темы, бочка стиля (PR-3c).

ADR-0149 §3.8, §3.10 п.1, §4.7. Все числа — таблицы ``knowledge``; стилевые — поля ``knowledge.Style`` (ADR-0153),
стиль приходит параметром; здесь только правила.

* **Уровни ролей.** ``Part.level_db`` — dB RMS роли в шкале модели громкости (перенос ``core/club_loudness``:
  ``knowledge.LANE_DB_AT_UNIT``). Цель роли — ``Style.role_level_db``; ``amp`` плеера выводится из уровня
  и синта (:func:`level_amp`), поэтому смена тембра не меняет баланс. Синт, который не дотягивает до цели
  на потолке ``amp``, получает в модели свой честный максимум, а не цель.
* **Сайдчейн «S».** Огибающая по 16-м от шагов триггера (:func:`duck_envelope`): мгновенная атака на ударе,
  подъём ``knowledge.SIDECHAIN_SHAPE`` — без ступеньки «на всю ноту» (v1: 0.25/0.75, ``club_arranger.pump_weights``).
  Рендер умножает на неё ``amplify`` ролей ``Mix.duck_roles``.
* **Вид секции (DJ_Dave, PR-7).** Энергия секции выбирает вид ``Style.looks`` (:func:`look`): рисунок бочки и
  глубину сайдчейна сразу — build ↔ drop одним переключением; триггер огибающей — бочка вида (``Mix.duck``).
* **LPF-свип (PR-7, ADR-0149 §3.12).** ``Style.section_lpf`` на басе и нотах одним «слайдером»
  (:func:`lpf_sweeps`): build открывается к дропу, дроп открыт, брейк прикрыт, хвост блэнда закрывается.
* **Мастер-шина (PR-7, §3.10).** :func:`set_master` — ``trim`` по энергии трека сета и профиль выравнивателя.
* **Дуга громкости.** :func:`section_arc` — смещение ``trim`` по секциям (``knowledge.SECTION_TRIM_DB``): build
  поднимается к дропу, брейк проваливается, второй дроп — пик.
* **Тембры.** Семья тембров стиля по теме (``knowledge.THEME_TIMBRE``) → синт роли по сиду трека (:func:`timbres`).
* **Бочка.** Сэмпл стиля из ``knowledge.KICK_SOUNDS`` (:func:`kick_sound`).
* **Стерео (PR-9, §3.9).** Ширина ролей — ``Style.stereo`` → ``Mix.stereo``; бочка и бас в центре.
  Ударные — сторона меняется на каждом ударе (:func:`alternate_pan`, перенос ``core/club_stereo.pan_steps``);
  тональная роль с двумя голосами — уровень делится на голоса (:func:`voice_amp`), громкость роли та же.
"""

from __future__ import annotations

import math
import random
from dataclasses import replace
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, SAMPLE_ROLES, STEPS_PER_BAR, Duck, Form, Mix, Part, Stereo, Sweep


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


def _level(style: kn.Style, role: str, part: Part) -> float:
    """Цель роли стиля, но не громче того, что синт даёт на потолке ``amp``."""
    unit, exponent = _unit(role, part)
    return round(min(style.role_level_db[role], layer_db(unit, exponent, kn.MAX_LAYER_AMP)), 2)


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


def timbres(style: kn.Style, theme_row: Optional[str], rng: random.Random) -> Dict[str, str]:
    """Синт тональных ролей из семьи тембров стиля по теме; выбор внутри семьи — ``rng`` (сид трека)."""
    family = style.timbres[kn.THEME_TIMBRE.get(theme_row or "", style.default_timbre)]
    return {role: rng.choice(family[role]) for role in kn.TONAL_ROLES}


def kick_sound(style: kn.Style) -> kn.KickSound:
    return kn.KICK_SOUNDS[style.kick_sound]


def look(style: kn.Style, energy: int) -> kn.Look:
    """Вид секции энергии ``energy`` (0..10): первый порог ``style.looks`` не выше энергии."""
    return next(v for threshold, v in style.looks if energy >= threshold)


def kick_steps(pattern: str) -> Tuple[int, ...]:
    return tuple(i for i, ch in enumerate(pattern) if ch == "X")


def lpf_sweeps(style: kn.Style, form: Form, roles: Sequence[str]) -> Dict[str, Tuple[Sweep, ...]]:
    """Свип роли по секциям: ``style.section_lpf`` на ``style.lpf_roles``, в хвосте блэнда — на всём, кроме бочки."""
    out: Dict[str, Tuple[Sweep, ...]] = {}
    for role in roles:
        if role == "kick":
            continue
        sweeps = tuple(style.section_lpf.get(sec.name, (kn.LPF_OPEN, kn.LPF_OPEN))
                       if role in style.lpf_roles or sec.name in style.lpf_tail_sections else (kn.LPF_OPEN, kn.LPF_OPEN)
                       for sec in form.sections)
        if any(hz != kn.LPF_OPEN for sweep in sweeps for hz in sweep):
            out[role] = sweeps
    return out


def set_master(energy: int) -> Dict[str, float]:
    """Ручки мастер-шины трека сета энергии ``energy``: ``trim`` после динамики и профиль выравнивателя сета."""
    return {"trim": kn.ENERGY_TRIM_DB[energy], **kn.SET_LEVELER}


def section_arc(form: Form, bpm: float) -> Tuple[Tuple[float, float, float], ...]:
    """Дуга громкости формы: на доле начала секции — смещение ``trim`` и время переезда, с.

    ``knowledge.SECTION_TRIM_DB``: секция с подъёмом едет к своему уровню всю длину (build → дроп), остальные
    встают за ``TRIM_LAG_S``. Песня — без дуги: её динамику делает состав куплетов."""
    if form.song:
        return ()
    out, beat = [], 0.0
    for sec in form.sections:
        beats = float(sec.bars * BEATS_PER_BAR)
        offset, rise = kn.SECTION_TRIM_DB.get(sec.name, (0.0, False))
        out.append((beat, offset, round(beats * 60.0 / bpm, 3) if rise else kn.TRIM_LAG_S))
        beat += beats
    return tuple(out)


def mix_parts(style: kn.Style, parts: Mapping[str, Part], form: Form) -> Tuple[Dict[str, Part], Mix]:
    """Партии с уровнями ролей и ``Mix`` трека: уровни, ширина ролей, сайдчейн и свип по видам секций формы.

    Песня (``form.song``, PR-11) — без клубного вида: ни сайдчейна, ни LPF-свипа; энергия куплетов (``SONG_VERSES``)
    — только состав ролей. ``trim`` у песни — дефолт 0: она играет вне сета (``Program.master`` пуст)."""
    leveled = {role: replace(part, level_db=_level(style, role, part)) for role, part in parts.items()}
    ducked = frozenset() if form.song else frozenset(r for r in style.duck_roles if r in leveled)
    stereo = {r: Stereo(**style.stereo[r]) for r in leveled if r in style.stereo}
    looks = [look(style, sec.energy) for sec in form.sections]
    duck = tuple(Duck(v.duck_depth, kick_steps(v.kick)) for v in looks)
    mix = Mix({r: p.level_db for r, p in leveled.items()}, stereo, duck if ducked else (), duck_roles=ducked,
              lpf={} if form.song else lpf_sweeps(style, form, sorted(leveled)))
    return leveled, mix


__all__ = ["alternate_pan", "duck_envelope", "kick_sound", "kick_steps", "layer_db", "level_amp", "look", "lpf_sweeps",
           "mix_parts", "section_arc", "set_master", "timbres", "voice_amp"]
