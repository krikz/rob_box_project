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
* **Тембры.** Семья тембров сета (``SetPlan.family``: строка темы — ``knowledge.THEME_TIMBRE``, вне таблицы — сид со штрафом, #3460) → синт роли по сиду трека со штрафом за
  недавние: бас и лид — :func:`role_timbre` (ADR-0152 PR-6), пэд — по рисунку (:func:`pad_timbre`, §3.2).
* **Рисунок пэда (ADR-0152 PR-5).** ``knowledge.PAD_FIGURES``: ``held`` не под сайдчейном, цель уровня рисунка —
  цель роли + ``level_offset_db`` (громкость как у ``pumped16``).
* **A9-модель трека (ADR-0152 §4 п.2).** Доля низа каждого дропа на роботе по полосам слоёв (:func:`a9_model`;
  пэд — со сдвигом синта на роботе ``knowledge.PAD_ROBOT_DB``, #3441); ниже
  ``Style.a9_model_low`` — пэд тише, потом бас громче (:func:`a9_trim`), поправка — ``Mix.a9_trim``.
* **Бочка.** Сэмпл из пула стиля ``knowledge.KICK_SOUNDS`` (:func:`kick_sound`), выбор — ``TrackPlan.kick``.
* **Стерео (PR-9, §3.9).** Ширина ролей — ``Style.stereo`` → ``Mix.stereo``; бочка и бас в центре.
  Ударные — сторона меняется на каждом ударе (:func:`alternate_pan`, перенос ``core/club_stereo.pan_steps``);
  тональная роль с двумя голосами — уровень делится на голоса (:func:`voice_amp`), громкость роли та же.
"""

from __future__ import annotations

import math
import random
from dataclasses import replace
from typing import Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..diversity import weighted_pick
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
    kick = kn.kick_of(part.play_symbol, part.sample) if role == "kick" else None
    if kick is not None:
        db += kn.KICK_SOUNDS[kick].loudness_offset_db
    return db, kn.AMP_EXPONENT.get(option, 1.0)


def level_amp(role: str, part: Part) -> float:
    """``amp`` плеера, при котором роль звучит на ``part.level_db`` (≤ ``knowledge.MAX_LAYER_AMP``)."""
    unit, exponent = _unit(role, part)
    return min(kn.MAX_LAYER_AMP, 10.0 ** ((part.level_db - unit) / (20.0 * exponent)))


def file_gain(name: str, reference: str) -> float:
    """Усиление файла пула против файла уровня партии (``Part.synth_or_sample``, #3432): ``amp`` роли выведен из
    ``mean_db`` эталона, файл громче него по каталогу тише на разницу. Тише — не поднимается: у файла длиннее удара
    ``mean_db`` занижен хвостом, а ``sus`` режет его до шага (psr_24: 3.1 с, −33 дБ — начало громче среднего)."""
    diff = kn.SAMPLE_CATALOG[reference].mean_db - kn.SAMPLE_CATALOG[name].mean_db
    return min(1.0, 10.0 ** (diff / 20.0))


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


def _cap(role: str, part: Part) -> float:
    """Громче не бывает: потолок роли и то, что синт даёт на потолке ``amp``."""
    unit, exponent = _unit(role, part)
    return min(kn.role_ceiling(role), layer_db(unit, exponent, kn.MAX_LAYER_AMP))


def target_db(style: kn.Style, role: str, pad_figure: Optional[str] = None) -> float:
    """Цель уровня роли: ``Style.role_level_db``, у пэда — + ``level_offset_db`` рисунка (до поправки A9)."""
    offset = kn.PAD_FIGURES[pad_figure].level_offset_db if role == "pad" and pad_figure else 0.0
    return style.role_level_db[role] + offset


def _level(style: kn.Style, role: str, part: Part, pad_figure: Optional[str]) -> float:
    """Цель роли (:func:`target_db`), но не громче :func:`_cap`."""
    return round(min(target_db(style, role, pad_figure), _cap(role, part)), 2)


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


def _family(style: kn.Style, family: str) -> Mapping[str, Tuple[str, ...]]:
    return style.timbres[family]


def role_timbre(style: kn.Style, family: str, role: str, recent: Sequence[Optional[str]],
                rng: random.Random) -> str:
    """Синт роли ``role`` (бас, лид) из семьи тембров стиля по теме со штрафом за недавние (``recent`` — свежие
    первыми, ``music_history.<роль>``); ГСЧ — сид трека на ось."""
    return weighted_pick(_family(style, family)[role], recent, rng)


def bass_pair_ok(figure: str, synth: str) -> bool:
    """Рисунок баса и синт сочетаются (``knowledge.BASS_FIGURE_SYNTHS``): названное в таблице звучит только вместе —
    рисунок с перечисленными синтами, синт с рисунками, которые его называют."""
    table = kn.BASS_FIGURE_SYNTHS
    paired = {s for synths in table.values() for s in synths}
    return synth in table[figure] if figure in table else synth not in paired


def bass_synths(style: kn.Style, family: str, figure: str) -> Tuple[str, ...]:
    """Басы семьи, которые играют рисунок ``figure``."""
    return tuple(s for s in _family(style, family)["bass"] if bass_pair_ok(figure, s))


def bass_figures(style: kn.Style, family: str) -> Tuple[str, ...]:
    """Рисунки пула стиля, у которых в семье темы есть бас (``acid16`` — только где в семье ``tb303``)."""
    return tuple(f for f in style.bass_figures if bass_synths(style, family, f))


def sustains_to_sus(synth: str) -> bool:
    """Синт звучит ровно ``sus`` — без собственного хвоста (``knowledge.SYNTH_TRAITS``)."""
    traits = kn.traits_of(synth)
    return traits is None or traits.tail == "short"


def pad_synths(style: kn.Style, family: str, figure: str) -> Tuple[str, ...]:
    """Пэды семьи, которые звучат рисунком ``figure``: синт с хвостом — только где ``long_tails``."""
    long_tails = kn.PAD_FIGURES[figure].long_tails
    return tuple(s for s in _family(style, family)["pad"] if long_tails or sustains_to_sus(s))


def pad_timbre(style: kn.Style, family: str, figure: str, recent: Sequence[Optional[str]],
               rng: random.Random) -> str:
    """Синт пэда рисунка ``figure`` со штрафом за недавние (``recent`` — свежие первыми)."""
    return weighted_pick(pad_synths(style, family, figure), recent, rng)


def kick_sound(style: kn.Style, name: Optional[str] = None) -> kn.KickSound:
    """Бочка трека: ``name`` из пула стиля (выбор плана, ``TrackPlan.kick``) или первая бочка пула."""
    name = name or style.kick_pool[0]
    if name not in style.kick_pool:
        raise ValueError(f"бочка {name!r} не из пула стиля {style.kick_pool}")
    return kn.KICK_SOUNDS[name]


def look(style: kn.Style, energy: int) -> kn.Look:
    """Вид секции энергии ``energy`` (0..10): первый порог ``style.looks`` не выше энергии."""
    return next(v for threshold, v in style.looks if energy >= threshold)


def kick_steps(pattern: str) -> Tuple[int, ...]:
    return tuple(i for i, ch in enumerate(pattern) if ch == "X")


def _note_lpf(part: Part) -> bool:
    """Партия со срезом на каждую ноту (``PitchEvent.lpf``, ``acid16``): общий свип секции на ней не нужен."""
    return any(ev.lpf for ev in part.pitches or ())


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


def _layer_low(role: str, part: Part) -> float:
    """Доля низа (< 150 Гц) слоя: бочка — целиком (так модель сверена с записью #3441; полосы NRT бочки — ``X`` без
    ``sample=``, на роботе другой звук), тональные — ``knowledge.LAYER_BANDS``, ударные и сэмплы — 0 (замера нет)."""
    if role == "kick":
        return 1.0
    return kn.LAYER_BANDS[role][part.synth_or_sample][0] if role in kn.TONAL_ROLES else 0.0


def _sounds(style: kn.Style, role: str, section_name: str, roles: frozenset) -> bool:
    if role in SAMPLE_ROLES:  # FX — один удар на секцию, не в счёт
        return role != "fx" and section_name in style.layer_sections.get(role, ())
    return role in roles


def _model_db(role: str, part: Part, pad_offset_db: float) -> float:
    """dB роли в шкале робота: пэд — без прибавки рисунка (шкала ``pumped16``) и со сдвигом синта на роботе
    (``knowledge.PAD_ROBOT_DB``; синт без замера — худший сдвиг)."""
    if role != "pad":
        return part.level_db
    return part.level_db - pad_offset_db + kn.PAD_ROBOT_DB.get(part.synth_or_sample, kn.PAD_ROBOT_DB_UNMEASURED)


def low_share(style: kn.Style, parts: Mapping[str, Part], form: Form, i: int, duck: Duck, ducked: frozenset,
              pad_offset_db: float) -> float:
    """Доля низа секции ``i`` на роботе: роль звучит всю секцию на ``level_db`` (пэд — :func:`_model_db`), под
    сайдчейном — × средняя мощность огибающей, бочка вида build — по числу ударов."""
    sec = form.sections[i]
    env = duck_envelope(duck.trigger, duck.depth)
    duck_power = sum(g * g for g in env) / len(env)
    low = total = 0.0
    for role, part in parts.items():
        if not _sounds(style, role, sec.name, sec.roles):
            continue
        power = 10.0 ** (_model_db(role, part, pad_offset_db) / 10.0)
        power *= duck_power if role in ducked else 1.0
        power *= look(style, sec.energy).kick.count("X") / 4 if role == "kick" else 1.0
        total += power
        low += power * _layer_low(role, part)
    return low / total if total else 0.0


def a9_model(style: kn.Style, parts: Mapping[str, Part], form: Form, duck: Sequence[Duck], ducked: frozenset,
             pad_offset_db: float) -> float:
    """Доля низа худшего дропа (секции ``drop*``) — A9 по дропам (ADR-0152 §4 п.1) в шкале робота."""
    shares = [low_share(style, parts, form, i, duck[i], ducked, pad_offset_db)
              for i, sec in enumerate(form.sections) if sec.name.startswith("drop")]
    return round(min(shares), 3) if shares else 1.0


def _a9_phases(parts: Mapping[str, Part]) -> Iterator[Iterator[Tuple[str, float]]]:
    """Ступени поправки по фазам: пэд тише до ``A9_PAD_FLOOR_DB``, потом бас громче до ``A9_BASS_BOOST_DB``."""
    step = kn.A9_STEP_DB
    if "pad" in parts:
        yield (("pad", -step * k) for k in range(1, int(-kn.A9_PAD_FLOOR_DB / step) + 1))
    if "bass" in parts:
        yield (("bass", step * k) for k in range(1, int(kn.A9_BASS_BOOST_DB / step) + 1))


def a9_trim(style: kn.Style, parts: Mapping[str, Part], form: Form, duck: Sequence[Duck], ducked: frozenset,
            pad_offset_db: float) -> Tuple[Dict[str, Part], Dict[str, float], float]:
    """Партии с поправкой A9: ступенями до порога ``Style.a9_model_low``. Ступень, которая не поднимает долю низа хотя
    бы на ``A9_MIN_GAIN`` (бас на потолке ``amp``, середину держит не пэд), не применяется и кончает фазу — пэд не
    глушится зря; не дотянули — доля честно ниже порога. Возвращает (партии, поправка роли в дБ, доля низа)."""
    out, share = dict(parts), a9_model(style, parts, form, duck, ducked, pad_offset_db)
    for phase in _a9_phases(parts):
        for role, db in phase:
            if share >= style.a9_model_low:
                break
            level = round(min(parts[role].level_db + db, _cap(role, parts[role])), 2)
            candidate = {**out, role: replace(parts[role], level_db=level)}
            gained = a9_model(style, candidate, form, duck, ducked, pad_offset_db)
            if gained - share < kn.A9_MIN_GAIN:
                break
            out, share = candidate, gained
    applied = {r: round(out[r].level_db - parts[r].level_db, 2) for r in ("pad", "bass")
               if r in parts and out[r].level_db != parts[r].level_db}
    return out, applied, share


def _ducked(style: kn.Style, form: Form, roles: Sequence[str], figure: Optional[kn.PadFigure]) -> frozenset:
    """Роли под сайдчейном: стиля, что есть в треке; пэд рисунка без насоса (``held``) — нет; песня — никто."""
    if form.song:
        return frozenset()
    return frozenset(r for r in style.duck_roles if r in roles and not (r == "pad" and figure and not figure.ducked))


def mix_parts(style: kn.Style, parts: Mapping[str, Part], form: Form,
              pad_figure: Optional[str] = None) -> Tuple[Dict[str, Part], Mix]:
    """Партии с уровнями ролей и ``Mix`` трека: уровни, ширина ролей, сайдчейн и свип по видам секций формы,
    рисунок пэда ``pad_figure`` (``knowledge.PAD_FIGURES``) и A9-модель трека (если есть бочка и бас).

    Песня (``form.song``, PR-11) — без клубного вида: ни сайдчейна, ни LPF-свипа, ни A9-модели; энергия куплетов
    (``SONG_VERSES``) — только состав ролей. ``trim`` у песни — дефолт 0: она играет вне сета (``Program.master``
    пуст)."""
    figure = kn.PAD_FIGURES[pad_figure] if pad_figure else None
    offset = figure.level_offset_db if figure else 0.0
    leveled = {role: replace(part, level_db=_level(style, role, part, pad_figure)) for role, part in parts.items()}
    ducked = _ducked(style, form, list(leveled), figure)
    looks = [look(style, sec.energy) for sec in form.sections]
    duck = tuple(Duck(v.duck_depth, kick_steps(v.kick)) for v in looks)
    trim: Dict[str, float] = {}
    a9: Optional[float] = None
    if not form.song and {"kick", "bass"} <= set(leveled):
        leveled, trim, a9 = a9_trim(style, leveled, form, duck, ducked, offset)
    stereo = {r: Stereo(**style.stereo[r]) for r in leveled if r in style.stereo}
    mix = Mix({r: p.level_db for r, p in leveled.items()}, stereo, duck if ducked else (), duck_roles=ducked,
              lpf={} if form.song else lpf_sweeps(style, form, [r for r in sorted(leveled) if not _note_lpf(leveled[r])]), a9_trim=trim, a9_model=a9)
    return leveled, mix


__all__ = ["a9_model", "a9_trim", "alternate_pan", "duck_envelope", "file_gain", "kick_sound", "kick_steps", "layer_db",
           "level_amp", "bass_figures", "bass_pair_ok", "bass_synths", "look", "low_share", "lpf_sweeps", "mix_parts", "pad_synths", "pad_timbre", "role_timbre",
           "section_arc", "set_master", "sustains_to_sus", "target_db", "voice_amp"]
