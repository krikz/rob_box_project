"""``compose(plan, track_no, ...)`` — трек сета по плану (ADR-0149 §3.3, §3.4, §4.4–§4.7; PR-3a, PR-3b).

Форма трека — шаблон ``Style.forms`` (ADR-0152 §3.5, PR-7: ``club48``/``short32``/``long64``/``dropfirst48``), ключ
``TrackPlan.template`` (план выбирает по энергии трека и истории, :func:`set_plan.pick_template`). База ``club48``,
48 тактов: intro (хэт, пэд; 4 такта) → intro_low (+бочка, бас; 4) → build (+клэп, начало хука) → drop (хук)
→ break (без бочки и баса, хук вдвое медленнее) → drop2 (хук в параллельных терциях) → outro (без лида; 4) →
outro_tail (хэт, пэд; 4). Первый трек сета — ``Style.opening_form`` (``dropfirst48``, #3427): дроп сразу после интро,
build — перед drop2, и хук №1 темы (порядок ``search.theme_hooks``), а не мелодия по сиду.
Интро и аутро поделены под блэнд двух дек (PR-8, ``model.blend_bars``): хвост уходящего и начало входящего
звучат вместе 8 тактов, бочка и бас меняются на такте свопа. Хук — начало мелодии темы из локальной
RTTTL-библиотеки (``arrange.hook``); тональность трека — тоника профиля и лад хука.
Нет годной мелодии темы — лид-мотив «вопрос/ответ» (PR-2) с тем же развитием, ``track.hook = None``.
Прогрессия подбирается под хук (``harmony.fit_progression``), бас в оффбит, пэд с голосоведением.
План назвал материал партитуры (``TrackPlan.material``, ADR-0154 PR-3) — хук и ступени слотов из него
(``hook.from_material``, ``harmony.from_material``), тоны баса — из басового голоса автора (``bass.material_tones``,
PR-4), ``drop2`` — ритм хука с контуром следующей фразы; не годится — путь выше без изменений.

Стиль (ADR-0153 S0): стилевые таблицы — ``knowledge.STYLES[plan.style]`` (форма, регистры, тембры, каркасы, виды
секций, прогрессии, микс); стиль идёт параметром в генераторы. Генератор роли выбирается по ключу фигуры стиля из
реестров :data:`BASS_GENERATORS`/:data:`PAD_GENERATORS`/:data:`LEAD_GENERATORS` — без ветвления по стилю.

Из плана (``set_plan``): темп сета, тоника трека (ход по квинтам), энергия трека — сдвиг энергии секций и
состав ролей (``knowledge.ENERGY_THIN_ROLES``), свинг хэтов. Секция перед дропом кончается fill-ом
(``arrange.rhythm``): ролл клэпа, бочка снята на последней доле. Клэп-бэкбит — только в дропах, в build
и break клэп звучит одним роллом fill-а.
Так рисунки ударных складываются в период 16 тактов — степень двойки, которую санитайзер v1 не трогает
(``_fix_pattern_length``; на роботе программа v2 пока идёт через ``execute_music_code``).

Микс (PR-3c, ``arrange.mix``): уровни ролей по модели громкости, тембры по семье темы и сиду трека, бочка стиля
с настоящим низом (``knowledge.KICK_SOUNDS``), сайдчейн-огибающая от рисунка бочки на басе и пэде.
Пэд (ADR-0152 PR-5): рисунок трека (``Style.pad_figures``: ``pumped16``/``held``/``stabs``) и синт семьи под рисунок —
по сиду со штрафом за недавнее (``music_history.pad_figure``/``pad``); A9-модель трека правит уровни пэда и баса
(``mix.mix_parts``).
Лид и бас (ADR-0152 PR-6): синт семьи темы (``Style.timbres``) и рисунок баса (``Style.bass_figures``:
``offbeat``/``rolling8``/``acid16``; пары рисунок ↔ синт — ``knowledge.BASS_FIGURE_SYNTHS``) — по сиду со штрафом за недавнее (``music_history.lead``/``bass``/``bass_figure``).

Разнообразие (PR-3d, ADR-0149 I17, A12, A13): ``history`` — строки ``music_history`` (свежие первыми). Каркас
ударных (``Style.kits``) не повторяет прошлый трек; прогрессия — не больше 3 раз за 10 треков; хук-фрагмент
(отпечаток без транспозиции) не повторяется подряд; сэмплы DJ_Dave — слой ``sample`` и ``fx`` по роли каталога
(``arrange.samples``). Выбор — ``diversity.weighted_pick`` со своим ГСЧ сида на каждую ось: сид меняет материал,
темп сета — нет. Что писать в историю — ``diversity.track_history(track)``.
"""

from __future__ import annotations

import hashlib
import logging
import random
from dataclasses import replace
from typing import Callable, Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..model import (
    BEATS_PER_BAR, Chord, Form, Grid, Harmony, HistoryKey, Hook, Key, Part, PitchEvent, Section, Track, Transition,
)
from ..diversity import fingerprint, last_opener, opening_order, recent_hooks, recent_values, weighted_pick
from ..material import ScoreMaterial
from ..rtttl import CONTOUR_NOTES, contour
from ..set_plan import SetPlan, TrackPlan, pick_kick, pick_template, seeded_plan
from ..theme import ThemeProfile
from . import bass, harmony, hook as hooks, lead, mix, pad, rhythm, samples

#: Генераторы ролей по ключу фигуры стиля (``Style.*_figures``, ADR-0153 §2.2). Тональные — ``(style, key,
#: bar_chords, synth, register) -> Part``; мотив лида без хука — ``(style, key, rng) -> ноты``.
BASS_GENERATORS: Mapping[str, Callable[..., Part]] = {
    "offbeat": bass.offbeat, "rolling8": bass.rolling8, "broken": bass.broken, "acid16": bass.acid16,
    "octave8": bass.octave8}
PAD_GENERATORS: Mapping[str, Callable[..., Part]] = {
    "pumped16": pad.pumped16, "held": pad.held, "stabs": pad.stabs, "arp": pad.arp}
LEAD_GENERATORS: Mapping[str, Callable[..., Tuple[PitchEvent, ...]]] = {"motif": lead.motif}
#: Нарезка лупа по ключу ``Style.loop_figure`` (ADR-0153 S3): ``(файл, ГСЧ) -> Part``.
LOOP_GENERATORS: Mapping[str, Callable[..., Part]] = {"chop": samples.loop_part,
                                                      "breakbeat_chop": samples.breakbeat_chop}
_LOG = logging.getLogger(__name__)
#: Длина секций с лидом (развитие хука считается от начала каждой).
SECTION_BARS = 8
CHORD_BARS = 2
#: Уровень партии до ``mix.mix_parts`` (он ставит уровень роли из ``Style.role_level_db``).
_UNLEVELED = 0.0
PAD_GAP = 3  # верх пэда ниже лида на ≥ 3 полутона (ADR-0149 §3.6)
LOOP_BEATS = CHORD_BARS * BEATS_PER_BAR * 4  # петля прогрессии — 4 аккорда


def _pad_floor(style: kn.Style) -> int:
    """Обычный низ пэда: коридор стиля выше на ``knowledge.PAD_WIDEN`` (ниже — только когда трезвучие иначе не помещается)."""
    return style.registers["pad"][0] + kn.PAD_WIDEN


def hook_register(style: kn.Style) -> Tuple[int, int]:
    """Коридор хука: низ лида ≥ обычный низ пэда + 9 + ``PAD_GAP``. Окно пэда в 10 нот (низ..низ+9) вмещает
    только трезвучия без части звуков лада (50..59 — без C и C#), поэтому «+9» не гарантировало обращения: пэд
    в :func:`_pad_chords` опускается до коридора, где окно в 12 нот (верх − 11) содержит каждый звук лада."""
    return _pad_floor(style) + 9 + PAD_GAP, style.registers["lead"][1]


def transition(style: kn.Style) -> Transition:
    """Переход трека (ADR-0149 §3.12, PR-8): блэнд и своп баса и бочки стиля — у входа и у выхода один."""
    return Transition(*style.blend, True)


#: Секции формы стиля: (имя, такты, энергия, роли) — значение ``Style.forms``.
FormSpec = kn.FormSpec


def form_spec(style: kn.Style, template: str) -> FormSpec:
    """Секции шаблона формы ``template`` (ключ ``Style.forms``)."""
    return style.forms[template]


def track_template(style: kn.Style, step: TrackPlan, track_no: int, history: Sequence[Mapping],
                   rng: random.Random) -> str:
    """Форма трека: из плана; план не назвал — первый трек сета ``Style.opening_form`` (тема раньше, #3427), остальные
    по энергии и истории (``pick_template``)."""
    if step.template:
        return step.template
    return style.opening_form if track_no == 1 else pick_template(style, step.energy, history, rng)


def _before_drop(spec: FormSpec, i: int) -> bool:
    return i + 1 < len(spec) and spec[i + 1][0].startswith("drop")


def _form(style: kn.Style, spec: FormSpec, energy: int) -> Form:
    """Секции трека энергии ``energy``: энергия секций сдвинута от средней (3), тонкие роли сняты. Fill — перед
    дропом (клэп-ролл, если энергия не сняла клэп) и в конце трека."""
    thin = frozenset(kn.ENERGY_THIN_ROLES.get(energy, ()))
    out = []
    for i, (name, bars, base, roles) in enumerate(spec):
        layers = {role for role, names in style.layer_sections.items() if name in names}
        roles = (roles | layers | ({"clap"} if _before_drop(spec, i) else set())) - thin
        fill = _before_drop(spec, i) or i == len(spec) - 1
        out.append(Section(name, bars, min(10, max(0, base + energy - 3)), frozenset(roles), fill))
    return Form(tuple(out))


def _sections(spec: FormSpec) -> Iterator[Tuple[int, int, str, frozenset]]:
    """(первый такт, такты, имя, роли) по форме."""
    start = 0
    for name, bars, _energy, roles in spec:
        yield start, bars, name, roles
        start += bars


def _bar_chords(spec: FormSpec, role: str, chords: Sequence[Chord]) -> List[Tuple[int, Chord]]:
    """(такт формы, аккорд петли) там, где звучит ``role``."""
    loop = len(chords) * CHORD_BARS
    return [(bar, chords[(bar % loop) // CHORD_BARS])
            for start, bars, _name, roles in _sections(spec) if role in roles for bar in range(start, start + bars)]


def _lead(style: kn.Style, spec: FormSpec, motif: Hook, key: Key, synth: str) -> Part:
    """Хук (или мотив) по секциям с лидом: развитие ``arrange.hook.develop`` от начала каждой секции."""
    events: List[PitchEvent] = []
    for start, bars, name, roles in _sections(spec):
        if "lead" in roles:
            offset = start * BEATS_PER_BAR
            events += [PitchEvent(e.midi, offset + e.beat, e.dur_beats, e.accent)
                       for e in hooks.develop(motif, name, bars, key)]
    total = sum(bars for _n, bars, _e, _r in spec) * 16
    grid = rhythm.grid({int(e.beat * 4) for e in events}, total)
    return Part("lead", synth, grid, tuple(events), _UNLEVELED, style.registers["lead"])


def _drums(style: kn.Style, form: Form, swing_ms: int, kit: str, kick_name: Optional[str] = None) -> Dict[str, Part]:
    """Бочка и клэп — на всю форму, хэты каркаса ``kit`` — такт со свингом. Бочка секции — рисунок её вида
    (``mix.look``: build ↔ drop). Клэп-бэкбит — в дропах; в остальных секциях клэп — только ролл: перед дропом —
    два такта (восьмые, затем 16-е, акцент растёт), в конце трека — полтакта. Бочка — сэмпл стиля с настоящим
    низом (``mix.kick_sound``)."""
    clap_bar = rhythm.clap_grid()
    silent = rhythm.grid(())
    index = {sec.name: i for i, sec in enumerate(form.sections)}

    def before_drop(sec: Section) -> bool:
        i = index[sec.name]
        return i + 1 < len(form.sections) and form.sections[i + 1].name.startswith("drop")

    def kick(sec: Section, bars_left: int) -> Grid:
        bar = rhythm.kick_grid(mix.look(style, sec.energy).kick)
        return rhythm.kick_fill(bar) if sec.fill_last_bar and bars_left == 1 else bar

    def clap(sec: Section, bars_left: int) -> Grid:
        bar = clap_bar if sec.name.startswith("drop") else silent
        if not sec.fill_last_bar:
            return bar
        if before_drop(sec) and bars_left <= rhythm.ROLL_BARS:
            return rhythm.clap_roll(rhythm.ROLL_BARS - bars_left)
        return rhythm.clap_fill(bar) if bars_left == 1 else bar

    grids = {"kick": rhythm.form_bars(form.sections, kick), "hats": rhythm.hats_grid(style, swing_ms, kit)}
    if any("clap" in sec.roles for sec in form.sections):
        grids["clap"] = rhythm.form_bars(form.sections, clap)
    kick = mix.kick_sound(style, kick_name)
    return {r: Part(r, kn.PLAY_SYNTH, g, None, _UNLEVELED, (0, 0), kick.sample if r == "kick" else 0,
                    symbol=kick.symbol if r == "kick" else "")
            for r, g in grids.items()}


def _hook_queue(profile: ThemeProfile, ids: Sequence[str], recent: Sequence[str], rng: random.Random) -> List[str]:
    """Очередь мелодий не первого трека: несыгранные (найденные по теме — в порядке профиля, пул — сидом), затем
    недавние (``recent`` — :func:`diversity.recent_hooks`), давние раньше свежих."""
    fresh = [i for i in ids if i not in recent]
    stale = sorted((i for i in ids if i in recent), key=recent.index, reverse=True)
    return (fresh if profile.theme_hooks else rng.sample(fresh, len(fresh))) + stale


def _least_recent(names: Sequence[str], recent: Sequence[str]) -> List[str]:
    """Несыгранные — в данном порядке, затем сыгранные: давние раньше свежих (``recent`` — свежие первыми)."""
    return sorted(names, key=lambda i: (1, -recent.index(i)) if i in recent else (0, 0))


def part_order(profile: ThemeProfile, ids: Sequence[str], recent: Sequence[str], track_no: int) -> List[str]:
    """Очередь хуков трека ``track_no`` темы-перечисления (``profile.theme_parts``, живой сет 06.10): часть
    ``track_no - 1`` по кругу первой (трек 1 — первая названная франшиза), за ней остальные части по кругу; внутри
    части — наименее недавняя версия (:func:`_least_recent`, ``recent`` — мелодии истории, свежие первыми). Повтор
    части — только когда круг частей пройден; сыгранная в прошлых сетах версия «Марио» не отдаёт трек ещё одной
    версии «Тетриса». Хуки ``ids`` вне частей (строка таблицы тем) — в конце."""
    allowed = set(ids)
    n = len(profile.theme_parts)
    out: List[str] = []
    for k in range(n):
        part = profile.theme_parts[(track_no - 1 + k) % n]
        out += _least_recent([i for i in part if i in allowed and i not in out], recent)
    return out + _least_recent([i for i in ids if i not in out], recent)


def hook_order(profile: ThemeProfile, melodies: Mapping[str, str], rng: random.Random,
               history: Sequence[Mapping] = (), opening: bool = False,
               track_no: int = 1, set_id: Optional[str] = None) -> List[str]:
    """Очередь мелодий трека ``track_no`` (первая — та, что сыграет, если она годится под гармонию): единственное
    место решения, его читают :func:`hook_candidates` (компоновка) и :func:`upcoming_hooks` (реплика «дальше будет»,
    #3497). Подробности порядка — в :func:`hook_candidates`."""
    last = history[0] if history else {}
    ids = [i for i in profile.hook_ids if i in melodies and i != last.get("melody_name")]
    recent = recent_hooks(history, set_id, {i: contour(melodies[i], CONTOUR_NOTES) or i for i in melodies})
    if profile.theme_parts:
        return part_order(profile, ids, recent, track_no)
    if opening:
        return opening_order(ids, recent, last_opener(history, set_id))
    return _hook_queue(profile, ids, recent, rng)


def upcoming_hooks(profile: ThemeProfile, melodies: Mapping[str, str], history: Sequence[Mapping],
                   set_id: Optional[str], first_no: int, count: int,
                   composed: Optional[Mapping[int, Optional[str]]] = None) -> List[Optional[str]]:
    """Мелодии треков ``first_no`` … ``first_no + count - 1`` темы (#3497, ADR-0148): трек, уже скомпонованный
    (``composed``: номер → мелодия), — как сыграет, остальные — первая в :func:`hook_order` при истории с
    предыдущими; очередь та же, что у компоновки. ``history`` — строки до ``first_no`` (свежие первыми). Тема без
    найденных по ней мелодий (пул выбирает сид) — пусто: заранее не известно."""
    if not profile.theme_hooks:
        return []
    rows = list(history)
    out: List[Optional[str]] = []
    for no in range(first_no, first_no + count):
        if composed is not None and no in composed:
            pick = composed[no]
        else:
            order = hook_order(profile, melodies, random.Random(f"upcoming:{set_id}:{no}"), rows,
                               opening=no == 1, track_no=no, set_id=set_id)
            pick = order[0] if order else None
        out.append(pick)
        rows.insert(0, {"melody_name": pick, "set_id": set_id})
    return out


def hook_candidates(profile: ThemeProfile, melodies: Mapping[str, str], rng: random.Random,
                    history: Sequence[Mapping] = (), opening: bool = False,
                    track_no: int = 1, set_id: Optional[str] = None) -> Iterator[Tuple[Hook, Key]]:
    """Годные мелодии темы: несыгранные — в порядке сида, недавние (история сета и прошлых сетов, I17) — в конце,
    давние раньше свежих; хук прошлого трека (мелодия или фрагмент) подряд не повторяется. Первый трек сета
    (``opening``) — :func:`opening_order`: хук №1 темы первым (#3427: порядок ``search.theme_hooks``), если он не
    звучал в последних сетах (#3399). Недавнее — :func:`diversity.recent_hooks` (``set_id`` — сет трека: его строки
    по записи, прошлых сетов — по мелодии, версии с общим контуром начала — одна мелодия).
    Хуки найдены по словам темы (``profile.theme_hooks``) — несыгранные идут в порядке профиля, а не сида: сет
    обходит найденные по очереди (у темы-перечисления — по кругу частей, ``search.round_robin``), повтор — только
    когда несыгранные кончились (06.10: 50 треков по кругу из трёх хуков). Тема-перечисление
    (``profile.theme_parts``) — очередь :func:`part_order` трека ``track_no`` и на первом треке тоже."""
    last = history[0] if history else {}
    register = hook_register(kn.STYLES[profile.style])
    order = hook_order(profile, melodies, rng, history, opening, track_no, set_id)
    for melody_id in order:
        try:
            hook, key = hooks.from_rtttl(melodies[melody_id], melody_id, profile.bpm, profile.root, profile.mode,
                                         register)
        except hooks.HookError:
            continue
        if fingerprint(hook.notes) != last.get("hook_fingerprint"):
            yield hook, key


def _motif(style: kn.Style, key: Key, rng: random.Random, last_fp: Optional[str]) -> Hook:
    """Мотив лида (PR-2) фигуры стиля, не совпадающий фрагментом с хуком прошлого трека."""
    figure = LEAD_GENERATORS[style.lead_figures[0]]
    for _ in range(8):
        motif = Hook(figure(style, key, rng), lead.MOTIF_BARS, None)
        if fingerprint(motif.notes) != last_fp:
            break
    return motif


Progression = Callable[[Sequence[PitchEvent]], Tuple[int, ...]]


def _arrange(style: kn.Style, spec: FormSpec, motif: Hook, key: Key, rng: random.Random, lead_synth: str,
             history: Sequence[Mapping] = (), progression: Optional[Progression] = None):
    """Лид, прогрессия под мотив и пэд под лидом; пэд не помещается под лидом — ``ValueError``. ``progression`` —
    ступени по нотам дропа (гармония материала); None — шаблон стиля под хук (``harmony.fit_progression``)."""
    lead_part = _lead(style, spec, motif, key, lead_synth)
    drop = [e for e in hooks.develop(motif, "drop", SECTION_BARS, key) if e.beat < LOOP_BEATS]
    if progression is None:
        recent = recent_values(history, "progression")
        degrees = harmony.fit_progression(style, key, drop, CHORD_BARS * BEATS_PER_BAR, rng, recent)
    else:
        degrees = progression(drop)
    pad_top = min(style.registers["pad"][1], min(e.midi for e in lead_part.pitches) - PAD_GAP)
    return (lead_part, degrees) + _pad_chords(style, key, degrees, pad_top)


def _pad_chords(style: kn.Style, key: Key, degrees: Sequence[int], top: int) -> Tuple[Tuple[int, int], Tuple]:
    """(регистр пэда, аккорды): обычный низ, а трезвучие в окне не помещается — низ на полутон ниже до коридора стиля
    (``knowledge.PAD_WIDEN``); окно в 12 нот содержит любой звук лада, так что дальше ``ValueError`` только у лида,
    поднятого выше ``hook_register``. Треки, которым хватало обычного окна, звучат как прежде."""
    lows = range(_pad_floor(style), style.registers["pad"][0] - 1, -1)
    for low in lows[:-1]:
        try:
            return (low, top), harmony.pad_chords(style, key, degrees, (low, top))
        except ValueError:
            continue
    return (lows[-1], top), harmony.pad_chords(style, key, degrees, (lows[-1], top))  # не помещается и в коридоре


def _theme_hook(style: kn.Style, spec: FormSpec, profile: ThemeProfile, melodies: Mapping[str, str],
                rng: random.Random, history: Sequence[Mapping], track_no: int, lead_synth: str,
                set_id: Optional[str] = None):
    """Первая мелодия темы (:func:`hook_candidates`), под которой складываются гармония и пэд: (хук, тональность,
    аранжировка) или None."""
    for candidate, key in hook_candidates(profile, melodies, rng, history, opening=track_no == 1, track_no=track_no,
                                          set_id=set_id):
        try:
            return candidate, key, _arrange(style, spec, candidate, key, rng, lead_synth, history), None
        except ValueError:
            continue
    return None


def _from_material(style: kn.Style, spec: FormSpec, material_id: Optional[str],
                   materials: Optional[Mapping[str, ScoreMaterial]], profile: ThemeProfile, rng: random.Random,
                   lead_synth: str):
    """Хук, лид, гармония и тоны баса трека из материала партитуры плана (ADR-0154 §3.3): хук —
    ``hook.from_material``, ступени — ``harmony.from_material``, тоны баса тактов петли — ``bass.material_tones`` по
    той же фразе и тому же множителю темпа. Материала нет или он не годится (хук, лад, регистр пэда) — None с
    причиной в логе (I12), трек идёт по хуку темы."""
    if not material_id:
        return None
    material = (materials or {}).get(material_id)
    if material is None:
        _LOG.info("🎵 [music v2] material=%s нет среди переданных — хук темы", material_id)
        return None
    try:
        motif, key = hooks.from_material(material, profile.bpm, profile.root, profile.mode, hook_register(style))
        scale = hooks.material_scale(material, profile.bpm)
        phrase = hooks.pick_phrase(material)

        def progression(drop: Sequence[PitchEvent]) -> Tuple[int, ...]:
            return harmony.from_material(material, phrase, key, drop, CHORD_BARS * BEATS_PER_BAR,
                                         max(1, motif.bars // CHORD_BARS), scale)

        arranged = _arrange(style, spec, motif, key, rng, lead_synth, progression=progression)
        tones = bass.material_tones(style, material, phrase, arranged[1], CHORD_BARS * BEATS_PER_BAR, scale)
    except ValueError as exc:  # HookError — тоже ValueError
        _LOG.info("🎵 [music v2] material=%s отказ: %s — хук темы", material_id, exc)
        return None
    _LOG.info("🎵 [music v2] material=%s ступени %s бас %s", material_id, harmony.progression_name(arranged[1]),
              " ".join(f"{t.anchor}{'+-'[t.approach < 0] if t.approach else ''}" for t in tones))
    return motif, key, arranged, tones


def _pad(style: kn.Style, family: str, bar_chords: List[Tuple[int, Chord]], key: Key,
         register: Tuple[int, int], history: Sequence[Mapping], seed: str) -> Tuple[str, Part]:
    """Рисунок пэда и партия: рисунок и синт семьи под него — со штрафом за недавние, свой ГСЧ сида на ось."""
    figure = weighted_pick(style.pad_figures, recent_values(history, "pad_figure"), random.Random(f"{seed}:pad_figure"))
    synth = mix.pad_timbre(style, family, figure, recent_values(history, "pad"), random.Random(f"{seed}:pad"))
    return figure, PAD_GENERATORS[figure](style, key, bar_chords, synth, register)


def _bass(style: kn.Style, family: str, bar_chords: List[Tuple[int, Chord]], key: Key,
          history: Sequence[Mapping], seed: str, tones: Optional[Sequence[bass.BassTone]] = None) -> Tuple[str, Part]:
    """Рисунок баса и партия: рисунок и синт семьи — со штрафом за недавние, свой ГСЧ сида на ось. ``tones`` — тоны
    тактов петли из материала (``bass.material_tones``), на такт формы — по кругу петли; None — тоника и квинта."""
    figure = weighted_pick(mix.bass_figures(style, family), recent_values(history, "bass_figure"),
                           random.Random(f"{seed}:bass_figure"))
    synth = weighted_pick(mix.bass_synths(style, family, figure), recent_values(history, "bass"),
                          random.Random(f"{seed}:bass"))
    by_bar = {bar: tones[bar % len(tones)] for bar, _chord in bar_chords} if tones else None
    return figure, BASS_GENERATORS[figure](style, key, bar_chords, synth, style.registers["bass"], by_bar)


def _kit(style: kn.Style, history: Sequence[Mapping], rng: random.Random) -> str:
    """Каркас ударных стиля: не прошлого трека, со штрафом за недавние."""
    recent = recent_values(history, "kit")
    options = [k for k in style.kits if not recent or k != recent[0]]
    return weighted_pick(options, recent, rng)


def compose(plan: SetPlan, track_no: int, *, melodies: Optional[Mapping[str, str]] = None,
            history: Sequence[Mapping] = (), deck: str = "A",
            materials: Optional[Mapping[str, ScoreMaterial]] = None) -> Track:
    """Трек ``track_no`` сета по плану. ``melodies`` — ``{id: rtttl}`` для ``plan.profile.hook_ids``; ``history`` —
    строки ``music_history`` (свежие первыми, ``MusicHistory.recent``); ``materials`` — ``{material_id:
    ScoreMaterial}`` для ``TrackPlan.material`` (ADR-0154).

    Темп — сета, тоника — ``plan.root(track_no)``, энергия — ``plan.track(track_no).energy``, стиль —
    ``knowledge.STYLES[plan.style]``. План назвал материал — хук и гармония автора из него (:func:`_from_material`);
    иначе (или материал не годится) хук — первая мелодия темы, под которой складываются гармония и пэд; ни одной —
    мотив лида (PR-2).
    """
    step = plan.track(track_no)
    style = plan.table
    template = track_template(style, step, track_no, history, random.Random(f"{plan.seed}:{track_no}:template"))
    spec = form_spec(style, template)
    profile = replace(plan.profile, bpm=plan.bpm, root=plan.root(track_no))
    rng = random.Random(f"{plan.seed}:{track_no}")
    seed = f"{plan.seed}:{track_no}"
    lead_synth = mix.role_timbre(style, plan.family, "lead", recent_values(history, "lead"),
                                 random.Random(f"{seed}:lead"))
    found = (_from_material(style, spec, step.material, materials, profile, rng, lead_synth)
             or _theme_hook(style, spec, profile, melodies or {}, rng, history, track_no, lead_synth, plan.set_id))
    if found is None:
        key = Key(profile.root, profile.mode)
        motif = _motif(style, key, rng, history[0].get("hook_fingerprint") if history else None)
        track_hook, arranged, tones = None, _arrange(style, spec, motif, key, rng, lead_synth, history), None
    else:
        track_hook = motif = found[0]
        key, arranged, tones = found[1], found[2], found[3]
    lead_part, degrees, pad_register, chords = arranged
    form = _form(style, spec, step.energy)
    axis = {name: random.Random(f"{plan.seed}:{track_no}:{name}") for name in ("kit", "sample", "loop", "fx")}
    kit = _kit(style, history, axis["kit"])
    perc = samples.perc_pool(key, history, axis["sample"]) if "sample" in style.layer_sections else ()
    loop = samples.pick(style.loop_roles, key, history, "sample", axis["loop"])
    fx = samples.pick(samples.FX_ROLES, key, history, "fx", axis["fx"])
    kick = step.kick or pick_kick(style, history, random.Random(f"{plan.seed}:{track_no}:kick"))
    drums = _drums(style, form, rhythm.swing_offset_ms(plan.swing, plan.bpm), kit, kick)
    bass_figure, bass_part = _bass(style, plan.family, _bar_chords(spec, "bass", chords), key, history, seed, tones)
    figure, pad_part = _pad(style, plan.family, _bar_chords(spec, "pad", chords), key, pad_register, history, seed)
    layers = {"sample": lambda: samples.perc_part(style, perc, kit, axis["sample"]),
              "loop": lambda: LOOP_GENERATORS[style.loop_figure](loop, axis["loop"]),
              "fx": lambda: samples.fx_part(fx, SECTION_BARS)}  # слой без секций стиля в трек не идёт
    parts, track_mix = mix.mix_parts(style, {
        **drums, "bass": bass_part, "pad": pad_part, "lead": lead_part,
        **{role: make() for role, make in layers.items() if role in style.layer_sections}}, form, figure)
    prog = harmony.progression_name(degrees)
    sha = hashlib.sha256(repr((plan.bpm, key, step, sorted(parts.items()), chords)).encode()).hexdigest()[:8]
    return Track(
        track_id=f"{plan.set_id}:{track_no:02d}:{deck}:{sha}", seed=plan.seed, bpm=plan.bpm, key=key, form=form,
        parts=parts, harmony=Harmony({n: chords for n, _b, _e, _r in spec}), hook=track_hook,
        mix=track_mix,
        energy=step.energy, transition_in=transition(style), transition_out=transition(style),
        history_key=HistoryKey(kit, prog, track_hook.source if track_hook else None, loop, key.root,
                               fingerprint(motif.notes), fx, ",".join(perc), figure, bass_figure, template, plan.genre,
                               plan.style, plan.family),
    )


def club_track(seed: int, *, set_id: str = "v2", deck: str = "A", track_no: int = 1) -> Track:
    """Трек без темы и без RTTTL-библиотеки (мотив лида) — для проверок плеера (PR-4) одним сидом."""
    rng = random.Random(seed)
    lo, hi = kn.STYLES[kn.DEFAULT_STYLE].bpm
    profile = ThemeProfile("", kn.DEFAULT_STYLE, rng.randint(lo, hi), rng.randrange(12), "minor", (), None)
    return compose(seeded_plan(profile, seed, set_id=set_id), track_no, deck=deck)


__all__ = ["BASS_GENERATORS", "FormSpec", "LEAD_GENERATORS", "LOOP_GENERATORS", "PAD_GENERATORS", "SECTION_BARS", "club_track",
           "compose", "form_spec", "hook_candidates", "hook_order", "hook_register", "opening_order", "part_order",
           "track_template", "transition", "upcoming_hooks"]
