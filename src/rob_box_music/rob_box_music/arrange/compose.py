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
Гармония — под звучащую мелодию каждой секции (аудит 07.10 Ф1, :func:`_harmonize`): аккорд на такт, Витерби по
выученной таблице (``harmony.melody_progression``), петля длиной в хук, build — педаль, break — аккорды вдвое длиннее
(``knowledge.HOOK_HARMONY``, ``SECTION_HARMONY``); бас в оффбит, пэд с голосоведением по всему треку.
План назвал материал партитуры (``TrackPlan.material``, ADR-0154 PR-3) — хук и ступени слотов из него
(``hook.from_material``, ``harmony.from_material``: аккорды автора, проверенные под звучащую мелодию), тоны баса —
из басового голоса автора (``bass.material_tones``, PR-4), ``drop2`` — ритм хука с контуром следующей фразы; не
годится — путь выше без изменений.
Тема целиком (ADR-0154 PR-7): в секции ``knowledge.THEME_SECTION`` (первый дроп) звучит вся тема — тематическая
секция материала фраза за фразой или вся RTTTL-мелодия до ``knowledge.THEME_MAX_BARS`` тактов (:func:`theme_limit`);
секция растёт на длину темы (:func:`theme_form`, трек не длиннее ``knowledge.TRACK_MAX_BARS``), гармония и бас — на
всю тему: аккорды и бас автора (материал) или Витерби по мелодии (RTTTL, ``harmony.melody_progression``).

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
from typing import Callable, Dict, Iterator, List, Mapping, NamedTuple, Optional, Sequence, Tuple

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
#: Уровень партии до ``mix.mix_parts`` (он ставит уровень роли из ``Style.role_level_db``).
_UNLEVELED = 0.0
PAD_GAP = 3  # верх пэда ниже лида на ≥ 3 полутона (ADR-0149 §3.6)


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


def theme_limit(spec: FormSpec) -> int:
    """Потолок темы целиком (тактов клуба) в форме ``spec``: ``knowledge.THEME_MAX_BARS``, но трек с растянутой
    секцией темы не длиннее ``knowledge.TRACK_MAX_BARS``; формы без ``knowledge.THEME_SECTION`` — 0 (темы нет)."""
    if all(name != kn.THEME_SECTION for name, _b, _e, _r in spec):
        return 0
    rest = sum(bars for name, bars, _e, _r in spec if name != kn.THEME_SECTION)
    return max(0, min(kn.THEME_MAX_BARS, kn.TRACK_MAX_BARS - rest))


def theme_form(spec: FormSpec, theme_bars: int) -> FormSpec:
    """Форма с секцией ``knowledge.THEME_SECTION`` не короче темы: длина формы — кратная ``knowledge.FORM_BARS_STEP``
    (период ударных), лишние такты секции — хук. Тема не длиннее секции — форма как есть."""
    rest = sum(bars for name, bars, _e, _r in spec if name != kn.THEME_SECTION)
    out = []
    for name, bars, energy, roles in spec:
        if name == kn.THEME_SECTION and theme_bars > bars:
            bars = theme_bars + (-(rest + theme_bars)) % kn.FORM_BARS_STEP
        out.append((name, bars, energy, roles))
    return tuple(out)


Degrees = Callable[[Sequence[PitchEvent], int], Tuple[int, ...]]
Tones = Callable[[Sequence[int]], Tuple[bass.BassTone, ...]]


class Harmonizer(NamedTuple):
    """Путь гармонии трека (аудит 07.10 Ф1): ступени петли хука и темы целиком под их мелодию — ``(ноты, слотов)
    -> ступени`` по слотам ``knowledge.HOOK_HARMONY.slot_bars``; тоны баса автора по ступеням (материал) — по такту,
    у RTTTL и мотива их нет (тоника и квинта)."""

    loop: Degrees
    theme: Degrees
    loop_tones: Optional[Tones] = None
    theme_tones: Optional[Tones] = None


class Arranged(NamedTuple):
    """Лид, петля хука (ступени по слотам — прогрессия трека в истории), регистр пэда, аккорд и тон баса автора
    каждого такта формы (тона нет — тоника и квинта)."""

    lead: Part
    loop: Tuple[int, ...]
    register: Tuple[int, int]
    chords: Dict[int, Chord]
    tones: Dict[int, bass.BassTone]


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


def _progression(spec: FormSpec, chords: Mapping[int, Chord]) -> Dict[str, Tuple[Chord, ...]]:
    """Аккорды секций по тактам (``Harmony.progression``) из аккорда такта формы."""
    return {name: tuple(chords[bar] for bar in range(start, start + bars)) for start, bars, name, _r in _sections(spec)}


def _bar_chords(spec: FormSpec, role: str, progression: Mapping[str, Sequence[Chord]]) -> List[Tuple[int, Chord]]:
    """(такт формы, аккорд) там, где звучит ``role``; ``progression`` — аккорды секций по тактам (``Harmony``)."""
    return [(start + i, chord) for start, _bars, name, roles in _sections(spec) if role in roles
            for i, chord in enumerate(progression[name])]


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
                    track_no: int = 1, set_id: Optional[str] = None, theme_max: int = 0) -> Iterator[Tuple[Hook, Key]]:
    """Годные мелодии темы: несыгранные — в порядке сида, недавние (история сета и прошлых сетов, I17) — в конце,
    давние раньше свежих; хук прошлого трека (мелодия или фрагмент) подряд не повторяется. Первый трек сета
    (``opening``) — :func:`opening_order`: хук №1 темы первым (#3427: порядок ``search.theme_hooks``), если он не
    звучал в последних сетах (#3399). Недавнее — :func:`diversity.recent_hooks` (``set_id`` — сет трека: его строки
    по записи, прошлых сетов — по мелодии, версии с общим контуром начала — одна мелодия).
    Хуки найдены по словам темы (``profile.theme_hooks``) — несыгранные идут в порядке профиля, а не сида: сет
    обходит найденные по очереди (у темы-перечисления — по кругу частей, ``search.round_robin``), повтор — только
    когда несыгранные кончились (06.10: 50 треков по кругу из трёх хуков). Тема-перечисление
    (``profile.theme_parts``) — очередь :func:`part_order` трека ``track_no`` и на первом треке тоже.
    ``theme_max`` > 0 — хук с темой целиком до ``theme_max`` тактов (``hook.with_theme``)."""
    last = history[0] if history else {}
    register = hook_register(kn.STYLES[profile.style])
    order = hook_order(profile, melodies, rng, history, opening, track_no, set_id)
    for melody_id in order:
        try:
            hook, key = hooks.from_rtttl(melodies[melody_id], melody_id, profile.bpm, profile.root, profile.mode,
                                         register, theme_max)
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


def _slot_beats() -> float:
    return kn.HOOK_HARMONY.slot_bars * BEATS_PER_BAR


def _per_bar(degrees: Sequence[int], factor: int) -> List[int]:
    return [d for d in degrees for _ in range(factor)]


def _section_degrees(key: Key, line: Sequence[PitchEvent], name: str, bars: int) -> List[int]:
    """Ступени тактов секции развития под её звучащую мелодию ``line`` (аудит Ф1, П2): педаль
    (``knowledge.PEDAL``) — одна ступень из ``knowledge.PEDAL_DEGREES`` на всю секцию; иначе Витерби слотами
    ``slot_bars × knowledge.SECTION_HARMONY[name]`` (``break`` — хук вдвое медленнее, аккорды вдвое длиннее)."""
    rule = kn.SECTION_HARMONY.get(name, 1)
    if rule == kn.PEDAL:
        return [harmony.viterbi(key, line, bars * BEATS_PER_BAR, 1, degrees=kn.PEDAL_DEGREES)[0]] * bars
    slot = kn.HOOK_HARMONY.slot_bars * rule
    return _per_bar(harmony.viterbi(key, line, slot * BEATS_PER_BAR, -(-bars // slot)), slot)[:bars]


def _harmonize(spec: FormSpec, motif: Hook, key: Key, harm: Harmonizer
               ) -> Tuple[Tuple[int, ...], Dict[int, int], Dict[int, bass.BassTone]]:
    """(петля хука по слотам, ступень такта формы, тон баса автора такта) — одна гармонизация под звучащую мелодию
    каждой секции (аудит 07.10 Ф1): петля — под хук (``harm.loop``; петля длиной в хук, и хук в 4 такта не уходит во
    второй половине на чужие аккорды), тема целиком — под тему (``harm.theme``); секция, где звучит хук по кругу
    (drop, остаток секции темы, drop2 — хук и терции над ним: гармонизуется хук, такт с b9 терции — ступень без неё,
    :meth:`_BarPlan.voiced`), — петля от начала секции; секция
    развития (build, break, ответ drop2 из материала) — под свою мелодию (:func:`_section_degrees`); секция без лида —
    петля по такту формы."""
    slot = kn.HOOK_HARMONY.slot_bars
    loop = harm.loop(motif.notes, max(1, motif.bars // slot))
    loop_bars = _per_bar(loop, slot)
    loop_tones = harm.loop_tones(loop) if harm.loop_tones else ()
    theme_slots = harm.theme(motif.theme, motif.theme_bars // slot) if motif.theme else ()
    theme = _per_bar(theme_slots, slot)
    theme_tones = harm.theme_tones(theme_slots) if theme_slots and harm.theme_tones else ()
    plain = replace(motif, theme=(), theme_bars=0)
    plan = _BarPlan({}, {})
    for start, bars, name, roles in _sections(spec):
        if "lead" not in roles:
            plan.put(range(start, start + bars), loop_bars, loop_tones, by_form_bar=True)
            continue
        span = len(theme) if name == kn.THEME_SECTION else 0
        plan.put(range(start, start + span), theme, theme_tones)
        line = hooks.develop(plain, name, bars, key)
        hook_line = hooks.develop(plain, "drop", bars, key)
        if span or set(hook_line) <= set(line):  # хук по кругу, в т. ч. с добавочным голосом (терции drop2)
            plan.put(range(start + span, start + bars), loop_bars, loop_tones)
            if not span and len(line) > len(hook_line):
                plan.voiced(key, start, bars, hook_line, line)
        else:
            plan.put(range(start, start + bars), _section_degrees(key, line, name, bars), ())
    return tuple(loop), plan.degrees, plan.tones


class _BarPlan(NamedTuple):
    """Ступень и тон баса автора по такту формы (собирается :func:`_harmonize`)."""

    degrees: Dict[int, int]
    tones: Dict[int, bass.BassTone]

    def put(self, bars: range, chords: Sequence[int], tones: Sequence[bass.BassTone], by_form_bar: bool = False
            ) -> None:
        """Такты ``bars`` — ``chords``/``tones`` по кругу: от начала отрезка или по номеру такта формы."""
        for i, bar in enumerate(bars):
            k = bar if by_form_bar else i
            self.degrees[bar] = chords[k % len(chords)]
            if tones:
                self.tones[bar] = tones[k % len(tones)]

    def voiced(self, key: Key, start: int, bars: int, hook_line: Sequence[PitchEvent], line: Sequence[PitchEvent]
               ) -> None:
        """Хук с добавочным голосом (терции drop2) над петлёй: такт, где какой-то голос дал малую нону на сильной доле,
        а есть ступень без неё ни в одном голосе, — эта ступень (лучшая под хук); иначе — петля, как есть."""
        for at in range(bars):
            def in_bar(notes: Sequence[PitchEvent]) -> List[PitchEvent]:
                return [replace(e, beat=e.beat - at * BEATS_PER_BAR) for e in notes
                        if int(e.beat // BEATS_PER_BAR) == at]
            voices, hook_bar = in_bar(line), in_bar(hook_line)
            if not harmony.strong_b9(key, voices, self.degrees[start + at]):
                continue
            clean = [d for d in range(7) if d not in harmony.diminished(key) and not harmony.strong_b9(key, voices, d)]
            if clean:
                self.degrees[start + at] = harmony.viterbi(key, hook_bar, BEATS_PER_BAR, 1, degrees=clean)[0]
                self.tones.pop(start + at, None)  # тон баса автора — к его аккорду, у нового — прима


def _arrange(style: kn.Style, spec: FormSpec, motif: Hook, key: Key, lead_synth: str, harm: Harmonizer) -> Arranged:
    """Лид, гармония под звучащую мелодию секций (:func:`_harmonize`) и пэд под лидом; пэд не помещается —
    ``ValueError``."""
    lead_part = _lead(style, spec, motif, key, lead_synth)
    loop, degrees, tones = _harmonize(spec, motif, key, harm)
    pad_top = min(style.registers["pad"][1], min(e.midi for e in lead_part.pitches) - PAD_GAP)
    bars = sorted(degrees)
    register, chords = _pad_chords(style, key, [degrees[b] for b in bars], pad_top)
    return Arranged(lead_part, loop, register, dict(zip(bars, chords)), tones)


def _pad_chords(style: kn.Style, key: Key, degrees: Sequence[int], top: int) -> Tuple[Tuple[int, int], Tuple]:
    """(регистр пэда, аккорды последовательности ``degrees`` с голосоведением ``harmony.voice_chain``): обычный низ, а
    трезвучие в окне не помещается — низ на полутон ниже до коридора стиля (``knowledge.PAD_WIDEN``); окно в 12 нот
    содержит любой звук лада, так что дальше ``ValueError`` только у лида, поднятого выше ``hook_register``. Треки,
    которым хватало обычного окна, звучат в нём."""
    lows = range(_pad_floor(style), style.registers["pad"][0] - 1, -1)
    for low in lows[:-1]:
        try:
            return (low, top), harmony.voice_chain(style, key, degrees, (low, top))
        except ValueError:
            continue
    return (lows[-1], top), harmony.voice_chain(style, key, degrees, (lows[-1], top))  # не помещается и в коридоре


def _arrange_theme(style: kn.Style, spec: FormSpec, motif: Hook, key: Key, lead_synth: str, harm: Harmonizer):
    """(хук, форма, аранжировка): хук с темой — форма под тему (:func:`theme_form`) и гармония на всю тему; гармония
    темы не складывается — хук без темы в исходной форме, причина в логе. Пэд не помещается под лидом без темы —
    ``ValueError``, как у :func:`_arrange`."""
    if motif.theme:
        form = theme_form(spec, motif.theme_bars)
        try:
            return motif, form, _arrange(style, form, motif, key, lead_synth, harm)
        except ValueError as exc:
            _LOG.info("🎵 [music v2] hook melody=%s тема без гармонии: %s — хук без темы", motif.source, exc)
            motif = replace(motif, theme=(), theme_bars=0)
    return motif, spec, _arrange(style, spec, motif, key, lead_synth, harm)


def melody_harmonizer(style: kn.Style, key: Key, history: Sequence[Mapping], rng: random.Random) -> Harmonizer:
    """Гармония без аккордов автора (хук и тема RTTTL, мотив): ``harmony.melody_progression`` — петля хука по кругу
    с A13 по истории (``music_history.progression``), тема — цепочкой; переходы петель стиля — априорный бонус."""
    recent = recent_values(history, "progression")
    beats = _slot_beats()

    def loop(notes: Sequence[PitchEvent], slots: int) -> Tuple[int, ...]:
        return harmony.melody_progression(key, notes, beats, slots, ring=True, progressions=style.progressions,
                                          recent=recent, rng=rng)

    def theme(notes: Sequence[PitchEvent], slots: int) -> Tuple[int, ...]:
        return harmony.melody_progression(key, notes, beats, slots, progressions=style.progressions)
    return Harmonizer(loop, theme)


def _theme_hook(style: kn.Style, spec: FormSpec, profile: ThemeProfile, melodies: Mapping[str, str],
                rng: random.Random, history: Sequence[Mapping], track_no: int, lead_synth: str,
                set_id: Optional[str] = None):
    """Первая мелодия темы (:func:`hook_candidates`), под которой складываются гармония и пэд: (хук, тональность,
    аранжировка, форма) или None. Гармония — Витерби под мелодию (:func:`melody_harmonizer`)."""
    for candidate, key in hook_candidates(profile, melodies, rng, history, opening=track_no == 1, track_no=track_no,
                                          set_id=set_id, theme_max=theme_limit(spec)):
        try:
            motif, form, arranged = _arrange_theme(style, spec, candidate, key, lead_synth,
                                                   melody_harmonizer(style, key, history, rng))
        except ValueError:
            continue
        return motif, key, arranged, form
    return None


def _from_material(style: kn.Style, spec: FormSpec, material_id: Optional[str],
                   materials: Optional[Mapping[str, ScoreMaterial]], profile: ThemeProfile,
                   lead_synth: str, melodies: Mapping[str, str]):
    """Хук, лид, гармония и тоны баса трека из материала партитуры плана (ADR-0154 §3.3): хук —
    ``hook.from_material``, ступени петли — ``harmony.from_material`` по фразе хука, тоны баса — ``bass.material_tones``
    по той же фразе и тому же множителю темпа; тема целиком — то же по тематической секции (``hook.theme_span``).
    Главный мотив — по RTTTL-эталонам темы (``profile.theme_hooks``, ``hook.for_theme``): голос и такт, где совпал
    их контур.
    Материала нет или он не годится (хук, лад, регистр пэда) — None с причиной в логе (I12), трек идёт по хуку
    темы."""
    if not material_id:
        return None
    material = (materials or {}).get(material_id)
    if material is None:
        _LOG.info("🎵 [music v2] material=%s нет среди переданных — хук темы", material_id)
        return None
    try:
        material, anchor = hooks.for_theme(material, [melodies[i] for i in profile.theme_hooks if i in melodies])
        motif, key = hooks.from_material(material, profile.bpm, profile.root, profile.mode, hook_register(style),
                                         theme_limit(spec), anchor)
        scale = hooks.material_scale(material, profile.bpm)
        phrase = hooks.pick_phrase(material, anchor)
        span = hooks.theme_span(material, phrase)
        beats = _slot_beats()
        harm = Harmonizer(
            lambda notes, slots: harmony.from_material(material, phrase, key, notes, beats, slots, scale),
            lambda notes, slots: harmony.from_material(material, span, key, notes, beats, slots, scale),
            lambda degrees: bass.material_tones(style, material, phrase, degrees, beats, scale),
            lambda degrees: bass.material_tones(style, material, span, degrees, beats, scale))
        motif, form, arranged = _arrange_theme(style, spec, motif, key, lead_synth, harm)
    except ValueError as exc:  # HookError — тоже ValueError
        _LOG.info("🎵 [music v2] material=%s отказ: %s — хук темы", material_id, exc)
        return None
    _LOG.info("🎵 [music v2] material=%s ступени %s тема %d тактов", material_id,
              harmony.progression_name(arranged.loop), motif.theme_bars)
    return motif, key, arranged, form


def _pad(style: kn.Style, family: str, bar_chords: List[Tuple[int, Chord]], key: Key,
         register: Tuple[int, int], history: Sequence[Mapping], seed: str) -> Tuple[str, Part]:
    """Рисунок пэда и партия: рисунок и синт семьи под него — со штрафом за недавние, свой ГСЧ сида на ось."""
    figure = weighted_pick(style.pad_figures, recent_values(history, "pad_figure"), random.Random(f"{seed}:pad_figure"))
    synth = mix.pad_timbre(style, family, figure, recent_values(history, "pad"), random.Random(f"{seed}:pad"))
    return figure, PAD_GENERATORS[figure](style, key, bar_chords, synth, register)


def _bass(style: kn.Style, family: str, bar_chords: List[Tuple[int, Chord]], key: Key,
          history: Sequence[Mapping], seed: str, by_bar: Optional[Mapping[int, bass.BassTone]] = None
          ) -> Tuple[str, Part]:
    """Рисунок баса и партия: рисунок и синт семьи — со штрафом за недавние, свой ГСЧ сида на ось. ``by_bar`` — тон
    такта формы из материала (:func:`_bar_tones`); None — тоника и квинта."""
    figure = weighted_pick(mix.bass_figures(style, family), recent_values(history, "bass_figure"),
                           random.Random(f"{seed}:bass_figure"))
    synth = weighted_pick(mix.bass_synths(style, family, figure), recent_values(history, "bass"),
                          random.Random(f"{seed}:bass"))
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
    found = (_from_material(style, spec, step.material, materials, profile, lead_synth, melodies or {})
             or _theme_hook(style, spec, profile, melodies or {}, rng, history, track_no, lead_synth, plan.set_id))
    if found is None:
        key = Key(profile.root, profile.mode)
        motif = _motif(style, key, rng, history[0].get("hook_fingerprint") if history else None)
        track_hook = None
        arranged = _arrange(style, spec, motif, key, lead_synth, melody_harmonizer(style, key, history, rng))
    else:
        track_hook = motif = found[0]
        key, arranged, spec = found[1:]
    progression = _progression(spec, arranged.chords)
    form = _form(style, spec, step.energy)
    axis = {name: random.Random(f"{plan.seed}:{track_no}:{name}") for name in ("kit", "sample", "loop", "fx")}
    kit = _kit(style, history, axis["kit"])
    perc = samples.perc_pool(key, history, axis["sample"]) if "sample" in style.layer_sections else ()
    loop = samples.pick(style.loop_roles, key, history, "sample", axis["loop"])
    fx = samples.pick(samples.FX_ROLES, key, history, "fx", axis["fx"])
    kick = step.kick or pick_kick(style, history, random.Random(f"{plan.seed}:{track_no}:kick"))
    drums = _drums(style, form, rhythm.swing_offset_ms(plan.swing, plan.bpm), kit, kick)
    bass_figure, bass_part = _bass(style, plan.family, _bar_chords(spec, "bass", progression), key, history, seed,
                                   arranged.tones or None)
    figure, pad_part = _pad(style, plan.family, _bar_chords(spec, "pad", progression), key, arranged.register,
                            history, seed)
    layers = {"sample": lambda: samples.perc_part(style, perc, kit, axis["sample"]),
              "loop": lambda: LOOP_GENERATORS[style.loop_figure](loop, axis["loop"]),
              "fx": lambda: samples.fx_part(fx, SECTION_BARS)}  # слой без секций стиля в трек не идёт
    parts, track_mix = mix.mix_parts(style, {
        **drums, "bass": bass_part, "pad": pad_part, "lead": arranged.lead,
        **{role: make() for role, make in layers.items() if role in style.layer_sections}}, form, figure)
    prog = harmony.progression_name(arranged.loop)
    chords = sorted((name, tuple((c.degree, c.voicing) for c in cs)) for name, cs in progression.items())
    sha = hashlib.sha256(repr((plan.bpm, key, step, sorted(parts.items()), chords)).encode()).hexdigest()[:8]
    return Track(
        track_id=f"{plan.set_id}:{track_no:02d}:{deck}:{sha}", seed=plan.seed, bpm=plan.bpm, key=key, form=form,
        parts=parts, harmony=Harmony(progression), hook=track_hook,
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


__all__ = ["Arranged", "BASS_GENERATORS", "FormSpec", "Harmonizer", "LEAD_GENERATORS", "LOOP_GENERATORS",
           "PAD_GENERATORS", "SECTION_BARS", "club_track", "compose", "form_spec", "hook_candidates", "hook_order", "hook_register", "opening_order", "part_order",
           "melody_harmonizer", "theme_form", "theme_limit", "track_template", "transition", "upcoming_hooks"]
