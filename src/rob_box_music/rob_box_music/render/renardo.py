"""``render(track, deck) -> Program`` — одна чистая функция рендера модели в Renardo (ADR-0149 §3.2).

Подмножество Renardo: шесть присваиваний плеерам деки. Ударные — ``play("<сетка>")``
по 16-м, тональные — список MIDI по 16-м (``None`` — пауза, кортеж — аккорд) в
``Scale.chromatic`` с ``root=0, oct=0``. Секции — ``amp=var([...], [доли])`` по
``Section.roles`` от начала формы (``start=Clock.next_bar()`` — доля, где встают плееры), уровень ``amp`` — из
уровня роли по модели громкости (``arrange.mix.level_amp``); акценты ×
сайдчейн-огибающая ролей ``Mix.duck_roles`` — ``amplify=[...]`` (период списка свёрнут); бочка — ``sample=`` из
``knowledge.KICK_SOUNDS``; LPF-свип ``Mix.lpf`` — ``lpf=var([...], [доли])`` по долям (PR-7);
свинг ``offset_ms`` — ``delay=[...]`` в долях
(глобальный ``Clock.swing`` не используется: сайдчейн не должен уехать от бочки, ADR-0149 §3.4).
Стерео ``Mix.stereo`` (§3.9): ударные — ``pan=[...]`` со сменой стороны на каждом ударе; два голоса тональной роли —
вложенные группы ``pan=((-w, w),), pshift=((0, d),), delay=((0, Хаас),)`` (Renardo раскрывает их на КАЖДУЮ ноту
аккорда: ``Player.send_osc_message``, проверено пробой на роботе). ``room2`` не используется: глушит выход
(``knowledge.ROLE_STEREO``).
Список нот и рисунок ударных свёрнуты до наименьшего периода в тактах, на котором модель
совпадает во всех звучащих секциях, — поэтому программа короткая, а события нот равны модели
(тест на ``render.events``); fill-ы перед дропом удлиняют период рисунка до формы.

Сэмплы (``sample``/``loop``/``fx``, PR-3d) — ``loop('<путь>')`` из ``knowledge.SAMPLE_CATALOG`` приёмами DJ_Dave
(:func:`_sample_line`): луп — нарезка на восьмые (``chop``), psr — файл на каждую 16-ю из пула (``c1.buf``), FX —
удар, звучащий один раз. Путь — от папки лупов пака 0 (``repr``).

Не умеет (честная ошибка :class:`RenderError`, а не тихая потеря):
ноты вне сетки 16-х, ноту в секции, где роль молчит.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

from .. import knowledge as kn
from ..arrange.mix import alternate_pan, duck_envelope, voice_amp
from ..model import BEATS_PER_BAR, SAMPLE_ROLES, STEPS_PER_BAR, Part, Stereo, Track, validate
from .program import Program

STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR
#: Начало формы для ``var`` секций: ``TimeVar`` Renardo считает доли от ``start``, плееры встают на ``next_bar`` —
#: секции идут от первой доли трека, где бы он ни встал (стык и блэнд PR-8 ставят трек не на ``k·форма``).
FORM_START = "Clock.next_bar()"
#: Роль → индекс слота в ``knowledge.DECK_SLOTS[deck]``: d-слоты ударным, p-слоты тональным.
ROLE_SLOT: Dict[str, int] = {"kick": 0, "hats": 1, "clap": 2, "perc": 2, "bass": 3, "pad": 4, "lead": 5,
                             "sample": 6, "fx": 7, "loop": 8}
_UNSET = object()

Cell = Tuple[Optional[Tuple[int, ...]], float, int]  # (ноты шага, sus в долях, акцент)


class RenderError(ValueError):
    """Модель валидна, но рендер её не выражает — громко, до exec (I25)."""


def _num(value: float) -> str:
    rounded = round(float(value), 3)
    return str(int(rounded)) if rounded.is_integer() else f"{rounded:.3f}".rstrip("0").rstrip(".")


def _list(values: Sequence[object]) -> str:
    return "[" + ", ".join(str(v) for v in values) + "]"


def _active_steps(track: Track, role: str) -> List[bool]:
    out: List[bool] = []
    for sec in track.form.sections:
        out += [role in sec.roles] * (sec.bars * STEPS_PER_BAR)
    return out


def _gate(track: Track, role: str, amp: float) -> str:
    """``amp=var`` по секциям: уровень роли там, где она заказана, иначе 0."""
    segments: List[List] = []
    for sec in track.form.sections:
        value = _num(amp) if role in sec.roles else "0"
        if segments and segments[-1][0] == value:
            segments[-1][1] += sec.bars * BEATS_PER_BAR
        else:
            segments.append([value, sec.bars * BEATS_PER_BAR])
    return f"var({_list(v for v, _ in segments)}, {_list(_num(b) for _, b in segments)}, start={FORM_START})"


def _period(values: Sequence[float]) -> List[float]:
    """Наименьший период списка (Renardo зацикливает ``amplify`` по номеру события)."""
    n = len(values)
    for p in (p for p in range(1, n + 1) if n % p == 0):
        if all(values[i] == values[i % p] for i in range(n)):
            return list(values[:p])
    return list(values)


def _section_steps(track: Track) -> List[int]:
    """Номер секции на каждой 16-й формы."""
    out: List[int] = []
    for i, sec in enumerate(track.form.sections):
        out += [i] * (sec.bars * STEPS_PER_BAR)
    return out


def _amplify(track: Track, role: str, accents: Sequence[int], varied: bool,
             gains: Optional[Sequence[float]] = None) -> List[float]:
    """Акцент (если акценты ударов разные) × сайдчейн вида секции по шагу такта; ``accents`` — по 16-м свёрнутого
    рисунка, ``gains`` — готовые усиления на всю форму (рисунок-объединение ударных, :func:`_drum_fold`).
    Огибающая своя у каждой секции (``Mix.duck``), поэтому список свёрнут по звучащим шагам формы."""
    envs = [duck_envelope(d.trigger, d.depth) for d in track.mix.duck] if role in track.mix.duck_roles else None
    out = []
    for i, sec in enumerate(_section_steps(track)):
        if gains is not None:
            gain = gains[i]
        else:
            gain = kn.ACCENT_AMPLIFY[accents[i % len(accents)]] if varied else 1.0
        out.append(round(gain * (envs[sec][i % STEPS_PER_BAR] if envs else 1.0), 3))
    folded = _fold(out, _active_steps(track, role), track.form.bars_total)
    return _period([1.0 if g is None else g for g in folded])


def _lpf(track: Track, role: str) -> List[str]:
    """``lpf=var([...], [доли])`` свипа роли по секциям (``Mix.lpf``): шаг — доля, кривая — экспонента (ровно по
    слуху), ``0`` — фильтр снят. ``var``, а не ``linvar``: скачок на границе секции без промежуточного сегмента."""
    sweeps = track.mix.lpf.get(role)
    if not sweeps:
        return []
    values: List[List] = []
    for sec, (a, b) in zip(track.form.sections, sweeps):
        beats = sec.bars * BEATS_PER_BAR
        steps = [a] * beats if a == b or not a or not b else [
            a * (b / a) ** (k / (beats - 1)) for k in range(beats)]
        for hz in steps:
            hz = _num(round(hz))
            if values and values[-1][0] == hz:
                values[-1][1] += 1
            else:
                values.append([hz, 1])
    return [f"lpf=var({_list(v for v, _ in values)}, {_list(_num(n) for _, n in values)}, start={FORM_START})"]


def _stereo(st: Optional[Stereo], hits: Optional[Sequence[bool]], bpm: int) -> List[str]:
    """Аргументы ширины: ``hits`` — удары свёрнутого рисунка ударной роли, ``None`` — тональная роль."""
    if st is None:
        return []
    opts = []
    if hits is not None and st.pan:
        opts.append(f"pan={_list(_num(p) for p in alternate_pan(hits, st.pan, st.first))}")
    elif st.voices == 2:
        haas = _delay_beats(st.haas_ms, bpm)
        opts += [f"pan=(({_num(-st.pan)}, {_num(st.pan)}),)", f"pshift=((0, {_num(st.detune)}),)"]
        opts += [f"delay=((0, {_num(haas)}),)"] if haas else []
    return opts


def _tail(track: Track, role: str, part: Part, accents: Sequence[int], varied: bool,
          hits: Optional[Sequence[bool]] = None, gains: Optional[Sequence[float]] = None) -> List[str]:
    st = track.mix.stereo.get(role)
    opts = [f"amp={_gate(track, role, voice_amp(role, part, st.voices if st else 1))}"]
    amplify = _amplify(track, role, accents, varied, gains)
    if len(set(amplify)) > 1:
        opts.append(f"amplify={_list(_num(a) for a in amplify)}")
    return opts + _stereo(st, hits, track.bpm) + _lpf(track, role)


def _delay_beats(offset_ms: int, bpm: int) -> float:
    return offset_ms * bpm / 60000.0


def _drum_fold(cells: Sequence[Optional[tuple]], active: Sequence[bool],
               bars_total: int) -> Tuple[List[Optional[tuple]], Optional[List[float]]]:
    """Рисунок ``play()`` с периодом — степенью двойки шагов (фаза слоёв; санитайзер v1 #1803 его бы достроил).

    Секции с разными рисунками (вид build ↔ drop, PR-7) складываются в период формы (48 тактов — не степень
    двойки). Тогда рисунок — объединение ударов на наибольшем периоде-степени двойки, а удары, которых в секции
    нет, глушит ``amplify`` 0: второй список — усиления на всю форму (акцент удара или 0)."""
    pattern = _fold(cells, active, bars_total)
    if not len(pattern) & (len(pattern) - 1):
        return pattern, None
    period = max(b for b in range(1, bars_total + 1) if bars_total % b == 0 and not b & (b - 1)) * STEPS_PER_BAR
    union: List[Optional[tuple]] = [None] * period
    for i, cell in enumerate(cells):
        if active[i] and cell is not None and union[i % period] is None:
            union[i % period] = cell
    gains = [1.0 if union[i % period] is None else (kn.ACCENT_AMPLIFY[cell[0]] if cell else 0.0)
             for i, cell in enumerate(cells)]
    return union, gains


def _drum_line(slot: str, role: str, part: Part, track: Track) -> str:
    steps = part.grid.steps
    total = track.form.bars_total * STEPS_PER_BAR
    cells = [(st.accent, st.offset_ms) if st.on else None for st in steps * (total // len(steps))]
    pattern, gains = _drum_fold(cells, _active_steps(track, role), track.form.bars_total)
    symbol = kn.DRUM_SYMBOLS[role]
    accents = [c[0] for c in pattern if c]
    full = [c[0] if c else 0 for c in pattern]
    opts = _tail(track, role, part, full, len(set(accents)) > 1, [c is not None for c in pattern], gains)
    delays = [_delay_beats(c[1], track.bpm) if c else 0.0 for c in pattern]
    if any(delays):
        opts.append(f"delay={_list(_num(d) for d in delays)}")
    text = "".join(symbol if c else "." for c in pattern)
    sample = [f"sample={part.sample}"] if part.sample else []
    return f'{slot} >> play("{text}", dur=1/4, ' + ", ".join(sample + opts) + ")"


def _one_shot_sus(info: kn.SampleInfo, gap_beats: float, bpm: int) -> float:
    """``sus`` удара: длина файла в долях, но не дольше шага — синт ``loop`` (``PlayBuf(loop: 1)`` на всю ``sus``,
    ``loop.scd``) иначе крутит файл по кругу (раунд 2 PR-3d)."""
    return min(info.seconds * bpm / 60.0, gap_beats)


def _gap(steps) -> int:
    on = [i for i, st in enumerate(steps) if st.on]
    return min((on[(k + 1) % len(on)] - on[k]) % len(steps) or len(steps) for k in range(len(on)))


def _chop_line(head: str, info: kn.SampleInfo, part: Part, gate: str) -> str:
    """Луп нарезкой DJ_Dave (``loopAt(l).chop(l*8).legato(1)``): кусок — шаг сетки, каждый перезапускается на своей
    доле с ``pos`` = начало куска в долях оригинала; ``tempo=`` — темп оригинала (Renardo: ``rate`` = bpm/tempo,
    ``pos`` × tempo — ``Players.py`` LoopPlayer), ``sus`` = кусок (legato 1)."""
    step = _gap(part.grid.steps) * STEP_BEATS
    beats = info.beats or 4
    pieces = _list(_num(k * step) for k in range(int(round(beats / step))))
    return head + f"{pieces}, dur={_num(step)}, sus={_num(step)}, tempo={info.bpm}, {gate})"


def _pool_line(slot: str, head: str, role: str, part: Part, track: Track, gate: str, stereo: List[str]) -> str:
    """psr-пул DJ_Dave: удар на каждую 16-ю, файл события — ``c1.buf = [...]`` по кругу (``Player.__setattr__``),
    ``sus`` — длина своего файла (≤ 16-й), ``amplify`` — акцент × сайдчейн-огибающая (как пэд и бас)."""
    steps = part.grid.steps
    gap = _gap(steps) * STEP_BEATS
    sus = _list(_num(_one_shot_sus(kn.SAMPLE_CATALOG[n], gap, track.bpm)) for n in part.pool)
    accents = [st.accent for st in steps]
    amplify = _amplify(track, role, accents, len(set(accents)) > 1)
    opts = [f"dur={_num(gap)}", f"sus={sus}", gate, f"amplify={_list(_num(a) for a in amplify)}"] + stereo
    bufs = ", ".join(f"Samples.loadBuffer({kn.SAMPLE_CATALOG[n].loop_arg!r})" for n in part.pool)
    return head + ", ".join(opts) + f")\n{slot}.buf = [{bufs}]"


def _sample_line(slot: str, role: str, part: Part, track: Track) -> str:
    """``loop()`` сэмпла каталога (PR-3d, приёмы DJ_Dave): ``loop`` — нарезка (:func:`_chop_line`), ``sample`` —
    psr-пул на 16-х (:func:`_pool_line`), ``fx`` — одиночный удар, звучит один раз. Ширина — ``Mix.stereo``."""
    info = kn.SAMPLE_CATALOG[part.synth_or_sample]
    st = track.mix.stereo.get(role)
    gate = f"amp={_gate(track, role, voice_amp(role, part, st.voices if st else 1))}"
    head = f"{slot} >> loop({info.loop_arg!r}, "
    if role == "loop":
        return _chop_line(head, info, part, gate)
    if part.pool:
        return _pool_line(slot, head, role, part, track, gate, _stereo(st, None, track.bpm))
    gap = _gap(part.grid.steps) * STEP_BEATS
    return head + f"dur={_num(gap)}, sus={_num(_one_shot_sus(info, gap, track.bpm))}, {gate})"


def _cells(role: str, part: Part, track: Track) -> List[Optional[Cell]]:
    """Ноты партии по 16-м всей формы; проверка сетки и секций."""
    active = _active_steps(track, role)
    grouped: Dict[int, List] = {}
    for ev in part.pitches or ():
        step = ev.beat / STEP_BEATS
        if abs(step - round(step)) > 1e-9:
            raise RenderError(f"parts.{role}: нота на доле {ev.beat} вне сетки 16-х")
        step = int(round(step))
        if not active[step]:
            raise RenderError(f"parts.{role}: нота на доле {ev.beat} в секции, где роль молчит")
        grouped.setdefault(step, []).append(ev)
    cells: List[Optional[Cell]] = [None] * len(active)
    for step, evs in grouped.items():
        if len({(e.dur_beats, e.accent) for e in evs}) != 1:
            raise RenderError(f"parts.{role}: аккорд на доле {step * STEP_BEATS} с разными sus/акцентом")
        cells[step] = (tuple(sorted(e.midi for e in evs)), evs[0].dur_beats, evs[0].accent)
    return cells


def _fold(cells: Sequence[Optional[tuple]], active: Sequence[bool], bars_total: int) -> List[Optional[tuple]]:
    """Наименьший период в тактах, на котором звучащие шаги модели совпадают."""
    for bars in (b for b in range(1, bars_total + 1) if bars_total % b == 0):
        period = bars * STEPS_PER_BAR
        pattern: List[object] = [_UNSET] * period
        for i, cell in enumerate(cells):
            if not active[i]:
                continue
            if pattern[i % period] is _UNSET:
                pattern[i % period] = cell
            elif pattern[i % period] != cell:
                break
        else:
            return [None if c is _UNSET else c for c in pattern]  # type: ignore[misc]
    raise RenderError("свёртка невозможна")  # период = вся форма всегда подходит


def _note(cell: Optional[Cell]) -> str:
    if cell is None:
        return "None"
    notes = cell[0]
    return str(notes[0]) if len(notes) == 1 else "(" + ", ".join(str(n) for n in notes) + ")"


def _tonal_line(slot: str, role: str, part: Part, track: Track) -> str:
    pattern = _fold(_cells(role, part, track), _active_steps(track, role), track.form.bars_total)
    sus = [c[1] if c else STEP_BEATS for c in pattern]
    accents = [c[2] if c else 0 for c in pattern]
    sus_text = _num(sus[0]) if len(set(sus)) == 1 else _list(_num(s) for s in sus)
    opts = [f"dur=1/4, sus={sus_text}, scale=Scale.chromatic, root=0, oct=0"]
    onsets = {c[2] for c in pattern if c}
    opts += _tail(track, role, part, accents, len(onsets) > 1)
    return f"{slot} >> {part.synth_or_sample}({_list(_note(c) for c in pattern)}, " + ", ".join(opts) + ")"


def _slots(track: Track, deck: str) -> Dict[str, str]:
    if deck not in kn.DECK_SLOTS:
        raise RenderError(f"деки {deck!r} нет (есть {sorted(kn.DECK_SLOTS)})")
    unknown = sorted(set(track.parts) - set(ROLE_SLOT))
    if unknown:
        raise RenderError(f"роли {unknown} рендер ещё не выражает (ADR-0149 PR-3)")
    slots = {role: kn.DECK_SLOTS[deck][ROLE_SLOT[role]] for role in track.parts}
    if len(set(slots.values())) != len(slots):
        raise RenderError(f"две роли в одном слоте: {slots}")
    return slots


def render(track: Track, deck: str) -> Program:
    """Модель трека → программа деки ``deck``. Детерминирована: одна модель — одна строка."""
    validate(track)
    slots = _slots(track, deck)
    key = track.key
    lines = [f"# {track.track_id} {kn.ROOTS[key.root]} {key.mode} {track.bpm} BPM deck {deck}"]
    for role in sorted(track.parts, key=ROLE_SLOT.__getitem__):
        part = track.parts[role]
        line = _tonal_line if role in kn.TONAL_ROLES else _sample_line if role in SAMPLE_ROLES else _drum_line
        lines.append(line(slots[role], role, part, track))
    tonal = {p.synth_or_sample for r, p in track.parts.items() if r in kn.TONAL_ROLES}
    drums = {kn.DRUM_SYMBOLS[r] + (f":{p.sample}" if p.sample else "")
             for r, p in track.parts.items() if r in kn.DRUM_SYMBOLS}
    files = {kn.SAMPLE_CATALOG[n].path for r, p in track.parts.items() if r in SAMPLE_ROLES
             for n in (p.synth_or_sample, *p.pool)}
    return Program(
        code="\n".join(lines) + "\n", track_id=track.track_id, deck=deck, bpm=track.bpm,
        form_beats=float(track.form.bars_total * BEATS_PER_BAR), slots=slots,
        synths=frozenset(tonal), samples=frozenset(drums), sample_files=frozenset(files),
    )


__all__ = ["FORM_START", "ROLE_SLOT", "RenderError", "render"]
