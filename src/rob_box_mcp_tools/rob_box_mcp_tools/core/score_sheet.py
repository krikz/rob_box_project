"""Партитура (score sheet) трека: что аранжировщик на самом деле сыграл.

Зачем (ADR-0132, PR-1)
======================

Модель видела из ~20 авто-решений аранжировщика три (название, форму,
повтор) и потому не могла ни проверить результат, ни честно о нём сказать.
Партитура — чистая функция от того, что уже построено (спецификация,
гармонизация, Renardo-код, записи ``decisions``), и отдаёт:

* словарь для ``data["score"]`` — структурно, для тестов и логов;
* компактный русский текст (~1 КБ) — для ``message`` тула.

Она ничего не решает и ничего не меняет: музыка байт-в-байт та же
(``test_arranger_golden``). Высоты — в ЗВУЧАЩЕЙ высоте: поправка на
собственное транспонирование синта (``SYNTH_SEMITONE_SHIFT``) снята, иначе
бас ``moogbass`` выглядел бы на две октавы выше, чем слышен.

Строка «Решения по умолчанию» перечисляет ручки в виде
``ручка=auto→значение``: сегодня модель их менять не может (ручки в
``compose_music`` — PR-4 ADR-0132), но уже видит, что выбрано за неё.
С PR-3 ядро принимает ручки (``harmonize.HarmonizeOptions``,
``arranger.ArrangeOptions``); ручка, заданная не ``auto``, пишется без
``auto→`` — её значением (``bass_style=root, шаг 1``, ``pad_style=off``),
а ручки, у которых ``auto`` в строке не видно (гармонический ритм,
регистр пэда, хэты, громкости), появляются только когда заданы.

Инвентаризация скрытых решений (ADR-0132 §2, 23 пункта) и где они видны
-----------------------------------------------------------------------

Формат: ``место → что решает → где в партитуре после PR-1``.

1. ``rtttl_library.get``/``_melody_quality`` → какая запись играет →
   ``title`` (+ ``data["alternatives"]`` от #2896, вне партитуры).
2. ``rtttl_compose._normalize_tempo`` → свёртка темпа в 60–180 →
   ``tempo_fold`` + пометка у bpm.
3. ``_snap_to_bar`` → хвостовая пауза →
   ``raw_decisions['prep']['tail_pad_beats']`` (только dict).
4. ``_normalize_lead_register`` → перенос темы октавами → ``lead_octave``.
5. ``_fix_isolated_lead_outliers`` → подтяжка выбросов → ``lead_outliers``.
6. ``detect_key`` → тональность аккомпанемента → ``key`` с разрывом и
   альтернативами (``detect_key_ranked``); root/scale вызова при ``name=``
   перегармонизируют тему (PR-2) → ``key=explicit→X (auto Y)`` и
   предупреждение, если тема в заданном ладу меньше чем на
   :data:`KEY_FIT_WARN`.
7. ``harmonize.DENSE_ONSETS_PER_BEAT`` → dense/sparse → ``density``.
8. ``_pick_chords`` → аккорды → ``chords`` по тактам.
9. ``_pad_ceiling``/``PAD_BASS_CLEARANCE``/``_stack_chord`` → регистр пэда →
   диапазон пэда + проверки «пэд над басом», «пэд vs низ темы».
10. ``_build_bass``/``_approach_note`` → бас → ``bass_style``,
    ``bass_approach``, проверка «бас вне лада».
11. ``_build_pad``/``PAD_STAB_SUS`` → стаккато пэда → ``pad_style``.
12. ``_build_counter`` → второй голос → ``counter`` с причиной.
13. ``_build_drums``/``_build_hats`` → ударные → строка «Ударные», ``drums``.
14. ``_should_octave_double`` → удвоение темы → ``theme_octaves`` с причиной.
15. ``SYNTH_SEMITONE_SHIFT`` → физика синта → «(синт ±N)» у партии;
    диапазоны — уже в звучащей высоте.
16. ``_heavy_brass_safety_net`` (tools/music.py) молча выключал второй
    голос и октавы у imperialbrass → с PR-6 удалён: ничего не выключается,
    по таблице :mod:`core.synth_traits` партитура предупреждает «<синт>:
    долгий релиз … = 3 голоса с хвостом → counter=off или theme_octaves=off».
17. ``resolve_form``/``_snap_plan_to_theme`` → секции под длину темы →
    ``form``, таймлайн секций; неизвестная форма в ``compose_music`` с PR-2 —
    ошибка (здесь остаётся страховка «→arc» с предупреждением).
18. ``ROLE_PROFILE``/``levels`` → баланс громкости → ``mix_balance`` (issue
    #2963, статическая оценка по коду ``render()``, не измерение микса; в
    текст выносится только при 3+ «тяжёлых» ролях). ``COUNTER_OF_LEAD``/
    ``FIXED_THEME_AMP_FLOOR``/``FORMS`` (динамика формы) — по-прежнему НЕ
    показаны (только таймлайн формы).
19. ``_motif_variants``/``_dur_var`` → вариации сочинённой музыки → пока НЕ
    показаны.
20. ``_autofill_bass`` → бас ``dub`` сам добавлен → виден как партия баса.
21. ``render``: кламп bpm, неверная тоника → C, swing ≤ 0.3,
    ``BARS_PER_CHORD`` → в ``compose_music`` с PR-2 неверный ввод — ошибка
    (``arranger.check_*``); кламп в ``render`` остался страховкой рантайма.
22. ``renardo_sanitizer`` (слоты, длина рисунка, кап amp) → раньше
    терялось в ``compose_music`` → внешние предупреждения партитуры.
23. ``_repeat_warning`` → «совпало с прошлым треком» → в ``message``, как
    и было.
"""

from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .arranger import (
    BPM_RANGE,
    FORMS,
    LOOP_BASE_AMP,
    LOOP_ROLE,
    PAD_STAB_SUS,
    ROLE_PROFILE,
    SCALE_INTERVALS,
    SYNTH_SEMITONE_SHIFT,
    VALID_ROOTS,
    form_duration_seconds,
    resolve_form,
)
from .harmonize import _weighted_percentile
from .synth_traits import theme_tail_warning

__all__ = ["describe", "analyze_melody", "note_name", "chord_name"]

#: Разрыв скоров первой и второй тональности, ниже которого выбор считается
#: неуверенным (ADR-0132 §6).
KEY_GAP_UNSURE = 0.05

#: Рабочий диапазон лида (ADR-0132 §8: инвариант «лид в [55, 88]»).
LEAD_RANGE = (55, 88)

#: Доля длительности темы в явно заданном ладу, ниже которой — предупреждение
#: (ADR-0132 PR-2): тональность выполнена, но спорит с мелодией — решает модель.
KEY_FIT_WARN = 0.7

#: Доля длительности баса вне лада, выше которой — предупреждение.
BASS_OUT_OF_KEY_WARN = 0.25

#: Сколько тактов аккордов показывать в тексте (весь список — в dict).
_TEXT_CHORD_BARS = 16

#: Предел длины одного предупреждения в тексте (полное — в dict).
_TEXT_WARNING_CHARS = 160

_ROLE_NAMES = {
    "lead": "лид", "bass": "бас", "pad": "пэд", "counter": "второй голос",
}

_PLAYER_RE = re.compile(r"^\s*([dp]\d)\s*>>", re.MULTILINE)


# ---------------------------------------------------------------------------
# Имена
# ---------------------------------------------------------------------------


def note_name(midi: Optional[int]) -> str:
    """MIDI → научная нотация (60 → ``C4``); ``None`` → ``—``."""
    if midi is None:
        return "—"
    return f"{VALID_ROOTS[int(midi) % 12]}{int(midi) // 12 - 1}"


def chord_name(pitch_classes: Sequence[int]) -> str:
    """Трезвучие ``(корень, терция, квинта)`` → ``Am`` / ``C``."""
    root = pitch_classes[0]
    minor = len(pitch_classes) > 1 and (pitch_classes[1] - root) % 12 == 3
    return VALID_ROOTS[root % 12] + ("m" if minor else "")


def _range_text(lo: Optional[int], hi: Optional[int]) -> str:
    if lo is None or hi is None:
        return "—"
    return f"{note_name(lo)}–{note_name(hi)}"


# ---------------------------------------------------------------------------
# Разбор построенного
# ---------------------------------------------------------------------------


def chords_by_bar(harmony) -> List[str]:
    """Аккорды по тактам: ровно ``harmony.bars`` строк (``Dm`` / ``Dm-A``)."""
    out: List[str] = []
    for bar in range(int(harmony.bars)):
        start, end = bar * 4.0, bar * 4.0 + 4.0
        names: List[str] = []
        for chord in harmony.chords:
            if chord.start < end and chord.start + chord.beats > start:
                name = chord_name(chord.pitch_classes)
                if not names or names[-1] != name:
                    names.append(name)
        out.append("-".join(names) or "—")
    return out


def _flatten(notes: Sequence[Any]) -> List[int]:
    flat: List[int] = []
    for note in notes:
        if isinstance(note, (tuple, list)):
            flat.extend(int(v) for v in note)
        elif note is not None:
            flat.append(int(note))
    return flat


def _part(layer) -> Dict[str, Any]:
    """Одна мелодическая партия: синт, сдвиг синта, звучащий диапазон."""
    shift = SYNTH_SEMITONE_SHIFT.get(layer.synth or "", 0)
    sounding = [n - shift for n in _flatten(layer.midi or ())]
    doubled = any(isinstance(n, (tuple, list)) for n in (layer.midi or ()))
    return {
        "synth": layer.synth,
        "synth_shift": shift,
        "lo": min(sounding) if sounding else None,
        "hi": max(sounding) if sounding else None,
        "octave_doubled": doubled and layer.role == "lead",
        "degrees": list(layer.degrees) if layer.midi is None else None,
        "sus": layer.sus,
    }


def _parts(spec) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, str]]:
    """Мелодические партии и рисунки ударных/лупа из спецификации."""
    parts: Dict[str, Dict[str, Any]] = {}
    drums: Dict[str, str] = {}
    for layer in spec.layers:
        if layer.synth:
            parts[layer.role] = _part(layer)
        elif layer.pattern:
            drums[layer.role] = layer.pattern
    return parts, drums


def _scale_pcs(root: str, scale: str) -> frozenset:
    semitone = VALID_ROOTS.index(root) if root in VALID_ROOTS else 0
    intervals = SCALE_INTERVALS.get(scale, SCALE_INTERVALS["minor"])
    return frozenset((semitone + i) % 12 for i in intervals)


def _bass_out_of_key(harmony) -> float:
    """Доля длительности баса вне лада темы (в звучащих нотах гармонизации)."""
    pcs = _scale_pcs(harmony.root, harmony.scale)
    total = sum(d for n, d in harmony.bass if n is not None)
    outside = sum(d for n, d in harmony.bass if n is not None and n % 12 not in pcs)
    return round(outside / total, 3) if total else 0.0


def _lead_body_low(harmony) -> Optional[int]:
    pairs = [(n, d) for n, d in harmony.lead if n is not None]
    return int(round(_weighted_percentile(pairs, 0.10))) if pairs else None


# ---------------------------------------------------------------------------
# Проверки и решения
# ---------------------------------------------------------------------------


def _checks(parts: Dict[str, Dict[str, Any]], harmony, code: str) -> Dict[str, Any]:
    """Инварианты ADR-0132 §8, посчитанные по сыгранному."""
    bass, pad, lead = parts.get("bass", {}), parts.get("pad", {}), parts.get("lead", {})
    slots = sorted(set(_PLAYER_RE.findall(code or "")))
    checks: Dict[str, Any] = {"slots": slots}
    if bass.get("hi") is not None and pad.get("lo") is not None:
        checks["pad_over_bass"] = pad["lo"] - bass["hi"]
    if lead.get("lo") is not None:
        checks["lead_in_range"] = LEAD_RANGE[0] <= lead["lo"] and lead["hi"] <= LEAD_RANGE[1]
    if harmony is not None:
        checks["bass_out_of_key"] = _bass_out_of_key(harmony)
        body_low = _lead_body_low(harmony)
        if body_low is not None and pad.get("hi") is not None:
            checks["pad_under_theme_body"] = pad["hi"] - body_low
    return checks


def _prep_decisions_text(prep: Dict[str, Any]) -> Dict[str, str]:
    source, bpm = prep.get("source_bpm"), prep.get("bpm")
    tempo = "без свёртки" if source == bpm else f"{source}→{bpm}"
    shift = int(prep.get("lead_shift", 0))
    octave_mode = prep.get("lead_octave_mode", "auto")
    shift_text = f"{shift // 12:+d} окт" if shift else "0"
    moved = prep.get("outliers_moved", 0)
    return {
        "tempo_fold": f"auto→{tempo}",
        "lead_octave": f"{_mode_text(octave_mode)}→{shift_text}",
        "lead_outliers": (
            "keep" if prep.get("lead_outliers_mode") == "keep" else f"auto→fix({moved} нот)"
        ),
    }


def _mode_text(mode: Any) -> str:
    """Значение ручки для партитуры: ``auto`` или заданное (``+1`` у октавы)."""
    if isinstance(mode, int) and not isinstance(mode, bool):
        return f"{mode:+d}"
    return str(mode)


def _key_decision_text(harmony, prep: Optional[Dict[str, Any]]) -> str:
    detected = (prep or {}).get("key_detected")
    if (prep or {}).get("key_source") == "explicit" and detected:
        return f"explicit→{harmony.root} {harmony.scale} (auto {detected[0]} {detected[1]})"
    method = (prep or {}).get("key_detection", "auto")
    return f"{method}→{harmony.root} {harmony.scale}"


def _knob_text(dec: Dict[str, Any], knob: str, auto_text: str, explicit_text: str) -> str:
    """``auto→…`` для ``auto``, иначе текст заданного значения (ADR-0132 PR-3)."""
    return auto_text if dec.get(f"knob_{knob}", "auto") == "auto" else explicit_text


def _bass_text(dec: Dict[str, Any], step: float) -> str:
    style = dec.get("knob_bass_style", "auto")
    if style in ("off", "pedal"):
        return str(style)
    return _knob_text(dec, "bass_style", f"auto→тоны аккорда, шаг {step:g}", f"{style}, шаг {step:g}")


def _pad_text(dec: Dict[str, Any], step: float) -> str:
    style = dec.get("knob_pad_style", "auto")
    stab = f"stab(шаг {dec.get('pad_step', step):g}, sus {PAD_STAB_SUS:g})"
    if style in ("off", "sustain"):
        return str(style)
    return _knob_text(dec, "pad_style", f"auto→{stab}", stab)


def _optional_knobs_text(dec: Dict[str, Any]) -> Dict[str, str]:
    """Ручки, которых в строке нет, пока они ``auto`` (ADR-0132 PR-3)."""
    out: Dict[str, str] = {}
    if dec.get("knob_harmonic_rhythm", "auto") != "auto":
        out["harmonic_rhythm"] = str(dec["knob_harmonic_rhythm"])
    register = dec.get("knob_pad_register", "auto")
    if register != "auto":
        out["pad_register"] = (
            _range_text(*register) if isinstance(register, tuple) else str(register)
        )
    if dec.get("knob_hats", "auto") != "auto":
        out["hats"] = "задан рисунком"
    return out


def _harmony_decisions_text(harmony, prep: Optional[Dict[str, Any]] = None) -> Dict[str, str]:
    dec = getattr(harmony, "decisions", {}) or {}
    kind = "dense" if harmony.dense else "sparse"
    step = dec.get("bass_step", 1.0 if harmony.dense else 2.0)
    measured = f"{harmony.density:.2f}/бит"
    approaches = dec.get("bass_approaches", "?")
    return {
        "density": _knob_text(
            dec, "density",
            f"auto→{kind}({measured}, порог {dec.get('dense_threshold', '?')})",
            f"{kind}(измерено {measured})",
        ),
        "key": _key_decision_text(harmony, prep),
        "chords": _knob_text(
            dec, "chords", f"auto→{len(harmony.chords)} смен",
            f"explicit→{len(harmony.chords)} смен",
        ),
        "bass_style": _bass_text(dec, step),
        "bass_approach": _knob_text(
            dec, "bass_approach", f"auto→{approaches}",
            f"{dec.get('knob_bass_approach')}→{approaches}",
        ),
        "pad_style": _pad_text(dec, step),
        "drums": _knob_text(
            dec, "drums", f"auto→выведены ({dec.get('drum_style', 'auto')})", "задан рисунком",
        ),
        **_optional_knobs_text(dec),
    }


def _decisions(spec, harmony, prep: Optional[Dict[str, Any]]) -> Dict[str, str]:
    """Ручки ``имя → 'auto→значение'`` (только то, что реально решалось)."""
    out: Dict[str, str] = {}
    if prep:
        out.update(_prep_decisions_text(prep))
    if harmony is not None:
        out.update(_harmony_decisions_text(harmony, prep))
    arr = getattr(spec, "decisions", {}) or {}
    knobs = arr.get("knobs") or {}
    for knob in ("counter", "theme_octaves"):
        if knob in arr:
            forced = knobs.get(knob, "auto") != "auto"
            out[knob] = str(arr[knob]) if forced else f"auto→{arr[knob]}"
    if arr.get("levels"):
        out["levels"] = ",".join(f"{role}×{value:g}" for role, value in arr["levels"].items())
    if arr.get("autofilled_roles"):
        # issue #2970: синт заказан, ступеней роли не было — партия
        # звучит по тонике лада, а не по нотам модели; партитура обязана
        # это назвать, а не просто показать партию как обычную.
        out["autofilled"] = (
            ",".join(arr["autofilled_roles"]) + " (тоника лада, нот не было)"
        )
    form_known = (spec.form or "").strip().lower() in FORMS
    out["form"] = spec.form if form_known else f"{spec.form}→arc (неизвестная)"
    return out


def _key_block(spec, harmony, prep: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    ranked = list((prep or {}).get("key_ranked") or [])
    if harmony is None:
        return {"root": spec.root, "scale": spec.scale, "source": "задана вызовом"}
    block = _explicit_key_fields(prep)
    return {
        "root": harmony.root,
        "scale": harmony.scale,
        "source": "задана вызовом" if block else "определена по теме",
        **block,
        "gap": (prep or {}).get("key_gap"),
        "alternatives": [
            {"root": r, "scale": s, "score": sc} for r, s, sc in ranked[1:]
        ],
        "score": ranked[0][2] if ranked else None,
    }


def _explicit_key_fields(prep: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Поля явной тональности (PR-2): что определилось бы само и насколько
    тема ложится в заданный лад. Пусто, если тональность не задавалась."""
    if (prep or {}).get("key_source") != "explicit":
        return {}
    detected = prep.get("key_detected") or ("?", "?")
    return {
        "detected": {"root": detected[0], "scale": detected[1]},
        "fit": prep.get("key_fit"),
    }


def _key_warnings(key: Dict[str, Any]) -> List[str]:
    if "detected" in key:
        return _explicit_key_warnings(key)
    gap, alts = key.get("gap"), key.get("alternatives") or []
    if gap is None or not alts or gap >= KEY_GAP_UNSURE:
        return []
    alt = alts[0]
    return [
        f"тональность неуверенная: разрыв {gap:.3f} с {alt['root']} {alt['scale']}"
    ]


def _explicit_key_warnings(key: Dict[str, Any]) -> List[str]:
    """Явная тональность спорит с темой — выполнено, но модель предупреждена.

    Разрыв «неуверенной» авто-тональности здесь не показывается: выбор
    сделан вызовом, а не автоматикой.
    """
    fit = key.get("fit")
    if fit is None or fit >= KEY_FIT_WARN:
        return []
    det = key["detected"]
    return [
        f"заданная тональность {key['root']} {key['scale']} спорит с темой: "
        f"в ладу только {fit:.0%} длительности нот (по нотам — "
        f"{det['root']} {det['scale']}); аккомпанемент построен в заданной"
    ]


def _check_warnings(checks: Dict[str, Any]) -> List[str]:
    out: List[str] = []
    gap = checks.get("pad_over_bass")
    if gap is not None and gap <= 0:
        out.append(f"пэд и бас пересекаются ({gap:+d} пт)")
    if checks.get("lead_in_range") is False:
        out.append(f"лид вне рабочего диапазона {LEAD_RANGE[0]}–{LEAD_RANGE[1]}")
    if checks.get("bass_out_of_key", 0.0) > BASS_OUT_OF_KEY_WARN:
        out.append(f"бас вне лада {checks['bass_out_of_key']:.0%} длительности")
    if len(checks.get("slots", ())) > 6:
        out.append("больше 6 плееров")
    return out


def _trait_warnings(parts: Dict[str, Dict[str, Any]]) -> List[str]:
    """Долгий хвост синта при трёх голосах темы (ADR-0132 PR-6)."""
    lead = parts.get("lead") or {}
    counter = parts.get("counter") or {}
    warning = theme_tail_warning(
        lead.get("synth"), counter.get("synth"), bool(lead.get("octave_doubled")),
    )
    return [warning] if warning else []


# ---------------------------------------------------------------------------
# Баланс громкости (issue #2963) — ADR-0132 §2 пункт 18: FORMS/ROLE_PROFILE/
# COUNTER_OF_LEAD/FIXED_THEME_AMP_FLOOR раньше не были видны партитуре вовсе.
# ---------------------------------------------------------------------------

#: Насколько октавное удвоение лида (вторая одновременная нота в том же
#: слое) прибавляет к его вкладу в общую громкость. НЕ измерение: два
#: одновременных тона синта складываются по мощности не строго линейно
#: (зависит от фазы/тембра), 1.5 — консервативная оценка «между 1 (не
#: считать вовсе) и 2 (считать как две независимые ноты)». Клиппинг живого
#: микса этим числом не подтверждён — см. issue #2963 (замер за Шифу).
_OCTAVE_DOUBLE_WEIGHT = 1.5

#: Роль относится к «тяжёлым» в конкретном треке, если её вклад в общую
#: громкость не меньше этой доли роли с максимальным вкладом. Только для
#: текста предупреждения — сам бюджет ничего не отсекает и не понижает
#: (ADR-0132 PR-6: тихого safety net нет, показываем факт и оставляем
#: решение модели/Шифу).
_HEAVY_LAYER_SHARE = 0.6


def _layer_peak_amp(layer, levels: Dict[str, float]) -> float:
    """Оценка пикового ``amp`` одного слоя: ``ROLE_PROFILE`` × ``levels``.

    Статическая оценка ИЗ КОДА (тот же расчёт, что делает ``render()`` для
    ``amp=``), не измерение микса на роботе. Октавное удвоение лида
    (``layer.midi`` держит пары нот вместо одиночных — см. ``_octave_double``
    в ``core.arranger``) добавляет :data:`_OCTAVE_DOUBLE_WEIGHT`: слой
    реально звучит двумя одновременными тонами, а не одним.
    """
    profile = ROLE_PROFILE.get(layer.role)
    if profile is not None:
        base_amp = profile[2]
    elif layer.role == LOOP_ROLE:
        base_amp = LOOP_BASE_AMP
    else:
        return 0.0
    level = float(levels.get(layer.role, 1.0))
    doubled = layer.role == "lead" and any(
        isinstance(note, tuple) for note in (layer.midi or ())
    )
    weight = _OCTAVE_DOUBLE_WEIGHT if doubled else 1.0
    return base_amp * level * weight


def _mix_balance(spec) -> Dict[str, Any]:
    """Оценка суммарной пиковой громкости слоёв (issue #2963).

    Партитура раньше не показывала баланс ролей вовсе (ADR-0132 §2 п.18).
    Это ЧЕСТНАЯ СТАТИЧЕСКАЯ ОЦЕНКА по формуле ``render()`` (сумма
    ``ROLE_PROFILE`` × ``levels``, удвоенный лид — с весом), не измерение
    клиппинга на мастер-шине scsynth: тул не имеет доступа к живому миксу
    робота. ``levels`` (ADR-0132 PR-3) с 24.09.2026 ограничена 0..1 —
    ручка может только притушить роль, не разогнать сумму выше того, что
    аранжировщик и так собрал бы по умолчанию для этого набора ролей.
    """
    levels = dict(getattr(spec, "levels", None) or {})
    layers: Dict[str, float] = {}
    for layer in spec.layers:
        amp = _layer_peak_amp(layer, levels)
        if amp > 0:
            layers[layer.role] = layers.get(layer.role, 0.0) + amp
    total = round(sum(layers.values()), 3)
    heavy = sorted(
        role for role, amp in layers.items()
        if layers and amp >= _HEAVY_LAYER_SHARE * max(layers.values())
    )
    return {
        "total": total,
        "layers": {role: round(amp, 3) for role, amp in layers.items()},
        "heavy_roles": heavy,
        "note": "оценка по ROLE_PROFILE×levels из кода, НЕ измерение микса",
    }


def _spec_warnings(spec, harmony) -> List[str]:
    out: List[str] = []
    bpm = float(spec.bpm)
    if not BPM_RANGE[0] <= bpm <= BPM_RANGE[1]:
        out.append(f"bpm {bpm:g} зажат в {BPM_RANGE[0]:g}–{BPM_RANGE[1]:g}")
    if (spec.form or "").strip().lower() not in FORMS:
        out.append(f"форма {spec.form!r} неизвестна — играет arc")
    if harmony is not None and (spec.root, spec.scale) != (harmony.root, harmony.scale):
        # Страховка рассинхрона: с PR-2 compose_music передаёт тональность
        # вызова в гармонизацию, и расхождения быть не должно.
        out.append(
            f"root/scale спецификации ({spec.root} {spec.scale}) расходятся с "
            f"аккомпанементом: он построен в {harmony.root} {harmony.scale}"
        )
    return out


# ---------------------------------------------------------------------------
# Текст
# ---------------------------------------------------------------------------


def _parts_text(parts: Dict[str, Dict[str, Any]]) -> str:
    chunks: List[str] = []
    for role in ("lead", "bass", "pad", "counter"):
        part = parts.get(role)
        if not part:
            continue
        where = (
            "ступени " + ",".join(f"{d:g}" for d in part["degrees"])
            if part["degrees"] is not None
            else _range_text(part["lo"], part["hi"])
        )
        extra = " ×окт" if part["octave_doubled"] else ""
        extra += " стаккато" if part.get("sus") else ""
        extra += f" (синт {part['synth_shift']:+d})" if part["synth_shift"] else ""
        chunks.append(f"{_ROLE_NAMES[role]} {part['synth']} {where}{extra}")
    return "; ".join(chunks) or "—"


def _key_text(key: Dict[str, Any]) -> str:
    base = f"{key['root']} {key['scale']}"
    if "detected" in key:
        det = key["detected"]
        fit = key.get("fit")
        fit_text = f", тема в ладу {fit:.0%}" if fit is not None else ""
        return f"{base} (задана вызовом; по нотам {det['root']} {det['scale']}{fit_text})"
    if key.get("gap") is None:
        return f"{base} ({key['source']})"
    alts = ", ".join(
        f"{a['root']} {a['scale']} {a['score']:.2f}" for a in key["alternatives"][:2]
    )
    return f"{base} (разрыв {key['gap']:.2f}; альт.: {alts})"


def _checks_text(checks: Dict[str, Any]) -> str:
    items: List[str] = []
    if "pad_over_bass" in checks:
        items.append(f"пэд над басом {checks['pad_over_bass']:+d} пт")
    if "pad_under_theme_body" in checks:
        items.append(f"пэд vs низ темы {checks['pad_under_theme_body']:+d} пт")
    if "bass_out_of_key" in checks:
        items.append(f"бас вне лада {checks['bass_out_of_key']:.0%}")
    if "lead_in_range" in checks:
        items.append(f"лид в {LEAD_RANGE[0]}–{LEAD_RANGE[1]}: {'да' if checks['lead_in_range'] else 'НЕТ'}")
    items.append(f"слоты {len(checks['slots'])}/6 ({' '.join(checks['slots'])})")
    return "; ".join(items)


def _chords_text(chords: Sequence[str]) -> str:
    shown = "|".join(chords[:_TEXT_CHORD_BARS])
    rest = len(chords) - _TEXT_CHORD_BARS
    return shown + (f" …(+{rest})" if rest > 0 else "")


def _render_text(sheet: Dict[str, Any]) -> str:
    form = sheet["form"]
    lines = [
        f"Партитура «{sheet['title'] or 'композиция'}»: {sheet['bpm']:g} bpm"
        f"{sheet['bpm_note']}, форма {form['name']}: {form['timeline']} = "
        f"{form['total_bars']} т., {sheet['duration_seconds']:.0f} с.",
        f"Тональность: {_key_text(sheet['key'])}.",
    ]
    if sheet["theme"]:
        lines.append(
            f"Тема: {sheet['theme']['bars']} т., {_range_text(sheet['theme']['lo'], sheet['theme']['hi'])}."
            f" Аккорды: {_chords_text(sheet['chords'])}."
        )
    lines.append(f"Партии (звучащая высота): {_parts_text(sheet['parts'])}.")
    if sheet["drums"]:
        lines.append("Ударные: " + "; ".join(f"{k} {v}" for k, v in sheet["drums"].items()))
    if sheet.get("preset"):
        lines.append(f"Пресет: {sheet['preset']}.")
    if sheet.get("inherited"):
        lines.append(f"{sheet['inherited']}.")
    lines.append(
        "Решения по умолчанию: "
        + "; ".join(f"{k}={v}" for k, v in sheet["decisions"].items()) + "."
    )
    lines.append(f"Проверки: {_checks_text(sheet['checks'])}.")
    balance = sheet.get("mix_balance") or {}
    if len(balance.get("heavy_roles") or ()) >= 3:
        lines.append(
            f"Баланс (оценка, не измерение): ~{balance['total']:g} суммарно, "
            f"тяжёлые роли {', '.join(balance['heavy_roles'])}."
        )
    if sheet["warnings"]:
        lines.append("⚠️ " + "; ".join(w[:_TEXT_WARNING_CHARS] for w in sheet["warnings"]) + ".")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Вход
# ---------------------------------------------------------------------------


def _theme_block(harmony) -> Optional[Dict[str, Any]]:
    if harmony is None:
        return None
    pitches = [n for n, _d in harmony.lead if n is not None]
    return {
        "bars": int(harmony.bars),
        "lo": min(pitches) if pitches else None,
        "hi": max(pitches) if pitches else None,
        "density": round(float(harmony.density), 3),
        "dense": bool(harmony.dense),
    }


def _form_block(spec) -> Dict[str, Any]:
    theme_bars = int(getattr(spec, "theme_bars", 0) or 0)
    plan = resolve_form(spec.form, theme_bars)
    return {
        "name": spec.form if (spec.form or "").strip().lower() in FORMS else "arc",
        "sections": [(name, int(bars)) for name, bars, _i in plan],
        "timeline": " ".join(f"{name}({int(bars)})" for name, bars, _i in plan),
        "total_bars": sum(int(bars) for _n, bars, _i in plan),
    }


def _bpm_note(spec, prep: Optional[Dict[str, Any]]) -> str:
    if not prep:
        return ""
    source = prep.get("source_bpm")
    if float(spec.bpm) != float(prep.get("bpm", spec.bpm)):
        return f" (задан вызовом; тема {prep.get('bpm')})"
    return f" (RTTTL {source}, свёрнут)" if source != prep.get("bpm") else ""


def describe(
    *,
    spec,
    code: str,
    harmony=None,
    prep_decisions: Optional[Dict[str, Any]] = None,
    title: Optional[str] = None,
    warnings: Sequence[str] = (),
    preset_note: Optional[str] = None,
    inherited_note: Optional[str] = None,
) -> Dict[str, Any]:
    """Партитура трека: dict + ``text`` (компактный русский, ~1 КБ).

    Args:
        spec: :class:`core.arranger.CompositionSpec` — то, что ушло в ``render``.
        code: итоговый Renardo-код (для подсчёта плееров/слотов).
        harmony: :class:`core.harmonize.Harmonization` при ``name=``; ``None``
            для сочинённого трека.
        prep_decisions: ``melody_to_compose_params(...)["decisions"]``.
        title: название сыгранной записи.
        warnings: внешние предупреждения (санитайзер).
        preset_note: ADR-0132 PR-7 — «<title> (ручка=значение, …)», когда
            вызов подмешал пресет ручек по мелодии; ``None`` — пресет не
            применялся.
        inherited_note: issue #2950 — «унаследовано от текущего трека:
            ручка=значение, …», когда вызов подмешал недостающие ручки от
            последнего сыгранного трека той же мелодии (подстройка
            звучания без пересборки всей аранжировки); ``None`` —
            наследования не было.
    """
    parts, drums = _parts(spec)
    checks = _checks(parts, harmony, code)
    key = _key_block(spec, harmony, prep_decisions)
    theme_bars = int(getattr(spec, "theme_bars", 0) or 0)
    sheet: Dict[str, Any] = {
        "title": title,
        "preset": preset_note,
        "inherited": inherited_note,
        "bpm": max(BPM_RANGE[0], min(BPM_RANGE[1], float(spec.bpm))),
        "bpm_note": _bpm_note(spec, prep_decisions),
        "duration_seconds": round(form_duration_seconds(spec.form, spec.bpm, theme_bars), 1),
        "form": _form_block(spec),
        "key": key,
        "theme": _theme_block(harmony),
        "chords": chords_by_bar(harmony) if harmony is not None else [],
        "parts": parts,
        "drums": drums,
        "mix_balance": _mix_balance(spec),
        "decisions": _decisions(spec, harmony, prep_decisions),
        # Сырые записи автоматики — для логов/тестов (в текст не идут).
        "raw_decisions": {
            "prep": dict(prep_decisions or {}),
            "harmony": dict(getattr(harmony, "decisions", {}) or {}),
            "arrangement": dict(getattr(spec, "decisions", {}) or {}),
        },
        "checks": checks,
        "warnings": list(warnings) + _spec_warnings(spec, harmony)
        + _key_warnings(key) + _check_warnings(checks) + _trait_warnings(parts),
    }
    sheet["text"] = _render_text(sheet)
    return sheet


def analyze_melody(params: Dict[str, Any]) -> Dict[str, Any]:
    """Короткий анализ темы для ``lookup_melody`` (ADR-0132): без аранжировки.

    ``params`` — результат ``melody_to_compose_params``.
    """
    harmony = params["harmony"]
    prep = params.get("decisions") or {}
    ranked = list(prep.get("key_ranked") or [])
    theme = _theme_block(harmony) or {}
    alts = [f"{r} {s} {sc:.2f}" for r, s, sc in ranked[1:3]]
    gap = prep.get("key_gap")
    text = (
        f"Анализ: {harmony.root} {harmony.scale}"
        + (f" (разрыв {gap:.2f}; альт.: {', '.join(alts)})" if gap is not None else "")
        + f"; {theme.get('bars')} т., {params.get('bpm')} bpm; "
        f"диапазон {_range_text(theme.get('lo'), theme.get('hi'))}; "
        f"плотность {theme.get('density', 0):.2f} атак/бит "
        f"({'плотная' if theme.get('dense') else 'редкая'})."
    )
    return {
        "root": harmony.root,
        "scale": harmony.scale,
        "key_gap": gap,
        "key_alternatives": [
            {"root": r, "scale": s, "score": sc} for r, s, sc in ranked[1:]
        ],
        "bpm": params.get("bpm"),
        "bars": theme.get("bars"),
        "lo": theme.get("lo"),
        "hi": theme.get("hi"),
        "density": theme.get("density"),
        "dense": theme.get("dense"),
        "text": text,
    }
