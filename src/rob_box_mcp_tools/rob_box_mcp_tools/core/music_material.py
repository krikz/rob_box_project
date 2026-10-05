"""music_material.py — музыкальный материал из сообщения человека → ноты → RTTTL (issue #3227).

Человек присылает в чат материал «для продолжения сэта»: RTTTL-строку,
Strudel/Tidal-код (``note("...")``), или просто список нот. :func:`parse_material`
превращает его в :class:`Material`, :func:`material_to_rtttl` — в RTTTL-строку,
дальше мелодия живёт в RTTTL-каталоге (её играет ``request_music`` по названию).
Нераспознанное — ``None``, ноты не выдумываются.

Соглашения Strudel (по документации): цикл = один такт из 4 долей;
``setcpm(X)`` — X циклов в минуту, значит ``bpm = X * 4``; идиома
``setcpm(83/4)`` даёт ровно bpm = 83. Без ``setcpm`` у Strudel 0.5 цикла в
секунду = 30 cpm = 120 bpm. ``setcps(X)`` → ``bpm = X * 60 * 4``.

Аккорды (``[e2,g2,b2]``, стеки слоёв) для мелодии сводятся к «верхнему
голосу»: в каждый момент звучит самая высокая из звучащих нот (skyline);
повторная атака той же высоты остаётся отдельной нотой.

Из нескольких паттернов выбирается «самый мелодичный»: см.
:func:`pattern_score` (хук-пригодность, разнообразие высот, число атак).
"""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from fractions import Fraction
from typing import List, Optional, Sequence, Tuple

from rob_box_music.arrange.hook import MIN_NOTES, MIN_PITCHES, MIN_RANGE

from .mini_notation import Unsupported, parse_events
from .rtttl_catalog import add_melody
from .rtttl import parse_rtttl

__all__ = ["Material", "material_name", "material_to_rtttl", "parse_material", "pattern_score", "store_material"]

Note = Tuple[Optional[int], float]

BEATS_PER_CYCLE = 4
DEFAULT_BPM = 120.0
MIN_SOUNDING = 4
MIN_DISTINCT = 3
_NAME_MAX = 20

_NOTE_PC = {"c": 0, "d": 2, "e": 4, "f": 5, "g": 7, "a": 9, "b": 11}
_NOTE_NAMES = ("c", "c#", "d", "d#", "e", "f", "f#", "g", "g#", "a", "a#", "b")
_NOTE_TOKEN = re.compile(r"^([a-g])([#sb]*)(-?\d+)?$", re.I)
_NUMBER_TOKEN = re.compile(r"^-?\d+(?:\.\d+)?$")

_SCALES = {
    "major": (0, 2, 4, 5, 7, 9, 11), "ionian": (0, 2, 4, 5, 7, 9, 11),
    "minor": (0, 2, 3, 5, 7, 8, 10), "aeolian": (0, 2, 3, 5, 7, 8, 10),
    "dorian": (0, 2, 3, 5, 7, 9, 10), "phrygian": (0, 1, 3, 5, 7, 8, 10),
    "lydian": (0, 2, 4, 6, 7, 9, 11), "mixolydian": (0, 2, 4, 5, 7, 9, 10),
    "locrian": (0, 1, 3, 5, 6, 8, 10), "harmonic minor": (0, 2, 3, 5, 7, 8, 11),
    "major pentatonic": (0, 2, 4, 7, 9), "minor pentatonic": (0, 3, 5, 7, 10),
}

#: Методы Strudel, меняющие сами ноты/порядок/время так, что мы их не моделируем:
#: паттерн с ними отвергается, а не «угадывается».
_UNFAITHFUL = frozenset({
    "add", "sub", "mul", "rev", "fast", "slow", "every", "ply", "arp", "off", "struct", "hurry",
    "segment", "chop", "iter", "palindrome", "early", "late", "range", "degradeby", "sometimes",
})


@dataclass(frozen=True)
class Material:
    """Ноты материала: ``(midi | None, доли)`` (четверть = 1.0), темп, происхождение."""

    notes: Tuple[Note, ...]
    bpm: float
    title: str = ""
    source_format: str = ""
    detail: str = ""
    rtttl: str = ""

    def sounding(self) -> List[int]:
        return [m for m, _d in self.notes if m is not None]

    def describe(self) -> str:
        n = len(self.sounding())
        where = {"rtttl": "RTTTL", "strudel": "Strudel", "notelist": "список нот"}.get(
            self.source_format, self.source_format)
        extra = f", паттерн {self.detail}" if self.detail else ""
        return f"{n} нот, {self.bpm:g} bpm, из формата {where}{extra}"


# ---------------------------------------------------------------------------
# Ноты и лады
# ---------------------------------------------------------------------------


def note_to_midi(token: str, default_octave: int = 3) -> Optional[int]:
    """``e2`` / ``c#4`` / ``bb3`` → MIDI (C4 = 60; без октавы — ``default_octave``, как в Strudel)."""
    m = _NOTE_TOKEN.match(token)
    if not m:
        return None
    acc = m.group(2).lower()
    shift = acc.count("#") + acc.count("s") - acc.count("b")
    octave = int(m.group(3)) if m.group(3) is not None else default_octave
    return 12 * (octave + 1) + _NOTE_PC[m.group(1).lower()] + shift


def parse_scale(spec: str) -> Optional[Tuple[int, Tuple[int, ...]]]:
    """``"c3:major"`` / ``"e:minor pentatonic"`` → ``(корень_midi, интервалы)``; неизвестный лад — ``None``."""
    root, _, mode = spec.strip().partition(":")
    midi = note_to_midi(root.strip())
    key = mode.strip().lower().replace(":", " ").replace("_", " ")
    if midi is None or key not in _SCALES:
        return None
    return midi, _SCALES[key]


def _degree_to_midi(value: float, scale: Tuple[int, Tuple[int, ...]]) -> int:
    root, steps = scale
    deg = int(round(value))
    return root + steps[deg % len(steps)] + 12 * (deg // len(steps))


def _token_to_midi(token: str, scale: Optional[Tuple[int, Tuple[int, ...]]], numeric_is_degree: bool) -> Optional[int]:
    if _NUMBER_TOKEN.match(token):
        if scale is not None:
            return _degree_to_midi(float(token), scale)
        return None if numeric_is_degree else int(round(float(token)))
    return note_to_midi(token)


# ---------------------------------------------------------------------------
# События → мелодия (верхний голос) → ноты
# ---------------------------------------------------------------------------


def _skyline(events: Sequence[Tuple[Fraction, Fraction, int]]) -> List[Tuple[Fraction, Fraction, int]]:
    """Верхний голос: ``[(старт, конец, midi)]``; соседние куски одной атаки склеены."""
    ends = [(s, s + d, m) for s, d, m in events]
    points = sorted({p for s, e, _m in ends for p in (s, e)})
    segs: List[Tuple[Fraction, Fraction, int, int]] = []
    for a, b in zip(points, points[1:]):
        live = [(m, i) for i, (s, e, m) in enumerate(ends) if s <= a < e]
        if live:
            top = max(live)
            if segs and segs[-1][1] == a and segs[-1][3] == top[1]:
                segs[-1] = (segs[-1][0], b, top[0], top[1])
            else:
                segs.append((a, b, top[0], top[1]))
    return [(a, b, m) for a, b, m, _i in segs]


def _segments_to_notes(segs: Sequence[Tuple[Fraction, Fraction, int]]) -> Tuple[Note, ...]:
    """Верхний голос → ``(midi|None, доли)``; ведущая тишина отброшена, внутренние паузы сохранены."""
    out: List[Note] = []
    cursor: Optional[Fraction] = None
    for start, end, midi in segs:
        if cursor is not None and start > cursor:
            out.append((None, float((start - cursor) * BEATS_PER_CYCLE)))
        out.append((midi, float((end - start) * BEATS_PER_CYCLE)))
        cursor = end
    return tuple(out)


# ---------------------------------------------------------------------------
# Strudel
# ---------------------------------------------------------------------------

_CALL_RE = re.compile(r"(?<![\w.$])(note|n)\s*\(")
_CPM_RE = re.compile(r"\bsetcpm\s*\(\s*([0-9.]+)\s*(?:/\s*([0-9.]+)\s*)?\)")
_CPS_RE = re.compile(r"\bsetcps\s*\(\s*([0-9.]+)\s*(?:/\s*([0-9.]+)\s*)?\)")
_LABEL_RE = re.compile(r"([A-Za-z_]\w*)\s*[:=]\s*$")
_COMMENT_RE = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)


def _read_balanced(text: str, open_idx: int) -> int:
    """Индекс ``)``, закрывающей ``(`` в ``open_idx`` (строки в кавычках/бэктиках пропускаются); ``-1`` — нет."""
    depth = 0
    i = open_idx
    quote = ""
    while i < len(text):
        ch = text[i]
        if quote:
            if ch == "\\":
                i += 1
            elif ch == quote:
                quote = ""
        elif ch in "\"'`":
            quote = ch
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return -1


def _string_literal(arg: str) -> Optional[str]:
    arg = arg.strip()
    if len(arg) >= 2 and arg[0] in "\"'`" and arg[-1] == arg[0] and arg[0] not in arg[1:-1]:
        return arg[1:-1]
    return None


def _method_chain(text: str, pos: int) -> List[Tuple[str, str]]:
    """``.метод(аргументы)`` подряд после ``pos`` (пробелы/переводы строк между ними допустимы)."""
    chain: List[Tuple[str, str]] = []
    while True:
        m = re.compile(r"\s*\.\s*(\w+)\s*\(").match(text, pos)
        if not m:
            return chain
        close = _read_balanced(text, m.end() - 1)
        if close < 0:
            return chain
        chain.append((m.group(1), text[m.end():close]))
        pos = close + 1


@dataclass(frozen=True)
class _Call:
    label: str
    pattern: str
    fn: str
    chain: Tuple[Tuple[str, str], ...]


def _find_calls(text: str) -> List[_Call]:
    calls: List[_Call] = []
    for m in _CALL_RE.finditer(text):
        close = _read_balanced(text, m.end() - 1)
        pattern = _string_literal(text[m.end():close]) if close > 0 else None
        if pattern is None:
            continue
        label = _LABEL_RE.search(text[:m.start()])
        calls.append(_Call(label.group(1) if label else "", pattern, m.group(1), tuple(_method_chain(text, close + 1))))
    return calls


def _chain_transform(chain: Sequence[Tuple[str, str]]) -> Optional[Tuple[Optional[tuple], int]]:
    """``(лад | None, сдвиг_в_полутонах)`` из цепочки методов; ``None`` — паттерн нельзя честно разобрать."""
    scale: Optional[tuple] = None
    shift = 0
    for method, args in chain:
        if method in _UNFAITHFUL:
            return None
        if method == "scale":
            lit = _string_literal(args)
            scale = parse_scale(lit) if lit is not None else None
            if scale is None:
                return None
        elif method in ("trans", "transpose"):
            if not re.fullmatch(r"\s*-?\d+\s*", args):
                return None
            shift += int(args)
    return scale, shift


def _call_to_material(call: _Call, bpm: float, title: str) -> Optional[Material]:
    transform = _chain_transform(call.chain)
    if transform is None:
        return None
    scale, shift = transform
    try:
        events, _cycles = parse_events(call.pattern)
    except Unsupported:
        return None
    timed: List[Tuple[Fraction, Fraction, int]] = []
    for start, dur, tok in events:
        midi = _token_to_midi(tok, scale, numeric_is_degree=call.fn == "n")
        if midi is None:
            return None
        timed.append((start, dur, midi + shift))
    notes = _segments_to_notes(_skyline(timed))
    return Material(notes, bpm, title, "strudel", call.label or call.fn)


def _strudel_bpm(text: str) -> float:
    m = _CPM_RE.search(text)
    if m:
        return float(m.group(1)) / float(m.group(2) or 1) * BEATS_PER_CYCLE
    m = _CPS_RE.search(text)
    if m:
        return float(m.group(1)) / float(m.group(2) or 1) * 60 * BEATS_PER_CYCLE
    return DEFAULT_BPM


def _strudel_title(raw: str) -> str:
    for line in re.findall(r"//\s*([^\n]*)", raw):
        line = line.strip()
        if line and "://" not in line and "@" not in line:
            return line
    return ""


def _usable(mat: Material) -> bool:
    pitches = mat.sounding()
    return len(pitches) >= MIN_SOUNDING and len(set(pitches)) >= MIN_DISTINCT and mat.bpm > 0


#: Доля звучащих 16-х в окне хука, ниже которой это «одни паузы» (порог клубного хука v1, #3225).
MIN_FILL = 0.35
#: Окно хука: 4 такта, если мелодия не короче, иначе 2 (как у клубного хука v1, #3181).
HOOK_WINDOW_BARS = (4, 2)
STEPS_PER_BAR = 16


def _grid(notes: Sequence[Tuple[Optional[int], float]]) -> List[Tuple[int, int, int]]:
    """Атаки ``(шаг 16-х, конец, MIDI)`` от первой звучащей ноты; партия быстрее 16-х растянута вдвое."""
    sounding = [d for m, d in notes if m is not None]
    scale = 2.0 if sounding and min(sounding) < 0.25 - 1e-9 else 1.0
    timed: List[Tuple[int, int, int]] = []
    cursor, start = 0.0, None
    for midi, beats in notes:
        if midi is not None:
            start = cursor if start is None else start
            onset = round((cursor - start) * scale * 4)
            if not timed or onset > timed[-1][0]:
                timed.append((onset, round((cursor - start + beats) * scale * 4), midi))
        cursor += beats
    return timed


def _hook_like(notes: Sequence[Tuple[Optional[int], float]]) -> bool:
    """Начало мелодии годится как хук: нот ≥ ``MIN_NOTES``, высот ≥ ``MIN_PITCHES``, диапазон ≥ ``MIN_RANGE``
    (пороги хука v2), звучащих 16-х ≥ ``MIN_FILL`` окна. Ноты — ``(MIDI | None, доли)`` в своём темпе."""
    timed = _grid(notes)
    if not timed:
        return False
    long_enough = timed[-1][1] >= HOOK_WINDOW_BARS[0] * STEPS_PER_BAR
    limit = (HOOK_WINDOW_BARS[0] if long_enough else HOOK_WINDOW_BARS[1]) * STEPS_PER_BAR
    nexts = [on for on, _end, _m in timed[1:]] + [limit]
    window = [(min(end, nxt, limit) - on, m) for (on, end, m), nxt in zip(timed, nexts) if on < limit]
    pitches = [m for _ln, m in window]
    return (len(window) >= MIN_NOTES and len(set(pitches)) >= MIN_PITCHES and max(pitches) - min(pitches) >= MIN_RANGE
            and sum(max(1, ln) for ln, _m in window) >= MIN_FILL * limit)


def pattern_score(mat: Material) -> Tuple[int, int, int, int]:
    """Чем больше — тем мелодичнее: ``(годится как хук, разных высот ≤ 8, число атак ≤ 16, не быстрее 16-х)``.

    Последний признак отсеивает FX-дорожки на 32-х (``*4``), у которых при
    равных нотах с основной партией темп нечитаем; при полном равенстве
    побеждает паттерн, стоящий в тексте раньше.

    «Годится как хук» — :func:`_hook_like`: окно начала мелодии на сетке 16-х с порогами хука движка v2
    (``rob_box_music.arrange.hook``) и заполненностью. Пэд из держащихся аккордов и партии с длинными
    паузами проигрывают короткому плотному мотиву: у них мало атак и низкая заполненность окна.
    """
    try:
        musical = int(_hook_like(parse_rtttl(material_to_rtttl(mat))[2]))
    except ValueError:
        musical = 0
    pitches = mat.sounding()
    shortest = min((d for m, d in mat.notes if m is not None), default=0.0)
    return musical, min(len(set(pitches)), 8), min(len(pitches), 16), int(shortest >= 0.25 - 1e-9)


def _parse_strudel(text: str) -> Optional[Material]:
    raw = text
    code = _COMMENT_RE.sub("", text)
    if not _CALL_RE.search(code):
        return None
    bpm = _strudel_bpm(code)
    title = _strudel_title(raw)
    best: Optional[Material] = None
    best_score: Tuple[int, ...] = (-1,)
    for call in _find_calls(code):
        mat = _call_to_material(call, bpm, title)
        if mat is None or not _usable(mat):
            continue
        score = pattern_score(mat)
        if score > best_score:
            best, best_score = mat, score
    return best


# ---------------------------------------------------------------------------
# RTTTL и список нот
# ---------------------------------------------------------------------------

_RTTTL_RE = re.compile(r"[^\s:,]{0,30}\s*:\s*d\s*=\s*\d+\s*,\s*o\s*=\s*\d+\s*,\s*b\s*=\s*\d+\s*:[^\n]+", re.I)
_BPM_RE = re.compile(r"(?:\bbpm\s*[=:]?\s*(\d{2,3})\b|\b(\d{2,3})\s*bpm\b)", re.I)
_LIST_TOKEN = re.compile(r"^([a-g][#b]?\d|p|~)$", re.I)


def _parse_rtttl_text(text: str) -> Optional[Material]:
    m = _RTTTL_RE.search(text)
    if not m:
        return None
    raw = re.sub(r"\s+", "", m.group().strip())
    try:
        name, bpm, notes = parse_rtttl(raw)
    except ValueError:
        return None
    mat = Material(tuple(notes), float(bpm), name, "rtttl", "", raw)
    return mat if mat.sounding() else None


def _parse_note_list(text: str) -> Optional[Material]:
    tokens = re.split(r"[\s,;]+", text.strip())
    run: List[str] = []
    best: List[str] = []
    for tok in tokens + [""]:
        if _LIST_TOKEN.match(tok):
            run.append(tok)
            continue
        best = run if len(run) > len(best) else best
        run = []
    notes: List[Note] = [
        (None if t.lower() in ("p", "~") else note_to_midi(t), 1.0) for t in best
    ]
    bpm = _BPM_RE.search(text)
    mat = Material(tuple(notes), float(bpm.group(1) or bpm.group(2)) if bpm else DEFAULT_BPM, "", "notelist")
    return mat if len(mat.sounding()) >= MIN_SOUNDING and len(set(mat.sounding())) >= MIN_DISTINCT else None


def parse_material(text: str) -> Optional[Material]:
    """Материал из текста сообщения: RTTTL → Strudel → список нот; иначе ``None``."""
    if not text or not text.strip():
        return None
    for parser in (_parse_rtttl_text, _parse_strudel, _parse_note_list):
        mat = parser(text)
        if mat is not None:
            return mat
    return None


# ---------------------------------------------------------------------------
# Материал → RTTTL
# ---------------------------------------------------------------------------

RTTTL_MIN_MIDI = 60  # C4 — нижняя граница октав RTTTL 4…7
RTTTL_MAX_MIDI = 107  # B7
_TARGET_MEAN = 78.0
_ALLOWED = sorted(
    ((4.0 / d * k, d, k == 1.5) for d in (1, 2, 4, 8, 16, 32) for k in (1.0, 1.5)), key=lambda x: x[0]
)
_MAX_BEATS = 4.0


def _fit_midis(midis: Sequence[int]) -> List[int]:
    """Сдвиг мелодии целыми октавами к середине диапазона RTTTL, остатки складываются в диапазон."""
    if not midis:
        return []
    mean = sum(midis) / len(midis)
    shift = 12 * round((_TARGET_MEAN - mean) / 12)
    out = []
    for m in midis:
        m += shift
        while m < RTTTL_MIN_MIDI:
            m += 12
        while m > RTTTL_MAX_MIDI:
            m -= 12
        out.append(m)
    return out


def _closest(beats: float) -> Tuple[float, int, bool]:
    return min((a for a in _ALLOWED if a[0] <= _MAX_BEATS), key=lambda a: abs(a[0] - beats))


def _rest_tokens(beats: float) -> List[str]:
    """Пауза в долях → жадно из допустимых длительностей RTTTL (остаток меньше 32-й отбрасывается)."""
    out: List[str] = []
    while beats >= _ALLOWED[0][0] - 1e-9:
        val, d, dotted = max((a for a in _ALLOWED if a[0] <= beats + 1e-9 and a[0] <= _MAX_BEATS), key=lambda a: a[0])
        out.append(f"{d}p{'.' if dotted else ''}")
        beats -= val
    return out


def _rtttl_name(title: str) -> str:
    clean = re.sub(r"[^0-9A-Za-zА-Яа-яЁё _-]", "", title).strip().replace(" ", "_")
    return (clean or "material")[:_NAME_MAX]


def _note_token(midi: int, d: int, dotted: bool) -> str:
    name = _NOTE_NAMES[midi % 12]
    return f"{d}{name}{midi // 12 - 1}{'.' if dotted else ''}"


def material_to_rtttl(material: Material) -> str:
    """Материал → RTTTL-строка ``имя:d=4,o=5,b=<bpm>:ноты`` (октавы 4…7, длительности 1…32 с точкой).

    Длительность каждой ноты — ближайшая допустимая к (накопленный точный конец −
    накопленный квантованный конец), поэтому ошибка квантования не копится.
    Нота длиннее целой обрезается до целой, остаток звучит паузой (RTTTL не
    умеет лиги). Исходный RTTTL из сообщения возвращается как есть.
    """
    if material.rtttl:
        return material.rtttl
    fitted = iter(_fit_midis(material.sounding()))
    tokens: List[str] = []
    exact = quant = 0.0
    for midi, beats in material.notes:
        exact += beats
        if midi is None:
            rests = _rest_tokens(exact - quant)
            tokens.extend(rests)
            quant += sum(_alloc_beats(t) for t in rests)
            continue
        val, d, dotted = _closest(exact - quant)
        tokens.append(_note_token(next(fitted), d, dotted))
        quant += val
        gap = _rest_tokens(exact - quant) if exact - quant >= 2 * _ALLOWED[0][0] else []
        tokens.extend(gap)
        quant += sum(_alloc_beats(t) for t in gap)
    return f"{_rtttl_name(material.title)}:d=4,o=5,b={max(1, round(material.bpm))}:{','.join(tokens)}"


def _alloc_beats(token: str) -> float:
    m = re.match(r"(\d+)p(\.?)", token)
    return 4.0 / int(m.group(1)) * (1.5 if m.group(2) else 1.0)


# ---------------------------------------------------------------------------
# Библиотека
# ---------------------------------------------------------------------------

USER_SOURCE = "user"
USER_TAGS = ["user", "dj-material"]


def material_name(material: Material) -> str:
    """Детерминированное имя записи: ``user_<слаг>_<хэш нот>`` (тот же материал — то же имя)."""
    rtttl = material_to_rtttl(material)
    slug = re.sub(r"[^a-z0-9]+", "_", material.title.lower()).strip("_")[:24] or "material"
    return f"user_{slug}_{hashlib.sha1(rtttl.encode('utf-8')).hexdigest()[:6]}"


def store_material(library, material: Material, title: str = "") -> Tuple[str, bool]:
    """Положить материал в RTTTL-библиотеку (``source=user``); ``(имя, добавлен_ли)``.

    ``False`` — точно такой же материал уже лежит (имя то же, идти можно по нему).
    """
    name = material_name(material)
    added = add_melody(
        library, material_to_rtttl(material), name, title=title or material.title or name,
        source=USER_SOURCE, tags=list(USER_TAGS),
    )
    return name, added
