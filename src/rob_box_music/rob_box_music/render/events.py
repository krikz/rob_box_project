"""События нот Renardo-программы без Renardo (issue #3154).

Перенос ``core/renardo_events.py`` (ADR-0149 §2.2, §9 PR-1b): единственная реализация, старый модуль —
реэкспорт. Таблицы ладов/тоник — из :mod:`rob_box_music.knowledge`; ``play_steps`` отсюда импортирует
старый санитайзер.

Модель громкости classic (``core.classic_loudness`` старого пути) считает энергию
каждой ноты, а ноты — это то, что Renardo сыграл бы по программе
аранжировщика. Модуль исполняет программу в ЗАГЛУШКАХ FoxDot (``var``,
``linvar``, ``Pvar``, ``Clock``, ``Root``, ``Scale``, плееры ``d1..p3``) и
разворачивает каждый плеер в события по семантике renardo_lib 0.9.13
(прочитано в исходниках, ``Players.py``/``Scale.py``):

* ``amp`` события = ``amp * amplify`` (``send_osc_message``); список —
  по номеру события, ``var`` — по доле клока;
* ``sus`` не задан → ``sus = dur`` (``Players.py:855``);
* высота: ``midi = 12·(oct + deg // n) + scale[deg % n] + root``
  (``Scale.midi``); у ``Scale.chromatic`` c ``oct=0, root=0`` это сама
  MIDI-нота; без ``scale=`` берутся ``Scale.default``/``Root.default``;
* ``Pvar`` — список ступеней по времени, элемент — по номеру события;
  кортеж — аккорд (``PGroup``), ``None`` — пауза.

* ``delay`` (доли) сдвигает момент события, не меняя ``amp``/``sus`` — так рендер v2
  выражает свинг (ADR-0149 §3.4); список — по номеру события, как ``amplify``.

Это не Renardo: ``Clock.latency``, ``nudge``/``Clock.swing``, ``every``, ``P[...]``
не моделируются. Программа исполняется с пустыми ``__builtins__`` —
только заглушки из :func:`_namespace`; незнакомая конструкция →
:class:`ProgramError` (вызывающий обязан решить, что делать без модели).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from ..knowledge import DECK_SLOTS, ROOTS, SCALES

NOTE_NAMES = ROOTS
SLOTS = tuple(slot for deck in sorted(DECK_SLOTS) for slot in DECK_SLOTS[deck])
#: Настоящий ``exec`` на момент импорта: тесты тула патчат ``builtins.exec``
#: (чтобы Renardo не исполнялся), а модели громкости нужен исполнитель.
_EXEC = exec
FX_KEYS = ("hpf", "lpf", "echo", "room")


class ProgramError(ValueError):
    """Программа не разворачивается в события (незнакомая конструкция)."""


class TimeVar:
    """``var``/``linvar``/``Pvar`` renardo по доле клока (цикл по сумме длительностей)."""

    def __init__(self, values: Sequence[Any], durs: Any, linear: bool = False) -> None:
        self.values = list(values)
        if isinstance(durs, (int, float)):
            durs = [durs] * len(self.values)
        self.durs = [float(d) for d in durs]
        self.linear = linear
        self.total = sum(self.durs)
        if not self.values or self.total <= 0:
            raise ProgramError("var без значений или с нулевой длиной")

    def at(self, beat: float) -> Any:
        pos = beat % self.total
        for i, dur in enumerate(self.durs):
            if pos < dur:
                if not self.linear:
                    return self.values[i]
                nxt = self.values[(i + 1) % len(self.values)]
                return self.values[i] + (nxt - self.values[i]) * pos / dur
            pos -= dur
        return self.values[-1]


def value_at(value: Any, index: int, beat: float) -> Any:
    """Атрибут плеера для события ``index`` на доле ``beat``."""
    if isinstance(value, list):
        value = value[index % len(value)]
    return value.at(beat) if isinstance(value, TimeVar) else value


@dataclass
class PlayerSpec:
    synth: str
    degree: Any
    kwargs: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class NoteEvent:
    """Одна нота (или удар сэмпла): что услышит scsynth."""

    beat: float
    slot: str
    synth: str
    amp: float
    gate: float
    sus_beats: float
    midi: Optional[float] = None
    sample: Optional[str] = None
    fx: Mapping[str, Any] = field(default_factory=dict)


@dataclass
class Program:
    players: Dict[str, PlayerSpec]
    bpm: float
    form_beats: Optional[float]
    root: Any
    scale: Any


class _Clock:
    def __init__(self) -> None:
        self.bpm = 120.0
        self.now_flag = False
        self.form_beats: Optional[float] = None

    def clear(self) -> None:
        return None

    def future(self, beats: float, _fn: Any) -> None:
        self.form_beats = float(beats)

    def set_time(self, *_args: Any) -> None:
        return None

    def swing(self, *_args: Any) -> None:
        return None

    def now(self) -> float:
        return 0.0


class _Default:
    default: Any = None


class _Scale(_Default):
    def __getattr__(self, name: str) -> str:
        if name in SCALES:
            return name
        raise ProgramError(f"незнакомый лад Scale.{name}")


class _Slot:
    def __init__(self, name: str, sink: Dict[str, PlayerSpec]) -> None:
        self.name, self.sink = name, sink

    def __rshift__(self, spec: PlayerSpec) -> None:
        self.sink[self.name] = spec


def _namespace(players: Dict[str, PlayerSpec], clock: _Clock, root: _Default, scale: _Scale) -> Dict[str, Any]:
    class Synths(dict):
        def __missing__(self, name: str) -> Any:
            if name.startswith("_"):
                raise ProgramError(f"имя {name!r} в программе")
            return lambda degree=None, *_a, **kw: PlayerSpec(name, degree, kw)

    ns = Synths(
        __builtins__={}, Clock=clock, Root=root, Scale=scale,
        var=lambda v, d, *_a: TimeVar(v, d),
        linvar=lambda v, d, *_a: TimeVar(v, d, linear=True),
        Pvar=lambda v, d, *_a: TimeVar(v, d),
    )
    for slot in SLOTS:
        ns[slot] = _Slot(slot, players)
    return ns


_GROUP_CLOSE = {"(": ")", "[": "]", "{": "}", "|": "|"}


def play_steps(pattern: str) -> Optional[List[str]]:
    """Разбить рисунок play() на шаги верхнего уровня.

    Группа ``(ab)`` (чередование), ``[ab]`` (деление шага), ``{ab}``
    (случайный выбор) и ``|x2|`` (выбор сэмпла) — это ОДИН шаг, а не
    ``len`` символов. ``None`` — рисунок не разбирается (слои ``<..>``,
    несбалансированные скобки): такой не трогаем.
    """
    if "<" in pattern or ">" in pattern:
        return None
    steps: List[str] = []
    i = 0
    while i < len(pattern):
        opener = pattern[i]
        if opener in ")]}":
            return None
        if opener not in _GROUP_CLOSE:
            steps.append(opener)
            i += 1
            continue
        depth, j = 0, i
        while j < len(pattern):
            ch = pattern[j]
            if opener == "|" and j > i and ch == "|":
                break
            if opener != "|":
                depth += ch == opener
                depth -= ch == _GROUP_CLOSE[opener]
                if depth == 0:
                    break
            j += 1
        if j >= len(pattern):
            return None
        steps.append(pattern[i:j + 1])
        i = j + 1
    return steps


def run_program(code: str) -> Program:
    """Исполнить программу в заглушках FoxDot → плееры, темп, длина формы."""
    players: Dict[str, PlayerSpec] = {}
    clock, root, scale = _Clock(), _Default(), _Scale()
    try:
        _EXEC(compile(code, "<renardo>", "exec"), _namespace(players, clock, root, scale))  # noqa: S102
    except ProgramError:
        raise
    except Exception as exc:  # noqa: BLE001 — любая незнакомая конструкция
        raise ProgramError(f"{type(exc).__name__}: {exc}") from exc
    return Program(players, float(clock.bpm), clock.form_beats, root.default, scale.default)


def _root_semitone(root: Any, beat: float) -> float:
    value = value_at(root, 0, beat)
    if isinstance(value, str):
        if value not in NOTE_NAMES:
            raise ProgramError(f"незнакомая тоника {value!r}")
        return float(NOTE_NAMES.index(value))
    return float(value or 0)


def degree_to_midi(degree: float, oct_: float, root: float, scale: str) -> float:
    """``Scale.midi`` renardo: ступень → MIDI (дробная ступень не встречается у аранжировщика)."""
    intervals = SCALES.get(scale)
    if intervals is None:
        raise ProgramError(f"незнакомый лад {scale!r}")
    step = int(degree // 1)
    n = len(intervals)
    return 12.0 * (oct_ + step // n) + intervals[step % n] + root


def _fx_at(kw: Mapping[str, Any], index: int, beat: float) -> Dict[str, Any]:
    fx: Dict[str, Any] = {}
    for key in FX_KEYS:
        if key in kw:
            value = value_at(kw[key], index, beat)
            if value:
                fx[key] = float(value)
                if key == "lpf" and isinstance(kw[key], TimeVar) and kw[key].linear:
                    fx["lpf_sweep"] = True
    if "room" in fx:
        fx["mix"] = float(value_at(kw.get("mix", 0.1), index, beat))
    return fx


def _play_symbol(steps: Sequence[str], index: int) -> str:
    step = steps[index % len(steps)]
    if step.startswith("("):
        inner = step[1:-1]
        step = inner[(index // len(steps)) % len(inner)]
    return step


def _pitches(spec: PlayerSpec, program: Program, index: int, beat: float) -> List[float]:
    kw = spec.kwargs
    degree = value_at(spec.degree, 0, beat) if isinstance(spec.degree, TimeVar) else spec.degree
    if isinstance(degree, list):
        degree = degree[index % len(degree)]
    notes = degree if isinstance(degree, tuple) else (degree,)
    scale = kw.get("scale") or value_at(program.scale, 0, beat) or "major"
    root = _root_semitone(kw["root"] if "root" in kw else program.root, beat)
    oct_ = float(value_at(kw.get("oct", 5), index, beat))
    return [degree_to_midi(float(n), oct_, root, scale) for n in notes if n is not None]


def events_for(slot: str, spec: PlayerSpec, program: Program, form_beats: float) -> List[NoteEvent]:
    """События одного плеера за ``form_beats`` долей (``loop`` не разворачивается)."""
    kw = spec.kwargs
    if spec.synth == "loop":
        return []
    steps = play_steps(spec.degree) if spec.synth == "play" else None
    if spec.synth == "play" and not steps:
        raise ProgramError(f"рисунок play не разбирается: {spec.degree!r}")
    out: List[NoteEvent] = []
    index, beat = 0, 0.0
    default_dur = 0.5 if spec.synth == "play" else 1.0
    while beat < form_beats - 1e-9:
        dur = float(value_at(kw.get("dur", default_dur), index, beat))
        if dur <= 0:
            raise ProgramError(f"{slot}: dur={dur}")
        gate = float(value_at(kw.get("amp", 1), index, beat))
        amp = gate * float(value_at(kw.get("amplify", 1), index, beat))
        if amp > 0:
            sus = float(value_at(kw["sus"], index, beat)) if "sus" in kw else dur
            onset = beat + float(value_at(kw.get("delay", 0), index, beat))
            base = dict(beat=onset, slot=slot, amp=amp, gate=gate, sus_beats=sus, fx=_fx_at(kw, index, beat))
            if steps is not None:
                symbol = _play_symbol(steps, index)
                if symbol not in ". ":
                    sample = int(value_at(kw.get("sample", 0), index, beat))
                    out.append(NoteEvent(synth="play", sample=f"{symbol}{sample}", **base))
            else:
                out.extend(NoteEvent(synth=spec.synth, midi=m, **base) for m in _pitches(spec, program, index, beat))
        index += 1
        beat += dur
    return out


def program_events(code: str, form_beats: Optional[float] = None) -> Tuple[Program, List[NoteEvent]]:
    """Программа → (плееры, события за одну форму)."""
    program = run_program(code)
    form = float(form_beats or program.form_beats or 0)
    if form <= 0:
        raise ProgramError("длина формы неизвестна (нет Clock.future и не передана)")
    events: List[NoteEvent] = []
    for slot, spec in program.players.items():
        events.extend(events_for(slot, spec, program, form))
    return program, events


__all__ = [
    "NoteEvent",
    "Program",
    "ProgramError",
    "TimeVar",
    "degree_to_midi",
    "events_for",
    "program_events",
    "run_program",
    "value_at",
]
