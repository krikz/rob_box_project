"""Аранжировщик: превращает компактную спецификацию трека в Renardo-код с формой.

Зачем (docs/analysis/2026-08-30-music-quality-audit.md, RC4)
===========================================================

Renardo-плеер — бесконечный луп. «Композиция» в нём = расписание изменений
во времени. До этого модуля расписания не было ни в промпте, ни в коде:
LLM писала ``p1 >> blip([0,2,4,7], dur=0.5)`` и это играло одинаково до
самой остановки. Отсюда жалоба «простая мелодия, которая повторяется всё
время» — она буквально повторяется, потому что форму никто не задаёт.

Разделение ответственности здесь такое:

* **LLM отдаёт материал** — тональность, темп, палитра синтов, 2-4 мотива.
  Это то, что языковая модель делает хорошо.
* **Этот модуль отдаёт форму** — когда слой входит, когда уходит, где брейк,
  где кульминация. Это то, что детерминированный код делает надёжно, а LLM
  на длинной дистанции держать не может.

Как форма кодируется
====================

Не через ``Clock.future`` и не через ``def intro()/verse()`` — а декларативно,
через ``var()``-огибающие на ``amp`` каждого слоя. Это идиоматичный для
FoxDot/Renardo приём (в пресете `rypop_120bpm` из миграции 006 живой
человек пишет ровно так: ``vv.amp = var([.5,0],[48,16])``), и у него три
важных свойства:

1. Точность по тактам — таймингом занимается Clock, а не Python-поток.
2. Автоматическое зацикливание — ``var`` проходит форму и начинает заново,
   поэтому DJ-режиму не нужен отдельный планировщик.
3. Проходит все существующие фильтры: нет ``def``, нет ``lambda``,
   нет ``Clock.future`` — то есть нечего вырезать и не на чем падать.

Единицы: длительности ``var`` считаются в БИТАХ (``TimeVar.__init__``:
``dur = metro.bar_length()`` = 4). Поэтому такты умножаются на
:data:`BEATS_PER_BAR`.

Регистры (RC2): октава назначается по РОЛИ слоя, а не по вкусу модели.
Бас, пэд, лид и колокольчики физически разведены по регистрам — без этого
микс слышится как одна дорожка независимо от того, сколько в нём слоёв.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

#: 1 такт = 4 бита в дефолтном метре Renardo (``TempoClock.bar_length()``).
BEATS_PER_BAR = 4

#: Роль -> (имя плеера, октава, базовая амплитуда).
#:
#: Плееры удержаны в d1-d3 / p1-p3 — это ограничение деплоя (d4+/p4+ на
#: роботе не звучат, см. music_skill_prompt.txt).
#:
#: Октавы разведены намеренно: бас 3, пэд 4, лид 5, колокольчики 6. Раньше
#: ``_cap_amp`` зажимал всё в oct<=4 и бас с лидом оказывались в соседних
#: октавах.
#:
#: Амплитуды — ОТНОСИТЕЛЬНЫЙ баланс ролей, а не абсолютная громкость.
#: Сумму держит synthdef ``masterlimiter`` в scsynth, абсолютный уровень —
#: ROS-параметр ``music_master_gain``.
ROLE_PROFILE: Dict[str, Tuple[str, int, float]] = {
    "drums": ("d1", 0, 0.55),
    "hats":  ("d2", 0, 0.24),
    "perc":  ("d3", 0, 0.30),
    "bass":  ("p1", 3, 0.50),
    "lead":  ("p2", 5, 0.52),
    "pad":   ("p3", 4, 0.30),
}

#: Роли, которые играют сэмплами через ``play(...)``, а не синтом.
DRUM_ROLES = frozenset({"drums", "hats", "perc"})

#: Формы: имя -> список секций ``(имя, тактов, {роль: интенсивность 0..1})``.
#:
#: Интенсивность умножается на базовую амплитуду роли. 0.0 = слой молчит
#: (нота всё равно шлётся, но с amp=0 — это дешевле, чем стоп/старт плеера,
#: и не даёт щелчка на входе).
#:
#: Формы намеренно НЕ симметричны: у секций разная длина, а кульминация
#: приходит после брейка. Симметричная сетка 16/16/16/16 на слух — то же
#: самое, что луп.
FORMS: Dict[str, List[Tuple[str, int, Dict[str, float]]]] = {
    # Универсальная дуга. Дефолт: работает и для DJ-петли, и для трека.
    "arc": [
        ("intro",  8,  {"pad": 0.65, "hats": 0.30}),
        ("build",  8,  {"pad": 0.70, "hats": 0.55, "drums": 0.50, "bass": 0.70}),
        ("main",  16,  {"pad": 0.45, "hats": 0.80, "drums": 1.00, "bass": 1.00,
                        "lead": 0.95, "perc": 0.60}),
        ("break",  8,  {"pad": 0.90, "hats": 0.25, "bass": 0.40, "lead": 0.55}),
        ("peak",  16,  {"pad": 0.55, "hats": 1.00, "drums": 1.00, "bass": 1.00,
                        "lead": 1.00, "perc": 0.85}),
        ("outro",  8,  {"pad": 0.50, "hats": 0.20, "drums": 0.35, "bass": 0.30}),
    ],
    # Песенная форма: куплет тише припева, бридж снимает барабаны.
    "verse_chorus": [
        ("intro",   8,  {"pad": 0.60, "hats": 0.35}),
        ("verse",  16,  {"pad": 0.50, "hats": 0.60, "drums": 0.70, "bass": 0.80,
                         "lead": 0.55}),
        ("chorus", 16,  {"pad": 0.60, "hats": 0.90, "drums": 1.00, "bass": 1.00,
                         "lead": 1.00, "perc": 0.70}),
        ("bridge",  8,  {"pad": 0.85, "hats": 0.20, "bass": 0.45, "lead": 0.60}),
        ("chorus2", 16, {"pad": 0.65, "hats": 1.00, "drums": 1.00, "bass": 1.00,
                         "lead": 1.00, "perc": 0.90}),
        ("outro",   8,  {"pad": 0.55, "hats": 0.25, "bass": 0.35}),
    ],
    # Клубная: длинный разгон, тишина перед дропом, дроп на полную.
    "buildup": [
        ("intro",   8,  {"pad": 0.55, "hats": 0.40, "drums": 0.35}),
        ("build",  16,  {"pad": 0.70, "hats": 0.75, "drums": 0.70, "bass": 0.75,
                         "lead": 0.40}),
        ("gap",     4,  {"pad": 0.80}),
        ("drop",   16,  {"hats": 1.00, "drums": 1.00, "bass": 1.00, "lead": 1.00,
                         "perc": 0.90, "pad": 0.35}),
        ("break",   8,  {"pad": 0.85, "bass": 0.45, "lead": 0.50}),
        ("drop2",  16,  {"hats": 1.00, "drums": 1.00, "bass": 1.00, "lead": 0.95,
                         "perc": 1.00, "pad": 0.40}),
        ("outro",   8,  {"pad": 0.60, "hats": 0.20}),
    ],
    # Без ударной сетки: медленные наплывы, для «сделай что-то для души».
    #
    # Live 30.08: первая версия открывалась 16 тактами ОДНОГО пэда. На 80 BPM
    # это 48 секунд почти неподвижного звука — слушатель успевает решить, что
    # робот сломался. Плюс сетка 16/16/16/16 симметрична, а симметричная
    # сетка на слух — тот же луп, только длиннее. Теперь мелодия входит
    # сразу, вполголоса, а секции разной длины.
    "ambient": [
        ("emerge",  8,  {"pad": 0.60, "lead": 0.30}),
        ("drift",  16,  {"pad": 0.85, "lead": 0.55}),
        ("swell",  20,  {"pad": 1.00, "lead": 0.75, "bass": 0.50}),
        ("recede", 12,  {"pad": 0.70, "lead": 0.35}),
    ],
}

#: Дефолтная форма, когда LLM не указала или указала неизвестную.
DEFAULT_FORM = "arc"

#: Диапазон BPM, за который выходить бессмысленно.
BPM_RANGE = (60.0, 180.0)

VALID_ROOTS = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")


class ArrangementError(ValueError):
    """Спецификация не может быть развёрнута в корректный Renardo-код."""


@dataclass
class Layer:
    """Один слой аранжировки — материал без формы.

    Attributes:
        role: ключ из :data:`ROLE_PROFILE`.
        synth: имя синта для мелодических ролей (``dub``, ``blip``, ...).
        pattern: строка паттерна для ударных ролей (``"X..o.X.o"``).
        degrees: ступени лада для мелодических ролей.
        dur: длительность ноты в битах.
        sample: индекс сэмпла для ударных.
        oct_shift: сдвиг относительно октавы роли — на случай, когда бас
            должен уйти ещё ниже или лид ещё выше.
    """

    role: str
    synth: Optional[str] = None
    pattern: Optional[str] = None
    degrees: Sequence[float] = field(default_factory=tuple)
    dur: float = 1.0
    sample: int = 0
    oct_shift: int = 0


@dataclass
class CompositionSpec:
    """Полная спецификация трека — то, что отдаёт LLM.

    Всё, что связано со временем и балансом, сюда НЕ входит: это забота
    аранжировщика.
    """

    bpm: float = 120.0
    root: str = "C"
    scale: str = "minor"
    form: str = DEFAULT_FORM
    layers: Sequence[Layer] = field(default_factory=tuple)
    #: Движение тоники по ступеням лада, например ``[0, 0, 5, 3]``. Один
    #: цикл прогрессии растягивается ровно на одну форму.
    progression: Sequence[int] = field(default_factory=tuple)
    #: Медленный фильтр-свип на всю форму. Даёт «дыхание» даже там, где
    #: слои статичны.
    filter_sweep: bool = True
    #: False -> в конце формы музыка останавливается сама. True (DJ-режим)
    #: -> форма зацикливается бесконечно.
    repeat: bool = True


def _fmt(value: float) -> str:
    """Компактно отформатировать число для Renardo-кода.

    ``1.0 -> "1"``, ``0.25 -> "0.25"``, ``0.5000000001 -> "0.5"``.
    Renardo-код читают люди (он попадает в логи и в save_track), поэтому
    ``0.30000000000000004`` там неуместен.
    """
    rounded = round(float(value), 4)
    if rounded == int(rounded):
        return str(int(rounded))
    return f"{rounded:g}"


def _fmt_list(values: Sequence[float]) -> str:
    return "[" + ", ".join(_fmt(v) for v in values) + "]"


def resolve_form(name: Optional[str]) -> List[Tuple[str, int, Dict[str, float]]]:
    """Вернуть план формы, молча падая на дефолт для неизвестного имени.

    Неизвестная форма — не повод отказать в музыке: LLM регулярно
    выдумывает названия, и лучше сыграть дугу, чем вернуть ошибку.
    """
    return FORMS.get((name or "").strip().lower(), FORMS[DEFAULT_FORM])


def _amp_envelope(
    role: str,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    base_amp: float,
) -> Tuple[List[float], List[int]]:
    """Собрать (значения amp, длительности в битах) для одной роли.

    Соседние секции с одинаковой интенсивностью склеиваются — иначе
    ``var([0.5, 0.5, 0.5], [32, 32, 64])`` порождает лишние переключения
    и делает код нечитаемым.
    """
    amps: List[float] = []
    durs: List[int] = []
    for _name, bars, intensities in plan:
        amp = round(base_amp * float(intensities.get(role, 0.0)), 4)
        beats = int(bars) * BEATS_PER_BAR
        if amps and amps[-1] == amp:
            durs[-1] += beats
        else:
            amps.append(amp)
            durs.append(beats)
    return amps, durs


def _render_layer(
    layer: Layer,
    plan: Sequence[Tuple[str, int, Dict[str, float]]],
    use_filter: bool,
) -> Optional[str]:
    """Отрендерить одну строку Renardo-кода, либо None если слой молчит."""
    profile = ROLE_PROFILE.get(layer.role)
    if profile is None:
        raise ArrangementError(
            f"Неизвестная роль слоя: {layer.role!r}. "
            f"Доступны: {', '.join(sorted(ROLE_PROFILE))}."
        )
    player, role_oct, base_amp = profile

    amps, durs = _amp_envelope(layer.role, plan, base_amp)
    if not any(amps):
        # Роль не участвует ни в одной секции этой формы (например drums в
        # ambient) — плеер не создаём вовсе, чтобы не гонять тихие ноты.
        return None

    if len(amps) == 1:
        amp_expr = _fmt(amps[0])
    else:
        amp_expr = f"var({_fmt_list(amps)}, {_fmt_list(durs)})"

    args: List[str] = []
    if layer.role in DRUM_ROLES:
        if not layer.pattern:
            raise ArrangementError(
                f"Роль {layer.role!r} играет сэмплами — нужен pattern, "
                'например "X..o.X.o".'
            )
        head = f'play({layer.pattern!r}'
        if layer.sample:
            args.append(f"sample={int(layer.sample)}")
    else:
        if not layer.synth:
            raise ArrangementError(
                f"Роль {layer.role!r} играет синтом — нужно поле synth."
            )
        if not layer.degrees:
            raise ArrangementError(
                f"Роль {layer.role!r} без degrees — играть нечего."
            )
        head = f"{layer.synth}({_fmt_list(layer.degrees)}"
        args.append(f"dur={_fmt(layer.dur)}")
        args.append(f"oct={max(2, min(7, role_oct + int(layer.oct_shift)))}")

    args.append(f"amp={amp_expr}")
    # Фильтр-свип вешаем на держащие слои. На ударные не вешаем: срезанная
    # атака бочки слышна как проваленный грув.
    if use_filter and layer.role in ("bass", "pad", "lead"):
        args.append("lpf=gflt")

    return f"{player} >> {head}, " + ", ".join(args) + ")"


def render(spec: CompositionSpec) -> str:
    """Развернуть спецификацию в Renardo-код с формой.

    Returns:
        Многострочный Renardo-код, готовый для ``execute_music_code``.

    Raises:
        ArrangementError: спецификация внутренне противоречива (неизвестная
            роль, ударный слой без паттерна, мелодический без ступеней).
    """
    if not spec.layers:
        raise ArrangementError("Спецификация без слоёв — играть нечего.")

    bpm = max(BPM_RANGE[0], min(BPM_RANGE[1], float(spec.bpm)))
    root = spec.root if spec.root in VALID_ROOTS else "C"
    plan = resolve_form(spec.form)
    total_beats = sum(int(bars) for _n, bars, _i in plan) * BEATS_PER_BAR

    lines: List[str] = ["Clock.clear()", f"Clock.bpm = {_fmt(bpm)}"]

    if spec.progression:
        # Один проход прогрессии растягивается ровно на одну форму, чтобы
        # гармония и аранжировка не разъезжались.
        step = max(1, total_beats // len(spec.progression))
        lines.append(
            f"Root.default = var({_fmt_list(spec.progression)}, {_fmt(step)})"
        )
    else:
        lines.append(f'Root.default = "{root}"')

    lines.append(f'Scale.default = "{spec.scale}"')

    if spec.filter_sweep:
        # Свип длиной в половину формы: за один проход формы фильтр
        # успевает открыться и закрыться.
        lines.append(f"gflt = linvar([700, 4500], {_fmt(total_beats // 2)})")

    for layer in spec.layers:
        rendered = _render_layer(layer, plan, use_filter=spec.filter_sweep)
        if rendered is not None:
            lines.append(rendered)

    if len(lines) <= 4:
        raise ArrangementError(
            "Ни один слой не звучит в выбранной форме — проверь роли "
            f"(форма {spec.form!r} задействует: "
            f"{', '.join(sorted({r for _n, _b, i in plan for r in i}))})."
        )

    if not spec.repeat:
        # Функция, а не lambda: lambda режется AST-фильтром, а Clock.clear
        # передаётся как объект и вызывается планировщиком.
        lines.append(f"Clock.future({total_beats}, Clock.clear)")

    return "\n".join(lines)


#: Длительность ноты по умолчанию для мелодических ролей (в битах).
#:
#: Задаётся здесь, а не запрашивается у LLM: выбор dur — это вопрос
#: плотности фактуры, где модель систематически ошибается в сторону
#: слишком коротких значений (лог: dur 2 -> 1 -> 0.5 -> 0.25 -> 0.125).
#: Роль знает свою плотность лучше.
ROLE_DEFAULT_DUR: Dict[str, float] = {"bass": 1.0, "lead": 0.5, "pad": 4.0}


def parse_notes(raw: Optional[str]) -> Tuple[float, ...]:
    """Разобрать ступени лада из строки вида ``"0, 2, 4, 7"``.

    Принимает и ``"[0,2,4]"``, и ``"0 2 4"`` — маленькие модели пишут
    и так, и так, а отклонять запрос из-за скобок бессмысленно.

    Returns:
        Кортеж ступеней; пустой, если строка пустая.

    Raises:
        ArrangementError: в строке есть что-то, что не число.
    """
    if not raw or not raw.strip():
        return ()
    cleaned = raw.strip().strip("[]()").replace(";", ",").replace(" ", ",")
    out: List[float] = []
    for chunk in cleaned.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            value = float(chunk)
        except ValueError as exc:
            raise ArrangementError(
                f"Ступени лада должны быть числами, получено {chunk!r} "
                f"в {raw!r}. Пример: \"0, 2, 4, 7\"."
            ) from exc
        out.append(int(value) if value == int(value) else value)
    return tuple(out)


def spec_from_flat(
    *,
    bpm: float = 120.0,
    root: str = "C",
    scale: str = "minor",
    form: str = DEFAULT_FORM,
    drums: Optional[str] = None,
    drums_sample: int = 0,
    hats: Optional[str] = None,
    bass_synth: Optional[str] = None,
    bass_notes: Optional[str] = None,
    lead_synth: Optional[str] = None,
    lead_notes: Optional[str] = None,
    pad_synth: Optional[str] = None,
    pad_notes: Optional[str] = None,
    progression: Optional[str] = None,
    repeat: bool = True,
) -> CompositionSpec:
    """Собрать :class:`CompositionSpec` из плоских скалярных аргументов.

    Плоский интерфейс, а не вложенный JSON: маленькие модели заметно
    надёжнее заполняют десяток простых полей, чем одну структуру с
    массивом объектов внутри.

    Слой добавляется только если для него есть И синт, И ступени —
    полупустой слой молча пропускается, а не роняет запрос.
    """
    layers: List[Layer] = []

    if drums and drums.strip():
        layers.append(
            Layer(role="drums", pattern=drums.strip(), sample=int(drums_sample or 0))
        )
    if hats and hats.strip():
        layers.append(Layer(role="hats", pattern=hats.strip(), sample=3))

    for role, synth, notes in (
        ("bass", bass_synth, bass_notes),
        ("lead", lead_synth, lead_notes),
        ("pad", pad_synth, pad_notes),
    ):
        degrees = parse_notes(notes)
        if synth and synth.strip() and degrees:
            layers.append(
                Layer(
                    role=role,
                    synth=synth.strip(),
                    degrees=degrees,
                    dur=ROLE_DEFAULT_DUR[role],
                )
            )

    return CompositionSpec(
        bpm=float(bpm),
        root=(root or "C").strip(),
        scale=(scale or "minor").strip(),
        form=(form or DEFAULT_FORM).strip(),
        layers=tuple(layers),
        progression=tuple(int(v) for v in parse_notes(progression)),
        repeat=bool(repeat),
    )


def form_summary(name: Optional[str]) -> str:
    """Однострочное описание формы — для сообщения LLM и для логов."""
    plan = resolve_form(name)
    total_bars = sum(int(bars) for _n, bars, _i in plan)
    sections = " → ".join(f"{n}({b})" for n, b, _i in plan)
    return f"{sections} = {total_bars} тактов"
