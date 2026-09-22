#!/usr/bin/env python3
"""voice_threshold_sweep.py — подобрать IDENTIFY_THRESHOLD по живым данным
робота, а не на глаз (issue #2747, ADR-0123 §6).

Зачем
-----
Порог узнавания диктора ``IDENTIFY_THRESHOLD = 0.72`` калибровался
(#2348) на СИНТЕТИЧЕСКИХ голосах MiniMax. На живом человеке он
недостижим: замер 22.09.2026 на роботе дал коридор 0.44–0.61 с
единственным выходом за 0.72 за пятнадцать минут разговора. Следствие
видно в базе — человек, которого перестали узнавать, представляется
заново, ``register_speaker`` заводит нового диктора, и пара дублей,
слитая вручную, восстанавливается за четверть часа.

Взять порог «покрупнее» нельзя: без измерения МЕЖПЕРСОННОГО разброса
любое число — тот же самый плейсхолдер 0.45, из-за которого чужого
человека опознали как хозяина (#2771, соседняя карточка про лицо).
Нужны два распределения: насколько далеко расходится один и тот же
голос и насколько близко подходят разные.

Откуда берутся данные (и почему робота менять не надо)
------------------------------------------------------
``speaker_id_node`` УЖЕ печатает на каждую реплику полный расклад по
кандидатам::

    identify candidates: best='Дэнчик'(1ae4b0ac) score=0.440 |
        second='Дэнчик'(c9e981cb) score=0.320 | gap=0.120

Если известно, КТО говорил в этот момент, одна и та же строка даёт
сразу обе выборки: скор против профиля говорившего — внутриперсонная
точка, скоры против всех прочих профилей — межперсонные. Поэтому сбор
данных это не новый код на роботе, а разметка уже существующего лога
по времени.

Протокол сбора (15–20 минут на двоих)
--------------------------------------
1. Человек А представляется роботу («меня зовут …») — создаётся профиль.
2. А разговаривает 15–20 реплик: близко и издалека, тихо и громко,
   сидя и стоя. Чем разнообразнее, тем честнее порог.
3. Человек Б представляется и так же разговаривает 15–20 реплик.
4. Снять лог и прогнать этот скрипт, разметив окна по времени::

       docker logs voice-assistant > /tmp/voice.log
       python3 voice_threshold_sweep.py /tmp/voice.log \\
           --speaker "Дэнчик:19:40-19:55" --speaker "Гость:19:56-20:10"

   Время — локальное на роботе (как показывает ``date``), формат
   ``ЧЧ:ММ``. Окна размечают, кто говорил; скрипт сам разберёт, какие
   скоры внутриперсонные, а какие межперсонные.

Что считает
-----------
Для каждой выборки — минимум, квартили, максимум. Дальше по двум
распределениям считается, во что обходится каждый возможный порог: FRR
(доля своих, которых порог отверг) и FAR (доля чужих, которых он
пропустил). Рекомендуется порог с минимальной суммой ошибок, и отдельно
показывается точка равных ошибок (EER) — классический ориентир для
биометрии.

Скрипт НИЧЕГО не меняет: ни на роботе, ни в конфигах. Он читает лог и
печатает таблицу. Решение о новом значении принимает человек, глядя на
разрыв между распределениями. Если распределения перекрываются, честный
вывод — «порогом эти голоса не развести, чинить надо тракт», а не
«возьмём серединку».
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


#: Строка ноды с раскладом по кандидатам. Внутри — повторяющиеся блоки
#: вида ``<роль>='<имя>'(<id>) score=<число>``; их разбирает _CANDIDATE_RE.
_LINE_RE = re.compile(r'identify candidates:\s*(?P<body>.+)')

#: Один кандидат в строке. ``sid`` — короткий префикс UUID, как его печатает
#: нода. Имени достаточно для разметки, id оставлен для отладки глазами.
_CANDIDATE_RE = re.compile(
    r"(?:best|second|third)='(?P<name>[^']*)'\((?P<sid>[0-9a-f]+)\)\s*"
    r"score=(?P<score>[0-9.]+)"
)

#: Отметка времени ROS в квадратных скобках — epoch с наносекундами.
_TS_RE = re.compile(r'\[(?P<ts>\d{10}\.\d+)\]')

#: Длительность РЕЧИ в реплике (после VAD) — приписывается к той же строке
#: с момента #2747. На старых логах её нет, и это нормально: тогда отчёт
#: просто не покажет разрез по длительности, вместо того чтобы врать.
_VOICED_RE = re.compile(r'речь=(?P<voiced>[0-9.]+)s')

#: Шаблон окна в аргументе --speaker: хвост строки «ЧЧ:ММ-ЧЧ:ММ».
_SPAN_RE = re.compile(
    r'(?P<h1>\d{1,2}):(?P<m1>\d{2})\s*-\s*(?P<h2>\d{1,2}):(?P<m2>\d{2})\s*$'
)


@dataclass
class Window:
    """Кто говорил в промежутке ``[start_min, end_min)`` локального времени."""

    speaker: str
    start_min: int
    end_min: int


@dataclass
class Samples:
    """Две выборки косинусов, ради которых всё и затевается."""

    intra: List[float] = field(default_factory=list)
    inter: List[float] = field(default_factory=list)
    #: (речь в секундах, косинус против своего профиля) — только для тех
    #: реплик, где нода напечатала ``речь=``. Нужна, чтобы отделить «порог
    #: высоковат» от «человеку нечего было сказать»: замер на этом же
    #: пайплайне даёт r(duration, score) = 0.88, так что низкий score на
    #: короткой речи — не повод трогать порог.
    intra_by_voiced: List[Tuple[float, float]] = field(default_factory=list)


def parse_window(spec: str) -> Window:
    """``"Имя:19:40-19:55"`` -> :class:`Window`.

    Разбираем С КОНЦА по шаблону времени, а не ``split(':')``: в имени
    двоеточий не ждём, но время само содержит два, и наивное разбиение
    развалило бы строку. Так ``"Дэнчик:19:40-19:55"`` читается
    однозначно, не требуя кавычек внутри кавычек.
    """
    m = _SPAN_RE.search(spec)
    if not m:
        raise argparse.ArgumentTypeError(
            f'не разобрал окно {spec!r}; ожидается «Имя:ЧЧ:ММ-ЧЧ:ММ», '
            'например «Дэнчик:19:40-19:55»'
        )
    name = spec[: m.start()].rstrip(': ').strip()
    if not name:
        raise argparse.ArgumentTypeError(f'в {spec!r} не указано имя говорившего')
    start = int(m.group('h1')) * 60 + int(m.group('m1'))
    end = int(m.group('h2')) * 60 + int(m.group('m2'))
    if end <= start:
        raise argparse.ArgumentTypeError(
            f'в {spec!r} конец окна не позже начала — окно через полночь '
            'разбей на два'
        )
    return Window(speaker=name, start_min=start, end_min=end)


def minutes_of_day(epoch: float, tz_offset_hours: float) -> int:
    """Локальные «минуты от полуночи» для epoch-отметки из лога.

    Робот пишет лог в UTC, а оператор размечает окна по настенным часам,
    поэтому смещение обязано быть явным параметром, а не браться из
    окружения машины, где скрипт запускают: её часовой пояс к логам
    робота отношения не имеет.
    """
    local = epoch + tz_offset_hours * 3600.0
    return int((local % 86400.0) // 60)


def speaker_at(windows: List[Window], minute: int) -> Optional[str]:
    for w in windows:
        if w.start_min <= minute < w.end_min:
            return w.speaker
    return None


def same_person(candidate_name: str, speaker: str) -> bool:
    """Сравнение ИМЁН — с точностью до регистра и пробелов.

    Сравниваем именно имена, а не id. Тёзки-дубли одного человека
    (#2747: два профиля «Дэнчик») обязаны попасть в ОДНУ внутриперсонную
    выборку — иначе собственный дубль засчитается как «чужой» и раздует
    межперсонное распределение, испортив ровно то, что измеряем.
    """
    return candidate_name.strip().lower() == speaker.strip().lower()


def collect(
    lines: List[str], windows: List[Window], tz_offset_hours: float
) -> Tuple[Samples, Dict[str, int], int]:
    """Разобрать лог в две выборки плюс счётчики для отчёта оператору."""
    samples = Samples()
    per_speaker: Dict[str, int] = {}
    unlabelled = 0
    for line in lines:
        lm = _LINE_RE.search(line)
        if not lm:
            continue
        tm = _TS_RE.search(line)
        if not tm:
            continue
        minute = minutes_of_day(float(tm.group('ts')), tz_offset_hours)
        speaker = speaker_at(windows, minute)
        if speaker is None:
            unlabelled += 1
            continue
        per_speaker[speaker] = per_speaker.get(speaker, 0) + 1
        vm = _VOICED_RE.search(line)
        voiced = float(vm.group('voiced')) if vm else None
        for cm in _CANDIDATE_RE.finditer(lm.group('body')):
            score = float(cm.group('score'))
            if same_person(cm.group('name'), speaker):
                samples.intra.append(score)
                if voiced is not None:
                    samples.intra_by_voiced.append((voiced, score))
            else:
                samples.inter.append(score)
    return samples, per_speaker, unlabelled


def quantile(sorted_values: List[float], q: float) -> float:
    if not sorted_values:
        return float('nan')
    idx = q * (len(sorted_values) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(sorted_values) - 1)
    frac = idx - lo
    return sorted_values[lo] * (1 - frac) + sorted_values[hi] * frac


def describe(name: str, values: List[float]) -> str:
    if not values:
        return f'  {name}: нет данных'
    v = sorted(values)
    return (
        f'  {name}: n={len(v)}  min={v[0]:.3f}  p05={quantile(v, 0.05):.3f}  '
        f'медиана={quantile(v, 0.5):.3f}  p95={quantile(v, 0.95):.3f}  '
        f'max={v[-1]:.3f}'
    )


def report_by_voiced(points: List[Tuple[float, float]]) -> str:
    """Разрез «сколько было речи → какой вышел скор».

    Главный вопрос, на который отвечает таблица: низкий скор это «порог
    высоковат» или «говорить было нечего». Если короткие реплики дают
    заметно худший медианный скор, чем длинные, чинить надо сбор аудио, а
    не порог — и наоборот, ровная картина по длительностям означает, что
    дело действительно в пороге.

    Границы корзин выбраны по тому, что реально приходит роботу (замер:
    4.3–5.6с), плюс отдельная корзина для длинных реплик, где по
    имеющимся данным скор заметно выше.
    """
    edges = [(0.0, 2.0), (2.0, 4.0), (4.0, 6.0), (6.0, 100.0)]
    lines = ['Скор в разрезе по длительности РЕЧИ:']
    for lo, hi in edges:
        vals = sorted(s for v, s in points if lo <= v < hi)
        if not vals:
            continue
        mid = vals[len(vals) // 2]
        label = f'{lo:.0f}-{hi:.0f}s' if hi < 100 else f'{lo:.0f}s+'
        lines.append(
            f'  речь {label:>8}: n={len(vals):<4} медиана={mid:.3f}  '
            f'min={vals[0]:.3f} max={vals[-1]:.3f}'
        )
    return "\n".join(lines)


def sweep(samples: Samples, step: float = 0.01) -> List[Tuple[float, float, float]]:
    """Для каждого порога — ``(порог, FRR, FAR)``.

    FRR — доля СВОИХ реплик, которые порог отверг («робот меня не
    узнаёт»). FAR — доля ЧУЖИХ, которые он пропустил («робот назвал меня
    чужим именем»). Обе считаются прямо по собранным выборкам, без
    предположений о форме распределения: точек всего пара десятков, и
    любая параметрическая модель здесь была бы додумыванием.
    """
    out: List[Tuple[float, float, float]] = []
    steps = int(round(1.0 / step))
    for i in range(steps + 1):
        t = i * step
        frr = (
            sum(1 for s in samples.intra if s < t) / len(samples.intra)
            if samples.intra else float('nan')
        )
        far = (
            sum(1 for s in samples.inter if s >= t) / len(samples.inter)
            if samples.inter else float('nan')
        )
        out.append((t, frr, far))
    return out


def _force_utf8_stdout() -> None:
    """Печатать кириллицу можно было и на Windows.

    Отчёт целиком на русском, а консоль Windows по умолчанию отдаёт
    Python'у cp1252 — первая же строка падает с ``UnicodeEncodeError``,
    ещё до того, как оператор увидит хоть одно число. Скрипт читает лог
    и ничего не меняет, поэтому запускают его откуда удобно, включая
    рабочую станцию. ``errors='replace'`` — на случай экзотической
    консоли, где и UTF-8 не встанет: лучше вопросики в паре символов,
    чем стек вместо отчёта.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding='utf-8', errors='replace')
        except (AttributeError, ValueError, OSError):
            pass


def main(argv: Optional[List[str]] = None) -> int:
    _force_utf8_stdout()
    ap = argparse.ArgumentParser(
        description=(
            'Подобрать порог узнавания диктора по живому логу робота '
            '(issue #2747). Ничего не меняет — только читает и считает.'
        )
    )
    ap.add_argument('logfile', help='файл с выводом `docker logs voice-assistant`')
    ap.add_argument(
        '--speaker', action='append', dest='windows', default=[], metavar='СПЕЦ',
        help=(
            'кто говорил в окне: «Имя:ЧЧ:ММ-ЧЧ:ММ» по локальному времени '
            'робота. Можно указывать много раз.'
        ),
    )
    ap.add_argument(
        '--tz-offset', type=float, default=3.0, metavar='ЧАСЫ',
        help=(
            'смещение локального времени робота от UTC (по умолчанию 3.0 — '
            'робот стоит в UTC+3). Задаётся явно, а не берётся из окружения '
            'этой машины: её часовой пояс к логам робота отношения не имеет.'
        ),
    )
    ap.add_argument(
        '--current-threshold', type=float, default=0.72, metavar='X',
        help='действующий порог, чтобы показать его цену (по умолчанию 0.72)',
    )
    args = ap.parse_args(argv)

    if not args.windows:
        ap.error(
            'нужно хотя бы одно --speaker: без разметки «кто говорил» лог это '
            'просто числа, и отличить свою реплику от чужой нечем'
        )
    windows = [parse_window(w) for w in args.windows]

    try:
        with open(args.logfile, encoding='utf-8', errors='replace') as fh:
            lines = fh.readlines()
    except OSError as exc:
        print(f'не открыть {args.logfile!r}: {exc}', file=sys.stderr)
        return 2

    samples, per_speaker, unlabelled = collect(lines, windows, args.tz_offset)

    print(f'Разобрано строк лога: {len(lines)}')
    for sp, n in sorted(per_speaker.items()):
        print(f'  реплик размечено как «{sp}»: {n}')
    if unlabelled:
        print(f'  вне окон (пропущено): {unlabelled}')
    print()

    if not samples.intra:
        print(
            'Внутриперсонных точек ноль. Чаще всего это значит, что имя в '
            '--speaker не совпадает с именем профиля в БД (сравнение по '
            'имени, регистр не важен) — сверься с `speaker_db_admin.py list`. '
            'Второй вариант: не то смещение --tz-offset, и окна не попали '
            'ни на одну реплику.'
        )
        return 1

    print('Распределения косинусов:')
    print(describe('свой голос  (intra)', samples.intra))
    print(describe('чужой голос (inter)', samples.inter))
    print()

    if samples.intra_by_voiced:
        print(report_by_voiced(samples.intra_by_voiced))
        print()
    else:
        print(
            'В логе нет пометок «речь=» — значит он снят до #2747. Разрез по '
            'длительности пропущен; на свежем логе он покажет, сколько '
            'отказов на самом деле объясняются короткой речью, а не порогом.'
        )
        print()

    if not samples.inter:
        print(
            'Межперсонных точек ноль — в логе был только один говорящий.\n'
            'Верхнюю границу порога это уже даёт (выше p05 своих ставить\n'
            'нельзя), но НЕ показывает, не пускает ли порог чужих. Для\n'
            'честного выбора нужен второй человек.'
        )
        print(f'  Порог не выше p05 своих: {quantile(sorted(samples.intra), 0.05):.2f}')
        return 0

    rows = sweep(samples)
    best = min(rows, key=lambda r: r[1] + r[2])
    eer = min(rows, key=lambda r: abs(r[1] - r[2]))

    cur = args.current_threshold
    cur_row = min(rows, key=lambda r: abs(r[0] - cur))
    print(
        f'Действующий порог {cur:.2f}: отвергает своих {cur_row[1] * 100:.0f}%, '
        f'пропускает чужих {cur_row[2] * 100:.0f}%'
    )
    print(
        f'Минимум суммы ошибок: {best[0]:.2f} — отвергает своих '
        f'{best[1] * 100:.0f}%, пропускает чужих {best[2] * 100:.0f}%'
    )
    print(
        f'Точка равных ошибок (EER): {eer[0]:.2f} — по '
        f'{eer[1] * 100:.0f}% тех и других'
    )
    print()

    overlap_lo = max(min(samples.inter), min(samples.intra))
    overlap_hi = min(max(samples.inter), max(samples.intra))
    if overlap_hi > overlap_lo:
        print(
            f'⚠ Распределения ПЕРЕКРЫВАЮТСЯ на {overlap_lo:.3f}–{overlap_hi:.3f}. '
            'В этой зоне свой от чужого не отделяется ни при каком пороге — '
            'числа выше показывают лучший компромисс, а не решение. Если '
            'перекрытие широкое, чинить надо не порог, а то, что даёт '
            'эмбеддинги: длительность реплики, тракт записи, модель.'
        )
    else:
        print(
            f'✅ Распределения РАЗДЕЛЯЮТСЯ: чужие не выше {max(samples.inter):.3f}, '
            f'свои не ниже {min(samples.intra):.3f}. Порог безопасно ставить '
            f'посередине: {(max(samples.inter) + min(samples.intra)) / 2:.2f}'
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
