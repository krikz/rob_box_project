"""club_transition.py — переход между треками фейдом, а не жёсткой склейкой.

Issue #3113, план docs/design/2026-09-28-dj-live-coding-quality-plan.md
§5 п.3, §7.1 п.7: у DJ Dave сет держит один темп, уходящий трек гасится
фильтром и фейдером, а не обрывается. ``render_club`` начинает программу с
``Clock.clear()`` — жёсткий стык. :func:`wrap_with_fade` оборачивает
ГОТОВУЮ программу нового трека (что бы ни стояло в её прелюдии) так:

1. Если что-то играет (``Clock.playing`` непуст) — считается доля смены
   ``_rbx_switch_beat = Clock.next_bar() + fade_beats - SWITCH_EARLY_BEATS``,
   и всем играющим плеерам через ``Master()`` ставятся ``lpf`` и
   ``amplify`` на ``linvar``, которые РОВНО к этой доле опускают фильтр
   4000→300 Гц и громкость 1→:data:`FADE_AMPLIFY_TO` (не в ноль, #3166).
2. Новый трек (исходная программа целиком, со своим ``Clock.clear()``)
   запускается функцией ``_rbx_next_track``, запланированной
   ``Clock.schedule`` на ``_rbx_switch_beat``. Пока фейд идёт, новые плееры
   не создаются, поэтому ``Master()`` видит только уходящий трек.
   ``Clock.clear()`` нового трека зовёт ``Player.kill`` → ``reset()``
   (``Players.py:1796``, ``:544``), и у тех же объектов ``d1..p3``
   ``amplify`` снова 1, ``lpf`` — 0: фейд уходящего трека на новый трек
   не протекает.
3. Последней строкой ``_rbx_next_track`` зовёт ``_rbx_track_started(F,
   entry)`` — колбэк владельца (ADR-0142 §10.1, PR-1): снимок фазы клока
   и строка ``[#3112] … started …`` в лог уже ПОСЛЕ ``Clock.clear`` →
   ``set_time`` → плееров нового трека, и teardown старых SC-нод в этот
   же момент (а не в начале фейда).
4. Если ничего не играет — ``_rbx_next_track()`` вызывается сразу.

Почему не настоящий кроссфейд двух треков: у робота звучат только слоты
``d1..d3``/``p1..p3`` (``renardo_sanitizer._ALLOWED_PLAYER_SLOTS``, #1804),
и новый трек переприсваивает ТЕ ЖЕ объекты плееров. Два трека одновременно
звучать не могут, поэтому стык делается так: уходящий трек уводится
фильтром и фейдером до уровня :data:`FADE_AMPLIFY_TO`, и в ту же долю
(без lead-паузы) входит новый — с секции основного уровня, если программа
собрана с ``dj_entry`` (``core/club_arranger._clock_lines``).

Тайм-лайн issue #3166 (124 BPM, 1 доля = 0,484 с; живой замер 29.09,
``[#3112] до exec=584.594 после exec=584.81``, ``transition=fade``) — ДО:

* exec на доле ≈584,6; ``linvar`` от ``Clock.now()`` доводит ``amplify``
  до 0 на доле ≈616,6 (линейно по амплитуде: последние ~3 доли уже
  ниже −20 dB);
* ``_rbx_next_track`` стоял на ``Clock.next_bar() + 32`` = 588 + 32 = 620:
  3,4 доли (≈1,65 с) громкость уже 0, а новый трек ещё не начат;
* внутри — ``Clock.clear()`` и ``set_time`` c ``ALIGN_LEAD_BEATS = 2``:
  плееры встают на ``next_bar()`` через 2 доли (≈0,97 с);
* первые ноты нового трека звучат ещё через ``Clock.latency`` = 0,25 с;
* итого ≈2,9 с от нуля фейда до первой ноты по расчёту (живой замер:
  2,4 с цифрового нуля + хвост −75…−92 dB; остаток — хвосты нот
  уходящего трека с ``sus``); и новый трек начинал с интро шаблона
  (``long_build_32``: только пэд 0.11) — ещё ~−40 dB.

ПОСЛЕ (по коду, на роботе НЕ проверено):

* ``linvar`` кончается ровно на ``_rbx_switch_beat`` (длительность
  ``_rbx_switch_beat − Clock.now()``), уровень уходящего трека в конце —
  ``FADE_AMPLIFY_TO`` (−6 dB по амплитуде, плюс ``lpf`` 300 Гц);
* ``_rbx_next_track`` — за ``SWITCH_EARLY_BEATS`` (1/16 доли, ≈30 мс) до
  границы такта: ``Clock.clear`` успевает вычистить из очереди события
  уходящих плееров на самой границе. Если бы колбэк стоял РОВНО на границе,
  он попал бы в один ``QueueBlock`` с событиями ``d1..p3`` уходящего трека
  (``TempoClock.py`` ``Queue.add``: одинаковая доля → тот же блок; функции
  в блоке зовутся первыми), и после ``d1 >> ...`` этот же объект ``d1``
  был бы вызван ещё раз из старого блока — две цепочки событий у одного
  плеера (двойные ноты до конца трека);
* с ``dj_entry`` (``ROB_BOX_MUSIC_ALIGN_CLOCK=1``) клок ставится ровно на
  ``k·F + entry`` без lead-долей, и плееры встают на ``now()``
  (``Clock.now_flag``) — первые ноты через ``Clock.latency`` = 0,25 с
  после колбэка, т.е. ≈0,22 с после границы такта уходящего трека;
* без ``ROB_BOX_MUSIC_ALIGN_CLOCK`` — плееры встают на ``next_bar()``, а
  это и есть граница смены (колбэк стоит на 1/16 доли раньше неё).

Опора на исходники renardo_lib 0.9.13 (прочитано, на роботе НЕ проверено):

* ``runtime/__init__.py``: ``def Master(): return Group(*Clock.playing)``;
  там же ``_futureBarDecorator`` — штатный приём Renardo «функция на
  границе такта с ``Clock.now_flag = True``»;
* ``Players.py`` ``Group.__setattr__`` — ``setattr`` на каждого плеера
  группы;
* ``Players.py`` ``send_osc_message``: ``amp = amp * amplify`` — секционные
  гейты ``amp=var(...)`` уходящего трека не перетираются (фейд идёт по
  ``amplify``; пампинг ``amplify=[...]`` на время фейда заменяется им);
* ``TimeVar.__init__(values, dur, start=0)``: ``start`` — сдвиг фазы в
  долях клока, поэтому ``start=Clock.now()`` начинает свип с момента
  исполнения;
* ``TempoClock.schedule(obj, beat)`` / ``next_bar()`` — вызов функции на
  абсолютной доле; ``TempoClock.clear()`` чистит очередь и убивает
  плееры (``Player.kill`` → ``reset``).

Санитайзер (``core/renardo_sanitizer.py``) НЕ ослабляется: ``def``/``if``/
``global`` и имена без двойного подчёркивания его AST-фильтр пропускает,
запрещённых токенов в обёртке нет — это закреплено тестом, который гоняет
обёрнутый ``render_club`` через ``sanitize_renando`` и сверяет, что
санитайзер не изменил ни одной строки обёртки.

Teardown (issue #3137/#3148/#3157): ``MusicManager.execute_code`` раньше
планировал ramp/freeAll от ``Clock.next_bar()`` СТАРОГО клока в момент
``exec`` обёртки — т.е. в первом такте фейда, и обрывал хвосты уходящего
трека (ADR-0142 §1 п.5). Теперь для обёрнутой программы
(:func:`is_fade_wrapped`) ``execute_code`` teardown НЕ планирует — его
делает ``_rbx_track_started`` в момент реального старта нового трека.
"""

from __future__ import annotations

import ast
import textwrap
from typing import List

#: Длина фейда уходящего трека в тактах (issue #3113: «за 4–8 тактов»).
FADE_BARS = 8
FADE_BARS_RANGE = (1, 16)
#: Свип фильтра уходящего трека, Гц.
FADE_LPF_FROM = 4000
FADE_LPF_TO = 300
#: Уровень ``amplify`` уходящего трека в конце фейда (issue #3166). Не 0:
#: новый трек в тех же слотах не может звучать поверх старого, поэтому
#: фейд в ноль = провал в тишину перед стыком (acceptance #3166: провал
#: ≤ 10 dB). 0.5 = −6 dB по амплитуде; вместе с ``lpf`` 300 Гц реальная
#: просадка RMS на роботе НЕ измерена — проверка jack_rec.
FADE_AMPLIFY_TO = 0.5
#: На сколько долей раньше границы такта зовётся ``_rbx_next_track``
#: (issue #3166, см. модуль): меньше шага самой мелкой сетки клубных
#: паттернов (1/8 доли у групп ``(.X)`` при ``dur=1/4``), больше джиттера
#: старта потока ``__run_block``. 1/16 доли = ≈30 мс на 124 BPM.
SWITCH_EARLY_BEATS = 0.0625

BEATS_PER_BAR = 4
#: Имя функции-запуска нового трека в пространстве имён Renardo.
NEXT_TRACK_FN = "_rbx_next_track"
#: Имя колбэка владельца «трек стартовал» (ADR-0142 §10.1). Кладёт
#: ``MusicManager.execute_code`` в пространство имён перед ``exec``.
TRACK_STARTED_FN = "_rbx_track_started"
#: Доля смены трека (глобальное имя в пространстве имён Renardo).
SWITCH_BEAT_VAR = "_rbx_switch_beat"
FADE_BEATS_VAR = "_rbx_fade_beats"


def _top_level_names(tree: ast.Module) -> List[str]:
    """Имена, которые программа связывает на верхнем уровне.

    Внутри функции они стали бы локальными — обёртка объявляет их
    ``global``, чтобы семантика программы не изменилась (``Clock.future``-
    колбэки и следующий вызов ``exec`` видят их как раньше).
    """
    names = set()
    for stmt in tree.body:
        if isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(stmt.name)
            continue
        for node in ast.walk(stmt):
            if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                names.add(node.id)
    return sorted(names)


def _check_indentable(tree: ast.Module) -> None:
    """Многострочный строковый литерал сдвиг отступа изменил бы — отказ."""
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and "\n" in node.value:
            raise ValueError("переход fade: многострочный строковый литерал в программе трека")


def fade_seconds(bpm: float, fade_bars: int = FADE_BARS) -> float:
    """Верхняя оценка задержки старта нового трека: фейд + до одного такта."""
    return (fade_bars + 1) * BEATS_PER_BAR * 60.0 / float(bpm)


def is_fade_wrapped(code: str) -> bool:
    """Программа — результат :func:`wrap_with_fade` (старт нового трека отложен)."""
    return f"def {NEXT_TRACK_FN}():" in code and f"{TRACK_STARTED_FN}(" in code


def _check_fade_bars(fade_bars: int) -> None:
    if isinstance(fade_bars, bool) or not isinstance(fade_bars, int):
        raise ValueError(f"fade_bars должен быть целым, получено {fade_bars!r}")
    if not FADE_BARS_RANGE[0] <= fade_bars <= FADE_BARS_RANGE[1]:
        raise ValueError(f"fade_bars вне диапазона {FADE_BARS_RANGE[0]}..{FADE_BARS_RANGE[1]}: {fade_bars}")


def wrap_with_fade(
    program: str, fade_bars: int = FADE_BARS, *, form_beats: int = 0, entry_beats: int = 0,
) -> str:
    """Обернуть программу нового трека в переход фейдом (см. модуль).

    Args:
        program: программа нового трека (со своим ``Clock.clear()``).
        fade_bars: длина фейда уходящего трека в тактах.
        form_beats, entry_beats: длина формы и доля входа нового трека —
            только для колбэка ``_rbx_track_started`` (лог фазы).

    Raises:
        ValueError: ``fade_bars`` вне :data:`FADE_BARS_RANGE`, программа не
            парсится или её нельзя безопасно сдвинуть внутрь функции.
    """
    _check_fade_bars(fade_bars)
    try:
        tree = ast.parse(program)
    except SyntaxError as exc:
        raise ValueError(f"переход fade: программа трека не парсится: {exc.msg}") from exc
    _check_indentable(tree)
    beats = fade_bars * BEATS_PER_BAR
    durs = f"[{FADE_BEATS_VAR}, {FADE_BEATS_VAR}, {FADE_BEATS_VAR}]"
    body = []
    names = _top_level_names(tree)
    if names:
        body.append("global " + ", ".join(names))
    body.append(program.rstrip("\n"))
    body.append(f"{TRACK_STARTED_FN}({int(form_beats)}, {int(entry_beats)})")
    lines = [
        f"# переход: фейд уходящего трека {fade_bars} тактов (lpf + amplify до {FADE_AMPLIFY_TO}), "
        "новый трек — в долю конца фейда",
        f"def {NEXT_TRACK_FN}():",
        textwrap.indent("\n".join(body), "    ", lambda line: bool(line.strip())),
        "",
        "if Clock.playing:",
        f"    {SWITCH_BEAT_VAR} = Clock.next_bar() + {beats} - {SWITCH_EARLY_BEATS}",
        f"    {FADE_BEATS_VAR} = {SWITCH_BEAT_VAR} - Clock.now()",
        f"    Master().lpf = linvar([{FADE_LPF_FROM}, {FADE_LPF_TO}, {FADE_LPF_TO}], {durs}, start=Clock.now())",
        f"    Master().amplify = linvar([1, {FADE_AMPLIFY_TO}, {FADE_AMPLIFY_TO}], {durs}, start=Clock.now())",
        f"    Clock.schedule({NEXT_TRACK_FN}, {SWITCH_BEAT_VAR})",
        "else:",
        f"    {NEXT_TRACK_FN}()",
    ]
    return "\n".join(lines) + "\n"


__all__ = [
    "FADE_AMPLIFY_TO",
    "FADE_BARS",
    "FADE_BARS_RANGE",
    "NEXT_TRACK_FN",
    "SWITCH_EARLY_BEATS",
    "TRACK_STARTED_FN",
    "fade_seconds",
    "is_fade_wrapped",
    "wrap_with_fade",
]
