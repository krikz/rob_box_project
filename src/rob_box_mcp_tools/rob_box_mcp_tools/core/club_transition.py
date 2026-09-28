"""club_transition.py — переход между треками фейдом, а не жёсткой склейкой.

Issue #3113, план docs/design/2026-09-28-dj-live-coding-quality-plan.md
§5 п.3, §7.1 п.7: у DJ Dave сет держит один темп, уходящий трек гасится
фильтром и фейдером, а не обрывается. ``render_club`` начинает программу с
``Clock.clear()`` — жёсткий стык. :func:`wrap_with_fade` оборачивает
ГОТОВУЮ программу нового трека (что бы ни стояло в её прелюдии) так:

1. Если что-то играет (``Clock.playing`` непуст) — всем играющим плеерам
   через ``Master()`` ставятся ``lpf`` и ``amplify`` на ``linvar``, которые
   за ``fade_bars`` тактов опускают фильтр 4000→300 Гц и громкость 1→0.
2. Новый трек (исходная программа целиком, со своим ``Clock.clear()``)
   запускается функцией, запланированной ``Clock.schedule`` на границу
   такта после конца фейда. Пока фейд идёт, новые плееры не создаются,
   поэтому ``Master()`` видит только уходящий трек.
3. Если ничего не играет — новый трек стартует сразу, как без перехода.

Опора на исходники renardo_lib (прочитано, на роботе НЕ проверено):

* ``runtime/__init__.py``: ``def Master(): return Group(*Clock.playing)``;
* ``Players.py`` ``Group.__setattr__`` — ``setattr`` на каждого плеера
  группы (там же ``Group.iterate`` сам пишет ``player.amplify=TimeVar(...)``);
* ``Players.py`` ``send_osc_message``: ``amp = amp * amplify`` — секционные
  гейты ``amp=var(...)`` уходящего трека не перетираются, слои, которые
  гейт держит в нуле, не «всплывают» (фейд идёт по ``amplify``; пампинг
  ``amplify=[...]`` на время фейда заменяется им — трек всё равно уходит);
* ``TimeVar.__init__(values, dur, start=0)``: ``start`` — сдвиг фазы в
  долях клока, поэтому ``start=Clock.now()`` начинает свип с момента
  исполнения, а не с доли 0 клока;
* ``TempoClock.schedule(obj, beat)`` / ``next_bar()`` — вызов функции на
  абсолютной доле; ``TempoClock.clear()`` чистит очередь и убивает
  плееры (его делает прелюдия нового трека в момент старта).

Санитайзер (``core/renardo_sanitizer.py``) НЕ ослабляется: ``def``/``if``/
``global`` и имена без двойного подчёркивания его AST-фильтр пропускает,
запрещённых токенов в обёртке нет — это закреплено тестом, который гоняет
обёрнутый ``render_club`` через ``sanitize_renando`` и сверяет, что
санитайзер не изменил ни одной строки обёртки.

Известное ограничение (задокументировано, не скрыто; issue #3137):
``MusicManager.execute_code`` видит строку ``Clock.clear()`` в исходном
тексте программы — она попадает в код и по ``wrap_with_fade`` (внутри тела
``_rbx_next_track``, ГДЕ ВЫПОЛНЕНИЕ ОТЛОЖЕНО до ``Clock.schedule``), и
проверка ``"Clock.clear()" in code`` этого не различает: она текстовая, не
про факт исполнения.

Ревью координатора по #3137 (R2) поправило корневой фикс в
``execute_code``: раньше он слал ramp/freeAll СИНХРОННО сразу после
``exec`` — теперь он ОТКЛАДЫВАЕТ его через
``MusicManager._schedule_transition_cleanup`` на момент, вычисленный из
``Clock.next_bar() - Clock.now()`` (в долях, переведённых в секунды по
текущему BPM) в момент, когда ``exec`` только что вернулся. Для прямого
(не-fade) перехода это ровно момент старта нового трека — дыра закрывается
(см. ``MusicManager._transition_cleanup_delay_seconds``). Для ЭТОГО,
fade-пути, это НЕ то же самое: в момент, когда ``exec`` обёртки
``wrap_with_fade`` возвращается, реально выполнилось только определение
``_rbx_next_track`` и запуск фейда (``Clock.schedule(_rbx_next_track,
Clock.next_bar() + fade_beats)``) — сам ``Clock.clear()`` уходящего трека
внутри ``_rbx_next_track`` ЕЩЁ НЕ исполнился. Поэтому
``_transition_cleanup_delay_seconds`` в этот момент видит ``Clock.next_bar()``
СТАРОГО (играющего) клока — обычно в пределах одного такта, а не
``fade_beats`` (по умолчанию 8 тактов = 32 доли) вперёд, когда реально
должен стартовать новый трек. Итог: ramp/freeAll теперь срабатывает не
мгновенно в начале фейда (как раньше), а где-то в течение первого такта
фейда — ближе к реальному старту нового трека, чем было, но всё ещё
СИЛЬНО раньше конца 8-тактового свипа ``Master().lpf``/``Master().amplify``,
то есть всё ещё обрывает фейд, а не даёт ему доиграть. Корректный фикс —
считать дедлайн от ``_rbx_next_track``'а (т.е. изнутри самой обёртки, где
уже известны ``fade_beats`` и реальный целевой ``Clock.next_bar()``), а не
от состояния клока на момент ``exec`` обёртки — отдельная правка,
зона #3112, здесь сознательно не сделана (шире карточки #3137).
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

BEATS_PER_BAR = 4
#: Имя функции-запуска нового трека в пространстве имён Renardo.
NEXT_TRACK_FN = "_rbx_next_track"


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


def wrap_with_fade(program: str, fade_bars: int = FADE_BARS) -> str:
    """Обернуть программу нового трека в переход фейдом (см. модуль).

    Raises:
        ValueError: ``fade_bars`` вне :data:`FADE_BARS_RANGE`, программа не
            парсится или её нельзя безопасно сдвинуть внутрь функции.
    """
    if isinstance(fade_bars, bool) or not isinstance(fade_bars, int):
        raise ValueError(f"fade_bars должен быть целым, получено {fade_bars!r}")
    if not FADE_BARS_RANGE[0] <= fade_bars <= FADE_BARS_RANGE[1]:
        raise ValueError(f"fade_bars вне диапазона {FADE_BARS_RANGE[0]}..{FADE_BARS_RANGE[1]}: {fade_bars}")
    try:
        tree = ast.parse(program)
    except SyntaxError as exc:
        raise ValueError(f"переход fade: программа трека не парсится: {exc.msg}") from exc
    _check_indentable(tree)
    beats = fade_bars * BEATS_PER_BAR
    durs = f"[{beats}, {beats}, {beats}]"
    body = []
    names = _top_level_names(tree)
    if names:
        body.append("global " + ", ".join(names))
    body.append(program.rstrip("\n"))
    lines = [
        f"# переход: фейд уходящего трека {fade_bars} тактов (lpf + amplify), "
        "потом новый трек с границы такта",
        f"def {NEXT_TRACK_FN}():",
        textwrap.indent("\n".join(body), "    ", lambda line: bool(line.strip())),
        "",
        "if Clock.playing:",
        f"    Master().lpf = linvar([{FADE_LPF_FROM}, {FADE_LPF_TO}, {FADE_LPF_TO}], {durs}, start=Clock.now())",
        f"    Master().amplify = linvar([1, 0, 0], {durs}, start=Clock.now())",
        f"    Clock.schedule({NEXT_TRACK_FN}, Clock.next_bar() + {beats})",
        "else:",
        f"    {NEXT_TRACK_FN}()",
    ]
    return "\n".join(lines) + "\n"


__all__ = ["FADE_BARS", "FADE_BARS_RANGE", "NEXT_TRACK_FN", "fade_seconds", "wrap_with_fade"]
