"""``render_audio(track, deck) -> Program`` — готовый звук на деке (ADR-0158 §2.4, PR-2).

Сгенерированный трек играет той же декой, что и синтез: программа — один плеер ``loop('<wav>')`` в слоте лупа деки,
владелец плеера ставит её ``play(..., once=True)`` — ``started`` по клоку, ``finished`` на конце файла, стоп и «что
играет» — его же. Громкость — мастер-шина (``masterfilter``): синт ``loop`` пишет в общую шину, как ноты синтеза.

``sus`` — длина файла в долях (``duration_s × bpm / 60``): файл звучит один раз целиком. Форма и ``dur`` — та же длина,
округлённая вверх до целого такта (``BEATS_PER_BAR``), хвост тишины — меньше такта. Целый такт обязателен: владелец
ставит клок на ``k·форма − lead``, Renardo ставит плеер на ``Clock.next_bar()`` (кратно такту, ``Players.py:892``) и
оттуда — на ближайшую долю, кратную сумме ``dur`` (``Player.count(event_after=True)``). Форма 47.3 доли через раз давала
next_bar за границей формы, и трек вставал на форму позже — живой прогон 13.10: песня через ~49 с после заказа.
``dur`` обязан равняться форме: ``dur`` в две формы через раз давал старт на форму позже (PR-2).
Повтор файла на доле ``finished`` (там владелец снимает деку, а бандл ноты уходит с ``latency`` вперёд) глушит
``amp=var([1, 0], [форма, форма])`` от старта трека.
Края огибающей — ``knowledge.AUDIO_EDGE_S`` внутри ``sus`` (патч ``loop.scd``, #3432).

Трек сета из генерации (ADR-0158 §2.7, PR-8) — ``xfade_bars``: кроссфейд с соседями огибающей того же синта. Форма —
длина файла, округлённая ВНИЗ до такта (хвост меньше такта срезан), ``sus`` равен форме: спад уходящего кончается ровно
на границе формы, входящий встаёт за ``xfade_bars`` до неё и нарастает столько же (``SetSession._blend_beats``). Не
последний трек сета (``once=False``) — глушитель с циклом в одну форму: не готов следующий — сет продлевает трек ещё
проходом файла («продлить, а не замолчать», I1), а не тишиной. ``dur`` трека сета — такт, а не форма: стык ставит
плеер на долю ``at_beat`` без ``set_time``, а Renardo ставит плеер на долю, кратную сумме ``dur`` (``Player.count``) —
с ``dur`` = 148 долей трек 3 живого рок-сета 13.10 встал на 296-й доле вместо 224-й (56 с тишины). Ноты остальных
тактов глушит ``amp`` (Renardo не шлёт ноту с amp 0, ``Players.py:1633``).

Стык петли с собой (хвост PR-8, живой замер 14.10: провал до −35.6 dB на ~2 с): раньше нота прохода k гасла к границе
формы, а нота прохода k+1 только с неё нарастала — спад, потом подъём, тишина посередине. Теперь проход не последнего
трека (``once=False``) короче файла на кроссфейд: ``form_beats`` = файл − кроссфейд, нота звучит весь файл, и её спад
идёт ВМЕСТЕ с подъёмом ноты следующего прохода (``Program.tail_beats``). Края — четверть синуса (кривая ``wel`` в патче
``loop.scd``): спад cos и подъём sin дают сумму мощностей 1 — стык без провала. Кроссфейд — целые такты: начало
прохода обязано быть долей ноты (``dur`` = такт). Следующий трек сета встаёт ровно на границу прохода — хвост уходящего
и есть кроссфейд (``SetSession._blend_beats``).

Дека файла (``fdeck``): нота файла звучит всю ``sus`` и после снятия плеера, а результат ожидания (``BridgeHold``)
входит посреди прохода моста-файла. Нота знает свою деку, и ``/n_set 1 fadeT <с> fade<дека> 0`` (по группе 1 — всем
нотам) гасит только её огибающей ``wel`` (:func:`deck_fade_args`), а не всю группу вместе с результатом.
"""

from __future__ import annotations

import math
from typing import Tuple

from .. import knowledge as kn
from ..model import BEATS_PER_BAR, AudioTrack
from .program import Program
from .renardo import FORM_START, ROLE_SLOT, RenderError, _num


def audio_bpm(track: AudioTrack) -> int:
    """Темп деки: номинал трека, иначе ``AUDIO_DEFAULT_BPM`` (доля — секунда)."""
    return int(track.bpm_nominal or kn.AUDIO_DEFAULT_BPM)


#: Дека → номер ``fdeck`` ноты файла (патч ``loop.scd``: ``Select.kr(fdeck, [1, fadeA, fadeB])``; 0 — не файл деки).
DECK_FADE = {"A": 1, "B": 2}


def deck_fade_args(deck: str, seconds: float) -> Tuple[object, ...]:
    """Аргументы ``/n_set 1 …``: ноты файла деки ``deck`` гаснут за ``seconds`` (четверть синуса), ноты другой деки и
    синтеза — нет (у них ``fdeck`` другой или 0). Новые ноты деки после команды звучат полностью."""
    if deck not in DECK_FADE:
        raise RenderError(f"деки {deck!r} нет (есть {sorted(DECK_FADE)})")
    return ("fadeT", round(max(0.01, float(seconds)), 3), f"fade{deck}", 0.0)


def _shape(track: AudioTrack, bpm: int, xfade_bars: int) -> Tuple[float, float, float, float]:
    """``(форма, sus, край, перекрытие)`` — доли, секунды, доли: одиночный трек — форма вверх до такта, края
    ``AUDIO_EDGE_S``; трек сета — форма вниз до такта (не меньше такта), край — кроссфейд целыми тактами, не длиннее
    трети файла; перекрытие проходов — край (файл короче трёх тактов — без перекрытия, край — треть)."""
    sus = round(track.duration_s * bpm / 60.0, 3)
    if not xfade_bars:
        return float(math.ceil(sus / BEATS_PER_BAR) * BEATS_PER_BAR), sus, 0.0, 0.0
    form = float(max(1, math.floor(sus / BEATS_PER_BAR)) * BEATS_PER_BAR)
    overlap = float(min(xfade_bars, int(form // (3 * BEATS_PER_BAR))) * BEATS_PER_BAR)
    return form, form, (overlap or form / 3.0), overlap


def render_audio(track: AudioTrack, deck: str, *, xfade_bars: int = 0, once: bool = True) -> Program:
    """Файл → программа деки ``deck``. Детерминирована: один трек — одна строка. ``xfade_bars`` — трек сета
    (кроссфейд, форма вниз до такта); ``once=False`` — повтор файла не глушится (продление трека сета), проход короче
    файла на кроссфейд: следующий проход нарастает, пока этот гаснет."""
    if deck not in kn.DECK_SLOTS:
        raise RenderError(f"деки {deck!r} нет (есть {sorted(kn.DECK_SLOTS)})")
    slot = kn.DECK_SLOTS[deck][ROLE_SLOT["loop"]]
    bpm = audio_bpm(track)
    form, sus, edge, overlap = _shape(track, bpm, xfade_bars)
    atk, rel = (round(edge * 60.0 / bpm, 3),) * 2 if xfade_bars else kn.AUDIO_EDGE_S
    tail = 0.0 if once else overlap
    period = form - tail  # проход: следующая нота файла — за перекрытие до конца этой
    dur, cycle = (form, 2 * form) if not xfade_bars else (float(BEATS_PER_BAR), 2 * form if once else period)
    amp = f"var([1, 0], [{_num(dur)}, {_num(cycle - dur)}], start={FORM_START})" if once or xfade_bars else "1"
    code = (f"# {track.track_id} {track.source} {track.duration_s:.1f} s deck {deck}\n"
            f"{slot} >> loop({track.path!r}, dur={_num(dur)}, sus={_num(sus)}, atk={_num(atk)}, "
            f"rel={_num(rel)}, amp={amp}, fdeck={DECK_FADE[deck]})\n")
    return Program(code=code, track_id=track.track_id, deck=deck, bpm=bpm, form_beats=float(period),
                   slots={"audio": slot}, synths=frozenset(), samples=frozenset(),
                   sample_files=frozenset({track.path}), tail_beats=tail)


__all__ = ["DECK_FADE", "audio_bpm", "deck_fade_args", "render_audio"]
