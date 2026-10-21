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

Трек сета из генерации (ADR-0158 §2.7, PR-8) — ``xfade_s``: кроссфейд с соседями огибающей того же синта, в СЕКУНДАХ
(``GEN_XFADE_S``, В15: у кэша темп часто неизвестен, 4 такта по 60 bpm = 16 с внахлёст). Файл звучит весь (``sus`` =
длина файла), край огибающей — кроссфейд: спад уходящего кончается ровно на конце файла, входящий нарастает столько же.
Плеер встаёт только на такт, а кроссфейд — секунды, поэтому проход считается целыми тактами ВНИЗ (``form_beats``), а
остаток — ``carry_beats`` — следующий трек сдвигает ``delay``: его звук встаёт ровно за кроссфейд до конца уходящего,
округление тактом хвост не съедает (``SetSession._entry_lead``). Не последний трек сета (``once=False``) — глушитель с
циклом в один проход: не готов следующий — сет продлевает трек ещё проходом файла («продлить, а не замолчать», I1), а не
тишиной; нота знает свой хвост за границей прохода (``Program.tail_beats``) — следующий встаёт на границу, хвост и есть
кроссфейд. Последний трек — один проход файла целиком. ``dur`` трека сета — такт, а не форма: стык ставит плеер на долю
``at_beat`` без ``set_time``, а Renardo ставит плеер на долю, кратную сумме ``dur`` (``Player.count``) — с ``dur`` = 148
долей трек 3 живого рок-сета 13.10 встал на 296-й доле вместо 224-й (56 с тишины). Ноты остальных тактов глушит ``amp``
(Renardo не шлёт ноту с amp 0, ``Players.py:1633``). Края — четверть синуса (кривая ``wel`` в патче ``loop.scd``):
спад cos и подъём sin дают сумму мощностей 1 — стык без провала.

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


def xfade_beats(bpm: int, duration_s: float = 0.0) -> float:
    """Кроссфейд ``GEN_XFADE_S`` в долях темпа ``bpm``; файл короче трёх кроссфейдов — треть файла."""
    seconds = min(kn.GEN_XFADE_S, duration_s / 3.0) if duration_s else kn.GEN_XFADE_S
    return seconds * bpm / 60.0


def entry_blend(bpm: int) -> Tuple[float, float]:
    """``(доли блэнда, сдвиг входа)`` файла поверх конца формы синтеза: плеер встаёт на такт за блэнд до границы,
    а звук — ``delay`` позже, ровно за кроссфейд до неё."""
    blend = max(1, math.ceil(xfade_beats(bpm) / BEATS_PER_BAR)) * float(BEATS_PER_BAR)
    return blend, blend - xfade_beats(bpm)


def set_track_seconds(duration_s: float, last: bool) -> float:
    """Сколько секунд сета занимает файл ``duration_s``: целиком, у не последнего за вычетом кроссфейда (следующий
    встаёт за него до конца файла). Темп не важен — остаток такта уходит в ``carry_beats``. Одно место с
    :func:`render_audio` — по нему считается длина сета «на N минут» (``source.route_seconds``)."""
    return duration_s if last else duration_s - min(kn.GEN_XFADE_S, duration_s / 3.0)


def _pass(sus: float, bpm: int, duration_s: float, lead: float, once: bool) -> Tuple[float, float, float]:
    """``(проход, хвост, остаток)`` трека сета, доли: проход — целые такты вниз от (сдвиг + файл − кроссфейд); хвост —
    сколько нота звучит за границей прохода (он же кроссфейд плюс остаток); остаток — сдвиг входа следующего."""
    total = lead + sus
    if once:
        return float(math.ceil(total / BEATS_PER_BAR) * BEATS_PER_BAR), 0.0, 0.0
    rest = max(0.0, total - xfade_beats(bpm, duration_s))
    period = float(max(1, math.floor(rest / BEATS_PER_BAR)) * BEATS_PER_BAR)
    return period, max(0.0, total - period), max(0.0, rest - period)


def render_audio(track: AudioTrack, deck: str, *, xfade_s: float = 0.0, once: bool = True,
                 lead_beats: float = 0.0) -> Program:
    """Файл → программа деки ``deck``. Детерминирована: один трек — одна строка. ``xfade_s`` — трек сета (кроссфейд
    секундами, проход целыми тактами); ``once=False`` — повтор файла не глушится (продление трека сета);
    ``lead_beats`` — сдвиг звука внутри такта (``delay``): остаток предыдущего трека или блэнда (:func:`entry_blend`)."""
    if deck not in kn.DECK_SLOTS:
        raise RenderError(f"деки {deck!r} нет (есть {sorted(kn.DECK_SLOTS)})")
    slot = kn.DECK_SLOTS[deck][ROLE_SLOT["loop"]]
    bpm = audio_bpm(track)
    sus = round(track.duration_s * bpm / 60.0, 3)
    if xfade_s:
        lead = min(max(float(lead_beats), 0.0), BEATS_PER_BAR - 0.001)
        period, tail, carry = _pass(sus, bpm, track.duration_s, lead, once)
        fade = round(min(xfade_s, track.duration_s / 3.0), 3)
        atk, rel = fade, (kn.AUDIO_EDGE_S[1] if once else fade)
        dur, cycle = float(BEATS_PER_BAR), (2 * period if once else period)
    else:
        lead, tail, carry = 0.0, 0.0, 0.0
        period = float(math.ceil(sus / BEATS_PER_BAR) * BEATS_PER_BAR)
        atk, rel = kn.AUDIO_EDGE_S
        dur, cycle = period, 2 * period
    level = _num(track.gain)
    amp = (f"var([{level}, 0], [{_num(dur)}, {_num(cycle - dur)}], start={FORM_START})"
           if (once or xfade_s) and cycle > dur else level)
    delay = f"delay={_num(round(lead, 3))}, " if lead else ""
    code = (f"# {track.track_id} {track.source} {track.duration_s:.1f} s deck {deck}\n"
            f"{slot} >> loop({track.path!r}, dur={_num(dur)}, sus={_num(sus)}, {delay}atk={_num(atk)}, "
            f"rel={_num(rel)}, amp={amp}, fdeck={DECK_FADE[deck]})\n")
    return Program(code=code, track_id=track.track_id, deck=deck, bpm=bpm, form_beats=period,
                   slots={"audio": slot}, synths=frozenset(), samples=frozenset(),
                   sample_files=frozenset({track.path}), tail_beats=tail, carry_beats=carry)


__all__ = ["DECK_FADE", "audio_bpm", "deck_fade_args", "entry_blend", "render_audio", "set_track_seconds", "xfade_beats"]
