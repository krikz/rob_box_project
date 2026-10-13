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
последний трек сета (``once=False``) играет без ``var``-глушителя: не готов следующий — сет продлевает трек ещё
проходом файла («продлить, а не замолчать», I1), а не тишиной.
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


def _shape(track: AudioTrack, bpm: int, xfade_bars: int) -> Tuple[float, float, float, float]:
    """``(форма, sus, атака, спад)`` — доли и секунды: одиночный трек — форма вверх до такта, края ``AUDIO_EDGE_S``;
    трек сета — форма вниз до такта (не меньше такта), края — кроссфейд, не длиннее трети файла."""
    sus = round(track.duration_s * bpm / 60.0, 3)
    if not xfade_bars:
        atk, rel = kn.AUDIO_EDGE_S
        return float(math.ceil(sus / BEATS_PER_BAR) * BEATS_PER_BAR), sus, atk, rel
    form = float(max(1, math.floor(sus / BEATS_PER_BAR)) * BEATS_PER_BAR)
    edge = round(min(xfade_bars * BEATS_PER_BAR, form / 3.0) * 60.0 / bpm, 3)
    return form, form, edge, edge


def render_audio(track: AudioTrack, deck: str, *, xfade_bars: int = 0, once: bool = True) -> Program:
    """Файл → программа деки ``deck``. Детерминирована: один трек — одна строка. ``xfade_bars`` — трек сета
    (кроссфейд, форма вниз до такта); ``once=False`` — повтор файла не глушится (продление трека сета)."""
    if deck not in kn.DECK_SLOTS:
        raise RenderError(f"деки {deck!r} нет (есть {sorted(kn.DECK_SLOTS)})")
    slot = kn.DECK_SLOTS[deck][ROLE_SLOT["loop"]]
    bpm = audio_bpm(track)
    form, sus, atk, rel = _shape(track, bpm, xfade_bars)
    amp = f"var([1, 0], [{_num(form)}, {_num(form)}], start={FORM_START})" if once else "1"
    code = (f"# {track.track_id} {track.source} {track.duration_s:.1f} s deck {deck}\n"
            f"{slot} >> loop({track.path!r}, dur={_num(form)}, sus={_num(sus)}, atk={_num(atk)}, "
            f"rel={_num(rel)}, amp={amp})\n")
    return Program(code=code, track_id=track.track_id, deck=deck, bpm=bpm, form_beats=float(form),
                   slots={"audio": slot}, synths=frozenset(), samples=frozenset(),
                   sample_files=frozenset({track.path}))


__all__ = ["audio_bpm", "render_audio"]
