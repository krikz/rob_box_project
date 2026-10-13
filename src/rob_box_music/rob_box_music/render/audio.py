"""``render_audio(track, deck) -> Program`` — готовый звук на деке (ADR-0158 §2.4, PR-2).

Сгенерированный трек играет той же декой, что и синтез: программа — один плеер ``loop('<wav>')`` в слоте лупа деки,
владелец плеера ставит её ``play(..., once=True)`` — ``started`` по клоку, ``finished`` на конце файла, стоп и «что
играет» — его же. Громкость — мастер-шина (``masterfilter``): синт ``loop`` пишет в общую шину, как ноты синтеза.

Форма — длина файла в долях (``duration_s × bpm / 60``), ``sus`` — та же длина: файл звучит один раз целиком. ``dur``
плеера — две формы: следующий запуск файла не успевает до ``finished`` (на той же доле владелец снимает деку).
Края огибающей — ``knowledge.AUDIO_EDGE_S`` внутри ``sus`` (патч ``loop.scd``, #3432).
"""

from __future__ import annotations

from .. import knowledge as kn
from ..model import AudioTrack
from .program import Program
from .renardo import ROLE_SLOT, RenderError, _num


def audio_bpm(track: AudioTrack) -> int:
    """Темп деки: номинал трека, иначе ``AUDIO_DEFAULT_BPM`` (доля — секунда)."""
    return int(track.bpm_nominal or kn.AUDIO_DEFAULT_BPM)


def render_audio(track: AudioTrack, deck: str) -> Program:
    """Файл → программа деки ``deck``. Детерминирована: один трек — одна строка."""
    if deck not in kn.DECK_SLOTS:
        raise RenderError(f"деки {deck!r} нет (есть {sorted(kn.DECK_SLOTS)})")
    slot = kn.DECK_SLOTS[deck][ROLE_SLOT["loop"]]
    bpm = audio_bpm(track)
    form = round(track.duration_s * bpm / 60.0, 3)
    atk, rel = kn.AUDIO_EDGE_S
    code = (f"# {track.track_id} {track.source} {track.duration_s:.1f} s deck {deck}\n"
            f"{slot} >> loop({track.path!r}, dur={_num(2 * form)}, sus={_num(form)}, atk={_num(atk)}, "
            f"rel={_num(rel)}, amp=1)\n")
    return Program(code=code, track_id=track.track_id, deck=deck, bpm=bpm, form_beats=float(form),
                   slots={"audio": slot}, synths=frozenset(), samples=frozenset(),
                   sample_files=frozenset({track.path}))


__all__ = ["audio_bpm", "render_audio"]
