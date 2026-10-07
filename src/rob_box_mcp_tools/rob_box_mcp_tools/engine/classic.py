"""Classic v2 — «поставь Калинку» на движке v2 (ADR-0149 §3.3, §9 PR-11; решение Шифу В5).

Поиск, разбор и гармонизация — СТАРЫЕ библиотеки, вызываются как есть (ADR-0149 §8.1: 10+ вшитых фиксов не
переписывать): ``engine.search.find`` (``RtttlLibrary.get`` как есть, затем слова запроса по основам) +
``match_info`` (поиск по имени, RU-алиасы, транслит ``translit_ru``), решение «это та самая мелодия» —
``named_play.melody_hit`` (одно правило с роутером v1: совпали все значимые слова, ни одно не осталось вне поиска), ``rtttl_compose.melody_to_compose_params`` (темп из RTTTL, затакт,
регистр, контур, тональность) → ``harmonize`` (бас, пэд, рисунки ударных). Отсюда — только перекладка
``Harmonization`` в :class:`rob_box_music.arrange.song.SongMaterial`; трек собирает ``song_track``, звук —
``render`` той же деки, что у club.

Произведение есть в индексе партитур (ADR-0154 PR-7B, §3.5: точное название в индексе партитур > RTTTL) — песня из
материала (:func:`pick_score`, ``song.score_song``): куплеты = секции материала, аккомпанемент — аккорды автора, а не
``harmonize``; ни один материал не сложился — RTTTL-песня, как раньше.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

from rob_box_music import knowledge as kn
from rob_box_music.arrange.song import SongMaterial, score_song, song_track
from rob_box_music.material import ScoreMaterial
from rob_box_music.render.program import Program
from rob_box_music.render.renardo import render

from ..core.rtttl_compose import melody_to_compose_params, rtttl_to_melody
from ..core.rtttl_library import human_track_title, match_info
from .search import find


@dataclass(frozen=True)
class ClassicPick:
    """Итог заказа по имени: ``found=False`` — честный промах поиска (I16), ``program`` — что играть."""

    query: str
    found: bool
    reason: str = ""
    melody_id: Optional[str] = None
    title: Optional[str] = None
    program: Optional[Program] = None
    bpm: Optional[int] = None
    key: Optional[str] = None


def find_record(library: Any, query: str) -> tuple:
    """``(запись, "")`` — нашлась ровно та мелодия, иначе ``(None, причина)``. Поиск — ``engine.search.find``
    (падежи, транслит, служебные слова), правило «та самая» — ``named_play.melody_hit`` по строке, которая
    запись нашла."""
    from rob_box_voice.core.named_play import melody_hit

    found = find(library, query)
    if not found.found:
        return None, "lookup: не найдена"
    record = found.record
    hit, reason = melody_hit({"name": record.get("name"), "title": record.get("title"),
                              "match": match_info(library, record, found.query)})
    return (record, "") if hit is not None else (None, reason)


def song_material(record: Dict[str, Any], title: str) -> SongMaterial:
    """RTTTL записи → ``melody_to_compose_params`` → ``harmonize`` → материал песни (ноты не трогаются)."""
    params = melody_to_compose_params(rtttl_to_melody(record["rtttl"]))
    harmony = params["harmony"]
    return SongMaterial(
        melody_id=str(record.get("name") or ""), title=title, bpm=int(params["bpm"]),
        root=kn.ROOTS.index(str(params["root"])), mode=str(params["scale"]),
        lead=harmony.lead, bass=harmony.bass, pad=harmony.pad, pad_sus=harmony.pad_sus,
        drums=harmony.drums, hats=harmony.hats,
    )


def pick_classic(library: Any, query: str, *, seed: int, deck: str = "A") -> ClassicPick:
    """Найти мелодию по словам человека и отрендерить песню; ``ValueError`` — материал не лёг в модель."""
    record, reason = find_record(library, query)
    if record is None:
        return ClassicPick(query, False, reason)
    title = human_track_title(library, record)
    track = song_track(song_material(record, title), seed=seed, deck=deck)
    return ClassicPick(query, True, melody_id=track.hook.source, title=title, program=render(track, deck),
                       bpm=track.bpm, key=f"{kn.ROOTS[track.key.root]} {track.key.mode}")


def pick_score(materials: Mapping[str, ScoreMaterial], ids: Sequence[str], query: str, *, seed: int,
               deck: str = "A", references: Sequence[str] = ()) -> ClassicPick:
    """Песня из первого материала ``ids`` (порядок поиска), из которого она складывается (``song.score_song``);
    ни одного — ``found=False`` с причинами по каждому (размер, нет аккордов, нет JSON, нет мотива RTTTL-эталона
    ``references``) — заказ идёт в RTTTL."""
    reasons = []
    for mid in ids:
        try:
            material = materials[mid]
            track = score_song(material, seed=seed, deck=deck, references=references)
        except (KeyError, ValueError) as exc:  # TrackError и MaterialError — тоже ValueError
            reasons.append(f"{mid}: {type(exc).__name__}: {exc}")
            continue
        return ClassicPick(query, True, melody_id=mid, title=material.title, program=render(track, deck),
                           bpm=track.bpm, key=f"{kn.ROOTS[track.key.root]} {track.key.mode}")
    return ClassicPick(query, False, "; ".join(reasons) or "партитур по названию нет")


def classic_picker(library_factory: Callable[[], Any]) -> Callable[..., ClassicPick]:
    """``pick(query, seed=…)`` над библиотекой, которая открывается при первом заказе."""
    box: Dict[str, Any] = {}

    def pick(query: str, *, seed: int, deck: str = "A") -> ClassicPick:
        if "lib" not in box:
            box["lib"] = library_factory()
        return pick_classic(box["lib"], query, seed=seed, deck=deck)

    return pick


__all__ = ["ClassicPick", "classic_picker", "find_record", "pick_classic", "pick_score", "song_material"]
