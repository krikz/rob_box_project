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

Одиночная пьеса fail-open (ADR-0157 §2.3, PR-2): кандидаты — партитуры произведения по рангу, затем RTTTL-хуки, которые
поиск темы и ``theme_links`` уже подтвердили по каталогу (не сырая фраза человека: «Токката ре минор Баха на
ксилофоне» промахивалась на любом лишнем слове, прогон 09.10). Недавно сыгранное — :class:`PieceMemory` (таблица
``music_history``, ADR-0146), очередь кандидатов — ``engine.piece`` (PR-3), перебор — :func:`pick_piece`.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.arrange.song import SongMaterial, score_song, song_track
from rob_box_music.diversity import track_history
from rob_box_music.instruments import LeadChoice
from rob_box_music.material import ScoreMaterial
from rob_box_music.render.program import Program
from rob_box_music.render.renardo import render
from rob_box_music.works import RTTTL, SCORE

from ..core.rtttl_compose import melody_to_compose_params, rtttl_to_melody
from ..core.rtttl_library import human_track_title, match_info, spoken_title
from .search import find


@dataclass(frozen=True)
class ClassicPick:
    """Итог заказа по имени: ``found=False`` — честный промах поиска (I16), ``program`` — что играть; ``source`` —
    :data:`SCORE`/:data:`RTTTL`, ``history`` — строка ``music_history`` трека (пишется после ``started``)."""

    query: str
    found: bool
    reason: str = ""
    melody_id: Optional[str] = None
    title: Optional[str] = None
    program: Optional[Program] = None
    bpm: Optional[int] = None
    key: Optional[str] = None
    source: str = RTTTL
    history: Mapping[str, Any] = field(default_factory=dict)
    composer: str = ""  # автор для фразы, по-русски (``spoken_title``): ``artist`` RTTTL, ``composer`` партитуры
    track: Any = None  # модель трека (``model.Track``) — в файл каталога моделей после ``started`` (I24)


def pick_piece(candidates: Iterable[Any],
               build: Callable[[Any], ClassicPick]) -> Tuple[Optional[ClassicPick], List[Tuple[Any, str]]]:
    """Первый кандидат, из которого сложилась песня, и отказы предыдущих ``(кандидат, причина)``; не сложился ни
    один — ``None``."""
    failed: List[Tuple[Any, str]] = []
    for candidate in candidates:
        pick = build(candidate)
        if pick.found:
            return pick, failed
        failed.append((candidate, pick.reason))
    return None, failed


class PieceMemory:
    """Что одиночный заказ играл недавно: строки ``music_history`` с ``kit=CLASSIC_HISTORY_KIT`` (ADR-0146 для
    одиночных пьес); БД нет — память процесса. Окно — ``knowledge.HOOK_FRESH_SETS`` последних исполнений."""

    def __init__(self, store: Any = None, depth: int = kn.HOOK_FRESH_SETS) -> None:
        self._store = store
        self._depth = depth
        self._local: List[Tuple[str, float]] = []  # (запись, ts), свежие первыми

    def recent(self) -> List[str]:
        """Записи последних исполнений, свежие первыми."""
        if self._store is not None and getattr(self._store, "available", False):
            rows = self._store.recent(self._depth, kit=kn.CLASSIC_HISTORY_KIT)
            return [str(r["melody_name"]) for r in rows if r.get("melody_name")]
        return [name for name, _ts in self._local[:self._depth]]

    def played(self) -> List[Tuple[str, float]]:
        """``(запись, ts)`` сыгранного за ``PLAY_COUNT_WINDOW_SEC``, свежие первыми — счётчик повторов для модели
        (тот же источник, что :meth:`recent`)."""
        if self._store is not None and getattr(self._store, "available", False):
            rows = self._store.recent(kn.PLAY_HISTORY_LIMIT, within_sec=kn.PLAY_COUNT_WINDOW_SEC,
                                      kit=kn.CLASSIC_HISTORY_KIT)
            return [(str(r["melody_name"]), float(r["ts"])) for r in rows if r.get("melody_name") and r.get("ts")]
        return list(self._local)

    def remember(self, row: Mapping[str, Any]) -> None:
        """Песня заиграла (``started``): строку — в историю."""
        if row.get("melody_name"):
            self._local.insert(0, (str(row["melody_name"]), float(row.get("ts") or time.time())))
        if self._store is not None and row:
            self._store.record(**row)


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


def _lead_kw(lead: Optional[LeadChoice]) -> Dict[str, Any]:
    """Лид инструмента заказа (ADR-0157 §2.4) для ``song_track``/``score_song``; без инструмента — пусто."""
    return {"lead_synth": lead.synth, "lead_shift": lead.octave_shift} if lead else {}


def pick_classic(library: Any, query: str, *, seed: int, deck: str = "A",
                 lead: Optional[LeadChoice] = None) -> ClassicPick:
    """Найти мелодию по словам человека и отрендерить песню; ``ValueError`` — материал не лёг в модель."""
    record, reason = find_record(library, query)
    if record is None:
        return ClassicPick(query, False, reason)
    title = human_track_title(library, record)
    track = song_track(song_material(record, title), seed=seed, deck=deck, **_lead_kw(lead))
    return _picked(query, track, title, deck, RTTTL, str(record.get("artist") or ""))


def _picked(query: str, track: Any, title: str, deck: str, source: str, composer: str = "") -> ClassicPick:
    """Собранная песня; название и автор — как их говорит робот (``spoken_title``, #3577: без мусора PDMX)."""
    title, composer = spoken_title(title, composer)
    return ClassicPick(query, True, melody_id=track.hook.source, title=title, program=render(track, deck),
                       bpm=track.bpm, key=f"{kn.ROOTS[track.key.root]} {track.key.mode}", source=source,
                       history=track_history(track), composer=composer, track=track)


def pick_record(record: Mapping[str, Any], title: str, query: str, *, seed: int, deck: str = "A",
                lead: Optional[LeadChoice] = None) -> ClassicPick:
    """Песня из записи RTTTL; не легла в модель — ``found=False`` с причиной (следующий кандидат)."""
    try:
        track = song_track(song_material(dict(record), title), seed=seed, deck=deck, **_lead_kw(lead))
    except (KeyError, ValueError) as exc:
        return ClassicPick(query, False, f"{record.get('name')}: {type(exc).__name__}: {exc}")
    return _picked(query, track, title, deck, RTTTL, str(record.get("artist") or ""))


def pick_score(materials: Mapping[str, ScoreMaterial], ids: Sequence[str], query: str, *, seed: int,
               deck: str = "A", references: Sequence[str] = (), lead: Optional[LeadChoice] = None,
               match_min: float = kn.CLASSIC_REF_MATCH_MIN) -> ClassicPick:
    """Песня из первого материала ``ids`` (порядок поиска), из которого она складывается (``song.score_song``);
    ни одного — ``found=False`` с причинами по каждому (размер, нет аккордов, нет JSON, нет мотива RTTTL-эталона
    ``references``) — заказ идёт в RTTTL."""
    reasons = []
    for mid in ids:
        try:
            material = materials[mid]
            track = score_song(material, seed=seed, deck=deck, references=references, match_min=match_min,
                               **_lead_kw(lead))
        except (KeyError, ValueError) as exc:  # TrackError и MaterialError — тоже ValueError
            reasons.append(f"{mid}: {type(exc).__name__}: {exc}")
            continue
        return _picked(query, track, material.title, deck, SCORE, material.composer)
    return ClassicPick(query, False, "; ".join(reasons) or "партитур по названию нет")


def classic_picker(library_factory: Callable[[], Any]) -> Callable[..., ClassicPick]:
    """``pick(query, seed=…)`` над библиотекой, которая открывается при первом заказе."""
    box: Dict[str, Any] = {}

    def pick(query: str, *, seed: int, deck: str = "A", lead: Optional[LeadChoice] = None) -> ClassicPick:
        if "lib" not in box:
            box["lib"] = library_factory()
        return pick_classic(box["lib"], query, seed=seed, deck=deck, lead=lead)

    return pick


__all__ = ["ClassicPick", "PieceMemory", "classic_picker", "find_record", "pick_classic", "pick_piece", "pick_record",
           "pick_score", "song_material"]
