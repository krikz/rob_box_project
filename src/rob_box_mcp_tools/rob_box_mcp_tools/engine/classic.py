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
ксилофоне» промахивалась на любом лишнем слове, прогон 09.10). Недавно сыгранное (:class:`PieceMemory`, таблица
``music_history``, ADR-0146) уходит в конец очереди (:func:`fresh_first`), перебор — :func:`pick_piece`.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.arrange.song import SongMaterial, score_song, song_track
from rob_box_music.diversity import track_history
from rob_box_music.instruments import LeadChoice
from rob_box_music.material import ScoreMaterial
from rob_box_music.render.program import Program
from rob_box_music.render.renardo import render

from ..core.rtttl_compose import melody_to_compose_params, rtttl_to_melody
from ..core.rtttl_library import human_track_title, match_info
from .search import _in, find

#: Откуда материал пьесы (поле ``played.source`` результата ``request_music``).
SCORE = "score"
RTTTL = "rtttl"


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
    composer: str = ""  # автор из каталога (``artist`` записи RTTTL, ``composer`` партитуры) — для фразы


@dataclass(frozen=True)
class Candidate:
    """Кандидат одиночной пьесы: материал партитуры (``kind=SCORE``) или запись RTTTL (``kind=RTTTL``)."""

    kind: str
    ref: str


def key_slot(text: str) -> Tuple[str, str]:
    """``(слова без тональности, "D minor")`` — «токкату ре минор» → ``("токкату", "D minor")``; тональности нет —
    ``(text, "")``. Слова — из таблиц ``knowledge.KEY_*`` (нота, знак, основа лада)."""
    words = re.findall(r"\w+", text.lower().replace("ё", "е"))
    for i, word in enumerate(words):
        if word not in kn.KEY_NOTE_WORDS:
            continue
        j = i + 1
        sign = kn.KEY_SIGN_WORDS.get(words[j], 0) if j < len(words) else 0
        j += bool(sign)
        mode = next((m for stem, m in kn.KEY_MODE_STEMS.items() if j < len(words) and words[j].startswith(stem)), "")
        if mode:
            root = kn.ROOTS[(kn.KEY_NOTE_WORDS[word] + sign) % 12]
            return " ".join(words[:i] + words[j + 1:]), f"{root} {mode}"
    return text, ""


def person_parasites(records: Mapping[str, Mapping[str, Any]], spellings: Iterable[str]) -> Set[str]:
    """Записи-паразиты имени человека (ADR-0157 §1.2 п.4, R3): слово заказа — имя автора (оно стоит в поле
    ``artist`` хотя бы одной записи-кандидата: «Bach» у «Brandenburg Concert»), а у записи оно совпало только в
    названии при другом исполнителе — «Mundian To Bach Ke» (Panjabi MC). Слово названия («toccata») именем не
    считается: в ``artist`` его нет ни у кого. Полное поле «композитор» реестра — PR-3."""
    persons = person_words(records, spellings)
    return {i for i, r in records.items()
            if any(_in(s, _field(r, "title")) or _in(s, i.lower()) for s in persons)
            and not any(_in(s, _field(r, "artist")) for s in persons)}


def _field(record: Mapping[str, Any], name: str) -> str:
    return str(record.get(name) or "").lower()


def person_words(records: Mapping[str, Mapping[str, Any]], spellings: Iterable[str]) -> Set[str]:
    """Написания слов заказа, которые — имя автора: стоят в ``artist`` хотя бы одной записи-кандидата."""
    return {s for s in spellings if s and any(_in(s, _field(r, "artist")) for r in records.values())}


def piece_order(candidates: Sequence[Candidate], recent: Sequence[str], tune: Optional[Mapping[str, Any]] = None,
                *, work_named: bool = True) -> List[Candidate]:
    """Очередь кандидатов пьесы. Произведение названо («Токката») — история не понижает материал уровнем: партитуры
    всегда раньше RTTTL, свежесть — только внутри уровня (решение координатора 12.10: единственную годную партитуру
    «Токкаты» история вытолкнула в рингтон). Назван только автор («сыграй Баха») — свежесть между произведениями
    по всей очереди (R4)."""
    if not work_named:
        return fresh_first(candidates, recent, tune)
    return [c for kind in (SCORE, RTTTL) for c in fresh_first([x for x in candidates if x.kind == kind], recent, tune)]


def fresh_first(candidates: Sequence[Candidate], recent: Sequence[str],
                tune: Optional[Mapping[str, Any]] = None) -> List[Candidate]:
    """Очередь кандидатов: не звучавшие в ``recent`` (свежие первыми) — в своём порядке, затем звучавшие — сначала
    самые давние (ADR-0157 R4: три «сыграй Баха» подряд — три разные записи, четвёртый — снова первая). ``tune`` —
    ключ мелодии записи (``rtttl.contour``, как у межсетовой свежести): версия сыгранной мелодии («mundiant» после
    «mundiant_2», прогон 12.10) — тоже сыграна."""
    keys = tune or {}
    order = list(recent)

    def age(c: Candidate) -> Optional[int]:
        key = keys.get(c.ref)
        hits = [i for i, r in enumerate(order) if r == c.ref or (key is not None and keys.get(r) == key)]
        return min(hits) if hits else None

    ages = {c: age(c) for c in candidates}
    fresh = [c for c in candidates if ages[c] is None]
    stale = sorted((c for c in candidates if ages[c] is not None), key=lambda c: -ages[c])
    return fresh + stale


def pick_piece(candidates: Iterable[Candidate],
               build: Callable[[Candidate], ClassicPick]) -> Tuple[Optional[ClassicPick], List[Tuple[Candidate, str]]]:
    """Первый кандидат, из которого сложилась песня, и отказы предыдущих ``(кандидат, причина)``; не сложился ни
    один — ``None``."""
    failed: List[Tuple[Candidate, str]] = []
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
        self._local: List[str] = []

    def recent(self) -> List[str]:
        """Записи последних исполнений, свежие первыми."""
        if self._store is not None and getattr(self._store, "available", False):
            rows = self._store.recent(self._depth, kit=kn.CLASSIC_HISTORY_KIT)
            return [str(r["melody_name"]) for r in rows if r.get("melody_name")]
        return self._local[:self._depth]

    def remember(self, row: Mapping[str, Any]) -> None:
        """Песня заиграла (``started``): строку — в историю."""
        if row.get("melody_name"):
            self._local.insert(0, str(row["melody_name"]))
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
    return ClassicPick(query, True, melody_id=track.hook.source, title=title, program=render(track, deck),
                       bpm=track.bpm, key=f"{kn.ROOTS[track.key.root]} {track.key.mode}", source=source,
                       history=track_history(track), composer=composer)


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


__all__ = ["RTTTL", "SCORE", "Candidate", "ClassicPick", "PieceMemory", "classic_picker", "find_record", "fresh_first",
           "key_slot", "person_parasites", "person_words", "piece_order", "pick_classic", "pick_piece", "pick_record",
           "pick_score", "song_material"]
