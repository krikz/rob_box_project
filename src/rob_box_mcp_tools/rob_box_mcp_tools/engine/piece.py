"""Одиночная пьеса: кандидаты произведения → выбор → перебор fail-open (ADR-0157 §2.3, §3.2, §3.3; PR-2, PR-3).

Кандидаты строит код (``works.WorkRef``): слова заказа делятся на композитора и название (композитор — слово, которое
в поле «композитор» каталога встречается чаще, чем в названиях; без индекса партитур — слово из ``artist`` записей
RTTTL, найденных поиском), тональность — слот. Партитуры — ``ScoreLibrary.work_candidates`` (поле ``composer``,
слово целиком), записи RTTTL — находки поиска темы и ``theme_links``, проверенные по ``artist``: «Mundian To Bach Ke»
(Panjabi MC) при «сыграй Баха» — не кандидат. Выбор — ``choice.DeterministicChooser`` (признаки §3.3, недавнее из
``music_history``); названо произведение — уровни: его партитуры, его RTTTL, затем остальное (ближайшее по
композитору), свежесть — внутри уровня (решение координатора 12.10: единственную годную партитуру «Токкаты» история
не выталкивает в рингтон). Не сложился ни один — поиск по названию целиком, затем ближайшее по словам заказа.
"""

from __future__ import annotations

import logging
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from rob_box_music.choice import ChoiceQuestion, DeterministicChooser, Option, recency
from rob_box_music.instruments import LeadChoice
from rob_box_music.rtttl import CONTOUR_NOTES, contour
from rob_box_music.works import RTTTL, SCORE, WorkCandidate, WorkRef, WorkWord

from .classic import ClassicPick, PieceMemory, pick_piece, pick_record, pick_score
from .score_library import PlanMaterials
from .search import Term, ThemeHits, part_query

_LOG = logging.getLogger(__name__)
#: Записи RTTTL по ``id`` с названием для голоса: ``ids -> {id: (запись, название)}``.
RecordLookup = Callable[[Iterable[str]], Dict[str, Tuple[Dict[str, Any], str]]]
#: Кандидатов в строке лога выбора.
_LOG_CANDIDATES = 8


def query_terms(hits: ThemeHits, work: str) -> Tuple[Term, ...]:
    """Слова заказа с написаниями в архиве — те, по которым искался поиск темы; подставной поиск строк не отдаёт —
    та же ``part_query`` без каталога RTTTL (семена реестра и звучание слова)."""
    parts = hits.query.parts if hits.query is not None else (part_query(None, work),)
    return tuple(t for p in parts for t in p.terms)


def work_ref(terms: Sequence[Term], key: str, scores: Any, records: Mapping[str, Mapping[str, Any]]) -> WorkRef:
    """Слова заказа → ``WorkRef``: композитор — по индексу партитур (``composer_role``), индекс не знает слова —
    по ``artist`` записей RTTTL (слово — имя автора хотя бы одной находки)."""
    composer, title = [], []
    for term in terms:
        word = WorkWord(tuple(a.lower() for a in term.alts), tuple(x for x in term.latin[:2] if x))
        role = scores.composer_role(word)
        if role is None:
            role = any(word.named_in(r.get("artist")) for r in records.values())
        (composer if role else title).append(word)
    return WorkRef(tuple(composer), tuple(title), key)


def tunes(records: Mapping[str, Tuple[Dict[str, Any], str]]) -> Dict[str, Any]:
    """Ключ мелодии записи — контур начала (``rtttl.contour``, как у межсетовой свежести): версии одной мелодии
    («mundiant»/«mundiant_2») — одна мелодия для признака ``recent``."""
    keys = {i: contour(str(r.get("rtttl") or ""), CONTOUR_NOTES) for i, (r, _t) in records.items()}
    return {i: k for i, k in keys.items() if k is not None}


def levels(ref: WorkRef, candidates: Sequence[WorkCandidate]) -> List[List[WorkCandidate]]:
    """Уровни перебора: названо произведение — его партитуры, его RTTTL, остальное; назван только автор — один."""
    if not ref.named:
        return [list(candidates)]
    own = [c for c in candidates if c.title_match]
    return [[c for c in own if c.kind == SCORE], [c for c in own if c.kind == RTTTL],
            [c for c in candidates if not c.title_match]]


class PieceChain:
    """Цепочка одиночной пьесы ``request_music(melody)``: что играть и в каком порядке пробовать."""

    def __init__(self, dj: Any, records: RecordLookup, words: Callable[[str], ThemeHits],
                 classic: Callable[..., ClassicPick], history: Any = None,
                 chooser: Optional[DeterministicChooser] = None, log: Callable[[], Any] = lambda: _LOG) -> None:
        self._dj = dj
        self._records = records
        self._words = words
        self._classic = classic
        self.memory = PieceMemory(history)  # что одиночный заказ играл недавно (music_history, ADR-0146)
        self._chooser = chooser or DeterministicChooser()
        self._log = log

    def play(self, work: str, key: str, seed: int, lead: Optional[LeadChoice]
             ) -> Tuple[Optional[ClassicPick], Dict[str, Any], List[str], int]:
        """Произведение → поиск по названию целиком (``find_record``) → ближайшее по словам заказа.
        ``(песня | None, played, отказы, сколько было кандидатов)``."""
        pick, failed, count, ref, chosen = self._work_pick(work, key, seed, lead)
        replacement = work if pick is not None and ref.named and chosen and not chosen.title_match else None
        if pick is None:
            old = self._classic(work, seed=seed, **({"lead": lead} if lead else {}))
            pick, count = (old, count + 1) if old.found else (None, count)
        if pick is None:
            near, more, extra = self._near_pick(work, key, seed, lead, [c.id for c, _why in failed])
            pick, failed, count, replacement = near, failed + more, count + extra, work if near else None
        reasons = [why for _c, why in failed]
        if pick is None:
            return None, {}, reasons, count
        played = {"work": work, "title": pick.title, "composer": pick.composer, "source": pick.source,
                  "replacement_for": replacement, "scores_failed": sum(c.kind == SCORE for c, _why in failed)}
        return pick, played, reasons, count

    def candidates(self, work: str, key: str = "") -> Tuple[WorkRef, List[WorkCandidate], Dict[str, Any]]:
        """``(WorkRef, кандидаты: партитуры и RTTTL, записи RTTTL)`` по словам произведения — без звука (их же
        читает офлайн-оценка ``scripts/research/jev_eval.py --domain music``)."""
        hits = self._dj.work_hits(work)
        records = self._records(hits.names)
        plain = {i: r for i, (r, _t) in records.items()}
        ref = work_ref(query_terms(hits, work), key, self._dj._scores, plain)
        return ref, [*self._dj._scores.work_candidates(ref), *ref.rtttl(plain)], records

    def _work_pick(self, work: str, key: str, seed: int, lead: Optional[LeadChoice]):
        ref, candidates, records = self.candidates(work, key)
        refs = [records[c.id][0]["rtttl"] for c in candidates if c.kind == RTTTL and c.title_match]  # эталоны
        materials = PlanMaterials(self._dj._scores, [c.id for c in candidates if c.kind == SCORE], self._log())
        build = self._builder(records, materials, refs, work, seed, lead)
        order = self._order(work, ref, candidates, records)
        pick, failed = pick_piece(order, build)
        chosen = next((c for c in order if pick is not None and c.id == pick.melody_id), None)
        return pick, failed, len(candidates), ref, chosen

    def _near_pick(self, work: str, key: str, seed: int, lead: Optional[LeadChoice], tried: Iterable[str]):
        """Ближайшее по словам заказа (композитор: «Токката Баха» → мелодии «Баха»): каждое слово — прямым поиском,
        без LLM, кандидаты — по композитору, если он назван; уже перебранное не повторяется."""
        records: Dict[str, Tuple[Dict[str, Any], str]] = {}
        terms: List[Term] = []
        for word in dict.fromkeys(w for w in work.split() if len(w) >= 4):
            hits = self._words(word)
            records.update(self._records([h for h in hits.names if h not in tried and h not in records]))
            terms += query_terms(hits, word)
        plain = {i: r for i, (r, _t) in records.items()}
        ref = WorkRef(work_ref(terms, key, self._dj._scores, plain).composer)
        candidates = ref.rtttl(plain)
        order = self._order(work, ref, candidates, records)
        pick, failed = pick_piece(order, self._builder(records, {}, [], work, seed, lead))
        return pick, failed, len(candidates)

    def _builder(self, records: Mapping[str, Tuple[Dict[str, Any], str]], materials: Any, refs: Sequence[str],
                 work: str, seed: int, lead: Optional[LeadChoice]) -> Callable[[WorkCandidate], ClassicPick]:
        def build(c: WorkCandidate) -> ClassicPick:
            if c.kind == RTTTL:
                return pick_record(records[c.id][0], records[c.id][1], work, seed=seed, lead=lead)
            return pick_score(materials, [c.id], work, seed=seed, references=refs, lead=lead)
        return build

    def questions(self, work: str, ref: WorkRef, candidates: Sequence[WorkCandidate],
                  records: Mapping[str, Tuple[Dict[str, Any], str]]) -> List[ChoiceQuestion]:
        """Вопросы выбора по уровням (:func:`levels`): признаки кандидатов + ``recent`` из истории."""
        recent, same = self.memory.recent(), tunes(records)
        state = {"composer": [w.spellings for w in ref.composer], "title": [w.spellings for w in ref.title],
                 "key": ref.key, "recent": list(recent)}
        return [ChoiceQuestion(work, tuple(Option(c.id, {**c.features(), "recent": recency(c.id, recent, same)},
                                                  f"{c.kind} «{c.title}» {c.composer}".strip()) for c in level), state)
                for level in levels(ref, candidates) if level]

    def _order(self, work: str, ref: WorkRef, candidates: Sequence[WorkCandidate],
               records: Mapping[str, Tuple[Dict[str, Any], str]]) -> List[WorkCandidate]:
        """Очередь перебора: по уровням, внутри уровня — ``Chooser``; выбор и признаки — в лог."""
        by_id = {c.id: c for c in candidates}
        order: List[WorkCandidate] = []
        for question in self.questions(work, ref, candidates, records):
            choice, ids = self._chooser.order(question)
            if not order:
                self._log_choice(work, ref, question, choice)
            order += [by_id[i] for i in ids]
        return order

    def _log_choice(self, work: str, ref: WorkRef, question: ChoiceQuestion, choice: Any) -> None:
        ranked = self._chooser.rank(question)[:_LOG_CANDIDATES]
        self._log().info(
            f"🎼 [classic] выбор «{work}»: композитор={[w.spellings[:2] for w in ref.composer]} "
            f"название={[w.spellings[:2] for w in ref.title]} тональность={ref.key or '—'} → {choice.id} "
            f"({choice.provider}, уверенность {choice.confidence:.2f}); кандидаты: "
            + "; ".join(f"{s:.2f} {o.id} {o.label}" for s, o in ranked))


__all__ = ["PieceChain", "RecordLookup", "levels", "query_terms", "tunes", "work_ref"]
