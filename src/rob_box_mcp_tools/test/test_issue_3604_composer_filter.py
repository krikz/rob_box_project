"""#3604 — «валькирии Вагнера»: названный композитор жёстко ограничивает кандидатов одиночной пьесы.

Решение Шифу 13.10: «валькирии» без автора — какая угодно (штраф за повтор работает); «валькирии Вагнера» — только из
Вагнера, даже если он недавно играл. Кандидаты не того композитора отсекает код ДО выбора, а не очки.
"""

from __future__ import annotations

from dataclasses import replace

from rob_box_mcp_tools.engine.classic import ClassicPick
from rob_box_mcp_tools.engine.search import PartQuery, Term, ThemeHits, ThemeQuery
from rob_box_mcp_tools.engine.tools_v2 import NOT_FOUND
from rob_box_music import knowledge as kn
from rob_box_music.diversity import MusicHistory
from rob_box_music.works import SCORE, WorkCandidate
from rob_box_voice.core.media_phrases import composer_missing_error, composer_missing_text

from .test_engine_session import _rig
from .test_piece_fail_open import Scores, _material, _tools
from melodies import CHROMATIC, LONG, SHORT  # noqa: E402  (путь добавлен test_piece_fail_open)

VALKYRIES = {"rideofth": ("Ride Of The Valkyries", LONG, "Wagner"),
             "valkyrie": ("Valkyrie", SHORT, "In Development"),
             "valkyr_2": ("Valkyrie Theme", CHROMATIC, "Somebody Else")}
HITS = ["rideofth", "valkyrie", "valkyr_2"]


def _both(names, composer=True):
    """Находки поиска по словам «валькирии» и (если названо) «вагнера» — с написаниями в архиве, как у theme_search."""
    parts = [PartQuery("валькирии", (Term("валькирии", ("valkyrie", "valkyries")),))]
    if composer:
        parts.append(PartQuery("вагнера", (Term("вагнера", ("wagner",)),)))
    return ThemeHits(tuple(names), query=ThemeQuery("валькирии вагнера", tuple(parts)))


def _ask(text, history=None, scores=None, table=None, classic=None):
    req = _tools(_rig(), hits=_both(HITS, "вагнер" in text), table=table or VALKYRIES,
                 history=history, scores=scores, classic=classic)
    return req.execute(intent="melody", text=text)


def _recent(history, *names):
    for name in names:
        history.record(kit=kn.CLASSIC_HISTORY_KIT, melody_name=name)


def test_named_composer_keeps_only_his_candidates_even_when_his_piece_just_played():
    """«валькирии Вагнера»: недавно играл `rideofth`, а чужой `valkyrie` свеж — играет всё равно Вагнер."""
    history = MusicHistory(":memory:")
    _recent(history, "rideofth", "rideofth")
    data = _ask("сыграй валькирии вагнера", history=history).data
    assert data["ok"] and data["melody_id"] == "rideofth"
    assert data["played"]["composer"] == "Вагнер"


def test_without_composer_the_repeat_penalty_still_moves_the_pick_away():
    """«валькирии» без автора — поведение прежнее: недавно сыгранный `rideofth` уступает свежим."""
    history = MusicHistory(":memory:")
    _recent(history, "rideofth", "rideofth")
    data = _ask("сыграй валькирии", history=history).data
    assert data["ok"] and data["melody_id"] != "rideofth"


def test_without_composer_all_three_stay_candidates():
    req = _tools(_rig(), hits=_both(HITS, False), table=VALKYRIES)
    ref, candidates, _records = req._pieces.candidates("валькирии")
    assert not ref.composer and {c.id for c in candidates} == set(HITS)


def test_named_composer_candidates_are_only_his():
    req = _tools(_rig(), hits=_both(HITS), table=VALKYRIES)
    ref, candidates, _records = req._pieces.candidates("валькирии вагнера")
    assert ref.composer and [c.id for c in candidates] == ["rideofth"]


class LeakyScores(Scores):
    """Индекс, который отдаёт партитуру без совпадения композитора (как если бы поле ``composer`` не проверили)."""

    def work_candidates(self, ref, limit=8, hooks=()):
        return [replace(c, composer_match=0.0, kind=SCORE) for c in super().work_candidates(ref, limit, hooks)]


def test_a_score_without_the_composer_match_never_reaches_the_chooser():
    scores = LeakyScores({"pdmx:x": replace(_material("pdmx:x"), title="Ride of the Valkyries", composer="Nobody")})
    req = _tools(_rig(), hits=_both(HITS), table=VALKYRIES, scores=scores)
    _ref, candidates, _records = req._pieces.candidates("валькирии вагнера")
    assert [c.id for c in candidates] == ["rideofth"]
    assert isinstance(candidates[0], WorkCandidate)


def test_whole_name_search_does_not_replace_a_named_composer_when_his_piece_fails():
    """`rideofth` не складывается; поиск по названию целиком композитора не знает и подсунул бы чужую «Valkyrie»."""
    asked = []

    def classic(query, *, seed, deck="A", lead=None):
        asked.append(query)
        return ClassicPick(query, True, melody_id="valkyrie", title="Valkyrie", composer="In Development")

    table = {**VALKYRIES, "rideofth": ("Ride Of The Valkyries", "ride:d=4,o=5,b=100:", "Wagner")}
    result = _ask("сыграй валькирии вагнера", table=table, classic=classic)
    assert asked == []
    assert result.data.get("melody_id") != "valkyrie"


def test_whole_name_search_still_runs_without_a_composer():
    asked = []

    def classic(query, *, seed, deck="A", lead=None):
        asked.append(query)
        return ClassicPick(query, False, "lookup: не найдена")

    _tools(_rig(), classic=classic).execute(intent="melody", text="поставь калинку")
    assert asked == ["калинку"]


class WagnerIndex(Scores):
    """Индекс партитур знает «Вагнера» как композитора, но его партитур по запросу нет."""

    def work_candidates(self, ref, limit=8, hooks=()):
        return []

    def composer_role(self, word):
        return "wagner" in word.spellings or None


def test_composer_without_any_candidate_is_a_code_phrase_not_a_foreign_piece():
    """Автор назван, его пьес нет: `not_found` с фразой кода, а чужая запись из находок не играет."""
    rig = _rig()
    req = _tools(rig, hits=_both(["valkyrie"]), scores=WagnerIndex(), table={"valkyrie": VALKYRIES["valkyrie"]})
    result = req.execute(intent="melody", text="сыграй валькирии вагнера")
    assert result.success is False and result.data["reason"] == NOT_FOUND
    assert result.data["message"] == composer_missing_text("вагнера")
    assert result.error == composer_missing_error(composer_missing_text("вагнера"))  # роутер скажет сам
    assert rig.events == []


def test_no_composer_and_no_candidates_keeps_the_plain_not_found():
    result = _tools(_rig()).execute(intent="melody", text="сыграй абвгдейку")
    assert result.data["reason"] == NOT_FOUND and "message" not in result.data
