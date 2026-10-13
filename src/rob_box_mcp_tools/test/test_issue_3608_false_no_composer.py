"""#3608 — «давай вагнера полет волкирий ебанем»: ложное «у автора ничего не нашлось» при 10 пьесах Вагнера.

Слово заказа, которое по звуку совпало с чьей-то фамилией («ебанем»), попадало в композиторы вместе с «вагнера»:
партитур с обоими нет, RTTTL с обоими в ``artist`` нет, кандидатов 0, ``no_composer``. Теперь композитор — слова,
которые вместе называют одного автора; остальные уходят в название. ``no_composer`` — только когда у автора 0 пьес.
Одиночная пьеса не ждёт LLM связей темы (6.3 с до звука).
"""

from __future__ import annotations

from rob_box_mcp_tools.engine.search import PartQuery, Term, ThemeHits, ThemeQuery
from rob_box_mcp_tools.engine.theme_links import ThemeLinks
from rob_box_mcp_tools.engine.tools_v2 import NOT_FOUND
from rob_box_voice.core.media_phrases import composer_missing_text

from .test_engine_session import _rig
from .test_issue_3604_composer_filter import VALKYRIES, _ask
from .test_piece_fail_open import Scores, _tools
from melodies import SHORT  # noqa: E402  (путь добавлен test_piece_fail_open)

TABLE = {**VALKYRIES, "treulich": ("Treulich Gefuhrt", SHORT, "Wagner")}
WORK = "вагнера полет волкирий ебанем"


class SoundAlikeIndex(Scores):
    """Индекс, для которого «ebanem» звучит как фамилия (роль композитора), но общих партитур с Вагнером нет."""

    def composer_role(self, word):
        return True if {"wagner", "ebanem"} & set(word.spellings) else None

    def composer_core(self, words):
        return tuple(w for w in words if "wagner" in w.spellings)


class NoPieces(SoundAlikeIndex):
    """Автор в индексе есть, а пьес по запросу нет."""

    def work_candidates(self, ref, limit=8, hooks=()):
        return []


def _hits(*names):
    parts = (PartQuery("вагнера", (Term("вагнера", ("wagner",)),)),
             PartQuery("полет", (Term("полет", ("flight",)),)),
             PartQuery("волкирий", (Term("волкирий", ("valkyrie", "valkyries")),)),
             PartQuery("ебанем", (Term("ебанем", ("ebanem",)),)))
    return ThemeHits(tuple(names), query=ThemeQuery(WORK, parts))


def _tool(*names, scores=None, table=None):
    return _tools(_rig(), hits=_hits(*(names or ("rideofth", "valkyrie", "treulich"))), table=table or TABLE,
                  scores=scores or SoundAlikeIndex())


def test_sound_alike_word_is_not_a_second_composer_and_the_found_melody_is_a_candidate():
    ref, candidates, _records = _tool()._pieces.candidates(WORK)
    assert [w.alts[0] for w in ref.composer] == ["wagner"]
    assert "ebanem" in [w.alts[0] for w in ref.title]
    assert "rideofth" in {c.id for c in candidates}
    assert {c.id for c in candidates} == {"rideofth", "treulich"}  # только Вагнер; чужая «Valkyrie» не подменяет


def test_request_plays_wagner_instead_of_no_composer():
    data = _tool().execute(intent="melody", text=WORK).data
    assert data["ok"] and data["played"]["composer"] == "Вагнер"
    assert "composer_missing" not in data


def test_title_mismatch_falls_back_to_pieces_of_the_composer():
    """Название не совпало вовсе — не «нет автора», а пьесы автора (ADR-0157 R5)."""
    data = _tool("treulich").execute(intent="melody", text=WORK).data
    assert data["ok"] and data["melody_id"] == "treulich"


def test_no_composer_only_when_the_author_has_no_pieces_at_all():
    result = _tool("valkyrie", scores=NoPieces(), table={"valkyrie": VALKYRIES["valkyrie"]}).execute(
        intent="melody", text=WORK)
    assert result.success is False and result.data["reason"] == NOT_FOUND and result.data["composer_missing"]
    assert result.data["message"] == composer_missing_text("вагнера")


def test_phrase_names_the_composer_and_does_not_quote_the_request():
    phrase = composer_missing_text("вагнера")
    assert "«Вагнера»" in phrase and "ебанем" not in phrase and "волкирий" not in phrase
    assert "названного автора" in composer_missing_text("")


def test_regression_valkyries_of_wagner_and_composer_alone():
    assert _ask("сыграй валькирии вагнера").data["melody_id"] == "rideofth"
    only = ThemeHits(("rideofth", "treulich"), query=ThemeQuery("вагнера", (
        PartQuery("вагнера", (Term("вагнера", ("wagner",)),)),)))
    ref, candidates, _r = _tools(_rig(), hits=only, table=TABLE)._pieces.candidates("вагнера")
    assert not ref.title and {c.id for c in candidates} == {"rideofth", "treulich"}


def test_single_piece_does_not_wait_for_the_theme_links_llm(tmp_path):
    """6.3 с: часть без связи в хранилище спрашивала LLM (до 8 с на пути звука). Одиночная пьеса — без вопроса."""
    asked = []
    links = ThemeLinks(ask=lambda *a: asked.append(a) or ("fail", None, ""), spawn=lambda fn: fn())
    db = str(tmp_path / "links.db")
    library = type("L", (), {"db_path": db, "_db_path": db})()
    hits = ThemeHits(missing=("ебанем",), named=True)
    assert links.expand(library, "ебанем", hits, ask=False) is hits and asked == []
    links.expand(library, "ебанем", hits)
    assert asked  # у сета спрашивает, как и раньше
