"""#3599: «сыграй полет волкирий» - опечатка STT и русское слово, которого нет в названии архива, не уводят заказ в LLM.

На настоящем архиве RTTTL (как ``test_engine_search``). Правило «та самая мелодия» - ``named_play.melody_hit``, то же,
что у роутера.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary, match_info
from rob_box_mcp_tools.engine.classic import find_record
from rob_box_mcp_tools.engine.search import find, find_rare, terms
from rob_box_voice.core.named_play import melody_hit

pytestmark = pytest.mark.unit


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("typo") / "voice_memory.db"))


@pytest.mark.parametrize("text", ["полет валькирий", "полёт валькирий", "полет волкирий", "сыграй полет волкирий"])
def test_valkyries_found_with_the_stt_typo_without_llm(library, text):
    record, reason = find_record(library, text)
    assert record is not None and record["name"] == "rideofth", reason


@pytest.mark.parametrize("text", ["полет валькирий", "полет волкирий"])
def test_rare_title_word_carries_the_name_when_the_other_word_is_unknown(library, text):
    """«полёт» архиву неизвестно, «валькирий» есть у двух записей: это та самая мелодия (``lookup_melody``)."""
    rare = find_rare(library, text)
    assert rare.found and rare.record["name"] == "rideofth"
    hit, why = melody_hit({"name": rare.record["name"], "title": rare.record["title"],
                           "match": match_info(library, rare.record, rare.query)})
    assert hit is not None and hit.key == "rideofth", why


@pytest.mark.parametrize("text", ["гимн вселенной", "гимн германии", "марио и тетрис", "сыграй полет", "король лев"])
def test_rare_word_rule_does_not_invent_a_match(library, text):
    """Понятие реестра («гимн»), слово, которое архив знает, а запись нет («германии», «марио»), и общее слово - не
    «редкое слово названия»."""
    assert not find_rare(library, text).found


def test_vowel_typo_fix_does_not_touch_known_words_or_invent_matches(library):
    assert [t.word for t in terms(library, "калинку")] == ["калинку"]
    assert [t.word for t in terms(library, "король лев")] == ["король"]  # «карол» ~ Carol - не опечатка
    assert [t.word for t in terms(library, "волкирий")] == ["валкирий"]  # исправленное слово архив знает
    assert [t.word for t in terms(library, "бухгалтерский")] == ["бухгалтерский"]
    assert not find(library, "бухгалтерский отчёт").found
