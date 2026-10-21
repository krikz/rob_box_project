"""Бэклог неадресованных фраз (ADR-0150 §7.3 В12; приёмка A31: 0 чужих фраз в LLM)."""

from rob_box_dialog.backlog import Backlog, Heard, same_direction
from rob_box_dialog.knowledge import BACKLOG_MAX_ENTRIES, BACKLOG_TTL_S


def _h(text, t, doa=None, person="", voice_s=0.0, faces=("f1",)):
    """По умолчанию в кадре кто-то есть: DOA засчитывается только тогда (PICKUP_DOA_NEEDS_FACE)."""
    return Heard(text=text, t=t, doa_deg=doa, person_ref=person, voice_s=voice_s, faces=frozenset(faces))


def test_tv_noise_never_reaches_the_llm_snapshot():
    """Сценарий (б): 5 фраз ТВ с другой стороны, потом «Робот, который час» — в снимке пусто."""
    backlog = Backlog()
    for i in range(5):
        backlog.add(_h(f"новости {i}", t=90.0 + i, doa=250.0))
    assert backlog.recent_unaddressed(_h("робот который час", t=100.0, doa=80.0)) is None
    assert backlog.background_phrases(100.0) == 5


def test_snapshot_gives_at_most_one_fresh_phrase_of_the_same_direction_as_structure():
    backlog = Backlog()
    backlog.add(_h("старая", t=60.0, doa=80.0))
    backlog.add(_h("покажи анимацию", t=95.0, doa=85.0))
    backlog.add(_h("телевизор", t=96.0, doa=250.0))
    assert backlog.recent_unaddressed(_h("робот", t=100.0, doa=80.0)) == {
        "text": "покажи анимацию", "age_s": 5.0, "same_direction": True}


def test_snapshot_by_person_is_not_same_direction():
    backlog = Backlog()
    backlog.add(_h("включи музыку", t=95.0, doa=10.0, person="p1", voice_s=3.0, faces=()))
    assert backlog.recent_unaddressed(_h("робот", t=100.0, doa=200.0, faces=()))["same_direction"] is False


def test_ttl_and_size_limit():
    backlog = Backlog()
    backlog.add(_h("очень старая", t=0.0))
    assert backlog.background_phrases(BACKLOG_TTL_S + 1.0) == 0
    for i in range(BACKLOG_MAX_ENTRIES + 5):
        backlog.add(_h(f"ф{i}", t=1000.0 + i))
    assert backlog.background_phrases(1000.0 + BACKLOG_MAX_ENTRIES + 5) == BACKLOG_MAX_ENTRIES


def test_pickup_takes_the_newest_matching_and_removes_it():
    backlog = Backlog()
    backlog.add(_h("первая", t=90.0, doa=80.0))
    backlog.add(_h("вторая", t=95.0, doa=80.0))
    assert backlog.pickup(_h("робот", t=100.0, doa=80.0))[0].text == "вторая"
    assert backlog.pickup(_h("робот", t=100.0, doa=80.0))[0].text == "первая"
    assert backlog.pickup(_h("робот", t=100.0, doa=80.0)) == (None, ())


def test_empty_text_is_not_stored_and_clear_empties():
    backlog = Backlog()
    backlog.add(_h("  ", t=1.0))
    backlog.add(_h("x", t=1.0))
    backlog.clear()
    assert backlog.background_phrases(1.0) == 0


def test_same_direction_sector():
    assert same_direction(350.0, 15.0) and same_direction(0.0, 60.0)
    assert not same_direction(0.0, 61.0) and not same_direction(None, 0.0)


def test_note_voice_marks_entry_by_utterance_id_for_later_pickup():
    """Биометрия приходит после STT: голос дописывается записи по utterance_id и даёт подхват без кадра и DOA."""
    backlog = Backlog()
    backlog.add(Heard(text="покажи анимацию сон", t=95.0, voice_s=2.4, utterance_id="u1"))
    assert backlog.pickup(_h("робот", t=100.0, faces=())) == (None, ())
    backlog.note_voice("u1", "p1")
    entry, channels = backlog.pickup(_h("робот", t=100.0, faces=()))
    assert entry.text == "покажи анимацию сон" and channels == ("voice",)
