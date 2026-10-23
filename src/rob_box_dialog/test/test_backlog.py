"""Бэклог неадресованных фраз (ADR-0150 §7.3 В12, #3767; приёмка A31: 0 чужих фраз в снимке адресованного хода)."""

from rob_box_dialog.backlog import Backlog, Heard, pickup_summary, same_direction
from rob_box_dialog.knowledge import (
    BACKGROUND_MAX_PHRASES,
    BACKGROUND_PHRASE_MAX_CHARS,
    BACKGROUND_WINDOW_S,
    BACKLOG_MAX_ENTRIES,
    BACKLOG_PICKUP_MAX,
    BACKLOG_TTL_S,
)
from rob_box_dialog.speaker_adapter import VoiceTag


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


def test_snapshot_gives_at_most_one_fresh_phrase_of_the_same_interlocutor_with_its_label():
    backlog = Backlog()
    backlog.add(_h("старая", t=60.0, doa=80.0))
    backlog.add(_h("покажи анимацию", t=95.0, doa=85.0, voice_s=2.0))
    backlog.add(_h("телевизор", t=96.0, doa=250.0))
    assert backlog.recent_unaddressed(_h("робот", t=100.0, doa=80.0)) == {
        "text": "покажи анимацию", "age_s": 5.0, "duration_s": 2.0, "doa_deg": 85, "same_direction": True,
        "speaker": {"identified": False, "label": "неопознанный", "bio": "none"}, "same_as_caller": True}


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


def test_pickup_takes_every_fresh_phrase_in_order_and_removes_them():
    backlog = Backlog()
    backlog.add(_h("первая", t=90.0, doa=80.0))
    backlog.add(_h("вторая", t=95.0, doa=200.0))
    assert [e.text for e in backlog.pickup(_h("робот", t=100.0, doa=80.0, faces=()))] == ["первая", "вторая"]
    assert backlog.pickup(_h("робот", t=100.0, doa=80.0)) == []


def test_long_story_24s_in_two_chunks_nobody_known_goes_whole_and_labelled_unknown():
    """#3767, живой лог #3762: 24.5 с речи → 2 куска (14.8 + 9.8 с), лиц нет, голос не узнан, «робот» через 4 с."""
    backlog = Backlog()
    backlog.add(Heard(text="слушай я вот что хотела рассказать", t=491.3, doa_deg=58.3, voice_s=14.8,
                      utterance_id="u1"))
    backlog.add(Heard(text="как лучше собрать новую полку", t=493.9, doa_deg=54.4, voice_s=9.8, utterance_id="u2"))
    backlog.note_voice("u1", VoiceTag("unknown_voice", candidate="Борис", candidate_score=0.41))
    heard = Heard(text="робот", t=498.6, doa_deg=299.0, voice_s=1.4)
    taken = backlog.pickup(heard)
    assert [e.text for e in taken] == ["слушай я вот что хотела рассказать", "как лучше собрать новую полку"]
    labels = [Backlog.label(e, heard) for e in taken]
    assert labels[0]["speaker"] == {"identified": False, "label": "неопознанный", "bio": "unknown_voice",
                                    "candidate": "Борис", "candidate_score": 0.41}
    assert labels[1]["speaker"]["label"] == "неопознанный" and labels[1]["speaker"]["bio"] == "none"
    assert pickup_summary(labels) == "n=2 speaker=неопознанный,неопознанный channels=- span=17.4s"


def test_earlier_chunk_of_the_same_story_older_than_the_window_still_goes():
    """Окно — от последнего куска: первый кусок кончился 25 с назад, но рассказ непрерывен."""
    backlog = Backlog()
    backlog.add(Heard(text="кусок 1", t=75.0, voice_s=15.0))
    backlog.add(Heard(text="кусок 2", t=89.9, voice_s=14.9))
    assert [e.text for e in backlog.pickup(Heard(text="робот", t=100.0))] == ["кусок 1", "кусок 2"]


def test_tv_phrase_older_than_the_window_does_not_go():
    backlog = Backlog()
    backlog.add(Heard(text="в эфире новости", t=60.0, voice_s=3.0, doa_deg=250.0))
    backlog.add(Heard(text="покажи анимацию", t=95.0, voice_s=2.0, doa_deg=90.0))
    assert [e.text for e in backlog.pickup(Heard(text="робот", t=100.0))] == ["покажи анимацию"]
    assert backlog.background_phrases(100.0) == 1, "ТВ остаётся в бэклоге до TTL"


def test_pickup_is_capped():
    backlog = Backlog()
    for i in range(BACKLOG_PICKUP_MAX + 3):
        backlog.add(Heard(text=f"ф{i}", t=90.0 + i))
    taken = backlog.pickup(Heard(text="робот", t=100.0))
    assert len(taken) == BACKLOG_PICKUP_MAX and taken[-1].text == f"ф{BACKLOG_PICKUP_MAX + 2}"


def test_empty_text_is_not_stored_and_clear_empties():
    backlog = Backlog()
    backlog.add(_h("  ", t=1.0))
    backlog.add(_h("x", t=1.0))
    backlog.clear()
    assert backlog.background_phrases(1.0) == 0


def test_same_direction_sector():
    assert same_direction(350.0, 15.0) and same_direction(0.0, 60.0)
    assert not same_direction(0.0, 61.0) and not same_direction(None, 0.0)


def test_note_voice_labels_the_phrase_with_the_identified_speaker():
    """Биометрия приходит после STT: голос дописывается записи по utterance_id — фраза подписана спикером."""
    backlog = Backlog()
    backlog.add(Heard(text="покажи анимацию сон", t=95.0, voice_s=2.4, utterance_id="u1"))
    backlog.note_voice("u1", VoiceTag("known", person_ref="p1", name="Борис", score=0.86, epithet="Кулибин"))
    heard = _h("робот", t=100.0, faces=())
    (entry,) = backlog.pickup(heard)
    assert Backlog.label(entry, heard)["speaker"] == {
        "identified": True, "name": "Борис", "person_ref": "p1", "channels": ["voice"], "score": 0.86,
        "epithet": "Кулибин"}
    assert Backlog.label(entry, heard)["same_as_caller"] is True


def test_background_window_gives_every_author_in_order_and_drops_older_phrases():
    """В23 (23.10): все фоновые фразы за последние 3 мин - кто, что, когда; старше окна нет; бэклог цел."""
    backlog = Backlog()
    backlog.add(_h("старше окна", t=100.0 - BACKGROUND_WINDOW_S - 5.0, doa=10.0))
    backlog.add(_h("закажи пиццу", t=90.0, doa=250.0, person="p1"))
    backlog.add(_h("не шумите", t=95.0, doa=10.0))
    section = backlog.background_since_reply(_h("робот кто говорил", t=100.0, doa=80.0))
    assert [(p["text"], p["age_s"]) for p in section] == [("закажи пиццу", 10.0), ("не шумите", 5.0)]
    assert all("speaker" in p for p in section)
    assert backlog.background_phrases(100.0) == 2


def test_background_window_is_capped_by_count_and_chars():
    backlog = Backlog()
    for i in range(BACKGROUND_MAX_PHRASES + 3):
        backlog.add(_h(f"ф{i}" + "я" * 500, t=60.0 + i))
    section = backlog.background_since_reply(_h("робот", t=100.0))
    assert len(section) == BACKGROUND_MAX_PHRASES and section[-1]["text"].startswith(f"ф{BACKGROUND_MAX_PHRASES + 2}")
    assert all(len(p["text"]) == BACKGROUND_PHRASE_MAX_CHARS for p in section)
