"""Бэклог неадресованных фраз (ADR-0150 §7.3 В12, #3767; приёмка A31: 0 чужих фраз в снимке адресованного хода)."""

from rob_box_dialog.address import decide
from rob_box_dialog.backlog import Backlog, Heard, pickup_summary, same_direction
from rob_box_dialog.knowledge import (
    BACKGROUND_MAX_PHRASES,
    BACKGROUND_PHRASE_MAX_CHARS,
    BACKGROUND_WINDOW_S,
    BACKLOG_FRESH_S,
    BACKLOG_MAX_ENTRIES,
    BACKLOG_PICKUP_MAX,
    BACKLOG_PICKUP_S,
    BACKLOG_TTL_S,
    DEFAULT_WAKE_WORDS,
)
from rob_box_dialog.speaker_adapter import VoiceTag, voice_tag
from rob_box_dialog.turn import recent_section


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
        "speaker": {"identified": False, "label": "неопознанный", "bio": "none"}, "speaker_id": "unknown",
        "same_as_caller": True}


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


def test_tv_phrase_older_than_the_fresh_border_does_not_go_with_a_fresh_one():
    """Есть свежая фраза - старая чужая (ТВ, 40 с) в ход не идёт, хотя окно бэклога её ещё держит (#3975)."""
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


def test_3975_nothing_fresh_pickup_reaches_back_to_the_last_phrase_of_the_window():
    """Свежего (моложе ``BACKLOG_FRESH_S``) нет - достаётся одна последняя фраза окна, остальное остаётся в бэклоге."""
    backlog = Backlog()
    backlog.add(_h("чужой разговор", t=800.0, doa=250.0, voice_s=3.0))
    backlog.add(_h("включи спокойное", t=900.0, doa=90.0, voice_s=2.0))
    wake_at = 900.0 + BACKLOG_FRESH_S + 9.0
    taken = backlog.pickup(_h("робот", t=wake_at, doa=90.0))
    assert [e.text for e in taken] == ["включи спокойное"]
    assert backlog.background_phrases(wake_at) == 1


def test_3975_reach_is_the_backlog_window_and_the_ttl_agrees():
    assert BACKLOG_PICKUP_S == BACKGROUND_WINDOW_S == BACKLOG_TTL_S > BACKLOG_FRESH_S
    backlog = Backlog()
    backlog.add(_h("на самом краю окна", t=1000.0 - BACKLOG_PICKUP_S, voice_s=2.0))
    assert [e.text for e in backlog.pickup(_h("робот", t=1000.0))] == ["на самом краю окна"]
    backlog.add(_h("за окном", t=2000.0 - BACKLOG_PICKUP_S - 0.5, voice_s=2.0))
    assert backlog.pickup(_h("робот", t=2000.0)) == []


def test_3975_far_pickup_is_capped_to_the_story_and_the_cap():
    """Куски одного рассказа далеко не тянутся без конца: потолок ``BACKLOG_PICKUP_MAX`` тот же."""
    backlog = Backlog()
    for i in range(BACKLOG_PICKUP_MAX + 3):
        backlog.add(Heard(text=f"кусок {i}", t=800.0 + 15.0 * i, voice_s=15.0))
    taken = backlog.pickup(Heard(text="робот", t=800.0 + 15.0 * (BACKLOG_PICKUP_MAX + 3) + 60.0))
    assert len(taken) == BACKLOG_PICKUP_MAX and taken[-1].text == f"кусок {BACKLOG_PICKUP_MAX + 2}"


def test_3975_addressed_turn_hides_earlier_phrases_from_the_far_pickup_until_clear():
    backlog = Backlog()
    backlog.add(_h("включи музыку", t=1000.0, voice_s=2.0))
    backlog.note_addressed(1005.0)
    assert backlog.pickup(_h("робот", t=1060.0)) == []
    assert backlog.background_phrases(1060.0) == 1, "для раздела снимка фраза живёт до TTL"
    backlog.add(_h("новая просьба", t=1070.0, voice_s=2.0))
    assert [e.text for e in backlog.pickup(_h("робот", t=1100.0))] == ["новая просьба"]
    backlog.clear()
    backlog.add(_h("после очистки", t=10.0, voice_s=2.0))
    assert [e.text for e in backlog.pickup(_h("робот", t=100.0))] == ["после очистки"]


# --- #4145 (решение Шифу 28.10): у каждой записи бэклога автор — speaker id или unknown ------------------------------


#: Биометрия фраз бэклога: узнан с именем, безымянный профиль с кличкой (#3858), незнакомый голос, спорный голос
#: (#2809: ``speaker_id`` — лишь ближайший кандидат), результата нет вовсе.
_AUTHORS = (
    ("u-named", {"is_known": True, "speaker_id": "spk-boris", "name": "Борис", "confidence": 0.9}, "spk-boris"),
    ("u-nameless", {"is_known": True, "speaker_id": "spk-76b59285", "epithet": "Наблюдатель", "confidence": 0.8},
     "spk-76b59285"),
    ("u-stranger", {"is_known": False}, "unknown"),
    ("u-contested", {"is_known": True, "speaker_id": "spk-sasha", "tentative_name": "Саша",
                     "tentative_kind": "contested", "confidence": 0.9}, "unknown"),
    ("u-none", None, "unknown"),
)


def test_every_backlog_entry_has_an_author_speaker_id_or_unknown_4145():
    """Шифу 28.10: «в бэклог должны уходить идентификаторы кто что сказал». Биометрия приходит после STT
    (``note_voice``) — у каждой записи свой автор: id профиля (и безымянного с кличкой) или ``unknown``."""
    backlog = Backlog()
    for i, (uid, payload, _author) in enumerate(_AUTHORS):
        backlog.add(Heard(text=f"фраза {uid}", t=90.0 + i, voice_s=3.0, utterance_id=uid))
        if payload is not None:
            backlog.note_voice(uid, voice_tag(payload))
    authors = [author for _uid, _payload, author in _AUTHORS]
    heard = _h("робот кто говорил", t=100.0, faces=())
    assert [p["speaker_id"] for p in backlog.background_since_reply(heard)] == authors
    taken = backlog.pickup(_h("робот", t=100.0, faces=()))
    assert [Backlog.label(e, heard)["speaker_id"] for e in taken] == authors


def test_author_of_a_backlog_phrase_reaches_the_wake_turn_snapshot_4145():
    """Фраза без вейка безымянного голоса «Наблюдатель» (#3858), потом тот же голос с вейком: в снимке хода
    (``recent_unaddressed``) — эта фраза и её автор, id профиля, а не только «неопознанный»."""
    backlog = Backlog()
    backlog.add(Heard(text="включи что нибудь весёлое", t=95.0, voice_s=3.0, utterance_id="u-bg"))
    backlog.note_voice("u-bg", voice_tag(_AUTHORS[1][1]))
    wake = Heard(text="Робот, ну что скажешь", t=100.0, voice_s=3.0, person_ref="spk-76b59285")
    (recent,) = recent_section(decide(wake, backlog, list(DEFAULT_WAKE_WORDS)))
    assert recent["text"] == "включи что нибудь весёлое"
    assert recent["speaker_id"] == "spk-76b59285" and recent["same_as_caller"] is True
    assert recent["who"] == "незнакомый голос"  # имени нет — вслух не называем, но автор известен по id
