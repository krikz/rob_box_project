"""Адаптер собеседника (ADR-0150 §7.5, ADR-0130 §2.6; #3695): SpeakerRef, «тот же собеседник», правило вопроса В17."""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.backlog import Heard
from rob_box_dialog.speaker_adapter import (
    ACCUMULATING,
    ASK,
    ASKED,
    COMMAND,
    NO_SPEECH,
    STRANGER,
    AskCandidate,
    AskPolicy,
    FaceSightings,
    ask_after_turn,
    same_interlocutor,
    speaker_block,
    speaker_ref,
)
from rob_box_dialog.session import SpeakerRef


def _h(t=100.0, doa=None, person="", voice_s=0.0, faces=(), name=""):
    return Heard(text="x", t=t, doa_deg=doa, person_ref=person, voice_s=voice_s, faces=frozenset(faces),
                 speaker_name=name)


# -- SpeakerRef ---------------------------------------------------------------------------------------------


def test_confident_voice_name_is_confirmed():
    ref = speaker_ref({"is_known": True, "speaker_id": "p1", "name": "Борис"}, now=5.0)
    assert (ref.person_ref, ref.name, ref.name_state, ref.channels) == ("p1", "Борис", "confirmed", ("voice",))


def test_tentative_voice_confirmed_only_by_face_with_the_same_name():
    sp = {"is_known": True, "speaker_id": "p1", "name": None, "tentative_name": "Саша"}
    assert speaker_ref(sp, now=1.0).name_state == "tentative"
    assert speaker_ref(sp, now=1.0, face_names={"литвин"}).name_state == "tentative"
    ref = speaker_ref(sp, now=1.0, face_names={"саша"})
    assert (ref.name_state, ref.name, ref.channels) == ("confirmed", "Саша", ("voice", "face"))


@pytest.mark.parametrize("sp", [
    {"is_known": False},
    {"is_known": False, "inconclusive": True},
    {"is_known": True, "speaker_id": "p9", "name": None, "tentative_kind": "contested"},
])
def test_no_name_is_unknown(sp):
    ref = speaker_ref(sp, now=1.0, face_names={"литвин"})
    assert ref.name_state == "unknown" and ref.name is None


def test_face_alone_never_names_the_speaker():
    """Фейковое лицо «Литвин» у стенда: лицо без голоса имени собеседнику не даёт (эпизод Б 20.10)."""
    assert speaker_ref({"is_known": False}, now=1.0, face_names={"литвин"}).name is None


def test_since_kept_for_the_same_person():
    first = speaker_ref({"is_known": True, "speaker_id": "p1", "name": "Борис"}, now=5.0)
    again = speaker_ref({"is_known": True, "speaker_id": "p1", "name": "Борис"}, now=9.0, prev=first)
    other = speaker_ref({"is_known": True, "speaker_id": "p2", "name": "Саша"}, now=9.0, prev=first)
    assert again.since == 5.0 and other.since == 9.0


# -- FaceSightings ------------------------------------------------------------------------------------------


class _Event:
    def __init__(self, event_type, embedding_id, display_name=""):
        self.event_type, self.embedding_id, self.display_name = event_type, embedding_id, display_name


def test_face_sightings_window_labels_and_history():
    faces = FaceSightings(history_s=60.0)
    faces.note_event(_Event("face", "f1", "Литвин"), 10.0)
    faces.note_event(_Event("face", "f2", ""), 11.0)
    faces.note_event(_Event("person", "", ""), 11.0)
    assert faces.during(9.0, 12.0) == {"литвин", "#f2"}
    assert faces.names_during(9.0, 12.0) == {"литвин"}
    assert faces.during(12.0, 20.0) == frozenset()
    faces.note("f3", "", 100.0)
    assert faces.during(0.0, 100.0) == {"#f3"}, "старше history_s выброшены"


# -- «тот же собеседник» (Шифу 21.10: ИЛИ, не И) ----------------------------------------------------------


def test_voice_of_long_backlog_phrase_alone_is_enough_nobody_in_frame():
    assert same_interlocutor(_h(person="p1", voice_s=2.5), _h(doa=200.0)) == ("voice",)


def test_short_backlog_phrase_voice_is_not_trusted():
    assert same_interlocutor(_h(person="p1", voice_s=1.0), _h()) == ()


def test_long_voice_of_another_person_contradicts_every_channel():
    entry = _h(person="p1", voice_s=3.0, doa=90.0, name="борис")
    heard = _h(person="p2", voice_s=3.0, doa=90.0, faces={"борис"})
    assert same_interlocutor(entry, heard) == ()


def test_short_wake_voice_of_another_person_is_ignored():
    """Короткому «робот» по голосу не верим: чужой id на нём ничего не решает."""
    assert same_interlocutor(_h(person="p1", voice_s=3.0), _h(person="p2", voice_s=0.6)) == ("voice",)


def test_person_whose_voice_was_recognised_is_seen_in_frame():
    entry = _h(person="p1", voice_s=1.0, name="Саша")
    assert same_interlocutor(entry, _h(faces={"саша"})) == ("face",)
    assert same_interlocutor(entry, _h(faces={"литвин"})) == ()


def test_someone_in_frame_is_not_the_same_person_by_itself():
    """Перед роботом кто-то сидит, телевизор говорит с другой стороны — подхвата нет."""
    assert same_interlocutor(_h(doa=250.0, faces={"литвин"}), _h(doa=80.0, faces={"литвин"})) == ()


@pytest.mark.parametrize("entry_doa, wake_doa, faces, same", [
    (310.0, 5.0, {"#f1"}, True),     # размах ReSpeaker 21.10 304°…65° (121°): ±60° ловит соседние отсчёты
    (10.0, 65.0, {"#f1"}, True),
    (0.0, kn.PICKUP_DOA_SECTOR_DEG + 1.0, {"#f1"}, False),
    (90.0, 95.0, (), False),         # в кадре никого — только голос
    (None, 95.0, {"#f1"}, False),
])
def test_doa_is_weak_sixty_degrees_and_needs_someone_in_frame(entry_doa, wake_doa, faces, same):
    assert bool(same_interlocutor(_h(doa=entry_doa), _h(doa=wake_doa, faces=faces))) is same


def test_channels_follow_the_knowledge_table(monkeypatch):
    monkeypatch.setattr(kn, "PICKUP_CHANNELS", ("voice", "face"))
    assert same_interlocutor(_h(doa=90.0), _h(doa=90.0, faces={"#f1"})) == ()


# -- В17: когда спрашивать имя ------------------------------------------------------------------------------


def test_never_on_a_command_turn_even_when_accumulated():
    policy = AskPolicy()
    for i in range(5):
        policy.note("p1", float(i))
    command = AskCandidate("p1", "single", "Саша", request="робот покажи анимацию сон")
    verdict = ask_after_turn(policy, command, t=5.0, tools=["play_animation", "speak_text"])
    assert (verdict.verdict, verdict.command_by) == (COMMAND, ("animation",))
    chat = AskCandidate("p1", "single", "Саша", request="робот как у тебя дела")
    assert ask_after_turn(policy, chat, t=5.0, tools=["speak_text"]).verdict == ASK


@pytest.mark.parametrize("request_text, tools", [
    ("робот как у тебя дела", ["get_robot_status", "speak_text"]),
    ("робот расскажи что нибудь интересное про луну", ["speak_text", "play_animation"]),
    ("который час и сколько заряда", ["get_current_time", "get_battery_level", "speak_text"]),
    ("найди в интернете что такое квазар", ["search_web", "speak_text"]),
    ("что ты видишь перед собой", ["get_perception_context", "speak_text"]),
])
def test_reads_and_llm_own_animation_are_not_a_command(request_text, tools):
    """Живой робот 21.10: тулы чтения и анимация, которую LLM добавила к болтовне, командой не делают."""
    policy = AskPolicy(min_phrases=1)
    policy.note("p1", 0.0)
    verdict = ask_after_turn(policy, AskCandidate("p1", "unknown", request=request_text), t=1.0, tools=tools)
    assert (verdict.verdict, verdict.command_by) == (ASK, ())


@pytest.mark.parametrize("request_text, tool, group", [
    ("робот покажи анимацию сон", "play_animation", "animation"),
    ("робот включи музыку", "request_music", "music"),
    ("поезжай на кухню", "navigate_to_waypoint", "navigation"),
    ("сделай погромче", "set_volume", "volume"),
    ("меня зовут Гриша", "register_speaker", "people"),
])
def test_executed_request_is_a_command(request_text, tool, group):
    policy = AskPolicy(min_phrases=1)
    policy.note("p1", 0.0)
    cand = AskCandidate("p1", "unknown", request=request_text)
    verdict = ask_after_turn(policy, cand, t=1.0, tools=[tool, "speak_text"])
    assert verdict.verdict == COMMAND and group in verdict.command_by


@pytest.mark.parametrize("request_text, tools", [
    ("ты меня узнал по голосу", ["speak_text"]),  # слово группы в болтовне, ничего не исполнено
    ("что ты обо мне помнишь", ["memory_search", "speak_text"]),  # группа названа, тул — чтение
])
def test_group_word_without_executed_action_is_not_a_command(request_text, tools):
    policy = AskPolicy(min_phrases=1)
    policy.note("p1", 0.0)
    verdict = ask_after_turn(policy, AskCandidate("p1", "unknown", request=request_text), t=1.0, tools=tools)
    assert (verdict.verdict, verdict.command_by) == (ASK, ())


def test_verdict_carries_its_reason():
    policy = AskPolicy()
    cand = AskCandidate(STRANGER, "unknown", request="расскажи анекдот")
    policy.note(STRANGER, 0.0)
    policy.note(STRANGER, 10.0)
    first = ask_after_turn(policy, cand, t=10.0, spoken_text="ок")
    assert (first.verdict, first.phrases, first.min_phrases, first.since_first_s, first.asked_at) == (
        ACCUMULATING, 2, kn.IDENTITY_ASK_MIN_PHRASES, 10.0, None)
    policy.note(STRANGER, 20.0)
    assert ask_after_turn(policy, cand, t=20.0, spoken_text="ок").verdict == ASK
    policy.note(STRANGER, 30.0)
    again = ask_after_turn(policy, cand, t=30.0, spoken_text="ок")
    assert (again.verdict, again.asked_at) == (ASKED, 20.0)
    assert "command_by=none" in again.describe(30.0) and "asked_before=yes(10s ago)" in again.describe(30.0)


def test_empty_turn_is_not_a_reason_to_ask():
    """Эпизод Б 20.10: ``spoken=''`` → «Как тебя зовут?» — больше нет."""
    policy = AskPolicy(min_phrases=1)
    policy.note(STRANGER, 0.0)
    assert ask_after_turn(policy, AskCandidate(STRANGER, "unknown"), t=1.0, spoken_text="").verdict == NO_SPEECH


def test_accumulates_three_phrases_then_asks_once():
    policy = AskPolicy()
    cand = AskCandidate("p1", "unknown")
    verdicts = []
    for i in range(5):
        policy.note("p1", float(i))
        verdicts.append(ask_after_turn(policy, cand, t=float(i), spoken_text="ок").verdict)
    assert verdicts == [ACCUMULATING, ACCUMULATING, ASK, ASKED, ASKED]


def test_sixty_seconds_with_two_phrases_is_enough():
    policy = AskPolicy()
    policy.note("p1", 0.0)
    policy.note("p1", kn.IDENTITY_ASK_MIN_S - 1.0)
    assert policy.verdict("p1", kn.IDENTITY_ASK_MIN_S - 1.0, has_actions=False, spoken=True) == ACCUMULATING
    assert policy.verdict("p1", kn.IDENTITY_ASK_MIN_S, has_actions=False, spoken=True) == ASK
    single = AskPolicy()
    single.note("p2", 0.0)
    assert single.verdict("p2", 300.0, has_actions=False, spoken=True) == ACCUMULATING


def test_stranger_again_only_after_ten_minutes():
    policy = AskPolicy(min_phrases=1)
    policy.note(STRANGER, 0.0)
    assert ask_after_turn(policy, AskCandidate(STRANGER, "unknown"), t=0.0, spoken_text="ок").verdict == ASK
    policy.note(STRANGER, 10.0)
    assert policy.verdict(STRANGER, 10.0, has_actions=False, spoken=True) == ASKED
    policy.note(STRANGER, kn.IDENTITY_ASK_STRANGER_GAP_S + 1.0)
    assert policy.verdict(STRANGER, kn.IDENTITY_ASK_STRANGER_GAP_S + 1.0, has_actions=False, spoken=True) == ASK


def test_question_texts_come_from_phrases():
    assert AskCandidate("p1", "single", "Саша").question() == "Саша, это ты?"
    assert AskCandidate("p1", "contested").question() == "Как тебя зовут?"
    assert AskCandidate(STRANGER, "unknown").question() == "Как тебя зовут?"


# -- раздел speaker снимка хода (§7.3, A30) ------------------------------------------------------------------


@pytest.mark.parametrize("ref, block", [
    (None, {"name": None, "name_state": "unknown"}),
    (SpeakerRef(person_ref="p1", name="Саша", name_state="confirmed"), {"name": "Саша", "name_state": "confirmed"}),
    (SpeakerRef(person_ref="p1", name="Литвин", name_state="tentative"), {"name": None, "name_state": "tentative"}),
    (SpeakerRef(), {"name": None, "name_state": "unknown"}),
])
def test_speaker_block_names_only_confirmed(ref, block):
    """Эпизод Б 20.10: гипотеза «Литвин» в снимок не попадает — обращаться по ней нельзя."""
    assert speaker_block(ref) == block


# -- состояние гипотезы имени (#2809 -> speaker_adapter, #3695 ч.2d) ------------------------------------------


def test_tentative_state_survives_answer_window_while_waiting_then_session_gap():
    from rob_box_dialog.speaker_adapter import TentativeStates

    states = TentativeStates(answer_window_s=180.0, session_gap_s=120.0)
    state = states.fresh("p1", 0.0)
    state["asked"] = True                                   # вопрос задан, ответа ждём
    assert states.fresh("p1", 170.0) is state               # 84 с и даже 170 с — ответ ещё засчитывается
    state["confirmed"] = True                               # ответ получен — окно сессии
    assert states.fresh("p1", 289.0) is state
    fresh = states.fresh("p1", 410.0)                       # пауза 121 с > 120 с — новый разговор
    assert fresh is not state and fresh == {"asked": False, "confirmed": None, "name": None,
                                            "growth_registered": False, "last_seen_at": 410.0}
    assert states.get("p1") is fresh and states.get("p2") is None
