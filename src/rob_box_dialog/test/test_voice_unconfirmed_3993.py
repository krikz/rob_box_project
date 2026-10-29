"""#3993 (остаток после PR #4048): честность спорного голоса — решает код (ADR-0148), без вопроса «ты Саша?»
(решение Шифу 26.10).

Акт 2b марафона 23.10: «Саша» 0.944 / «Борис» 0.661 — имя скрыто (``contested``). После #4048 код на «запомни …»
говорил ``local.rejected.no_person`` «Не узнаю твой голос, поэтому не запомню.» — неправда: голос узнан, не
подтверждено только имя. Теперь своя причина ``voice_unconfirmed`` и честная фраза; гипотеза голоса (``tentative``)
для памяти — как спор (вопроса личности нет, чужие факты хуже пустых); спорная фраза бэклога — никому.
"""

import asyncio
import types

from rob_box_dialog import knowledge as kn
from rob_box_dialog import respond
from rob_box_dialog.address import person_ref_from
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.command import ToolCall
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.memory import bind_person, unbound_reason
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.speaker_adapter import VoiceTag, phrase_speaker, speaker_ref, voice_tag
from rob_box_dialog.turn_log import TurnLog

#: Акт 2b, 21:30:37: голос узнан как «Саша» (0.944), имя скрыто спором (конкурент «Борис»).
CONTESTED = {"is_known": True, "speaker_id": "spk-sasha", "name": None, "confidence": 0.944, "epithet": "Спутник",
             "tentative_conf": 0.944, "tentative_kind": "contested", "utterance_id": "u-2b"}
TENTATIVE = {"is_known": True, "speaker_id": "spk-sasha", "name": None, "confidence": 0.78,
             "tentative_name": "Саша", "tentative_kind": "single", "utterance_id": "u-t"}
CONFIRMED = {"is_known": True, "speaker_id": "spk-sasha", "name": "Саша", "confidence": 0.9, "utterance_id": "u-c"}
NAMELESS = {"is_known": True, "speaker_id": "spk-guest-3", "name": None, "epithet": "Гость", "utterance_id": "u-n"}
HONEST = "Голос знакомый, но не уверен чей — запоминать не буду."


def _snapshot(result):
    session = Session("personality")
    session.set_speaker(speaker_ref(result, now=1.0))
    return session.snapshot


class Owner:
    """Владелец памяти-заглушка: ``saved{fact_id}`` на всё; запросы запоминает."""

    def __init__(self):
        self.requests = []
        self.executor = Executor(self)

    def __call__(self, request):
        self.requests.append(request)
        event = OwnerEvent(request.turn_id, "done", data={**request.args, "fact_id": 1})
        asyncio.get_running_loop().call_soon(self.executor.on_event, event)

    def run(self, calls, snapshot):
        return asyncio.run(self.executor.run(calls, snapshot, "t1"))


# -- (1) своя причина и честная фраза -------------------------------------------------------------------------


def test_contested_memory_save_says_the_voice_is_familiar_not_unknown():
    owner = Owner()
    [res] = owner.run([ToolCall("memory_save", {"fact": "я люблю чай"})], _snapshot(CONTESTED))
    assert (res.status, res.reason) == ("rejected", kn.VOICE_UNCONFIRMED_REASON) and owner.requests == []
    said = respond.phrase_from_result(res)
    assert said.text == HONEST and "Не узнаю" not in said.text and "?" not in said.text  # без вопроса личности
    assert said.emotion == "sad"  # отказ — грустно, как любой отказ


def test_unknown_voice_keeps_no_person_and_its_phrase():
    for snapshot in (_snapshot({"is_known": False}), Session("personality").snapshot):
        owner = Owner()
        [res] = owner.run([ToolCall("memory_save", {"fact": "x"})], snapshot)
        assert res.reason == kn.NO_PERSON_REASON and owner.requests == []
        assert respond.phrase_from_result(res).text == "Не узнаю твой голос, поэтому не запомню."


def test_unbound_reason_is_a_function_of_the_snapshot_speaker():
    assert unbound_reason(_snapshot(CONTESTED)) == kn.VOICE_UNCONFIRMED_REASON
    assert unbound_reason(_snapshot(TENTATIVE)) == kn.VOICE_UNCONFIRMED_REASON
    assert unbound_reason(_snapshot({"is_known": False})) == kn.NO_PERSON_REASON
    assert unbound_reason(Session("personality").snapshot) == kn.NO_PERSON_REASON


# -- (1b) гипотеза голоса для памяти — как спор ---------------------------------------------------------------


def test_tentative_voice_neither_reads_nor_writes_the_candidates_facts():
    snap = _snapshot(TENTATIVE)
    assert (snap.speaker.person_ref, snap.speaker.name_state) == ("spk-sasha", "tentative")  # кандидат в снимке есть
    owner = Owner()
    res = owner.run([ToolCall("memory_save", {"fact": "пью зелёный чай"}), ToolCall("memory_search", {"query": "чай"}),
                     ToolCall("memory_context", {})], snap)
    assert [(r.status, r.reason) for r in res] == [("rejected", kn.VOICE_UNCONFIRMED_REASON)] * 3
    assert owner.requests == [], "к владельцу памяти не ушло ничего — ни запись, ни чтение кандидата"


def test_confirmed_and_nameless_own_profile_still_bind():
    for result, person in ((CONFIRMED, "spk-sasha"), (NAMELESS, "spk-guest-3")):
        args = bind_person(ToolCall("memory_save", {"fact": "x"}), _snapshot(result))
        assert args is not None and args[kn.PERSON_ARG] == person
    assert bind_person(ToolCall("memory_save", {"fact": "x"}), _snapshot(TENTATIVE)) is None
    assert bind_person(ToolCall("get_current_time", {}), _snapshot(TENTATIVE)) == {}, "не личный тул — как был"


def test_rejected_unconfirmed_fact_does_not_stay_in_the_llm_window():
    """Как ``no_person`` (#3881): отклонённый факт спорного голоса не переезжает к следующему собеседнику."""
    log = TurnLog()
    log.begin("t1", 0, "запомни я пью чай", "незнакомый голос")
    rejected = respond.ActionResult("t1/0", 0, "personality", "memory_save", "action.local", "rejected",
                                    kn.VOICE_UNCONFIRMED_REASON)
    log.result("t1", rejected)
    assert not [e for e in log.window() if e.role == "tool"]
    log.result("t1", respond.ActionResult("t1/1", 0, "personality", "memory_save", "action.local", "rejected",
                                          "error"))
    assert [e for e in log.window() if e.role == "tool"], "прочие отказы памяти в окне остаются"


# -- (1c) спорная фраза бэклога — никому ----------------------------------------------------------------------


def test_contested_voice_tag_has_no_candidate_person_name_or_epithet():
    tag = voice_tag(CONTESTED)
    assert tag == VoiceTag("contested")
    assert (tag.person_ref, tag.name, tag.tentative, tag.candidate, tag.epithet) == ("", "", "", "", "")
    out = phrase_speaker(tag, faces={"саша"})
    assert out == {"identified": False, "label": kn.UNIDENTIFIED_SPEAKER, "bio": "contested"}


def test_tagged_voices_other_than_contested_are_as_before():
    assert voice_tag(CONFIRMED).person_ref == "spk-sasha" and voice_tag(CONFIRMED).name == "Саша"
    assert voice_tag(TENTATIVE) == VoiceTag("known", "spk-sasha", tentative="Саша", score=0.78)
    assert voice_tag(NAMELESS).person_ref == "spk-guest-3"


def test_contested_phrase_in_the_backlog_is_attributed_to_nobody():
    backlog = Backlog()
    backlog.add(Heard(text="запомни что я люблю чай", t=95.0, voice_s=2.6, utterance_id="u-2b"))
    backlog.note_voice("u-2b", voice_tag(CONTESTED))
    heard = Heard(text="робот", t=100.0)
    (entry,) = backlog.pickup(heard)
    assert (entry.person_ref, entry.speaker_name) == ("", "")
    assert Backlog.label(entry, heard)["speaker"]["identified"] is False


def test_heard_of_a_contested_phrase_has_no_person_ref():
    registry = types.SimpleNamespace(peek={"u-2b": CONTESTED, "u-c": CONFIRMED, "u-n": NAMELESS,
                                           "u-x": {"is_known": False, "speaker_id": "zzz"}}.get)
    assert person_ref_from(registry, "u-2b") == ""
    assert person_ref_from(registry, "u-c") == "spk-sasha" and person_ref_from(registry, "u-n") == "spk-guest-3"
    assert person_ref_from(registry, "u-x") == "" and person_ref_from(registry, "nope") == ""


def test_contested_snapshot_speaker_is_still_nobody():
    """Регресс #4048: собеседник снимка при споре — без профиля, но честно ``contested``."""
    ref = speaker_ref(CONTESTED, now=1.0, prev=SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed"))
    assert (ref.person_ref, ref.name, ref.name_state) == (None, None, "contested")
