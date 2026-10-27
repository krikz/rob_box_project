"""Память о людях (PR-11 #3705): собеседник - из снимка, «Запомнил» - только по saved{fact_id} (ADR-0148)."""

import asyncio

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_core.speaker_wire import fill_speaker_result, speaker_result_to_payload
from rob_box_dialog import knowledge as kn
from rob_box_dialog import respond
from rob_box_dialog.command import ToolCall
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.memory import profiled_person
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.speaker_adapter import speaker_ref


def _snapshot(agent="personality", person="spk-sasha", name="Саша", state="confirmed", speaker=True):
    session = Session(agent)
    if speaker:
        session.set_speaker(SpeakerRef(person_ref=person, name=name, name_state=state))
    return session.snapshot


class Owner:
    """Владелец памяти-заглушка: отвечает saved{fact_id} (или своим ответом); запросы запоминает."""

    def __init__(self, answer=None):
        self.requests = []
        self.answer = answer or (lambda r: OwnerEvent(r.turn_id, "done", data={**r.args, "fact_id": 1}))
        self.executor = Executor(self)

    def __call__(self, request):
        self.requests.append(request)
        asyncio.get_running_loop().call_soon(self.executor.on_event, self.answer(request))

    def run(self, calls, snapshot):
        return asyncio.run(self.executor.run(calls, snapshot, "t1"))


def test_person_comes_from_the_snapshot_not_from_the_llm():
    owner = Owner()
    [res] = owner.run([ToolCall("memory_save", {"fact": "любит чай", kn.PERSON_ARG: "spk-boris"})], _snapshot())
    assert owner.requests[0].args[kn.PERSON_ARG] == "spk-sasha"  # слова модели о собеседнике код не слушает
    assert res.status == "done" and respond.phrase_from_result(res).text == "Запомнил: любит чай."


def test_search_and_context_are_scoped_to_the_snapshot_person_too():
    owner = Owner(lambda r: OwnerEvent(r.turn_id, "done"))
    owner.run([ToolCall("memory_search", {"query": "чай"}), ToolCall("memory_context", {})], _snapshot())
    assert [r.args[kn.PERSON_ARG] for r in owner.requests] == ["spk-sasha", "spk-sasha"]


def test_nickname_only_person_is_a_person_too():
    """#3858: у профиля может быть лишь кличка, имени нет - факт всё равно привязан к person_ref."""
    owner = Owner()
    [res] = owner.run([ToolCall("memory_save", {"fact": "любит джаз"})],
                      _snapshot(person="spk-unknown-3", name=None, state="unknown"))
    assert owner.requests[0].args[kn.PERSON_ARG] == "spk-unknown-3" and res.status == "done"


def test_no_person_in_snapshot_means_rejected_and_nothing_is_sent():
    for snapshot in (_snapshot(person=None, name=None, state="unknown"), _snapshot(speaker=False)):
        owner = Owner()
        res = owner.run([ToolCall("memory_save", {"fact": "x"}), ToolCall("get_current_time", {})], snapshot)
        assert [(r.tool, r.status, r.reason) for r in res] == [
            ("memory_save", "rejected", kn.NO_PERSON_REASON), ("get_current_time", "done", None)]
        assert [r.tool for r in owner.requests] == ["get_current_time"]  # в память не ушло
        assert respond.phrase_from_result(res[0]).text == "Не узнаю твой голос, поэтому не запомню."


def test_contested_voice_neither_reads_nor_writes_the_candidates_facts():
    """#4023 (марафон 23.10, акт 2b): голос «Саша» 0.944 / «Борис» 0.661 при band_high 0.99 — имя скрыто (contested).
    Раньше собеседником становился профиль кандидата: memory_search{query:'Саша'} без speaker_id читал факты Саши.
    Теперь спорный голос — как незнакомец: и поиск, и запись — rejected{no_person}, к владельцу памяти не уходит
    ничего, даже если модель сама вписала id кандидата."""
    wire = install_dialog_msgs_stub().SpeakerResult()
    result = speaker_result_to_payload(fill_speaker_result(wire, {
        "is_known": True, "speaker_id": "spk-sasha", "name": None, "confidence": 0.944, "epithet": "Спутник",
        "tentative_conf": 0.944, "tentative_kind": "contested", "utterance_id": "u-2b"}))
    session = Session("personality")
    session.set_speaker(SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed"))  # до спора — Саша
    session.set_speaker(speaker_ref(result, now=1.0, prev=session.snapshot.speaker))
    owner = Owner()
    res = owner.run([ToolCall("memory_search", {"query": "Саша"}),
                     ToolCall("memory_save", {"fact": "я люблю чай", kn.PERSON_ARG: "spk-sasha"}),
                     ToolCall("memory_context", {})], session.snapshot)
    assert [(r.tool, r.status, r.reason) for r in res] == [
        ("memory_search", "rejected", kn.NO_PERSON_REASON), ("memory_save", "rejected", kn.NO_PERSON_REASON),
        ("memory_context", "rejected", kn.NO_PERSON_REASON)]
    assert owner.requests == []  # к владельцу памяти не ушло ни одного запроса
    assert profiled_person(session.snapshot.speaker) is None  # и профиль собеседника по спорному голосу не ведётся


def test_contested_turn_snapshot_says_familiar_voice_and_shows_no_facts_of_the_candidate():
    """#3993: снимок хода в споре — не «unknown» (из него модель вывела «голос я пока не запомнил»), а ``contested``;
    факты Саши, взятые на начале хода, когда он был подтверждён, в снимок спорного голоса не попадают."""
    from rob_box_dialog.music_state import MusicStateMemory
    from rob_box_dialog.turn import host_sections

    session = Session("personality")
    session.set_speaker(SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed"))
    noted = (profiled_person(session.snapshot.speaker), {"dialog_count": 3, "facts": ["пьёт зелёный чай"]})
    contested = {"is_known": True, "speaker_id": "spk-sasha", "name": None, "confidence": 0.892,
                 "tentative_conf": 0.892, "tentative_kind": "contested"}
    session.set_speaker(speaker_ref(contested, now=2.0, prev=session.snapshot.speaker))
    sections = host_sections(session.snapshot, MusicStateMemory(), (), noted)
    assert sections["speaker"] == {"name": None, "name_state": "contested"}
    assert "speaker_memory" not in sections


def test_done_without_saved_evidence_is_not_remembered():
    owner = Owner(lambda r: OwnerEvent(r.turn_id, "done", data={"fact": "x"}))  # владелец не вернул fact_id
    [res] = owner.run([ToolCall("memory_save", {"fact": "x"})], _snapshot())
    assert (res.status, res.reason) == ("rejected", kn.NO_EVIDENCE_REASON)
    assert "Запомнил" not in respond.phrase_from_result(res).text


def test_agents_without_interlocutor_keep_their_args():
    owner = Owner()
    [res] = owner.run([ToolCall("memory_save", {"fact": "x"})], _snapshot(agent="telegram", speaker=False))
    assert kn.PERSON_ARG not in owner.requests[0].args and res.status == "done"


def test_speaker_memory_block_is_data_capped_by_knowledge():
    from rob_box_dialog.memory import speaker_memory_block

    assert speaker_memory_block(None, []) is None
    assert speaker_memory_block({"dialog_count": 0}, []) == {"dialog_count": 0}  # ключ без данных пропущен
    block = speaker_memory_block({"dialog_count": 3}, [" любит чай ", "", "я" * 999] + ["x"] * 50)
    assert block["dialog_count"] == 3 and len(block["facts"]) == kn.SPEAKER_FACTS_MAX
    assert block["facts"][0] == "любит чай" and len(block["facts"][1]) == kn.SPEAKER_FACT_MAX_CHARS
    assert profiled_person(_snapshot().speaker) == "spk-sasha"
    assert profiled_person(_snapshot(state="tentative").speaker) is None  # догадка по голосу в профиль не идёт
    assert profiled_person(_snapshot(person=None).speaker) is None
