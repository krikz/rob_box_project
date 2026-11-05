"""Подхват бэклога: автор фразы и инициатор подхвата (решение Шифу 05.11, М12 n213; ``pickup_parties``).

Живой сценарий — CI run 41969501900 (L: E2E Voice Test, 05.11 04:52 UTC), лог voice_e2e_41969501900.log:

* 1793854341.31 ``identify candidates: best='Саша'(2036e9af) score=0.956 | second='Борис' score=0.694 | речь=4.77s``
  (фраза 8.92 с, utterance_id=72e99aaddafe), STT 343.09: «ребят поищи у себя в памяти что я говорил про чай»
  (вейк потерян) → ``[backlog] accumulated``;
* 1793854349.50 ``best='Саша' score=0.764 | second='Борис' score=0.671 | gap=0.092 | речь=0.96s`` → ``contested``
  (utterance_id=6156bed9d5b3), STT 350.57: «робот» → ``pickup n=1 speakers=Саша``;
* модель: ``memory_search {"query": "чай"}`` → ``отказ чтения памяти [('memory_search', 'voice_unconfirmed')]`` →
  «Голос знакомый, но не уверен чей — не буду гадать, что помню.»

Должно: права хода — у автора фразы (Саша, 0.956), модель видит автора и инициатора отдельно; память одного
человека другому без признака не уходит.
"""

import asyncio
import json

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import BACKLOG_PICKUP, VOICE, Addressed, decide
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.pickup_parties import (PICKUP_SECTION, phrase_certainty, pickup_section, pickup_speaker,
                                           rights_speaker, turn_speaker)
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.speaker_adapter import speaker_block, speaker_ref, voice_tag
from rob_box_dialog.turn import Answer, TurnEngine

SASHA_ID, BORIS_ID = "2036e9af", "17214765"
#: Фраза Саши без вейка (n213, 1793854341.31): уверенно, 0.956.
SASHA_PHRASE = {"is_known": True, "speaker_id": SASHA_ID, "name": "Саша", "confidence": 0.956,
                "epithet": "Собеседник", "utterance_id": "72e99aaddafe"}
#: Голое «Робот!» (n213, 1793854349.50): спор «Саша» 0.764 / «Борис» 0.671, имя скрыто.
WAKE_CONTESTED = {"is_known": True, "speaker_id": SASHA_ID, "name": None, "confidence": 0.764, "epithet": "Собеседник",
                  "tentative_conf": 0.764, "tentative_kind": "contested", "utterance_id": "6156bed9d5b3"}
BORIS_PHRASE = {"is_known": True, "speaker_id": BORIS_ID, "name": "Борис", "confidence": 0.91, "utterance_id": "u-b"}
UNKNOWN_PHRASE = {"is_known": False, "candidate_name": "Борис", "candidate_score": 0.41, "utterance_id": "u-x"}
TEA = "ребят поищи у себя в памяти что я говорил про чай"


def _ref(payload, now=349.5):
    return speaker_ref(payload, now=now)


def _pickup(phrases, wake_t=350.57, wake_faces=frozenset(), wake_doa=None):
    """Бэклог с фразами ``(текст, t, voice_s, payload биометрии, doa, лица)`` и голое «робот» — решение адресации."""
    backlog = Backlog()
    for n, (text, t, voice_s, payload, doa, faces) in enumerate(phrases):
        uid = payload.get("utterance_id") or f"u{n}"
        backlog.add(Heard(text, t=t, voice_s=voice_s, utterance_id=uid, doa_deg=doa, faces=frozenset(faces)))
        if payload:
            backlog.note_voice(uid, voice_tag(payload))
    return decide(Heard("робот", t=wake_t, voice_s=5.08, utterance_id="6156bed9d5b3", doa_deg=wake_doa,
                        faces=frozenset(wake_faces)), backlog, ["робот"])


def _n213():
    return _pickup([(TEA, 343.09, 8.92, SASHA_PHRASE, None, ())])


class MemoryPorts:
    """Транспорт хода с настоящим исполнителем памяти: запрос владельцу уходит только с ``person_ref`` снимка."""

    def __init__(self, answers):
        self.answers, self.asked, self.said, self.requests = list(answers), [], [], []
        self.executor = Executor(self._owner)

    def _owner(self, request):
        self.requests.append(request)
        data = {**request.args, "fact_id": 1, "results": [{"fact": "пьёт только зелёный чай"}]}
        asyncio.get_running_loop().call_soon(self.executor.on_event, OwnerEvent(request.turn_id, "done", data=data))

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return await self.executor.run(actions, snapshot, turn_id)

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        return None

    def cancel_speech(self, *args, **kwargs):
        return None

    def intent(self, *args, **kwargs):
        return None


def _run(addressed, initiator, answers):
    ports = MemoryPorts(answers)
    ids = iter(f"t{n}" for n in range(1, 50))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids),
                        sections=lambda snap, _text: {"speaker": speaker_block(snap.speaker)})  # как host_sections
    if initiator is not None:
        engine.session.set_speaker(initiator)
    asyncio.run(engine.run(addressed))
    return ports, engine


def _section(ports):
    """Раздел ``backlog_pickup`` первого запроса к модели."""
    context = ports.asked[0][-1]["content"]
    start = context.index(f"<{PICKUP_SECTION}>") + len(PICKUP_SECTION) + 2
    return json.loads(context[start:context.index(f"</{PICKUP_SECTION}>")])


_SEARCH = Command(actions=(ToolCall("memory_search", {"query": "чай"}),))
_SAVE = Command(actions=(ToolCall("memory_save", {"fact": "пьёт только зелёный чай"}),))


# -- живой n213 -------------------------------------------------------------------------------------------------


def test_n213_memory_search_goes_to_the_phrase_author_not_the_contested_wake():
    """Было: ``memory_search`` → ``rejected{voice_unconfirmed}`` и «Голос знакомый, но не уверен чей…».
    Стало: запрос владельцу памяти с ``person_ref`` Саши — автора фразы (0.956); короткий спорный вейк прав не
    решает."""
    addressed = _n213()
    assert isinstance(addressed, Addressed) and addressed.source == BACKLOG_PICKUP
    answers = [Answer(_SEARCH, "x"), Answer(Command(speech="Зелёный чай."), "x")]
    ports, _ = _run(addressed, _ref(WAKE_CONTESTED), answers)
    assert [(r.tool, r.args[kn.PERSON_ARG]) for r in ports.requests] == [("memory_search", SASHA_ID)]
    assert not any("не уверен чей" in text for text in ports.said)


def test_n213_model_sees_author_and_initiator_separately():
    """В окне модели: фраза — автор «Саша», ``confirmed`` 0.956, текст; инициатор — спорный знакомый голос;
    чья память — Саши; один ли человек — неизвестно (вейк спорный). ``speaker`` снимка — автор."""
    ports, _ = _run(_n213(), _ref(WAKE_CONTESTED), [Answer(Command(speech="Сейчас."), "x")])
    section = _section(ports)
    assert section["phrases"] == [{"author": "Саша", "speaker_id": SASHA_ID, "certainty": "confirmed", "score": 0.956,
                                   "epithet": "Собеседник", "text": TEA, "age_s": 7.5}]
    assert section["initiator"] == {"who": kn.CONTESTED_SPEAKER_LABEL, "certainty": "contested"}
    assert section["same_person"] is None and section["memory_of"] == "Саша"
    context = ports.asked[0][-1]["content"]
    assert '<speaker>{"name":"Саша","name_state":"confirmed"}</speaker>' in context
    assert context.endswith(f"[говорит: Саша] {TEA}")


def test_n213_memory_save_from_the_phrase_goes_to_its_author():
    """Факт из подхваченной фразы Саши пишется Саше: спорный собеседник сессии — не «смена собеседника»."""
    ports, _ = _run(_n213(), _ref(WAKE_CONTESTED), [Answer(_SAVE, "x")])
    assert [(r.tool, r.args[kn.PERSON_ARG]) for r in ports.requests] == [("memory_save", SASHA_ID)]


# -- разные люди ------------------------------------------------------------------------------------------------


def test_author_sasha_initiator_boris_memory_is_sashas_and_model_sees_both():
    """Саша попросил без вейка, Борис уверенно сказал «Робот» (просит за Сашу): память — Саши (его просьба), модель
    видит обоих и ``same_person=False``; данные Бориса не подставляются."""
    boris = SpeakerRef(person_ref=BORIS_ID, name="Борис", name_state="confirmed")
    ports, _ = _run(_n213(), boris, [Answer(_SEARCH, "x"), Answer(Command(speech="Саша, ты пьёшь зелёный."), "x")])
    assert [r.args[kn.PERSON_ARG] for r in ports.requests] == [SASHA_ID]
    section = _section(ports)
    assert section["initiator"] == {"who": "Борис", "certainty": "confirmed"}
    assert section["same_person"] is False and section["memory_of"] == "Саша"


def test_author_boris_initiator_sasha_never_gets_sashas_memory():
    """Обратный случай: просьба Бориса, «Робот» — уверенный Саша. Память — Бориса, не Саши."""
    addressed = _pickup([("поищи что я говорил про спартак", 343.0, 4.0, BORIS_PHRASE, None, ())])
    sasha = SpeakerRef(person_ref=SASHA_ID, name="Саша", name_state="confirmed")
    ports, _ = _run(addressed, sasha, [Answer(_SEARCH, "x"), Answer(Command(speech="Спартак."), "x")])
    assert [r.args[kn.PERSON_ARG] for r in ports.requests] == [BORIS_ID]


def test_unknown_author_does_not_get_the_initiators_memory_without_a_sign():
    """Фразу сказал незнакомый голос, «Робот» — уверенный Саша, признака «это Саша» нет (ни голоса, ни лица, ни
    направления): память Саши незнакомцу не уходит — отказ кодом, владельцу запроса нет."""
    addressed = _pickup([("поищи что я говорил про чай", 343.0, 4.0, UNKNOWN_PHRASE, None, ())])
    sasha = SpeakerRef(person_ref=SASHA_ID, name="Саша", name_state="confirmed")
    ports, _ = _run(addressed, sasha, [Answer(_SEARCH, "x"), Answer(Command(speech="Не знаю."), "x")])
    assert ports.requests == []
    assert _section(ports)["memory_of"] is None


def test_silent_author_with_a_sign_keeps_the_initiators_rights():
    """Биометрия фразы не пришла, но автор — тот же человек по признаку кода (лицо Саши в кадре во время вейка,
    ``same_as_caller``): права — инициатора, как было до 05.11."""
    addressed = _pickup([("поищи что я говорил про чай", 343.0, 4.0, {}, 30.0, ("саша",))],
                        wake_faces=("саша",), wake_doa=35.0)
    assert addressed.picked[0]["same_as_caller"] is True and addressed.picked[0]["speaker_id"] == "unknown"
    sasha = SpeakerRef(person_ref=SASHA_ID, name="Саша", name_state="confirmed")
    ports, _ = _run(addressed, sasha, [Answer(_SEARCH, "x"), Answer(Command(speech="Зелёный."), "x")])
    assert [r.args[kn.PERSON_ARG] for r in ports.requests] == [SASHA_ID]
    assert _section(ports)["same_person"] is True


def test_tentative_author_and_sure_initiator_bind_nobody():
    """Голос фразы — лишь гипотеза «Саша» (без лица), «Робот» — уверенный Борис. ``same_as_caller`` у фразы есть
    (голос длинной фразы узнан), но это знание об авторе, а не признак «просьба Бориса»: память ничья."""
    tentative = {"is_known": True, "speaker_id": SASHA_ID, "name": None, "confidence": 0.78,
                 "tentative_name": "Саша", "tentative_kind": "single", "utterance_id": "u-t"}
    addressed = _pickup([(TEA, 343.0, 4.0, tentative, None, ())])
    assert addressed.picked[0]["same_as_caller"] is True
    boris = SpeakerRef(person_ref=BORIS_ID, name="Борис", name_state="confirmed")
    ports, _ = _run(addressed, boris, [Answer(_SEARCH, "x"), Answer(Command(speech="Не скажу."), "x")])
    assert ports.requests == []
    section = _section(ports)
    assert section["phrases"][0]["certainty"] == "tentative" and section["memory_of"] is None


def test_two_authors_in_one_pickup_bind_nobody():
    """Подхват унёс фразы Бориса и незнакомца: чей запрос — не решено, память ничья (``voice_unconfirmed``)."""
    addressed = _pickup([("закажи пиццу", 340.0, 2.0, BORIS_PHRASE, None, ()),
                         ("не шумите тут", 344.0, 1.5, UNKNOWN_PHRASE, None, ())])
    sasha = SpeakerRef(person_ref=SASHA_ID, name="Саша", name_state="confirmed")
    speaker = turn_speaker(addressed.picked, sasha)
    assert speaker.person_ref is None and speaker.name_state == "contested"
    ports, _ = _run(addressed, sasha, [Answer(_SEARCH, "x"), Answer(Command(speech="Не скажу."), "x")])
    assert ports.requests == []


# -- правило модуля ---------------------------------------------------------------------------------------------


def test_phrase_certainty_words():
    assert phrase_certainty({"identified": True, "name": "Саша"}) == "confirmed"
    assert phrase_certainty({"identified": False, "bio": "known"}) == "nameless"
    assert phrase_certainty({"identified": False, "bio": "known", "candidate": "Саша"}) == "tentative"
    assert phrase_certainty({"identified": False, "bio": "contested"}) == "contested"
    assert phrase_certainty({"identified": False, "bio": "unknown_voice"}) == "unknown"
    assert phrase_certainty({"identified": False, "bio": "inconclusive"}) == "inconclusive"
    assert phrase_certainty({"identified": False, "bio": "none"}) == "inconclusive"


def test_nameless_author_gets_rights_with_its_own_profile():
    """Свой профиль без имени (кличка, #3858) — автор с правами: факт пишется в его профиль, имени нет."""
    nameless = {"is_known": True, "speaker_id": "spk-guest-3", "name": None, "epithet": "Гость", "utterance_id": "u-n"}
    addressed = _pickup([("запомни я люблю чай", 343.0, 4.0, nameless, None, ())])
    speaker = turn_speaker(addressed.picked, _ref(WAKE_CONTESTED))
    assert (speaker.person_ref, speaker.name, speaker.name_state) == ("spk-guest-3", None, "unknown")
    section = pickup_section(addressed.picked, [kn.NAMELESS_SPEAKER_LABEL], _ref(WAKE_CONTESTED), speaker)
    assert section["phrases"][0]["certainty"] == "nameless"
    assert section["memory_of"] == kn.NAMELESS_SPEAKER_LABEL


def test_not_a_pickup_keeps_the_session_speaker():
    sasha = SpeakerRef(person_ref=SASHA_ID, name="Саша", name_state="confirmed")
    assert pickup_speaker(Addressed("привет", VOICE), sasha) is None
    assert rights_speaker(Addressed("привет", VOICE), sasha) is sasha
    assert rights_speaker(_n213(), _ref(WAKE_CONTESTED)).person_ref == SASHA_ID
    assert pickup_section((), (), sasha, sasha) == {}
