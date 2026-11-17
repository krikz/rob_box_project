"""#4141 (хвосты #3993 после PR #4121): на «ты меня узнал?» код отвечает по голосу ЭТОЙ фразы, а короткая фраза не
сбрасывает узнанного собеседника.

Два риска по коду и зонду ревьюера PR #4121:

1. Короткий вопрос (речи меньше ``identify_min_voiced_sec``) - ``speaker_id_node`` публикует ``inconclusive``
   (``too_short_for_biometry``), а ``dialog_node._on_speaker`` через ``speaker_ref`` делал узнанного Сашу незнакомцем:
   код твёрдо говорил ему «По голосу пока не узнаю.». Контракт #2863 (``speaker_id_node._inconclusive_reason``):
   «не смог оценить» - не «это другой», потребители узнанного не сбрасывают. Теперь решает ``speaker_ref``: прежний
   собеседник.
2. Ответ брал собеседника сессии - голос ПРОШЛОЙ фразы. Опоздавший SpeakerResult новой фразы (биометрия и STT не
   упорядочены, у профиля из фразы перед публикацией ещё запись в БД) дал бы новому голосу «Узнал — ты Саша.». Теперь
   ход ждёт SpeakerResult своей фразы по её ``utterance_id`` (тот же хеш звука у STT и биометрии, ADR-0131) не дольше
   ``RECOGNIZED_VOICE_WAIT_S``; не пришёл - «не уверен», а не прошлый собеседник.

Хост здесь - :class:`Biometry`: ``hear`` делает то же, что ``dialog_node._on_speaker`` (реестр по id + собеседник
сессии через ``speaker_ref``), ``resolve`` - как ``UtteranceSpeakerRegistry.resolve`` (ждёт не дольше окна). Живой
реестр и нода - ``rob_box_voice/test/unit/node/test_dialog_node.py`` (``…_4141``).
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import BACKLOG_PICKUP, VOICE, Addressed, decide
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.command import Command
from rob_box_dialog.memory import memory_owner, profiled_person
from rob_box_dialog.session import Session
from rob_box_dialog.speaker_adapter import speaker_ref
from rob_box_dialog.turn import Answer, TurnEngine

SASHA = {"is_known": True, "speaker_id": "spk-sasha", "name": "Саша", "confidence": 0.93}
BORIS = {"is_known": True, "speaker_id": "spk-boris", "name": "Борис", "confidence": 0.91}
NEW_VOICE = {"is_known": False, "candidate_name": "Саша", "candidate_score": 0.41}
#: Короткая фраза: ``speaker_id_node._inconclusive_reason`` - речи меньше ``identify_min_voiced_sec`` (1.0 с).
TOO_SHORT = {"is_known": False, "inconclusive": True, "reason": "too_short_for_biometry"}
ENROLLED_NOW = {"is_known": True, "speaker_id": "spk-guest", "name": None, "epithet": "Гость", "source": "enroll"}

YES_SASHA = "Узнал — ты Саша."
YES_BORIS = "Узнал — ты Борис."
NOT_KNOWN = "По голосу пока не узнаю."
UNHEARD = "Пока не уверен, кто ты: голос этой фразы не успел разобрать."
ASK = "ты меня узнал?"


class Biometry:
    """Хост биометрии для хода: ``hear`` - ``dialog_node._on_speaker`` (реестр по id фразы + собеседник сессии),
    ``resolve`` - ``UtteranceSpeakerRegistry.resolve`` (ждёт результат фразы не дольше окна). ``log`` - общий журнал
    событий с портами хода: порядок «стадия - ожидание голоса - речь - итог»."""

    def __init__(self, session, log):
        self.session, self.log, self.results = session, log, {}

    def hear(self, uid, result, now=1.0):
        payload = {**result, "utterance_id": uid}
        self.results[uid] = payload
        self.session.set_speaker(speaker_ref(payload, now=now, prev=self.session.snapshot.speaker))
        self.log.append(("speaker", uid))

    async def resolve(self, uid, wait_s):
        self.log.append(("wait_voice", uid, wait_s))
        if not uid:  # как у реестра: без id фразы ждать нечего
            return None
        loop = asyncio.get_running_loop()
        deadline = loop.time() + wait_s
        while uid not in self.results and loop.time() < deadline:
            await asyncio.sleep(0.005)
        return self.results.get(uid)


class Ports:
    """Транспорт хода: ответы LLM по очереди (пусто - вызывать модель нельзя), события - в общий журнал."""

    def __init__(self, log, answers=()):
        self.log, self.answers, self.asked, self.said = log, list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return []

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        self.log.append(("say", text))
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, *args, **kwargs):
        self.log.append(("stage", stage, status))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


@pytest.fixture()
def stand():
    log = []
    session = Session("personality")
    bio = Biometry(session, log)
    ports = Ports(log)
    engine = TurnEngine(session, ports, system_prompt="Ты робот.", phrase_voice=bio.resolve)
    return engine, bio, ports, log


def _turn(engine, uid, text=ASK, then=()):
    """Ход фразы ``uid``; ``then`` - ``(через сколько с, вызов)`` на петле хода, пока он идёт (биометрия опоздала)."""

    async def run():
        loop = asyncio.get_running_loop()
        for delay, call in then:
            loop.call_later(delay, call)
        return await engine.run(Addressed(text, VOICE, utterance_id=uid))

    return asyncio.run(run())


# -- 1. короткая фраза не сбрасывает узнанного (#2863) ---------------------------------------------------------


def test_inconclusive_phrase_keeps_the_recognized_speaker():
    """Одно место решения - ``speaker_ref``: ``inconclusive`` при узнанном собеседнике - он же (с прежним ``since``);
    без прежнего - незнакомец, как раньше."""
    sasha = speaker_ref({**SASHA, "utterance_id": "u1"}, now=5.0)
    held = speaker_ref({**TOO_SHORT, "utterance_id": "u2"}, now=9.0, prev=sasha)
    assert held == sasha and held.since == 5.0
    assert speaker_ref(TOO_SHORT, now=9.0).name_state == "unknown"
    assert speaker_ref({**NEW_VOICE, "utterance_id": "u3"}, now=9.0, prev=sasha).person_ref is None, \
        "оценённый чужой голос - смена собеседника, не удержание"


def test_short_phrase_keeps_the_memory_owner(stand):
    """Следствие #2863 для памяти и профиля: короткая фраза Саши - всё ещё Саша (раньше владелец памяти пропадал)."""
    engine, bio, _ports, _log = stand
    bio.hear("u1", SASHA)
    bio.hear("u2", TOO_SHORT)
    snap = engine.session.snapshot
    assert (memory_owner(snap), profiled_person(snap.speaker)) == ("spk-sasha", "spk-sasha")


def test_confirmed_then_short_question_is_still_recognized(stand):
    """Зонд ревьюера PR #4121: Саша подтверждён → «ты меня узнал?» короче 1 с речи (``inconclusive``) → раньше
    «По голосу пока не узнаю.», теперь «Узнал — ты Саша.». Модель не зовётся."""
    engine, bio, ports, _log = stand
    bio.hear("u1", SASHA)
    bio.hear("u2", TOO_SHORT)
    agg = _turn(engine, "u2")
    assert ports.said == [YES_SASHA] and ports.asked == []
    assert (agg.stage, agg.status, agg.llm_calls) == ("done", "finished", 0)


def test_short_result_arriving_while_the_turn_waits_keeps_sasha(stand):
    """STT раньше биометрии: ход уже идёт, ``inconclusive`` этой фразы приходит во время ожидания - всё равно Саша."""
    engine, bio, ports, _log = stand
    bio.hear("u1", SASHA)
    _turn(engine, "u2", then=[(0.05, lambda: bio.hear("u2", TOO_SHORT))])
    assert ports.said == [YES_SASHA]


def test_short_question_after_enrollment_is_not_recognized(stand):
    """Удержан профиль, заведённый из прошлой фразы (``first_heard``): узнавать голос было не по чему - «не узнаю»,
    а не «голос твой помню» (ревью PR #4121 держится и при удержании)."""
    engine, bio, ports, _log = stand
    bio.hear("u1", ENROLLED_NOW)
    bio.hear("u2", TOO_SHORT)
    _turn(engine, "u2")
    assert ports.said == [NOT_KNOWN]


# -- 2. ответ по SpeakerResult ЭТОЙ фразы -----------------------------------------------------------------------


def test_new_voice_with_a_late_result_hears_not_sure_not_sashas_name(stand, monkeypatch):
    """Саша говорил раньше; вопрос задаёт новый голос, его SpeakerResult опоздал дальше окна - «не уверен», без имени
    Саши (раньше - «Узнал — ты Саша.» по собеседнику сессии)."""
    monkeypatch.setattr(kn, "RECOGNIZED_VOICE_WAIT_S", 0.1)
    engine, bio, ports, log = stand
    bio.hear("u1", SASHA)
    _turn(engine, "u2")
    assert engine.session.snapshot.speaker.name == "Саша", "собеседник сессии - всё ещё Саша"
    assert ports.said == [UNHEARD] and ports.asked == []
    assert "Саш" not in ports.said[0]
    assert ("wait_voice", "u2", 0.1) in log


def test_result_of_this_phrase_arriving_during_the_wait_decides(stand):
    """Ход начался при собеседнике Саше (снимок хода), SpeakerResult фразы пришёл во время ожидания - Борис: ответ по
    нему. Раньше код ответил бы по снимку - «Узнал — ты Саша.» Борису."""
    engine, bio, ports, _log = stand
    bio.hear("u1", SASHA)
    _turn(engine, "u2", then=[(0.05, lambda: bio.hear("u2", BORIS))])
    assert ports.said == [YES_BORIS]


@pytest.mark.parametrize("result, said", [(BORIS, YES_BORIS), (NEW_VOICE, NOT_KNOWN)], ids=["boris", "new_voice"])
def test_result_of_this_phrase_before_the_answer_decides_even_if_another_phrase_came_later(stand, result, said):
    """SpeakerResult фразы пришёл до хода, а за ним - результат чужой фоновой фразы (Саша сменил собеседника сессии):
    ответ - по своей фразе (по ``utterance_id``), а не по последнему голосу."""
    engine, bio, ports, _log = stand
    bio.hear("u2", result)
    bio.hear("u-bg", SASHA)
    _turn(engine, "u2")
    assert ports.said == [said]


def test_phrase_without_id_is_not_sure(stand):
    """У реплики нет id фразы (не голос с микрофона) - голоса нет, «не уверен», а не собеседник сессии."""
    engine, bio, ports, _log = stand
    bio.hear("u1", SASHA)
    _turn(engine, "")
    assert ports.said == [UNHEARD]


def test_host_without_biometry_is_not_sure():
    """Хост не дал голос фразы (``phrase_voice`` по умолчанию): «не уверен», и модель не зовётся."""
    log = []
    session = Session("personality")
    session.set_speaker(speaker_ref(SASHA, now=1.0))
    ports = Ports(log)
    asyncio.run(TurnEngine(session, ports, system_prompt="Ты робот.").run(Addressed(ASK, VOICE, utterance_id="u1")))
    assert ports.said == [UNHEARD] and ports.asked == []


# -- порядок событий хода ---------------------------------------------------------------------------------------


def test_turn_order_addressed_then_wait_then_speech_then_done(stand):
    """Ожидание голоса - внутри хода, после «addressed» и до речи; итог хода - после речи. SpeakerResult, пришедший
    во время ожидания, ход не рвёт."""
    engine, bio, _ports, log = stand
    bio.hear("u1", SASHA)
    _turn(engine, "u2", then=[(0.05, lambda: bio.hear("u2", TOO_SHORT))])
    order = [e[:2] for e in log if e[0] != "stage" or e[1] in ("addressed", "done")]
    assert order == [("speaker", "u1"), ("stage", "addressed"), ("wait_voice", "u2"), ("speaker", "u2"),
                     ("say", YES_SASHA), ("stage", "done")]


def test_other_turns_do_not_wait_for_the_voice(stand):
    """Только вопрос узнавания ждёт голос фразы: обычный ход идёт к модели сразу."""
    engine, bio, ports, log = stand
    ports.answers.append(Answer(Command(speech="Включаю."), "deepseek"))
    bio.hear("u1", SASHA)
    _turn(engine, "u2", text="расскажи анекдот")
    assert ports.said == ["Включаю."] and not [e for e in log if e[0] == "wait_voice"]


def test_stop_during_the_wait_drops_the_answer(stand, monkeypatch):
    """Стоп (эпоха + 1) пока ход ждёт голос: ответ старой эпохи не звучит (``stale_epoch``), как любой ответ хода."""
    monkeypatch.setattr(kn, "RECOGNIZED_VOICE_WAIT_S", 0.1)
    engine, bio, ports, _log = stand
    bio.hear("u1", SASHA)
    _turn(engine, "u2", then=[(0.02, engine.session.replace_epoch)])
    assert ports.said == []


# -- id фразы хода (адресация) ----------------------------------------------------------------------------------


def test_voice_turn_carries_the_id_of_its_phrase_and_pickup_does_not():
    """``address.decide``: ход из самой фразы помнит её ``utterance_id``; подхват бэклога голым «робот» - нет (текст
    хода - чужие фразы бэклога, и вопрос узнавания там кодом не отвечается)."""
    heard = Heard(text="Робот, ты меня узнал?", t=100.0, utterance_id="u9")
    out = decide(heard, Backlog(), ["робот"])
    assert isinstance(out, Addressed) and (out.source, out.utterance_id) == (VOICE, "u9")
    backlog = Backlog()
    backlog.add(Heard(text="ты меня узнал", t=100.0, utterance_id="u10"))
    picked = decide(Heard(text="Робот", t=101.0, utterance_id="u11"), backlog, ["робот"])
    assert isinstance(picked, Addressed) and (picked.source, picked.utterance_id) == (BACKLOG_PICKUP, "")
