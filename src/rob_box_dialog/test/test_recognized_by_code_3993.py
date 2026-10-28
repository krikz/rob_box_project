"""#3993 (б): на «ты меня узнал?» отвечает КОД по собеседнику снимка, а не модель (решение Шифу 26.10, ADR-0148).

Марафон 23.10, акт 2b: голос узнан (0.944), имя скрыто спором - модель сказала «Голос я пока не запомнил, но имя
твоё знаю — Саша»: неправда и о голосе, и об имени. Вопрос «ты Саша?» в v2 не задаём (решение Шифу 26.10). Теперь
реплика, в которой нет ничего, кроме вопроса узнавания (глагол и дополнение где угодно, пояснения «я тот же», «мы
недавно уже говорили» - тоже часть вопроса), закрывается фразой кода по ``name_state`` снимка: узнан с именем /
знакомый, не уверен чей (без имени кандидата) / профиль без имени / не узнаю. LLM не зовётся. Ревью PR #4121: на
настоящих фразах акта 2b прежний матч «реплика целиком = глагол + дополнение» не срабатывал ни разу - они здесь
(``ACT2B``), как положительные и как честно отрицательные. Круг 1 ревью PR #4121: профиль, заведённый из этой же
фразы (#3858, ``source=enroll``), - голос слышен впервые, ответ «не узнаю» (``ENROLLED_NOW``); утверждения и обрывки
STT («ты же меня знаешь», «знаешь, что меня…») - ход модели.
"""

import asyncio

import pytest

from rob_box_core.speaker_wire import SOURCE_ENROLL
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.speaker_adapter import asks_recognized, recognition_reply, recognition_state, speaker_ref
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import strip_wake_word

#: Акт 2b, 21:30:14: «Саша» 0.944, конкурент «Борис» - имя скрыто спором.
CONTESTED = {"is_known": True, "speaker_id": "spk-sasha", "name": None, "confidence": 0.944, "epithet": "Спутник",
             "tentative_conf": 0.944, "tentative_kind": "contested"}
TENTATIVE = {"is_known": True, "speaker_id": "spk-sasha", "name": None, "confidence": 0.78,
             "tentative_name": "Саша", "tentative_kind": "single"}
CONFIRMED = {"is_known": True, "speaker_id": "spk-sasha", "name": "Саша", "confidence": 0.944}
NAMELESS = {"is_known": True, "speaker_id": "spk-guest-3", "name": None, "epithet": "Гость"}
#: Ревью PR #4121 (круг 1, блокер): незнакомый голос, фраза ≥ 3 с - speaker_id_node (#3858, в бою
#: ``unnamed_profiles_enabled: true``) заводит профиль без имени ИЗ ЭТОЙ ЖЕ фразы и публикует его так
#: (``_publish_result(enrolled, source=SOURCE_ENROLL)``). Реплика n702 23.10 - «речь=3.96s».
ENROLLED_NOW = {"is_known": True, "speaker_id": "spk-guest-4", "name": None, "epithet": "Гость", "confidence": 0.0,
                "source": SOURCE_ENROLL}
UNKNOWN = {"is_known": False}

YES_SASHA = "Узнал — ты Саша."
UNSURE = "Голос знакомый, но не уверен, чей."
NOT_KNOWN = "По голосу пока не узнаю."
NAMELESS_SAID = "Голос твой помню, а имени не знаю."

#: Реплики рукописного акта 2b (``.github/e2e/scenarios/night/night_marathon_act2b_identity_question_v1.json``, текст
#: шага как есть; ход получает его без вейка - ``strip_wake_word``, как ``address.decide``) и отвечает ли на них код.
#: Сверка с файлом сценария - ``tests/unit/e2e_scripts/test_issue_3993_act2b_recognized_by_code.py``.
ACT2B = (
    ("n701", "Робот, здравствуй, я Саша, чиню технику, сегодня перебираю колёса.", False),
    ("n702", "Робот, ты меня узнал по голосу, я тот же, кто заходил недавно?", True),
    ("n703", "Робот, а что ты про меня помнишь с прошлого раза, какой чай я обычно пью?", False),  # память - модели
    ("n704", "Робот, ты ещё здесь, помнишь меня? Мы недавно уже говорили.", True),
    ("n705", "Робот, привет, я Борис, заглянул проверить проводку, в щитке у окна опять что-то потрескивает.", False),
    ("n706", "Робот, а меня ты по голосу узнаёшь? Скажи, кто я такой и откуда ты меня знаешь.", True),
    ("n707", "Робот, а что ты про меня помнишь, за какую футбольную команду я болею?", False),  # память - модели
    # выбор из имён («я или Борис?») - не вопрос «узнал ли»: ход модели со снимком (name_state), кодом не покрыт
    ("n708", "Робот, а сейчас с тобой кто говорит, я или Борис? Ответь по голосу.", False),
    ("n709", "Робот, я заскочила на секунду, представляться пока не буду.", False),
    ("n710", "Робот, а ты меня по голосу узнаёшь? Кто я такая, как думаешь?", True),
)
ACT2B_BY_CODE = [(label, text) for label, text, by_code in ACT2B if by_code]


def _ref(result):
    return speaker_ref(result, now=1.0)


# -- вопрос целиком ---------------------------------------------------------------------------------------------


@pytest.mark.parametrize("text", ["ты меня узнал?", "Ты меня узнаёшь?", "узнал меня?", "ну что, узнаёшь меня?",
                                  "ты меня не узнал?", "ты узнал мой голос?", "узнал, кто я?",
                                  "ты меня по голосу узнал?", "а ты меня узнаешь ли?"])
def test_whole_recognition_question(text):
    assert asks_recognized(text)


@pytest.mark.parametrize("text", ["помнишь меня?", "а ты меня помнишь", "знаешь, кто я?", "вы меня узнаёте?",
                                  "ты меня узнал или нет?", "ты меня узнал, да?", "ты меня запомнил?",
                                  "ты меня узнал по голосу я тот же кто заходил недавно"])  # последняя - STT без знаков
def test_recognition_question_vocabulary(text):
    """Словарь ревью PR #4121: «помнишь меня?», «знаешь, кто я?», «вы меня узнаёте?», «…или нет?», «…, да?»."""
    assert asks_recognized(text)


@pytest.mark.parametrize("text", ["ты меня не помнишь?", "не узнал меня?", "ты не узнал меня теперь",
                                  "ты меня узнаешь, да?", "а ты знаешь меня?", "ты знаешь, кто я такой?"])
def test_negated_and_tagged_recognition_question(text):
    """Круг 1 ревью PR #4121: отрицание глагола узнавания и переспрос в конце - всё ещё вопрос узнавания."""
    assert asks_recognized(text)


@pytest.mark.parametrize("text", ["ты меня узнал? включи музыку", "узнай погоду", "что ты обо мне помнишь?",
                                  "ты узнал, что Саша любит чай?", "как меня зовут?", "узнал", "ты меня слышишь?",
                                  "узнал узнал меня", "", "откуда ты меня знаешь?", "что ты про меня помнишь?",
                                  "ты знаешь, кто такой Гагарин?", "помнишь, что я люблю зелёный чай?",
                                  "мы уже знакомы?", "ты меня узнал? кстати, какая погода"])
def test_not_the_recognition_question(text):
    assert not asks_recognized(text)


@pytest.mark.parametrize("text", ["ты меня не знаешь", "да ты меня знаешь", "ну ты меня знаешь", "ты же меня знаешь",
                                  "знаешь что меня", "ты помнишь что меня", "ты меня не помнишь и не запомнил",
                                  "ты знаешь что у меня", "ты меня или нет узнал"])
def test_statement_or_stt_fragment_goes_to_the_model(text):
    """Круг 1 ревью PR #4121 (зонд probe_live): утверждение («да/ну/же ты меня знаешь», «ты меня не знаешь») и обрывок
    STT («знаешь, что меня…») - не вопрос «узнал ли»: «Узнал — ты Саша.» на них звучит странно, ход модели, как до
    #3993. «Да» - переспрос только в конце, «что» - только в «ну что», «не знаешь» - не глагол узнавания."""
    assert not asks_recognized(text)


@pytest.mark.parametrize("label, text, by_code", ACT2B, ids=[a[0] for a in ACT2B])
def test_act2b_lines_recognition_by_code(label, text, by_code):
    """Ревью PR #4121 (блокер): реплики акта 2b, ради которых заведена карточка, - вопрос узнавания (n702, n704,
    n706, n710) отвечает код; память (n703, n707), самопредставление и выбор из имён (n708) - модель."""
    assert asks_recognized(strip_wake_word(text)) is by_code


# -- ответ по собеседнику снимка -------------------------------------------------------------------------------


def test_reply_by_name_state():
    assert recognition_reply("ты меня узнал?", _ref(CONFIRMED)) == ("confirmed", YES_SASHA)
    assert recognition_reply("ты меня узнал?", _ref(CONTESTED)) == ("contested", UNSURE)
    assert recognition_reply("ты меня узнал?", _ref(TENTATIVE)) == ("tentative", UNSURE)
    assert recognition_reply("ты меня узнал?", _ref(NAMELESS)) == ("nameless", NAMELESS_SAID)
    assert recognition_reply("ты меня узнал?", _ref(UNKNOWN)) == ("unknown", NOT_KNOWN)
    assert recognition_reply("ты меня узнал?", None)[0] == "unknown"
    assert recognition_reply("включи музыку", _ref(CONFIRMED)) is None


@pytest.mark.parametrize("label, text", [("short", "ты меня узнал?")] + ACT2B_BY_CODE,
                         ids=["short"] + [a[0] for a in ACT2B_BY_CODE])
def test_voice_enrolled_from_this_very_phrase_is_not_recognized(label, text):
    """Круг 1 ревью PR #4121 (блокер): профиль без имени, заведённый из ЭТОЙ фразы (``source=enroll``), - голос слышен
    впервые: «Голос твой помню» ему - неправда (обратная сторона лжи #3993). Ответ - «по голосу пока не узнаю», как
    незнакомцу; профиль для памяти при этом свой (``person_ref`` есть)."""
    ref = _ref(ENROLLED_NOW)
    assert (ref.person_ref, ref.first_heard) == ("spk-guest-4", True)
    assert recognition_reply(strip_wake_word(text), ref) == ("unknown", NOT_KNOWN)


def test_next_phrase_of_the_enrolled_voice_is_the_nameless_profile():
    """Следующая фраза того же голоса - обычное узнавание (``source`` нет): робот его уже слышал - «голос помню»."""
    first = _ref(ENROLLED_NOW)
    again = speaker_ref({**ENROLLED_NOW, "confidence": 0.86, "source": None}, now=2.0, prev=first)
    assert (again.person_ref, again.first_heard, again.since) == ("spk-guest-4", False, first.since)
    assert recognition_reply("ты меня узнал?", again) == ("nameless", NAMELESS_SAID)


@pytest.mark.parametrize("result", [CONTESTED, TENTATIVE])
def test_unconfirmed_voice_gets_no_candidate_name_and_no_identity_question(result):
    """Решение Шифу 26.10: «ты Саша?» не спрашиваем; имя кандидата вслух - утечка (n210 must_not_say)."""
    _state, said = recognition_reply("ты меня узнал?", _ref(result))
    assert "Саша" not in said and "?" not in said
    assert "не запомнил" not in said, "голос узнан - «не запомнил» неправда (акт 2b 23.10)"


@pytest.mark.parametrize("result", [CONFIRMED, CONTESTED, TENTATIVE, NAMELESS, UNKNOWN])
def test_reply_to_a_negative_question_is_not_a_bare_yes_or_no(result):
    """Ревью PR #4121: на «ты меня НЕ узнал?» «Да, …»/«Нет, …» двусмысленны - фраза кода без них в начале."""
    _state, said = recognition_reply("ты меня не узнал?", _ref(result))
    assert not said.lower().startswith(("да", "нет")), said


def test_confirmed_without_name_is_not_a_hollow_yes():
    """Битый снимок «confirmed» без имени не даёт «Да, , узнал.»."""
    assert recognition_state(SpeakerRef(person_ref="spk-x", name=None, name_state="confirmed")) == "nameless"
    assert recognition_state(SpeakerRef(person_ref=None, name=None, name_state="confirmed")) == "unknown"


def test_every_name_state_has_a_reply_phrase():
    for state in (*kn.VOICE_UNCONFIRMED_STATES, "confirmed", "nameless", "unknown"):
        assert phrases.text(kn.RECOGNIZED_REPLIES[state], name="Саша"), state
    assert set(kn.RECOGNIZED_REPLIES.values()) <= set(kn.PHRASE_KEYS)


# -- ход целиком -------------------------------------------------------------------------------------------------


class Ports:
    """Транспорт хода: ответы LLM по очереди (пусто - вызывать модель нельзя)."""

    def __init__(self, answers=()):
        self.answers, self.asked, self.said = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return []

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _engine(result, answers=(), agent="personality"):
    session = Session(agent)
    if result is not None:
        session.set_speaker(_ref(result))
    ports = Ports(answers)
    return TurnEngine(session, ports, system_prompt="Ты робот."), ports, session


@pytest.mark.parametrize("result, said", [(CONTESTED, UNSURE), (CONFIRMED, YES_SASHA)])
def test_recognition_question_is_answered_by_code_without_llm(result, said):
    engine, ports, _ = _engine(result)
    agg = asyncio.run(engine.run(Addressed("ты меня узнал?", VOICE)))
    assert ports.asked == [], "модель не зовётся"
    assert ports.said == [said]
    assert (agg.stage, agg.status, agg.llm_calls) == ("done", "finished", 0)


@pytest.mark.parametrize("label, text", ACT2B_BY_CODE, ids=[a[0] for a in ACT2B_BY_CODE])
@pytest.mark.parametrize("result, said", [(CONTESTED, UNSURE), (CONFIRMED, YES_SASHA), (NAMELESS, NAMELESS_SAID),
                                          (ENROLLED_NOW, NOT_KNOWN)],
                         ids=["contested", "confirmed", "nameless", "enrolled_now"])
def test_act2b_question_is_answered_by_code_without_llm(label, text, result, said):
    """Акт 2b через ход: реплика без вейка (как у ``address.decide``) - 0 вызовов LLM, фраза кода по снимку. Спорный
    голос 23.10 (0.944, имя скрыто) - «знакомый, не уверен», а не «голос не запомнил, но имя знаю — Саша»."""
    engine, ports, _ = _engine(result)
    agg = asyncio.run(engine.run(Addressed(strip_wake_word(text), VOICE)))
    assert ports.asked == [], "модель не зовётся"
    assert ports.said == [said]
    assert (agg.stage, agg.status, agg.llm_calls) == ("done", "finished", 0)


def test_act2b_memory_question_still_goes_to_the_model():
    """n703 «что ты про меня помнишь…» - вопрос о фактах, а не «узнал ли»: ход модели (память - её тулом)."""
    engine, ports, _ = _engine(CONFIRMED, [Answer(Command(speech="Ты пьёшь зелёный чай."), "deepseek")])
    asyncio.run(engine.run(Addressed(strip_wake_word(ACT2B[2][1]), VOICE)))
    assert len(ports.asked) == 1 and ports.said == ["Ты пьёшь зелёный чай."]


def test_question_inside_another_request_goes_to_the_model():
    engine, ports, _ = _engine(CONTESTED, [Answer(Command(speech="Включаю."), "deepseek")])
    asyncio.run(engine.run(Addressed("ты меня узнал? включи музыку", VOICE)))
    assert len(ports.asked) == 1 and ports.said == ["Включаю."]


def test_silence_swallows_the_question_too():
    engine, ports, session = _engine(CONFIRMED)
    session.push_scene("Silence", "user")
    asyncio.run(engine.run(Addressed("ты меня узнал?", VOICE)))
    assert ports.asked == [] and ports.said == []


def test_backlog_pickup_is_not_answered_by_the_callers_voice():
    """Подхват бэклога: вопрос мог задать не тот, кто сказал «робот» - отвечает модель по меткам фраз."""
    engine, ports, _ = _engine(CONFIRMED, [Answer(Command(speech="Кто спрашивал?"), "deepseek")])
    asyncio.run(engine.run(Addressed("ты меня узнал?", VOICE, pickup=True)))
    assert len(ports.asked) == 1 and YES_SASHA not in ports.said


def test_telegram_chat_has_no_voice_and_asks_the_model():
    engine, ports, _ = _engine(None, [Answer(Command(speech="Я вижу только текст."), "deepseek")], agent="telegram")
    asyncio.run(engine.run(Addressed("ты меня узнал?", "telegram")))
    assert len(ports.asked) == 1


def test_code_reply_lands_in_the_window_as_robot_speech():
    """Ответ кода - реплика робота в окне (после ``SpeechEvent``), как любая речь хода."""
    engine, ports, _ = _engine(CONTESTED)
    asyncio.run(engine.run(Addressed("ты меня узнал?", VOICE)))
    engine.speech_ended("sp1", len(UNSURE))
    assert [(e.role, e.content) for e in engine.log.window()][-1] == ("assistant", UNSURE)
