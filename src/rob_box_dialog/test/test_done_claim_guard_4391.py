"""Речь о сделанном без тула (#4391 п.2, HIL 04.11; ADR-0148: фразу об успехе строит код по результату тула).

«Удали ещё точку хил-окно» → ``verdict=speech actions=[]``, модель сказала «Готово, хил-окно удалена.» без
``delete_waypoint``. Речь, утверждающая сделанное («удалил», «сохранил точку», «поехал»), без тула, что это делает, -
нарушение ``VIOLATION_DONE_CLAIM``: повтор моста с подсказкой, затем фраза кода. Тот же механизм, что
``music_promise``/``voice_promise``/``search_promise`` (таблица слов в ``knowledge``, сверка в ``words``). Ждёт решения
Шифу: продолжение ``*_promise`` или ``TEMP(ADR-0148)``.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Invalid, ToolCall, explain, parse, unkept_done_claims, validate
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import done_claims

#: Речь модели из HIL 04.11 23:41 UTC дословно (verdict=speech actions=[]).
HIL_LIE = "Готово, хил-окно удалена."
TOOLS = ("delete_waypoint", "clear_waypoints", "list_waypoints", "save_waypoint", "play_animation", "memory_save")


class Ports:
    """Транспорт хода: модель отвечает заготовками, исполнитель всё делает (``done``), речь - в ``said``."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.asked = []
        self.executed = []
        self.said = []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append([t.name for t in tools])
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, call in enumerate(actions):
            self.executed.append((call.tool, dict(call.args)))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, call.tool,
                                    kn.TOOL_CLASSES[call.tool], "done", None, dict(call.args)))
        return out

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


def _run(ports, text):
    ids = iter(f"t{n}" for n in range(1, 100))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return engine


# ---------------------------------------------------------------------------------------------------------------
# 2б. речь о сделанном без тула - нарушение, после повтора говорит код
# ---------------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("text, kinds", [
    (HIL_LIE, ("delete",)),
    ("Удалил точку кухня.", ("delete",)),
    ("Все точки стёрты.", ("delete",)),
    ("Сохранил точку у окна.", ("waypoint_save",)),
    ("Точку запомнил, называется кухня.", ("waypoint_save",)),
    ("Поехал на кухню!", ("motion",)),
    ("Выезжаю к окну.", ("motion",)),
    ("Сделал звук глуше и тише.", ("volume",)),       # #4480, М14 n511: ни одного тула, музыка играла дальше
    ("Приглушил музыку.", ("volume",)),
    ("Убавил громкость.", ("volume",)),
])
def test_done_claim_words(text, kinds):
    assert done_claims(text) == kinds


@pytest.mark.parametrize("text", [
    "Точку хил-окно я уже удалил раньше.",       # было в прошлом ходе
    "Не удалил: такой точки нет.",
    "Удалить точку хил-окно?",
    "Запомнил, тебя зовут Саша.",                # знакомство - речь кода, не точка
    "Музыку выключил.",                          # после кодового стопа - правда (MUSIC_PROMISES)
    "Люблю вкусную еду.",                        # «еду» - не езда
    "Сделаю потише.",                            # обещание, его ловит music_promise, не done_claim
    "Звук я не сделал глуше: не просили.",
    "Сделал бутерброд.",
])
def test_not_a_done_claim(text):
    assert done_claims(text) == ()


def test_hil_lie_without_a_tool_is_a_violation_with_a_hint_and_the_tools_that_would_do_it():
    result = validate(parse({"speech": HIL_LIE}), TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_DONE_CLAIM
    assert "delete_waypoint" in result.all[0].allowed
    assert kn.DONE_CLAIM_HINT in explain(result)


def test_the_claim_with_its_tool_is_not_a_done_claim():
    cmd = parse({"actions": [{"tool": "delete_waypoint", "args": {"name": "хил-окно"}}]})
    assert unkept_done_claims(cmd) == ()
    cmd = Command(speech=HIL_LIE, actions=(ToolCall("delete_waypoint", {"name": "хил-окно"}),))
    assert unkept_done_claims(cmd) == ()


def test_after_the_exhausted_retry_the_code_says_nothing_was_done():
    """Повтор моста не помог - невалидная команда доходит до хода: ложь «удалена» не звучит, звучит фраза кода."""
    ports = Ports([Answer(Invalid(kn.VIOLATION_DONE_CLAIM, "speech утверждает delete без тула"), "deepseek")])
    _run(ports, "Робот, удали точку хил-окно")
    assert ports.executed == []
    assert ports.said == [phrases.text(kn.DONE_CLAIM_PHRASE)] and HIL_LIE not in ports.said
    assert done_claims(ports.said[0]) == ()


def test_the_done_claim_phrase_is_in_the_catalog():
    assert kn.DONE_CLAIM_PHRASE in kn.PHRASE_KEYS and kn.INVALID_COMMAND_PHRASES[kn.VIOLATION_DONE_CLAIM]


VOLUME_LIE = "Сделал звук глуше и тише."
VOLUME_TOOLS = (*TOOLS, "set_volume", "edit_music", "stop_music")


def test_4480_volume_claim_without_a_tool_is_a_violation_with_volume_tools_allowed():
    result = validate(parse({"speech": VOLUME_LIE}), VOLUME_TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_DONE_CLAIM
    assert set(result.all[0].allowed) & set(kn.DONE_CLAIMS["volume"].tools)


def test_4480_volume_claim_with_its_tool_is_not_a_violation():
    for tool in kn.DONE_CLAIMS["volume"].tools:
        cmd = Command(speech=VOLUME_LIE, actions=(ToolCall(tool, {}),))
        assert unkept_done_claims(cmd) == ()


def test_4480_exhausted_retry_does_not_voice_the_volume_lie():
    ports = Ports([Answer(Invalid(kn.VIOLATION_DONE_CLAIM, "speech утверждает volume без тула"), "deepseek")])
    _run(ports, "Робот, что там со звуком")
    assert ports.executed == []
    assert ports.said == [phrases.text(kn.DONE_CLAIM_PHRASE)] and VOLUME_LIE not in ports.said


# ---------------------------------------------------------------------------------------------------------------
# #5007: «говорю медленнее» без set_speed (акт 4 n409, run 44890078495)
# ---------------------------------------------------------------------------------------------------------------

#: Речь модели из n409 дословно (command, actions=[], строки ``[set_speed]`` нет).
SPEED_LIE = "Хорошо, говорю медленнее. Тётя Крыша в коридоре меня и так услышит."
SPEED_TOOLS = (*TOOLS, "set_speed", "set_pitch", "set_volume")


@pytest.mark.parametrize("text", [
    SPEED_LIE,
    "Говорю быстрее.",
    "Теперь я тараторю быстрее.",
    "Я замедлился.",
    "Стал говорить помедленнее.",
])
def test_5007_speed_claim_words(text):
    assert done_claims(text) == ("speed",)


@pytest.mark.parametrize("text", [
    "Заговорю помедленнее.",          # будущее - обещание voice_promise, не утверждение о сделанном
    "Не говорю медленнее: не просили.",
    "Уже говорю медленнее.",          # было раньше, не этот ход
    "Тараторю.",                      # сравнительной в окне нет - не утверждение о смене
    "Говорю о погоде.",
])
def test_5007_not_a_speed_claim(text):
    assert "speed" not in done_claims(text)


def test_5007_speed_claim_without_a_tool_is_a_violation_with_set_speed_allowed():
    result = validate(parse({"speech": SPEED_LIE, "actions": []}), SPEED_TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_DONE_CLAIM
    assert "set_speed" in result.all[0].allowed
    assert "set_speed" in explain(result)


def test_5007_speed_claim_with_its_tool_is_not_a_violation():
    cmd = Command(speech=SPEED_LIE, actions=(ToolCall("set_speed", {"action": "slower"}),))
    assert unkept_done_claims(cmd) == ()


def test_5007_other_tools_do_not_back_the_speed_claim():
    cmd = Command(speech=SPEED_LIE, actions=(ToolCall("set_volume", {}), ToolCall("set_pitch", {})))
    assert unkept_done_claims(cmd) == ("speed",)


def test_5007_exhausted_retry_does_not_voice_the_speed_lie():
    ports = Ports([Answer(Invalid(kn.VIOLATION_DONE_CLAIM, "speech утверждает speed без тула"), "deepseek")])
    # «говори помедленнее» закрывает Tier-1 voice_step до модели: вход без просьбы о темпе
    _run(ports, "Робот, что там с речью")
    assert ports.executed == []
    assert ports.said == [phrases.text(kn.DONE_CLAIM_PHRASE)] and SPEED_LIE not in ports.said


def test_5007_code_speed_phrase_is_the_same_words_and_is_said_by_code_after_the_tool():
    """``speed.slower`` / ``speed.faster`` - те же слова, что ложь n409: таблица их ловит, говорит их код после тула."""
    for key in ("speed.slower", "speed.faster"):
        assert done_claims(phrases.text(key)) == ("speed",)


# ---------------------------------------------------------------------------------------------------------------
# #4541: «так и буду тебя узнавать» без записи человека
# ---------------------------------------------------------------------------------------------------------------

REMEMBER_LIE = "Кирилл, принял! Голос, выходит, детский — так и буду тебя узнавать. Приятно познакомиться."
PERSON_TOOLS = (*TOOLS, "register_speaker", "name_person")


def test_4541_live_phrase_without_a_tool_is_a_violation():
    assert done_claims(REMEMBER_LIE) == ("recognize_person",)
    result = validate(parse({"speech": REMEMBER_LIE, "actions": []}), PERSON_TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_DONE_CLAIM
    assert "register_speaker" in result.all[0].allowed


@pytest.mark.parametrize("tool", kn.PERSON_RECORD_TOOLS)
def test_4541_live_phrase_with_a_person_tool_is_not_a_violation(tool):
    cmd = Command(speech=REMEMBER_LIE, actions=(ToolCall(tool, {"name": "Кирилл"}),))
    assert unkept_done_claims(cmd) == ()


def test_4541_memory_save_does_not_back_the_claim():
    cmd = Command(speech=REMEMBER_LIE, actions=(ToolCall("memory_save", {}),))
    assert unkept_done_claims(cmd) == ("recognize_person",)


@pytest.mark.parametrize("text", ["Запомнил твой голос.", "Лицо твоё запомнила."])
def test_4541_remembered_voice_or_face_without_a_tool_is_a_claim(text):
    assert done_claims(text) == ("remember_person",)


def test_4541_waypoint_save_is_unchanged():
    text = "Запомнил точку кухня."
    assert done_claims(text) == ("waypoint_save",)
    assert unkept_done_claims(Command(speech=text, actions=(ToolCall("save_waypoint", {"name": "кухня"}),))) == ()
    assert unkept_done_claims(Command(speech=text)) == ("waypoint_save",)


@pytest.mark.parametrize("text", [
    "Я тебя уже запомнил.",
    "Ты меня запомнил?",
    "Не запомнил твой голос: слишком тихо.",
    "Я уже запомнила твоё лицо.",
    "Не буду тебя узнавать, пока не представишься.",
])
def test_4541_not_a_claim(text):
    assert done_claims(text) == ()


# ---------------------------------------------------------------------------------------------------------------
# n211 21.11 (run 44875774940): «запомнил двоих» - отчёт о памяти, не заявление о новой записи
# ---------------------------------------------------------------------------------------------------------------

#: Речь модели дословно, второй вызов хода n211_who_do_you_know (voice_e2e_44875774940.log, turn 2f3b3370c63d):
#: «speech утверждает remember_person без тула», код заменил её фразой command.done_claim. В БД акта было ровно
#: Саша и Борис - отчёт правдив, записывать было некого.
N211_REPORT = ("Сегодня по голосу я запомнил двоих: тебя, Саша, и Бориса, твоего друга с пиццей по пятницам. "
               "Дама, что шла мимо, имя называть отказалась, так что её я не запомнил.")
N211_REPORT_FIRST = ("Сегодня по голосу я запомнил двоих. Саша, это ты, и Борис, твой друг с пиццей по пятницам. "
                     "Дама, что шла мимо, имя называть отказалась, так что её я не запомнил.")


@pytest.mark.parametrize("text", [N211_REPORT, N211_REPORT_FIRST])
def test_n211_honest_count_is_not_a_remember_person_claim(text):
    """Речь n211 - честный отчёт, не ложь о записи: ``validate`` принимает её как команду, ``remember_person`` молчит.

    speech рядом с memory_search на голосе по-прежнему ``speech_with_actions`` (query - loud, вне этого фикса), поэтому
    команда со speech и memory_search тут не собирается: ``_check_done_claim`` на такой речи молчит сам, потому что речь
    не звучит. Валидная команда - речь без тула записи."""
    assert done_claims(text) == ()
    cmd = Command(speech=text, actions=(ToolCall("memory_search", {"query": "Борис"}),))
    assert unkept_done_claims(cmd) == ()
    result = validate(parse({"speech": text}), PERSON_TOOLS)
    assert isinstance(result, Command) and result.speech == text


@pytest.mark.parametrize("text", [
    "Запомнил двоих.",
    "Запомнил троих.",
    "Запомнил двух.",
    "Запомнил нескольких.",
    "Запомнил двоих по голосу.",
    "Запомнил по голосу двоих: тебя и Бориса.",
    "По голосу запомнил троих.",
    "Запомнил всех по голосу.",
    "Запомнил никого по голосу.",
    "Запомнил пару голосов.",
])
def test_n211_count_after_verb_is_a_report_not_a_claim(text):
    """Объект «запомнил» - счёт уже записанного (в окне после глагола): remember_person не срабатывает."""
    assert done_claims(text) == ()


@pytest.mark.parametrize("text", [
    "Запомнил твой голос.",
    "Я запомнил твой голос.",
    # голое «запомнил тебя/его/её/вас» правило #4541 не ловит и до фикса: в окне нет основы «голос»/«лиц», а ловить его
    # префиксом нельзя («вас» - начало «василий»). Слова о голосе рядом - ловят, это и закреплено ниже.
    "Я запомнил этот голос.",
    "Запомнил голос Бориса.",
    "Запомнил тебя по голосу.",
    "Запомнил по голосу тебя.",
    "Запомнил по голосу Бориса.",
    "Запомнил Бориса по голосу.",
    "По голосу я запомнил Сашу.",
    "Запомнил человека по голосу.",
    "Запомнил новое лицо.",
    "Лицо твоё запомнила.",
])
def test_n211_person_or_voice_object_stays_a_claim(text):
    """#4541 держится: объект - человек или его голос/лицо, а не счёт. «запомнил тебя по голосу» - тоже запись."""
    assert done_claims(text) == ("remember_person",)


def test_n211_remembered_a_person_is_not_a_claim():
    """«запомнил человека» без слова о голосе или лице записи не утверждает: объекта-записи в окне нет."""
    assert done_claims("Запомнил человека.") == ()
    assert done_claims("Запомнил нового человека.") == ()


# ---------------------------------------------------------------------------------------------------------------
# #5128: «Запомнил, Борис: …» без memory_save — утверждение о записи факта
# ---------------------------------------------------------------------------------------------------------------

#: Речь модели из n206 марафона №3 (22.11): actions=[], voice_facts пусто, год дописан моделью.
N206_LIE = "Запомнил, Борис: ты болеешь за Спартак с девяносто восьмого года и по пятницам приносишь пиццу."


@pytest.mark.parametrize("text", [
    N206_LIE,
    "Запомнил!",
    "Запомнил про тебя: любишь чай.",
    "Записала: Борис болеет за Спартак.",
    "Сохранил, что ты приносишь пиццу.",
    "Я запомнил это.",
])
def test_5128_a_fact_save_without_its_object_is_a_remember_fact_claim(text):
    """Глагол записи факта без объекта «точка»/«голос»/«лицо» — вид ``remember_fact``, не чужой."""
    assert done_claims(text) == ("remember_fact",)


def test_5128_the_lie_without_a_tool_is_a_violation_and_memory_save_would_make_it_true():
    result = validate(parse({"speech": N206_LIE, "actions": []}), TOOLS)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_DONE_CLAIM
    assert result.all[0].allowed == ("memory_save",)
    assert "запомнил" in explain(result)


def test_5128_memory_save_in_the_command_backs_the_claim():
    cmd = Command(speech=N206_LIE, actions=(ToolCall("memory_save", {"fact": "болеет за Спартак"}),))
    assert unkept_done_claims(cmd) == ()
    assert isinstance(validate(cmd, TOOLS), Command)


@pytest.mark.parametrize("text", [
    "Запомнил, тебя зовут Саша.",
    "Помню: ты болеешь за Спартак.",
    "Ты болеешь за Спартак.",
    "Запомнил?",
    "Не запомнил: ты не сказал.",
    "Я уже запомнил.",
    "Запомнил человека.",
    "Записала",
    "Запомнил двоих.",
    "Запомнил по голосу двоих.",
    "Хорошо, Тамара, запомнил.",
])
def test_5128_these_are_not_a_fact_save_claim(text):
    """Знакомство, вопрос, отказ, счёт, обращение по имени до глагола — не запись факта."""
    assert "remember_fact" not in done_claims(text)
