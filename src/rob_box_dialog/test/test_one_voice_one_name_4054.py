"""#4054 (решение Шифу 27.10, порт #3801 в v2): один голос — одна личность, имя живёт в профиле, а не фактом.

Марафон 27.10, акт 2c (run 40476941395): голос уверенно «Нина» (0.930) сказал «я тамара мы с тобой еще не знакомы
запомни мой голос» → LLM: ``register_speaker("Тамара")`` + ``memory_save("Пользователя зовут Тамара")`` → второй
профиль, факт об имени в профиль Нины, в эфире «Запомнил тебя, Тамара. Запомнил: Пользователя зовут Тамара.».

Теперь (сторона диалога; переименование профиля — ``speaker_id_node``, его тесты в ``rob_box_voice``):

* исход ``register_speaker`` ``done{renamed}`` — фраза кода «мы знакомы» из таблицы фраз, вариант выбирает код;
* ``memory_save`` факта-имени РЯДОМ с ``register_speaker`` того же имени владельцу памяти не идёт: его исход - исход
  регистрации с причиной ``name_in_profile``, без своей фразы, «Запомнил: …» не звучит; без регистрации в команде
  факт-имя - обычный факт владельцу памяти (ревью PR #4087: ``done`` только с доказательством);
* «запомни мой голос» открывает группу people (``register_speaker`` в срезе), а не voice.
"""

import asyncio
from types import SimpleNamespace

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import memory, phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult, audible, phrase_from_result
from rob_box_dialog.session import Session, SessionSnapshot, SpeakerRef
from rob_box_dialog.tools_view import available_tools, groups_for, voice_asked
from rob_box_dialog.turn import Answer, TurnEngine, execute_for_speaker, outcome_phrase

#: Реплика акта 2c (STT дословно, raw #4054).
TAMARA = "робот привет я тамара мы с тобой еще не знакомы запомни мой голос пожалуйста"
NAME_FACT = ToolCall("memory_save", {"fact": "Пользователя зовут Тамара"})
REGISTER = ToolCall("register_speaker", {"name": "Тамара"})
RENAMED = phrases.catalog()["identity.done.renamed"]


def _snap(agent="personality", person="spk-nina", name="Нина", state="confirmed"):
    speaker = SpeakerRef(person_ref=person, name=name, name_state=state)
    return SessionSnapshot(agent=agent, session_id="t", epoch=0, started_at=0.0, speaker=speaker)


def _result(tool, status, reason=None, data=None, turn_id="t1/0"):
    return ActionResult(turn_id, 0, "personality", tool, kn.TOOL_CLASSES[tool], status, reason, data or {})


class _Ports:
    """Транспорт хода: что ушло владельцам и что прозвучало. Исход ``register_speaker`` - ``register`` (по умолчанию
    ``done{renamed}``: так его отдаёт тул по ``registered`` с ``new_name`` от ``speaker_id_node``), остальные тулы -
    ``done`` с доказательством памяти ``fact_id``."""

    def __init__(self, answers, register=("done", "renamed")):
        self.answers, self.asked, self.executed, self.said = list(answers), [], [], []
        self.register = register

    async def ask(self, messages, tools, timeout_s):
        self.asked.append([t.name for t in tools])
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([a.tool for a in actions])
        return [_result(a.tool, *(self.register if a.tool == "register_speaker" else ("done", None)),
                        {"name": a.args.get("name"), "fact": a.args.get("fact"), "fact_id": 1}, f"{turn_id}/{n}")
                for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, *args, **kwargs):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, *args):
        pass

    def perform(self, *args):
        return None


def _turn(command, register=("done", "renamed")):
    """Ход Личности на реплику акта 2c с подтверждённым собеседником «Нина»; модель отвечает ``command``."""
    ports = _Ports([Answer(command, "x")], register)
    ids = iter(f"t{n}" for n in range(1, 10))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    engine.session.set_speaker(SpeakerRef(person_ref="spk-nina", name="Нина", name_state="confirmed"))
    asyncio.run(engine.run(Addressed(TAMARA, VOICE)))
    return ports


# ── фраза исхода register_speaker ──────────────────────────────────────────────────────────────────────────────────

def test_renamed_registration_says_we_know_each_other_from_the_catalog():
    said = phrase_from_result(_result("register_speaker", "done", "renamed", {"name": "Тамара"})).text
    assert said in [v.format(name="Тамара") for v in RENAMED]
    assert "Запомнил тебя" not in said


def test_renamed_phrase_has_mild_and_joking_variants_chosen_by_code():
    assert 3 <= len(RENAMED) <= 5 and all("{name}" in v for v in RENAMED)
    said = {phrase_from_result(_result("register_speaker", "done", "renamed", {"name": "Тамара"},
                                       turn_id=f"t{n}/0")).text for n in range(40)}
    assert len(said) > 1  # вариант выбирает код по ходу, а не один и тот же текст


def test_plain_registration_still_says_remembered():
    said = phrase_from_result(_result("register_speaker", "done", None, {"name": "Тамара"})).text
    assert said == "Запомнил тебя, Тамара."


def test_renamed_is_said_even_next_to_model_speech():
    """Модель сказала «Приятно познакомиться» — переименование всё равно говорит код: исход не тот, что подразумевал
    ответ модели (как ``already_known``, #3953)."""
    results = [_result("register_speaker", "done", "renamed", {"name": "Тамара"})]
    assert list(audible(results, speaks=True)) == results


# ── факт-имя в памяти: только рядом с register_speaker того же имени ──────────────────────────────────────────────

@pytest.mark.parametrize("fact, name", [
    ("Пользователя зовут Тамара", "Тамара"), ("Меня зовут Денчик.", "денчик"), ("Имя собеседника — Нина", "Нина"),
    ("Зовут Тамара", "Тамара"), ("Пользователь представился как Гоша", "Гоша"),
    ("Пользователя теперь зовут Тамара", "Тамара"), ("Моё имя: Денчик", "Денчик"),
    ("Пользователя зовут Анна Мария", "Анна Мария"), ("Пользователя зовут Тамара", "Тамара."),
])
def test_fact_that_repeats_the_registered_name_is_a_name_fact(fact, name):
    actions = [ToolCall("memory_save", {"fact": fact}), ToolCall("register_speaker", {"name": name})]
    assert memory.name_fact(actions[0], _snap(), actions) == 1  # индекс регистрации, чей исход станет исходом факта


@pytest.mark.parametrize("fact", [
    "Сестру пользователя зовут Оля", "Тамара любит зелёный чай", "Пользователя зовут Тамара, она пьёт только чай",
    "Собаку зовут Шарик", "зовут", "Пьёт только зелёный чай", "Пользователь не представился",
    "Имя собеседника неизвестно", "Пользователя зовут Тамарой", "Пользователя зовут Нина",
])
def test_fact_that_is_not_the_registered_name_is_an_ordinary_fact(fact):
    """Ревью PR #4087: ложные срабатывания прежнего правила («не представился», «имя неизвестно») и другое имя -
    обычный факт, владельцу памяти."""
    actions = [REGISTER, ToolCall("memory_save", {"fact": fact})]
    assert memory.name_fact(actions[1], _snap(), actions) is None


def test_name_fact_without_registration_in_the_command_is_an_ordinary_fact():
    """Ревью PR #4087 (блокер): без ``register_speaker`` этого имени в команде имя никуда не пишется - факт идёт
    владельцу памяти, а не ``done{name_in_profile}`` без доказательства."""
    assert memory.name_fact(NAME_FACT, _snap(), [NAME_FACT]) is None
    other = ToolCall("register_speaker", {"name": "Денчик"})
    assert memory.name_fact(NAME_FACT, _snap(), [other, NAME_FACT]) is None


def test_name_facts_maps_each_name_fact_to_its_registration():
    tea = ToolCall("memory_save", {"fact": "Пьёт только зелёный чай"})
    assert memory.name_facts([REGISTER, tea, NAME_FACT], _snap()) == {2: 0}
    assert memory.name_facts([tea, NAME_FACT], _snap()) == {}


def test_telegram_name_fact_stays_a_fact_there_is_no_voice_profile():
    assert memory.name_fact(NAME_FACT, _snap(agent="telegram"), [REGISTER, NAME_FACT]) is None
    search = ToolCall("memory_search", {"query": "Пользователя зовут Тамара"})
    assert memory.name_fact(search, _snap(), [REGISTER, search]) is None


@pytest.mark.parametrize("status, reason", [("done", "renamed"), ("done", None), ("done", "already_known"),
                                            ("rejected", "too_short"), ("timeout", None)])
def test_name_fact_shares_the_fate_of_the_registration(status, reason):
    """``done`` факта-имени - только по ``done`` регистрации (событие владельца голосов): регистрация не прошла -
    имени нет нигде, и факт не ``done``."""
    assert memory.name_outcome(_result("register_speaker", status, reason)) == (status, kn.NAME_IN_PROFILE_REASON)


@pytest.mark.parametrize("status", ["done", "rejected", "timeout"])
def test_name_fact_has_no_phrase_of_its_own(status):
    """О факте-имени говорит фраза регистрации («Мы знакомы…», «Не смог запомнить голос.») - своей нет."""
    assert phrase_from_result(_result("memory_save", status, kn.NAME_IN_PROFILE_REASON,
                                      {"fact": "Пользователя зовут Тамара"})) is None


def test_execute_sends_registration_and_holds_the_name_fact():
    ports = _Ports([])
    results = asyncio.run(execute_for_speaker(ports, [REGISTER, NAME_FACT], _snap(), _snap(), "t1"))
    assert ports.executed == [["register_speaker"]]  # факт-имя владельцу памяти не ушёл
    assert [(r.tool, r.status, r.reason) for r in results] == [
        ("register_speaker", "done", "renamed"), ("memory_save", "done", kn.NAME_IN_PROFILE_REASON)]
    assert results[1].turn_id == "t1/1"
    said = outcome_phrase(results)
    assert said in [v.format(name="Тамара") for v in RENAMED] and "Запомнил:" not in said


def test_failed_registration_fails_the_name_fact_too():
    """Регистрация отклонена (``too_short``) - имя не в профиле: факт ``rejected{name_in_profile}``, не ``done``;
    звучит одна фраза регистрации, «Запомнил: …» нет."""
    ports = _Ports([], register=("rejected", "too_short"))
    results = asyncio.run(execute_for_speaker(ports, [NAME_FACT, REGISTER], _snap(), _snap(), "t1"))
    assert ports.executed == [["register_speaker"]]
    assert [(r.tool, r.status, r.reason) for r in results] == [
        ("memory_save", "rejected", kn.NAME_IN_PROFILE_REASON), ("register_speaker", "rejected", "too_short")]
    assert outcome_phrase(results) == phrases.text("identity.rejected.too_short")


def test_name_fact_of_a_stranger_follows_the_registration_not_no_person():
    """Незнакомый голос: регистрация заводит профиль, факт-имя - её исход, а не «Не узнаю твой голос, поэтому не
    запомню» владельца памяти."""
    stranger = _snap(person=None, name=None, state="unknown")
    ports = _Ports([], register=("done", None))
    results = asyncio.run(execute_for_speaker(ports, [REGISTER, NAME_FACT], stranger, stranger, "t1"))
    assert ports.executed == [["register_speaker"]]
    assert (results[1].status, results[1].reason) == ("done", kn.NAME_IN_PROFILE_REASON)
    assert outcome_phrase(results) == "Запомнил тебя, Тамара."


def test_name_fact_without_registration_is_the_memory_owners_result():
    """Ревью PR #4087 (блокер): без ``register_speaker`` исход факта-имени - событие владельца памяти (``fact_id``), а
    не ``done{name_in_profile}`` кода."""
    ports = _Ports([])
    results = asyncio.run(execute_for_speaker(ports, [NAME_FACT], _snap(), _snap(), "t1"))
    assert ports.executed == [["memory_save"]]
    assert [(r.status, r.reason, r.data.get("fact_id")) for r in results] == [("done", None, 1)]
    assert outcome_phrase(results) == "Запомнил: Пользователя зовут Тамара."


def test_name_fact_with_another_name_goes_to_the_memory_owner():
    fact = ToolCall("memory_save", {"fact": "Пользователя зовут Нина"})
    ports = _Ports([])
    results = asyncio.run(execute_for_speaker(ports, [REGISTER, fact], _snap(), _snap(), "t1"))
    assert ports.executed == [["register_speaker", "memory_save"]]
    assert (results[1].status, results[1].reason) == ("done", None)


# ── ход целиком ───────────────────────────────────────────────────────────────────────────────────────────────────

def test_act_2c_turn_sees_register_speaker_and_says_only_we_know_each_other():
    """Акт 2c n722: голос «Нина» уверенно, «я тамара … запомни мой голос» - регистрация (переименование у владельца
    голосов), факт-имя не пишется, звучит одна фраза кода «мы знакомы»."""
    ports = _turn(Command(actions=(REGISTER, NAME_FACT)))
    assert "register_speaker" in ports.asked[0]  # срез: регистрация видна с первого запроса
    assert ports.executed == [["register_speaker"]]
    assert len(ports.said) == 1 and ports.said[0] in [v.format(name="Тамара") for v in RENAMED]
    assert not any("Запомнил" in s for s in ports.said)


def test_name_fact_alone_next_to_speech_is_written_by_the_memory_owner():
    """Ревью PR #4087, пробник (а): речь модели + ``memory_save`` имени БЕЗ ``register_speaker`` - факт уходит владельцу
    памяти (``saved{fact_id}``), как до #4054, а не ``done{name_in_profile}`` без записи нигде под «запомнил»."""
    ports = _turn(Command(speech="Хорошо, Тамара, запомнил.", actions=(NAME_FACT,)))
    assert ports.executed == [["memory_save"]]
    assert ports.said == ["Хорошо, Тамара, запомнил."]


def test_name_fact_alone_without_speech_says_what_the_owner_saved():
    """Ревью PR #4087, пробник (б): тот же факт без речи - не молчаливый ход: владелец записал, код говорит что."""
    ports = _turn(Command(actions=(NAME_FACT,)))
    assert ports.executed == [["memory_save"]]
    assert ports.said == ["Запомнил: Пользователя зовут Тамара."]


def test_registration_rejected_in_the_turn_says_only_the_registration_failure():
    ports = _turn(Command(actions=(REGISTER, NAME_FACT)), register=("rejected", "too_short"))
    assert ports.executed == [["register_speaker"]]
    assert ports.said == [phrases.text("identity.rejected.too_short")]


# ── срез тулов ────────────────────────────────────────────────────────────────────────────────────────────────────

def test_remember_my_voice_opens_people_not_robot_voice():
    snap = _snap()
    assert groups_for(snap, TAMARA) == ("people",)
    assert "register_speaker" in [t.name for t in available_tools(snap, text=TAMARA)]
    assert not voice_asked(snap, TAMARA)  # голос робота сам собой не меняется


@pytest.mark.parametrize("text", [
    "меня зовут денчик", "зови меня денчик", "робот запомни мой голос", "ты узнаешь меня по голосу",
    "я тамара мы не знакомы запомни моего голоса звучание",
])
def test_self_intro_phrases_see_register_speaker(text):
    assert "register_speaker" in [t.name for t in available_tools(_snap(), text=text)]


def test_robot_voice_request_next_to_my_voice_still_opens_voice():
    groups = groups_for(_snap(), "запомни мой голос и смени свой на яндекс")
    assert "voice" in groups and "people" in groups
    assert groups_for(_snap(), "смени голос на яндекс") == ("voice",)


def test_snapshot_namespace_is_enough_for_name_fact():
    """``name_fact`` читает у снимка только агента — заглушка снимка тоже годится."""
    assert memory.name_fact(NAME_FACT, SimpleNamespace(agent="personality"), [NAME_FACT, REGISTER]) == 1
