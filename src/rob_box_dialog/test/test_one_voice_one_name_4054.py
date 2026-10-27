"""#4054 (решение Шифу 27.10, порт #3801 в v2): один голос — одна личность, имя живёт в профиле, а не фактом.

Марафон 27.10, акт 2c (run 40476941395): голос уверенно «Нина» (0.930) сказал «я тамара мы с тобой еще не знакомы
запомни мой голос» → LLM: ``register_speaker("Тамара")`` + ``memory_save("Пользователя зовут Тамара")`` → второй
профиль, факт об имени в профиль Нины, в эфире «Запомнил тебя, Тамара. Запомнил: Пользователя зовут Тамара.».

Теперь (сторона диалога; переименование профиля — ``speaker_id_node``, его тесты в ``rob_box_voice``):

* исход ``register_speaker`` ``done{renamed}`` — фраза кода «мы знакомы» из таблицы фраз, вариант выбирает код;
* ``memory_save`` факта-имени владельцу не идёт: ``done{name_in_profile}`` без фразы, «Запомнил: …» не звучит;
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
    """Транспорт хода: что ушло владельцам и что прозвучало; исход ``register_speaker`` — ``done{renamed}`` (так его
    отдаёт тул по ``registered`` с ``new_name`` от ``speaker_id_node``)."""

    def __init__(self, answers):
        self.answers, self.asked, self.executed, self.said = list(answers), [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append([t.name for t in tools])
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([a.tool for a in actions])
        return [_result(a.tool, "done", "renamed" if a.tool == "register_speaker" else None,
                        {"name": a.args.get("name"), "fact_id": 1}, f"{turn_id}/{n}") for n, a in enumerate(actions)]

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


# ── факт-имя в памяти ─────────────────────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("fact", [
    "Пользователя зовут Тамара", "Меня зовут Денчик.", "Имя собеседника — Нина", "Зовут Тамара",
    "Пользователь представился как Гоша", "Пользователя теперь зовут Тамара", "Моё имя: Денчик",
])
def test_fact_that_is_only_the_name_is_a_name_fact(fact):
    assert memory.name_fact(ToolCall("memory_save", {"fact": fact}), _snap())


@pytest.mark.parametrize("fact", [
    "Сестру пользователя зовут Оля", "Тамара любит зелёный чай", "Пользователя зовут Тамара, она пьёт только чай",
    "Собаку зовут Шарик", "зовут", "Пьёт только зелёный чай",
])
def test_fact_with_more_than_the_name_is_an_ordinary_fact(fact):
    assert not memory.name_fact(ToolCall("memory_save", {"fact": fact}), _snap())


def test_telegram_name_fact_stays_a_fact_there_is_no_voice_profile():
    assert not memory.name_fact(NAME_FACT, _snap(agent="telegram"))
    assert not memory.name_fact(ToolCall("memory_search", {"query": "Пользователя зовут Тамара"}), _snap())


def test_name_fact_is_decided_by_code_before_the_owner():
    assert memory.code_outcome(NAME_FACT, _snap(), _snap()) == ("done", kn.NAME_IN_PROFILE_REASON)
    fact = ToolCall("memory_save", {"fact": "пьёт только зелёный чай"})
    assert memory.code_outcome(fact, _snap(), _snap()) is None
    assert memory.code_outcome(fact, _snap(), _snap(person="spk-boris")) == ("rejected", kn.SPEAKER_CHANGED_REASON)


def test_name_fact_is_silent():
    assert phrase_from_result(_result("memory_save", "done", kn.NAME_IN_PROFILE_REASON,
                                      {"fact": "Пользователя зовут Тамара"})) is None


def test_name_fact_of_a_stranger_is_silent_not_no_person():
    """Незнакомый голос: регистрация заводит профиль, факт-имя — не «Не узнаю твой голос, поэтому не запомню»."""
    stranger = _snap(person=None, name=None, state="unknown")
    assert memory.code_outcome(NAME_FACT, stranger, stranger) == ("done", kn.NAME_IN_PROFILE_REASON)


def test_execute_sends_registration_and_drops_the_name_fact():
    ports = _Ports([])
    results = asyncio.run(execute_for_speaker(ports, [REGISTER, NAME_FACT], _snap(), _snap(), "t1"))
    assert ports.executed == [["register_speaker"]]  # факт-имя владельцу памяти не ушёл
    assert [(r.tool, r.status, r.reason) for r in results] == [
        ("register_speaker", "done", "renamed"), ("memory_save", "done", kn.NAME_IN_PROFILE_REASON)]
    said = outcome_phrase(results)
    assert said in [v.format(name="Тамара") for v in RENAMED] and "Запомнил:" not in said


# ── ход целиком: акт 2c n722 ──────────────────────────────────────────────────────────────────────────────────────

def test_act_2c_turn_sees_register_speaker_and_says_only_we_know_each_other():
    ports = _Ports([Answer(Command(actions=(REGISTER, NAME_FACT)), "x")])
    ids = iter(f"t{n}" for n in range(1, 10))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    engine.session.set_speaker(SpeakerRef(person_ref="spk-nina", name="Нина", name_state="confirmed"))
    asyncio.run(engine.run(Addressed(TAMARA, VOICE)))
    assert "register_speaker" in ports.asked[0]  # срез: регистрация видна с первого запроса
    assert ports.executed == [["register_speaker"]]
    assert len(ports.said) == 1 and ports.said[0] in [v.format(name="Тамара") for v in RENAMED]
    assert not any("Запомнил" in s for s in ports.said)


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
    """``name_fact`` читает у снимка только агента — заглушка снимка режима команд v1 тоже годится."""
    assert memory.name_fact(NAME_FACT, SimpleNamespace(agent="personality"))
