"""М11 n109 (#4236, run 41945071345): «Робот, какие звуки ты вообще умеешь издавать?» → ``get_sound_info`` → команда из
одних ``play_sound`` без слов → ход закрылся без речи (робот пискнул и не ответил).

Успех звука молчит (``knowledge.SILENT_DONE_TOOLS``), и звук после запроса проходил как законный ответ без слов
(``Form.speech_required`` судит только «лицо или анимацию», ``command.bare_answer``). Решает код (ADR-0148): на ВОПРОС
после чтения звук без слов - нарушение ``VIOLATION_SOUND_ANSWER`` (``turn.sound_owed``): модели повтор с подсказкой,
повтора нет - код говорит по данным запроса (``turn.data_close``). Звук на ПРОСЬБУ после чтения - законный ответ,
как был.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, _Turn, owed_reason, sound_owed

N109 = "Робот, какие звуки ты вообще умеешь издавать?"
SOUNDS = ToolCall("get_sound_info", {})
HAPPY = ToolCall("play_sound", {"sound": "robot_happy"})
CHIME = ToolCall("play_sound", {"sound": "ui_chime"})
FACE = Expression(emotion="happy", earcon="done")
SOUND_DATA = {"total_sounds": 51, "categories": {"base": 21, "ui": 12, "robot": 18, "other": 0},
              "all_triggers": ["activation", "affirmation", "robot_happy"]}
SOUNDS_PHRASE = "Звуков у меня 51: эмоции — 21, интерфейс — 12, эффекты — 18. Назвать какие-нибудь?"


class Ports:
    """Транспорт хода: ответы LLM по очереди, исход действия - аргументы вызова и данные тула поверх них."""

    def __init__(self, answers=(), outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.asked, self.executed, self.said = [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        out = []
        for n, a in enumerate(actions):
            status, data = self.outcomes.get(a.tool, ("done", {}))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool,
                                    kn.call_class(a.tool, a.args), status, None, {**dict(a.args), **data}))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="",
              expression=None):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _answer(**kw):
    return Answer(Command(**kw), "deepseek", follow=lambda data: ({"role": "tool", "content": data},))


def _run(ports, text=N109):
    return asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))


def _played(ports):
    return [call for batch in ports.executed for call in batch if call[0] == "play_sound"]


# -- ход ---------------------------------------------------------------------------------------------------------------


def test_n109_sound_after_the_query_is_returned_to_the_model_and_its_words_are_heard():
    """Живой лог n109: запрос, затем звук без слов. Звук не исполняется - модели уходит нарушение с подсказкой
    «ответь словами по этим данным»; её слова звучат."""
    ports = Ports([_answer(actions=(SOUNDS,)), _answer(actions=(HAPPY,), expression=FACE),
                   _answer(speech="Звуков у меня полсотни: эмоции, интерфейс и эффекты.")],
                  {"get_sound_info": ("done", SOUND_DATA)})
    _run(ports)
    assert ports.said == ["Звуков у меня полсотни: эмоции, интерфейс и эффекты."]
    assert _played(ports) == [], "звук вместо ответа не исполняется"
    hint = ports.asked[2][-1]["content"]
    assert kn.VIOLATION_SOUND_ANSWER in hint and kn.SOUND_ANSWER_HINT in hint
    assert "данные запроса уже получены" in hint


def test_n109_model_insists_on_the_sound_the_code_names_the_categories():
    """Модель и после повтора и повторов за словами (``SPEECH_PUSHES_MAX``, как у ``silent_answer``) отвечает звуком:
    ход не закрывается молча - код говорит по данным ``get_sound_info``; звук вместо ответа не исполняется."""
    sound = _answer(actions=(HAPPY,), expression=FACE)
    ports = Ports([_answer(actions=(SOUNDS,)), _answer(actions=(HAPPY, CHIME), expression=FACE), sound,
                   *[sound] * kn.SPEECH_PUSHES_MAX],
                  {"get_sound_info": ("done", SOUND_DATA)})
    _run(ports)
    assert ports.said == [SOUNDS_PHRASE]
    assert len(ports.asked) == 3 + kn.SPEECH_PUSHES_MAX and ports.answers == []
    assert _played(ports) == []


def test_sound_on_a_request_after_the_query_stays_a_valid_answer():
    """Просьба, а не вопрос: «включи какой-нибудь звук» → ``get_sound_info`` → ``play_sound`` - звук и есть ответ,
    повтора нет, слов кода нет (успех звука молчит)."""
    ports = Ports([_answer(actions=(SOUNDS,)), _answer(actions=(HAPPY,))], {"get_sound_info": ("done", SOUND_DATA)})
    _run(ports, "Робот, включи какой-нибудь звук")
    assert _played(ports) == [("play_sound", {"sound": "robot_happy"})]
    assert len(ports.asked) == 2 and ports.said == []


# -- правило -----------------------------------------------------------------------------------------------------------


def _turn(text=N109, queried=True):
    done = (ActionResult("t1/0", 0, "personality", "get_sound_info", "query", "done", None, SOUND_DATA),)
    return _Turn("t1", 0, text, VOICE, deadline=None, queried=done if queried else ())


def _snap(agent="personality"):
    return Session(agent).snapshot


@pytest.mark.parametrize("cmd, kw, agent, expected", [
    (Command(actions=(HAPPY,)), {}, "personality", True),
    (Command(actions=(HAPPY, CHIME), expression=FACE), {}, "personality", True),
    (Command(actions=(HAPPY,)), {"queried": False}, "personality", False),        # без чтения - звук законен
    (Command(actions=(HAPPY,)), {"text": "Робот, включи звук"}, "personality", False),  # просьба, не вопрос
    (Command(actions=(HAPPY,)), {}, "telegram", False),                         # не в динамики
    (Command(speech="Звуков полсотни."), {}, "personality", False),              # слова есть
    (Command(actions=(ToolCall("stop_music", {}),)), {}, "personality", False),  # не тихий успех
])
def test_sound_owed(cmd, kw, agent, expected):
    assert sound_owed(_snap(agent), _turn(**kw), cmd) is expected


def test_owed_reason_names_the_sound_answer():
    assert owed_reason(_snap(), _turn(), Command(actions=(HAPPY,))) == kn.VIOLATION_SOUND_ANSWER
    assert owed_reason(_snap(), _turn(), Command(actions=(ToolCall("play_animation", {"animation": "happy"}),))) == \
        kn.VIOLATION_SILENT_ANSWER, "голая анимация - прежнее нарушение"
