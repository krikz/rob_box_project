"""#4083 (Шифу 27.10): «покажи молча» — лицом без слов; показ по просьбе держится ≥ 5 с; пачка — по очереди.

Живые факты 27.10 (dev-d68a784): «Робот, покажи молча, что ты задумался» → ответ одним лицом → нарушение
``silent_answer`` → модель вынуждена сказать «Молчу и думаю.», а ``play_animation`` той же просьбой проходил молча;
рефлекс ``done`` красил лицо на 3 с; ``fields_dropped=['expression.duration_s']``; три ``play_animation`` одной команды
шли разом, последняя перекрывала остальные. Решает код (ADR-0148): таблица слов «без слов» в ``knowledge``, время
показа — ``expression.show_seconds``, очередь — ``expression.queued``.
"""

import asyncio
import math

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, ToolCall
from rob_box_dialog.expression import held_face, hold_ms, merge, queued, reflex, show_seconds
from rob_box_dialog.face_scene import FaceScene
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import needs_words, show_asked, silent_asked
from rob_box_dialog.turn import Answer, Shown, TurnEngine, command_face, held_expression

#: 27.10 08:20 «Робот, покажи молча, что ты задумался» (вейк снимает адресация).
N_SILENT = "покажи молча что ты задумался"
#: Та же просьба без «молча»: «задумался» лицо не называет (``FACE_SHOW_WHAT_STEMS``) — правило #3995 как было.
N_PLAIN = "покажи что ты задумался"
#: Живая проверка из карточки: пачка из трёх сцен.
N_BATCH = "покажи по очереди радость грусть и удивление"
THINKING = Expression(emotion="thinking")
DEFAULT = kn.FACE_SHOW_DEFAULT_S


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход действия — аргументы вызова поверх данных тула."""

    def __init__(self, answers=()):
        self.answers = list(answers)
        self.asked, self.executed, self.said, self.stages, self.faces = [], [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool], "done",
                             None, dict(a.args)) for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, voice, emotion))
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="",
              expression=None):
        self.stages.append((stage, status, reason))
        if expression is not None:
            self.faces.append((stage, expression))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(ports, text):
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def _animation(name, duration=None):
    args = {"animation": name} if duration is None else {"animation": name, "duration": duration}
    return ToolCall("play_animation", args)


def _done_plan(expression):
    """Что уйдёт лицу на итоге (как ``dialog_v2_ports.stage_plan``): рефлекс ``done`` со сценой и временем хода."""
    seconds = expression.duration_s
    return merge(reflex("done"), "done", expression.emotion, expression.gesture, held_face(expression.face, seconds),
                 hold_ms(seconds))


# -- (1) «молча»: лицом и анимацией одинаково, без речи и без нарушения -----------------------------------------------


@pytest.mark.parametrize("cmd, face", [
    (Command(expression=THINKING), Expression(emotion="thinking", duration_s=DEFAULT)),
    (Command(actions=(_animation("thinking"),)), Shown(emotion="thinking", duration_s=DEFAULT)),
], ids=["expression", "play_animation"])
def test_silent_ask_closes_the_turn_without_speech_and_without_the_violation(cmd, face):
    """08:20: «покажи молча, что ты задумался» — ответ одним лицом больше не нарушение ``silent_answer``: один вызов
    LLM, речи нет («Молчу и думаю.» не звучит), ход ``done/finished``, лицо — показ по просьбе (≥ минимума)."""
    ports = Ports([Answer(cmd, "deepseek")])
    agg = _run(ports, N_SILENT)
    assert len(ports.asked) == 1 and agg.llm_calls == 1, "нарушения silent_answer нет — повторного вызова нет"
    assert ports.said == [] and (agg.stage, agg.status) == ("done", "finished")
    assert ports.stages[-1] == ("done", "finished", "")
    assert ports.faces == [("done", face)]
    assert _done_plan(face).ttl_ms >= kn.FACE_SHOW_MIN_S * 1000 > reflex("done").ttl_ms, "не 3 с рефлекса"


@pytest.mark.parametrize("cmd", [Command(expression=THINKING), Command(actions=(_animation("thinking"),))],
                         ids=["expression", "play_animation"])
def test_without_silent_ask_the_3995_rule_is_as_before(cmd):
    """Та же просьба без «молча»: лицо словами не названо — голый ответ лицом возвращается модели нарушением (#3995,
    #3997), вторая итерация говорит словами."""
    fixed = Command(speech="Задумался.", expression=THINKING)
    ports = Ports([Answer(cmd, "deepseek"), Answer(fixed, "deepseek")])
    _run(ports, N_PLAIN)
    assert len(ports.asked) == 2 and kn.VIOLATION_SILENT_ANSWER in ports.asked[1][-1]["content"]
    assert [t for t, _, _ in ports.said] == ["Задумался."]


@pytest.mark.parametrize("text", [
    N_SILENT, "Робот, покажи молча, что ты задумался", "без слов покажи, что ты устал",
    "тихо покажи, что ты задумался", "покажи тихонько, как ты спишь", "ничего не говори, просто покажи, что думаешь",
    "не говоря ни слова, покажи удивление", "молчком изобрази кота", "покажи беззвучно, что ты рад",
])
def test_silent_asked_by_the_table(text):
    assert silent_asked(text) and show_asked(text)
    assert not needs_words(text, Command(expression=THINKING))
    assert not needs_words(text, Command(actions=(_animation("thinking"),)))


@pytest.mark.parametrize("text", [
    "не молчи", "помолчи немного", "что ты молчишь", "молчание — золото", "говори тише", "тихо включи музыку",
    "сделай потише", N_PLAIN, "ты меня слышишь",
])
def test_not_a_silent_ask(text):
    """«Не молчи», «помолчи» (тишину решает Tier-1), «тихо» без «покажи» (громкость) — без слов не просят."""
    assert not silent_asked(text)


def test_needs_words_for_a_question_is_unchanged():
    assert needs_words("ты меня слышишь", Command(expression=THINKING))
    assert needs_words("как дела", Command(actions=(_animation("happy"),)))


# -- (2) время показа: одно правило ------------------------------------------------------------------------------------


@pytest.mark.parametrize("asked, seconds", [
    (None, DEFAULT), (1, kn.FACE_SHOW_MIN_S), (3.0, kn.FACE_SHOW_MIN_S), (kn.FACE_SHOW_MIN_S, kn.FACE_SHOW_MIN_S),
    (8, 8.0), (12.5, 12.5), (60, kn.FACE_SCENE_MAX_S), (math.inf, kn.FACE_SCENE_MAX_S), (0, DEFAULT), (-4, DEFAULT),
    (True, DEFAULT), ("долго", DEFAULT), (math.nan, DEFAULT),
])
def test_show_seconds_is_one_rule_with_a_floor(asked, seconds):
    assert show_seconds(asked) == seconds
    assert kn.FACE_SHOW_MIN_S <= show_seconds(asked) <= kn.FACE_SCENE_MAX_S


def test_floor_is_five_seconds_and_the_default_is_within_shifus_five_to_ten():
    """Шифу 27.10: «не микросекунды, а полноценно хотя бы 5–10 секунд»."""
    assert kn.FACE_SHOW_MIN_S == 5.0 and 5.0 <= kn.FACE_SHOW_DEFAULT_S <= 10.0
    assert kn.FACE_SHOW_MIN_S * 1000 > reflex("done").ttl_ms, "рефлекс done короче показа: его время сцену не режет"


def test_expression_duration_reaches_the_done_plan_and_the_face_scene():
    """Модель задала ``expression.duration_s`` (раньше — ``fields_dropped``): 8 с доходят до ``ttl_ms`` и
    ``duration_s`` сцены лица."""
    face = FaceScene(eyes="half", mouth="flat", tint="cold")
    cmd = Command(speech="Тихо. Я тоже помолчу.", expression=Expression(emotion="sleepy", face=face, duration_s=8))
    ports = Ports([Answer(cmd, "deepseek")])
    _run(ports, "все спасибо можно отдыхать")
    done = ports.faces[0][1]
    assert done.duration_s == 8.0 and done.face.duration_s == 8.0
    plan = _done_plan(done)
    assert (plan.emotion, plan.ttl_ms, plan.face["duration_s"]) == ("sleep", 8000, 8.0)


def test_model_duration_below_the_floor_is_raised_to_it():
    cmd = Command(speech="Ок.", expression=Expression(emotion="sad", duration_s=2))
    ports = Ports([Answer(cmd, "deepseek")])
    _run(ports, "привет")
    assert ports.faces[0][1].duration_s == kn.FACE_SHOW_MIN_S


def test_asked_face_holds_the_show_time_not_the_done_reflex():
    """«Покажи, как тебе грустно» → ``{emotion: sad}``: время итога — показ по просьбе, а не 3 с рефлекса ``done``."""
    ports = Ports([Answer(Command(expression=Expression(emotion="sad")), "deepseek")])
    _run(ports, "покажи как тебе грустно")
    plan = _done_plan(ports.faces[0][1])
    assert (plan.emotion, plan.ttl_ms) == ("sad", int(DEFAULT * 1000))


def test_ordinary_reply_keeps_the_reflex_time():
    """Обычный ответ (показ не просили, время не задано): лицо команды держится временем рефлекса, как было."""
    ports = Ports([Answer(Command(speech="Привет!", expression=Expression(emotion="happy")), "deepseek")])
    _run(ports, "привет")
    assert ports.faces == [("done", Expression(emotion="happy"))]
    assert _done_plan(ports.faces[0][1]).ttl_ms == reflex("done").ttl_ms
    assert held_expression(Expression(emotion="happy"), "привет") == Expression(emotion="happy")


# -- (3) пачка play_animation — по очереди, каждая ≥ минимума ---------------------------------------------------------


def test_batch_of_three_animations_is_a_queue_not_an_overwrite():
    """«Покажи по очереди радость, грусть и удивление» → три ``play_animation``: первая — сцена итога, две — очередь
    за ней; ни одна не перекрывает другую, каждая держится ≥ ``FACE_SHOW_MIN_S``."""
    calls = (_animation("happy"), _animation("sad", 3), _animation("surprised", 12))
    ports = Ports([Answer(Command(actions=calls), "deepseek")])
    agg = _run(ports, N_BATCH)
    assert ports.said == [] and (agg.stage, agg.status) == ("done", "finished")
    first = ports.faces[0][1]
    assert first == Shown(emotion="happy", duration_s=DEFAULT,
                          then=(Shown(emotion="sad", duration_s=kn.FACE_SHOW_MIN_S),
                                Shown(emotion="surprised", duration_s=12.0)))
    steps = queued(first.duration_s, first.then)
    assert [(at, p.emotion, p.ttl_ms) for at, p in steps] == [
        (DEFAULT, "sad", int(kn.FACE_SHOW_MIN_S * 1000)), (DEFAULT + kn.FACE_SHOW_MIN_S, "surprised", 12000)]
    assert all(p.ttl_ms >= kn.FACE_SHOW_MIN_S * 1000 and not p.earcon and not p.gesture for _, p in steps)


def test_queue_starts_each_scene_when_the_previous_one_ends():
    scenes = [Shown(emotion=e, duration_s=s) for e, s in (("sad", 6.0), ("angry", None), ("victory", 1.0))]
    steps = queued(7.0, scenes)
    assert [at for at, _ in steps] == [7.0, 13.0, 13.0 + DEFAULT]
    assert [p.ttl_ms for _, p in steps] == [6000, int(DEFAULT * 1000), int(kn.FACE_SHOW_MIN_S * 1000)]
    assert queued(7.0, ()) == ()


def test_queued_face_scene_lives_as_long_as_its_scene():
    face = FaceScene(eyes="wide", tint="red", duration_s=2.0)
    (at, plan), = queued(5.0, [Expression(face=face, duration_s=9.0)])
    assert at == 5.0 and plan.emotion == "" and plan.face["duration_s"] == 9.0 and plan.ttl_ms == 9000


def test_failed_action_in_the_batch_shows_no_scene():
    res = [ActionResult("t/0", 0, "personality", "play_animation", "action.local", "done", None, {"animation": "sad"}),
           ActionResult("t/1", 0, "personality", "stop_music", "action.media", "rejected")]
    assert command_face(None, res, N_BATCH) is None, "«радостно» о провале не бывает — и очереди нет"
