"""#4103 n905 (комментарии Шифу 27.10): «а теперь победную анимацию» - сцена ``victory`` не пропадает, когда ход продолжается.

Живой марафон 27.10 20:13 UTC (dev-d97bb04), акт 9, n905: «Робот, а теперь победную анимацию — блок питания наконец не
гудит!» -> ``play_animation {animation: victory, duration: 5}`` и ``get_current_time`` в одной команде, затем речь («Сейчас
двадцать три тринадцать…»). На лице: ``сцена по запросу: happy -> happy (3.0 с)`` - рефлекс ``done`` на 3 с, победной
сцены нет. Причина в ходе: ``get_current_time`` - ``query``, модель зовётся второй раз, а ``TurnEngine._speak`` второй
итерации красил итог ``command_face`` ТОЛЬКО своим результатам и затирал сцену ``victory`` первой итерации. Теперь сцены
исполненных ``play_animation`` прошлых итераций хода остаются на итоге (``command_face(..., earlier=turn.expression)``),
новые встают очередью за ними. Просьба показать без слова «покажи» («<имя> анимацию», «анимацию <имени>») - показ по
просьбе (``tools_view.show_asked``: основа «анимац» в ``FACE_ASK_STEMS``) - зафиксирована здесь же.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, ToolCall
from rob_box_dialog.expression import held_face, hold_ms, merge, queued, reflex
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import needs_words, show_asked
from rob_box_dialog.turn import SAD_FACE, Answer, Shown, TurnEngine

#: Фраза n905 как её слышит ход (вейк снимает адресация).
N905 = "Робот, а теперь победную анимацию — блок питания наконец не гудит!"
SPEECH = "Сейчас двадцать три тринадцать."
VICTORY = ToolCall("play_animation", {"animation": "victory", "duration": 5})
TIME = ToolCall("get_current_time", {})
DEFAULT = kn.FACE_SHOW_DEFAULT_S


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход действия - аргументы вызова поверх данных тула."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.asked, self.executed, self.said, self.faces = [], [], [], []

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


def _run(ports, text=N905):
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def _requests(expression):
    """Что уйдёт лицу ExpressionRequest-ами (как ``dialog_v2_ports``): ``(через сколько секунд, сцена, ttl_ms)``."""
    seconds = expression.duration_s
    done = merge(reflex("done"), "done", expression.emotion, expression.gesture, held_face(expression.face, seconds),
                 hold_ms(seconds))
    rest = queued(expression.duration_s, getattr(expression, "then", ()))
    return [(0.0, done.emotion, done.ttl_ms)] + [(at, p.emotion, p.ttl_ms) for at, p in rest]


def test_victory_scene_of_the_first_iteration_survives_the_spoken_second_one():
    """n905: ``[play_animation victory 5, get_current_time]`` -> данные модели -> речь без экспрессии. На итоге ``done`` -
    сцена ``victory`` на 5 с (не рефлекс ``happy`` на 3 с), речь прозвучала."""
    ports = Ports([Answer(Command(actions=(VICTORY, TIME)), "deepseek"), Answer(Command(speech=SPEECH), "deepseek")])
    agg = _run(ports)
    assert len(ports.asked) == 2 and agg.llm_calls == 2 and (agg.stage, agg.status) == ("done", "finished")
    assert [text for text, _, _ in ports.said] == [SPEECH]
    (stage, face), = ports.faces
    assert stage == "done" and face == Shown(emotion="victory", duration_s=5.0)
    (at, emotion, ttl), = _requests(face)
    assert (at, emotion, ttl) == (0.0, "victory", 5000)
    assert ttl >= kn.FACE_SHOW_MIN_S * 1000 > reflex("done").ttl_ms, "не 3 с рефлекса done"


def test_models_own_expression_in_the_second_iteration_does_not_replace_the_shown_scene():
    """Вторая итерация со своей экспрессией (``happy``): сцена, о которой просили, красит итог, а не настроение речи."""
    ports = Ports([Answer(Command(actions=(VICTORY, TIME)), "deepseek"),
                   Answer(Command(speech=SPEECH, expression=Expression(emotion="happy")), "deepseek")])
    _run(ports)
    (_, face), = ports.faces
    assert face == Shown(emotion="victory", duration_s=5.0)


def test_scene_of_the_second_iteration_goes_to_the_queue_after_the_first():
    """Сцены по порядку вызовов: ``victory`` первой итерации - на итоге, ``sad`` второй - очередью за ней."""
    sad = ToolCall("play_animation", {"animation": "sad", "duration": 7})
    ports = Ports([Answer(Command(actions=(VICTORY, TIME)), "deepseek"),
                   Answer(Command(speech=SPEECH, actions=(sad,)), "deepseek")])
    _run(ports)
    (_, face), = ports.faces
    assert face == Shown(emotion="victory", duration_s=5.0, then=(Shown(emotion="sad", duration_s=7.0),))
    assert _requests(face) == [(0.0, "victory", 5000), (5.0, "sad", 7000)]


def test_two_query_iterations_keep_every_scene():
    """Сцены копятся по итерациям с ``query``: ни одна не затирается следующей."""
    sad = ToolCall("play_animation", {"animation": "sad", "duration": 6})
    ports = Ports([Answer(Command(actions=(VICTORY, TIME)), "deepseek"),
                   Answer(Command(actions=(sad, TIME)), "deepseek"),
                   Answer(Command(speech=SPEECH), "deepseek")])
    _run(ports)
    (_, face), = ports.faces
    assert [(e, ttl) for _, e, ttl in _requests(face)] == [("victory", 5000), ("sad", 6000)]


def test_same_scene_repeated_by_the_model_on_the_next_iteration_is_shown_once():
    """Модель повторяет ``[play_animation victory, query]`` на каждой итерации (потолок #4065): одна и та же сцена в
    очередь второй раз не встаёт - лицо не держит ``victory`` втрое дольше, чем попросили."""
    ports = Ports([Answer(Command(actions=(VICTORY, TIME)), "deepseek"),
                   Answer(Command(actions=(VICTORY, TIME)), "deepseek"),
                   Answer(Command(speech=SPEECH), "deepseek")])
    _run(ports)
    (_, face), = ports.faces
    assert face == Shown(emotion="victory", duration_s=5.0)


SAD = ToolCall("play_animation", {"animation": "sad", "duration": 6})
SHOWN_TWICE = Shown(emotion="victory", duration_s=5.0, then=(Shown(emotion="sad", duration_s=6.0),))


def test_list_of_two_scenes_repeated_by_the_model_on_every_iteration_is_shown_once():
    """Ревью PR #4118: сравнивалась только первая новая сцена с последней стоящей, и список ``[victory, sad]``, который
    модель зовёт на каждой итерации (потолок #4065), множился: victory, sad, victory, sad. Теперь повтор всей
    последовательности в очередь второй раз не встаёт."""
    ports = Ports([Answer(Command(actions=(VICTORY, SAD, TIME)), "deepseek"),
                   Answer(Command(actions=(VICTORY, SAD, TIME)), "deepseek"),
                   Answer(Command(actions=(VICTORY, SAD, TIME)), "deepseek"),
                   Answer(Command(speech=SPEECH), "deepseek")])
    _run(ports)
    (_, face), = ports.faces
    assert face == SHOWN_TWICE
    assert _requests(face) == [(0.0, "victory", 5000), (5.0, "sad", 6000)]


def test_repeated_list_with_a_new_scene_at_the_end_adds_only_the_new_scene():
    """Вторая итерация повторяет стоящее и добавляет своё: стоящий хвост не дублируется, новое встаёт за ним."""
    happy = ToolCall("play_animation", {"animation": "happy", "duration": 7})
    ports = Ports([Answer(Command(actions=(VICTORY, SAD, TIME)), "deepseek"),
                   Answer(Command(actions=(VICTORY, SAD, happy, TIME)), "deepseek"),
                   Answer(Command(speech=SPEECH), "deepseek")])
    _run(ports)
    (_, face), = ports.faces
    assert [(e, ttl) for _, e, ttl in _requests(face)] == [("victory", 5000), ("sad", 6000), ("happy", 7000)]


def test_second_iteration_may_repeat_only_the_last_scene_and_go_on():
    """Прежнее правило (одна сцена) сохранено: повтор последней стоящей сцены схлопывается, остальное встаёт очередью."""
    ports = Ports([Answer(Command(actions=(VICTORY, TIME)), "deepseek"),
                   Answer(Command(actions=(VICTORY, SAD, TIME)), "deepseek"),
                   Answer(Command(speech=SPEECH), "deepseek")])
    _run(ports)
    (_, face), = ports.faces
    assert face == SHOWN_TWICE


def test_scene_by_face_only_repeated_on_the_next_iteration_is_shown_once():
    """Сцена только лицом (без ``animation``) - обычная ``Expression``, а стоящая на итоге первая сцена - ``Shown``: повтор
    узнаётся по полям сцены, а не по классу (иначе сцена встала бы в очередь за собой)."""
    from rob_box_dialog.turn import command_face

    result = ActionResult("t/0", 1, "personality", "play_animation", kn.TOOL_CLASSES["play_animation"], "done", None,
                          {"face": {"eyes": "wide", "mouth": "flat"}, "duration": 7})
    first = command_face(None, [result])
    assert type(first) is Shown and first.then == () and first.duration_s == 7.0 and first.face.eyes == "wide"
    assert command_face(None, [result], earlier=first) == first
    assert command_face(None, [result], earlier=command_face(None, [result], earlier=first)) == first


def test_one_command_with_the_same_scene_twice_is_still_two_scenes():
    """«Покажи победу два раза» одной командой - две сцены подряд: схлопывается только повтор между итерациями."""
    ports = Ports([Answer(Command(actions=(VICTORY, VICTORY)), "deepseek")])
    _run(ports, "покажи победную анимацию два раза")
    (_, face), = ports.faces
    assert face == Shown(emotion="victory", duration_s=5.0, then=(Shown(emotion="victory", duration_s=5.0),))


def test_bare_victory_animation_is_shown_for_its_time_without_words():
    """Просьба n905 без лишнего вызова: один ``play_animation victory 5`` - ответ лицом без слов (показ по просьбе, не
    нарушение ``silent_answer``), на итоге ``done`` сцена ``victory`` на 5 с, а не рефлекс на 3 с."""
    ports = Ports([Answer(Command(actions=(VICTORY,)), "deepseek")])
    agg = _run(ports)
    assert len(ports.asked) == 1 and agg.llm_calls == 1 and ports.said == []
    (stage, face), = ports.faces
    assert stage == "done" and face == Shown(emotion="victory", duration_s=5.0)
    assert _requests(face) == [(0.0, "victory", 5000)]


def test_single_command_with_speech_is_as_before():
    """Одна итерация (без ``query``): ``play_animation`` и речь в одной команде - сцена на итоге, как было."""
    ports = Ports([Answer(Command(speech="Победа!", actions=(VICTORY,)), "deepseek")])
    _run(ports)
    (_, face), = ports.faces
    assert face == Shown(emotion="victory", duration_s=5.0)


def test_failed_action_of_the_later_iteration_leaves_the_reflex():
    """«Радостно» о провале не бывает: действие второй итерации провалилось - лица команды нет, а лицо итога - грусть
    фразы провала, а не рефлекс ``happy`` (#4069 п.8)."""
    class Failing(Ports):
        async def execute(self, actions, snapshot, turn_id):
            done = await super().execute(actions, snapshot, turn_id)
            if any(a.tool == "stop_music" for a in actions):
                return [ActionResult(r.turn_id, r.epoch, r.agent, r.tool, r.cls, "rejected") for r in done]
            return done

    ports = Failing([Answer(Command(actions=(VICTORY, TIME)), "deepseek"),
                     Answer(Command(speech=SPEECH, actions=(ToolCall("stop_music", {}),)), "deepseek")])
    _run(ports)
    assert ports.faces == [("done", SAD_FACE)]


@pytest.mark.parametrize("text", [
    N905, "а теперь победную анимацию", "победная анимация", "анимацию победы", "включи анимацию радости",
    "а теперь грустную анимацию", "покажи победу", "анимацию sleep",
])
def test_animation_named_without_show_verb_is_a_show_request(text):
    """«<имя анимации> анимацию» и «анимацию <имени>» - просьба показа без «покажи»: лицо без слов - норма, время сцены -
    по правилу показа (``FACE_SHOW_MIN_S``)."""
    assert show_asked(text)
    assert not needs_words(text, Command(actions=(VICTORY,)))


@pytest.mark.parametrize("text", ["как дела", "который час", "что ты умеешь"])
def test_chat_is_not_a_show_request(text):
    assert not show_asked(text)
    assert needs_words(text, Command(actions=(VICTORY,)))


def test_victory_is_a_scene_of_its_own():
    """``victory`` - своя сцена лица (не ``happy``): в словаре анимаций, в эмоциях команды и в сцене показа."""
    from rob_box_dialog.expression import command_scene, shown_scene

    assert "victory" in kn.KNOWN_ANIMATIONS
    assert {kn.animation_name(w) for w in ("победа", "победу", "победную", "победное", "победный")} == {"victory"}
    assert command_scene("victory") == "victory" == shown_scene("victory") != "happy"


def test_victory_adjective_is_a_name_in_the_show_queue():
    """Слово из n905 («победную») стоит в словаре алиасов: «покажи победную анимацию и грусть» - очередь кода из двух сцен,
    а не путь модели с неизвестным словом."""
    from rob_box_dialog.grammar import show_queue_scenes

    assert show_queue_scenes("покажи победную анимацию и грусть") == ("victory", "sad")
