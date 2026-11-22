"""#5020: «сирену и мигалку» исполняет код, не модель (ADR-0148).

Живой n903 (марафон №3, 22.11): даже дословная реплика «сирену и мигалку покажи» дала только
``play_animation``. Перепись ``ambulance`` → ``police_lights`` (#4694) звук не добавляет.
Что человек назвал, код читает из ``knowledge.SOUND_ALIASES_RU`` и ставит вызов сам.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog import siren_ask
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine

N_N903 = "Робот, а теперь сирену и мигалку покажи, напугаем Валеру."
ANIM = ToolCall("play_animation", {"animation": "police_lights"})
AMBULANCE = ToolCall("play_animation", {"animation": "ambulance"})
SOUND = ToolCall("play_sound", {"sound": "robot_alert"})
BOTH = (("play_animation", {"animation": "police_lights"}), ("play_sound", {"sound": "robot_alert"}))


class Ports:
    """Ответы модели по очереди; исполненные вызовы — доказательство, что звук и сцена дошли до тула."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.executed = []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        from rob_box_dialog.respond import ActionResult

        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        return [
            ActionResult(
                f"{turn_id}/{n}",
                snapshot.epoch,
                snapshot.agent,
                a.tool,
                kn.TOOL_CLASSES[a.tool],
                "done",
                None,
                dict(a.args),
            )
            for n, a in enumerate(actions)
        ]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        return "sp"

    def stage(
        self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="", expression=None
    ):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(text, actions):
    ports = Ports([Answer(Command(speech="Включаю!", actions=actions), "deepseek")])
    asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))
    return ports.executed


def test_n903_text_with_only_animation_gets_the_siren_sound_and_police_lights():
    """Дословный n903 и одна анимация модели: код добавляет ``play_sound{robot_alert}`` и оставляет мигалку."""
    assert _run(N_N903, (ANIM,)) == [list(BOTH)]


def test_n903_ambulance_becomes_police_lights_and_gains_the_siren():
    """Модель назвала ``ambulance``: #4694 переписывает сцену, #5020 добавляет звук, без второй анимации."""
    assert _run(N_N903, (AMBULANCE,)) == [list(BOTH)]


def test_flasher_alone_paints_the_scene_and_does_not_play_a_sound():
    """«Покажи мигалку» — сцена лица, не звук. Модель промолчала действием — сцену ставит код."""
    assert _run("покажи мигалку", ()) == [[("play_animation", {"animation": "police_lights"})]]


def test_siren_without_flasher_plays_the_sound_and_does_not_force_the_scene():
    """«Сирена» без мигалки — только названный звук, сцену код не выдумывает."""
    assert _run("сирена", ()) == [[("play_sound", {"sound": "robot_alert"})]]


def test_named_sound_and_scene_already_present_are_not_duplicated():
    """Оба вызова уже в команде — код ничего не добавляет."""
    assert _run(N_N903, (ANIM, SOUND)) == [list(BOTH)]


def test_empty_and_noise_do_not_add_a_call_and_do_not_break():
    """Пусто и шум: ни звука, ни сцены, исключения нет."""
    assert _run("", ()) == []
    assert _run("ааа", ()) == []
    assert _run("привет", ()) == []


def test_explicit_ambulance_does_not_gain_the_siren_sound():
    """«Включи сирену скорой помощи»: скорая остаётся скорой, ``robot_alert`` код не подкладывает."""
    assert _run("включи сирену скорой помощи", (AMBULANCE,)) == [[("play_animation", {"animation": "ambulance"})]]


def test_with_asked_skips_a_query_and_salvage_still_adds_the_sound():
    """Пока модель читает — не вмешиваемся. После исчерпания повторов спасённые действия получают звук."""
    reading = Command(actions=(ToolCall("music_library", {"action": "list"}),))
    assert siren_ask.with_asked("включи сирену", reading)[1] == ""
    salvage = siren_ask.salvage_with_asked("включи сирену и мигалку", ())
    assert [(c.tool, dict(c.args)) for c in salvage] == list(BOTH)
