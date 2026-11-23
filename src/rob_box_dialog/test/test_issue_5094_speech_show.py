"""#5094: «анекдот и анимация» — составная просьба, код не выбирает одну из двух (ADR-0148).

Живой n906 (run 44999727503): модель прислала одну ``play_animation``, речь пропала. Отказ одной
мигалки (``speech_asked``) уже на ``develop`` и анекдот не сочиняет. Здесь — обратная дыра: модель
дала речь или выступление и забыла сцену. Сцену добавляет код из слов человека, не из ответа модели.

Порядок документирован и проверяется: действия исполняются первыми (``_execute``), затем речь
(``_say``) и выступление (``hand_perform``). Воспроизведение анимации ход не ждёт.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog import speech_show
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Performance, ToolCall
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine

N906 = "Робот, расскажи анекдот и покажи под него весёлую анимацию."
TELL = "расскажи анекдот"
SHOW = "покажи анимацию"
JOKE = "Заходит робот в бар."
HAPPY = ("play_animation", {"animation": "happy"})
SAD = ("play_animation", {"animation": "sad"})


class Ports:
    """Ответ модели один; порядок доказательств — список ``order``: сначала действия, потом речь."""

    def __init__(self, answer):
        self.answer = answer
        self.executed, self.said, self.performed, self.order = [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answer

    async def execute(self, actions, snapshot, turn_id):
        from rob_box_dialog.respond import ActionResult

        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        self.order.append("actions")
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
        self.said.append(text)
        self.order.append("speech")
        return "sp"

    def perform(self, turn_id, epoch, perf, edit):
        self.performed.append(list(perf.segments))
        self.order.append("perform")
        return {"status": "started"}

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass


def _run(text, cmd):
    ports = Ports(Answer(cmd, "deepseek"))
    asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))
    return ports


def test_n906_speech_without_animation_gains_the_happy_scene_and_keeps_the_joke():
    """Дословный n906, модель дала только речь: код добавляет ``happy``, шутку не подменяет.

    Порядок: сцена исполняется первой, речь звучит после. Анекдот — слова модели, не код.
    """
    ports = _run(N906, Command(speech=JOKE))
    assert ports.executed == [[HAPPY]]
    assert JOKE in ports.said
    assert ports.order.index("actions") < ports.order.index("speech")


def test_n906_perform_without_animation_gains_the_scene_and_hands_the_performance():
    """Модель ответила выступлением без сцены: выступление остаётся, сцена добавляется.

    Порядок: действия, затем выступление владельцу. Воспроизведение ход не ждёт.
    """
    told = Command(perform=Performance(kind="tale", segments=(JOKE,)))
    ports = _run(N906, told)
    assert ports.executed == [[HAPPY]]
    assert ports.performed == [[JOKE]]
    assert ports.order.index("actions") < ports.order.index("perform")


def test_both_already_present_are_not_duplicated():
    """Речь и сцена уже в команде — код ничего не добавляет и шутку не трогает."""
    ports = _run(N906, Command(speech=JOKE, actions=(ToolCall("play_animation", {"animation": "happy"}),)))
    assert ports.executed == [[HAPPY]]
    assert ports.said == [JOKE]


def test_a_named_scene_is_kept_and_a_second_is_not_added():
    """Модель уже заказала названную сцену — второй ``play_animation`` нет."""
    ports = _run(
        "расскажи анекдот и покажи грусть",
        Command(speech=JOKE, actions=(ToolCall("play_animation", {"animation": "sad"}),)),
    )
    assert ports.executed == [[SAD]]


def test_a_named_scene_the_model_skipped_is_taken_from_the_words():
    """«Покажи грусть» и речь без сцены: имя берётся из реплики, не из ``DEFAULT_SHOW``."""
    ports = _run("расскажи анекдот и покажи грусть", Command(speech=JOKE))
    assert ports.executed == [[SAD]]
    assert JOKE in ports.said


def test_a_bare_tell_does_not_gain_an_animation():
    """Обычная одиночная команда «расскажи анекдот»: речи довольно, сцену код не выдумывает."""
    ports = _run(TELL, Command(speech=JOKE))
    assert ports.executed == []
    assert ports.said == [JOKE]


def test_a_bare_show_stays_a_silent_blink():
    """Обычная одиночная команда «покажи анимацию»: мигалка без речи, как просили (#4021)."""
    ports = _run(SHOW, Command(actions=(ToolCall("play_animation", {"animation": "happy"}),)))
    assert ports.executed == [[HAPPY]]
    assert ports.said == []


def test_negated_tell_with_a_show_does_not_become_a_compound():
    """«Не рассказывай, покажи анимацию» — отказ говорить, не составная просьба."""
    ports = _run(
        "не рассказывай, просто покажи анимацию", Command(actions=(ToolCall("play_animation", {"animation": "happy"}),))
    )
    assert ports.executed == [[HAPPY]]
    assert ports.said == []
    assert not speech_show.compound("не рассказывай, просто покажи анимацию")


def test_with_show_skips_a_query_and_an_edit_and_salvage_still_adds_the_scene():
    """Пока модель читает или правит идущее — не вмешиваемся. Спасённые действия сцену получают."""
    reading = Command(actions=(ToolCall("music_library", {"action": "list"}),))
    assert speech_show.with_show(N906, reading)[1] == ""
    editing = Command(edit_pending=True, perform=Performance(kind="tale", segments=(JOKE,)))
    assert speech_show.with_show(N906, editing)[1] == ""
    salvage = speech_show.salvage_with_show(N906, ())
    assert [(c.tool, dict(c.args)) for c in salvage] == [HAPPY]


def test_veseluyu_is_happy_and_an_unknown_word_is_not_a_scene():
    """«Весёлую» — строка словаря, не догадка. Слова вне словаря сценой не становятся."""
    assert kn.animation_name("весёлую") == "happy"
    assert kn.animation_name("веселую") == "happy"
    assert kn.animation_name("анекдот") is None
    assert speech_show.scene_named(N906) == "happy"
    assert speech_show.scene_named("расскажи анекдот и покажи анимацию") is None
