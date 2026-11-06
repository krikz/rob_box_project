"""Смену темы играющего сета строит код из слов человека (#4347, М10 n604, ADR-0148).

Лог М10 (run 41827304636, n604_dj_rock): гард ``music_promise`` отбил первую попытку, повтор модели снова обещал
(«заявку принял: рок будет») без ``dj_set`` - только ``play_animation`` - и был принят как ``command``: пустое обещание
прозвучало, тема сета не сменилась. Здесь двойник LLM отвечает ходу так же (её повторы уже потрачены мостом), и
проверяется поведение хода: обещание не звучит, смену темы исполняет код.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Invalid, ToolCall
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.theme_switch import with_switch
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import music_promises, theme_switch_ask

#: n604 М10 дословно: реплика человека и ответ модели на повторе (``verdict=command actions=['play_animation']``).
N604_HEARD = "Робот, уводи в сторону рока, тётя Гриша и подруги добреют"
N604_PROMISE = "Тётю Гришу и подруг не знаю, но заявку принял: рок будет…"
FACE = ToolCall("play_animation", {"name": "happy"})


class Ports:
    """Транспорт хода: ответы LLM по очереди, исполненное и сказанное."""

    def __init__(self, answers):
        self.answers, self.executed, self.said = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        from rob_box_dialog.respond import ActionResult

        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool], "done",
                             None, {}) for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *a, **kw):
        pass

    def cancel_speech(self, *a, **kw):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, *a, **kw):
        pass

    def perform(self, *a, **kw):
        return None


def _run(answer, text=N604_HEARD, dj=True):
    ports = Ports([Answer(answer, "deepseek")])
    ids = iter(f"t{n}" for n in range(1, 100))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    engine.session._snap = engine.session.snapshot.__class__(
        **{**vars(engine.session.snapshot), "music": MusicStateRef("playing")})
    if dj:
        engine.session.push_scene("DJ", "code")
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return ports


def _tools(ports):
    return [tool for batch in ports.executed for tool, _ in batch]


def test_n604_the_empty_promise_after_the_retry_is_not_spoken_and_the_code_switches_the_theme():
    """Сам n604 М10: модель дважды обещала без тула - обещание не звучит, ``dj_set`` с темой человека кладёт код."""
    assert music_promises(N604_PROMISE) == ()  # слов обещания из таблицы нет: гард текста модели его не ловит
    ports = _run(Command(speech=N604_PROMISE, actions=(FACE,)))
    assert N604_PROMISE not in ports.said
    assert ports.executed == [[("play_animation", {"name": "happy"}), ("dj_set", {"action": "start", "theme": "рока"})]]


def test_the_retry_rejected_again_by_music_promise_still_switches_the_theme_by_code():
    """Повтор тоже отбит гардом (``Invalid music_promise``, исполняется ``salvage``): смену исполняет код."""
    invalid = Invalid(kn.VIOLATION_MUSIC_PROMISE, "speech обещает switch без тула", salvage=(FACE,))
    ports = _run(invalid)
    assert ports.executed == [[("play_animation", {"name": "happy"}), ("dj_set", {"action": "start", "theme": "рока"})]]
    assert phrases.text(kn.MUSIC_PROMISE_PHRASES["playing"]) not in ports.said


def test_a_switch_without_a_style_is_not_guessed_the_code_says_it_honestly():
    ports = _run(Command(speech="Будет сделано, меняю!"), text="Робот, смени музыку")
    assert ports.said == [phrases.text(kn.THEME_SWITCH_FAILED_PHRASE)] and ports.executed == []


def test_without_a_playing_set_the_command_stays_as_it_is():
    ports = _run(Command(speech="Рок - отличный выбор."), dj=False)
    assert ports.said == ["Рок - отличный выбор."] and ports.executed == []


def test_the_models_own_dj_set_is_kept_and_not_doubled():
    own = ToolCall("dj_set", {"action": "start", "theme": "рок"})
    ports = _run(Command(actions=(own,)))
    assert _tools(ports) == ["dj_set"] and ports.executed[0][0][1]["theme"] == "рок"


@pytest.mark.parametrize("text, theme", [
    (N604_HEARD, "рока"),
    ("Переключи на джаз", "джаз"),
    ("сворачивай в техно", "техно"),
    ("Робот, смени музыку", ""),
    ("поменяй тему сета", ""),
    ("Робот, уведи в сторону, тётя Гриша", None),  # стиля и слова о музыке нет - не смена темы
    ("поставь рок", None),                          # заказ, не смена: решает модель
    ("сверни налево", None),
    ("попробуй ещё раз", None),
])
def test_theme_switch_ask_reads_the_words_of_the_human(text, theme):
    assert theme_switch_ask(text) == theme


def test_a_reading_or_asking_model_keeps_its_turn():
    read = Command(actions=(ToolCall("music_library", {"action": "list"}),))
    assert with_switch(True, N604_HEARD, read) == (read, "")
    asks = Command(speech="В какой рок?", ask=None)
    assert with_switch(True, N604_HEARD, asks)[1] == "built"
    acted = (ToolCall("dj_set", {"action": "start", "theme": "рок"}),)
    assert with_switch(True, N604_HEARD, Command(speech=N604_PROMISE), acted)[1] == ""
