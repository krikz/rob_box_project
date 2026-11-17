"""#4207 (М9 04.11, акт 8, n806 → n807): реплика, которая ещё звучит, видна в окне LLM словами, а не «ответом лицом».

Живой лог Sound Pi (UTC): n806 «посчитай вслух от одного до двадцати спокойно» - модель ответила речью
«Один. Два. … Двадцать.» (192 знака, лицо команды happy). В 00:07:04 пришла правка n807 «и после каждого числа добавляй
слово шаг», счёт звучал (154 из 192 знаков, ``finished`` - только в 00:07:07). В окне модели на ход счёта стояло
``[ответ лицом: сцена лица — показано]``: реплика попадала в окно лишь по ``finished``, а до него ход считался отвеченным
одним лицом. Модель не знала, что считает, и начала заново: «Начинаю, Саша. Один шаг.».

Решает код (ADR-0148): реплика, отданная владельцу речи и ещё не отзвучавшая, стоит в окне меткой
``knowledge.WINDOW_SOUNDING_REPLY`` с её текстом; метка «ответ лицом» у такого хода не ставится. Отзвучала - на её месте
озвученная часть, как раньше. «Повтори» звучащую реплику не берёт: она ещё не прозвучала.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.session import Session
from rob_box_dialog.turn_log import ASSISTANT, TOOL, USER, TurnLog

N806 = "посчитай вслух от одного до двадцати спокойно"
N807 = "и после каждого числа добавляй слово шаг"
#: Ответ модели на n806 в М9 (LLM RESPONSE 00:06:46), целиком.
COUNT = ("Один. Два. Три. Четыре. Пять. Шесть. Семь. Восемь. Девять. Десять. Одиннадцать. Двенадцать. Тринадцать. "
         "Четырнадцать. Пятнадцать. Шестнадцать. Семнадцать. Восемнадцать. Девятнадцать. Двадцать.")
SOUNDING = kn.WINDOW_SOUNDING_REPLY.format(text=COUNT)


def _dialogue(window):
    return [(e.role, e.content) for e in window if e.role != TOOL]


# -- журнал -----------------------------------------------------------------------------------------------------------


def test_sounding_speech_is_in_the_window_not_the_face_reply():
    """n806 в момент правки: реплика отдана, ``finished`` не пришёл, лицо команды показано."""
    log = TurnLog()
    log.begin("t1", 0, N806, "Саша")
    log.said("t1", "s1", COUNT)
    log.faced("t1", kn.WINDOW_FACE_SCENE)
    assert _dialogue(log.window()) == [(USER, N806), (ASSISTANT, SOUNDING)]


def test_after_finished_the_spoken_text_replaces_the_mark():
    log = TurnLog()
    log.begin("t1", 0, N806)
    log.said("t1", "s1", COUNT)
    log.faced("t1", kn.WINDOW_FACE_SCENE)
    log.spoken("s1", len(COUNT))
    assert _dialogue(log.window()) == [(USER, N806), (ASSISTANT, COUNT)]


def test_cut_speech_leaves_only_the_spoken_part():
    log = TurnLog()
    log.begin("t1", 0, N806)
    log.said("t1", "s1", COUNT)
    log.spoken("s1", len("Один. Два."))
    assert _dialogue(log.window()) == [(USER, N806), (ASSISTANT, "Один. Два.")]


def test_unsaid_speech_with_nothing_heard_falls_back_to_the_face_reply():
    """Реплика снята, не прозвучав (``cancelled`` с ``spoken_chars=0``): ход отвечен лицом, как было."""
    log = TurnLog()
    log.begin("t1", 0, N806)
    log.said("t1", "s1", COUNT)
    log.faced("t1", "happy")
    log.spoken("s1", 0)
    assert _dialogue(log.window()) == [(USER, N806), (ASSISTANT, kn.WINDOW_FACE_REPLY.format(face="happy"))]


def test_sounding_speech_is_not_what_repeat_says():
    log = TurnLog()
    log.begin("t1", 0, "как дела")
    log.said("t1", "s0", "Отлично!")
    log.spoken("s0", 8)
    log.begin("t2", 0, N806)
    log.said("t2", "s1", COUNT)
    assert log.last_spoken() == "Отлично!"


def test_filler_still_sounding_is_not_shown_as_the_answer():
    """Заглушка ожидания («Сейчас, соображаю.») - не ответ хода: пока звучит, в окне её нет."""
    log = TurnLog()
    log.begin("t1", 0, N806)
    log.said("t1", "s1", "Сейчас, соображаю.", filler=True)
    assert _dialogue(log.window()) == [(USER, N806)]


# -- ход целиком: что видит модель на n807 -----------------------------------------------------------------------------


class Ports:
    """Транспорт хода: ответы LLM по очереди; ``SpeechEvent`` от владельца речи не приходит - реплика звучит."""

    def __init__(self, answers):
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


def test_n807_model_sees_the_count_that_is_still_sounding():
    """М9 n806 → n807: второй ход спрашивает модель, пока счёт первого звучит. В истории - счёт, а не лицо."""
    ports = Ports([Answer(Command(speech=COUNT, expression=Expression(emotion="happy")), "deepseek"),
                   Answer(Command(speech="Восемнадцать шаг."), "deepseek")])
    ids = iter(f"t{n}" for n in range(1, 100))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    asyncio.run(engine.run(Addressed(N806, VOICE)))
    assert ports.said == [COUNT]
    asyncio.run(engine.run(Addressed(N807, VOICE)))
    history = ports.asked[1]
    assert [m["role"] for m in history] == ["system", "user", "assistant", "user"]
    assert history[2]["content"] == SOUNDING
    assert "ответ лицом" not in history[2]["content"]
