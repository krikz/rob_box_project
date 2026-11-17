"""«Песня про енота» после заказа «про енота-механика» - ссылка на только что сочинённую, её ставит код (М11 n707).

Марафон М11 05.11: 02:19:04 сочинён «Енот-механик в мастерской» (``e7dd8301``), 02:20:48 человек сказал «Робот,
расскажи про песню про енота: кто, про что, сколько длится.», а модель позвала ``music_library info`` по песне прошлого
прогона (``3feb08b7``). «Эта песня» в реплике нет - прежнее правило (#4216) не сработало. Теперь тема реплики сверяется с
``theme`` последнего заказа ``generate_music`` этого разговора: совпала - ``track_ref=last_generated`` (id кладёт тул).
Ход целиком на фейковых портах: проверяется, какие аргументы ушли владельцу.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import track_ref
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine

OLD = "3feb08b7aa5c4e0f9d1c2b3a4e5f6a7b"  # песня М10 (из лога n707 - префикс id)
ORDER = "Робот, сгенерируй песню своим голосом про енота-механика."
N707 = "Робот, расскажи про песню про енота: кто, про что, сколько длится."
REF = {"track_ref": "last_generated"}


class Ports:
    """Транспорт хода: ответы LLM по очереди, исполнитель записывает вызовы и отвечает ``status``."""

    def __init__(self, answers, gen_status="done"):
        self.answers = list(answers)
        self.gen_status = gen_status
        self.executed, self.said = [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                             self.gen_status if a.tool == kn.GENERATE_TOOL else "done",
                             "provider_failed" if a.tool == kn.GENERATE_TOOL and self.gen_status != "done" else None,
                             dict(a.args)) for n, a in enumerate(actions)]

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


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def _info():
    return ToolCall("music_library", {"action": "info", "track_id": OLD})


def _library_calls(text, theme="енот-механик", gen_status="done", after_s=104.0):
    """Заказ генерации с ``theme``, через ``after_s`` секунд реплика ``text``, на неё модель зовёт ``info`` по
    старой песне. Вызовы ``music_library``, что ушли владельцу."""
    gen = ToolCall("generate_music", {"kind": "song", "theme": theme})
    # заказ: фразу о старте строит код по исходу тула, второй ответ модели не нужен
    ports = Ports([Answer(Command(actions=(gen,)), "deepseek"),
                   Answer(Command(actions=(_info(),)), "deepseek"), Answer(Command(speech="Готово."), "deepseek")],
                  gen_status=gen_status)
    ids = iter(f"t{n}" for n in range(1, 50))
    clock = Clock()
    engine = TurnEngine(Session("personality", clock=clock), ports, system_prompt="Ты робот.", clock=clock,
                        new_id=lambda: next(ids))
    asyncio.run(engine.run(Addressed(ORDER, VOICE)))
    clock.now += after_s
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return [c for batch in ports.executed for c in batch if c[0] == "music_library"]


def test_n707_song_named_by_the_order_theme_gets_the_reference():
    """Живая реплика n707: тема «енот» - из заказа «енот-механик»; ``track_id`` модели остаётся, тул его заменит."""
    assert _library_calls(N707) == [("music_library", {"action": "info", "track_id": OLD, **REF})]


@pytest.mark.parametrize("text", [
    "Робот, включи песню про механика", "Робот, что за трек про енотов?", "Робот, расскажи про песню о еноте",
    "Робот, включи песню про ЕНОТА с начала",
])
def test_other_wordings_of_the_order_theme_get_the_reference(text):
    assert _library_calls(text)[0][1].get("track_ref") == "last_generated"


@pytest.mark.parametrize("text", [
    "Робот, расскажи про песню про кота",  # тема не та
    "Робот, включи вчерашнюю песню про енота",  # названа старая версия
    "Робот, расскажи про другую песню про енота",
    "Робот, а еноты умеют чинить машины?",  # о песне не говорит
])
def test_without_the_order_theme_the_call_is_untouched(text):
    assert _library_calls(text) == [("music_library", {"action": "info", "track_id": OLD})]


def test_failed_order_gives_no_theme():
    assert _library_calls(N707, gen_status="rejected") == [("music_library", {"action": "info", "track_id": OLD})]


def test_order_older_than_the_window_gives_no_theme():
    calls = _library_calls(N707, after_s=track_ref.ORDER_FRESH_S + 1)
    assert calls == [("music_library", {"action": "info", "track_id": OLD})]


def test_wake_word_is_an_address_not_the_theme():
    """Песня «про робота»: «Робот, включи песню про дождь» - обращение, а не тема заказа."""
    assert _library_calls("Робот, включи песню про дождь", theme="робот") == [
        ("music_library", {"action": "info", "track_id": OLD})]
    assert _library_calls("Робот, включи песню про робота", theme="робот")[0][1].get("track_ref") == "last_generated"


def test_names_generated_topic_without_order_is_false():
    assert not track_ref.names_generated_topic(N707, ())
