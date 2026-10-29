"""«Эта песня» после генерации - ссылка на только что сочинённую, её ставит код (#4216, ADR-0148).

Марафон-5 29.10 (n707/n708): робот сочинил «Енота-механика», человек сказал «включи эту песню про енота», а модель
отправила ``music_library`` с ``track_id`` вчерашней версии из списка. Реплика с «эта/эту песню» получает вызов
``music_library`` с ``track_ref=last_generated`` (id трека знает тул, ``MusicLibraryTool``); других реплик и других действий
это не касается. Ход целиком на фейковых портах: проверяется, какие аргументы ушли владельцу.
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

OLD = "b90b61bfb32f46569ee5ebae59dca82d"
REF = {"track_ref": "last_generated"}


class Ports:
    """Транспорт хода: ответы LLM по очереди, исполнитель записывает вызовы и отвечает ``done``."""

    def __init__(self, answers=()):
        self.answers = list(answers)
        self.executed, self.said = [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool], "done",
                             None, dict(a.args)) for n, a in enumerate(actions)]

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


def _call(action, **args):
    return ToolCall("music_library", {"action": action, "track_id": OLD, **args})


def _executed(text, call):
    """Вызовы, что ушли владельцу, когда модель на ``text`` позвала ``call``."""
    ports = Ports([Answer(Command(actions=(call,)), "deepseek"), Answer(Command(speech="Готово."), "deepseek")])
    ids = iter(f"t{n}" for n in range(1, 50))
    engine = TurnEngine(Session("personality", clock=lambda: 1000.0), ports, system_prompt="Ты робот.",
                        new_id=lambda: next(ids))
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return [c for batch in ports.executed for c in batch if c[0] == "music_library"]


@pytest.mark.parametrize("text", [
    "Робот, включи эту песню про енота с начала.", "Робот, что за песня? Расскажи про эту песню.",
    "Робот, поставь эту песню ещё раз", "Робот, а этот трек про что?", "Робот, включи эту новую песню",
    "Робот, включи только что сочинённую песню", "Робот, расскажи про песню, которую ты только что написал",
    "Робот, ЭТУ ПЕСНЮ включи",
])
def test_this_song_gets_the_reference_and_the_models_track_id_stays_for_the_tool_to_replace(text):
    sent = _executed(text, _call("info"))
    assert sent == [("music_library", {"action": "info", "track_id": OLD, **REF})]


@pytest.mark.parametrize("text", [
    "Робот, включи песню про кота", "Робот, расскажи про песню про енота", "Робот, что у тебя есть сгенерированного?",
    "Робот, включи эти песни по кругу", "Робот, в этом году я слушал много песен", "Робот, только что пришёл домой",
    "Робот, на этой неделе включи песню про дождь", "Робот, это что за трек",
])
def test_without_the_this_wording_the_call_is_untouched(text):
    sent = _executed(text, _call("play"))
    assert sent == [("music_library", {"action": "play", "track_id": OLD})]


@pytest.mark.parametrize("action", ["info", "play", "update", "delete"])
def test_id_actions_get_the_reference(action):
    assert track_ref.bind_last_generated([_call(action)], "включи эту песню")[0].args["track_ref"] == "last_generated"


@pytest.mark.parametrize("call", [
    ToolCall("music_library", {"action": "list"}), ToolCall("music_library", {"action": "search", "query": "енот"}),
    ToolCall("music_library", {"action": "rate", "verdict": "like"}), ToolCall("stop_music", {}),
    ToolCall("generate_music", {"kind": "song", "theme": "про эту песню"}),
])
def test_other_actions_and_tools_are_not_touched(call):
    assert track_ref.bind_last_generated([call], "Робот, оцени эту песню") == (call,)


def test_only_library_calls_of_a_mixed_command_are_bound():
    stop, info = ToolCall("stop_music", {}), _call("info")
    bound = track_ref.bind_last_generated([stop, info], "останови и расскажи про эту песню")
    assert bound[0] is stop and bound[1].args == {**info.args, **REF}


def test_reference_matches_the_tool_schema():
    """Значение ссылки - из схемы тула (каталог, из которого модель видит тул): код и схема не расходятся."""
    from rob_box_core.tool_catalog import get_tool

    enum = get_tool("music_library").parameters["properties"][track_ref.REF_ARG]["enum"]
    assert track_ref.LAST_GENERATED in enum and set(enum) == {"", track_ref.LAST_GENERATED}


def test_confirmed_delete_is_an_answer_about_the_asked_track_and_is_not_redirected():
    call = _call("delete", confirm=True)
    assert track_ref.bind_last_generated([call], "Робот, да, удаляй эту песню") == (call,)
