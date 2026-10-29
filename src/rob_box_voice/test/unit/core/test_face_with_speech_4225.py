"""#4225 (марафон-6 29.10, dev-980c053, акт 9, n904): лицо команды в речевом ходе доходит до лица.

Факт: ``LLM RESPONSE command {"expression": {"emotion": "sad", "face": {eyes: half, gaze: down, mouth: frown, tint:
blue, …}, "duration_s": 15}, "speech": "…Глаза опустил, свечусь синим — вот так мне грустно."}`` → ``🧾 [command]
verdict=speech actions=[]`` → строки «🎭 лицо команды на done» нет → ``face_node: сцена по запросу: happy -> happy (3.0
с)`` (рефлекс ``done`` без окраски). Все 58 ходов «речь + эмоция» прогона - так же. Причина: мост команд
(``command_bridge.to_legacy``) отдавал реплику без действий ТЕКСТОМ, а ``dialog_v2_ports.to_result`` собирал из текста
``Command(speech=…)`` - ``expression`` терялась по дороге, ход красить итог было нечем. Ход с одной ``expression`` (без
речи) шёл вызовом ``command`` (#3974) - поэтому строка «лицо команды на done» была только у ходов без речи.

Тесты гонят ответ модели через НАСТОЯЩИЙ мост (``CommandModeLLM``) и ``V2Ports`` до ``ExpressionRequest`` итога, а не
подставляют готовую команду мимо моста (так прежние тесты #3997/#4049 и пропустили потерю).
"""

from __future__ import annotations

import asyncio
import json
import types

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.address import VOICE, Addressed  # noqa: E402
from rob_box_dialog.command import Command, Expression, parse  # noqa: E402
from rob_box_dialog.expression import reflex  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_dialog.turn import TurnEngine  # noqa: E402
from rob_box_llm.provider import LLMResponse, ToolCall  # noqa: E402
from rob_box_llm.providers.deepseek import DeepSeekProvider  # noqa: E402
from rob_box_voice.core import command_bridge as cb  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports, counted_wrapper, to_result  # noqa: E402

TIME = type(M.TurnEvent().stamp)
#: Реплика n904 как её слышит ход (вейк снимает адресация) и ответ модели из лога (речь - полностью).
N904 = "покажи как тебе грустно что смена такая длинная"
SPEECH = ("Ох, Саша… смена тянется уже полтора часа, а я всё катаюсь и катаюсь. "
          "Глаза опустил, свечусь синим — вот так мне грустно.")
FACE = {"eyes": "half", "gaze": "down", "mouth": "frown", "tint": "blue", "tempo": "slow", "rear": "off", "bg": "none"}
N904_ARGS = {"expression": {"emotion": "sad", "face": FACE, "earcon": "heard", "duration_s": 15}, "speech": SPEECH}


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Node:
    def __init__(self):
        self.pubs = {}

    def create_publisher(self, msg_type, topic, depth):
        return self.pubs.setdefault(topic, _Pub())

    def get_clock(self):
        stamp = TIME(sec=1, nanosec=0)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))


class _DeepSeek:
    """Провайдер DeepSeek (его capabilities: мост в режиме ``tool_choice``), ответы по очереди."""

    name = "deepseek"
    model = "deepseek-chat"

    def __init__(self, replies):
        self.replies = list(replies)
        self.capabilities = DeepSeekProvider(api_key="test").capabilities

    def capabilities_for(self, model):
        return self.capabilities

    async def complete(self, messages, *, tools=(), settings=None):
        return self.replies.pop(0)


class _Tools:
    async def execute(self, call):  # речевой ход действий не исполняет
        raise AssertionError(call)


def _turn(args, text=N904):
    """Ход Личности: ответ модели ``args`` (вызов ``command``) через мост команд и ``V2Ports``."""
    lines, node, counter = [], _Node(), CallCounter()
    llm = counted_wrapper(cb.wrapper_for("deepseek", log=lines.append), counter)(
        _DeepSeek([LLMResponse(tool_calls=(ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments=args),))]))
    ports = V2Ports(node, llm, counter, _Tools(), log=lines.append)
    agg = asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))
    return agg, node, lines


def _done(node):
    """``ExpressionRequest`` итога хода: последний на топике."""
    return node.pubs["/voice/expression/request"].sent[-1]


def test_n904_speech_with_sad_face_reaches_the_face_on_done_not_the_happy_reflex():
    agg, node, lines = _turn(N904_ARGS)
    assert (agg.stage, agg.status) == ("done", "finished") and agg.llm_calls == 1
    assert [s.text for s in node.pubs["/voice/speech/request"].sent] == [f"<speak>{SPEECH}</speak>"]
    done = _done(node)
    assert done.emotion == "sad" != reflex("done").emotion, "рефлекс done (happy) не перетирает лицо команды"
    assert done.ttl_ms == 15000 > reflex("done").ttl_ms, "время показа - duration_s команды, а не 3 с рефлекса"
    assert json.loads(done.face_json) == {**FACE, "duration_s": 15.0}
    assert done.turn_id == agg.turn_id
    assert f"🎭 [turn-v2] лицо команды на done: emotion=sad gesture=- face=llm ttl_ms=15000 turn={agg.turn_id}" in lines
    assert any("verdict=speech actions=[]" in line for line in lines), "ответ модели - речь, вердикт лога тот же"


def test_ordinary_reply_with_an_emotion_colours_done_with_it():
    """Обычный ответ (показ не просили, время не задано): эмоция модели - на итоге, время рефлекса."""
    agg, node, lines = _turn({"speech": "Сейчас посмотрю, подожди.", "expression": {"emotion": "thinking"}},
                             text="робот а который сейчас год по твоему")
    done = _done(node)
    assert (done.emotion, done.ttl_ms, done.face_json) == (kn.EMOTIONS["thinking"], reflex("done").ttl_ms, "")
    assert any("лицо команды на done: emotion=" in line for line in lines)


def test_reply_without_expression_is_still_spoken_text_and_the_plain_reflex():
    """Голая реплика (``expression`` не задана) - как раньше: текст моста, рефлекс ``done`` как есть."""
    text, calls, verdict = cb.to_legacy(Command(speech="Привет!"))
    assert (text, calls, verdict) == ("Привет!", (), "speech")
    agg, node, lines = _turn({"speech": "Привет!"}, text="робот привет")
    assert _done(node).emotion == reflex("done").emotion
    assert not any("лицо команды" in line for line in lines)


def test_bridge_gives_speech_with_face_as_one_command_call_that_round_trips():
    cmd = parse(N904_ARGS)
    text, calls, verdict = cb.to_legacy(cmd)
    assert (text, verdict, [c.name for c in calls]) == ("", "speech", [kn.COMMAND_TOOL])
    back = to_result(LLMResponse(tool_calls=calls))
    assert back.speech == SPEECH and back.expression == cmd.expression
    assert back.expression.emotion == "sad" and back.expression.duration_s == 15.0


def test_actions_and_voice_keep_the_command_verdict():
    """Вердикт ``command`` - у ответа с действием или голосом реплики, с лицом или без."""
    face = {"emotion": "happy"}
    acted = parse({"speech": "Включаю.", "actions": [{"tool": "play_sound", "args": {"sound": "boop"}}],
                   "expression": face})
    assert cb.to_legacy(acted)[2] == "command"
    assert cb.to_legacy(Command(expression=Expression(emotion="sad")))[2] == "command", "лицо без слов (#3974)"
