"""#4225 хвост (марафон-7 29.10, dev-2aaa62b, акт 4, n404): «скажи этим голосом» после ``set_voice alena`` звучит Алёной.

Факт (docker logs voice-assistant, UTC):
```
06:40:54.621 set_voice {'voice': 'alena', 'provider': 'yandex'} → set_voice applied: Yandex voice_id=alena
06:41:18.480 STT: робот скажи этим голосом что нибудь бодрое чтобы я услышал разницу
06:41:20.635 LLM: command {"speech": "Ну что, Саша, утро начинается…", "voice": "al…"} → verdict=invalid:bad_enum detail='voice=alena'
06:41:22.184 TTS: speech_id=e4ac3b6f voice=arina …
06:41:23.690 SpeechEvent voice_changed voice_requested=arina voice_applied=male-qn-qingse provider=minimax
```
Причина: голос реплики сверялся с каталогом РОЛЕЙ (``plan_host.perform_busy``: русские голоса звучащего провайдера
без голоса робота - рассказчика), текущего голоса ``alena`` в нём нет нарочно → ``bad_enum`` → повтор с перечнем
нарушения → модель взяла ``arina``. Теперь голос не из каталога (и текущий голос робота) - лишнее поле
(``fields_dropped``), команда валидна с первого ответа, реплика звучит голосом сессии.

Тест гонит ответ модели через НАСТОЯЩИЙ мост (``CommandModeLLM``) с каталогом ``perform_busy`` ноды после ``set_voice``.
"""

from __future__ import annotations

import asyncio
import types

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.address import VOICE, Addressed  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_dialog.tools_view import voice_asked  # noqa: E402
from rob_box_dialog.turn import TurnEngine  # noqa: E402
from rob_box_llm.provider import LLMResponse, ToolCall  # noqa: E402
from rob_box_llm.providers.deepseek import DeepSeekProvider  # noqa: E402
from rob_box_voice.core import command_bridge as cb  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports, counted_wrapper  # noqa: E402
from rob_box_voice.core.plan_host import perform_busy  # noqa: E402

TIME = type(M.TurnEvent().stamp)
N404 = "робот скажи этим голосом что нибудь бодрое чтобы я услышал разницу"
SPEECH = "Ну что, Саша, утро начинается — давай бодро!"


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Node:
    """ROS-нода для портов и для ``perform_busy``: звучит Yandex, голос сессии - тот, что применил ``set_voice``."""

    def __init__(self, session):
        self.pubs, self._session, self._actual_tts_provider = {}, session, "yandex"

    def create_publisher(self, msg_type, topic, depth):
        return self.pubs.setdefault(topic, _Pub())

    def get_clock(self):
        stamp = TIME(sec=1, nanosec=0)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))


class _DeepSeek:
    name, model = "deepseek", "deepseek-chat"

    def __init__(self, replies):
        self.replies, self.asked = list(replies), 0
        self.capabilities = DeepSeekProvider(api_key="test").capabilities

    def capabilities_for(self, model):
        return self.capabilities

    async def complete(self, messages, *, tools=(), settings=None):
        self.asked += 1
        return self.replies.pop(0)


class _Tools:
    async def execute(self, call):
        raise AssertionError(call)


def _turn(args, text=N404):
    session = Session("personality")
    session.set_voice(requested="alena", applied="alena", provider="yandex")  # n403: set_voice alena (yandex)
    node, lines, counter = _Node(session), [], CallCounter()
    inner = _DeepSeek([LLMResponse(tool_calls=(ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments=args),)),
                       LLMResponse(tool_calls=(ToolCall(id="c2", name=kn.COMMAND_TOOL,
                                                        arguments={**args, "voice": "arina"}),))])
    llm = counted_wrapper(cb.wrapper_for("deepseek", log=lines.append, busy=lambda: perform_busy(node)), counter)(inner)
    ports = V2Ports(node, llm, counter, _Tools(), log=lines.append)
    agg = asyncio.run(TurnEngine(session, ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))
    return agg, node, lines, inner


def test_catalogue_is_the_role_voices_without_the_robot_voice():
    """Почему ``alena`` не в перечне: каталог голоса реплики - роли выступления, голос робота (рассказчик) исключён."""
    session = Session("personality")
    session.set_voice(requested="alena", applied="alena", provider="yandex")
    voices = perform_busy(_Node(session)).voices
    assert "alena" not in voices and "arina" in voices
    assert voice_asked(session.snapshot, N404), "«скажи этим голосом» - просьба о голосе: поле voice открыто"


def test_n404_speech_with_the_current_voice_is_valid_at_once_and_sounds_in_the_session_voice():
    agg, node, lines, inner = _turn({"speech": SPEECH, "voice": "alena", "expression": {"emotion": "happy"}})
    assert inner.asked == 1 and agg.llm_calls == 1, "без повтора: голос реплики - не нарушение"
    assert (agg.stage, agg.status) == ("done", "finished")
    (said,) = node.pubs["/voice/speech/request"].sent
    assert said.text == f"<speak>{SPEECH}</speak>"
    assert said.voice == "", "голос реплики не задан - TTS говорит голосом сессии (alena, yandex), а не чужим"
    (verdict,) = [line for line in lines if line.startswith(cb.COMMAND_LOG_MARK) and "verdict=" in line]
    assert "verdict=speech" in verdict and "fields_dropped=['voice']" in verdict and "invalid" not in verdict


def test_foreign_voice_name_is_dropped_too_and_never_reaches_tts():
    agg, node, lines, inner = _turn({"speech": SPEECH, "voice": "male-qn-qingse"})
    assert inner.asked == 1
    (said,) = node.pubs["/voice/speech/request"].sent
    assert said.voice == ""


def test_voice_from_the_role_catalogue_asked_by_name_still_sounds():
    """«Скажи голосом Арины» - голос из каталога уходит в реплику как раньше."""
    agg, node, lines, inner = _turn({"speech": SPEECH, "voice": "arina"}, text="робот скажи это голосом арины")
    (said,) = node.pubs["/voice/speech/request"].sent
    assert said.voice == "arina" and inner.asked == 1
