"""#4086: Tier-1 «включи голос <имя>» в диалоге v2 находит голос по реестру звучащего провайдера (ADR-0148, ADR-0150 §4.2).

Живой робот 27.10 20:14 (dev-d97bb04): «Робот, говори голосом Весельчак» - модель ответила «Такого голоса у меня нет»,
``set_voice`` не вызван. Голос в каталоге был: ``dialog_node`` строил ``TurnEngine`` без ``voices=``, реестр имён
(``grammar._voice``) был пуст, и по имени голоса Tier-1 не срабатывал (срабатывал только «обычным голосом», он реестр не
смотрит). В v1 тот же список давал ``tier1_host.voices`` - порт на общую функцию ``turn_context.tts_voices``.
"""

from __future__ import annotations

import sys
import types

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
try:  # настоящий rosidl-пакет (CI) или заглушка сервиса DialogControl (Windows)
    import rob_box_dialog_msgs.srv  # noqa: F401
except ImportError:
    class _Request:
        ACTION_HOLD, ACTION_RESUME, ACTION_RESET = 0, 1, 2

        def __init__(self, action=0, set_by="", ttl_s=0.0):
            self.action, self.set_by, self.ttl_s = action, set_by, ttl_s

    _srv = types.ModuleType("rob_box_dialog_msgs.srv")
    _srv.DialogControl = type("DialogControl", (), {"Request": _Request,
                                                    "Response": lambda: types.SimpleNamespace()})
    sys.modules["rob_box_dialog_msgs.srv"] = _srv
    sys.modules["rob_box_dialog_msgs"].srv = _srv

from rob_box_llm.provider import LLMResponse, ToolResult  # noqa: E402
from rob_box_voice import dialog_node as dn  # noqa: E402
from rob_box_voice.core.tier1_host import Tier1Runner  # noqa: E402
from rob_box_voice.core.turn_context import tts_voices  # noqa: E402
from rob_box_voice.tts_voice_registry import PROVIDER_VOICES, VOICE_METADATA, named_voices_for  # noqa: E402

TIME = type(M.TurnEvent().stamp)


class _LLM:
    def __init__(self, responses):
        self.responses, self.calls = list(responses), 0
        self.answered_by = "deepseek"

    async def complete(self, messages, tools=(), settings=None):
        self.calls += 1
        return self.responses.pop(0)


class _Tools:
    def __init__(self):
        self.calls = []

    async def execute(self, call):
        self.calls.append((call.name, dict(call.arguments)))
        return ToolResult(tool_call_id=call.id, content="ok", data={"status": "done"})


@pytest.fixture()
def node(monkeypatch):
    llm, tools = _LLM([LLMResponse(content="Не понял, какой голос.")]), _Tools()
    monkeypatch.setattr(dn, "build_llm", lambda node, counter, agent="personality": (llm, "Ты робот."))
    monkeypatch.setattr(dn, "build_tools", lambda node: tools)
    monkeypatch.setattr(dn, "build_chat_tools", lambda node, personality_tools: tools)
    made = dn.DialogNode()
    stamp = TIME(sec=1, nanosec=0)
    made.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    made.test_llm, made.test_tools = llm, tools
    yield made
    made.shutdown()


def _say(made, text):
    msg = M.Utterance()
    msg.text, msg.source, msg.utterance_id = text, "respeaker", "u1"
    made._on_utterance(msg)
    made.last_turn.result(timeout=10)


# -- ход целиком: Tier-1 по реестру, 0 вызовов LLM -------------------------------------------------------------------


@pytest.mark.parametrize("phrase, voice", [
    ("Робот, говори голосом Весельчак", "English_Jovialman"),
    ("Робот, говори голосом Jovialman", "English_Jovialman"),
    ("Робот, включи голос надёжного мужика", "Russian_ReliableMan"),
])
def test_minimax_voice_by_name_is_set_by_code_without_llm(node, phrase, voice):
    _say(node, phrase)
    assert node.test_tools.calls == [("set_voice", {"voice": voice})]
    assert node.test_llm.calls == 0


def test_yandex_voice_by_name_follows_the_sounding_provider(node):
    node._actual_tts_provider = "yandex"  # факт от tts_node (/voice/tts/provider_state), не параметр
    _say(node, "Робот, включи голос Алёны")
    assert node.test_tools.calls == [("set_voice", {"voice": "alena"})]
    assert node.test_llm.calls == 0


def test_voice_of_another_provider_is_not_guessed(node):
    """«Весельчак» - голос MiniMax: на Yandex такого имени в реестре нет - решает модель, а не угадывание кодом."""
    node._actual_tts_provider = "yandex"
    _say(node, "Робот, говори голосом Весельчак")
    assert node.test_tools.calls == [] and node.test_llm.calls == 1


@pytest.mark.parametrize("phrase", [
    "Робот, включи голос парень",  # «Привлекательный парень» и «Вспыльчивый парень»: двое - код не выбирает
    "Робот, включи голос Чебурашки",  # такого нет в реестре
])
def test_ambiguous_or_unknown_name_goes_to_the_model(node, phrase):
    _say(node, phrase)
    assert node.test_tools.calls == [] and node.test_llm.calls == 1


# -- функция реестра имён: одна для v1 и v2 ---------------------------------------------------------------------------


def test_registry_pairs_are_catalog_ids_with_russian_display_names():
    pairs = dict(named_voices_for("minimax"))
    assert list(pairs) == PROVIDER_VOICES["minimax"], "все голоса каталога, в порядке каталога"
    assert pairs["English_Jovialman"] == "Весельчак" == VOICE_METADATA["minimax:English_Jovialman"]["display_name"]
    assert named_voices_for("unknown") == []


def test_tts_voices_follow_the_sounding_provider_then_the_parameter_then_the_robot_default():
    facts = types.SimpleNamespace(_actual_tts_provider="yandex")
    assert tts_voices(facts) == named_voices_for("yandex")
    label = types.SimpleNamespace(_actual_tts_provider="prefetch")  # метка не из каталога факт не заменяет (#4024)
    assert tts_voices(label) == named_voices_for("minimax"), "нет параметра - провайдер робота по умолчанию"
    param = types.SimpleNamespace(_actual_tts_provider=None, get_parameter=lambda _n: types.SimpleNamespace(value="silero"))
    assert tts_voices(param) == named_voices_for("silero")


def test_v1_tier1_runner_takes_the_same_voices_as_v2():
    node_v1 = types.SimpleNamespace(_actual_tts_provider="yandex")
    assert Tier1Runner(node_v1).voices() == tts_voices(node_v1) == named_voices_for("yandex")
