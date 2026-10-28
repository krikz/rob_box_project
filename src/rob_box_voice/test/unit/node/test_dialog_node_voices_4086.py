"""#4086: Tier-1 «включи голос <имя>» в диалоге v2 находит голос по реестру звучащего провайдера (ADR-0148, ADR-0150 §4.2).

Живой робот 27.10 20:14 (dev-d97bb04): «Робот, говори голосом Весельчак» - модель ответила «Такого голоса у меня нет»,
``set_voice`` не вызван. Голос в каталоге был: ``dialog_node`` строил ``TurnEngine`` без ``voices=``, реестр имён
(``grammar._voice``) был пуст, и по имени голоса Tier-1 не срабатывал (срабатывал только «обычным голосом», он реестр не
смотрит). В v1 тот же список давал ``tier1_host.voices`` - порт на общую функцию ``turn_context.tts_voices``.
"""

from __future__ import annotations

import re
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
from rob_box_voice.tts_voice_registry import (  # noqa: E402
    PROVIDER_VOICES,
    VOICE_ALIASES,
    VOICE_METADATA,
    named_voices_for,
)

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
    # кириллические алиасы (VOICE_ALIASES, ревью PR #4114): так голос называет STT
    ("Робот, говори голосом джовиалмен", "English_Jovialman"),
    ("Робот, говори голосом Джовиалмэн", "English_Jovialman"),
    ("Робот, включи голос Вайз Скулар", "English_WiseScholar"),
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


def test_registry_entries_are_catalog_ids_with_russian_display_names_and_aliases_apart():
    """Запись - ``(id, display_name, *алиасы)``: название и каждый алиас отдельными именами, не склейкой (ревью PR #4129:
    склеенная строка не находилась ``voice_asked`` ни по названию, ни по алиасу)."""
    entries = {vid: names for vid, *names in named_voices_for("minimax")}
    assert list(entries) == PROVIDER_VOICES["minimax"], "все голоса каталога, в порядке каталога"
    display = VOICE_METADATA["minimax:English_Jovialman"]["display_name"]
    assert display == "Весельчак" and entries["English_Jovialman"] == [display, "Джовиалмен", "Джовиалмэн"]
    assert entries["Russian_ReliableMan"] == [VOICE_METADATA["minimax:Russian_ReliableMan"]["display_name"]]
    assert named_voices_for("unknown") == []


def _name_words(voice_id, display):
    """Слова, которыми голос называли до алиасов: русское название и хвост id по границам регистра."""
    tail = re.sub(r"([a-z])([A-Z])", r" ", voice_id.split("_", 1)[-1])
    return {w for w in re.findall(r"[^\W_]+", f"{display} {tail}") if len(w) >= 3}


def test_aliases_do_not_break_any_name_word_that_led_to_a_voice_before_them():
    """Ревью PR #4129: алиас «Виспериг гёрл» («гёрл» ~ «героиня» по трём буквам) делал «включи голос героиня»
    неоднозначной, и фраза уходила модели. Обход ВСЕХ голосов ВСЕХ провайдеров реестра: каждое слово названия и хвоста
    id, которое с реестром ``(id, название)`` вело к голосу, с алиасами ведёт к нему же."""
    from rob_box_dialog.grammar import match
    from rob_box_dialog.session import Session

    snapshot = Session("personality").snapshot
    checked = 0
    for provider in PROVIDER_VOICES:
        registry = named_voices_for(provider)
        before_aliases = [entry[:2] for entry in registry]
        for vid, display, *_ in registry:
            for word in _name_words(vid, display):
                was = match(f"включи голос {word}", snapshot, before_aliases)
                if was is not None:
                    checked += 1
                    assert match(f"включи голос {word}", snapshot, registry) == was, (provider, vid, word)
    assert checked > 30, "обход не пустой: слов-имён, ведущих к голосу, на 28.10 было 82"
    heroine = match("включи голос героиня", snapshot, named_voices_for("minimax"))
    assert heroine is not None and heroine.args["voice"] == "Russian_BrightHeroine"


def test_the_form_of_the_turn_sees_the_display_name_and_every_alias_of_every_voice_of_the_registry():
    """Ревью PR #4129: ``command_form`` берёт имена из записей реестра; пока они были склейкой, форма хода закрывала
    ``voice`` на «скажи как Весельчак» (и ещё четыре голоса каста). Каждое название (``VOICE_METADATA``) и каждый
    алиас (``VOICE_ALIASES``) - просьба о голосе; имена берутся из источников, а не из записей реестра: склейка в
    записи не должна подгонять ожидание под себя."""
    from rob_box_dialog.address import VOICE
    from rob_box_dialog.session import Session
    from rob_box_dialog.turn import _Turn, command_form

    snapshot = Session("personality").snapshot
    checked = 0
    for provider in PROVIDER_VOICES:
        registry = named_voices_for(provider)
        for vid, *_ in registry:
            key = f"{provider}:{vid}"
            display = (VOICE_METADATA.get(key) or {}).get("display_name")
            names = [*([display] if display else []), *VOICE_ALIASES.get(key, ())]
            for name in names:
                turn = _Turn("t1", 0, f"Робот, скажи как {name}", VOICE, deadline=None)
                if command_form(turn, snapshot).voice:
                    continue  # слово само открывает группу голоса: реестр тут ничего не решает
                checked += 1
                assert command_form(turn, snapshot, registry).voice, (provider, vid, name)
    assert checked > 15, "обход не пустой: названий и алиасов вне групп голоса на 28.10 было 39"


def test_every_alias_of_the_registry_is_a_name_the_tier1_grammar_resolves():
    """Одно знание об именах - ``VOICE_ALIASES``: каждый алиас реестра, названный вслух, грамматика Tier-1 сводит к его
    голосу кодом (``set_voice``), без модели."""
    from rob_box_dialog.grammar import match
    from rob_box_dialog.session import Session

    snapshot = Session("personality").snapshot
    for key, aliases in VOICE_ALIASES.items():
        provider, voice = key.split(":", 1)
        voices = named_voices_for(provider)
        for alias in aliases:
            command = match(f"говори голосом {alias}", snapshot, voices)
            assert command is not None and command.kind == "set_voice" and command.args["voice"] == voice, (key, alias)


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
