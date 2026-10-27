"""#4054 + #2925: гейт ``register_speaker`` в диалоге v2 - не переименовать живой профиль чужим именем.

Марафон 27.10, акт 2c: голос уверенно «Нина» сказал «я тамара … запомни мой голос» - до #4054 это был лишний профиль
«Тамара», после - переименование профиля Нины (``speaker_id_node``: один голос - одна личность). Тот же механизм сработал
бы на «запомни, что Саша любит чай» голосом Нины, если модель позовёт ``register_speaker("Саша")``: гейт #2925 в v2
молчал, потому что контекст хода нёс только ``utterance_id`` и ``heard_text`` (``self_intro_name`` и
``known_speaker_name`` не передавались). Живая /data общая с людьми: профиль нельзя портить.

Путь тот же, что у v2 на роботе: контекст хода (``self_intro_name`` / ``known_speaker_name``, собирает
``dialog_node.voice_turn_context`` из ``speaker_adapter.named_self`` / ``confident_name``; его собственный тест - в
``rob_box_voice/test/unit/node/test_dialog_node.py``) → настоящий ``LLMToolCallAdapter`` (подпись личности) →
``ROSMCPToolProvider`` → настоящий ``MCPToolRegistry`` и ``RegisterSpeakerTool`` на отдельном узле → шина →
``Executor`` диалога (исход ``owner_event``) → фраза кода (``respond``). Владелец голосов (``speaker_id_node``) - двойник
по контракту: ответ ``SpeakerResult registered`` на запрос регистрации, с ``new_name`` при переименовании. Само правило
переименования (уверенный голос + другое имя) - его тесты, ``rob_box_voice/test/unit/node/
test_issue_4054_one_voice_renames_profile.py``; здесь доказано, что запрос владельцу НЕ доходит там, где человек себя
не называл, и доходит там, где называл.
"""

from __future__ import annotations

import asyncio
import importlib.util
import json
import sys
from pathlib import Path
from unittest import mock
from typing import Any, Dict, Optional

import pytest

_HERE = Path(__file__).resolve().parent


def _load_harness():
    name = "_rsp_2842_harness_for_4054"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, _HERE / "test_register_speaker_ros_mcp_utterance.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_h = _load_harness()

from rob_box_dialog import phrases  # noqa: E402
from rob_box_dialog.command import ToolCall  # noqa: E402
from rob_box_dialog.execute import ActionRequest, Executor  # noqa: E402
from rob_box_dialog.respond import audible, join  # noqa: E402
from rob_box_dialog.session import SessionSnapshot, SpeakerRef  # noqa: E402
from rob_box_dialog.speaker_adapter import confident_name, named_self  # noqa: E402
from rob_box_harness.core.tool_loop.command_turn import owner_event  # noqa: E402
from rob_box_harness.executors import ROSMCPToolProvider, adapt_tool_provider  # noqa: E402
from rob_box_llm.provider import ToolCall as LLMToolCall  # noqa: E402
from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter  # noqa: E402
from rob_box_mcp_tools.mcp_auth import PERSONALITY_SENDER  # noqa: E402

UID = "u4054"
#: Результаты биометрии фразы (как их кладёт ``speaker_id_node``): голос Нины уверенно, голос Эйджика, чужой голос.
NINA = {"is_known": True, "speaker_id": "e35517b2", "name": "Нина", "confidence": 0.93}
AGIK = {"is_known": True, "speaker_id": "a1b2c3d4", "name": "Эйджик", "confidence": 0.91}
STRANGER = {"is_known": False}


@pytest.fixture(autouse=True)
def _auth_token(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ROB_BOX_MCP_TOKEN", _h._TOKEN)
    monkeypatch.setattr(_h._DIALOGUE, "register_deadline_s", lambda: 0.5)


def _owner(bus, renames: bool):
    """Двойник ``speaker_id_node`` по контракту: на запрос регистрации - ``registered`` с тем же ``utterance_id``; при
    переименовании уверенно узнанного профиля (#4054) ещё и ``new_name``."""
    from rob_box_dialog_msgs.msg import SpeakerResult

    def on_register(msg: Any) -> None:
        body = json.loads(msg.data)
        ack = SpeakerResult()
        ack.event, ack.utterance_id, ack.name = SpeakerResult.EVENT_REGISTERED, body["utterance_id"], body["name"]
        if renames:
            ack.new_name, ack.reused_profile = body["name"], True
        for callback in bus.subscribers.get("/voice/speaker/result", []):
            callback(ack)

    bus.subscribers.setdefault("/voice/speaker/register", []).append(on_register)


class _V2Turn(_h._Node):
    """Узел диалога v2: контекст хода - тот же набор ключей, что собирает ``dialog_node.voice_turn_context`` (ключ - только
    если значение известно). ``voice`` - результат биометрии фразы; ``None`` - ход без голоса (Telegram, синтетика)."""

    def __init__(self, bus, heard: str, voice: Optional[Dict[str, Any]], voiced: bool = True) -> None:
        super().__init__("dialog_node", bus)
        self._context: Dict[str, Any] = {"utterance_id": UID if voiced else None, "heard_text": heard}
        if voiced:
            for key, value in (("self_intro_name", named_self(heard)), ("known_speaker_name", confident_name(voice))):
                if value:
                    self._context[key] = value

    def turn_context(self) -> Dict[str, Any]:
        return dict(self._context)


def _snapshot(voice: Optional[Dict[str, Any]]) -> SessionSnapshot:
    speaker = SpeakerRef(person_ref=(voice or {}).get("speaker_id"), name=(voice or {}).get("name"),
                         name_state="confirmed" if (voice or {}).get("name") else "unknown")
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, speaker=speaker)


def _v2_register(heard: str, voice: Optional[Dict[str, Any]], name: str, renames: bool = False, voiced: bool = True):
    """Один ход v2 до ``register_speaker(name)``: ``(результат действия, что ушло владельцу голосов, фраза кода)``."""
    bus = _h._Bus()
    with mock.patch.object(_h, "_speaker_id_owner", lambda b: _owner(b, renames)):  # владелец сервера - двойник ниже
        _h._MCPServer(bus)
    node = _V2Turn(bus, heard, voice, voiced)
    bridge = LLMToolCallAdapter(node, sender=PERSONALITY_SENDER, turn_context=node.turn_context)
    provider = ROSMCPToolProvider(bridge)
    provider.update_tools([_h.RegisterSpeakerTool(_h._Node("mcp_server", bus)).to_openai_tool_format()])
    tools = adapt_tool_provider(provider)
    executor = Executor(lambda request: _send(executor, tools, request))
    [result] = asyncio.run(executor.run([ToolCall("register_speaker", {"name": name})], _snapshot(voice), "t1"))
    sent = [json.loads(d) for d in bus.published.get("/voice/speaker/register", [])]
    said = join(audible([result], speaks=False))
    return result, sent, said.text if said is not None else None


async def _send(executor: Executor, tools: Any, request: ActionRequest) -> None:
    """Как ``V2Ports._send``: вызов тула, исход - событие владельца из данных результата (``owner_event``)."""
    call = LLMToolCall(id=f"v2-{request.turn_id}", name=request.tool, arguments=dict(request.args))
    executor.on_event(owner_event(request, await tools.execute(call)))


def _renamed_phrases(name: str) -> set:
    return {text.format(name=name) for text in phrases.catalog()["identity.done.renamed"]}


def test_nina_voice_and_a_fact_about_sasha_does_not_rename_nina() -> None:
    """Нина уверенно + «Робот, запомни, что Саша любит чай» + ``register_speaker("Саша")``: гейт ``not_self_introduction``,
    владельцу голосов запрос не уходит (профиль «Нина» цел), человеку - честный отказ, а не «Запомнил тебя, Саша»."""
    result, sent, said = _v2_register("запомни, что Саша любит чай", NINA, "Саша", renames=True)
    assert sent == [], "запрос регистрации дошёл бы до speaker_id_node и переименовал бы Нину в Сашу"
    assert (result.status, result.reason) == ("rejected", "not_self_introduction")
    assert said == "Имя Саша к твоему голосу не записываю: себя ты так не называл, а голос твой я уже знаю."
    assert "Запомнил тебя" not in said and "Мы знакомы" not in said
    assert "Нина" not in said, "чьим голосом сказано, вслух не называем (как у identity.done.renamed)"


def test_nina_voice_and_an_introduction_as_tamara_renames_the_profile() -> None:
    """Нина уверенно + «Робот, привет, я Тамара, мы с тобой ещё не знакомы, запомни мой голос»: человек назвался -
    запрос владельцу уходит с ``utterance_id`` хода, владелец переименовывает (``new_name``), фраза кода - «мы знакомы»."""
    heard = "привет, я Тамара, мы с тобой ещё не знакомы, запомни мой голос"
    result, sent, said = _v2_register(heard, NINA, "Тамара", renames=True)
    assert sent == [{"name": "Тамара", "utterance_id": UID}]
    assert (result.status, result.reason) == ("done", "renamed")
    assert said in _renamed_phrases("Тамара")


def test_agik_voice_and_my_name_is_denchik_renames_the_profile() -> None:
    result, sent, said = _v2_register("меня зовут Денчик", AGIK, "Денчик", renames=True)
    assert sent == [{"name": "Денчик", "utterance_id": UID}]
    assert (result.status, result.reason) == ("done", "renamed")
    assert said in _renamed_phrases("Денчик")


def test_unknown_voice_and_my_name_is_oleg_registers_a_new_profile() -> None:
    """Незнакомый голос: гейт молчит (знать нечего), обычная регистрация - «Запомнил тебя, Олег.»."""
    result, sent, said = _v2_register("меня зовут Олег", STRANGER, "Олег")
    assert sent == [{"name": "Олег", "utterance_id": UID}]
    assert (result.status, result.reason) == ("done", None)
    assert said == "Запомнил тебя, Олег."


@pytest.mark.parametrize("heard", ["зови меня Тамара", "запомни меня как Тамара", "меня зовут не Нина, а Тамара"])
def test_asking_to_be_called_otherwise_is_calling_oneself_a_name(heard: str) -> None:
    """Просьба звать иначе - тоже «человек назвал себя» (грамматика #3801 ``classify_rename``): гейт пропускает."""
    _result, sent, said = _v2_register(heard, NINA, "Тамара", renames=True)
    assert sent == [{"name": "Тамара", "utterance_id": UID}] and said in _renamed_phrases("Тамара")


def test_a_name_other_than_the_one_the_person_gave_is_refused() -> None:
    """Человек назвался Денчиком, а модель регистрирует «Денис» (искажение, ослышка): ту же регистрацию, что назвал
    человек, гейт пропускает, другое имя - нет (поведение #2925 не изменилось, в v2 оно теперь есть)."""
    result, sent, _said = _v2_register("меня зовут Денчик", AGIK, "Денис", renames=True)
    assert sent == [] and result.reason == "not_self_introduction"


def test_same_name_as_the_confident_voice_goes_to_already_known() -> None:
    """Повторный ``register_speaker("Нина")`` голосом Нины - дальше «уже знаю» решает владелец (#2863), гейт не мешает."""
    _result, sent, _said = _v2_register("запомни меня, я Нина", NINA, "Нина")
    assert sent == [{"name": "Нина", "utterance_id": UID}]


def test_turn_without_a_voice_keeps_the_gate_silent() -> None:
    """Ход без голоса (Telegram, синтетика): ``utterance_id`` нет, ключей гейта нет - поведение как раньше (запрос не
    уходит по другой причине - нет фразы, ``no_utterance_context``, #2829, а не из-за гейта)."""
    result, sent, _said = _v2_register("запомни, что Саша любит чай", NINA, "Саша", voiced=False)
    assert sent == [] and result.reason == "no_utterance_context"


def test_gate_refusal_phrase_is_in_the_catalog_and_not_the_generic_one() -> None:
    """Причина отказа гейта - ключ каталога фраз, а не общая «Не смог запомнить голос» (она вводила бы в заблуждение:
    голос записан, не записано чужое имя)."""
    catalog = phrases.catalog()
    assert "identity.rejected.not_self_introduction" in catalog
    assert catalog["identity.rejected.not_self_introduction"] != catalog["identity.rejected"]
