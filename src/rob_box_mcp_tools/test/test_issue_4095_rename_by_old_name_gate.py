"""#4095 + #2925: переименование ``register_speaker(old_name=…)`` проходит тот же гейт, что и регистрация.

До поправки ``RegisterSpeakerTool.execute`` публиковал ``/voice/speaker/rename`` по ``old_name`` ДО гейта #2925: на
«запомни, что Саша любит чай» голосом Нины ``register_speaker(name='Саша', old_name='Нина')`` переименовывал живой
профиль Нины в «Саша» (тот же класс ошибки, что закрыт в #4087 для регистрации; живая /data общая с людьми). Теперь
(``rename_gate``): голос уверенно узнан, а ``old_name`` — другой профиль → ``rejected{not_own_profile}``; профиль свой,
а новое имя человек себе не давал → ``rejected{not_self_introduction}``. Запрос на ``/voice/speaker/rename`` в обоих
случаях НЕ уходит, фразу говорит код (``identity.rejected.<причина>``).

Путь тот же, что у v2 на роботе и в ``test_issue_4054_register_gate_v2``: контекст хода → настоящий
``LLMToolCallAdapter`` → ``ROSMCPToolProvider`` → настоящий ``RegisterSpeakerTool`` на отдельном узле → шина →
``Executor`` диалога → фраза кода. Двойник владельца голосов (``speaker_id_node``) отвечает ``SpeakerResult
registered`` на регистрацию; смотрим, что дошло до ``/voice/speaker/rename``.
"""

from __future__ import annotations

import asyncio
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional
from unittest import mock

import pytest

_HERE = Path(__file__).resolve().parent


def _load(name: str, file: str):
    """Модуль соседнего теста по пути: общий стенд (шина, узлы, двойник владельца) - один, а не копия."""
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, _HERE / file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_v2 = _load("_rsp_4054_v2_for_4095", "test_issue_4054_register_gate_v2.py")
_h = _v2._h

from rob_box_dialog import phrases  # noqa: E402
from rob_box_dialog.command import ToolCall  # noqa: E402
from rob_box_dialog.execute import Executor  # noqa: E402
from rob_box_dialog.respond import audible, join  # noqa: E402
from rob_box_harness.executors import ROSMCPToolProvider, adapt_tool_provider  # noqa: E402
from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter  # noqa: E402
from rob_box_mcp_tools.mcp_auth import PERSONALITY_SENDER  # noqa: E402

NINA, AGIK, STRANGER, UID = _v2.NINA, _v2.AGIK, _v2.STRANGER, _v2.UID
#: Модуль тулов диалога так, как его грузит стенд (без настоящего rclpy).
rename_gate, _rename_source = _h._DIALOGUE.rename_gate, _h._DIALOGUE._rename_source


@pytest.fixture(autouse=True)
def _auth_token(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ROB_BOX_MCP_TOKEN", _h._TOKEN)
    monkeypatch.setattr(_h._DIALOGUE, "register_deadline_s", lambda: 0.5)


def _renamed(heard: str, voice: Optional[Dict[str, Any]], args: Dict[str, Any], voiced: bool = True):
    """Один ход v2 до ``register_speaker(**args)``: ``(результат, что ушло на /voice/speaker/rename, что ушло на
    регистрацию, фраза кода)``."""
    bus = _h._Bus()
    with mock.patch.object(_h, "_speaker_id_owner", lambda b: _v2._owner(b, True)):
        _h._MCPServer(bus)
    node = _v2._V2Turn(bus, heard, voice, voiced)
    provider = ROSMCPToolProvider(LLMToolCallAdapter(node, sender=PERSONALITY_SENDER, turn_context=node.turn_context))
    provider.update_tools([_h.RegisterSpeakerTool(_h._Node("mcp_server", bus)).to_openai_tool_format()])
    tools = adapt_tool_provider(provider)
    executor = Executor(lambda request: _v2._send(executor, tools, request))
    [result] = asyncio.run(executor.run([ToolCall("register_speaker", args)], _v2._snapshot(voice), "t1"))
    renames: List[Dict[str, Any]] = [json.loads(d) for d in bus.published.get("/voice/speaker/rename", [])]
    sent = [json.loads(d) for d in bus.published.get("/voice/speaker/register", [])]
    said = join(audible([result], speaks=False))
    return result, renames, sent, said.text if said is not None else None


def test_a_fact_about_sasha_with_old_name_nina_does_not_rename_nina() -> None:
    """Нина уверенно + «запомни, что Саша любит чай» + ``register_speaker(name='Саша', old_name='Нина')``: переименования
    нет, регистрации нет, ``rejected{not_self_introduction}``, человеку - фраза кода, а не «Запомнил»."""
    result, renames, sent, said = _renamed("запомни, что Саша любит чай", NINA, {"name": "Саша", "old_name": "Нина"})
    assert renames == [], "профиль Нины переименовался бы в Сашу: живая /data общая с людьми"
    assert sent == []
    assert (result.status, result.reason) == ("rejected", "not_self_introduction")
    assert said == phrases.text("identity.rejected.not_self_introduction", received="Саша")
    assert "Нина" not in said and "Запомнил" not in said


def test_call_me_tamara_with_old_name_nina_renames_the_profile() -> None:
    """Нина уверенно + «зови меня Тамара» + ``old_name='Нина'``: человек назвал себя, профиль - его голоса. Переименование
    уходит на ``/voice/speaker/rename`` как раньше, регистрация - владельцу голосов, фраза кода - «мы знакомы»."""
    result, renames, sent, said = _renamed("зови меня Тамара", NINA, {"name": "Тамара", "old_name": "Нина"})
    assert renames == [{"old_name": "Нина", "new_name": "Тамара"}]
    assert sent == [{"name": "Тамара", "utterance_id": UID}]
    assert (result.status, result.reason) == ("done", "renamed")
    assert said in _v2._renamed_phrases("Тамара")


def test_someone_elses_profile_is_not_renamed_by_this_voice() -> None:
    """Нина уверенно + «зови меня Тамара» + ``old_name='Саша'`` (модель взяла чужую запись из разговора): имя Тамара человек
    себе давал, регистрационный гейт молчит, но профиль Саши - не этого голоса. Ни переименования, ни регистрации;
    причина ``not_own_profile`` и своя фраза: «себя ты так не называл» было бы неправдой."""
    result, renames, sent, said = _renamed("зови меня Тамара", NINA, {"name": "Тамара", "old_name": "Саша"})
    assert renames == [] and sent == []
    assert (result.status, result.reason) == ("rejected", "not_own_profile")
    assert said == phrases.text("identity.rejected.not_own_profile")
    assert "Саша" not in said and "Нина" not in said and "себя ты так не называл" not in said


def test_correction_by_a_voice_that_is_not_confident_still_renames() -> None:
    """«Я не Эйджик, я Денчик» голосом, который биометрия уверенно не назвала: гейту опереться не на что (как у регистрации),
    поправка #1101 работает как раньше."""
    result, renames, sent, _said = _renamed("я не Эйджик, я Денчик", STRANGER, {"name": "Денчик", "old_name": "Эйджик"})
    assert renames == [{"old_name": "Эйджик", "new_name": "Денчик"}]
    assert sent == [{"name": "Денчик", "utterance_id": UID}]
    assert result.status == "done"


def test_correction_by_the_confident_voice_of_that_profile_renames() -> None:
    """Эйджик уверенно + «меня зовут не Эйджик, а Денчик» + ``old_name='Эйджик'``: профиль свой, имя названо - переименуем."""
    result, renames, _sent, said = _renamed("меня зовут не Эйджик, а Денчик", AGIK,
                                            {"name": "Денчик", "old_name": "Эйджик"})
    assert renames == [{"old_name": "Эйджик", "new_name": "Денчик"}]
    assert (result.status, result.reason) == ("done", "renamed") and said in _v2._renamed_phrases("Денчик")


def test_turn_without_a_voice_renames_as_before() -> None:
    """Ход без голоса (Telegram, синтетика): ключей гейта нет - «я не X, я Y» переименовывает, как до поправки (регистрацию
    в таком ходе блокирует другое: нет ``utterance_id``, #2829)."""
    result, renames, sent, _said = _renamed("я не Эйджик, я Денчик", NINA, {"name": "Денчик", "old_name": "Эйджик"},
                                            voiced=False)
    assert renames == [{"old_name": "Эйджик", "new_name": "Денчик"}]
    assert sent == [] and result.reason == "no_utterance_context"


@pytest.mark.parametrize("old, new, intro, known, verdict", [
    ("Нина", "Саша", None, "Нина", "not_self_introduction"),
    ("Нина", "Саша", "Тамара", "Нина", "not_self_introduction"),
    ("Нина", "Тамара", "Тамара", "Нина", None),
    ("нина", "тамара", "Тамара", "Нина", None),
    ("Нина", "Нина", None, "Нина", None),
    ("Саша", "Тамара", "Тамара", "Нина", "not_own_profile"),
    ("Саша", "Саша", None, "Нина", "not_own_profile"),
    ("Эйджик", "Денчик", None, None, None),
    ("Эйджик", "Денчик", "Денчик", None, None),
    ("Нина", None, None, "Нина", None),
    ("Нина", "  ", None, "Нина", None),
    ("Алёна", "Тамара", "Тамара", "Алена", None),
], ids=["fact", "other-name", "self-intro", "case", "same-name", "other-profile", "other-profile-same-name",
        "no-confident-voice", "no-confident-voice-intro", "no-new-name", "blank-new-name", "yo"])
def test_rename_gate_table(old, new, intro, known, verdict) -> None:
    assert rename_gate(old, new, intro, known) == verdict


@pytest.mark.parametrize("old, source", [("Нина", "Нина"), ("  Нина ", "Нина"), (None, None), ("", None), (" ", None),
                                         ("null", None), ("None", None), ("Н", None)])
def test_only_a_real_name_asks_for_a_rename(old, source) -> None:
    """Одно правило «просят ли переименовать»: публикация и гейт читают его из ``_rename_source`` (литерал ``null`` из
    tool-call, пусто и однобуквенное - не имя, #1101)."""
    assert _rename_source(old) == source


def test_refusal_phrases_are_in_the_catalog() -> None:
    catalog = phrases.catalog()
    assert "identity.rejected.not_own_profile" in catalog
    assert catalog["identity.rejected.not_own_profile"] != catalog["identity.rejected"]
