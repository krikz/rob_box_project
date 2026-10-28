"""#4045: рассказчик сказки - голос робота из ``session.voice``; герой его голосом не звучит, подмена голоса сегмента
выступления голос робота не меняет.

Первая часть карточки (``voice_changed`` сегмента не идёт в ``session.voice``) сделана в #4053 (``robot_voice_changed``,
тест ``test_4045_*`` в ``test_dialog_node.py``). Вторая - нет: нода v2 не хранит ``_actual_tts_voice`` (его пишет только
v1), и каталог голосов РОЛЕЙ исключал голос провайдера по умолчанию (у MiniMax это китайский ``male-qn-qingse``, в
каталоге русских голосов его и так нет), а не голос, которым робот говорит по ``set_voice``. После «говори голосом
Надёжный» герой сказки получал тот же ``Russian_ReliableMan``, что и рассказчик (реплики ``""`` - голос робота, #4084).
"""

from __future__ import annotations

import sys
import types

import pytest
from rob_box_core import speech_client
from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog.command import Command, parse, validate

M = install_dialog_msgs_stub()
try:  # настоящий rosidl-пакет (CI) или заглушка сервиса DialogControl (Windows) — как в test_dialog_node.py
    import rob_box_dialog_msgs.srv  # noqa: F401
except ImportError:
    class _Request:
        ACTION_HOLD, ACTION_RESUME, ACTION_RESET = 0, 1, 2

    _srv = types.ModuleType("rob_box_dialog_msgs.srv")
    _srv.DialogControl = type("DialogControl", (), {"Request": _Request,
                                                    "Response": lambda: types.SimpleNamespace()})
    sys.modules["rob_box_dialog_msgs.srv"] = _srv
    sys.modules["rob_box_dialog_msgs"].srv = _srv

from rob_box_voice import dialog_node as dn  # noqa: E402
from rob_box_voice.core import plan_host  # noqa: E402
from rob_box_voice.tts_voice_registry import default_voice_for, gender_of, role_voices_for  # noqa: E402

ALL_TOOLS = [t.name for t in TOOL_CATALOG]
ROBOT = "Russian_ReliableMan"  # первый мужской голос каталога ролей MiniMax: именно его получал бы первый герой-мужчина
TALE = {
    "kind": "tale",
    "backing": "none",
    "segments": ["Жил-был в лесу ёжик.", "Лиса, давай дружить!", "Давай, колючий.", "И пошли они вместе."],
    "cast": [{"name": "Ёж", "gender": "male"}, {"name": "Лиса", "gender": "female"}],
    "role_per_segment": ["", "Ёж", "Лиса", ""],
}


class _Tools:
    async def execute(self, call):  # pragma: no cover — ходов эти тесты не делают
        raise AssertionError(call.name)


@pytest.fixture()
def node(monkeypatch):
    class _LLM:
        answered_by = "deepseek"

        async def complete(self, messages, tools=(), settings=None):  # pragma: no cover
            raise AssertionError("ход не ожидался")

    monkeypatch.setattr(dn, "build_llm", lambda node, counter, agent="personality": (_LLM(), "Ты робот."))
    monkeypatch.setattr(dn, "build_tools", lambda node: _Tools())
    monkeypatch.setattr(dn, "build_chat_tools", lambda node, personality_tools: _Tools())
    made = dn.DialogNode()
    made._actual_tts_provider = "minimax"  # факт от tts_node (/voice/tts/provider_state)
    yield made
    made.shutdown()


def _voice_changed(requested, applied, provider, speech_id="", agent=speech_client.VOICE_OWNER_AGENT):
    """``voice_changed`` владельца голоса (ответ на ``set_voice``); с ``speech_id`` - подмена голоса сегмента."""
    event = M.SpeechEvent()
    event.kind, event.voice_requested, event.voice_applied, event.provider = \
        M.SpeechEvent.KIND_VOICE_CHANGED, requested, applied, provider
    event.speech_id, event.agent, event.turn_id = speech_id, agent, "req-1" if not speech_id else "t-tale"
    return event


def _tale_voices(node):
    """Тот же путь, что у хода: команда модели -> валидатор с ``Busy`` ноды -> ``voice_per_segment`` выступления."""
    res = validate(parse({"perform": TALE}), ALL_TOOLS, busy=plan_host.perform_busy(node))
    assert isinstance(res, Command), res
    return res.perform.voice_per_segment


def test_hero_of_a_tale_never_sounds_like_the_narrator_after_set_voice(node):
    """«Говори голосом Надёжный», потом «расскажи сказку про ежа и лису»: рассказчик - голос робота (``""``), у
    героя-мужчины не тот же голос. Без голоса робота из сессии каталог ролей исключал ``male-qn-qingse`` и отдавал
    ёжику Надёжного."""
    node._on_speech_event(_voice_changed(ROBOT, ROBOT, "minimax"))
    narrator, hedgehog, fox = _tale_voices(node)[:3]
    assert narrator == "" and hedgehog and fox
    assert ROBOT not in (hedgehog, fox) and hedgehog != fox
    assert (gender_of("minimax", hedgehog), gender_of("minimax", fox)) == ("male", "female")
    assert node._session.snapshot.voice.applied == ROBOT


def test_without_set_voice_the_narrator_is_the_provider_default_as_before(node):
    """Голос робота не менялся (сессия пуста): исключается голос провайдера по умолчанию, как раньше."""
    assert node._session.snapshot.voice.applied is None
    assert plan_host.perform_busy(node).voices == tuple(role_voices_for("minimax", default_voice_for("minimax")))
    assert _tale_voices(node)[1] == "Russian_ReliableMan"  # первый свободный мужской голос каталога


def test_segment_voice_substitution_neither_changes_the_robot_voice_nor_the_role_catalog(node):
    """Сказка с недоступным голосом роли (#4043): ``voice_changed`` сегмента (есть ``speech_id``) - подмена одной
    реплики. ``session.voice`` и рассказчик остаются голосом робота, каталог ролей не сдвигается."""
    node._on_speech_event(_voice_changed(ROBOT, ROBOT, "minimax"))
    before_state, before_pool = node._session.snapshot.voice, plan_host.perform_busy(node).voices
    node._on_speech_event(_voice_changed("Russian_AttractiveGuy", "Russian_HandsomeChildhoodFriend", "minimax",
                                         speech_id="seg-2", agent="personality"))
    node._on_speech_event(_voice_changed("Russian_BrightHeroine", ROBOT, "minimax", speech_id="seg-3",
                                         agent="personality"))
    assert node._session.snapshot.voice == before_state
    assert plan_host.perform_busy(node).voices == before_pool and ROBOT not in before_pool
    assert _tale_voices(node)[0] == ""


def test_new_set_voice_moves_the_narrator_exclusion_with_it(node):
    node._actual_tts_provider = "yandex"
    node._on_speech_event(_voice_changed("alena", "alena", "yandex"))
    pool = plan_host.perform_busy(node).voices
    assert "alena" not in pool and "anton" in pool  # anton - голос по умолчанию, но не голос робота сейчас
    node._on_speech_event(_voice_changed("anton", "anton", "yandex"))
    pool = plan_host.perform_busy(node).voices
    assert "anton" not in pool and "alena" in pool


def test_voice_of_another_provider_does_not_touch_the_catalog_of_the_sounding_one(node):
    """Основной провайдер упал, звучит запасной: применённый голос MiniMax в каталоге Yandex ничего не исключает."""
    node._on_speech_event(_voice_changed(ROBOT, ROBOT, "minimax"))
    node._actual_tts_provider = "yandex"
    assert plan_host.perform_busy(node).voices == tuple(role_voices_for("yandex", default_voice_for("yandex")))


def test_robot_voice_reads_only_what_the_owner_applied():
    def host(voice):
        return types.SimpleNamespace(_session=types.SimpleNamespace(snapshot=types.SimpleNamespace(voice=voice)))

    applied = types.SimpleNamespace
    assert plan_host.robot_voice(host(applied(applied="alena", provider="yandex")), "yandex", "anton") == "alena"
    assert plan_host.robot_voice(host(applied(applied="alena", provider=None)), "yandex", "anton") == "alena"
    assert plan_host.robot_voice(host(applied(applied="alena", provider="")), "yandex", "anton") == "alena"
    assert plan_host.robot_voice(host(applied(applied="alena", provider="minimax")), "yandex", "anton") == "anton"
    assert plan_host.robot_voice(host(applied(applied=None, provider="yandex")), "yandex", "anton") == "anton"
    assert plan_host.robot_voice(types.SimpleNamespace(), "yandex", "anton") == "anton"  # нода v1: сессии нет
