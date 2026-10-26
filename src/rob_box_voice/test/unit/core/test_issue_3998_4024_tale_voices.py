"""#3998 + #4024: голоса ролей сказки - русские голоса звучащего провайдера, и проверка не выключается молча.

Марафон 23.10, акт 4 (n412): сказка звучала китайским ``male-qn-qingse`` на русском тексте (#3998 - в перечень голосов
ролей попадал весь каталог MiniMax с legacy-id), а во второй сказке LLM взяла Yandex-имена alena/omazh/zahar, которые
MiniMax молча заменил: ``provider_state`` с ``prefetch`` затёр фактического провайдера, ``voices_for('prefetch') == []``,
и ``_check_voices`` ничего не сверял (#4024).

Тесты гонят настоящий путь хоста: события ``/voice/tts/provider_state`` -> ``plan_host`` -> ``Busy.voices`` -> схема и
валидатор команды ``rob_box_dialog``.
"""

from __future__ import annotations

import json
from types import SimpleNamespace

import pytest
from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, Invalid, parse, validate

from rob_box_voice.core import plan_host
from rob_box_voice.tts_voice_registry import (
    default_voice_for,
    role_voices_for,
    voices_for,
)

ALL_TOOLS = [t.name for t in TOOL_CATALOG]
SEGMENTS = ["Жила-была девочка.", "Куда идёшь?", "К бабушке.", "Дорога длинная."]


def _node(param: str | None = "minimax") -> SimpleNamespace:
    """Нода хоста v2: фактический провайдер ещё не известен, настроенный - ``tts_provider``."""
    node = SimpleNamespace(_actual_tts_provider=None)

    def get_parameter(name):
        if param is None or name != "tts_provider":
            raise KeyError(name)
        return SimpleNamespace(value=param)

    node.get_parameter = get_parameter
    return node


def _state(node, provider, reason="synthesis_ok", voice=None) -> None:
    """Событие ``/voice/tts/provider_state`` от tts_node."""
    payload = {
        "provider": provider,
        "voice": voice or default_voice_for(provider),
        "reason": reason,
    }
    plan_host.on_tts_provider_state(node, SimpleNamespace(data=json.dumps(payload)))


def _tale(*voices):
    return {
        "perform": {
            "kind": "tale",
            "segments": SEGMENTS[: len(voices)],
            "voice_per_segment": list(voices),
        }
    }


def _judge(node, raw):
    return validate(parse(raw), ALL_TOOLS, busy=plan_host.perform_busy(node))


# -- #3998: перечень голосов ролей --------------------------------------------------------------------------------


def test_minimax_tale_voices_are_the_russian_catalogue_without_the_narrators_chinese_voice():
    busy = plan_host.perform_busy(_node("minimax"))
    assert busy.voices and all(
        v.startswith("Russian_") for v in busy.voices
    ), busy.voices
    assert "male-qn-qingse" not in busy.voices and "female-shaonv" not in busy.voices
    assert busy.voices == tuple(
        role_voices_for("minimax", default_voice_for("minimax"))
    )


@pytest.mark.parametrize("provider", ["yandex", "minimax", "silero"])
def test_busy_voices_follow_the_sounding_provider(provider):
    node = _node("minimax")
    _state(node, provider)
    assert plan_host.perform_busy(node).voices == tuple(
        role_voices_for(provider, default_voice_for(provider))
    )


def test_the_chinese_voice_is_a_violation_in_a_tale():
    res = _judge(_node("minimax"), _tale("", "male-qn-qingse", "Russian_BrightHeroine"))
    assert isinstance(res, Invalid) and res.reason == "bad_enum", res
    assert "male-qn-qingse" in res.violations[0].detail
    assert set(res.violations[0].allowed) == set(
        plan_host.perform_busy(_node("minimax")).voices
    )


def test_russian_voices_pass_and_the_command_keeps_them():
    node = _node("minimax")
    res = _judge(
        node,
        _tale(
            "", "Russian_AttractiveGuy", "Russian_BrightHeroine", "Russian_CrazyQueen"
        ),
    )
    assert isinstance(res, Command), res
    assert res.perform.voice_per_segment == (
        "",
        "Russian_AttractiveGuy",
        "Russian_BrightHeroine",
        "Russian_CrazyQueen",
    )


def test_command_schema_offers_the_model_only_role_voices():
    items = kn.with_voices(
        kn.command_schema("json"), plan_host.perform_busy(_node("minimax")).voices
    )
    offered = items["properties"]["perform"]["properties"]["voice_per_segment"][
        "items"
    ]["enum"]
    assert offered[0] == "" and offered[1:] == sorted(
        role_voices_for("minimax", default_voice_for("minimax"))
    )
    assert "male-qn-qingse" not in offered


def test_set_voice_still_sees_the_whole_catalogue():
    """Голос робота меняет ``set_voice`` по полному каталогу; сужены только роли выступления."""
    assert "male-qn-qingse" in voices_for("minimax")


# -- #4024: метка prefetch не провайдер ---------------------------------------------------------------------------


def test_prefetch_state_does_not_overwrite_the_actual_provider():
    node = _node("minimax")
    _state(node, "minimax")
    for _ in range(8):  # как в mcp_server 22:03-22:04: восемь раз подряд
        _state(node, "prefetch", reason="synthesis_ok")
    assert node._actual_tts_provider == "minimax"


@pytest.mark.parametrize(
    "data",
    [
        "",
        "not json",
        "[]",
        "null",
        json.dumps({"provider": ""}),
        json.dumps({"voice": "x"}),
    ],
)
def test_garbage_provider_state_is_ignored(data):
    node = _node("minimax")
    _state(node, "yandex")
    plan_host.on_tts_provider_state(node, SimpleNamespace(data=data))
    assert node._actual_tts_provider == "yandex"


def test_prefetch_before_a_tale_with_yandex_names_is_bad_enum_not_a_silent_swap():
    """Раньше: ``prefetch`` -> каталог пуст -> проверка выключена -> alena/omazh/zahar доходили до MiniMax и молча
    заменялись китайским голосом. Теперь та же последовательность событий даёт нарушение с перечнем допустимых.
    """
    node = _node("minimax")
    _state(node, "minimax")
    _state(node, "prefetch")
    res = _judge(node, _tale("", "alena", "omazh", "zahar"))

    assert isinstance(res, Invalid) and res.reason == "bad_enum", res
    assert "alena" in res.violations[0].detail
    assert res.violations[0].allowed and all(
        v.startswith("Russian_") for v in res.violations[0].allowed
    )


def test_a_foreign_provider_label_set_by_another_writer_falls_back_to_the_configured_catalogue():
    """Читатель не доверяет полю, даже если его записал кто-то в обход ``on_tts_provider_state`` (v1)."""
    node = _node("yandex")
    node._actual_tts_provider = "prefetch"
    busy = plan_host.perform_busy(node)
    assert (
        busy.voices == tuple(role_voices_for("yandex", default_voice_for("yandex")))
        and busy.voices
    )
    res = _judge(node, _tale("", "Russian_BrightHeroine"))
    assert isinstance(res, Invalid) and res.reason == "bad_enum"


def test_without_actual_provider_and_parameter_the_robot_default_catalogue_is_used():
    busy = plan_host.perform_busy(_node(param=None))
    assert busy.voices and all(v.startswith("Russian_") for v in busy.voices)


def test_unknown_configured_provider_does_not_empty_the_catalogue():
    assert plan_host.perform_busy(_node(param="auto")).voices


def test_tts_provider_for_the_perform_host_skips_the_label_too():
    node = _node("minimax")
    node._actual_tts_provider = "prefetch"
    assert plan_host._tts_provider(node) == "minimax"
    _state(node, "yandex")
    assert plan_host._tts_provider(node) == "yandex"
