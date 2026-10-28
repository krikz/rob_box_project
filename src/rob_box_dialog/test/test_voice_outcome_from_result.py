"""Фраза смены голоса - из РЕЗУЛЬТАТА ``set_voice`` / ``set_tts_provider`` (ADR-0148), а не слова тула.

Живое 28.10 00:48:51, «Робот, говори обычным голосом»: тул сам озвучил «Я и так говорю своим обычным голосом.», а код сброса
тона следом - «Тон голоса обычный.». Теперь тул молчит, диалог говорит одну реплику по данным исхода. Данные собирает
``rob_box_mcp_tools.voice_request.outcome_data``; сверку поле в поле держит ``test_voice_outcome_data_contract`` там же.
"""

from __future__ import annotations

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases, respond
from rob_box_dialog.respond import ActionResult

NAMES = {"voice_name": "Весельчак", "requested_name": "Мудрый учёный", "provider_label": "Яндекс"}


def _result(tool, status, data, reason=None):
    return ActionResult("t/0", 0, "personality", tool, kn.TOOL_CLASSES[tool], status, reason, data)


@pytest.mark.parametrize("tool, status, data, reason, expected", [
    ("set_voice", "done", {"status": "applied", "voice_default": False}, None,
     "Говорю голосом Весельчак."),
    ("set_voice", "done", {"status": "applied", "voice_default": True}, None, "Говорю своим обычным голосом."),
    ("set_voice", "done", {"status": "already_default", "voice_default": True}, None,
     "Я и так говорю своим обычным голосом."),
    ("set_voice", "rejected", {"status": "fallback", "voice_default": False}, "voice_unavailable",
     "Голос Мудрый учёный недоступен, остаюсь на Весельчак."),
    ("set_voice", "rejected", {"status": "fallback", "voice_default": True}, "voice_unavailable",
     "Голос Мудрый учёный недоступен, остаюсь на обычном."),
    ("set_voice", "rejected", {"status": "fallback"}, "unknown_provider",
     "Не получилось сменить голос: провайдер Яндекс не подходит."),
    ("set_voice", "rejected", {"status": "timeout"}, "timeout",
     "Не дождался подтверждения смены голоса, не знаю, какой сейчас."),
    ("set_voice", "timeout", {}, None, "Не дождался подтверждения смены голоса, не знаю, какой сейчас."),
    ("set_tts_provider", "done", {"status": "applied", "voice_default": False}, None, "Говорю голосом Весельчак."),
    ("set_tts_provider", "done", {"status": "noop"}, None, "Я уже говорю через Яндекс."),
])
def test_voice_phrase_is_built_from_the_data_of_the_result(tool, status, data, reason, expected):
    utterance = respond.phrase_from_result(_result(tool, status, {**NAMES, **data}, reason))
    assert utterance is not None and utterance.text == expected


def test_result_without_owner_data_gets_the_honest_class_phrase_not_silence():
    """Отказ тула до владельца (пустой голос, чужой провайдер) данных исхода не несёт: «Не вышло.», а не молчание."""
    assert respond.phrase_from_result(_result("set_voice", "rejected", {})).text == phrases.text("local.rejected")


def test_every_key_of_the_voice_tables_is_in_the_catalog():
    keys = [*kn.VOICE_DONE.all_keys(), *kn.VOICE_FAILED.all_keys(), kn.VOICE_DEFAULT_PHRASE]
    assert [k for k in keys if k not in phrases.catalog()] == []


def test_voice_tools_do_not_speak_elsewhere():
    """Нет тулов голоса среди «говорит другой код»: об исходе одна фраза диалога (иначе тул и код говорят вдвоём)."""
    assert not {"set_voice", "set_tts_provider"} & set(kn.OUTCOME_SPOKEN_ELSEWHERE)
