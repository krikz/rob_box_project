"""М12 n909 (#4236): «скажи по-китайски спокойной ночи» - языки звучащего голоса кладёт в окно код, по вендору TTS.

В М11 робот (MiniMax) сказал «вань ань», в М12 отказался «только по-русски»: в окне не было факта, что звучащий
движок умеет. Решение Шифу 05.11: решать от возможностей вендора - MiniMax говорит любым голосом на любом языке,
Silero - только по-русски. Проверяют раздел ``tts_context`` и окно (ADR-0148), не текст промпта.
"""

from __future__ import annotations

import json
import types

from rob_box_core.msg_stub import install_dialog_msgs_stub

install_dialog_msgs_stub()

from rob_box_dialog.context import build_context  # noqa: E402

from rob_box_voice.core import plan_host  # noqa: E402
from rob_box_dialog_host.tts_voice_registry import ANY_LANGUAGE, spoken_languages  # noqa: E402


def _node(provider: str, applied: str | None = None) -> types.SimpleNamespace:
    voice = types.SimpleNamespace(applied=applied, provider=provider if applied else None)
    return types.SimpleNamespace(_actual_tts_provider=provider, _actual_tts_voice=None,
                                 _session=types.SimpleNamespace(snapshot=types.SimpleNamespace(voice=voice)))


def _window_languages(provider: str, applied: str | None = None):
    section = plan_host.tts_context(_node(provider, applied))
    window = build_context({"tts_context": section})
    assert json.dumps(section, ensure_ascii=False, separators=(",", ":")) in window
    return section["languages"]


def test_minimax_any_voice_speaks_any_language() -> None:
    """MiniMax: язык задаёт ``language_boost`` при любом голосе - в окне «любой», у русского голоса и у английского."""
    assert _window_languages("minimax") == ANY_LANGUAGE
    assert _window_languages("minimax", "Russian_ReliableMan") == ANY_LANGUAGE
    assert _window_languages("minimax", "English_Jovialman") == ANY_LANGUAGE


def test_silero_speaks_only_russian() -> None:
    """Silero (``v5_ru``): только русский - модель честно говорит, что этим голосом не умеет."""
    assert _window_languages("silero") == ["ru"]
    assert _window_languages("silero", "baya") == ["ru"]


def test_yandex_language_is_the_sounding_voice_language() -> None:
    """Yandex: язык прибит к голосу; реплика v2 поля ``language`` не ставит, поэтому - язык звучащего голоса."""
    assert _window_languages("yandex") == ["ru"]
    assert _window_languages("yandex", "john") == ["en"]


def test_unknown_provider_claims_nothing() -> None:
    """Про незнакомый провайдер языков не выдумываем."""
    assert spoken_languages("nobody") == []
