"""Unit-тесты ``voice_picker.validate_voice_choice`` — pure-функции для issue #2138.

Тесты чистые — не требуют rclpy/geometry_msgs/audio_common_msgs, чтобы
запускаться на dev-env без Docker (соответствует pattern
``test_voice_state.py``, ``test_battery.py`` и др. в этом каталоге).

Контекст (см. docs/plans/2026-09-08-operator-agent-live-debug-handoff.md
§6 «Открытое» и task body t_0980642f):

До фикса ``QuestBridge.set_voice`` валидировал ``voice_id`` через
компайл-тайм реестр ``rob_box_voice.tts_voice_registry``. В образе
``rob-box-quest`` этого пакета нет (как и в mcp_tools — см. защищённый
импорт с fallback ``[]``). В итоге на роботе ``voices_for(provider)``
всегда возвращал пустой список → ``set_voice`` ВСЕГДА отдавал
``tts_unreachable`` для любого голоса, при живом TTS-стеке. Issue #2138.

Фикс: источник истины — то, что tts_node реально опубликовал в
latched-топик ``/voice/tts/voices`` (его кэш уже держит QuestBridge).
Кэш — это ``list[dict[...]]`` от ``voice_info_for``, валидация чистая.

Этот модуль выделяет чистую часть (cache + voice_id → verdict), чтобы
её можно было тестировать без ROS-стека и без импорта ``QuestBridge``
(который тянет ``audio_common_msgs`` и skip'ается на dev-env).
"""

from __future__ import annotations

import pytest

from rob_box_quest.streams.voice_picker import (
    VOICE_UNAVAILABLE_REASON,
    VOICE_UNREACHABLE_REASON,
    VoiceChoice,
    pick_voice,
)


# --- Базовый контракт -------------------------------------------------------


class TestPickVoiceContract:
    """pick_voice — чистая функция, контракт из task body t_0980642f."""

    def test_cache_with_voice_returns_ok_with_applied_id(self):
        cache = [
            {"voice_id": "alena"},
            {"voice_id": "anton"},
        ]
        result = pick_voice(cache, voice_id="alena")
        assert result == VoiceChoice(ok=True, reason=None, available=None)

    def test_cache_empty_returns_tts_unreachable(self):
        """Пустой кэш = tts_node ещё не прислал список. Честный nack.

        Это capability-honest поведение из task body: «если кэш пуст или
        протух — честный tts_unreachable, не молчаливый 'ок'».
        """
        result = pick_voice([], voice_id="alena")
        assert result == VoiceChoice(
            ok=False,
            reason=VOICE_UNREACHABLE_REASON,
            available=None,
        )

    def test_voice_not_in_cache_returns_voice_unavailable_with_available(self):
        """Голоса нет в кэше → voice_unavailable + список доступных id.

        Контракт ws_server_voice.py теста (line ~664):
        ``sorted(available) == ["alena", "anton"]`` — то есть список
        id в виде list[str], не dict-структуры (клиенту нужно для UI).
        """
        cache = [
            {"voice_id": "alena"},
            {"voice_id": "anton"},
        ]
        result = pick_voice(cache, voice_id="bogus")
        assert result.ok is False
        assert result.reason == VOICE_UNAVAILABLE_REASON
        assert sorted(result.available or []) == ["alena", "anton"]

    def test_voice_id_match_against_mixed_cache_ignores_extra_fields(self):
        """Голоса в кэше идут с полями display_name/language/gender/presets —
        мы должны матчить только по voice_id, игнорируя остальное.
        """
        cache = [
            {
                "voice_id": "alena",
                "display_name": "Алёна",
                "language": "ru-RU",
                "gender": "female",
                "presets": ["standard", "friendly"],
                "provider": "yandex",
            },
        ]
        result = pick_voice(cache, voice_id="alena")
        assert result.ok is True

    def test_voice_id_match_is_strict_string(self):
        """Никаких fuzzy/contains — точное равенство строк."""
        cache = [{"voice_id": "alena"}]
        # 'alen' — НЕ 'alena', не должно проходить.
        result = pick_voice(cache, voice_id="alen")
        assert result.ok is False
        assert result.reason == VOICE_UNAVAILABLE_REASON


# --- Capability-honest: пустой кэш ≠ ОК -------------------------------------


class TestCapabilityHonestEmptyCache:
    """Главный регресс #2138: пустой кэш НЕ молча «ок».

    На роботе кэш приходит (latched-топик от tts_node), но в текущем
    коде валидация идёт через ``voices_for(provider)`` из
    ``tts_voice_registry`` — который на образе ``rob-box-quest`` отсутствует
    (fallback ``[]``). В итоге set_voice ВСЕГДА возвращал tts_unreachable
    при живом провайдере. Этот набор тестов фиксирует правильное
    поведение: cache empty → nack, cache has voices → use them.
    """

    def test_cache_populated_after_latched_publish_accepts_valid_voice(self):
        """Имитация «tts_node прислал /voice/tts/voices» → cache has voices.

        Имитация post-fix: pick_voice работает ТОЛЬКО по cache. Если
        tts_node прислал реальный список — голос из него должен
        проходить валидацию, а НЕ блокироваться фоллбэком «не в
        реестре» (как было до фикса).
        """
        cache = [
            {"voice_id": "alena"},
            {"voice_id": "anton"},
        ]
        # 'alena' есть в кэше → ок, даже если бы «реестр» его не знал.
        result = pick_voice(cache, voice_id="alena")
        assert result.ok is True

    def test_cache_with_unknown_voice_returns_unavailable_not_unreachable(self):
        """Если голоса нет в КЭШЕ, но сам кэш непустой — это voice_unavailable,
        не tts_unreachable. Иначе UI врёт: «TTS недостижим» хотя TTS живой.
        """
        cache = [{"voice_id": "anton"}]  # только Антон
        result = pick_voice(cache, voice_id="alena")
        assert result.ok is False
        assert result.reason == VOICE_UNAVAILABLE_REASON
        # available заполнен — UI-подсказка.
        assert result.available == ["anton"]

    def test_cache_none_treated_as_empty(self):
        """Cache может быть None (на свежем старте до первого snapshot)."""
        result = pick_voice(None, voice_id="alena")  # type: ignore[arg-type]
        assert result.ok is False
        assert result.reason == VOICE_UNREACHABLE_REASON


# --- Граничные случаи -------------------------------------------------------


class TestPickVoiceEdgeCases:
    def test_cache_with_empty_voice_id_entry_is_ignored(self):
        """Голос без voice_id в кэше — мусор, не должен мешать валидации."""
        cache = [
            {"voice_id": ""},
            {"display_name": "no-id"},  # нет voice_id
            {"voice_id": "alena"},
        ]
        # 'alena' всё равно находится.
        result = pick_voice(cache, voice_id="alena")
        assert result.ok is True

    def test_available_dedupes_repeating_voice_ids(self):
        """Если в кэше voice_id задвоился — available не должен дублировать."""
        cache = [
            {"voice_id": "alena"},
            {"voice_id": "alena"},
            {"voice_id": "anton"},
        ]
        result = pick_voice(cache, voice_id="bogus")
        assert result.reason == VOICE_UNAVAILABLE_REASON
        # Дубликат 'alena' НЕ должен попасть в available.
        assert sorted(result.available or []) == ["alena", "anton"]

    def test_voice_picker_returns_namedtuple_instance(self):
        """VoiceChoice — NamedTuple для удобства сравнения и печати."""
        result = pick_voice([], voice_id="x")
        assert isinstance(result, VoiceChoice)
        # NamedTuple даёт доступ через поля.
        assert hasattr(result, "ok")
        assert hasattr(result, "reason")
        assert hasattr(result, "available")


# --- Регресс #2138: «cache populated but voice rejected» --------------------


class TestRegression2138:
    """Регресс из task body t_0980642f (issue #2138.A).

    До фикса ``set_voice`` ходил через ``_voices_for(provider)`` из
    реестра. В образе ``rob-box-quest`` реестра нет → fallback ``[]``
    → ВСЕГДА ``tts_unreachable``, при живом TTS.

    Эти тесты — после выделения ``pick_voice`` — фиксируют, что
    валидация работает по КЭШУ (реальному ответу провайдера), а не по
    компайл-тайм реестру. Это даёт следующему агенту простой способ
    проверить: «fix сломался?» — запустил тест, увидел PASS/FAIL.
    """

    def test_post_fix_scenario_voice_accepted_when_cache_says_so(self):
        """Главный сценарий #2138: cache populated, voice_id in cache → OK."""
        cache = [
            {"voice_id": "male-qn-qingse"},  # MiniMax legacy
            {"voice_id": "Russian_ReliableMan"},
        ]
        # 'Russian_ReliableMan' есть в кэше → голос подтверждён.
        result = pick_voice(cache, voice_id="Russian_ReliableMan")
        assert result.ok is True, (
            "Главный сценарий #2138: при наличии голоса в КЭШЕ "
            "валидация должна пройти. Если это не так — фикс "
            "вернул валидацию через tts_voice_registry и сломал "
            "выбор голоса на роботе."
        )

    def test_post_fix_scenario_cache_empty_returns_unreachable(self):
        """Если кэш пуст (tts_node ещё не прислал, или упал) — tts_unreachable.

        Не «голос найден в реестре и валиден» — это capability-honest:
        если провайдер реально ничего не отдал, мы не делаем вид, что
        всё ОК.
        """
        # Кэш пустой (latched-publish от tts_node ещё не пришёл).
        result = pick_voice([], voice_id="alena")
        assert result.ok is False
        assert result.reason == VOICE_UNREACHABLE_REASON
        assert result.available is None

    def test_post_fix_scenario_available_is_list_of_strings_not_dicts(self):
        """available в возврате — list[str], не list[dict].

        Контракт фиксируется тестами test_quest_bridge.py:665 и
        test_ws_server_voice.py — клиенту нужен список id.
        """
        cache = [
            {"voice_id": "alena", "display_name": "Алёна"},
            {"voice_id": "anton", "display_name": "Антон"},
        ]
        result = pick_voice(cache, voice_id="bogus")
        assert result.reason == VOICE_UNAVAILABLE_REASON
        # Каждый элемент available — это строка, не dict.
        for v in result.available or []:
            assert isinstance(v, str), (
                f"available должен быть list[str], получили {type(v).__name__}: {v!r}"
            )


# --- Sanity: pytest parametrize ---------------------------------------------


@pytest.mark.parametrize(
    "voice_id,expected_ok,expected_reason",
    [
        ("alena", True, None),
        ("anton", True, None),
        ("bogus", False, VOICE_UNAVAILABLE_REASON),
    ],
)
def test_pick_voice_parametrized(voice_id, expected_ok, expected_reason):
    cache = [{"voice_id": "alena"}, {"voice_id": "anton"}]
    result = pick_voice(cache, voice_id=voice_id)
    assert result.ok is expected_ok
    assert result.reason == expected_reason
