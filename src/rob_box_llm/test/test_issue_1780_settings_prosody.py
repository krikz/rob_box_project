"""Юнит-тесты для issue #1780: комплексный «TTSSettings → MiniMax T2A v2 payload».

Этот файл — **дополнительный** слой покрытия к существующим тестам:

* ``test_minimax_tts_provider.py::TestBuildPayload::*`` — проверяет
  отдельные поля (voice_id / speed / format / volume in range / vol out
  of range) по одному.
* ``test_minimax_tts_provider_extra.py`` — проверяет allow-list для
  ``extra`` (анти-инъекция).
* ``test_minimax_provider_httpx.py`` — end-to-end через ``MockTransport``.

Здесь мы покрываем сценарии, которые выпали из этих разрезов:

1. **Все 4 поля prosody (emotion/pitch/volume/pronunciation_dict) в одной
   ``TTSSettings``** → полный ``voice_setting`` и top-level payload
   (issue #1780 — фикс из t_4e98182a, «feature»-коммит
   ``a72227a3``). Существующие тесты берут поля по одному, поэтому
   регрессия вида «volume есть, но pitch пропал» прошла бы.

2. **Граничные значения ``volume`` в стыке с ``pitch``/``emotion``** —
   ранее ``test_volume_in_range_accepted`` и
   ``test_volume_out_of_range_rejected`` запускались на минимальной
   ``TTSSettings``. Нужно подтвердить, что валидация диапазона
   [0.0, 10.0] работает даже когда другие поля prosody заполнены.

3. **Безмолвное исключение SSML-атрибутов на MiniMax-пути** — MiniMax
   берёт emotion/pitch/volume из ``TTSSettings``, а не из
   ``ssml_attributes``. Это намеренно: MiniMax и Yandex имеют разные
   семантики SSML, и tts_node._synthesize_minimax_async НЕ передаёт
   ssml_attributes в MiniMax. Этот тест защищает решение, чтобы при
   будущих рефакторингах кто-то не «унифицировал» пути и не сломал
   спецификацию MiniMax.
"""

from __future__ import annotations

import pytest

from rob_box_llm.errors import TTSBadRequestError
from rob_box_llm.providers.minimax_tts import _build_payload
from rob_box_llm.tts import TTSFormat, TTSSettings


# ─────────────────────────────────────────────────────────────────────────────
# 1. Все 4 поля prosody одновременно → корректный voice_setting
# ─────────────────────────────────────────────────────────────────────────────


class TestAllFourProsodyFieldsTogether:
    """emotion + pitch + volume + pronunciation_dict в одной TTSSettings.

    Сценарий из task body: «все 4 поля корректно прокидываются».
    Минимальная единица, которая ловит регрессию вида «volume
    пробрасывается, а pitch/emotion — нет».
    """

    def test_voice_setting_has_emotion_pitch_vol(self):
        """emotion/pitch/volume попадают в ``voice_setting.*``."""
        s = TTSSettings(
            voice="russian_calm_woman",
            emotion="happy",
            pitch=3,
            volume=1.5,
        )
        payload = _build_payload(
            "privet", s, stream=False,
            default_voice="russian_calm_woman",
            default_model="speech-02-hd",
        )
        vs = payload["voice_setting"]
        assert vs["voice_id"] == "russian_calm_woman"
        assert vs["emotion"] == "happy"
        assert vs["pitch"] == 3
        assert vs["vol"] == 1.5

    def test_pronunciation_dict_is_top_level_not_inside_voice_setting(self):
        """MiniMax T2A v2: ``pronunciation_dict`` идёт top-level.

        Это критично — спецификация MiniMax разделяет ``voice_setting``
        (голос/эмоция/тон/громкость) и верхний уровень (custom
        произношение). Если кто-то «оптимизирует» и засунет его в
        ``voice_setting``, API вернёт 400 и юзер услышит
        произношение по дефолту.
        """
        pron = {"tone": ["Alice/ˈælɪs"], "phoneme": ["read/rɛd"]}
        s = TTSSettings(pronunciation_dict=pron)
        payload = _build_payload(
            "privet", s, stream=False,
            default_voice="russian_calm_woman",
            default_model="speech-02-hd",
        )
        # Top-level.
        assert payload["pronunciation_dict"] == pron
        # НЕ внутри voice_setting.
        assert "pronunciation_dict" not in payload["voice_setting"]

    def test_all_four_fields_together_full_payload(self):
        """Все 4 поля одновременно — финальная структура payload.

        Это интеграционный тест «от и до»: собираем TTSSettings,
        прогоняем через ``_build_payload``, проверяем всю форму.
        """
        pron = {"tone": ["Иван/ˈivan"]}
        s = TTSSettings(
            voice="russian_calm_woman",
            emotion="neutral",
            pitch=-2,
            volume=2.5,
            pronunciation_dict=pron,
            language="ru",
            speed=1.1,
            sample_rate=16000,
            format=TTSFormat.PCM,
        )
        payload = _build_payload(
            "privet", s, stream=False,
            default_voice="russian_calm_woman",
            default_model="speech-02-hd",
        )

        # voice_setting — компактная, все 4 поля + базовые.
        vs = payload["voice_setting"]
        assert vs == {
            "voice_id": "russian_calm_woman",
            "speed": 1.1,
            "vol": 2.5,
            "pitch": -2,
            "emotion": "neutral",
            "language": "Russian",
        }

        # pronunciation_dict — top-level.
        assert payload["pronunciation_dict"] == pron

        # audio_setting — стандартные дефолты.
        assert payload["audio_setting"] == {
            "sample_rate": 16000,
            "bitrate": 128000,
            "format": "pcm",
            "channel": 1,
        }

        # model/text/stream — стандартная обвязка.
        assert payload["model"] == "speech-02-hd"
        assert payload["text"] == "privet"
        assert payload["stream"] is False


# ─────────────────────────────────────────────────────────────────────────────
# 2. Граничные значения volume при заполненных других prosody-полях
# ─────────────────────────────────────────────────────────────────────────────


class TestVolumeBoundariesWithProsody:
    """Валидация [0.0, 10.0] должна срабатывать даже если заполнены
    emotion/pitch/pronunciation_dict — раньше такие тесты гонялись
    на минимальной TTSSettings и могли не покрыть «volume вне
    диапазона в комбинации»."""

    @pytest.mark.parametrize(
        "bad_volume",
        [-0.1, -1.0, 10.0001, 11.0, 100.0],
    )
    def test_volume_out_of_range_raises_even_with_other_prosody(self, bad_volume):
        """volume вне [0.0, 10.0] + emotion/pitch/pronunciation_dict →
        TTSBadRequestError.

        Защищает от регрессии вида «если заполнен pitch, валидация
        volume пропускается».
        """
        s = TTSSettings(
            voice="russian_calm_woman",
            emotion="happy",
            pitch=3,
            volume=bad_volume,
            pronunciation_dict={"tone": ["x/y"]},
        )
        with pytest.raises(TTSBadRequestError) as exc:
            _build_payload(
                "x", s, stream=False,
                default_voice="russian_calm_woman",
                default_model="speech-02-hd",
            )
        # Сообщение должно содержать и значение, и диапазон — оператор
        # в логах должен сразу понять, что чинить.
        msg = str(exc.value)
        assert "out of range" in msg
        assert str(bad_volume) in msg
        assert "[0.0, 10.0]" in msg

    @pytest.mark.parametrize("boundary_volume", [0.0, 10.0])
    def test_volume_at_boundary_accepted_with_prosody(self, boundary_volume):
        """0.0 и 10.0 включительно — граница диапазона MiniMax T2A v2."""
        s = TTSSettings(
            voice="russian_calm_woman",
            emotion="neutral",
            pitch=0,
            volume=boundary_volume,
            pronunciation_dict={"phoneme": ["read/rɛd"]},
        )
        payload = _build_payload(
            "x", s, stream=False,
            default_voice="russian_calm_woman",
            default_model="speech-02-hd",
        )
        assert payload["voice_setting"]["vol"] == boundary_volume
        # pitch=0 — это явное значение, оно попадает в payload как int 0.
        assert payload["voice_setting"]["pitch"] == 0
        # pronunciation_dict не должен «потеряться» на границе.
        assert payload["pronunciation_dict"] == {"phoneme": ["read/rɛd"]}


# ─────────────────────────────────────────────────────────────────────────────
# 3. Безмолвное исключение SSML на MiniMax-пути
# ─────────────────────────────────────────────────────────────────────────────


class TestMiniMaxIgnoresSSMLAttributes:
    """Контракт: MiniMax использует emotion/pitch/volume из
    ``TTSSettings`` (поля ROS-параметров), а НЕ из ``<prosody>``
    в SSML. Yandex использует SSML; MiniMax — нет. Это намеренно,
    и tts_node._synthesize_minimax_async НЕ передаёт ssml_attributes
    в ``TTSSettings``."""

    def test_minimax_settings_take_emotion_not_ssml(self):
        """TTSSettings.emotion берётся из settings, а не из SSML."""
        # В tts_node._synthesize_minimax_async ssml_attributes НЕ
        # конвертируются в emotion/pitch/volume (только ``rate``/``speed``).
        # Этот тест фиксирует контракт на уровне MiniMax-провайдера:
        # provider видит только то, что в TTSSettings.
        s = TTSSettings(emotion="happy", pitch=2, volume=3.0)
        payload = _build_payload(
            "x", s, stream=False,
            default_voice="russian_calm_woman",
            default_model="speech-02-hd",
        )
        # В payload нет «ssml» или «prosody» — MiniMax-провайдер
        # ничего не знает про SSML-парсер.
        assert "ssml" not in payload
        assert "prosody" not in payload
        # Эмоция и pitch/volume — от settings.
        assert payload["voice_setting"]["emotion"] == "happy"
        assert payload["voice_setting"]["pitch"] == 2
        assert payload["voice_setting"]["vol"] == 3.0

    def test_minimax_pronunciation_dict_overrides_extra(self):
        """``TTSSettings.pronunciation_dict`` побеждает ``extra``."""
        # В MiniMax-провайдере есть fallback: ``extra`` может
        # содержать ``pronunciation_dict``. Если заданы оба — побеждает
        # типизированное поле (см. комментарий в minimax_tts.py:305).
        pron_typed = {"tone": ["typed/ˈtʌɪpt"]}
        pron_extra = {"tone": ["extra/ˈɛkstrə"]}
        s = TTSSettings(
            pronunciation_dict=pron_typed,
            extra={"pronunciation_dict": pron_extra},
        )
        payload = _build_payload(
            "x", s, stream=False,
            default_voice="russian_calm_woman",
            default_model="speech-02-hd",
        )
        assert payload["pronunciation_dict"] == pron_typed
        assert "pronunciation_dict" not in payload["voice_setting"]


# ─────────────────────────────────────────────────────────────────────────────
# 4. Default-ы при пустых полях — поведение pre-#1780
# ─────────────────────────────────────────────────────────────────────────────


class TestDefaultsPreserved:
    """Pre-#1780 поведение: пустые emotion/pitch/volume/pronunciation_dict
    → отсутствуют в voice_setting (или эмоция = ``neutral``)."""

    def test_minimal_settings_no_prosody(self):
        """TTSSettings() без полей → минимальный voice_setting."""
        s = TTSSettings(voice="russian_calm_woman")
        payload = _build_payload(
            "x", s, stream=False,
            default_voice="russian_calm_woman",
            default_model="speech-02-hd",
        )
        vs = payload["voice_setting"]
        assert vs == {"voice_id": "russian_calm_woman"}
        # Нет pitch/vol/emotion/pronunciation_dict.
        assert "pitch" not in vs
        assert "vol" not in vs
        assert "emotion" not in vs
        # pronunciation_dict — top-level — тоже отсутствует.
        assert "pronunciation_dict" not in payload

    def test_empty_pronunciation_dict_omitted(self):
        """Пустой dict ``pronunciation_dict`` (а не None) тоже не попадает
        в payload — MiniMax отвергает пустые override-ы."""
        s = TTSSettings(pronunciation_dict={})
        payload = _build_payload(
            "x", s, stream=False,
            default_voice="russian_calm_woman",
            default_model="speech-02-hd",
        )
        # В minimax_tts.py:336 стоит ``if pronunciation_dict:`` —
        # пустой dict трактуется как отсутствующий.
        assert "pronunciation_dict" not in payload