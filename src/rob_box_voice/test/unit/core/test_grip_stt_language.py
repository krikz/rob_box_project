"""Язык панели идёт только в STT грипа и возвращается на ru (Refs #4694)."""

from __future__ import annotations

from rob_box_voice.core.grip_stt_language import (
    DIALOG_STT_LANGUAGE,
    grip_providers,
    panel_language,
    recognition_for,
    refusal,
)

DIALOG = ("ru-RU", "ru", ["yandex", "minimax", "vosk"])


def test_panel_language_reads_voice_pipeline_json():
    raw = '{"llm_enabled": false, "preset": "", "language": "fr"}'
    assert panel_language(raw) == "fr"
    assert panel_language("не json") is None
    assert panel_language("") is None


def test_grip_language_fr_does_not_change_wake_and_returns_to_ru():
    """grip language=fr не меняет wake language; после grip снова ru."""
    grip = "fr"
    wake = recognition_for("wake", grip, DIALOG)
    mic = recognition_for("respeaker", grip, DIALOG)
    ptt = recognition_for("ptt", grip, DIALOG)

    assert wake["yandex_language"] == "ru-RU"
    assert wake["minimax_language"] == "ru"
    assert wake["only"] == ["yandex", "minimax", "vosk"]
    assert wake["refuse"] is None
    assert mic == wake

    assert ptt["yandex_language"] == "fr-FR"
    assert ptt["minimax_language"] == "fr"
    assert ptt["only"] == ["yandex", "minimax"]
    assert "vosk" not in ptt["only"]
    assert ptt["refuse"] is None

    after = recognition_for("wake", grip, DIALOG)
    assert after["yandex_language"] == "ru-RU"
    assert DIALOG[0] == "ru-RU"
    assert grip == "fr"


def test_after_grip_dialog_language_is_ru_again():
    assert recognition_for("ptt", "fr", DIALOG)["yandex_language"] == "fr-FR"
    assert (
        recognition_for("respeaker", DIALOG_STT_LANGUAGE, DIALOG)["yandex_language"]
        == "ru-RU"
    )
    assert recognition_for("wake", "fr", DIALOG)["minimax_language"] == "ru"


def test_unsupported_grip_language_refuses_instead_of_empty_or_russian():
    choice = recognition_for("ptt", "zh", ("ru-RU", "ru", ["yandex", "vosk"]))
    assert choice["refuse"] == refusal("zh")
    assert choice["only"] == []
    assert "русск" not in choice["refuse"]
    assert choice["refuse"]
    assert (
        recognition_for("wake", "zh", ("ru-RU", "ru", ["yandex", "vosk"]))["refuse"]
        is None
    )


def test_hi_needs_minimax_vosk_is_not_a_foreign_recognizer():
    assert grip_providers(
        "hi", ["yandex", "minimax", "vosk"], yandex=True, minimax=True
    ) == ["minimax"]
    assert grip_providers("hi", ["yandex", "vosk"], yandex=True, minimax=False) == []
    assert "хинди" in refusal("hi")
