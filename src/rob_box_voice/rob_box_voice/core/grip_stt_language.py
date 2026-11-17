"""Язык распознавания только для грипа (левый PTT шлема).

Панель шлема уже шлёт язык в ``/avatar/voice_pipeline`` (поле ``language``).
До этого модуля тот же JSON кормил только TTS грипа. Общий параметр
``yandex_language`` / ``minimax_stt_language`` трогать нельзя: им пользуются
ReSpeaker и wake-поток, и смена на fr сломала бы русский диалог.

Решение здесь, не в промпте (ADR-0148): код знает, какой язык умеет облачный
STT, и на грипе ставит его только на время одной фразы. Если не умеет —
честный отказ, а не пустая строка и не русский текст чужой речи.
"""

from __future__ import annotations

import json
from typing import Optional

DIALOG_STT_LANGUAGE = "ru"
GRIP_STT_TOPIC = "/avatar/voice_pipeline"

# Коды панели (voice_presets.yaml) → код, который провайдер реально принимает.
# Yandex SpeechKit v3: https://yandex.cloud/en/docs/speechkit/stt/models
# MiniMax speech-to-text: language = ISO-639, либо не передавать (auto).
_YANDEX_CODES: dict[str, str] = {
    "ru": "ru-RU",
    "en": "en-US",
    "de": "de-DE",
    "fr": "fr-FR",
}
# Панель шлема: ru en fr de zh hi (voice_presets.yaml / LANG_ORDER).
_MINIMAX_CODES: frozenset[str] = frozenset({"ru", "en", "fr", "de", "zh", "hi"})

_LANGUAGE_NAMES_RU: dict[str, str] = {
    "ru": "русском",
    "en": "английском",
    "fr": "французском",
    "de": "немецком",
    "zh": "китайском",
    "hi": "хинди",
}


def panel_language(raw: str) -> Optional[str]:
    """Код языка из JSON ``/avatar/voice_pipeline``; битый payload — ``None``."""
    text = (raw or "").strip()
    if not text:
        return None
    try:
        data = json.loads(text)
    except (json.JSONDecodeError, TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    language = str(data.get("language") or "").strip().lower().split("-")[0]
    return language or None


def yandex_code(language: str) -> Optional[str]:
    """BCP-47 для Yandex или ``None``, если этот язык Yandex не распознаёт."""
    return _YANDEX_CODES.get(_short(language))


def minimax_code(language: str) -> Optional[str]:
    """ISO-639 для MiniMax STT или ``None``, если язык вне его списка."""
    code = _short(language)
    return code if code in _MINIMAX_CODES else None


def grip_providers(language: str, chain, *, yandex: bool, minimax: bool) -> list[str]:
    """Провайдеры цепочки, которые на ``language`` можно звать.

    Vosk образа знает только русский. На другом языке грипа его нет в списке:
    иначе фолбек отдаст русский текст чужой речи.
    """
    code = _short(language)
    allowed: list[str] = []
    for name in chain:
        if (
            name == "yandex"
            and yandex
            and (code == DIALOG_STT_LANGUAGE or yandex_code(code))
        ):
            allowed.append(name)
        elif (
            name == "minimax"
            and minimax
            and (code == DIALOG_STT_LANGUAGE or minimax_code(code))
        ):
            allowed.append(name)
        elif name == "vosk" and code == DIALOG_STT_LANGUAGE:
            allowed.append(name)
    return allowed


def recognition_for(
    source: str,
    grip_language: str,
    dialog: tuple,
    *,
    yandex: bool = True,
    minimax: bool = True,
) -> dict:
    """Чем распознавать эту фразу.

    ``source`` не ``ptt`` — всегда язык диалога, целиком цепочка. Грип —
    язык панели и только те провайдеры, которые его умеют. Решение не
    пишет параметры ноды: вызывающий подменяет их на время одного вызова.
    """
    yandex_language, minimax_language, chain = dialog
    if source != "ptt":
        return {
            "yandex_language": yandex_language,
            "minimax_language": minimax_language,
            "only": list(chain),
            "refuse": None,
        }
    allowed = grip_providers(grip_language, chain, yandex=yandex, minimax=minimax)
    return {
        "yandex_language": yandex_code(grip_language) or yandex_language,
        "minimax_language": minimax_code(grip_language) or minimax_language,
        "only": allowed,
        "refuse": None if allowed else refusal(grip_language),
    }


def grip_can_recognize(language: str, *, yandex: bool, minimax: bool) -> bool:
    """Есть ли живой облачный провайдер, который умеет распознать ``language``.

    Vosk в образе — только ``vosk-model-small-ru-0.22``. Для не-русского он
    не считается умением: русский текст чужой речи хуже честного отказа.
    """
    code = _short(language)
    if not code:
        return False
    if code == DIALOG_STT_LANGUAGE:
        return True
    return (yandex and yandex_code(code) is not None) or (
        minimax and minimax_code(code) is not None
    )


def refusal(language: str) -> str:
    """Фраза, которую код говорит, когда STT грипа язык не умеет (ADR-0148)."""
    code = _short(language) or language
    name = _LANGUAGE_NAMES_RU.get(code, code)
    return f"Не могу распознать речь на {name}: распознавание грипа этот язык не умеет."


def _short(language: str) -> str:
    return str(language or "").strip().lower().split("-")[0]


class GripSttLanguage:
    """Язык панели для STT грипа. Параметры диалога ноды не пишет.

    Нода держит один экземпляр и зовёт его из колбэка топика и из распознавания.
    Методы здесь, а не на ``STTNode``: класс ноды уже на потолке бюджета (ADR-0145).
    """

    def __init__(self, node) -> None:
        self._node = node
        self.language = DIALOG_STT_LANGUAGE

    def on_pipeline(self, msg) -> None:
        language = panel_language(getattr(msg, "data", "") or "")
        if not language:
            return
        self.language = language
        self._node.get_logger().info(
            f"grip STT language={language} (dialog stays {self._node.yandex_language})"
        )

    def recognize(self, audio_bytes: bytes, source: str):
        node = self._node
        choice = recognition_for(
            source,
            self.language,
            (
                node.yandex_language,
                node.minimax_stt_language,
                list(node.provider_chain),
            ),
            yandex=node.yandex_stub is not None,
            minimax=bool(node.minimax_stt_enabled and node.minimax_stt_api_key),
        )
        if choice["refuse"]:
            self.refuse(choice["refuse"])
            return None, []
        from rob_box_voice.stt_node import _run_recognition  # noqa: PLC0415

        return _run_recognition(
            node,
            audio_bytes,
            yandex_language=choice["yandex_language"],
            minimax_language=choice["minimax_language"],
            only=choice["only"],
        )

    def refuse(self, text: str) -> None:
        """Отказ, если STT язык не умеет (ADR-0148). Не пустая фраза."""
        from rob_box_core import speech_client
        from rob_box_core.utterance import Utterance as SpokenUtterance

        self._node.get_logger().warning(f"grip STT refusal: {text}")
        speech_client.say(
            self._node,
            SpokenUtterance(text=text, sink="speakers", priority="operator", language="ru-RU"),
            agent="operator",
        )
