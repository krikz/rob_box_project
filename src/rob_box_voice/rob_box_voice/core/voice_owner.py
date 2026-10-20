"""VoiceOwner - единственный владелец выбранного голоса TTS (ADR-0150 §8, PR-4 ч.3, #3693).

``VoiceState{requested, applied, provider}`` живёт в ``tts_node``: он один знает, какой голос реально звучит
(фолбек провайдера, голос не из каталога). Тул ``set_voice`` ничего не хранит - он просит
(``/voice/tts/voice_request``) и ждёт ``SpeechEvent voice_changed{requested, applied, provider}``; фраза «Говорю
голосом X» / «Голос X недоступен, остаюсь на Y» строится кодом по этому событию (ADR-0148). Выбор переживает
рестарт: файл ``voice_state_file`` рядом с ``tts_provider_state.json`` (тот же механизм, что у tts_node; другие
файлы ``/data`` не трогаем).
"""

from __future__ import annotations

import json
import os
import types
from typing import Any, Callable, Dict

from ..tts_voice_registry import resolve_voice

PROVIDERS = ("yandex", "minimax", "silero")


def read_state(path: str) -> Dict[str, Any]:
    """Сохранённый выбор; нет файла или он битый - пусто (старт с дефолтов провайдера)."""
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def write_state(path: str, state: Dict[str, Any]) -> None:
    """Атомарная запись (tmp + replace), как у ``_persist_provider_state``."""
    tmp = path + ".tmp"
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(state, fh, ensure_ascii=False)
    os.replace(tmp, path)


def attach_voice_owner(node: Any) -> "VoiceOwner":
    """Владелец голоса на ноде ``tts_node``: параметр файла, подписка на запросы, восстановление после рестарта."""
    node.declare_parameter("voice_state_file", "/data/tts_voice_state.json")
    owner = VoiceOwner(node, str(node.get_parameter("voice_state_file").value), node_voice_changed_emitter(node))
    from std_msgs.msg import String  # noqa: PLC0415

    node.voice_request_sub = node.create_subscription(String, "/voice/tts/voice_request", owner.on_request, 10)
    owner.restore()
    return owner


def node_voice_changed_emitter(node: Any) -> Callable[..., None]:
    """Событие ``voice_changed`` через владельца речи ноды (ленивый импорт: цикл core -> speech_owner)."""
    from .speech_owner import owner_of  # noqa: PLC0415

    return lambda *args: owner_of(node).voice_changed(*args)


class VoiceOwner:
    """Применяет запросы смены голоса к ``tts_node`` и помнит результат."""

    def __init__(self, node: Any, path: str, emit: Callable[[str, str, str, str, str], None]) -> None:
        self._node = node
        self._path = path
        self._emit = emit  # (request_id, requested, applied, provider, reason)
        self.state: Dict[str, Any] = {}

    def restore(self) -> None:
        """Применить сохранённый выбор (после рестарта голос не сбрасывается на дефолт провайдера)."""
        saved = read_state(self._path)
        provider, applied = saved.get("provider"), saved.get("applied")
        if provider in PROVIDERS and applied:
            self._apply(str(provider), str(applied), "restore")
            self.state = saved
            self._node.get_logger().info(f"🎙️ голос восстановлен из {self._path}: {provider}/{applied}")

    def on_request(self, msg: Any) -> None:
        """``/voice/tts/voice_request`` ``{"request_id", "voice", "provider"?}`` -> применить -> событие."""
        try:
            payload = json.loads(msg.data or "{}")
        except (TypeError, ValueError):
            payload = {}
        rid, voice = str(payload.get("request_id") or ""), str(payload.get("voice") or "").strip()
        provider = str(payload.get("provider") or "").strip().lower() or str(self._node._effective_provider() or "")
        if provider not in PROVIDERS:
            self._emit(rid, voice, "", provider, "unknown_provider")
            return
        applied, fell_back = resolve_voice(provider, voice)
        self._apply(provider, applied, "voice_request")
        self.state = {"provider": provider, "requested": voice, "applied": applied}
        if self._path:
            write_state(self._path, self.state)
        self._emit(rid, voice, applied, provider, "voice_unavailable" if voice and fell_back else "")

    def _apply(self, provider: str, voice: str, source: str) -> None:
        """Выставить провайдера (цепочка) и голос в живой параметр ``tts_node``, переопубликовать ``provider_state``."""
        if provider != self._node._effective_provider():
            self._node._on_set_provider(types.SimpleNamespace(data=json.dumps(
                {"provider": provider, "voice": voice, "source": source})))
        param_name, display = self._node._param_for_set_voice_provider(provider)
        self._node._apply_set_voice(param_name, display, provider, voice, source)
        self._node._publish_provider_state("voice_changed", provider=provider, voice=voice)
