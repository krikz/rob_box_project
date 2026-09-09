"""voice_state bridge: ROS2 → WS-фрейм (0x1202).

Источник истины для payload-стороны ROS:
    docs/recon/voice-dialogue-state-payload.md (t_1886f7be, parent recon).

Источник истины для wire-стороны WS:
    docs/architecture/meta-quest-api.md §4 (voice_state 0x1202).

Что делает этот модуль:

1. Хранит явную таблицу маппинга ``DialogueStateKind.name`` →
   bridge-``state`` (lowercase, стабильный контракт для клиента).
   Допустимые bridge-state'ы (``meta-quest-api.md §4``):
       "idle" | "listening" | "thinking" | "speaking"

2. ``normalize_voice_state(payload) -> dict`` — чистая функция, которая
   принимает ``std_msgs/String`` (или просто строку) и возвращает
   нормализованный dict ``{state, detail}`` для последующей сериализации
   через ``protocol.topics.encode_voice_state``. Функция намеренно
   сделана чистой и без зависимостей от ROS/rclpy, чтобы её можно было
   тестировать unit-тестом на dev-env без Docker.

3. Никакой публикации в ROS/WS здесь нет — это просто преобразователь.
   Подписка и отправка живут в ``quest_node._on_dialogue_state``.

Ключевые правила (решение по итогам recon t_1886f7be):
- ``IDLE``      → ``state="idle"``
- ``LISTENING`` → ``state="listening"``
- ``DIALOGUE``  → ``state="speaking"`` (LLM + TTS фаза — единый
   «громкий» state для UI; ``thinking`` спека отдельно не отдаёт, а
   DialogueStateKind его не различает)
- ``SILENCED``  → ``state="idle"``, ``detail="silenced"`` (робот молчит,
   для UI это отдельный визуальный режим «mute»; см. §5.1 recon-отчёта)
- Любая другая строка → ``state="idle"`` + ``WARNING`` лог (без краша,
   чтобы неустойчивый publisher не ронял ноду).

Прячем всё в модуле, чтобы в ``quest_node.py`` осталась одна короткая
подписка + encode_voice_state + publish.
"""

from __future__ import annotations

import logging
from typing import Any, Mapping

logger = logging.getLogger(__name__)


# --- Bridge contract (meta-quest-api.md §4) --------------------------------
# Набор допустимых bridge-значений state. meta-quest-api.md §4 объявляет
# "idle"|"listening"|"thinking"|"speaking"; "thinking" зарезервирован и
# сейчас не используется — нормализатор его не вернёт, но расширять
# таблицу можно одной строкой + тестом (см. правило ниже).
BRIDGE_STATES: tuple[str, ...] = ("idle", "listening", "thinking", "speaking")

# Имя bridge-state для случая «не смогли распознать» (см. правило про
# SILENCED и про «неизвестная строка»).
FALLBACK_STATE: str = "idle"


# --- DialogueStateKind.name → bridge state ---------------------------------
# Источник:
#   src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py:41-54
#   (DialogueStateKind: IDLE/LISTENING/DIALOGUE/SILENCED, значения .name).
#
# Ключи — UPPERCASE литералы, чтобы случайный lower-case из upstream не
# прокрался молча. Если FSM добавит новое значение (или upstream сменит
# регистр) — нормализатор выдаст WARNING и свалится на fallback. Это
# явный, не «втихую», сигнал для разработчика добавить ветку в таблицу.
#
# Контракт маппинга зафиксирован в recon-отчёте t_1886f7be §2/§5.2.
DIALOGUE_STATE_TO_BRIDGE: Mapping[str, tuple[str, str | None]] = {
    # DialogueStateKind.name → (bridge state, optional detail для UI)
    "IDLE":      ("idle",      None),
    "LISTENING": ("listening", None),
    "DIALOGUE":  ("speaking",  None),
    "SILENCED":  ("idle",      "silenced"),
}


# --- Public helpers --------------------------------------------------------

def _extract_state_str(payload: Any) -> str:
    """Достать строковое значение из ``payload``.

    Принимаем:
      - ``std_msgs/String`` (ROS2 msg) — берём ``.data``.
      - любой объект с ``.data: str``.
      - ``str`` / ``bytes`` (для тестов и юзер-френдли вызовов).

    Пустая/None строка → ``""`` (нормализатор дальше свалится на fallback
    с WARNING). Это сознательно: лучше видимый WARNING, чем молчаливый
    краш в encode_voice_state.
    """
    if payload is None:
        return ""
    if isinstance(payload, str):
        return payload
    if isinstance(payload, (bytes, bytearray)):
        try:
            return bytes(payload).decode("utf-8")
        except UnicodeDecodeError:
            return ""
    # std_msgs.String / generic wrapper with .data
    data = getattr(payload, "data", None)
    if data is None:
        return ""
    if isinstance(data, str):
        return data
    if isinstance(data, (bytes, bytearray)):
        try:
            return bytes(data).decode("utf-8")
        except UnicodeDecodeError:
            return ""
    return str(data)


def normalize_voice_state(
    payload: Any,
) -> dict[str, Any]:
    """Нормализовать ``payload`` ROS-топика ``/voice/dialogue/state``.

    Возвращает dict:

        {
            "state":  "idle" | "listening" | "thinking" | "speaking",
            "detail": str | None,
        }

    ``ts_ms`` (server clock) добавляется на стороне подписчика
    (``quest_node._on_dialogue_state``), нормализатор остаётся чисто
    stateless и не зависит от часов.

    Правила:
      - известное значение из ``DIALOGUE_STATE_TO_BRIDGE`` → пара
        (bridge_state, detail);
      - пустая строка → WARNING + ``FALLBACK_STATE``;
      - неизвестная непустая строка → WARNING + ``FALLBACK_STATE``;
        в лог пишем raw payload, чтобы dev видел, что пришло.

    Это сознательно видимое поведение (AGENTS.md / ADR-0018 «честный
    FAIL лучше красивого PASS»): лучше один WARNING на старте ноды +
    FALLBACK_STATE, чем silent-downgrade или краш.
    """
    raw = _extract_state_str(payload).strip()
    mapped = DIALOGUE_STATE_TO_BRIDGE.get(raw)

    if mapped is None:
        # Не нашли ни по пустой (raw == ""), ни по непустой строке.
        # Не крашим, но сигналим в лог: если upstream добавит новое
        # значение FSM, разработчик увидит WARNING и пополнит таблицу.
        if raw:
            logger.warning(
                "voice_state: unknown dialogue state %r → fallback to %r "
                "(обнови DIALOGUE_STATE_TO_BRIDGE в streams/voice_state.py)",
                raw, FALLBACK_STATE,
            )
        else:
            logger.debug(
                "voice_state: empty payload → fallback to %r",
                FALLBACK_STATE,
            )
        state = FALLBACK_STATE
        detail: str | None = None
    else:
        state, detail = mapped

    return {"state": state, "detail": detail}


__all__ = [
    "BRIDGE_STATES",
    "FALLBACK_STATE",
    "DIALOGUE_STATE_TO_BRIDGE",
    "normalize_voice_state",
]
