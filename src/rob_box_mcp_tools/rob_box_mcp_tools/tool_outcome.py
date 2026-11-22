"""Строка лога об исходе тула — из результата исполнения, не из флага ``success`` вслепую.

Живёт отдельно от ``engine.song_order`` (#5092): тот тянет студию и ROS-соседей, а эту строку
печатают и ``mcp_server``, и сухой стенд. Стенд из поставленного ядра диалога видит только
неполный ``rob_box_mcp_tools`` (``packaging/dialog-core``): модуль без импортов, чтобы стенд
не разошёлся с роботом и не тянул студию.
"""

from typing import Any, Dict, Optional, Tuple

#: Значения ``data["pending"]``, при которых тул только ПРИНЯЛ фоновый заказ, а не исполнил его.
#: «выполнен успешно» про такой заказ — ложь: исход скажет поток, когда закончит (#5092, ADR-0148).
PENDING_ORDERS = ("song", "instrumental")


def tool_outcome(
    tool_name: str, success: bool, data: Optional[Dict[str, Any]], error: Optional[str]
) -> Tuple[str, str]:
    """``(уровень, строка)`` исхода тула: ``warning`` — провал, ``info`` — успех и принятый заказ.

    Уровень решает код из результата, не первый символ строки: харнесс и разбор лога ищут отказ
    на WARNING, как раньше."""
    line = tool_outcome_line(tool_name, success, data, error)
    return ("info" if success else "warning"), line


def tool_outcome_line(tool_name: str, success: bool, data: Optional[Dict[str, Any]], error: Optional[str]) -> str:
    """Успех — «выполнен успешно». Провал — «завершился с ошибкой» и текст ошибки.

    Фоновый заказ (``data["pending"]`` из :data:`PENDING_ORDERS`) — «принят, готовится»: работа
    ещё идёт, и строка успеха про неё врала бы (акт 7, #5092: успех, через 14 мс отказ студии)."""
    pending = (data or {}).get("pending") if success else None
    if pending in PENDING_ORDERS:
        return f"⏳ Инструмент {tool_name} принят, готовится: {pending}"
    if success:
        return f"✅ Инструмент {tool_name} выполнен успешно"
    return f"❌ Инструмент {tool_name} завершился с ошибкой: {error}"
