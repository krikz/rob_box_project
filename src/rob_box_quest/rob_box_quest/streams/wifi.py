"""Wi-Fi RSSI для robot_status (Wave 3.A / R8).

ROS-топика с Wi-Fi-качеством в проекте нет, поэтому читаем локальный
``/proc/net/wireless`` на той машине, где крутится ``rob_box_quest``
(Vision Pi). Это тот же линк, по которому идёт WSS до Quest — то есть
именно та цифра, которая интересует оператора.

Формат ``/proc/net/wireless`` (ядро Linux, `net/wireless/wext-proc.c`)::

    Inter-| sta-|   Quality        |   Discarded packets   | Missed | WE
     face | tus | link level noise |  nwid  crypt   frag  retry | beacon | 22
     wlan0: 0000   62.  -48.  -256        0      0      0     0      0        0

Поле ``level`` (3-я колонка Quality) — RSSI в dBm. Значения приходят
с точкой на конце (``-48.``) — это не float, а «текущее значение»
в терминах wext.

Модуль — чистая логика: путь к файлу инжектится, что позволяет
тестировать без Wi-Fi (см. test/unit/streams/test_wifi.py).
"""

from __future__ import annotations

from typing import Optional

PROC_NET_WIRELESS = "/proc/net/wireless"


def parse_wireless(text: str, iface: Optional[str] = None) -> Optional[int]:
    """Достать RSSI (dBm) из содержимого ``/proc/net/wireless``.

    :param text: содержимое файла целиком.
    :param iface: имя интерфейса (``wlan0``); ``None`` — взять первый
        попавшийся (на Pi Wi-Fi-интерфейс ровно один).
    :return: RSSI в dBm (обычно от -30 до -90) или ``None``, если строку
        не удалось разобрать / интерфейс не найден.
    """
    for line in text.splitlines():
        if ":" not in line:
            continue  # шапка таблицы (2 строки)
        name, _, rest = line.partition(":")
        name = name.strip()
        if not name:
            continue
        if iface is not None and name != iface:
            continue
        parts = rest.split()
        # [status, quality_link, quality_level, quality_noise, ...]
        if len(parts) < 3:
            continue
        raw = parts[2].rstrip(".")
        try:
            return int(float(raw))
        except ValueError:
            continue
    return None


def read_wifi_rssi(path: str = PROC_NET_WIRELESS, iface: Optional[str] = None) -> Optional[int]:
    """Прочитать RSSI из ``/proc/net/wireless``.

    Возвращает ``None`` на любой ошибке (нет файла — например, на
    dev-машине под Windows, нет прав, пустая таблица). Вызывающий код
    обязан пережить ``None``: в payload уйдёт sentinel 0.
    """
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            text = fh.read()
    except OSError:
        return None
    return parse_wireless(text, iface=iface)
