"""rtabmap_mode.py - режим RTAB-Map из параметра самой ноды rtabmap (issue #4352).

Раньше ``context_aggregator_node`` читал ``/maps/mapping_state.json``. Такого файла в контейнере
perception нет (замер 04.11.2026: ``ls: cannot access '/maps/mapping_state.json'``), а
``rob_box_mcp_tools.mapping_state`` с тех пор держит состояние в памяти и файл не пишет. Поэтому
``mapping_mode`` всегда был ``unknown``.

Источник правды - нода ``/rtabmap/rtabmap``: её параметр ``Mem/IncrementalMemory`` равен ``true``
в режиме картографирования и ``false`` в режиме локализации. На роботе 04.11.2026
``ros2 param get /rtabmap/rtabmap Mem/IncrementalMemory`` вернул ``String value is: false``.

Чистый Python, без rclpy: на вход - поля ``rcl_interfaces/ParameterValue`` (``type``,
``bool_value``, ``string_value``).
"""

from __future__ import annotations

from typing import Any, Optional

MAPPING = 'mapping'
LOCALIZATION = 'localization'
UNKNOWN = 'unknown'

#: Параметр rtabmap, который задаёт режим.
MODE_PARAMETER = 'Mem/IncrementalMemory'

#: ``rcl_interfaces/ParameterType``: PARAMETER_BOOL = 1, PARAMETER_STRING = 4.
_PARAMETER_BOOL = 1
_PARAMETER_STRING = 4

#: Ответ старше этого - режим ``unknown``: rtabmap мог упасть или смениться.
STALE_AFTER_S = 15.0


def mode_from_parameter(value: Any) -> str:
    """``ParameterValue`` параметра ``Mem/IncrementalMemory`` -> ``mapping``/``localization``/``unknown``."""
    if value is None:
        return UNKNOWN
    kind = int(getattr(value, 'type', 0))
    if kind == _PARAMETER_BOOL:
        return MAPPING if bool(value.bool_value) else LOCALIZATION
    if kind == _PARAMETER_STRING:
        text = str(value.string_value).strip().lower()
        if text == 'true':
            return MAPPING
        if text == 'false':
            return LOCALIZATION
    return UNKNOWN


def current_mode(mode: str, answered_at: Optional[float], now: float) -> str:
    """Режим для публикации: последний ответ rtabmap, если он свежий, иначе ``unknown``."""
    if answered_at is None or now - answered_at > STALE_AFTER_S:
        return UNKNOWN
    return mode


class RtabmapModeProbe:
    """Опрос режима у ``/rtabmap/rtabmap`` через ``GetParameters`` (асинхронно, без блокировки ноды).

    ``client`` - клиент сервиса ``rcl_interfaces/srv/GetParameters`` (``None`` - сервиса нет,
    режим всегда ``unknown``), ``make_request`` - фабрика ``GetParameters.Request``.
    """

    def __init__(self, client: Any, make_request: Any, clock: Any, logger: Any = None) -> None:
        """Запомнить клиента сервиса, фабрику запроса, часы и логгер."""
        self._client = client
        self._make_request = make_request
        self._clock = clock
        self._logger = logger
        self._mode = UNKNOWN
        self._answered_at: Optional[float] = None

    def poll(self) -> None:
        """Спросить ``Mem/IncrementalMemory``, если сервис готов; ответ придёт в ``on_response``."""
        if self._client is None or not self._client.service_is_ready():
            return
        request = self._make_request()
        request.names = [MODE_PARAMETER]
        self._client.call_async(request).add_done_callback(self.on_response)

    def on_response(self, future: Any) -> None:
        """Ответ ``GetParameters``: запомнить режим и время ответа."""
        try:
            values = list(future.result().values)
        except Exception as exc:  # noqa: BLE001
            if self._logger is not None:
                self._logger.warning(f'rtabmap get_parameters: {exc!r}')
            return
        self._mode = mode_from_parameter(values[0] if values else None)
        self._answered_at = self._clock()

    def mode(self, now: float) -> str:
        """Режим на момент ``now``: свежий ответ rtabmap или ``unknown``."""
        return current_mode(self._mode, self._answered_at, now)
