"""Утилиты разработчика для rob_box_core (issue #2199 follow-up).

CLI-запускаемые скрипты, не ROS-зависимые. Запуск:

    python3 -m rob_box_core.tools.check_ts_sync

Зачем отдельный модуль
----------------------
Эти утилиты не нужны рантайму rob_box и не должны попадать в
ament/colcon install-space как часть пакета (они для разработчика и CI).
Выделены в подмодуль ``tools``, чтобы было видно: это **не** часть
stable API ``rob_box_core``.
"""