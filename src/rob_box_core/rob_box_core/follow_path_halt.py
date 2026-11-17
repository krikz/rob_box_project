"""Общие имена остановки «сироты» FollowPath (#4435, #4458): один путь для тулов навигации и сторожа.

``FOLLOW_PATH_CANCEL_SRV`` — сервис отмены целей ``controller_server``; ``NAV_CMD_VEL_TOPIC`` — выход
``controller_server`` в ``twist_mux`` (вход navigation), куда идёт нулевая скорость. Тулы навигации
(``rob_box_mcp_tools.tools.navigation.ControllerHalt``) и нода-сторож (``rob_box_teleop.followpath_watchdog_node``)
берут их отсюда.
"""

FOLLOW_PATH_CANCEL_SRV = "/follow_path/_action/cancel_goal"
NAV_CMD_VEL_TOPIC = "/cmd_vel"
#: Сколько нулевых ``Twist`` подряд публикуется при остановке (с запасом на потерю одного кадра).
ZERO_TWIST_REPEATS = 3
