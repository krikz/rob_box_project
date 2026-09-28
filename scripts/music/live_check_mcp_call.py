#!/usr/bin/env python3
"""live_check_mcp_call.py — один подписанный вызов MCP-тула в обход LLM (#3115).

Запускается ВНУТРИ контейнера ``voice-assistant`` (там же живёт
``mcp_server``), скрипт ``live_check_3115.sh`` подаёт его через stdin::

    docker exec -i voice-assistant bash -lc \
        'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; \
         python3 - compose_music <params_b64> 90' < live_check_mcp_call.py

Путь тот же, что у LLM: ``/mcp/execute`` → ``MCPServer.on_execute_request``
→ ``registry.execute``. Запрос подписывается штатным
:class:`rob_box_mcp_tools.mcp_auth.RequestAuthenticator` с sender
``harness`` (он есть в ``DEFAULT_ALLOWED_SENDERS`` и в срезе
``personality`` из ``data/slice_policy.yaml``, где лежат
``execute_music_code`` / ``compose_music`` / ``stop_music``). Секрет —
тот же файл ``/data/.mcp_token`` (или ``ROB_BOX_MCP_TOKEN``), что читает
``mcp_server``; по топику ходит только подпись.

Печатает в stdout ОДНУ строку JSON — ответ ``/mcp/result`` целиком
(``{"tool_name", "request_id", "result": {...}}``); в ``result.data.code``
у ``execute_music_code``/``compose_music`` лежит реально исполненный код
после санитайзера. Код выхода: 0 — ответ получен (успех тула смотри в
``result.success``), 2 — таймаут, 3 — нечем подписать.
"""

from __future__ import annotations

import base64
import json
import sys
import time
import uuid


def main(argv: list) -> int:
    if len(argv) < 3:
        print("usage: live_check_mcp_call.py <tool_name> <params_json_b64> [timeout_s]", file=sys.stderr)
        return 64
    tool_name = argv[1]
    params = json.loads(base64.b64decode(argv[2]).decode("utf-8"))
    timeout_s = float(argv[3]) if len(argv) > 3 else 90.0

    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from rob_box_mcp_tools.mcp_auth import RequestAuthenticator

    auth = RequestAuthenticator.from_env(sender="harness")
    if not auth.can_sign:
        print(json.dumps({"error": "нет общего секрета /mcp/execute — подписать нечем"}, ensure_ascii=False))
        return 3

    rclpy.init()
    node = rclpy.create_node("live_check_3115")
    # Тот же профиль, что у mcp_server (reliable, keep_last 10).
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    request_id = f"live3115-{uuid.uuid4().hex[:12]}"
    box: dict = {}

    def on_result(msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if payload.get("request_id") == request_id:
            box["payload"] = payload

    node.create_subscription(String, "/mcp/result", on_result, qos)
    pub = node.create_publisher(String, "/mcp/execute", qos)

    # Ждём discovery: без подписчика reliable-сообщение уйдёт в никуда.
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"[live_check] /mcp/execute subscribers={pub.get_subscription_count()}", file=sys.stderr)

    request = {"tool_name": tool_name, "parameters": params, "request_id": request_id}
    auth.sign(request)  # подпись ставится прямо перед публикацией (окно clock_skew)
    msg = String()
    msg.data = json.dumps(request, ensure_ascii=False)
    pub.publish(msg)
    print(f"[live_check] sent {tool_name} request_id={request_id}", file=sys.stderr)

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and "payload" not in box:
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()
    if "payload" not in box:
        print(json.dumps({"error": f"таймаут {timeout_s:.0f}s: нет /mcp/result для {request_id}"}, ensure_ascii=False))
        return 2
    print(json.dumps(box["payload"], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
