#!/usr/bin/env python3
"""
scenario_runner/runner.py — Исполнитель интеграционных тестов для dialogue_node.

Логика:
  1. Читает все *.yaml файлы из SCENARIOS_DIR
  2. Для каждого сценария выполняет шаги:
     - inject_stt      → публикует строку в /voice/stt/result
     - inject_vad      → публикует bool в /audio/vad
     - wait_ms         → пауза без публикации
  3. После каждого inject_stt ждёт ответ на /voice/dialogue/response
  4. Проверяет:
       assert_response_contains    — ответ содержит строку/список строк
       assert_response_not_empty   — ответ не пустой
       assert_response_valid_json  — ответ парсится как JSON
       assert_response_has_key     — JSON ответ содержит ключ
       assert_animation            — last animation содержит строку
       assert_no_response          — ответа быть не должно
  5. Выводит результаты и записывает results.json
  6. Выход с кодом 0 (все прошли) или 1 (есть провалы)

LLM: Ollama (qwen2.5:0.5b) — OpenAI-совместимый API на порту 11434.

Запуск: python3 runner.py
"""

import json
import os
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any, Optional

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String


# ── Конфигурация ──────────────────────────────────────────────────────────────

SCENARIOS_DIR = os.getenv("SCENARIOS_DIR", "/scenarios")
RESULTS_FILE = os.getenv("RESULTS_FILE", "/results/test_results.json")
OVERALL_TIMEOUT = int(os.getenv("OVERALL_TIMEOUT", "120"))

TOPIC_STT = "/voice/stt/result"
TOPIC_VAD = "/audio/vad"
TOPIC_RESPONSE = "/voice/dialogue/response"
TOPIC_ANIMATION = "/voice/animation/request"
TOPIC_SOUND = "/voice/sound/trigger"

TOPIC_MCP_TOOLS = "/mcp/tools"
TOPIC_MCP_EXECUTE = "/mcp/execute"
TOPIC_MCP_RESULT = "/mcp/result"

QOS_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10,
)
QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── Mock tools list — полный набор тулов как на реальном роботе ──────────────
MOCK_MCP_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "speak_text",
            "description": (
                "Произнести текст голосом через TTS. "
                "ИСПОЛЬЗУЙ ЭТО вместо возврата JSON с SSML. "
                "ОБЯЗАТЕЛЬНО указывай animation — покажет анимацию на LED матрице робота."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "text": {"type": "string", "description": "Текст для произнесения"},
                    "animation": {
                        "type": "string",
                        "description": "Анимация: happy, sad, angry, surprised, thinking, idle, victory, police_lights и др.",
                        "default": "neutral",
                    },
                },
                "required": ["text"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "listen_for_response",
            "description": (
                "Прослушать ответ пользователя (активирует STT). "
                "ПОСЛЕ вызова этого инструмента НЕЛЬЗЯ вызывать НИКАКИЕ другие инструменты!"
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "timeout_seconds": {"type": "integer", "description": "Время ожидания (сек)", "default": 5},
                    "prompt_text": {"type": "string", "description": "Подсказка о том, чего ожидаем"},
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "play_sound",
            "description": "Воспроизвести звуковой эффект",
            "parameters": {
                "type": "object",
                "properties": {
                    "sound_name": {"type": "string", "description": "Название звука (robot_happy, ui_chime и др.)"}
                },
                "required": ["sound_name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "play_animation",
            "description": "Воспроизвести анимацию на LED матрице робота",
            "parameters": {
                "type": "object",
                "properties": {
                    "animation_name": {"type": "string", "description": "Название анимации"},
                    "duration": {"type": "number", "description": "Длительность (сек)"},
                },
                "required": ["animation_name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_robot_status",
            "description": "Получить текущий статус робота: заряд батареи, местоположение, состояние",
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_waypoint",
            "description": "Отправить робота к указанной точке или комнате",
            "parameters": {
                "type": "object",
                "properties": {
                    "waypoint": {"type": "string", "description": "Название точки или комнаты"}
                },
                "required": ["waypoint"],
            },
        },
    },
]


# ── ROS2 нода ────────────────────────────────────────────────────────────────

class ScenarioRunner(Node):
    def __init__(self):
        super().__init__("scenario_runner")

        # Publishers
        self.stt_pub = self.create_publisher(String, TOPIC_STT, 10)
        self.vad_pub = self.create_publisher(Bool, TOPIC_VAD, 10)

        # ── Mock MCP Server ──────────────────────────────────────────────────
        self.mcp_tools_pub = self.create_publisher(String, TOPIC_MCP_TOOLS, QOS_RELIABLE)
        self.mcp_result_pub = self.create_publisher(String, TOPIC_MCP_RESULT, QOS_RELIABLE)
        # Publisher для имитации speech response (speak_text → dialogue/response)
        self.dialogue_resp_pub = self.create_publisher(String, TOPIC_RESPONSE, 10)
        self.create_subscription(
            String, TOPIC_MCP_EXECUTE, self._on_mcp_execute, QOS_RELIABLE
        )
        # Периодически рассылаем список тулов чтобы dialogue_node его получил
        self.create_timer(2.0, self._publish_mock_tools)
        # Лог вызовов для assert_tool_called
        self._mcp_calls: list[dict] = []

        # Subscribers (накапливаем последние сообщения)
        self._last_response: Optional[str] = None
        self._last_animation: Optional[str] = None
        self._last_sound: Optional[str] = None
        self._response_ts: float = 0.0
        self._response_event = threading.Event()  # Сигнализируем main-потоку без spin_once

        self.create_subscription(String, TOPIC_RESPONSE, self._on_response, 10)
        self.create_subscription(String, TOPIC_ANIMATION, self._on_animation, 10)
        self.create_subscription(String, TOPIC_SOUND, self._on_sound, 10)

        self.get_logger().info("ScenarioRunner ready (mock MCP enabled)")

    def _on_response(self, msg: String):
        self._last_response = msg.data
        self._response_ts = time.time()
        self._response_event.set()  # Будим wait_for_response
        self.get_logger().info(f"[response] {msg.data[:80]}")

    def _on_animation(self, msg: String):
        self._last_animation = msg.data

    def _on_sound(self, msg: String):
        self._last_sound = msg.data

    # ── Mock MCP ─────────────────────────────────────────────────────────────

    def _publish_mock_tools(self):
        msg = String()
        msg.data = json.dumps(MOCK_MCP_TOOLS, ensure_ascii=False)
        self.mcp_tools_pub.publish(msg)

    def _on_mcp_execute(self, msg: String):
        """Перехватываем tool call от dialogue_node и отвечаем mock-результатом."""
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"[mock-mcp] Bad execute request: {msg.data[:80]}")
            return

        tool_name = req.get("tool_name", "unknown")
        parameters = req.get("parameters", {})
        request_id = req.get("request_id", "")

        self._mcp_calls.append({"tool_name": tool_name, "parameters": parameters})
        self.get_logger().info(f"[mock-mcp] CALL: {tool_name}({parameters})")

        # Mock ответ — всегда success
        # ── Если speak_text — публикуем текст как dialogue response ──────────
        if tool_name == "speak_text":
            text = parameters.get("text", "")
            animation = parameters.get("animation", "neutral")
            if text.strip():
                resp_msg = String()
                resp_msg.data = json.dumps({
                    "chunk": "final",
                    "ssml": f"<speak>{text}</speak>",
                    "emotion": animation,
                    "message": "mock speak_text",
                }, ensure_ascii=False)
                self.dialogue_resp_pub.publish(resp_msg)
                self.get_logger().info(f"[mock-mcp] speak_text → /voice/dialogue/response: {text[:60]}")

        mock_results = {
            "speak_text": {
                "success": True, "message": f"Произношение: {parameters.get('text', '')[:40]}"
            },
            "listen_for_response": {
                "success": True, "heard": "", "message": "Ожидание завершено (mock)"
            },
            "play_sound": {
                "success": True, "sound": parameters.get("sound_name", "")
            },
            "get_robot_status": {
                "battery": 87, "location": "гостиная", "state": "idle"
            },
            "navigate_to_waypoint": {
                "accepted": True,
                "destination": parameters.get("waypoint", "неизвестно")
            },
            "play_animation": {
                "accepted": True,
                "animation": parameters.get("animation_name", "default")
            },
        }
        result_data = mock_results.get(tool_name, {"success": True})

        result_msg = String()
        result_msg.data = json.dumps({
            "tool_name": tool_name,
            "request_id": request_id,
            "result": {"success": True, "data": result_data},
        }, ensure_ascii=False)
        self.mcp_result_pub.publish(result_msg)
        self.get_logger().info(f"[mock-mcp] RESULT sent for {tool_name}")

    # ── Actions ──────────────────────────────────────────────────────────────

    def inject_stt(self, text: str):
        msg = String()
        msg.data = text
        self.stt_pub.publish(msg)
        self.get_logger().info(f"[inject_stt] {text!r}")

    def inject_vad(self, active: bool):
        msg = Bool()
        msg.data = active
        self.vad_pub.publish(msg)

    def clear_received(self):
        self._last_response = None
        self._last_animation = None
        self._last_sound = None
        self._response_ts = 0.0
        self._response_event.clear()  # Сбрасываем event для следующего ожидания
        self._mcp_calls.clear()

    def wait_for_response(self, timeout_s: float) -> Optional[str]:
        """Ждать ответ на /voice/dialogue/response.
        Использует threading.Event — основной поток НЕ блокирует ROS executor.
        """
        self._response_event.wait(timeout=timeout_s)
        return self._last_response


# ── Шаг сценария ─────────────────────────────────────────────────────────────

def run_step(node: ScenarioRunner, step: dict) -> tuple[bool, str]:
    """Выполнить один шаг. Возвращает (passed, message)."""

    # inject_stt
    if "inject_stt" in step:
        node.clear_received()
        node.inject_stt(step["inject_stt"])

        timeout_s = step.get("timeout_s", 15.0)
        no_response_expected = step.get("assert_no_response", False)

        if no_response_expected:
            # Ждём немного и убеждаемся что ответа НЕТ — executor спинит фоново
            time.sleep(timeout_s)
            if node._last_response is not None:
                return False, f"Unexpected response received: {node._last_response[:80]}"
            return True, "No response (as expected)"

        # Ждём ответ
        response = node.wait_for_response(timeout_s)
        if response is None:
            return False, f"No response within {timeout_s}s"

        # Парсим JSON ответ dialogue_node
        response_text = _extract_text(response)

        # assert_response_not_empty
        if step.get("assert_response_not_empty"):
            if not response_text.strip():
                return False, "Response is empty"

        # assert_response_contains
        if "assert_response_contains" in step:
            expected = step["assert_response_contains"]
            if isinstance(expected, str):
                expected = [expected]
            for phrase in expected:
                if phrase.lower() not in response_text.lower():
                    return False, f"Response {response_text!r} does not contain {phrase!r}"

        # assert_response_valid_json
        if step.get("assert_response_valid_json"):
            try:
                json.loads(response)
            except json.JSONDecodeError as e:
                return False, f"Response is not valid JSON: {e}"

        # assert_response_has_key
        if "assert_response_has_key" in step:
            key = step["assert_response_has_key"]
            try:
                data = json.loads(response)
                if key not in data:
                    return False, f"JSON response missing key {key!r}. Keys: {list(data.keys())}"
            except json.JSONDecodeError as e:
                return False, f"Cannot check key — response is not JSON: {e}"

        # assert_animation
        if "assert_animation" in step:
            expected_anim = step["assert_animation"]
            actual_anim = node._last_animation or ""
            if expected_anim not in actual_anim:
                return False, f"Animation {actual_anim!r} does not contain {expected_anim!r}"

        # assert_tool_called — проверяем что MCP тул был вызван
        if "assert_tool_called" in step:
            expected_tool = step["assert_tool_called"]
            called = [c["tool_name"] for c in node._mcp_calls]
            if expected_tool not in called:
                return False, f"Tool {expected_tool!r} was not called. Called: {called}"

        return True, f"OK: {response_text[:60]}"

    # inject_vad
    elif "inject_vad" in step:
        node.inject_vad(bool(step["inject_vad"]))
        return True, f"VAD injected: {step['inject_vad']}"

    # wait_ms
    elif "wait_ms" in step:
        time.sleep(step["wait_ms"] / 1000.0)
        return True, f"Waited {step['wait_ms']}ms"

    # set_llm_responses — устарело (было для mock-llm), пропускаем
    elif "set_llm_responses" in step:
        node.get_logger().warning("set_llm_responses step is deprecated (mock-llm removed), skipping")
        return True, "SKIP: set_llm_responses (mock-llm not used)"

    else:
        return False, f"Unknown step keys: {list(step.keys())}"


def _extract_text(response_json: str) -> str:
    """Извлечь текст из JSON ответа dialogue_node."""
    try:
        data = json.loads(response_json)
        ssml = data.get("ssml", "")
        # Убираем теги SSML
        import re
        text = re.sub(r"<[^>]+>", "", ssml).strip()
        return text or response_json
    except json.JSONDecodeError:
        return response_json


# ── Запуск сценария ───────────────────────────────────────────────────────────

def run_scenario(node: ScenarioRunner, scenario: dict) -> dict:
    name = scenario.get("name", "unnamed")
    steps = scenario.get("steps", [])
    results = []
    passed = True

    print(f"\n{'='*60}")
    print(f"  SCENARIO: {name}")
    print(f"{'='*60}")

    for i, step in enumerate(steps):
        try:
            ok, msg = run_step(node, step)
        except Exception as e:
            ok = False
            msg = f"Exception: {e}\n{traceback.format_exc()}"

        status = "✓ PASS" if ok else "✗ FAIL"
        print(f"  Step {i+1}: {status} — {msg}")
        results.append({"step": i + 1, "passed": ok, "message": msg, "step_def": step})
        if not ok:
            passed = False
            if scenario.get("fail_fast", True):
                break

    overall = "PASS" if passed else "FAIL"
    print(f"\n  → Scenario {overall}")
    return {"name": name, "passed": passed, "steps": results}


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ScenarioRunner()

    # ── Фоновый executor — спиним в отдельном потоке, никогда не блокируем main ──
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # ── Глобальный дедлайн ────────────────────────────────────────────────────
    global_deadline = time.time() + OVERALL_TIMEOUT

    # Ждём Zenoh/ROS2 готовности (dialogue_node должен быть виден)
    print(f"\n[runner] Waiting for dialogue_node to appear...")
    wait_dl = time.time() + 60
    while time.time() < wait_dl:
        names = node.get_node_names()
        if "dialogue_node" in names:
            print(f"[runner] dialogue_node found!")
            break
        time.sleep(1.0)
    else:
        print("[runner] WARNING: dialogue_node not found, continuing anyway")

    # Небольшой прогрев — даём dialogue_node получить MCP tools list
    time.sleep(3.0)

    # Загружаем сценарии
    scenarios_path = Path(SCENARIOS_DIR)
    scenario_files = sorted(scenarios_path.glob("*.yaml"))
    print(f"\n[runner] Found {len(scenario_files)} scenario file(s)")

    if not scenario_files:
        print(f"[runner] ERROR: No scenario files in {SCENARIOS_DIR} — aborting")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    all_results = []
    for path in scenario_files:
        if time.time() > global_deadline:
            print(f"[runner] OVERALL_TIMEOUT={OVERALL_TIMEOUT}s exceeded, stopping")
            break

        print(f"\n[runner] Loading {path.name}")
        with open(path) as f:
            data = yaml.safe_load(f)

        scenarios = data.get("scenarios", [])
        for scenario in scenarios:
            if time.time() > global_deadline:
                print(f"[runner] OVERALL_TIMEOUT exceeded, skipping remaining scenarios")
                all_results.append({
                    "name": scenario.get("name", "unknown"),
                    "passed": False,
                    "steps": [],
                    "skipped": True,
                })
                continue

            result = run_scenario(node, scenario)
            all_results.append(result)

            # Пауза между сценариями — dialogue_node должен вернуться в IDLE
            # (после speak_text нода делает ещё один LLM запрос ~2-3s)
            time.sleep(5.0)

    # Итоги
    total = len(all_results)
    passed_n = sum(1 for r in all_results if r["passed"])
    failed_n = total - passed_n

    print(f"\n{'='*60}")
    print(f"  RESULTS: {passed_n}/{total} passed")
    for r in all_results:
        icon = "✓" if r["passed"] else "✗"
        print(f"  {icon} {r['name']}")
    print(f"{'='*60}\n")

    # Сохраняем JSON
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    with open(RESULTS_FILE, "w") as f:
        json.dump({
            "total": total,
            "passed": passed_n,
            "failed": failed_n,
            "scenarios": all_results,
        }, f, indent=2, ensure_ascii=False)
    print(f"[runner] Results saved to {RESULTS_FILE}")

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if failed_n == 0 else 1)


if __name__ == "__main__":
    main()
