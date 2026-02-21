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
import time
import traceback
from pathlib import Path
from typing import Any, Optional

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
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

QOS_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10,
)


# ── ROS2 нода ────────────────────────────────────────────────────────────────

class ScenarioRunner(Node):
    def __init__(self):
        super().__init__("scenario_runner")

        # Publishers
        self.stt_pub = self.create_publisher(String, TOPIC_STT, 10)
        self.vad_pub = self.create_publisher(Bool, TOPIC_VAD, 10)

        # Subscribers (накапливаем последние сообщения)
        self._last_response: Optional[str] = None
        self._last_animation: Optional[str] = None
        self._last_sound: Optional[str] = None
        self._response_ts: float = 0.0

        self.create_subscription(String, TOPIC_RESPONSE, self._on_response, 10)
        self.create_subscription(String, TOPIC_ANIMATION, self._on_animation, 10)
        self.create_subscription(String, TOPIC_SOUND, self._on_sound, 10)

        self.get_logger().info("ScenarioRunner ready")

    def _on_response(self, msg: String):
        self._last_response = msg.data
        self._response_ts = time.time()
        self.get_logger().info(f"[response] {msg.data[:80]}")

    def _on_animation(self, msg: String):
        self._last_animation = msg.data

    def _on_sound(self, msg: String):
        self._last_sound = msg.data

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

    def wait_for_response(self, timeout_s: float) -> Optional[str]:
        """Ждать первый ответ на /voice/dialogue/response до timeout_s секунд."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_response is not None:
                return self._last_response
        return None


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
            # Ждём немного и убеждаемся что ответа НЕТ
            time.sleep(timeout_s)
            rclpy.spin_once(node, timeout_sec=0.1)
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

    # Ждём Zenoh/ROS2 готовности (dialogue_node должен быть виден)
    print(f"\n[runner] Waiting for dialogue_node to appear...")
    deadline = time.time() + 60
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=1.0)
        names = node.get_node_names()
        if "dialogue_node" in names:
            print(f"[runner] dialogue_node found!")
            break
    else:
        print("[runner] WARNING: dialogue_node not found, continuing anyway")

    # Небольшой прогрев
    time.sleep(2.0)

    # Загружаем сценарии
    scenarios_path = Path(SCENARIOS_DIR)
    scenario_files = sorted(scenarios_path.glob("*.yaml"))
    print(f"\n[runner] Found {len(scenario_files)} scenario file(s)")

    if not scenario_files:
        print(f"[runner] ERROR: No scenario files in {SCENARIOS_DIR} — aborting")
        sys.exit(1)

    all_results = []
    for path in scenario_files:
        print(f"\n[runner] Loading {path.name}")
        with open(path) as f:
            data = yaml.safe_load(f)

        scenarios = data.get("scenarios", [])
        for scenario in scenarios:
            result = run_scenario(node, scenario)
            all_results.append(result)

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

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if failed_n == 0 else 1)


if __name__ == "__main__":
    main()
