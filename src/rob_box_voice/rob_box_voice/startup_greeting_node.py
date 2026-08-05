#!/usr/bin/env python3
"""
startup_greeting_node.py — Приветствие при старте voice-assistant (issue #1003).

Один раз при старте:
1. [звук] thinking (через /voice/sound/trigger) — сразу при инициализации
2. Ждём готовности voice-стека — публикатор на /voice/dialogue/response жив
   (значит dialogue_node и tts_node уже поднялись и могут принять TTS)
3. [звук] cute / very_cute (случайно)
4. [TTS] случайная прикольная фраза через /voice/dialogue/response
5. Завершаемся через 3 секунды (без once=True — Humble не поддерживает).

Раньше эта нода жила в rob_box_perception и ждала /perception/context_update.
После рефактора W10 (commit 7552418a) perception-пайплайн упростили и топик
пропал — переносим сюда, в rob_box_voice, и проверяем готовность через
обнаружение подписчиков на /voice/dialogue/response (tts_node).

Критерии (acceptance):
* после рестарта voice-assistant робот за 10–30 секунд говорит случайную
  прикольную фразу и издаёт звук;
* не перебивает диалог (запускается один раз, потом нода умирает);
* не ломает DJ-mode (запускается ДО любой LLM-сессии — DJ ещё не активен);
* звук "thinking" играется только при старте системы.
"""

from __future__ import annotations

import json
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StartupGreetingNode(Node):
    """Нода одноразового приветствия при старте системы."""

    # Прикольные варианты. Тон — дружелюбный кот-робот, не сухой.
    # Их сейчас 9, попадает под требование "6–10 вариантов".
    GREETINGS: tuple[str, ...] = (
        "Мяу, я на связи! Все системы в норме, можно играть!",
        "Привет-привет! Только что проснулся, готов к приключениям!",
        "Я вернулся! Датчики заряжены, мурчание включено.",
        "Здравствуйте! Загрузка завершена — скучать было некогда.",
        "Онлайн! Усы на месте, ушки на макушке.",
        "Готов помогать! Только не обижайте мои сенсоры.",
        "Рад вас слышать! Все модули мурлычут.",
        "Пи-пи-пип! Ой, то есть — доброе утро, хозяин.",
        "Слушаю вас внимательно. Ну, насколько микрофон позволяет.",
    )

    # Звуки, которые мы умеем триггерить через sound_node.
    THINKING_SOUND = "thinking"
    FINISH_SOUNDS = ("cute", "very_cute")

    # Топики — те же, что использует dialogue_node и tts_node.
    SOUND_TOPIC = "/voice/sound/trigger"
    TTS_TOPIC = "/voice/dialogue/response"

    # Готовность проверяем по наличию подписчика на TTS_TOPIC (tts_node жив).
    READINESS_TOPIC = TTS_TOPIC

    def __init__(self) -> None:
        super().__init__("startup_greeting_node")

        # Параметры. ``or default`` страхует от Mock-тестов, где
        # ``get_parameter(...).value`` возвращает None — и от опечаток в YAML.
        self.declare_parameter("wait_time", 5.0)         # секунд до первой проверки
        self.declare_parameter("check_timeout", 30.0)    # макс. ожидание готовности
        self.declare_parameter("enable_greeting", True)  # можно отключить для CI
        self.declare_parameter(
            "readiness_check_interval", 1.0
        )                                                # как часто опрашивать

        self.wait_time = self.get_parameter("wait_time").value or 5.0
        self.check_timeout = self.get_parameter("check_timeout").value or 30.0
        self.enable_greeting = bool(
            self.get_parameter("enable_greeting").value
            if self.get_parameter("enable_greeting").value is not None
            else True
        )
        self.readiness_check_interval = (
            self.get_parameter("readiness_check_interval").value or 1.0
        )

        # Publishers.
        self.sound_pub = self.create_publisher(String, self.SOUND_TOPIC, 10)
        self.tts_pub = self.create_publisher(String, self.TTS_TOPIC, 10)

        # Состояние.
        self.greeting_done = False
        self._start_time = time.monotonic()

        self.get_logger().info("Startup Greeting инициализирован (issue #1003)")
        self.get_logger().info("  → проигрываю звук загрузки...")
        # Сразу — thinking, чтобы юзер слышал "что-то происходит".
        self.play_sound(self.THINKING_SOUND)

        # Таймер проверки готовности.
        self.check_timer = self.create_timer(
            self.readiness_check_interval, self.check_readiness
        )

    # ------------------------------------------------------------------ sound
    def play_sound(self, sound_name: str) -> None:
        """Триггернуть звук через sound_node."""
        msg = String()
        msg.data = sound_name
        self.sound_pub.publish(msg)

    # ---------------------------------------------------------- readiness check
    def _tts_subscribers_present(self) -> bool:
        """True, если на /voice/dialogue/response есть хотя бы один подписчик.

        tts_node подписывается на этот топик при старте — пока он жив, мы
        можем безопасно слать туда приветствие.
        """
        try:
            info = self.get_subscriptions_info_by_topic(self.READINESS_TOPIC)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(
                f"get_subscriptions_info_by_topic не доступен: {exc}"
            )
            return False
        return bool(info)

    def check_readiness(self) -> None:
        """Тикает раз в секунду: когда стек готов — говорим и закрываемся."""
        if self.greeting_done:
            self.check_timer.cancel()
            return

        elapsed = time.monotonic() - self._start_time

        # Таймаут: лучше сказать приветствие поздно, чем не сказать.
        if elapsed > self.check_timeout:
            self.get_logger().warning(
                f"Таймаут готовности ({self.check_timeout:.1f}s) — "
                "говорю приветствие вслепую"
            )
            self.say_greeting()
            return

        # Слишком рано.
        if elapsed < self.wait_time:
            return

        # tts_node подключился — пора.
        if self._tts_subscribers_present():
            self.get_logger().info("tts_node подключён — говорю приветствие")
            self.say_greeting()

    # ---------------------------------------------------------- greeting logic
    def say_greeting(self) -> None:
        """Произнести случайное приветствие. Запускается строго один раз."""
        if self.greeting_done or not self.enable_greeting:
            return

        self.greeting_done = True

        # 1) Радостный звук.
        sound = random.choice(self.FINISH_SOUNDS)
        self.get_logger().info(f"Проигрываю звук: {sound}")
        self.play_sound(sound)

        # 2) Пауза, чтобы sound_node успел отработать и не наложиться на TTS.
        time.sleep(1.5)

        # 3) TTS-фраза.
        greeting = random.choice(self.GREETINGS)
        self.get_logger().info(f'Говорю: "{greeting}"')

        payload = json.dumps(
            {"ssml": f"<speak>{greeting}</speak>"},
            ensure_ascii=False,
        )
        msg = String()
        msg.data = payload
        self.tts_pub.publish(msg)

        # 4) Через 3 секунды завершаем ноду — задача выполнена.
        self.shutdown_timer = self.create_timer(3.0, self.shutdown_node)

    def shutdown_node(self) -> None:
        """Завершить ноду после приветствия."""
        self.get_logger().info("Приветствие завершено — завершаю работу ноды")
        if hasattr(self, "shutdown_timer") and self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
        if hasattr(self, "check_timer") and self.check_timer is not None:
            self.check_timer.cancel()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = StartupGreetingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
