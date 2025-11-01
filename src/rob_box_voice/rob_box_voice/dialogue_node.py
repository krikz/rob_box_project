#!/usr/bin/env python3
"""
DialogueNode - LLM диалоговая система с DeepSeek API (streaming)

Подписывается на: /voice/stt/result (распознанная речь)
Публикует: /voice/dialogue/response (JSON chunks для TTS)
Использует: DeepSeek API streaming + accent_replacer
"""

import json
import os
import re
import subprocess
import sys
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

# Импортируем из scripts
scripts_path = Path(__file__).parent.parent / "scripts"
sys.path.insert(0, str(scripts_path))

try:
    from accent_replacer import AccentReplacer
    from openai import OpenAI
except ImportError as e:
    print(f"❌ Ошибка импорта: {e}")
    print("Установите: pip install openai")
    sys.exit(1)


class DialogueNode(Node):
    """ROS2 нода для LLM диалога с DeepSeek"""

    def __init__(self):
        super().__init__("dialogue_node")

        # Параметры
        self.declare_parameter("api_key", "")
        self.declare_parameter("base_url", "https://api.deepseek.com")
        self.declare_parameter("model", "deepseek-chat")
        self.declare_parameter("temperature", 0.7)
        self.declare_parameter("max_tokens", 500)
        self.declare_parameter("system_prompt_file", "master_prompt_simple.txt")
        self.declare_parameter("wake_words", ["робок", "робот", "роббокс"])
        self.declare_parameter("silence_commands", ["помолч", "замолч", "хватит"])

        api_key = self.get_parameter("api_key").value
        if not api_key:
            api_key = os.getenv("DEEPSEEK_API_KEY")

        if not api_key:
            self.get_logger().error("❌ DEEPSEEK_API_KEY не найден!")
            raise RuntimeError("DEEPSEEK_API_KEY required")

        base_url = self.get_parameter("base_url").value
        self.model = self.get_parameter("model").value
        self.temperature = self.get_parameter("temperature").value
        self.max_tokens = self.get_parameter("max_tokens").value

        # DeepSeek client
        self.client = OpenAI(api_key=api_key, base_url=base_url)

        # Accent replacer
        self.accent_replacer = AccentReplacer()
        stats = self.accent_replacer.get_stats()
        self.get_logger().info(f'📖 Словарь ударений: {stats["total_words"]} слов')

        # System prompt
        self.system_prompt = self._load_system_prompt()

        # История диалога
        self.conversation_history = []

        # Подписка на распознанную речь
        self.stt_sub = self.create_subscription(String, "/voice/stt/result", self.stt_callback, 10)

        # Подписка на feedback от command_node (Phase 5)
        self.command_feedback_sub = self.create_subscription(
            String, "/voice/command/feedback", self.command_feedback_callback, 10
        )

        # Публикация ответов (JSON chunks)
        self.response_pub = self.create_publisher(String, "/voice/dialogue/response", 10)

        # Публикация в TTS для синтеза (Phase 6 - добавлено!)
        self.tts_pub = self.create_publisher(String, "/voice/tts/request", 10)

        # Публикация звуковых триггеров (Phase 4)
        self.sound_trigger_pub = self.create_publisher(String, "/voice/sound/trigger", 10)

        # Публикация control commands в TTS
        self.tts_control_pub = self.create_publisher(String, "/voice/tts/control", 10)

        # Публикация state для других нод (command_node)
        self.state_pub = self.create_publisher(String, "/voice/dialogue/state", 10)

        # Публикация срочных запросов к внутреннему диалогу (reflection)
        self.reflection_request_pub = self.create_publisher(String, "/perception/user_speech", 10)

        # ============ Internet Status Monitoring & Time Awareness ============
        self.internet_available = True  # Assume available by default
        self.current_time_info = None  # Store time information from perception

        # Подписка на perception context для мониторинга интернета и времени
        try:
            from rob_box_perception_msgs.msg import PerceptionEvent

            self.perception_sub = self.create_subscription(
                PerceptionEvent, "/perception/context_update", self._on_perception_update, 10
            )
            self.get_logger().info("✅ Подписан на /perception/context_update для мониторинга интернета и времени")
        except ImportError:
            self.get_logger().warning("⚠️  PerceptionEvent не найден - мониторинг интернета и времени отключен")
            self.perception_sub = None

        # ============ State Machine ============
        # IDLE -> LISTENING -> DIALOGUE -> SILENCED
        self.state = "IDLE"  # IDLE | LISTENING | DIALOGUE | SILENCED
        self.silence_until = None  # Timestamp когда закончится SILENCED
        self.last_interaction_time = time.time()
        self.dialogue_timeout = 30.0  # секунд без активности -> IDLE

        # Wake words и silence commands из параметров
        self.wake_words = self.get_parameter("wake_words").value
        self.silence_commands = self.get_parameter("silence_commands").value

        # Unsilence commands - команды для выхода из SILENCED режима
        self.unsilence_commands = [
            "говори",
            "включ",  # включись, включайся
            "работ",  # работай, работайте
            "отвеч",  # отвечай, отвечайте
            "разговар",  # разговаривай
        ]

        # Флаг что dialogue_node обработал запрос (чтобы игнорировать command feedback)
        self.dialogue_in_progress = False

        # Текущий streaming запрос (для прерывания)
        self.current_stream = None

        # Dialogue session tracking (для синхронизации с TTS)
        self.current_dialogue_id = None

        # ============ RTABMap Control (Mapping Commands) ============
        # Service clients для управления картографией
        self.reset_memory_client = self.create_client(Empty, "/rtabmap/reset_memory")
        self.set_mode_mapping_client = self.create_client(Empty, "/rtabmap/set_mode_mapping")
        self.set_mode_localization_client = self.create_client(Empty, "/rtabmap/set_mode_localization")

        # Mapping intent patterns
        self.mapping_intents = {
            "start_mapping": [
                r"исследуй территорию",
                r"начни исследование",
                r"создай новую карту",
                r"начни картографию",
                r"новая карта",
                r"начать сначала",
                r"исследовать",
            ],
            "continue_mapping": [
                r"продолжи исследование",
                r"продолжить картографию",
                r"продолжай карту",
                r"добавь к карте",
                r"продолжи создание карты",
                r"продолжить",
            ],
            "finish_mapping": [
                r"закончи исследование",
                r"завершить картографию",
                r"перейди в навигацию",
                r"режим локализации",
                r"карта готова",
                r"хватит исследовать",
                r"закончить",
            ],
        }

        # Система подтверждения (для start_mapping)
        self.pending_confirmation = None  # 'start_mapping' или None
        self.confirmation_time = None  # Timestamp запроса подтверждения
        self.confirmation_timeout = 30.0  # секунд для ответа

        # Таймер для проверки dialogue timeout
        self.timeout_timer = self.create_timer(5.0, self._check_dialogue_timeout)

        self.get_logger().info("✅ DialogueNode инициализирован")
        self.get_logger().info(f'  Wake words: {", ".join(self.wake_words)}')
        self.get_logger().info(f'  Silence commands: {", ".join(self.silence_commands)}')
        self.get_logger().info(f"  Model: {self.model}")
        self.get_logger().info(f"  Temperature: {self.temperature}")
        self.get_logger().info(f"  Max tokens: {self.max_tokens}")
        self.get_logger().info(f"  Dialogue timeout: {self.dialogue_timeout}s")

    def _load_system_prompt(self) -> str:
        """Загрузить упрощённый system prompt"""
        prompt_file = self.get_parameter("system_prompt_file").value

        # Ищем в share/rob_box_voice/prompts/
        from ament_index_python.packages import get_package_share_directory

        try:
            pkg_share = get_package_share_directory("rob_box_voice")
            prompt_path = os.path.join(pkg_share, "prompts", prompt_file)

            with open(prompt_path, "r", encoding="utf-8") as f:
                prompt = f.read()

            self.get_logger().info(f"✅ Загружен prompt: {prompt_file} ({len(prompt)} байт)")
            return prompt
        except Exception as e:
            self.get_logger().warning(f"⚠ Не удалось загрузить prompt: {e}")
            return 'Ты ROBBOX - мобильный робот-ассистент. Отвечай в JSON: {"ssml": "<speak>...</speak>"}'

    # ============================================================
    # Wake Word & Silence Detection
    # ============================================================

    def _has_wake_word(self, text: str) -> bool:
        """Проверка наличия wake word"""
        for wake_word in self.wake_words:
            if wake_word in text:
                return True
        return False

    def _remove_wake_word(self, text: str) -> str:
        """Убрать wake word из текста"""
        for wake_word in self.wake_words:
            text = text.replace(wake_word, "").strip()
        return text

    def _is_silence_command(self, text: str) -> bool:
        """Проверка: команда замолчать?"""
        # Используем silence_commands из параметров
        for command in self.silence_commands:
            if command in text:
                return True

        return False

    def _is_unsilence_command(self, text: str) -> bool:
        """Проверка: команда выхода из silence режима?"""
        for command in self.unsilence_commands:
            if command in text:
                return True

        return False

    def _handle_silence_command(self):
        """Обработка команды silence"""
        self.get_logger().warning("🔇 SILENCE: останавливаем TTS и переходим в SILENCED")

        # 1. Прервать текущий streaming
        if self.current_stream:
            try:
                # Не можем прервать генератор, но можем установить флаг
                self.current_stream = None
            except Exception as e:
                self.get_logger().error(f"Ошибка прерывания stream: {e}")

        # 2. Отправить STOP в TTS
        stop_msg = String()
        stop_msg.data = "STOP"
        self.tts_control_pub.publish(stop_msg)
        self.get_logger().info("  → STOP отправлен в TTS")

        # 3. Перейти в SILENCED на 5 минут
        self.state = "SILENCED"
        self.silence_until = time.time() + 300  # 5 минут
        self._publish_state()
        self.get_logger().info("  → State: SILENCED (5 минут)")

        # 4. Короткое подтверждение (через TTS напрямую)
        self._speak_simple("Хорошо, молчу")

    def _speak_simple(self, text: str):
        """Простая речь без LLM"""
        # Генерируем новый dialogue_id для каждого простого ответа
        dialogue_id = str(uuid.uuid4())
        self.current_dialogue_id = dialogue_id

        response_json = {"dialogue_id": dialogue_id, "ssml": f"<speak>{text}</speak>"}

        response_msg = String()
        response_msg.data = json.dumps(response_json, ensure_ascii=False)
        self.response_pub.publish(response_msg)
        # NOTE: НЕ публикуем в tts_pub - tts_node уже подписан на response_pub

    def _publish_state(self):
        """Публикация текущего состояния dialogue_node"""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _on_perception_update(self, msg):
        """Обработка обновления контекста восприятия для мониторинга интернета и времени"""
        # Update internet status
        if hasattr(msg, "internet_available"):
            was_available = self.internet_available
            self.internet_available = msg.internet_available

            # Логируем изменения статуса
            if was_available and not self.internet_available:
                self.get_logger().warning("⚠️  Интернет недоступен - переход на fallback режим")
            elif not was_available and self.internet_available:
                self.get_logger().info("✅ Интернет восстановлен - нормальный режим")

        # Update time information
        if hasattr(msg, "time_context_json") and msg.time_context_json:
            try:
                self.current_time_info = json.loads(msg.time_context_json)
                self.get_logger().debug(f'🕐 Обновлено время: {self.current_time_info.get("time_only", "N/A")}')
            except json.JSONDecodeError as e:
                self.get_logger().warning(f"⚠️  Ошибка парсинга time_context_json: {e}")
                self.get_logger().debug(f"   Raw JSON: {msg.time_context_json[:100]}...")

    def _generate_fallback_response(self, user_message: str) -> str:
        """Генерация fallback ответа когда интернет недоступен"""
        user_lower = user_message.lower()

        # Простые правила для fallback
        if any(word in user_lower for word in ["привет", "здравствуй", "хай", "hello"]):
            return "Привет! Извините, сейчас нет подключения к интернету, мои возможности ограничены."
        elif any(word in user_lower for word in ["как дела", "как ты", "что делаешь"]):
            return "Всё работает, но интернет недоступен. Мои возможности сейчас ограничены простыми ответами."
        elif any(word in user_lower for word in ["спасибо", "благодар"]):
            return "Пожалуйста!"
        elif any(word in user_lower for word in ["пока", "до свидания", "bye"]):
            return "До свидания!"
        else:
            return "Извините, интернет сейчас недоступен. Я могу только отвечать на простые приветствия."

    # ============================================================
    # Main Callback
    # ============================================================

    def stt_callback(self, msg: String):
        """Обработка распознанной речи с State Machine"""
        user_message = msg.data.strip()
        if not user_message:
            return

        user_message_lower = user_message.lower()
        self.get_logger().info(f"👤 User: {user_message} [State: {self.state}]")

        # ============ ПРИОРИТЕТ 1: Проверка SILENCE command ============
        if self._is_silence_command(user_message_lower):
            self.get_logger().warning("🔇 SILENCE COMMAND обнаружена!")
            self._handle_silence_command()
            return

        # ============ ПРИОРИТЕТ 2: Проверка SILENCED state ============
        if self.state == "SILENCED":
            # В SILENCED: проверяем unsilence команды с wake word
            if self._has_wake_word(user_message_lower):
                # Проверяем: команда выхода из silence?
                if self._is_unsilence_command(user_message_lower):
                    self.get_logger().info("🔓 Unsilence command обнаружена → IDLE")
                    self.state = "IDLE"
                    self.silence_until = None
                    self._publish_state()
                    self._speak_simple("Хорошо, слушаю!")
                    return
                else:
                    # Обычная команда с wake word в SILENCED
                    self.get_logger().info("🔓 Wake word в SILENCED → разрешаем ТОЛЬКО команды")
                    # TODO: передать в command_node для навигации/LED
                    # Пока просто логируем
                    self.get_logger().info("  → Команда должна быть обработана command_node")
                    return
            else:
                self.get_logger().debug("🔇 SILENCED: игнорируем (нет wake word)")
                return

        # ============ ПРИОРИТЕТ 3: Wake Word Detection ============
        if self.state == "IDLE":
            # В IDLE: требуется wake word
            if self._has_wake_word(user_message_lower):
                self.get_logger().info("👋 Wake word обнаружен → LISTENING")
                self.state = "LISTENING"
                self._publish_state()
                self.last_interaction_time = time.time()

                # Убираем wake word из текста
                user_message_clean = self._remove_wake_word(user_message_lower)
                if not user_message_clean:
                    # Только wake word без команды/вопроса
                    self._speak_simple("Слушаю!")
                    return

                user_message = user_message_clean
            else:
                self.get_logger().debug("⏸️  IDLE: игнорируем (нет wake word)")
                return

        # ============ State: LISTENING или DIALOGUE ============
        self.state = "DIALOGUE"
        self._publish_state()
        self.last_interaction_time = time.time()
        self.dialogue_in_progress = True

        # ============ ПРИОРИТЕТ 4: Проверка подтверждения (start_mapping) ============
        if self.pending_confirmation:
            elapsed = time.time() - self.confirmation_time if self.confirmation_time else 999

            if elapsed > self.confirmation_timeout:
                # Timeout подтверждения
                self.get_logger().warning("⏰ Confirmation timeout → отмена")
                self.pending_confirmation = None
                self.confirmation_time = None
                self._speak_simple("Время ожидания истекло. Операция отменена.")
                self.dialogue_in_progress = False
                return

            # Проверяем ответ: да/нет
            if any(
                word in user_message_lower for word in ["да", "давай", "начинай", "начни", "подтверждаю", "ок", "угу"]
            ):
                self.get_logger().info("✅ Подтверждение получено!")

                if self.pending_confirmation == "start_mapping":
                    # Выполнить start_mapping
                    import asyncio

                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    response = loop.run_until_complete(self._confirm_start_mapping())
                    loop.close()

                    self.pending_confirmation = None
                    self.confirmation_time = None
                    self._speak_simple(response)
                    self.dialogue_in_progress = False
                    return

            elif any(word in user_message_lower for word in ["нет", "отмена", "стоп", "не надо", "передумал"]):
                self.get_logger().info("❌ Подтверждение отклонено")
                self.pending_confirmation = None
                self.confirmation_time = None
                self._speak_simple("Хорошо, операция отменена.")
                self.dialogue_in_progress = False
                return
            else:
                # Неясный ответ - повторить вопрос
                self.get_logger().warning("⚠️ Неясный ответ на подтверждение")
                self._speak_simple("Пожалуйста, ответьте да или нет.")
                self.dialogue_in_progress = False
                return

        # ============ ПРИОРИТЕТ 5: Проверка Volume Control Commands ============
        volume_intent = self._detect_volume_intent(user_message_lower)
        if volume_intent:
            self.get_logger().info(f"🔊 Обнаружена volume команда: {volume_intent}")
            response = self._handle_volume_command(volume_intent)
            self._speak_simple(response)
            self.dialogue_in_progress = False
            return

        # ============ ПРИОРИТЕТ 6: Проверка Pitch Control Commands ============
        pitch_intent = self._detect_pitch_intent(user_message_lower)
        if pitch_intent:
            self.get_logger().info(f"🎵 Обнаружена pitch команда: {pitch_intent}")
            response = self._handle_pitch_command(pitch_intent)
            self._speak_simple(response)
            self.dialogue_in_progress = False
            return

        # ============ ПРИОРИТЕТ 7: Проверка Mapping Commands ============
        mapping_intent = self._detect_mapping_intent(user_message_lower)
        if mapping_intent:
            self.get_logger().info(f"🗺️ Обнаружена mapping команда: {mapping_intent}")

            # Обработать команду
            import asyncio

            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            response = loop.run_until_complete(self._handle_mapping_command(mapping_intent, user_message))
            loop.close()

            if response:
                self._speak_simple(response)
                self.dialogue_in_progress = False
                return

        # ============ ПРИОРИТЕТ 8: Проверка доступности интернета ============
        if not self.internet_available:
            self.get_logger().warning("⚠️  Интернет недоступен - используем fallback")
            fallback_response = self._generate_fallback_response(user_message)
            self._speak_simple(fallback_response)
            self.dialogue_in_progress = False
            return

        # ============ ПРИОРИТЕТ 9: Обычный диалог с LLM ============
        # Добавляем в историю
        self.conversation_history.append({"role": "user", "content": user_message})

        # Ограничиваем историю (последние 10 сообщений)
        if len(self.conversation_history) > 10:
            self.conversation_history = self.conversation_history[-10:]

        # Триггер звука "thinking" (Phase 4)
        self._trigger_sound("thinking")

        # Запрос к DeepSeek (streaming)
        self._ask_deepseek_streaming()

    # Marker for inserting time context in system prompt
    TIME_CONTEXT_MARKER = "# Формат ответа"
    TIME_CONTEXT_SECTION_TITLE = "# Текущее время"

    def _build_system_prompt_with_context(self) -> str:
        """Построить system prompt с добавлением текущего времени"""
        base_prompt = self.system_prompt

        # Добавляем информацию о текущем времени, если доступна
        if self.current_time_info:
            time_context = []
            time_context.append(f"\n{self.TIME_CONTEXT_SECTION_TITLE}\n")
            time_context.append(f"**Сейчас:** {self.current_time_info.get('time_only', 'N/A')}")
            time_context.append(f"**Дата:** {self.current_time_info.get('date_only', 'N/A')}")
            time_context.append(f"**Период суток:** {self.current_time_info.get('period_ru', 'N/A')}")
            time_context.append(f"**День недели:** {self.current_time_info.get('weekday_ru', 'N/A')}")

            time_info = "\n".join(time_context)

            # Вставляем время после характеристик робота, перед форматом ответа
            # Используем маркер для надёжного определения места вставки
            if self.TIME_CONTEXT_MARKER in base_prompt:
                prompt_parts = base_prompt.split(self.TIME_CONTEXT_MARKER, 1)
                return f"{prompt_parts[0]}{time_info}\n\n{self.TIME_CONTEXT_MARKER}{prompt_parts[1]}"
            else:
                # Если маркер не найден, добавляем в конец
                self.get_logger().warning(
                    f'⚠️  Маркер "{self.TIME_CONTEXT_MARKER}" не найден в промпте, ' "добавляем время в конец"
                )
                return f"{base_prompt}\n{time_info}"

        return base_prompt

    def _ask_deepseek_streaming(self):
        """Streaming запрос к DeepSeek с парсингом JSON chunks"""
        # Генерируем новый dialogue_id для этого диалога
        dialogue_id = str(uuid.uuid4())
        self.current_dialogue_id = dialogue_id
        self.get_logger().info(f"🆔 Новый диалог: {dialogue_id[:8]}...")

        # Используем system prompt с контекстом времени
        system_prompt_with_context = self._build_system_prompt_with_context()

        messages = [{"role": "system", "content": system_prompt_with_context}, *self.conversation_history]

        self.get_logger().info("🤔 Запрос к DeepSeek...")

        try:
            stream = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                stream=True,
            )

            # Накопление response
            full_response = ""
            current_chunk = ""
            brace_count = 0
            in_json = False
            chunk_count = 0

            # Обработка streaming chunks
            for chunk in stream:
                if chunk.choices[0].delta.content:
                    token = chunk.choices[0].delta.content
                    full_response += token
                    current_chunk += token

                    # Подсчёт скобок для определения границ JSON
                    for char in token:
                        if char == "{":
                            brace_count += 1
                            in_json = True
                        elif char == "}":
                            brace_count -= 1

                    # Если скобки сбалансированы - парсим
                    if in_json and brace_count == 0:
                        json_text = current_chunk.strip()

                        # Убираем markdown ```json если есть
                        if json_text.startswith("```json"):
                            json_text = json_text.replace("```json", "").replace("```", "").strip()

                        # Парсим JSON
                        try:
                            chunk_data = json.loads(json_text)

                            # ============ ПРОВЕРКА: ask_reflection команда ============
                            if "action" in chunk_data and chunk_data["action"] == "ask_reflection":
                                question = chunk_data.get("question", "")
                                self.get_logger().warning(f'🔁 DeepSeek перенаправляет к Reflection: "{question}"')

                                # Публикуем в /perception/user_speech для reflection_node
                                reflection_msg = String()
                                reflection_msg.data = question
                                self.reflection_request_pub.publish(reflection_msg)
                                self.get_logger().info("  → Запрос отправлен к внутреннему диалогу")

                                # Сброс для следующего chunk
                                current_chunk = ""
                                in_json = False
                                brace_count = 0
                                continue  # Не обрабатываем дальше как обычный chunk

                            # Применяем автоударения
                            if "ssml" in chunk_data:
                                ssml = chunk_data["ssml"]
                                ssml_with_accents = self.accent_replacer.add_accents(ssml)
                                chunk_data["ssml"] = ssml_with_accents

                                # Добавляем dialogue_id к chunk
                                chunk_data["dialogue_id"] = dialogue_id

                                # Публикуем chunk
                                chunk_count += 1
                                self.get_logger().info(
                                    f"📤 Chunk {chunk_count} (dialogue_id: {dialogue_id[:8]}...): {ssml[:50]}..."
                                )

                                # Обновляем время взаимодействия (робот говорит)
                                self.last_interaction_time = time.time()

                                response_msg = String()
                                response_msg.data = json.dumps(chunk_data, ensure_ascii=False)
                                self.response_pub.publish(response_msg)
                                # NOTE: НЕ публикуем в tts_pub - tts_node уже подписан на response_pub

                                self.get_logger().info(f"🔊 Отправлено в TTS: chunk {chunk_count}")

                        except json.JSONDecodeError:
                            pass  # Ждём больше данных

                        # Сброс для следующего chunk
                        current_chunk = ""
                        in_json = False
                        brace_count = 0

            # Сохраняем ответ в историю
            self.conversation_history.append({"role": "assistant", "content": full_response})

            self.get_logger().info(f"✅ DeepSeek ответил ({chunk_count} chunks)")

            # Сбрасываем флаг после успешного ответа
            self.dialogue_in_progress = False

        except Exception as e:
            self.get_logger().error(f"❌ Ошибка DeepSeek: {e}")
            # Сбрасываем флаг даже при ошибке
            self.dialogue_in_progress = False

    def _trigger_sound(self, sound_name: str):
        """Триггер звукового эффекта (Phase 4)"""
        try:
            msg = String()
            msg.data = sound_name
            self.sound_trigger_pub.publish(msg)
            self.get_logger().debug(f"🔔 Триггер звука: {sound_name}")
        except Exception as e:
            self.get_logger().warning(f"⚠️ Ошибка триггера звука: {e}")

    # ============================================================
    # Volume Control Commands
    # ============================================================

    def _detect_volume_intent(self, text: str):
        """Определить intent для команд управления громкостью
        
        Returns:
            str: 'louder', 'quieter', 'max', 'normal', None
        """
        text_lower = text.lower()
        
        # Паттерны для различных команд громкости
        volume_patterns = {
            'louder': [
                r'громче',
                r'громко',
                r'прибав\w* громкост',
                r'увелич\w* громкост',
            ],
            'quieter': [
                r'тише',
                r'потише',
                r'убав\w* громкост',
                r'уменьш\w* громкост',
            ],
            'max': [
                r'говори громко',
                r'максимальн\w* громкост',
                r'на полную громкост',
            ],
            'normal': [
                r'нормальн\w* громкост',
                r'стандартн\w* громкост',
                r'обычн\w* громкост',
            ],
        }
        
        for intent, patterns in volume_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    return intent
        
        return None
    
    def _handle_volume_command(self, intent: str) -> str:
        """Обработка команды изменения громкости
        
        Args:
            intent: 'louder', 'quieter', 'max', 'normal'
            
        Returns:
            str: Ответ для пользователя
        """
        # Получаем текущую громкость TTS ноды через ROS параметры
        try:
            from rcl_interfaces.srv import GetParameters, SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            
            # Создаем клиенты для работы с параметрами TTS и Sound нод
            tts_get_params_client = self.create_client(GetParameters, '/tts_node/get_parameters')
            tts_set_params_client = self.create_client(SetParameters, '/tts_node/set_parameters')
            sound_set_params_client = self.create_client(SetParameters, '/sound_node/set_parameters')
            
            # Ждем доступности сервисов
            if not tts_get_params_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("❌ TTS node параметры недоступны")
                return "Извините, не могу изменить громкость. Система недоступна."
            
            # Получаем текущий volume_db
            get_request = GetParameters.Request()
            get_request.names = ['volume_db']
            future = tts_get_params_client.call_async(get_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is None:
                self.get_logger().error("❌ Не удалось получить volume_db")
                return "Извините, не могу изменить громкость."
            
            current_volume_db = future.result().values[0].double_value
            self.get_logger().info(f"📊 Текущая громкость: {current_volume_db:.1f} dB")
            
            # Вычисляем новую громкость
            new_volume_db = current_volume_db
            response_text = ""
            
            if intent == 'louder':
                new_volume_db = min(current_volume_db + 3.0, 6.0)  # +3dB, макс +6dB
                response_text = "Делаю громче"
            elif intent == 'quieter':
                new_volume_db = max(current_volume_db - 3.0, -20.0)  # -3dB, мин -20dB
                response_text = "Делаю тише"
            elif intent == 'max':
                new_volume_db = 6.0  # Максимальная громкость +6dB (~2x)
                response_text = "Максимальная громкость"
            elif intent == 'normal':
                new_volume_db = -3.0  # Нормальная громкость -3dB (70%)
                response_text = "Нормальная громкость"
            
            # Устанавливаем новую громкость
            if abs(new_volume_db - current_volume_db) < 0.1:
                # Громкость уже на пределе
                if intent == 'louder':
                    return "Громкость уже максимальная"
                elif intent == 'quieter':
                    return "Громкость уже минимальная"
            
            # Устанавливаем параметры для TTS ноды
            set_request = SetParameters.Request()
            param = Parameter()
            param.name = 'volume_db'
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = new_volume_db
            set_request.parameters = [param]
            
            future = tts_set_params_client.call_async(set_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is None or not future.result().results[0].successful:
                self.get_logger().error("❌ Не удалось установить volume_db для TTS")
                return "Извините, не могу изменить громкость."
            
            # Также устанавливаем для Sound ноды (если доступна)
            if sound_set_params_client.wait_for_service(timeout_sec=0.5):
                future = sound_set_params_client.call_async(set_request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                if future.result() and future.result().results[0].successful:
                    self.get_logger().info("✅ Громкость звуков также изменена")
            
            self.get_logger().info(f"✅ Громкость изменена: {current_volume_db:.1f} → {new_volume_db:.1f} dB")
            return response_text
            
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка изменения громкости: {e}")
            return "Извините, произошла ошибка."

    # ============================================================
    # Pitch Control Commands
    # ============================================================

    def _detect_pitch_intent(self, text: str):
        """Определить intent для команд управления высотой голоса (pitch)
        
        Returns:
            str: 'higher', 'lower', 'normal', None
        """
        text_lower = text.lower()
        
        # Паттерны для различных команд pitch
        pitch_patterns = {
            'higher': [
                r'говори выше',
                r'голос выше',
                r'повыс\w* голос',
                r'выше говор',
            ],
            'lower': [
                r'говори ниже',
                r'голос ниже',
                r'пониж\w* голос',
                r'ниже говор',
            ],
            'normal': [
                r'нормальн\w* голос',
                r'обычн\w* голос',
                r'говори нормально',
            ],
        }
        
        for intent, patterns in pitch_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    return intent
        
        return None
    
    def _handle_pitch_command(self, intent: str) -> str:
        """Обработка команды изменения высоты голоса через pitch_shift
        
        Args:
            intent: 'higher', 'lower', 'normal'
            
        Returns:
            str: Ответ для пользователя
        """
        # Управляем pitch через pitch_shift параметр
        # Более высокий pitch_shift = более высокий голос (chipmunk effect)
        # Более низкий pitch_shift = более низкий голос
        try:
            from rcl_interfaces.srv import GetParameters, SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            
            # Создаем клиенты для работы с параметрами TTS ноды
            tts_get_params_client = self.create_client(GetParameters, '/tts_node/get_parameters')
            tts_set_params_client = self.create_client(SetParameters, '/tts_node/set_parameters')
            
            # Ждем доступности сервисов
            if not tts_get_params_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("❌ TTS node параметры недоступны")
                return "Извините, не могу изменить высоту голоса. Система недоступна."
            
            # Получаем текущий pitch_shift
            get_request = GetParameters.Request()
            get_request.names = ['pitch_shift']
            future = tts_get_params_client.call_async(get_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is None:
                self.get_logger().error("❌ Не удалось получить pitch_shift")
                return "Извините, не могу изменить высоту голоса."
            
            # pitch_shift обычно double
            param_value = future.result().values[0]
            if param_value.type == ParameterType.PARAMETER_INTEGER:
                current_pitch = float(param_value.integer_value)
            elif param_value.type == ParameterType.PARAMETER_DOUBLE:
                current_pitch = param_value.double_value
            else:
                self.get_logger().error(f"❌ Неожиданный тип параметра pitch_shift: {param_value.type}")
                return "Извините, не могу изменить высоту голоса."
            
            self.get_logger().info(f"📊 Текущий pitch (pitch_shift): {current_pitch:.2f}")
            
            # Вычисляем новый pitch
            # Диапазон: 1.0 (нормальный) - 3.0 (очень высокий)
            # Для chipmunk эффекта: 2.0 = оригинальный ROBBOX
            new_pitch = current_pitch
            response_text = ""
            
            if intent == 'higher':
                new_pitch = min(current_pitch + 0.2, 3.0)  # +0.2, макс 3.0
                response_text = "Говорю выше"
            elif intent == 'lower':
                new_pitch = max(current_pitch - 0.2, 1.0)  # -0.2, мин 1.0 (без эффекта)
                response_text = "Говорю ниже"
            elif intent == 'normal':
                new_pitch = 2.0  # Нормальный ROBBOX голос (chipmunk)
                response_text = "Нормальный голос"
            
            # Проверяем изменение
            if abs(new_pitch - current_pitch) < 0.01:
                # Pitch уже на пределе
                if intent == 'higher':
                    return "Голос уже максимально высокий"
                elif intent == 'lower':
                    return "Голос уже минимально низкий"
            
            # Устанавливаем параметры для TTS ноды
            set_request = SetParameters.Request()
            param = Parameter()
            param.name = 'pitch_shift'
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = new_pitch
            set_request.parameters = [param]
            
            future = tts_set_params_client.call_async(set_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is None or not future.result().results[0].successful:
                self.get_logger().error("❌ Не удалось установить pitch_shift для TTS")
                return "Извините, не могу изменить высоту голоса."
            
            self.get_logger().info(f"✅ Pitch изменен: {current_pitch} → {new_pitch}")
            return response_text
            
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка изменения pitch: {e}")
            return "Извините, произошла ошибка."

    # ============================================================
    # Mapping Commands (RTABMap Control)
    # ============================================================

    def _detect_mapping_intent(self, text: str):
        """Определить intent для команд картографии"""
        text_lower = text.lower()

        for intent, patterns in self.mapping_intents.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    return intent

        return None

    async def _backup_rtabmap_db(self) -> bool:
        """Создать backup текущей БД RTABMap через Docker"""
        try:
            # Docker exec на Main Pi для backup
            # Предполагается что контейнер rtabmap доступен
            result = subprocess.run(
                [
                    "docker",
                    "exec",
                    "rtabmap",
                    "bash",
                    "-c",
                    "mkdir -p /maps/backups && "
                    "cp /maps/rtabmap.db /maps/backups/rtabmap_backup_$(date +%Y%m%d_%H%M%S).db && "
                    "echo OK",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )

            if result.returncode == 0 and "OK" in result.stdout:
                self.get_logger().info("✅ RTABMap backup создан")
                return True
            else:
                self.get_logger().error(f"❌ Backup failed: {result.stderr}")
                return False
        except subprocess.TimeoutExpired:
            self.get_logger().error("❌ Backup timeout (10s)")
            return False
        except Exception as e:
            self.get_logger().error(f"❌ Backup error: {e}")
            return False

    async def _handle_mapping_command(self, intent: str, text: str) -> str:
        """Обработка команд картографии"""
        self.get_logger().info(f"🗺️ Mapping intent: {intent}")

        if intent == "start_mapping":
            # Запрос подтверждения
            self.pending_confirmation = "start_mapping"
            self.confirmation_time = time.time()
            self._trigger_sound("confused")  # Звук вопроса
            return "Начать новое исследование? Старая карта будет сохранена в резервную копию."

        elif intent == "continue_mapping":
            # Переключить в mapping mode
            try:
                self.get_logger().info("  → Переключение в SLAM mode...")
                _future = self.set_mode_mapping_client.call_async(Empty.Request())  # noqa: F841
                # Не ждём ответа (async), просто отправляем
                self._trigger_sound("cute")  # Звук подтверждения
                return "Продолжаю исследование территории. Добавляю новые области к карте."
            except Exception as e:
                self.get_logger().error(f"❌ Ошибка set_mode_mapping: {e}")
                self._trigger_sound("confused")
                return "Извините, не удалось переключить режим картографии."

        elif intent == "finish_mapping":
            # Переключить в localization mode
            try:
                self.get_logger().info("  → Переключение в Localization mode...")
                _future = self.set_mode_localization_client.call_async(Empty.Request())  # noqa: F841
                # Не ждём ответа (async), просто отправляем
                self._trigger_sound("cute")  # Звук подтверждения
                return "Заканчиваю исследование. Переключаюсь в режим навигации по готовой карте."
            except Exception as e:
                self.get_logger().error(f"❌ Ошибка set_mode_localization: {e}")
                self._trigger_sound("confused")
                return "Извините, не удалось переключить в режим локализации."

        return None

    async def _confirm_start_mapping(self) -> str:
        """Подтверждение start_mapping - создать backup и reset БД"""
        self.get_logger().warning("🗺️ Подтверждение start_mapping...")

        # 1. Backup
        backup_ok = await self._backup_rtabmap_db()
        if not backup_ok:
            self._trigger_sound("angry_2")
            return "Не удалось создать резервную копию карты. Операция отменена."

        # 2. Reset memory
        try:
            self.get_logger().info("  → Reset RTABMap memory...")
            _future = self.reset_memory_client.call_async(Empty.Request())  # noqa: F841
            # Не ждём ответа (async)
            self._trigger_sound("cute")  # Звук успеха
            return "Начинаю исследование. Старая карта сохранена в резервной копии."
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка reset_memory: {e}")
            self._trigger_sound("angry_2")
            return "Не удалось сбросить память RTABMap. Попробуйте позже."

    def command_feedback_callback(self, msg: String):
        """Обработка feedback от command_node (Phase 5)"""
        feedback = msg.data.strip()
        if not feedback:
            return

        # Игнорируем feedback если dialogue уже обработал запрос
        if self.dialogue_in_progress:
            self.get_logger().debug(f"🔇 Игнор command feedback (dialogue in progress): {feedback}")
            return

        self.get_logger().info(f"📢 Command feedback: {feedback}")

        # Генерируем новый dialogue_id для feedback
        dialogue_id = str(uuid.uuid4())
        self.current_dialogue_id = dialogue_id

        # Отправить feedback в TTS (напрямую в response)
        response_json = {"dialogue_id": dialogue_id, "ssml": f"<speak>{feedback}</speak>"}

        response_msg = String()
        response_msg.data = json.dumps(response_json, ensure_ascii=False)
        self.response_pub.publish(response_msg)

    def _check_dialogue_timeout(self):
        """Проверить тайм-аут диалога и вернуться в IDLE если нет активности"""
        if self.state not in ["LISTENING", "DIALOGUE"]:
            return

        elapsed = time.time() - self.last_interaction_time
        if elapsed > self.dialogue_timeout:
            self.get_logger().info(f"⏰ Dialogue timeout ({elapsed:.1f}s) → IDLE")
            self.state = "IDLE"
            self._publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = DialogueNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
