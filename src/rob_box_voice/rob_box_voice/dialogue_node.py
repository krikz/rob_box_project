#!/usr/bin/env python3
"""
DialogueNode - LLM диалоговая система с MCP Tools интеграцией

Подписывается на: /voice/stt/result (распознанная речь)
Публикует: /voice/dialogue/response (JSON chunks для TTS)
Использует: LLM API (DeepSeek/Qwen) streaming + MCP Tools + accent_replacer

Features:
- Поддержка tool_calls для управления функциями робота
- Автоматический fallback при недоступности интернета или MCP сервера
- Поддержка нескольких LLM провайдеров (Qwen, DeepSeek)
"""

import json
import os
import re
import subprocess
import sys
import time
import uuid
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError
from pathlib import Path
from typing import Dict, Any, List, Optional

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

# Импорт MCP Tools (опционально - fallback если не установлено)
try:
    from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter
    MCP_TOOLS_AVAILABLE = True
except ImportError:
    MCP_TOOLS_AVAILABLE = False
    print("⚠️  rob_box_mcp_tools не найден - работа без tool_calls")

# Импорт DialogueManager и ConversationHistory
from rob_box_voice.core.dialogue_manager import DialogueManager, DialogueState
from rob_box_voice.core.conversation_history import ConversationHistory

# Долгосрочная память (VoiceMemory) — опциональная
try:
    from rob_box_voice.core.voice_memory import VoiceMemory
    VOICE_MEMORY_AVAILABLE = True
except ImportError:
    VOICE_MEMORY_AVAILABLE = False
    print("⚠️  VoiceMemory не найдена — долгосрочная память отключена")


class DialogueNode(Node):
    """ROS2 нода для LLM диалога с поддержкой Qwen и DeepSeek"""

    # Конфигурации провайдеров
    PROVIDERS = {
        "qwen": {
            "base_url": "https://dashscope-intl.aliyuncs.com/compatible-mode/v1",
            "model": "qwen-max",
            "env_var": "QWEN_API_KEY",  # Основная переменная для Qwen
            "fallback_env": "LLM_API_KEY",  # Fallback на унифицированную
            "name": "Qwen",
        },
        "deepseek": {
            "base_url": "https://api.deepseek.com",
            "model": "deepseek-chat",
            "env_var": "DEEPSEEK_API_KEY",  # Основная переменная для DeepSeek
            "fallback_env": "LLM_API_KEY",  # Fallback на унифицированную
            "name": "DeepSeek",
        }
    }

    # Лимит итераций агентного цикла (защита от бесконечной рекурсии)
    MAX_ITERATIONS = 30

    def __init__(self):
        super().__init__("dialogue_node")

        # Параметры
        self.declare_parameter("provider", "deepseek")  # qwen | deepseek
        self.declare_parameter("enable_fallback", False)  # Автоматический fallback на другой провайдер
        self.declare_parameter("api_key", "")
        self.declare_parameter("base_url", "")  # Если пусто - берём из PROVIDERS
        self.declare_parameter("model", "")      # Если пусто - берём из PROVIDERS
        self.declare_parameter("temperature", 0.7)
        self.declare_parameter("max_tokens", 500)
        self.declare_parameter("system_prompt_file", "master_prompt_compact.txt")  # Агентный промпт с tool calls
        self.declare_parameter("streaming", True)  # Включаем streaming
        self.declare_parameter("wake_words", ["робок", "робот", "роббокс"])
        self.declare_parameter("silence_commands", ["помолч", "замолч", "хватит"])
        self.declare_parameter("unsilence_commands", ["говори", "включ", "работ", "отвеч", "разговар"])
        self.declare_parameter("dialogue_timeout", 30.0)  # секунд без активности -> IDLE
        self.declare_parameter("query_accumulation_timeout", 2.5)  # секунд для накопления запросов

        # Выбор провайдера с fallback
        self.primary_provider = self.get_parameter("provider").value
        self.enable_fallback = self.get_parameter("enable_fallback").value
        self.current_provider = self.primary_provider
        self.provider_error_count = 0  # Счётчик ошибок текущего провайдера
        self.provider_error_threshold = 3  # Порог для переключения на fallback
        
        # Инициализация клиента с выбранным провайдером
        self._init_llm_client()
        
        # ThreadPoolExecutor для timeout на blocking операциях (API create())
        self._llm_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="llm-api")

        # Accent replacer
        self.accent_replacer = AccentReplacer()
        stats = self.accent_replacer.get_stats()
        self.get_logger().info(f'📖 Словарь ударений: {stats["total_words"]} слов')

        # System prompt
        self.system_prompt = self._load_system_prompt()

        # История диалога (используем ConversationHistory модуль)
        self.conversation_history = ConversationHistory(max_messages=20)

        # Долгосрочная память (сохраняется между рестартами)
        self.voice_memory = None
        if VOICE_MEMORY_AVAILABLE:
            self._init_voice_memory()

        # Подписка на распознанную речь
        self.stt_sub = self.create_subscription(String, "/voice/stt/result", self.stt_callback, 10)
        
        # Подписка на hardware VAD для мгновенного прерывания при новой речи
        from std_msgs.msg import Bool
        self.vad_sub = self.create_subscription(Bool, "/audio/vad", self.vad_callback, 10)
        self.vad_speech_detected = False  # Текущее состояние VAD

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

        # Публикация запросов анимаций (emotion-based)
        self.animation_pub = self.create_publisher(String, "/voice/animation/request", 10)

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

        # ============ MCP Tools Integration ============
        self.declare_parameter("enable_mcp_tools", True)  # Включить MCP tools
        self.enable_mcp_tools = self.get_parameter("enable_mcp_tools").value
        self.mcp_adapter = None
        self.available_tools = []  # Список доступных инструментов
        self.mcp_tools_available = False  # Флаг доступности MCP сервера
        
        if self.enable_mcp_tools and MCP_TOOLS_AVAILABLE:
            try:
                # Инициализация адаптера MCP tools
                self.mcp_adapter = LLMToolCallAdapter(self)
                
                # Подписка на список инструментов из MCP сервера
                self.tools_sub = self.create_subscription(
                    String, "/mcp/tools", self._on_mcp_tools_update, 10
                )
                
                self.get_logger().info("✅ MCP Tools интеграция активирована")
                self.get_logger().info("   Ожидание списка инструментов из MCP Server...")
            except Exception as e:
                self.get_logger().error(f"❌ Ошибка инициализации MCP Tools: {e}")
                self.enable_mcp_tools = False
        elif not MCP_TOOLS_AVAILABLE:
            self.get_logger().warning("⚠️  rob_box_mcp_tools не установлен - работа без tool_calls")
        else:
            self.get_logger().info("ℹ️  MCP Tools отключены в параметрах")

        # ============ State Machine - DialogueManager ============
        self.dialogue_manager = DialogueManager(
            wake_words=self.get_parameter("wake_words").value,
            silence_commands=self.get_parameter("silence_commands").value,
            unsilence_commands=self.get_parameter("unsilence_commands").value,
            dialogue_timeout=self.get_parameter("dialogue_timeout").value,
            query_accumulation_timeout=self.get_parameter("query_accumulation_timeout").value
        )

        # Флаг что dialogue_node обработал запрос (чтобы игнорировать command feedback)
        self.dialogue_in_progress = False

        # Текущий streaming запрос (для прерывания)
        self.current_stream = None

        # Dialogue session tracking (для синхронизации с TTS)
        self.current_dialogue_id = None

        # ============ Query Queue System ============
        self.accumulation_timer = None  # Таймер для проверки накопления
        self._timeout_retry_timer = None  # Таймер для тихого retry после timeout LLM
        self.llm_processing = False  # Флаг что LLM сейчас обрабатывает запрос
        self.interrupt_agent_loop = False  # Флаг прерывания агентного цикла при новом запросе
        self._listen_response_waiting = False  # Флаг ожидания listen_for_response — не сбрасывать dialogue_in_progress
        self.error_retry_delay = 1.0  # секунд задержки перед повтором при ошибке LLM

        # Счётчик поколений стримов — инкрементируется при каждом новом LLM-запросе/retry.
        # Стейл-потоки (ThreadPoolExecutor shutdown(wait=False)) захватывают поколение
        # при старте и проверяют его перед response_pub.publish(). Если не совпадает —
        # поток знает что он "прошлого поколения" и молча отбрасывает данные.
        self._stream_generation = 0

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
        self.get_logger().info(f"  🤖 Provider: {self.PROVIDERS[self.current_provider]['name']} (fallback: {'ON' if self.enable_fallback else 'OFF'})")
        self.get_logger().info(f'  Wake words: {", ".join(self.dialogue_manager.wake_words)}')
        self.get_logger().info(f'  Silence commands: {", ".join(self.dialogue_manager.silence_commands)}')
        self.get_logger().info(f"  Temperature: {self.temperature}")
        self.get_logger().info(f"  Max tokens: {self.max_tokens}")
        self.get_logger().info(f"  Dialogue timeout: {self.dialogue_manager.dialogue_timeout}s")
        self.get_logger().info(f"  Query accumulation timeout: {self.dialogue_manager.query_accumulation_timeout}s")

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

    def _init_voice_memory(self) -> None:
        """
        Инициализация VoiceMemory и восстановление контекста из предыдущих сессий.

        Не падает при ошибках — долгосрочная память является опциональной функцией.
        """
        import os

        db_path = os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        ollama_url = os.getenv("OLLAMA_BASE_URL", "http://localhost:11434")

        try:
            self.voice_memory = VoiceMemory(db_path=db_path, ollama_base_url=ollama_url)
            stats = self.voice_memory.get_stats()
            self.get_logger().info(
                f"🧠 VoiceMemory инициализирована: {db_path} "
                f"(turns={stats['turn_count']}, sessions={stats['session_count']}, "
                f"facts={stats['fact_count']}, vec={stats['vec_enabled']})"
            )

            # NOTE: Прошлые сессии НЕ загружаем в conversation_history.
            # Это приводит к "каше" — LLM видит старые диалоги как текущий контекст
            # и продолжает их вместо ответа на новый запрос.
            # Для доступа к прошлым сессиям LLM использует инструмент memory_context.

        except Exception as exc:
            self.get_logger().warning(f"⚠️ Ошибка инициализации VoiceMemory: {exc}")
            self.voice_memory = None

    def _map_emotion_to_animation(self, emotion: str) -> str:
        """Маппинг эмоций от DeepSeek в имена анимаций"""
        emotion_map = {
            "happy": "happy",
            "sad": "sad",
            "angry": "angry",
            "surprised": "surprised",
            "neutral": "idle",
            "thinking": "thinking",
            "excited": "victory",
            "confused": "thinking",
            "worried": "sad",
            "calm": "idle"
        }
        return emotion_map.get(emotion.lower(), "idle")

    def _init_llm_client(self):
        """Инициализация LLM клиента с выбранным провайдером"""
        provider_name = self.current_provider
        
        if provider_name not in self.PROVIDERS:
            self.get_logger().error(f"❌ Неизвестный провайдер: {provider_name}")
            raise RuntimeError(f"Unknown provider: {provider_name}")
        
        provider_config = self.PROVIDERS[provider_name]
        
        # API Key - проверяем параметр, потом env переменные
        api_key = self.get_parameter("api_key").value
        if not api_key:
            # Пробуем унифицированную переменную
            api_key = os.getenv(provider_config["env_var"])
        if not api_key:
            # Пробуем специфичную для провайдера
            api_key = os.getenv(provider_config["fallback_env"])
        
        if not api_key:
            self.get_logger().error(
                f"❌ API ключ не найден для {provider_config['name']}! "
                f"Установите {provider_config['env_var']} или {provider_config['fallback_env']}"
            )
            raise RuntimeError(f"API key required for {provider_name}")
        
        # Base URL - из параметра или конфига провайдера
        base_url = self.get_parameter("base_url").value
        if not base_url:
            base_url = provider_config["base_url"]
        
        # Model - из параметра или конфига провайдера
        model = self.get_parameter("model").value
        if not model:
            model = provider_config["model"]
        
        self.model = model
        self.temperature = self.get_parameter("temperature").value
        self.max_tokens = self.get_parameter("max_tokens").value
        self.streaming = self.get_parameter("streaming").value

        # Сохраняем для последующего пересоздания клиента
        self._llm_api_key = api_key
        self._llm_base_url = base_url

        self._recreate_llm_client(log_init=True)

        self.get_logger().info(f"  📡 Base URL: {base_url}")
        self.get_logger().info(f"  🤖 Model: {self.model}")
        self.get_logger().info(f"  🌊 Streaming: {self.streaming}")
    
    def _recreate_llm_client(self, log_init: bool = False):
        """Пересоздаёт httpx.Client + OpenAI wrapper на свежих сокетах.

        Вызывать после каждого сетевого таймаута — старый httpx.Client может
        содержать сокеты в CLOSE_WAIT / broken-pipe состоянии.
        """
        from httpx import Timeout, Limits
        import httpx

        # Закрываем старый клиент если есть (освобождает сокеты)
        try:
            if hasattr(self, "client") and self.client is not None:
                self.client.close()
        except Exception:
            pass

        http_client = httpx.Client(
            timeout=Timeout(60.0, connect=10.0),
            limits=Limits(max_connections=5, max_keepalive_connections=0),
            follow_redirects=True,
        )
        self.client = OpenAI(
            api_key=self._llm_api_key,
            base_url=self._llm_base_url,
            http_client=http_client,
        )
        if log_init:
            provider_name = self.PROVIDERS[self.current_provider]["name"]
            self.get_logger().info(f"✅ LLM клиент инициализирован: {provider_name} (no connection pooling)")
        else:
            self.get_logger().info("🔄 httpx.Client пересоздан на свежих сокетах")

    def _try_fallback_provider(self):
        """Попытка переключиться на резервный провайдер"""
        if not self.enable_fallback:
            self.get_logger().warning("⚠️ Fallback отключён в настройках")
            return False
        
        # Определяем fallback провайдера
        fallback_provider = "deepseek" if self.current_provider == "qwen" else "qwen"
        
        if fallback_provider not in self.PROVIDERS:
            self.get_logger().error(f"❌ Fallback провайдер недоступен: {fallback_provider}")
            return False
        
        self.get_logger().warning(f"🔄 Переключение на резервный провайдер: {self.PROVIDERS[fallback_provider]['name']}")
        
        try:
            # Сохраняем текущий провайдер
            old_provider = self.current_provider
            self.current_provider = fallback_provider
            
            # Пробуем инициализировать нового провайдера
            self._init_llm_client()
            
            self.get_logger().info(f"✅ Успешно переключились с {self.PROVIDERS[old_provider]['name']} на {self.PROVIDERS[fallback_provider]['name']}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Не удалось переключиться на fallback: {e}")
            # Восстанавливаем старый провайдер
            self.current_provider = old_provider
            return False

    # ============================================================
    # Silence Mode Handling
    # ============================================================

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

        # 2. Очистить очередь запросов
        if self.dialogue_manager.pending_queries:
            cleared_count = len(self.dialogue_manager.pending_queries)
            self.dialogue_manager.pending_queries.clear()
            self.get_logger().info(f"  → Очищено {cleared_count} запросов из очереди")

        # 3. Остановить таймер накопления
        if self.accumulation_timer is not None:
            self.accumulation_timer.cancel()
            self.accumulation_timer = None
            self.get_logger().info("  → Таймер накопления остановлен")

        # 4. Сбросить флаги обработки
        self.llm_processing = False

        # 5. Отправить STOP в TTS
        stop_msg = String()
        stop_msg.data = "STOP"
        self.tts_control_pub.publish(stop_msg)
        self.get_logger().info("  → STOP отправлен в TTS")

        # 6. Перейти в SILENCED на 5 минут
        self.dialogue_manager.enable_silence(duration=300.0)
        self._publish_state()
        self.get_logger().info("  → State: SILENCED (5 минут)")

        # 7. Короткое подтверждение (через TTS напрямую)
        self._speak_simple("Хорошо, молчу")

    def _speak_simple(self, text: str, show_error_animation: bool = False):
        """Простая речь без LLM
        
        Args:
            text: Текст для произнесения
            show_error_animation: Если True, показывает анимацию ошибки
        """
        # Генерируем новый dialogue_id для каждого простого ответа
        dialogue_id = str(uuid.uuid4())
        self.current_dialogue_id = dialogue_id
        
        # Показываем анимацию ошибки если нужно
        if show_error_animation:
            try:
                anim_msg = String()
                anim_msg.data = "error:5"  # Показываем ошибку на 5 секунд
                self.animation_pub.publish(anim_msg)
                self.get_logger().info("🎨 Показываю анимацию ошибки")
            except Exception as e:
                self.get_logger().warning(f"⚠️ Не удалось показать анимацию ошибки: {e}")

        response_json = {"dialogue_id": dialogue_id, "ssml": f"<speak>{text}</speak>"}

        response_msg = String()
        response_msg.data = json.dumps(response_json, ensure_ascii=False)
        self.response_pub.publish(response_msg)
        # NOTE: НЕ публикуем в tts_pub - tts_node уже подписан на response_pub

    def _publish_state(self):
        """Публикация текущего состояния dialogue_node"""
        msg = String()
        msg.data = self.dialogue_manager.state.value
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

    def _on_mcp_tools_update(self, msg: String):
        """Обработка обновления списка инструментов из MCP сервера"""
        try:
            tools = json.loads(msg.data)
            tool_names = [tool.get("function", {}).get("name", "unknown") for tool in tools]
            prev_names = [t.get("function", {}).get("name", "unknown") for t in self.available_tools]
            changed = tool_names != prev_names
            self.available_tools = tools
            self.mcp_tools_available = True
            if changed:
                self.get_logger().info(f"🛠️  Получено {len(tools)} инструментов из MCP сервера: {', '.join(tool_names)}")
            else:
                self.get_logger().debug(f"🛠️  MCP tools обновлены ({len(tools)} шт.) — список не изменился")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Ошибка парсинга списка инструментов: {e}")
            self.mcp_tools_available = False

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
    # Main Callbacks
    # ============================================================
    
    def vad_callback(self, msg):
        """
        Callback для hardware VAD от ReSpeaker
        
        Отслеживает начало речи пользователя для мгновенного прерывания
        LLM обработки при новом запросе (barge-in).
        """
        from std_msgs.msg import Bool
        
        vad_active = msg.data
        
        # Rising edge detection: speech start
        if vad_active and not self.vad_speech_detected:
            self.vad_speech_detected = True
            self.get_logger().debug("🎤 VAD: Speech START detected")
            
            # Прерывание при детекции новой речи во время LLM обработки
            if self.llm_processing:
                self.get_logger().warning(
                    "🛑 VAD: Новая речь обнаружена во время LLM обработки → прерывание"
                )
                self.interrupt_agent_loop = True
                
                # Прерываем MCP tool calls если есть
                if self.mcp_tools_available and hasattr(self, 'mcp_adapter'):
                    # Используем async wrapper для вызова из sync context
                    import asyncio
                    try:
                        loop = asyncio.get_event_loop()
                        if loop.is_running():
                            # Если event loop уже запущен, создаём task
                            asyncio.create_task(self.mcp_adapter.new_user_request())
                        else:
                            # Иначе запускаем синхронно
                            loop.run_until_complete(self.mcp_adapter.new_user_request())
                    except RuntimeError:
                        # Fallback если нет event loop
                        self.get_logger().warning("⚠️ Cannot interrupt MCP tasks: no event loop")
        
        # Falling edge: speech end
        elif not vad_active and self.vad_speech_detected:
            self.vad_speech_detected = False
            self.get_logger().debug("🎤 VAD: Speech END detected")

    def stt_callback(self, msg: String):
        """Обработка распознанной речи с State Machine"""
        user_message = msg.data.strip()
        if not user_message:
            return

        user_message_lower = user_message.lower()
        self.get_logger().info(f"👤 User: {user_message} [State: {self.dialogue_manager.state.value}]")

        # ============ ПРИОРИТЕТ 1: Проверка SILENCE command ============
        if self.dialogue_manager.is_silence_command(user_message_lower):
            self.get_logger().warning("🔇 SILENCE COMMAND обнаружена!")
            self._handle_silence_command()
            return

        # ============ ПРИОРИТЕТ 2: Проверка SILENCED state ============
        if self.dialogue_manager.is_silenced():
            # В SILENCED: проверяем unsilence команды с wake word
            if self.dialogue_manager.has_wake_word(user_message_lower):
                # Проверяем: команда выхода из silence?
                if self.dialogue_manager.is_unsilence_command(user_message_lower):
                    self.get_logger().info("🔓 Unsilence command обнаружена → IDLE")
                    self.dialogue_manager.disable_silence()
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
        if self.dialogue_manager.state == DialogueState.IDLE:
            # В IDLE: требуется wake word
            if self.dialogue_manager.has_wake_word(user_message_lower):
                self.get_logger().info("👋 Wake word обнаружен → LISTENING")
                self.dialogue_manager.transition_state(DialogueState.LISTENING)
                self._publish_state()

                # Очищаем накопленные мусорные запросы (фоновая речь, обрывки)
                if self.dialogue_manager.pending_queries:
                    cleared_count = len(self.dialogue_manager.pending_queries)
                    self.dialogue_manager.pending_queries.clear()
                    self.get_logger().info(f"🗑️  Очищено {cleared_count} мусорных запросов перед обработкой")

                # Новая тема — сбрасываем историю диалога.
                # Прошлые сессии доступны через memory_context, не должны грузиться сюда.
                old_len = len(self.conversation_history.get_messages())
                self.conversation_history.clear()
                if old_len > 0:
                    self.get_logger().info(f"🧹 conversation_history очищена при новом wake word ({old_len} сообщений удалено)")

                # Сбрасываем счётчик ошибок — новый диалог не должен платить за прошлые таймауты.
                if self.provider_error_count > 0:
                    self.get_logger().info(f"♻️ provider_error_count сброшен при wake word ({self.provider_error_count} → 0)")
                    self.provider_error_count = 0

                # Отменяем pending retry таймер — не нужен при новом wake word
                if self._timeout_retry_timer is not None:
                    self._timeout_retry_timer.cancel()
                    self._timeout_retry_timer = None

                # Убираем wake word из текста
                user_message_clean = self.dialogue_manager.remove_wake_word(user_message_lower)
                if not user_message_clean:
                    # Только wake word без команды/вопроса
                    self._speak_simple("Слушаю!")
                    return

                user_message = user_message_clean
            else:
                self.get_logger().debug("⏸️  IDLE: игнорируем (нет wake word)")
                return

        # ============ State: LISTENING или DIALOGUE ============
        self.dialogue_manager.transition_state(DialogueState.DIALOGUE)
        self._publish_state()
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

        # ============ ПРИОРИТЕТ 7: Проверка Speed Control Commands ============
        speed_intent = self._detect_speed_intent(user_message_lower)
        if speed_intent:
            self.get_logger().info(f"⚡ Обнаружена speed команда: {speed_intent}")
            response = self._handle_speed_command(speed_intent)
            self._speak_simple(response)
            self.dialogue_in_progress = False
            return

        # ============ ПРИОРИТЕТ 8: Проверка Mapping Commands ============
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
        # Фильтруем короткие бесполезные фразы (меньше 3 слов)
        word_count = len(user_message.split())
        if word_count < 3:
            self.get_logger().debug(f"⏸️  Игнорируем короткую фразу: '{user_message}' ({word_count} слов)")
            self.dialogue_in_progress = False
            return

        # Добавляем запрос в очередь для накопления
        self.dialogue_manager.add_query(user_message)

        self.get_logger().info(f"📥 Запрос добавлен в очередь (всего: {len(self.dialogue_manager.pending_queries)})")

        # Если LLM уже обрабатывает запрос - прерываем агентный цикл
        if self.llm_processing:
            self.get_logger().warning("⚠️ Новый запрос пользователя - прерываю текущий агентный цикл!")
            self.interrupt_agent_loop = True
            return

        # Если это первый запрос или прошло достаточно времени - начинаем накопление
        if self.accumulation_timer is None:
            # Создаём таймер для проверки когда можно обработать накопленные запросы
            self.accumulation_timer = self.create_timer(
                self.dialogue_manager.query_accumulation_timeout, 
                self._check_and_process_queue
            )
            self.get_logger().info(f"⏰ Запущен таймер накопления ({self.dialogue_manager.query_accumulation_timeout}s)")

        # Триггер звука "thinking" только при первом запросе
        if len(self.dialogue_manager.pending_queries) == 1:
            self._trigger_sound("thinking")

    def _check_and_process_queue(self):
        """Проверить очередь и обработать накопленные запросы"""
        # Останавливаем таймер
        if self.accumulation_timer is not None:
            self.accumulation_timer.cancel()
            self.accumulation_timer = None

        # Проверяем готовность через DialogueManager
        if not self.dialogue_manager.should_process_queries():
            # Ещё рано, перезапускаем таймер
            if self.dialogue_manager.last_query_time:
                elapsed = time.time() - self.dialogue_manager.last_query_time
                remaining = self.dialogue_manager.query_accumulation_timeout - elapsed
                if remaining > 0:
                    self.accumulation_timer = self.create_timer(remaining, self._check_and_process_queue)
                    self.get_logger().debug(f"⏰ Ещё рано, жду {remaining:.1f}s")
            return

        # Забираем все накопленные запросы
        queries_to_process = self.dialogue_manager.get_accumulated_queries()
        query_count = len(queries_to_process)
        self.get_logger().info(f"🔄 Обрабатываю {query_count} накопленных запросов")

        # Объединяем запросы в один контекст
        if query_count == 1:
            # Один запрос - обрабатываем как обычно
            combined_message = queries_to_process[0]
            self.get_logger().info(f"💬 Один запрос: {combined_message}")
        else:
            # Несколько запросов - формируем нумерованный список для понятности LLM
            questions_list = "\n".join([f"{i+1}. {q}" for i, q in enumerate(queries_to_process)])
            combined_message = f"Ответь на следующие вопросы:\n{questions_list}"
            self.get_logger().info(f"💬 Пакетный запрос ({query_count} вопросов):\n{combined_message}")

        # Добавляем в историю диалога
        self.conversation_history.add_user_message(combined_message)

        # Сохраняем реплику в долгосрочную память
        if self.voice_memory is not None:
            self.voice_memory.save_turn("user", combined_message)

        # Устанавливаем флаг обработки
        self.llm_processing = True

        # Запрос к LLM (streaming или обычный)
        if self.streaming:
            self._ask_llm_streaming()
        else:
            self._ask_llm_non_streaming()

    def _do_timeout_retry(self):
        """One-shot таймер: тихий retry после timeout LLM (count < threshold)"""
        if self._timeout_retry_timer is not None:
            self._timeout_retry_timer.cancel()
            self._timeout_retry_timer = None
        if not self.llm_processing and self.dialogue_in_progress:
            self.get_logger().info("♻️ Выполняю тихий retry запроса к LLM...")
            self.llm_processing = True
            self._ask_llm_streaming()
        else:
            self.get_logger().debug(
                f"⏭️ Retry пропущен (llm_processing={self.llm_processing}, dialogue_in_progress={self.dialogue_in_progress})"
            )

    def _ask_llm_streaming(self):
        """Streaming запрос к LLM провайдеру с парсингом JSON chunks и timeout"""
        # Генерируем новый dialogue_id для этого диалога
        dialogue_id = str(uuid.uuid4())
        self.current_dialogue_id = dialogue_id
        self.get_logger().info(f"🆔 Новый диалог: {dialogue_id[:8]}...")

        # Очищаем tool messages из истории при новом диалоге
        # API требует: tool messages должны быть ответом на tool_calls из предыдущего assistant message
        # При новом диалоге старые tool results недействительны
        self.conversation_history.remove_tool_messages()
        self.get_logger().debug(f"🧹 История очищена от tool messages, осталось: {len(self.conversation_history.get_messages())} сообщений")

        # System prompt статичен — время LLM получает через get_current_time тул
        messages = [{"role": "system", "content": self.system_prompt}] + self.conversation_history.get_messages()

        provider_name = self.PROVIDERS[self.current_provider]["name"]
        self.get_logger().info(f"🤔 Запрос к {provider_name}...")

        # Timeout между chunks - если нет данных 15 секунд, прерываем
        CHUNK_TIMEOUT = 15.0
        # Общий timeout для всего запроса - 60 секунд
        TOTAL_REQUEST_TIMEOUT = 60.0

        # Результаты streaming (для передачи между потоками)
        streaming_result = {"full_response": "", "chunk_count": 0, "error": None}

        # Инкрементируем поколение — все предыдущие стейл-потоки перестанут публиковать
        self._stream_generation += 1
        _my_gen = self._stream_generation

        def _do_streaming():
            """Внутренняя функция для streaming в отдельном потоке"""
            full_response = ""
            current_chunk = ""
            brace_count = 0
            in_json = False
            chunk_count = 0
            start_time = time.time()  # Засекаем время начала
            last_chunk_time = start_time  # Время последнего chunk с контентом
            
            # Накопитель для tool_calls (могут приходить по частям в streaming)
            tool_calls_accumulator = {}  # index -> {id, function: {name, arguments}}

            # Определяем нужно ли добавлять tools
            request_params = {
                "model": self.model,
                "messages": messages,
                "temperature": self.temperature,
                "max_tokens": self.max_tokens,
                "stream": True,
                "stream_options": {"include_usage": True}  # Включаем информацию о токенах
            }
            
            # Добавляем tools только если MCP доступен и есть инструменты
            if self.enable_mcp_tools and self.mcp_tools_available and self.available_tools:
                request_params["tools"] = self.available_tools
                self.get_logger().info(f"🛠️  Отправка запроса с {len(self.available_tools)} MCP инструментами")
            else:
                self.get_logger().info(f"🚫 MCP инструменты НЕ отправлены (enable={self.enable_mcp_tools}, available={self.mcp_tools_available}, tools={len(self.available_tools) if self.available_tools else 0})")

            # Создаём stream напрямую (мы уже внутри ThreadPoolExecutor)
            # httpx client имеет свой timeout (60s total, 10s connect)
            try:
                stream = self.client.chat.completions.create(**request_params)
            except Exception as e:
                self.get_logger().error(f"⏱️ Ошибка создания stream: {e}")
                streaming_result["error"] = f"Failed to create stream: {e}"
                return

            for chunk in stream:
                # Timeout если между chunks прошло слишком много времени
                # Проверяем на каждой итерации - защита от зависания на любом этапе
                elapsed_since_content = time.time() - last_chunk_time
                if elapsed_since_content > CHUNK_TIMEOUT:
                    streaming_result["error"] = f"No data for {elapsed_since_content:.1f}s (after {chunk_count} chunks)"
                    return

                # ============ Обработка tool_calls (приходят по частям в streaming) ============
                if hasattr(chunk.choices[0].delta, 'tool_calls') and chunk.choices[0].delta.tool_calls:
                    for tc_chunk in chunk.choices[0].delta.tool_calls:
                        idx = tc_chunk.index
                        
                        # Инициализируем накопитель для этого tool_call
                        if idx not in tool_calls_accumulator:
                            tool_calls_accumulator[idx] = {
                                'id': '',
                                'type': 'function',
                                'function': {
                                    'name': '',
                                    'arguments': ''
                                }
                            }
                        
                        # Накапливаем данные
                        if tc_chunk.id:
                            tool_calls_accumulator[idx]['id'] = tc_chunk.id
                        if hasattr(tc_chunk, 'function'):
                            if tc_chunk.function.name:
                                tool_calls_accumulator[idx]['function']['name'] = tc_chunk.function.name
                            if tc_chunk.function.arguments:
                                tool_calls_accumulator[idx]['function']['arguments'] += tc_chunk.function.arguments
                        
                        # Обновляем время - tool_calls идут
                        last_chunk_time = time.time()
                        self.get_logger().debug(f"🔧 Tool call chunk получен: index={idx}")

                # ВАЖНО: Сначала обрабатываем content, потом проверяем finish_reason!
                # У Qwen последний chunk может содержать и content и finish_reason одновременно
                if chunk.choices[0].delta.content:
                    token = chunk.choices[0].delta.content
                    full_response += token
                    current_chunk += token
                    last_chunk_time = time.time()  # Обновляем время - контент идёт
                    
                    # DEBUG: Показываем сырые данные
                    self.get_logger().debug(f"📦 Raw token: {repr(token[:100])}")

                    # Подсчёт скобок для определения границ JSON
                    for char in token:
                        if char == "{":
                            brace_count += 1
                            in_json = True
                        elif char == "}":
                            brace_count -= 1

                    # Если скобки сбалансированы - парсим
                    if in_json and brace_count == 0:
                        # Может быть несколько JSON объектов в current_chunk
                        # Два формата:
                        # 1. DeepSeek: {"chunk":1}{"chunk":2} (без переносов)
                        # 2. Qwen: {"chunk":1}\n{"chunk":2} (с переносами)
                        # Универсальное решение: split по \n, потом по }{
                        import re
                        json_objects = []
                        
                        # Разбиваем по переносам строк
                        lines = current_chunk.strip().split('\n')
                        
                        for line in lines:
                            line = line.strip()
                            if not line:
                                continue
                            # В каждой строке может быть несколько JSON подряд: }{
                            parts = re.split(r'(?<=\})(?=\{)', line)
                            json_objects.extend([p.strip() for p in parts if p.strip()])
                        
                        for json_text in json_objects:
                            json_text = json_text.strip()
                            if not json_text:
                                continue
                            
                            # DEBUG: Показываем что пытаемся парсить
                            self.get_logger().info(f"🔍 Пытаюсь парсить JSON: {json_text[:200]}...")

                            # Убираем markdown ```json если есть
                            if json_text.startswith("```json"):
                                json_text = json_text.replace("```json", "").replace("```", "").strip()

                            # Парсим JSON
                            try:
                                chunk_data = json.loads(json_text)
                                self.get_logger().info(f"✅ JSON успешно распарсен: chunk={chunk_data.get('chunk', '?')}, emotion={chunk_data.get('emotion', '?')}")

                                # ============ ПРОВЕРКА: ask_reflection команда ============
                                if "action" in chunk_data and chunk_data["action"] == "ask_reflection":
                                    question = chunk_data.get("question", "")
                                    self.get_logger().warning(f'🔁 LLM перенаправляет к Reflection: "{question}"')

                                    # Публикуем в /perception/user_speech для reflection_node
                                    reflection_msg = String()
                                    reflection_msg.data = question
                                    self.reflection_request_pub.publish(reflection_msg)
                                    self.get_logger().info("  → Запрос отправлен к внутреннему диалогу")
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
                                    
                                    # Публикуем анимацию для первого chunk (до начала речи)
                                    if chunk_count == 1:
                                        emotion = chunk_data.get("emotion", "neutral")
                                        animation_name = self._map_emotion_to_animation(emotion)
                                        
                                        if animation_name and animation_name != "idle":
                                            anim_msg = String()
                                            anim_msg.data = animation_name
                                            self.animation_pub.publish(anim_msg)
                                            self.get_logger().info(f"🎨 Отправлена анимация: {animation_name} (emotion: {emotion})")
                                    
                                    self.get_logger().info(
                                        f"📤 Chunk {chunk_count} (dialogue_id: {dialogue_id[:8]}...): {ssml[:50]}..."
                                    )

                                    # Обновляем время взаимодействия (робот говорит)
                                    self.last_interaction_time = time.time()

                                    response_msg = String()
                                    response_msg.data = json.dumps(chunk_data, ensure_ascii=False)
                                    if _my_gen != self._stream_generation:
                                        self.get_logger().warning(f"🚫 Стейл-поток (gen {_my_gen} != {self._stream_generation}) — chunk отброшен")
                                        return
                                    self.response_pub.publish(response_msg)
                                    # NOTE: НЕ публикуем в tts_pub - tts_node уже подписан на response_pub

                                    self.get_logger().info(f"🔊 Отправлено в TTS: chunk {chunk_count}")

                            except json.JSONDecodeError as e:
                                self.get_logger().warning(f"⚠️  JSON decode failed: {e}, text: {json_text[:200]}...")
                                pass  # Этот JSON неполный

                        # Сброс для следующего chunk
                        current_chunk = ""
                        in_json = False
                        brace_count = 0

                # ВАЖНО: Проверяем finish_reason ПОСЛЕ обработки всего content
                # У Qwen последний chunk может содержать и content и finish_reason
                if chunk.choices[0].finish_reason:
                    finish_reason = chunk.choices[0].finish_reason
                    self.get_logger().info(f"🏁 Stream завершён: {finish_reason} (обработано {chunk_count} chunks)")
                    
                    # ============ Логируем использование токенов ============
                    if hasattr(chunk, 'usage') and chunk.usage:
                        usage = chunk.usage
                        prompt_tokens = getattr(usage, 'prompt_tokens', 0)
                        completion_tokens = getattr(usage, 'completion_tokens', 0)
                        total_tokens = getattr(usage, 'total_tokens', 0)
                        self.get_logger().info(
                            f"📊 Token usage: input={prompt_tokens}, output={completion_tokens}, total={total_tokens}"
                        )
                    
                    # ============ Обработка tool_calls если LLM запросил выполнение инструментов ============
                    if finish_reason == 'tool_calls' and tool_calls_accumulator:
                        self.get_logger().info(f"🔧 LLM запросил выполнение {len(tool_calls_accumulator)} инструментов")
                        
                        # Сохраняем результат - нужно будет обработать tool_calls снаружи
                        streaming_result["tool_calls"] = list(tool_calls_accumulator.values())
                        streaming_result["full_response"] = full_response
                        streaming_result["chunk_count"] = chunk_count
                        return  # Выходим из streaming для обработки tool_calls
                    
                    break

            # Сохраняем результаты
            streaming_result["full_response"] = full_response
            streaming_result["chunk_count"] = chunk_count
            
            # DEBUG: Итоговая статистика
            self.get_logger().info(f"📊 Stream завершён: {len(full_response)} chars, {chunk_count} chunks")
            
            # FALLBACK: Если получен ответ без JSON разметки - отправляем как plain text
            if chunk_count == 0 and len(full_response) > 0:
                self.get_logger().warning(f"⚠️  Получен plain text без JSON ({len(full_response)} chars), отправляю как один chunk")
                
                # Формируем JSON с SSML для TTS
                text = full_response.strip()
                chunk_data = {
                    "chunk": "end",
                    "ssml": f"<speak>{text}</speak>",
                    "emotion": "neutral",
                    "dialogue_id": dialogue_id,
                    "speech_id": str(uuid.uuid4())
                }
                
                # Публикуем в response (tts_node подписан на него)
                response_msg = String()
                response_msg.data = json.dumps(chunk_data, ensure_ascii=False)
                if _my_gen != self._stream_generation:
                    self.get_logger().warning(f"🚫 Стейл-поток plain-text (gen {_my_gen} != {self._stream_generation}) — отброшен")
                    return
                self.response_pub.publish(response_msg)
                
                chunk_count = 1  # Считаем это как 1 chunk
                streaming_result["chunk_count"] = 1
                
                self.get_logger().info(f"🔊 Plain text отправлен в TTS как 1 chunk (SSML создан)")

        # Запускаем streaming в отдельном потоке с timeout
        # ВАЖНО: НЕ используем `with ThreadPoolExecutor` как context manager!
        # При timeout future.result() бросает FuturesTimeoutError, но __exit__ вызывает shutdown(wait=True)
        # что ЖДЁТ завершения _do_streaming пока тот висит на сетевом I/O → вечное зависание
        # и блокировка ROS2 callback thread → stt_callback не может выполниться.
        try:
            _stream_executor = ThreadPoolExecutor(max_workers=1)
            future = _stream_executor.submit(_do_streaming)
            try:
                future.result(timeout=TOTAL_REQUEST_TIMEOUT)
            finally:
                _stream_executor.shutdown(wait=False)  # Не блокировать! Фоновый поток умрёт сам

            # Проверяем внутренний timeout
            if streaming_result["error"]:
                raise TimeoutError(streaming_result["error"])

            # ============ Обработка tool_calls если LLM запросил выполнение ============
            if "tool_calls" in streaming_result and streaming_result["tool_calls"]:
                self.get_logger().info("🔧 Обнаружены tool_calls от LLM - начинаю выполнение")
                
                # Проверяем доступность MCP
                if not self.enable_mcp_tools or not self.mcp_adapter:
                    self.get_logger().error("❌ Tool calls запрошены но MCP не доступен - fallback на обычный ответ")
                    self._speak_simple("Извините, функции управления сейчас недоступны", show_error_animation=True)
                    self.llm_processing = False
                    self.dialogue_in_progress = False
                    return
                
                # Выполняем tool_calls через MCP adapter
                tool_calls = streaming_result["tool_calls"]
                tool_results = self._execute_tool_calls(tool_calls, messages)
                
                if tool_results is None:
                    # Ошибка выполнения - уже залогирована в _execute_tool_calls
                    self.llm_processing = False
                    self.dialogue_in_progress = False
                    return
                
                # Продолжаем диалог с результатами tool calls
                self._continue_after_tool_calls(messages, tool_calls, tool_results)
                return  # Выходим - _continue_after_tool_calls сам завершит обработку

            # Streaming успешно завершён (без tool_calls)
            full_response = streaming_result["full_response"]
            chunk_count = streaming_result["chunk_count"]

            # Сохраняем ответ в историю
            self.conversation_history.add_assistant_message(full_response)
            if self.voice_memory is not None:
                self.voice_memory.save_turn("assistant", full_response)

            # Успешный запрос - сбрасываем счётчик ошибок
            if self.provider_error_count > 0:
                self.get_logger().info(f"✅ Провайдер работает! Сброс счётчика ошибок ({self.provider_error_count} → 0)")
                self.provider_error_count = 0

            provider_name = self.PROVIDERS[self.current_provider]["name"]
            self.get_logger().info(f"✅ {provider_name} ответил ({chunk_count} chunks)")

            # Сбрасываем флаг обработки LLM
            self.llm_processing = False

            # Проверяем очередь - есть ли ещё накопленные запросы
            if self.dialogue_manager.pending_queries:
                self.get_logger().info(f"📬 В очереди есть ещё запросы ({len(self.dialogue_manager.pending_queries)})")
                # Обновляем время последнего запроса для корректного накопления
                self.dialogue_manager.last_query_time = time.time()
                # Запускаем таймер накопления чтобы дождаться возможных дополнительных запросов
                if self.accumulation_timer is None:
                    self.accumulation_timer = self.create_timer(
                        self.dialogue_manager.query_accumulation_timeout, self._check_and_process_queue
                    )
                    self.get_logger().info(
                        f"⏰ Запущен таймер накопления для оставшихся запросов ({self.dialogue_manager.query_accumulation_timeout}s)"
                    )
                # Не сбрасываем dialogue_in_progress - ещё есть запросы в очереди
            else:
                # Очередь пуста - завершаем диалог
                self.dialogue_in_progress = False

        except (FuturesTimeoutError, TimeoutError) as e:
            provider_name = self.PROVIDERS[self.current_provider]["name"]
            self.get_logger().error(f"⏱️ TIMEOUT: {provider_name} streaming не ответил за {TOTAL_REQUEST_TIMEOUT}s - {e}")
            
            # Увеличиваем счётчик ошибок
            self.provider_error_count += 1
            self.get_logger().warning(f"⚠️ Ошибка {self.provider_error_count}/{self.provider_error_threshold} для {provider_name}")

            # Пересоздаём клиент — старый httpx.Client может содержать сломанные сокеты
            self._recreate_llm_client()
            
            # Пробуем fallback если превысили порог
            if self.provider_error_count >= self.provider_error_threshold:
                self.get_logger().warning(f"🔄 Слишком много ошибок, пытаемся переключиться на fallback...")
                if self._try_fallback_provider():
                    self.provider_error_count = 0  # Сбрасываем счётчик
                    # Пробуем снова с новым провайдером если есть запросы в очереди
                    if self.dialogue_manager.pending_queries:
                        self.get_logger().info("♻️ Повторяем запрос с новым провайдером")
                        self.llm_processing = False
                        self.dialogue_manager.last_query_time = time.time()
                        if self.accumulation_timer is not None:
                            self.accumulation_timer.cancel()
                        self.accumulation_timer = self.create_timer(0.5, self._check_and_process_queue)
                        return
                # Fallback тоже не помог — говорим ошибку
                self._speak_simple("Извините, я сейчас не в настроении думать", show_error_animation=True)
                self.llm_processing = False
                self.dialogue_in_progress = False
            else:
                # count < threshold — тихий retry через 2s (conversation_history уже содержит user message)
                self.get_logger().warning(
                    f"♻️ Тихий retry через 2s (ошибка {self.provider_error_count}/{self.provider_error_threshold})"
                )
                self.llm_processing = False
                if self._timeout_retry_timer is not None:
                    self._timeout_retry_timer.cancel()
                self._timeout_retry_timer = self.create_timer(2.0, self._do_timeout_retry)

        except Exception as e:
            provider_name = self.PROVIDERS[self.current_provider]["name"]
            self.get_logger().error(f"❌ Ошибка {provider_name}: {e}")
            
            # Увеличиваем счётчик ошибок
            self.provider_error_count += 1
            self.get_logger().warning(f"⚠️ Ошибка {self.provider_error_count}/{self.provider_error_threshold} для {provider_name}")
            
            # Пробуем fallback если превысили порог
            if self.provider_error_count >= self.provider_error_threshold:
                self.get_logger().warning(f"🔄 Слишком много ошибок, пытаемся переключиться на fallback...")
                if self._try_fallback_provider():
                    self.provider_error_count = 0  # Сбрасываем счётчик
            
            # Сбрасываем флаг обработки LLM
            self.llm_processing = False

            # Проверяем очередь даже при ошибке
            if self.dialogue_manager.pending_queries:
                self.get_logger().info(
                    f"📬 В очереди есть запросы ({len(self.dialogue_manager.pending_queries)}), пробую снова после задержки..."
                )
                # Обновляем время последнего запроса
                self.dialogue_manager.last_query_time = time.time()
                # Небольшая задержка перед повтором при ошибке
                # Отменяем существующий таймер если есть
                if self.accumulation_timer is not None:
                    self.accumulation_timer.cancel()
                # Создаём новый таймер с задержкой для повтора
                self.accumulation_timer = self.create_timer(self.error_retry_delay, self._check_and_process_queue)
                # Не сбрасываем dialogue_in_progress - ещё есть запросы для повтора
            else:
                # Очередь пуста - завершаем диалог даже при ошибке
                self.dialogue_in_progress = False

    def _ask_llm_non_streaming(self):
        """Non-streaming запрос к LLM провайдеру с поддержкой tool calls"""
        try:
            # Собираем все сообщения для контекста
            messages = [{"role": "system", "content": self.system_prompt}]
            messages.extend(self.conversation_history.get_messages())

            provider_name = self.PROVIDERS[self.current_provider]["name"]
            last_msg = self.conversation_history.get_last_message()
            if last_msg:
                self.get_logger().info(f"🤖 {provider_name} запрос (non-streaming): {getattr(last_msg, 'content', '')[:80]}...")

            # Параметры запроса
            request_params = {
                "model": self.model,
                "messages": messages,
                "temperature": self.temperature,
                "max_tokens": self.max_tokens,
                "stream": False,
            }

            # Формируем extra_body в зависимости от провайдера
            if self.current_provider == "qwen":
                request_params["extra_body"] = {"enable_search": True}

            # Добавляем tools только если MCP доступен и есть инструменты
            if self.enable_mcp_tools and self.mcp_tools_available and self.available_tools:
                request_params["tools"] = self.available_tools
                self.get_logger().info(f"🛠️  Отправка non-streaming запроса с {len(self.available_tools)} MCP инструментами")
            else:
                self.get_logger().info(f"🚫 MCP инструменты НЕ отправлены (enable={self.enable_mcp_tools}, available={self.mcp_tools_available}, tools={len(self.available_tools) if self.available_tools else 0})")

            # Цикл обработки tool calls (агентный workflow)
            max_iterations = 30  # Защита от бесконечного цикла
            iteration = 0
            failed_tools_count = 0  # Счетчик failed tools подряд

            while iteration < max_iterations:
                iteration += 1
                self.get_logger().info(f"🔄 Агентный цикл: итерация {iteration}/{max_iterations}")
                # Сбрасываем тайм-аут диалога — LLM активно работает, диалог живой
                self.dialogue_manager.last_interaction_time = time.time()

                # Проверка прерывания от нового запроса пользователя
                if self.interrupt_agent_loop:
                    self.get_logger().warning("🛑 Агентный цикл прерван новым запросом пользователя")
                    self.interrupt_agent_loop = False
                    break

                # Делаем запрос к LLM
                response = self.client.chat.completions.create(**request_params)
                message = response.choices[0].message

                # Проверяем наличие tool_calls
                if hasattr(message, 'tool_calls') and message.tool_calls:
                    tool_calls = message.tool_calls
                    self.get_logger().info(f"🔧 LLM вернул {len(tool_calls)} tool_calls")

                    # Добавляем assistant message с tool_calls в историю
                    assistant_message = {
                        "role": "assistant",
                        "content": message.content or "",
                        "tool_calls": [
                            {
                                "id": tc.id,
                                "type": "function",
                                "function": {
                                    "name": tc.function.name,
                                    "arguments": tc.function.arguments
                                }
                            }
                            for tc in tool_calls
                        ]
                    }
                    self.conversation_history.add_assistant_message_with_tools(None, assistant_message["tool_calls"])
                    messages.append(assistant_message)

                    # Выполняем tool_calls через LLMToolCallAdapter
                    for tool_call in tool_calls:
                        tool_name = tool_call.function.name
                        tool_args_str = tool_call.function.arguments
                        tool_id = tool_call.id

                        self.get_logger().info(f"🛠️  Выполнение: {tool_name}({tool_args_str[:100]}...)")

                        # Парсим аргументы
                        try:
                            tool_args = json.loads(tool_args_str)
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f"❌ Не удалось распарсить аргументы для {tool_name}: {e}")
                            result = {
                                'success': False,
                                'error': f'Неверный формат аргументов: {e}'
                            }
                        else:
                            # Используем синхронное выполнение через MCP adapter
                            result = self.mcp_adapter.execute_tool_call_sync(tool_name, tool_args, timeout=2.0)

                        # Добавляем результат в историю
                        self.conversation_history.add_tool_message(
                            content=json.dumps(result, ensure_ascii=False),
                            tool_call_id=tool_id,
                            name=tool_name
                        )
                        tool_result_message = {
                            "role": "tool",
                            "tool_call_id": tool_id,
                            "name": tool_name,
                            "content": json.dumps(result, ensure_ascii=False)
                        }
                        messages.append(tool_result_message)

                        if result.get('success'):
                            self.get_logger().info(f"✅ {tool_name} выполнен: {result.get('message', 'OK')[:50]}")
                            failed_tools_count = 0  # Сброс счетчика при успехе
                        else:
                            self.get_logger().warning(f"⚠️  {tool_name} вернул ошибку: {result.get('error', 'Unknown')}")
                            failed_tools_count += 1
                            
                        # Выход при множественных ошибках
                        if failed_tools_count >= 3:
                            self.get_logger().error(f"❌ Слишком много ошибок tool calls ({failed_tools_count}), прерываю агентный цикл")
                            break

                    # Если достигли лимита ошибок - выходим из агентного цикла
                    if failed_tools_count >= 3:
                        error_msg = "Извините, у меня технические проблемы с выполнением команд. Попробуйте позже."
                        self._speak_simple(error_msg, show_error_animation=True)
                        break

                    # Обновляем request_params для следующей итерации
                    request_params["messages"] = messages

                else:
                    # Нет tool_calls - это финальный ответ
                    final_content = message.content or ""
                    self.get_logger().info(f"📥 {provider_name} финальный ответ: {len(final_content)} символов")

                    # Успешный запрос - сбрасываем счётчик ошибок
                    if self.provider_error_count > 0:
                        self.get_logger().info(f"✅ Провайдер работает! Сброс счётчика ({self.provider_error_count} → 0)")
                        self.provider_error_count = 0

                    # Сохраняем финальный ответ в историю
                    self.conversation_history.add_assistant_message(final_content)
                    if self.voice_memory is not None and final_content.strip():
                        self.voice_memory.save_turn("assistant", final_content)

                    # Если есть текст - публикуем как финальный chunk (для legacy совместимости)
                    if final_content.strip():
                        self.get_logger().info(f"💬 Финальное сообщение от LLM: {final_content[:100]}...")
                        # Публикуем с SSML оборачиванием
                        text = final_content.strip()
                        end_msg = String()
                        end_msg.data = json.dumps({
                            "chunk": "final",
                            "ssml": f"<speak>{text}</speak>",
                            "emotion": "neutral",
                            "message": "LLM завершил агентный диалог"
                        })
                        self.response_pub.publish(end_msg)

                    # Выход из цикла - диалог завершен
                    break

            if iteration >= max_iterations:
                self.get_logger().warning(f"⚠️  Достигнут лимит итераций ({max_iterations}), завершаю агентный цикл")

            # Сбрасываем флаг обработки LLM
            self.llm_processing = False

            # Проверяем очередь - есть ли ещё накопленные запросы
            if self.dialogue_manager.pending_queries:
                self.get_logger().info(f"📬 В очереди есть ещё запросы ({len(self.dialogue_manager.pending_queries)})")
                self.dialogue_manager.last_query_time = time.time()
                if self.accumulation_timer is None:
                    self.accumulation_timer = self.create_timer(
                        self.dialogue_manager.query_accumulation_timeout, self._check_and_process_queue
                    )
            else:
                self.dialogue_in_progress = False

        except Exception as e:
            provider_name = self.PROVIDERS[self.current_provider]["name"]
            self.get_logger().error(f"❌ Ошибка {provider_name} non-streaming: {e}")
            
            # Увеличиваем счётчик ошибок
            self.provider_error_count += 1
            self.get_logger().warning(f"⚠️ Ошибка {self.provider_error_count}/{self.provider_error_threshold} для {provider_name}")
            
            # Пробуем fallback если превысили порог
            if self.provider_error_count >= self.provider_error_threshold:
                self.get_logger().warning(f"🔄 Слишком много ошибок, пытаемся переключиться на fallback...")
                if self._try_fallback_provider():
                    self.provider_error_count = 0  # Сбрасываем счётчик
            
            self.llm_processing = False
            self._speak_simple("Извините, возникла проблема с ответом", show_error_animation=True)
            
            if self.dialogue_manager.pending_queries:
                self.dialogue_manager.last_query_time = time.time()
                if self.accumulation_timer is not None:
                    self.accumulation_timer.cancel()
                self.accumulation_timer = self.create_timer(self.error_retry_delay, self._check_and_process_queue)
            else:
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
        # ВАЖНО: max/normal должны быть раньше louder/quieter,
        # т.к. r"громко" матчит внутри "громкость"
        volume_patterns = {
            "max": [
                r"говори громко",
                r"максимальн\w* громкост",
                r"на полную громкост",
            ],
            "normal": [
                r"нормальн\w* громкост",
                r"стандартн\w* громкост",
                r"обычн\w* громкост",
            ],
            "louder": [
                r"громче",
                r"громко",
                r"прибав\w* громкост",
                r"увелич\w* громкост",
            ],
            "quieter": [
                r"тише",
                r"потише",
                r"убав\w* громкост",
                r"уменьш\w* громкост",
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
            from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
            from rcl_interfaces.srv import GetParameters, SetParameters

            # Создаем клиенты для работы с параметрами TTS и Sound нод
            tts_get_params_client = self.create_client(GetParameters, "/tts_node/get_parameters")
            tts_set_params_client = self.create_client(SetParameters, "/tts_node/set_parameters")
            sound_set_params_client = self.create_client(SetParameters, "/sound_node/set_parameters")

            # Ждем доступности сервисов
            if not tts_get_params_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("❌ TTS node параметры недоступны")
                return "Извините, не могу изменить громкость. Система недоступна."

            # Получаем текущий volume_db
            get_request = GetParameters.Request()
            get_request.names = ["volume_db"]
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

            if intent == "louder":
                new_volume_db = min(current_volume_db + 3.0, 6.0)  # +3dB, макс +6dB
                response_text = "Делаю громче"
            elif intent == "quieter":
                new_volume_db = max(current_volume_db - 3.0, -20.0)  # -3dB, мин -20dB
                response_text = "Делаю тише"
            elif intent == "max":
                new_volume_db = 6.0  # Максимальная громкость +6dB (~2x)
                response_text = "Максимальная громкость"
            elif intent == "normal":
                new_volume_db = -3.0  # Нормальная громкость -3dB (70%)
                response_text = "Нормальная громкость"

            # Устанавливаем новую громкость
            if abs(new_volume_db - current_volume_db) < 0.1:
                # Громкость уже на пределе
                if intent == "louder":
                    return "Громкость уже максимальная"
                elif intent == "quieter":
                    return "Громкость уже минимальная"

            # Устанавливаем параметры для TTS ноды
            set_request = SetParameters.Request()
            param = Parameter()
            param.name = "volume_db"
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
            "higher": [
                r"говори выше",
                r"голос выше",
                r"повыс\w* голос",
                r"выше говор",
            ],
            "lower": [
                r"говори ниже",
                r"голос ниже",
                r"пониж\w* голос",
                r"ниже говор",
            ],
            "normal": [
                r"нормальн\w* голос",
                r"обычн\w* голос",
                r"говори нормально",
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
        # pitch_shift - это дополнительный множитель к базовому эффекту бурундука (2.0x)
        # pitch_shift=1.0 → стандартный ROBBOX (2.0x эффект)
        # pitch_shift=1.5 → голос ещё выше (3.0x эффект)
        # pitch_shift=0.8 → голос ниже (1.6x эффект)
        # ВАЖНО: chipmunk_mode всегда остаётся True для сохранения эффекта ROBBOX
        try:
            from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
            from rcl_interfaces.srv import GetParameters, SetParameters

            # Создаем клиенты для работы с параметрами TTS ноды
            tts_get_params_client = self.create_client(GetParameters, "/tts_node/get_parameters")
            tts_set_params_client = self.create_client(SetParameters, "/tts_node/set_parameters")

            # Ждем доступности сервисов
            if not tts_get_params_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("❌ TTS node параметры недоступны")
                return "Извините, не могу изменить высоту голоса. Система недоступна."

            # Получаем текущий pitch_shift
            get_request = GetParameters.Request()
            get_request.names = ["pitch_shift"]
            future = tts_get_params_client.call_async(get_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() is None or len(future.result().values) < 1:
                self.get_logger().error("❌ Не удалось получить параметры TTS")
                return "Извините, не могу изменить высоту голоса."

            # Извлекаем текущее значение pitch_shift
            current_pitch = future.result().values[0].double_value

            self.get_logger().info(
                f"📊 Текущий pitch_shift: {current_pitch:.2f} (итого: {2.0 * current_pitch:.1f}x эффект)"
            )

            # Вычисляем новое значение pitch_shift
            # Диапазон: 0.5 (низкий бурундук, 1.0x эффект) - 2.0 (очень высокий, 4.0x эффект)
            # ROBBOX стандарт: 1.0 = оригинальный голос бурундука (2.0x эффект)
            new_pitch = current_pitch
            response_text = ""

            if intent == "higher":
                # Увеличиваем pitch - делаем голос выше
                new_pitch = min(current_pitch + 0.2, 2.0)  # +0.2, макс 2.0 (4.0x эффект!)
                response_text = "Говорю выше"
            elif intent == "lower":
                # Уменьшаем pitch - делаем голос ниже (но всё ещё бурундук!)
                new_pitch = max(current_pitch - 0.2, 0.5)  # -0.2, мин 0.5 (1.0x эффект - лёгкий)
                response_text = "Говорю ниже"
            elif intent == "normal":
                # Нормальный ROBBOX голос с эффектом бурундука
                new_pitch = 1.0  # 2.0x эффект - как в оригинале!
                response_text = "Нормальный голос"

            # Проверяем изменение pitch
            if abs(new_pitch - current_pitch) < 0.01:
                # Параметр не изменился
                if intent == "higher":
                    return "Голос уже максимально высокий"
                elif intent == "lower":
                    return "Голос уже минимально низкий"

            # Устанавливаем новый pitch_shift для TTS ноды
            set_request = SetParameters.Request()

            param_pitch = Parameter()
            param_pitch.name = "pitch_shift"
            param_pitch.value = ParameterValue()
            param_pitch.value.type = ParameterType.PARAMETER_DOUBLE
            param_pitch.value.double_value = new_pitch

            set_request.parameters = [param_pitch]

            future = tts_set_params_client.call_async(set_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() is None:
                self.get_logger().error("❌ Не удалось установить параметры TTS")
                return "Извините, не могу изменить высоту голоса."

            # Проверяем успешность установки
            if not future.result().results[0].successful:
                self.get_logger().error("❌ Параметр pitch_shift не установился")
                return "Извините, не могу изменить высоту голоса."

            self.get_logger().info(
                f"✅ Pitch изменён: {current_pitch:.2f} → {new_pitch:.2f} "
                f"(эффект: {2.0 * current_pitch:.1f}x → {2.0 * new_pitch:.1f}x)"
            )
            return response_text

        except Exception as e:
            self.get_logger().error(f"❌ Ошибка изменения pitch: {e}")
            return "Извините, произошла ошибка."

    # ============================================================
    # Speed Control Commands
    # ============================================================

    def _detect_speed_intent(self, text: str):
        """Определить intent для команд управления скоростью речи

        Returns:
            str: 'faster', 'slower', 'normal', None
        """
        text_lower = text.lower()

        # Паттерны для различных команд скорости
        speed_patterns = {
            "faster": [
                r"говори быстрее",
                r"быстрее",
                r"ускор\w*",
                r"побыстрее",
            ],
            "slower": [
                r"говори медленнее",
                r"медленнее",
                r"замедл\w*",
                r"помедленнее",
            ],
            "normal": [
                r"нормальн\w* скорост",
                r"обычн\w* скорост",
            ],
        }

        for intent, patterns in speed_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    return intent

        return None

    def _handle_speed_command(self, intent: str) -> str:
        """Обработка команды изменения скорости речи через yandex_speed

        Args:
            intent: 'faster', 'slower', 'normal'

        Returns:
            str: Ответ для пользователя
        """
        # Управляем скоростью синтеза через yandex_speed параметр
        # Диапазон: 0.1-3.0, где 1.0 = нормальная скорость
        try:
            from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
            from rcl_interfaces.srv import GetParameters, SetParameters

            # Создаем клиенты для работы с параметрами TTS ноды
            tts_get_params_client = self.create_client(GetParameters, "/tts_node/get_parameters")
            tts_set_params_client = self.create_client(SetParameters, "/tts_node/set_parameters")

            # Ждем доступности сервисов
            if not tts_get_params_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("❌ TTS node параметры недоступны")
                return "Извините, не могу изменить скорость речи. Система недоступна."

            # Получаем текущий yandex_speed
            get_request = GetParameters.Request()
            get_request.names = ["yandex_speed"]
            future = tts_get_params_client.call_async(get_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() is None or len(future.result().values) < 1:
                self.get_logger().error("❌ Не удалось получить yandex_speed")
                return "Извините, не могу изменить скорость речи."

            current_speed = future.result().values[0].double_value
            self.get_logger().info(f"📊 Текущая скорость синтеза: {current_speed:.2f}")

            # Вычисляем новую скорость
            # Диапазон: 0.5 (медленно) - 2.0 (быстро), 1.0 = нормально
            new_speed = current_speed
            response_text = ""

            if intent == "faster":
                new_speed = min(current_speed + 0.2, 2.0)  # +0.2, макс 2.0
                response_text = "Говорю быстрее"
            elif intent == "slower":
                new_speed = max(current_speed - 0.2, 0.5)  # -0.2, мин 0.5
                response_text = "Говорю медленнее"
            elif intent == "normal":
                new_speed = 1.0  # Нормальная скорость
                response_text = "Нормальная скорость"

            # Проверяем изменение
            if abs(new_speed - current_speed) < 0.01:
                if intent == "faster":
                    return "Скорость уже максимальная"
                elif intent == "slower":
                    return "Скорость уже минимальная"

            # Устанавливаем новую скорость
            set_request = SetParameters.Request()
            param = Parameter()
            param.name = "yandex_speed"
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = new_speed
            set_request.parameters = [param]

            future = tts_set_params_client.call_async(set_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() is None or not future.result().results[0].successful:
                self.get_logger().error("❌ Не удалось установить yandex_speed")
                return "Извините, не могу изменить скорость речи."

            self.get_logger().info(f"✅ Скорость изменена: {current_speed:.2f} → {new_speed:.2f}")
            return response_text

        except Exception as e:
            self.get_logger().error(f"❌ Ошибка изменения скорости: {e}")
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
            self._trigger_sound("error")
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
            self._trigger_sound("error")
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
        if self.dialogue_manager.check_timeout():
            self.get_logger().info(f"⏰ Dialogue timeout → IDLE")
            self._publish_state()

    # ============================================================
    # MCP Tools Integration Methods
    # ============================================================

    def _execute_tool_calls(self, tool_calls: List[Dict[str, Any]], messages: List[Dict[str, Any]]) -> Optional[List[Dict[str, Any]]]:
        """
        Выполняет tool_calls через MCP adapter
        
        Args:
            tool_calls: Список tool calls от LLM
            messages: Текущая история сообщений
            
        Returns:
            Список результатов выполнения или None в случае ошибки
        """
        # Лимит на количество tool_calls за раз (защита от зацикливания)
        MAX_TOOL_CALLS = 5
        if len(tool_calls) > MAX_TOOL_CALLS:
            self.get_logger().warning(f"⚠️ LLM запросил {len(tool_calls)} инструментов, обрезаю до {MAX_TOOL_CALLS}")
            tool_calls = tool_calls[:MAX_TOOL_CALLS]
        
        tool_results = []
        failed_count = 0
        
        for tool_call in tool_calls:
            try:
                tool_name = tool_call['function']['name']
                tool_args_json = tool_call['function']['arguments']
                tool_id = tool_call.get('id', 'unknown')
                
                self.get_logger().info(f"🔧 Выполнение: {tool_name}")
                self.get_logger().debug(f"   Аргументы: {tool_args_json[:200]}...")
                
                # Парсим аргументы
                try:
                    tool_args = json.loads(tool_args_json)
                except json.JSONDecodeError as e:
                    self.get_logger().error(f"❌ Не удалось распарсить аргументы для {tool_name}: {e}")
                    tool_results.append({
                        'tool_call_id': tool_id,
                        'tool_name': tool_name,
                        'success': False,
                        'error': 'Неверный формат аргументов'
                    })
                    failed_count += 1
                    continue
                
                # Выполняем через MCP adapter
                result = self.mcp_adapter.execute_tool_call_sync(tool_name, tool_args, timeout=2.0)
                
                # Добавляем метаданные
                result['tool_call_id'] = tool_id
                result['tool_name'] = tool_name
                tool_results.append(result)
                
                if result.get('success'):
                    self.get_logger().info(f"✅ {tool_name} выполнен успешно")
                else:
                    self.get_logger().warning(f"⚠️ {tool_name} вернул ошибку: {result.get('error', 'Unknown')}")
                    failed_count += 1
                    
                # Если слишком много ошибок подряд - останавливаем выполнение
                if failed_count >= 3:
                    self.get_logger().error(f"❌ Слишком много ошибок ({failed_count}), прерываю выполнение tool_calls")
                    break
                    
            except Exception as e:
                self.get_logger().error(f"❌ Ошибка выполнения tool call: {e}")
                tool_results.append({
                    'tool_call_id': tool_call.get('id', 'unknown'),
                    'tool_name': tool_call.get('function', {}).get('name', 'unknown'),
                    'success': False,
                    'error': str(e)
                })
                failed_count += 1
        
        return tool_results

    def _continue_after_tool_calls(self, messages: List[Dict[str, Any]], tool_calls: List[Dict[str, Any]], tool_results: List[Dict[str, Any]]):
        """
        Итеративный агентный цикл после выполнения tool_calls.

        Заменяет рекурсивную реализацию: стек остаётся плоским (1 фрейм),
        один ThreadPoolExecutor на весь цикл, единственный finally-блок.

        BUG-10/BUG-15 fix:
        - Не создаём новый ThreadPoolExecutor на каждой итерации (устраняет утечку потоков).
        - interrupt_agent_loop проверяется в начале каждой итерации (не только после выхода
          из future.result), что достаточно для текущей архитектуры.
        - Retry реализован как локальная переменная, не рекурсия.
        """
        iteration = 0
        current_tool_calls = tool_calls
        current_tool_results = tool_results

        try:
            while True:
                iteration += 1

                # ── Защита от бесконечного цикла ──────────────────────────────────
                if iteration > self.MAX_ITERATIONS:
                    self.get_logger().error(
                        f"❌ Достигнут лимит итераций агентного цикла ({self.MAX_ITERATIONS}). Прерываю."
                    )
                    self._speak_simple("Извините, я столкнулся с проблемой и не могу продолжить.", show_error_animation=True)
                    return

                # ── 🛑 Прерывание если пользователь заговорил ─────────────────────
                if self.interrupt_agent_loop:
                    self.get_logger().warning(
                        f"🛑 Агентный цикл прерван на итерации {iteration} — новый запрос пользователя"
                    )
                    self.interrupt_agent_loop = False
                    return

                self.get_logger().info(
                    f"🔄 Агентный цикл: итерация {iteration}/{self.MAX_ITERATIONS} (стек: плоский)"
                )
                self.dialogue_manager.last_interaction_time = time.time()

                # ── 🛑 listen_for_response — останавливаем цикл, ждём STT ─────────
                for result in current_tool_results:
                    if result.get('tool_name') == 'listen_for_response':
                        self.get_logger().info("🛑 ОСТАНОВКА: listen_for_response — жду ответа пользователя")
                        self.dialogue_manager.last_interaction_time = time.time()
                        self._listen_response_waiting = True  # защита finally-блока

                        # Сбрасываем очередь — сообщения накопились ПОКА LLM обрабатывал,
                        # они не являются ответом пользователя на listen_for_response.
                        stale = len(self.dialogue_manager.pending_queries)
                        if stale > 0:
                            self.dialogue_manager.pending_queries.clear()
                            self.get_logger().info(
                                f"🧹 pending_queries очищена перед listen_for_response ({stale} устаревших сообщений)"
                            )

                        self.llm_processing = False
                        return

                # ── Добавляем assistant + tool messages в историю ─────────────────
                assistant_msg = {
                    'role': 'assistant',
                    'content': None,
                    'tool_calls': current_tool_calls,
                }
                messages.append(assistant_msg)
                self.conversation_history.add_assistant_message_with_tools(None, current_tool_calls)

                for result in current_tool_results:
                    tool_call_id = result.get('tool_call_id', '')
                    tool_name = result.get('tool_name', 'unknown')
                    if result.get('success'):
                        content = result.get('message', 'Выполнено успешно')
                        if result.get('data'):
                            content += f"\nДанные: {json.dumps(result['data'], ensure_ascii=False)}"
                    else:
                        content = f"Ошибка: {result.get('error', 'Неизвестная ошибка')}"
                    tool_msg = {
                        'role': 'tool',
                        'tool_call_id': tool_call_id,
                        'name': tool_name,
                        'content': content,
                    }
                    messages.append(tool_msg)
                    self.conversation_history.add_tool_message(content, tool_call_id, tool_name)
                    self.get_logger().debug(f"   Tool result для {tool_name}: {content[:100]}...")

                # ── LLM запрос + 1 retry при timeout ─────────────────────────────
                self.get_logger().info("🤖 Запрос к LLM с результатами инструментов")
                recursive_result: Dict[str, Any] = {"full_response": "", "chunk_count": 0, "error": None, "tool_calls": None}
                succeeded = False

                for attempt in range(2):  # attempt 0 = первый запрос, attempt 1 = retry
                    if attempt > 0:
                        self.get_logger().warning(f"♻️ Retry запроса (итерация {iteration})...")
                        # Сбрасываем результат перед повторной попыткой
                        recursive_result = {"full_response": "", "chunk_count": 0, "error": None, "tool_calls": None}

                    # Инкрементируем поколение — стейл-поток attempt N-1 перестанет публиковать
                    self._stream_generation += 1
                    _rec_gen = self._stream_generation

                    def _do_recursive_streaming(_result=recursive_result, _my_gen=_rec_gen):
                        """Streaming в отдельном потоке; захватывает _result по значению (текущий dict)."""
                        try:
                            messages[0] = {"role": "system", "content": self.system_prompt}
                            request_params = {
                                "model": self.model,
                                "messages": messages,
                                "temperature": self.temperature,
                                "max_tokens": self.max_tokens,
                                "stream": True,
                                "stream_options": {"include_usage": True},
                            }
                            if self.enable_mcp_tools and self.mcp_tools_available and self.available_tools:
                                request_params["tools"] = self.available_tools
                                self.get_logger().info(f"🛠️  Запрос С {len(self.available_tools)} MCP инструментами")

                            self.get_logger().info("📞 Создание stream...")
                            try:
                                stream = self.client.chat.completions.create(**request_params)
                            except Exception as e:
                                self.get_logger().error(f"❌ Ошибка создания stream: {e}")
                                _result["error"] = f"Failed to create stream: {e}"
                                return
                            self.get_logger().info("✅ Stream создан, начинаю итерацию...")

                            tool_calls_accumulator: Dict[int, Any] = {}
                            dialogue_id = str(uuid.uuid4())
                            self.current_dialogue_id = dialogue_id
                            chunk_count = 0
                            full_response = ""
                            current_chunk = ""
                            brace_count = 0
                            in_json = False

                            stream_start_time = time.time()
                            last_chunk_time = stream_start_time
                            CHUNK_TIMEOUT = 15.0
                            TOTAL_TIMEOUT = 60.0

                            for chunk in stream:
                                current_time = time.time()
                                if current_time - last_chunk_time > CHUNK_TIMEOUT:
                                    msg = f"stream timeout: no chunks for {CHUNK_TIMEOUT}s"
                                    self.get_logger().error(f"⏱️ {msg}")
                                    _result["error"] = msg
                                    return
                                if current_time - stream_start_time > TOTAL_TIMEOUT:
                                    msg = f"stream total timeout: {TOTAL_TIMEOUT}s"
                                    self.get_logger().error(f"⏱️ {msg}")
                                    _result["error"] = msg
                                    return
                                last_chunk_time = current_time

                                if hasattr(chunk.choices[0].delta, 'tool_calls') and chunk.choices[0].delta.tool_calls:
                                    for tc_chunk in chunk.choices[0].delta.tool_calls:
                                        idx = tc_chunk.index
                                        if idx not in tool_calls_accumulator:
                                            tool_calls_accumulator[idx] = {
                                                'id': '', 'type': 'function',
                                                'function': {'name': '', 'arguments': ''},
                                            }
                                        if tc_chunk.id:
                                            tool_calls_accumulator[idx]['id'] = tc_chunk.id
                                        if hasattr(tc_chunk, 'function'):
                                            if tc_chunk.function.name:
                                                tool_calls_accumulator[idx]['function']['name'] = tc_chunk.function.name
                                            if tc_chunk.function.arguments:
                                                tool_calls_accumulator[idx]['function']['arguments'] += tc_chunk.function.arguments

                                if chunk.choices[0].delta.content:
                                    token = chunk.choices[0].delta.content
                                    full_response += token
                                    current_chunk += token
                                    for char in token:
                                        if char == "{":
                                            brace_count += 1
                                            in_json = True
                                        elif char == "}":
                                            brace_count -= 1
                                    if in_json and brace_count == 0:
                                        try:
                                            chunk_data = json.loads(current_chunk.strip())
                                            if "ssml" in chunk_data:
                                                ssml_with_accents = self.accent_replacer.add_accents(chunk_data["ssml"])
                                                chunk_data["ssml"] = ssml_with_accents
                                                chunk_data["dialogue_id"] = dialogue_id
                                                chunk_count += 1
                                                response_msg = String()
                                                response_msg.data = json.dumps(chunk_data, ensure_ascii=False)
                                                if _my_gen != self._stream_generation:
                                                    self.get_logger().warning(f"🚫 Стейл rec-поток (gen {_my_gen}) — chunk отброшен")
                                                    return
                                                self.response_pub.publish(response_msg)
                                        except json.JSONDecodeError:
                                            pass
                                        current_chunk = ""
                                        in_json = False
                                        brace_count = 0

                                if chunk.choices[0].finish_reason:
                                    finish_reason = chunk.choices[0].finish_reason
                                    self.get_logger().info(f"🏁 Stream завершён: {finish_reason}")
                                    if hasattr(chunk, 'usage') and chunk.usage:
                                        usage = chunk.usage
                                        self.get_logger().info(
                                            f"📊 Token usage (iter {iteration}): "
                                            f"input={getattr(usage,'prompt_tokens',0)}, "
                                            f"output={getattr(usage,'completion_tokens',0)}, "
                                            f"total={getattr(usage,'total_tokens',0)}"
                                        )
                                    if finish_reason == 'tool_calls' and tool_calls_accumulator:
                                        _result["tool_calls"] = list(tool_calls_accumulator.values())
                                    break

                            _result["full_response"] = full_response
                            _result["chunk_count"] = chunk_count

                            if chunk_count == 0 and len(full_response) > 0:
                                text = full_response.strip()
                                chunk_data = {
                                    "chunk": "end",
                                    "ssml": f"<speak>{text}</speak>",
                                    "emotion": "neutral",
                                    "dialogue_id": dialogue_id,
                                    "speech_id": str(uuid.uuid4()),
                                }
                                response_msg = String()
                                response_msg.data = json.dumps(chunk_data, ensure_ascii=False)
                                if _my_gen != self._stream_generation:
                                    self.get_logger().warning(f"🚫 Стейл rec-поток plain-text (gen {_my_gen}) — отброшен")
                                    return
                                self.response_pub.publish(response_msg)
                                _result["chunk_count"] = 1
                                self.get_logger().info("🔊 Plain text обёрнут в SSML")

                        except Exception as e:
                            import traceback
                            self.get_logger().error(f"❌ Exception в _do_recursive_streaming: {e}")
                            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
                            _result["error"] = str(e)

                    # ── Запускаем в отдельном потоке с timeout ────────────────────
                    # ВАЖНО: НЕ используем `with ThreadPoolExecutor` — см. _ask_llm_streaming
                    _recursive_executor = ThreadPoolExecutor(max_workers=1)
                    future = _recursive_executor.submit(_do_recursive_streaming)
                    future_timeout = False
                    try:
                        future.result(timeout=60.0)
                    except FuturesTimeoutError as e:
                        self.get_logger().error(f"⏱️ future.result timeout (итерация {iteration}, попытка {attempt}): {e}")
                        future_timeout = True
                    finally:
                        _recursive_executor.shutdown(wait=False)  # поток умрёт по httpx timeout (~60s)

                    # Проверяем ошибку (внутренняя или внешняя)
                    if future_timeout or recursive_result["error"]:
                        if attempt == 0:
                            continue  # retry
                        # retry тоже не помог
                        self.get_logger().error(f"⏱️ TIMEOUT после retry (итерация {iteration})")
                        self._speak_simple("Извините, я слишком долго думал", show_error_animation=True)
                        return

                    succeeded = True
                    break  # успех, выходим из retry-loop

                if not succeeded:
                    return  # уже обработано выше (speak_simple + return)

                # ── Следующая итерация если LLM снова запросил инструменты ────────
                if recursive_result.get("tool_calls"):
                    new_tool_calls = recursive_result["tool_calls"]
                    self.get_logger().info(
                        f"🔧 LLM запросил {len(new_tool_calls)} инструментов — продолжаю цикл"
                    )
                    new_tool_results = self._execute_tool_calls(new_tool_calls, messages)
                    if new_tool_results is None:
                        self.get_logger().error("❌ Ошибка выполнения инструментов")
                        return
                    current_tool_calls = new_tool_calls
                    current_tool_results = new_tool_results
                    continue  # следующая итерация while, без рекурсии

                # ── Финальный ответ (нет tool_calls) ─────────────────────────────
                full_response = recursive_result["full_response"]
                chunk_count = recursive_result["chunk_count"]
                if full_response:
                    self.get_logger().info(f"📝 LLM финальный ответ: {full_response[:100]}...")
                    self.conversation_history.add_assistant_message(full_response)
                    if self.voice_memory is not None:
                        self.voice_memory.save_turn("assistant", full_response)
                    self.get_logger().info(f"✅ Агентный диалог завершён ({chunk_count} chunks, итераций: {iteration})")
                return  # Done

        except Exception as e:
            import traceback
            self.get_logger().error(f"❌ Ошибка в агентном цикле: {e}")
            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            self._speak_simple("Извините, произошла ошибка", show_error_animation=True)

        finally:
            # Единственный finally — сбрасываем флаги ровно один раз
            self.llm_processing = False
            if not self._listen_response_waiting:
                self.dialogue_in_progress = False
            self._listen_response_waiting = False


def main(args=None):
    rclpy.init(args=args)
    node = DialogueNode()

    # Используем MultiThreadedExecutor чтобы on_result callbacks могли обрабатываться
    # параллельно с execute_tool_call_sync ожиданием (через threading.Event)
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
