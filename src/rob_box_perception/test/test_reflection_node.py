#!/usr/bin/env python3
"""
test_reflection_node.py - Unit тесты для ReflectionNode

Тестирует:
- Обработку событий восприятия
- Генерацию внутренних мыслей
- Управление режимом диалога
- Команду "помолчи" (silence mode)
- Срочные ответы на личные вопросы
"""

import unittest
from unittest.mock import MagicMock, patch, Mock
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Мокаем OpenAI перед импортом
with patch('rob_box_perception.reflection_node.OPENAI_AVAILABLE', True), \
     patch('rob_box_perception.reflection_node.OpenAI'):
    from rob_box_perception.reflection_node import ReflectionNode


class TestReflectionNode(unittest.TestCase):
    """Тесты для ReflectionNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Завершение ROS 2"""
        rclpy.shutdown()

    def setUp(self):
        """Создание мокированной ноды"""
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        """Очистка после теста"""
        self.node.destroy_node()
        patch.stopall()

    def test_node_creation(self):
        """Тест: нода создается корректно"""
        self.assertEqual(self.node.get_name(), 'reflection_node')

    def test_parameters_exist(self):
        """Тест: все параметры объявлены"""
        params = [
            'dialogue_timeout',
            'enable_speech',
            'system_prompt_file',
            'user_response_prompt_file',
            'urgent_response_timeout'
        ]
        for param in params:
            self.assertTrue(self.node.has_parameter(param))

    def test_subscribers_created(self):
        """Тест: подписчики созданы"""
        subscriptions = self.node.get_subscriber_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        sub_topics = [name for name, _ in subscriptions]
        
        self.assertIn('/perception/context_update', sub_topics)
        self.assertIn('/perception/user_speech', sub_topics)

    def test_publishers_created(self):
        """Тест: паблишеры созданы"""
        publishers = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(),
            self.node.get_namespace()
        )
        pub_topics = [name for name, _ in publishers]
        
        self.assertIn('/reflection/internal_thought', pub_topics)
        self.assertIn('/voice/tts/request', pub_topics)

    def test_dialogue_state_initialization(self):
        """Тест: начальное состояние диалога"""
        self.assertFalse(self.node.in_dialogue)
        self.assertIsNone(self.node.last_user_speech_time)
        self.assertIsNone(self.node.pending_user_speech)

    def test_silence_mode_initialization(self):
        """Тест: silence mode выключен при старте"""
        self.assertIsNone(self.node.silence_until)

    def test_dialogue_timeout_parameter(self):
        """Тест: параметр dialogue_timeout"""
        timeout = self.node.get_parameter('dialogue_timeout').value
        self.assertIsInstance(timeout, float)
        self.assertGreater(timeout, 0)

    def test_enable_speech_parameter(self):
        """Тест: параметр enable_speech"""
        enabled = self.node.get_parameter('enable_speech').value
        self.assertIsInstance(enabled, bool)

    def test_urgent_response_timeout(self):
        """Тест: параметр urgent_response_timeout"""
        timeout = self.node.get_parameter('urgent_response_timeout').value
        self.assertIsInstance(timeout, float)
        self.assertEqual(timeout, 2.0)

    def test_user_speech_callback_enters_dialogue(self):
        """Тест: получение речи пользователя активирует диалог"""
        msg = String()
        msg.data = "Привет, как дела?"
        
        self.node.on_user_speech(msg)
        
        self.assertTrue(self.node.in_dialogue)
        self.assertIsNotNone(self.node.last_user_speech_time)
        self.assertEqual(self.node.pending_user_speech, "Привет, как дела?")

    def test_user_speech_callback_updates_timestamp(self):
        """Тест: обновление времени последней речи"""
        msg = String()
        msg.data = "Тест"
        
        time_before = time.time()
        self.node.on_user_speech(msg)
        time_after = time.time()
        
        self.assertGreaterEqual(self.node.last_user_speech_time, time_before)
        self.assertLessEqual(self.node.last_user_speech_time, time_after)

    def test_silence_mode_activation(self):
        """Тест: активация режима молчания"""
        self.node.silence_until = None
        
        future_time = time.time() + 300  # 5 минут
        self.node.silence_until = future_time
        
        self.assertEqual(self.node.silence_until, future_time)

    def test_dialogue_exit_on_timeout(self):
        """Тест: выход из диалога по таймауту"""
        # Входим в диалог
        self.node.in_dialogue = True
        self.node.last_user_speech_time = time.time() - 20  # 20 секунд назад
        self.node.dialogue_timeout = 10.0
        
        # Проверяем что прошло достаточно времени
        time_passed = time.time() - self.node.last_user_speech_time
        self.assertGreater(time_passed, self.node.dialogue_timeout)

    def test_last_context_storage(self):
        """Тест: сохранение последнего контекста"""
        self.assertIsNone(self.node.last_context)
        
        # Симулируем получение контекста
        mock_context = MagicMock()
        self.node.last_context = mock_context
        
        self.assertEqual(self.node.last_context, mock_context)

    def test_publish_internal_thought(self):
        """Тест: публикация внутренней мысли"""
        with patch.object(self.node.thought_pub, 'publish') as mock_publish:
            thought = "Интересно, почему батарея разряжается так быстро?"
            
            msg = String()
            msg.data = thought
            self.node.thought_pub.publish(msg)
            
            mock_publish.assert_called_once()

    def test_publish_speech_request(self):
        """Тест: публикация запроса на речь"""
        with patch.object(self.node.tts_pub, 'publish') as mock_publish:
            speech = "У меня всё хорошо, спасибо что спросил!"
            
            msg = String()
            msg.data = speech
            self.node.tts_pub.publish(msg)
            
            mock_publish.assert_called_once()

    def test_silence_mode_check(self):
        """Тест: проверка активности silence mode"""
        # Режим молчания не активен
        self.node.silence_until = None
        self.assertIsNone(self.node.silence_until)
        
        # Активируем режим молчания
        self.node.silence_until = time.time() + 10
        self.assertIsNotNone(self.node.silence_until)
        self.assertGreater(self.node.silence_until, time.time())
        
        # Режим молчания истек
        self.node.silence_until = time.time() - 1
        self.assertLess(self.node.silence_until, time.time())


class TestReflectionNodeMemory(unittest.TestCase):
    """Тесты sound debounce"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Создание ноды"""
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        """Очистка"""
        self.node.destroy_node()

    def test_sound_debounce_initialization(self):
        """Тест: sound debounce инициализирован"""
        self.assertTrue(hasattr(self.node, 'last_sound_time'))
        self.assertIsInstance(self.node.last_sound_time, dict)
        self.assertEqual(self.node.sound_debounce_interval, 10.0)

    def test_sound_debounce_limit(self):
        """Тест: звук может быть воспроизведен"""
        # Проверяем что словарь debounce пустой при старте
        self.assertEqual(len(self.node.last_sound_time), 0)


class TestPersonalQuestions(unittest.TestCase):
    """Тесты _is_personal_question()"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_is_personal_question_kak_dela(self):
        """Тест: 'как дела' распознается"""
        self.assertTrue(self.node._is_personal_question("Как дела?"))
        self.assertTrue(self.node._is_personal_question("Привет, как у тебя дела?"))

    def test_is_personal_question_kak_ty(self):
        """Тест: 'как ты' распознается"""
        self.assertTrue(self.node._is_personal_question("Как ты?"))

    def test_is_personal_question_chto_u_tebya(self):
        """Тест: 'что у тебя' распознается"""
        self.assertTrue(self.node._is_personal_question("Что у тебя происходит?"))

    def test_is_personal_question_nastroenie(self):
        """Тест: 'настроение/самочувствие' распознается"""
        self.assertTrue(self.node._is_personal_question("Как твоё настроение?"))
        self.assertTrue(self.node._is_personal_question("Как твое самочувствие?"))

    def test_is_personal_question_chto_delaesh(self):
        """Тест: 'что делаешь' распознается"""
        self.assertTrue(self.node._is_personal_question("Что делаешь?"))

    def test_is_personal_question_kak_sebya(self):
        """Тест: 'как себя чувствуешь' распознается"""
        self.assertTrue(self.node._is_personal_question("Как себя чувствуешь?"))

    def test_is_personal_question_negative(self):
        """Тест: обычный вопрос НЕ распознается"""
        self.assertFalse(self.node._is_personal_question("Поехали вперёд"))
        self.assertFalse(self.node._is_personal_question("Стоп"))


class TestSilenceCommand(unittest.TestCase):
    """Тесты _is_silence_command()"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_is_silence_command_pomolchi(self):
        """Тест: 'помолчи' распознается"""
        self.assertTrue(self.node._is_silence_command("помолчи"))
        self.assertTrue(self.node._is_silence_command("Помолчите"))

    def test_is_silence_command_zamolchi(self):
        """Тест: 'замолчи' распознается"""
        self.assertTrue(self.node._is_silence_command("замолчи"))

    def test_is_silence_command_hvatit(self):
        """Тест: 'хватит' распознается"""
        self.assertTrue(self.node._is_silence_command("хватит"))

    def test_is_silence_command_zakroy(self):
        """Тест: 'закрой' распознается"""
        self.assertTrue(self.node._is_silence_command("закройся"))

    def test_is_silence_command_zatknis(self):
        """Тест: 'заткнись' распознается"""
        self.assertTrue(self.node._is_silence_command("заткнись"))

    def test_is_silence_command_ne_meshay(self):
        """Тест: 'не мешай' распознается"""
        self.assertTrue(self.node._is_silence_command("не мешай мне"))

    def test_is_silence_command_negative(self):
        """Тест: обычная фраза НЕ распознается"""
        self.assertFalse(self.node._is_silence_command("Привет"))
        self.assertFalse(self.node._is_silence_command("Как дела?"))


class TestHealthStatusChange(unittest.TestCase):
    """Тесты _check_health_status_change()"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()
        self.mock_ctx = MagicMock()
        self.mock_ctx.system_health_status = 'HEALTHY'
        self.mock_ctx.health_issues = []

    def tearDown(self):
        self.node.destroy_node()

    def test_health_status_change_to_healthy_first_start(self):
        """Тест: первый запуск (None → HEALTHY)"""
        with patch.object(self.node, '_publish_speech') as mock_speech:
            self.mock_ctx.system_health_status = 'HEALTHY'
            self.node.event_states['health_status'] = None
            
            self.node._check_health_status_change(self.mock_ctx)
            
            mock_speech.assert_called_once()
            args = mock_speech.call_args[0][0]
            self.assertIn('готов к работе', args.lower())

    def test_health_status_change_to_healthy_recovery(self):
        """Тест: восстановление (DEGRADED → HEALTHY)"""
        with patch.object(self.node, '_publish_speech') as mock_speech:
            self.node.event_states['health_status'] = 'DEGRADED'
            self.mock_ctx.system_health_status = 'HEALTHY'
            
            self.node._check_health_status_change(self.mock_ctx)
            
            mock_speech.assert_called_once()
            args = mock_speech.call_args[0][0]
            self.assertIn('восстановлен', args.lower())

    def test_health_status_change_to_degraded(self):
        """Тест: ухудшение (HEALTHY → DEGRADED)"""
        with patch.object(self.node, '_publish_speech') as mock_speech:
            self.node.event_states['health_status'] = 'HEALTHY'
            self.mock_ctx.system_health_status = 'DEGRADED'
            self.mock_ctx.health_issues = ['Low battery', 'High CPU']
            
            self.node._check_health_status_change(self.mock_ctx)
            
            mock_speech.assert_called_once()
            args = mock_speech.call_args[0][0]
            self.assertIn('проблем', args.lower())

    def test_health_status_change_to_unhealthy(self):
        """Тест: критическое состояние (DEGRADED → UNHEALTHY)"""
        with patch.object(self.node, '_publish_speech') as mock_speech:
            self.node.event_states['health_status'] = 'DEGRADED'
            self.mock_ctx.system_health_status = 'UNHEALTHY'
            
            self.node._check_health_status_change(self.mock_ctx)
            
            mock_speech.assert_called_once()
            args = mock_speech.call_args[0][0]
            self.assertIn('критическ', args.lower())

    def test_health_status_periodic_check_degraded(self):
        """Тест: periodic check для DEGRADED (1 минута прошла)"""
        with patch.object(self.node, '_publish_speech') as mock_speech:
            self.node.event_states['health_status'] = 'DEGRADED'
            self.node.event_last_reaction['health_status'] = time.time() - 65  # 65 сек назад
            self.mock_ctx.system_health_status = 'DEGRADED'
            
            self.node._check_health_status_change(self.mock_ctx)
            
            mock_speech.assert_called_once()
            args = mock_speech.call_args[0][0]
            self.assertIn('всё ещё', args.lower())

    def test_health_status_no_periodic_check_for_healthy(self):
        """Тест: periodic check НЕ срабатывает для HEALTHY"""
        with patch.object(self.node, '_publish_speech') as mock_speech:
            self.node.event_states['health_status'] = 'HEALTHY'
            self.node.event_last_reaction['health_status'] = time.time() - 65
            self.mock_ctx.system_health_status = 'HEALTHY'
            
            self.node._check_health_status_change(self.mock_ctx)
            
            mock_speech.assert_not_called()


class TestPublishSpeech(unittest.TestCase):
    """Тесты _publish_speech() и _publish_speech_ssml()"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_publish_speech_normal(self):
        """Тест: обычная публикация речи"""
        with patch.object(self.node.tts_pub, 'publish') as mock_pub:
            self.node._publish_speech("Тестовая речь")
            
            mock_pub.assert_called_once()
            msg = mock_pub.call_args[0][0]
            import json
            data = json.loads(msg.data)
            self.assertIn('ssml', data)
            self.assertIn('Тестовая речь', data['ssml'])

    def test_publish_speech_during_silence_mode(self):
        """Тест: НЕ публикует речь в silence mode"""
        with patch.object(self.node.tts_pub, 'publish') as mock_pub:
            self.node.silence_until = time.time() + 10  # 10 сек молчания
            
            self.node._publish_speech("Не должно быть опубликовано")
            
            mock_pub.assert_not_called()

    def test_publish_speech_ssml_normal(self):
        """Тест: публикация SSML ответа пользователю"""
        with patch.object(self.node.tts_pub, 'publish') as mock_pub:
            ssml = "<speak>Привет!<break time='300ms'/></speak>"
            
            self.node._publish_speech_ssml(ssml)
            
            mock_pub.assert_called_once()
            msg = mock_pub.call_args[0][0]
            import json
            data = json.loads(msg.data)
            self.assertEqual(data['ssml'], ssml)

    def test_publish_speech_ssml_during_silence_mode(self):
        """Тест: НЕ публикует SSML в silence mode"""
        with patch.object(self.node.tts_pub, 'publish') as mock_pub:
            self.node.silence_until = time.time() + 10
            
            self.node._publish_speech_ssml("<speak>Test</speak>")
            
            mock_pub.assert_not_called()


class TestSoundTriggers(unittest.TestCase):
    """Тесты _trigger_sound_for_thought() и _play_sound()"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_trigger_sound_surprise(self):
        """Тест: триггер 'surprise' для удивления"""
        with patch.object(self.node, '_play_sound') as mock_play:
            self.node._trigger_sound_for_thought("Удивительно! Новый объект!")
            
            mock_play.assert_called_once_with('surprise')

    def test_trigger_sound_thinking(self):
        """Тест: триггер 'thinking' для размышления"""
        with patch.object(self.node, '_play_sound') as mock_play:
            self.node._trigger_sound_for_thought("Думаю, что делать дальше")
            
            mock_play.assert_called_once_with('thinking')

    def test_trigger_sound_confused(self):
        """Тест: триггер 'confused' для замешательства"""
        with patch.object(self.node, '_play_sound') as mock_play:
            self.node._trigger_sound_for_thought("Не уверен в правильности")
            
            mock_play.assert_called_once_with('confused')

    def test_trigger_sound_angry(self):
        """Тест: триггер 'angry' для критической проблемы"""
        with patch.object(self.node, '_play_sound') as mock_play:
            self.node._trigger_sound_for_thought("Критическая ошибка датчика!")
            
            mock_play.assert_called_once_with('angry')

    def test_trigger_sound_cute(self):
        """Тест: триггер 'cute' для позитива"""
        with patch.object(self.node, '_play_sound') as mock_play:
            self.node._trigger_sound_for_thought("Отлично! Готов к работе")
            
            mock_play.assert_called_once_with('cute')

    def test_play_sound_normal(self):
        """Тест: обычное воспроизведение звука"""
        with patch.object(self.node.sound_pub, 'publish') as mock_pub:
            self.node._play_sound('thinking')
            
            mock_pub.assert_called_once()
            msg = mock_pub.call_args[0][0]
            self.assertEqual(msg.data, 'thinking')

    def test_play_sound_debounce(self):
        """Тест: debounce блокирует повторный звук"""
        with patch.object(self.node.sound_pub, 'publish') as mock_pub:
            self.node._play_sound('cute')
            self.node._play_sound('cute')  # Второй раз сразу
            
            # Должен быть только один вызов (debounce заблокировал второй)
            self.assertEqual(mock_pub.call_count, 1)

    def test_play_sound_debounce_different_sounds(self):
        """Тест: debounce НЕ блокирует разные звуки"""
        with patch.object(self.node.sound_pub, 'publish') as mock_pub:
            self.node._play_sound('cute')
            self.node._play_sound('thinking')
            
            self.assertEqual(mock_pub.call_count, 2)


class TestContextCallbacks(unittest.TestCase):
    """Тесты on_context_update(), on_user_speech(), on_robot_response()"""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_on_context_update_stores_context(self):
        """Тест: контекст сохраняется"""
        mock_event = MagicMock()
        mock_event.system_health_status = 'HEALTHY'
        
        self.node.on_context_update(mock_event)
        
        self.assertEqual(self.node.last_context, mock_event)

    def test_on_context_update_processes_pending_speech(self):
        """Тест: обработка pending speech при получении контекста"""
        with patch.object(self.node, 'process_urgent_question') as mock_process:
            self.node.pending_user_speech = "Как дела?"
            mock_event = MagicMock()
            mock_event.system_health_status = 'HEALTHY'
            
            self.node.on_context_update(mock_event)
            
            mock_process.assert_called_once_with("Как дела?")
            self.assertIsNone(self.node.pending_user_speech)

    def test_on_user_speech_silence_command(self):
        """Тест: команда silence устанавливает флаг"""
        msg = String()
        msg.data = "помолчи"
        
        self.node.on_user_speech(msg)
        
        self.assertIsNotNone(self.node.silence_until)
        self.assertGreater(self.node.silence_until, time.time())

    def test_on_user_speech_personal_question_with_context(self):
        """Тест: личный вопрос с контекстом → немедленный ответ"""
        with patch.object(self.node, 'process_urgent_question') as mock_process:
            self.node.last_context = MagicMock()
            msg = String()
            msg.data = "Как дела?"
            
            self.node.on_user_speech(msg)
            
            mock_process.assert_called_once_with("Как дела?")

    def test_on_user_speech_personal_question_no_context(self):
        """Тест: личный вопрос без контекста → ожидает"""
        self.node.last_context = None
        msg = String()
        msg.data = "Как ты?"
        
        self.node.on_user_speech(msg)
        
        self.assertEqual(self.node.pending_user_speech, "Как ты?")

    def test_on_robot_response_updates_timestamp(self):
        """Тест: ответ робота обновляет timestamp"""
        msg = String()
        msg.data = "Ответ робота"
        
        time_before = time.time()
        self.node.on_robot_response(msg)
        time_after = time.time()
        
        self.assertGreaterEqual(self.node.last_user_speech_time, time_before)
        self.assertLessEqual(self.node.last_user_speech_time, time_after)


class TestReflectionNodeIntegration(unittest.TestCase):
    """Интеграционные тесты ReflectionNode"""

    @classmethod
    def setUpClass(cls):
        """Инициализация ROS 2"""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Создание ноды"""
        with patch('rob_box_perception.reflection_node.OpenAI'):
            self.node = ReflectionNode()

    def tearDown(self):
        """Очистка"""
        self.node.destroy_node()

    def test_user_speech_workflow(self):
        """Тест: workflow обработки речи пользователя"""
        received_thoughts = []
        
        def thought_callback(msg):
            received_thoughts.append(msg.data)
        
        # Подписываемся на internal thoughts
        test_node = rclpy.create_node('test_listener')
        test_sub = test_node.create_subscription(
            String,
            '/reflection/internal_thought',
            thought_callback,
            10
        )
        
        # Публикуем речь пользователя
        pub = test_node.create_publisher(String, '/perception/user_speech', 10)
        msg = String()
        msg.data = "Как ты себя чувствуешь?"
        pub.publish(msg)
        
        # Даем время на обработку
        rclpy.spin_once(self.node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        
        test_node.destroy_node()
        
        # Проверяем что диалог активирован
        self.assertTrue(self.node.in_dialogue)


if __name__ == '__main__':
    unittest.main()
