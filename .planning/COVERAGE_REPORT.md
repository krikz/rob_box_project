# Отчёт покрытия тестами — Rob Box

**Дата:** 2026-05-15
**Инструмент:** pytest-cov (coverage.py 6.2.5)
**Источник данных:** htmlcov/ (исторический прогон, 23 файла) + coverage.json (1 файл, dialogue_node.py)

> ⚠️ **Важно о данных:** Попытка перезапустить pytest --cov прервалась с `ModuleNotFoundError: No module named 'rob_box_voice'` — ROS 2 пакеты не установлены как Python-пакеты на dev-машине (только в ROS 2 workspace через colcon). Данные ниже взяты из последнего успешного прогона (htmlcov/) + coverage.json. Дата последнего успешного прогона: зафиксирована в htmlcov/status.json.

---

## Сводная таблица по модулям

| Модуль | Покрытие | Статус | Причина низкого покрытия |
|--------|----------|--------|--------------------------|
| rob_box_voice/audio_node.py | 0% | ❌ | ROS 2 нода; требует rclpy + audio hardware для init |
| rob_box_voice/dialogue_node.py | 0% (htmlcov) / 13.4% (coverage.json) | ❌ | Монолит 2040 строк; test_dialogue_node.py содержит 13+ пустых методов (FA-5) |
| rob_box_voice/stt_node.py | 0% | ❌ | ROS 2 нода; требует ReSpeaker hardware |
| rob_box_voice/tts_node.py | 0% | ❌ | ROS 2 нода; требует audio playback pipeline |
| rob_box_perception/vision_stub_node.py | 0% | ❌ | Stub-нода; нет логики для тестирования |
| rob_box_perception/utils/long_term_memory.py | 0% | ❌ | Нет тестов написано (модуль тестируем без hardware!) |
| rob_box_voice/utils/respeaker_interface.py | 20% | ⚠️ | Hardware dependency (ReSpeaker USB) |
| rob_box_voice/utils/audio_utils.py | 21% | ⚠️ | Hardware dependency (pyaudio) |
| rob_box_voice/audio_playback_manager.py | 24% | ⚠️ | Зависит от audio hardware |
| rob_box_voice/command_node.py | 28% | ⚠️ | Stub-методы с `pass`; ROS callbacks трудно мокировать |
| rob_box_perception/reflection_node.py | 28% | ⚠️ | Сложная инициализация с ROS параметрами |
| rob_box_perception/utils/node_monitor.py | 37% | ⚠️ | Частичное покрытие; остальное требует запущенного ROS |
| rob_box_perception/utils/time_provider.py | 39% | ⚠️ | Clock-зависимый код |
| rob_box_perception/context_aggregator_node.py | 43% | ⚠️ | ROS subscriber callbacks не покрыты |
| rob_box_voice/sound_node.py | 53% | ✅ | |
| rob_box_voice/led_node.py | 62% | ✅ | |
| rob_box_perception/health_monitor.py | 69% | ✅ | |
| rob_box_perception/utils/internet_monitor.py | 71% | ✅ | |
| rob_box_perception/startup_greeting_node.py | 77% | ✅ | |

**Модулей ниже 50%:** 14 из 19 (74%)
**Модулей с 0% покрытием:** 6

---

## Анализ причин низкого покрытия

### 1. ROS 2 hardware dependency (audio_node, stt_node, tts_node, respeaker_interface)

ROS 2 ноды вызывают `super().__init__()` в конструкторе, что требует запущенного rclpy контекста. Без `rclpy.init()` тест падает при попытке создать объект. Unit-тесты этих нод невозможны без мокирования rclpy на уровне инициализации.

Дополнительно: `audio_node`, `stt_node`, `tts_node` требуют физического оборудования (ReSpeaker USB, PyAudio output device). Без железа инициализация падает на уровне Python до ROS.

**Путь к улучшению (Milestone 3):** Инъекция зависимостей через constructor injection; отделение бизнес-логики от ROS glue. Тестировать pure-python части изолированно.

### 2. Монолитная структура dialogue_node.py (TD-1, FA-5)

2040 строк в одном классе создают 74 метода с взаимными зависимостями. Тест-файл `test_dialogue_node.py` содержит 13+ методов-заглушек:

```python
def test_handle_voice_input(self):
    pass  # TODO: implement

def test_agent_run(self):
    pass  # TODO: implement
```

Эти методы **не проверяют ничего** и не влияют на coverage негативно, но создают ложное ощущение тест-инфраструктуры. Зафиксировано как FA-5.

Расхождение coverage.json (13.4%) vs htmlcov (0%): coverage.json содержит данные одного запуска `pytest --cov` с одним конкретным тестом; htmlcov — результат более широкого прогона по всем тестам, который показывает 0% для dialogue_node.py.

**Путь к улучшению (Milestone 3):** Декомпозиция на 6 компонентов (AgentFactory, AgentRunner, ConversationHistory, DjModeManager, VoiceAssistantConfig, EventProfileLoader) — см. DIALOGUE_NODE_REFACTORING.md.

### 3. Отсутствие тестов (long_term_memory)

`long_term_memory.py` — полноценный модуль с логикой кэширования и персистентности без единого теста. В отличие от ROS 2 нод, этот модуль **не зависит от hardware или rclpy** и может быть покрыт unit-тестами прямо сейчас.

**Путь к улучшению (Milestone 2):** Написать unit-тесты; модуль тестируем без ROS окружения.

### 4. Невозможность запустить pytest с покрытием на dev-машине

ROS 2 пакеты устанавливаются только через `colcon build` в ROS workspace. На dev-машине без инициализированного `source install/setup.bash` модули недоступны как Python пакеты. Данный отчёт основан на исторических данных htmlcov/.

**Путь к улучшению:** Добавить `python setup.py develop` или `pip install -e .` в CI pipeline для dev-машины. Или запускать coverage-тесты только в Docker-контейнере с ROS 2.

---

## Пустые тест-методы (FA-5)

`src/rob_box_voice/test/test_dialogue_node.py` содержит 13+ методов-заглушек:

| Метод | Статус |
|-------|--------|
| test_handle_voice_input | `pass` only |
| test_agent_run | `pass` only |
| test_barge_in | `pass` only |
| test_dj_mode_activation | `pass` only |
| test_dj_mode_deactivation | `pass` only |
| test_handle_tts_complete | `pass` only |
| test_audio_playback_complete | `pass` only |
| test_sound_event | `pass` only |
| test_llm_provider_switch | `pass` only |
| test_faq_mode | `pass` only |
| test_event_mode | `pass` only |
| test_context_trim | `pass` only |
| test_history_output_truncation | `pass` only |

Эти методы дают **ложное ощущение тест-покрытия**. Рекомендация: маркировать `@pytest.mark.skip(reason="Not implemented")` до момента реализации, чтобы coverage отображал честную картину.

---

## Рекомендации по улучшению покрытия

| Приоритет | Модуль | Что сделать | Milestone |
|-----------|--------|-------------|-----------|
| HIGH | long_term_memory.py | Написать unit-тесты (не требует hardware или ROS) | M2 |
| HIGH | dialogue_node.py | Декомпозиция + тесты для изолированных компонентов | M3 |
| HIGH | test_dialogue_node.py | Убрать пустые методы или маркировать @skip | M2 |
| MEDIUM | command_node.py | Мокировать ROS subscribers; тестировать parse логику | M3 |
| MEDIUM | reflection_node.py | Тестировать без ROS через мокирование node.get_parameter() | M3 |
| LOW | audio_node, stt_node, tts_node | Инъекция зависимостей → unit-тесты без hardware | M3 |

---

## Инфраструктура для запуска coverage на dev-машине

Для воспроизводимого запуска coverage без полного ROS 2 stack:

```bash
# Вариант 1: source ROS 2 workspace
source /home/ros2/rob_box_project/install/setup.bash
python3 -m pytest src/rob_box_voice/test/ ... --cov=...

# Вариант 2: PYTHONPATH (dev-машина без colcon)
PYTHONPATH=/home/ros2/rob_box_project/src/rob_box_voice:\
          /home/ros2/rob_box_project/src/rob_box_perception:\
          /home/ros2/rob_box_project/src/rob_box_mcp_tools \
  python3 -m pytest src/ --cov=src/ --cov-report=html:htmlcov/

# Вариант 3: Docker (с ROS 2 + coverage pre-installed)
docker run --rm -v $(pwd):/workspace ros2-zenoh-base \
  bash -c "cd /workspace && python3 -m pytest ... --cov=..."
```

---
*Отчёт создан: Phase 3 Milestone 1*
*Следующий запуск: начало Milestone 2 (в Docker с source install/setup.bash)*
