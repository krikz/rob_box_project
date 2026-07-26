# 🎤 Silero TTS v5 - Краткая справка

**Версия:** 5.0  
**Дата обновления:** 2025-11-14  
**Проект:** Rob Box (РОББОКС)

---

## ⚡ Быстрый старт

### Проверка версии

```bash
# В логах voice-assistant должно быть:
docker logs voice-assistant | grep "Silero"
# Ожидается: "✅ Silero TTS v5 загружен..."
```

### Основные параметры

```yaml
# Конфигурация в voice_assistant.yaml
tts_node:
  # Голоса v5
  silero_speaker: "baya"  # aidar, baya, kseniya, xenia, eugene (NEW!)
  silero_sample_rate: 48000  # Повышено с 24000
  
  # Новые флаги v5
  silero_put_accent: true       # Ударения в словах
  silero_put_yo: true           # Автоматическая ё
  silero_put_stress_homo: true  # Ударения в омографах ⭐
  silero_put_yo_homo: true      # Ударения в омографах с ё
```

### Смена голоса (runtime)

```bash
# Список доступных голосов v5:
# - aidar (нейтральный М)
# - baya (спокойный М) - по умолчанию
# - kseniya (нейтральный Ж)
# - xenia (энергичный Ж)
# - eugene (новый М) ⭐

# Смена голоса
ros2 param set /tts_node silero_speaker eugene
```

---

## 📊 Производительность

### Скорость генерации (Raspberry Pi 5)

| Метрика | v4 | v5 | Улучшение |
|---------|----|----|-----------|
| CPU, 1 поток | ~15-20 с/с | ~25-30 с/с | **1.5-1.7x** |
| CPU, 4 потока | ~25-30 с/с | **40-50 с/с** | **1.6-2.0x** |
| CPU usage | 60-80% | 40-60% | **-30%** |

> **с/с** = секунд синтезированного аудио в секунду

### Качество

- ✅ Правильные ударения в омографах
- ✅ Улучшенная интонация
- ✅ Меньше артефактов на длинных фразах
- ✅ Более стабильная генерация

---

## 🎯 Новые возможности v5

### Расстановка ударений в омографах

**Что это:**
Автоматическое определение правильного ударения в словах с одинаковым написанием.

**Примеры:**
- `замОк` (дверной) vs `зАмок` (здание)
- `готОв` (прилагательное) vs `гОтов` (племя)
- `мукА` (продукт) vs `мУка` (страдание)

**Включить:**
```yaml
silero_put_stress_homo: true  # По умолчанию включено
```

**Тест:**
```bash
ros2 topic pub --once /voice/tts/request std_msgs/msg/String \
  "{data: 'Я живу в замке. Открой замок. Я готов. Из готов пришли послы.'}"
```

### Новый голос eugene

```bash
# Попробовать новый голос
ros2 param set /tts_node silero_speaker eugene
ros2 topic pub --once /voice/tts/request std_msgs/msg/String \
  "{data: 'Привет! Меня зовут Евгений, новый голос в Silero версии пять.'}"
```

---

## 🔧 Troubleshooting

### Проблема: Модель не загружается

**Симптомы:**
```
❌ Ошибка загрузки Silero: ...
```

**Решение:**
```bash
# 1. Проверить наличие модели
docker exec voice-assistant ls -lh /models/silero_v5_ru.pt

# 2. Если отсутствует - пересобрать образ
cd ~/rob_box_project/docker/vision
docker-compose build voice-assistant

# 3. Или скачать вручную
docker exec voice-assistant wget -O /models/silero_v5_ru.pt \
  https://models.silero.ai/models/tts/ru/v5_ru.pt
```

### Проблема: Медленная генерация

**Симптомы:**
Скорость < 30 с/с на Raspberry Pi 5.

**Диагностика:**
```bash
# Проверить CPU threads
docker exec voice-assistant python3 -c "import torch; print(torch.get_num_threads())"
# Должно быть: 4
```

**Решение:**
Настройки в коде уже оптимизированы для ARM64:
```python
torch.set_num_threads(4)  # Использовать все 4 ядра
torch.set_grad_enabled(False)  # Отключить градиенты
```

### Проблема: Неправильные ударения

**Симптомы:**
Омографы произносятся неправильно.

**Решение:**
```bash
# Проверить флаги
ros2 param get /tts_node silero_put_stress_homo
# Должно быть: true

# Если false - установить
ros2 param set /tts_node silero_put_stress_homo true
```

---

## 📚 Документация

**Подробные документы:**

1. **Анализ v5:** `docs/reports/SILERO_V5_ANALYSIS.md`
   - Полное сравнение v4 vs v5
   - Технические детали
   - Оценка рисков и ROI

2. **Руководство по миграции:** `docs/reports/SILERO_V5_MIGRATION_GUIDE.md`
   - Все изменения в коде
   - Процедуры тестирования
   - Инструкции по откату

3. **Итоговый отчет:** `docs/reports/SILERO_V5_IMPLEMENTATION_SUMMARY.md`
   - Статистика изменений
   - Метрики успеха

---

## 🧪 Тестовые команды

### Базовый тест
```bash
ros2 topic pub --once /voice/tts/request std_msgs/msg/String \
  "{data: 'Привет, робот! Проверка Silero версии пять.'}"
```

### Тест омографов
```bash
ros2 topic pub --once /voice/tts/request std_msgs/msg/String \
  "{data: 'Меня зовут Лева Королев. Я из готов. И я уже готов открыть все ваши замки любой сложности!'}"
```

### Тест производительности
```bash
# Длинный текст (~30 секунд аудио)
ros2 topic pub --once /voice/tts/request std_msgs/msg/String \
  "{data: 'Робот РОББОКС - это автономный колёсный ровер, построенный на ROS 2 kilted. Он оснащён камерой OAK-D для зрения, LiDAR сканером для навигации, и голосовым ассистентом на основе Silero TTS версии пять. Новая версия в полтора-два раза быстрее предыдущей, что критично для работы на Raspberry Pi. Кроме того, она умеет правильно расставлять ударения в омографах, таких как замок и замок, готов и готов, мука и мука.'}"
```

### Тест всех голосов
```bash
# Функция для теста всех голосов
for voice in aidar baya kseniya xenia eugene; do
  echo "Тестирование голоса: $voice"
  ros2 param set /tts_node silero_speaker $voice
  ros2 topic pub --once /voice/tts/request std_msgs/msg/String \
    "{data: 'Меня зовут $voice. Я голос Silero версии пять.'}"
  sleep 5
done
```

---

## 🔄 Откат на v4 (если нужно)

### Быстрый откат

```bash
# 1. Git revert
git revert a62b19b  # commit hash обновления на v5
git push origin develop

# 2. CI/CD автоматически соберет v4

# 3. Обновить на Pi
ssh ros2@10.1.1.21
cd ~/rob_box_project/docker/vision
./scripts/update_and_restart.sh
```

---

## 📊 Мониторинг

### Метрики для отслеживания

```bash
# CPU usage
docker stats voice-assistant --no-stream

# Логи генерации
docker logs voice-assistant | grep "Silero v5 fallback успешен"

# Скорость генерации (вычислить из логов)
# Ищем: "{N} samples @ {SR} Hz"
# Скорость = (N / SR) / время_генерации
```

### Ожидаемые значения

| Метрика | Норма |
|---------|-------|
| CPU usage | 40-60% при синтезе |
| Memory | 400-500 MB |
| Скорость | 40-50 с/с (CPU, 4 потока) |
| Latency | < 200ms для 1 сек аудио |

---

## ✅ Чеклист после обновления

- [ ] Модель v5 загружается без ошибок
- [ ] Базовый тест пройден (генерация работает)
- [ ] Скорость >= 40 с/с (для Pi 5)
- [ ] Омографы произносятся правильно
- [ ] Новый голос eugene работает
- [ ] CPU usage приемлемый (< 80%)
- [ ] Память в норме (< 600 MB)
- [ ] Нет ошибок в логах

---

## 🆘 Поддержка

**Документация:**
- [Полный анализ](./SILERO_V5_ANALYSIS.md)
- [Руководство по миграции](./SILERO_V5_MIGRATION_GUIDE.md)
- [Итоговый отчет](./SILERO_V5_IMPLEMENTATION_SUMMARY.md)

**Официальные ресурсы:**
- [Silero GitHub](https://github.com/snakers4/silero-models)
- [Статья на Habr](https://habr.com/ru/articles/XXX/)

**В проекте:**
- GitHub Issues: создать issue с тегом `voice` и `tts`
- Логи: `docker logs voice-assistant`

---

**Обновлено:** 2025-11-14  
**Версия справки:** 1.0  
**Статус:** ✅ Актуально
