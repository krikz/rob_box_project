# Развёртывание усиленного исправления Zenoh Transport (2025-11-10)

## 📋 Краткая информация

**Дата:** 2025-11-10  
**Проблема:** Постоянные ошибки "Unable to push non droppable network message" и "Cannot find link 1"  
**Решение:** Усиленная конфигурация с приоритизацией критичных очередей  
**Файлы:** `docker/main/config/zenoh_router_config.json5`, `docker/vision/config/zenoh_router_config.json5`

---

## 🎯 Что изменилось

### Основные изменения

1. **Критичные очереди увеличены в 2 раза:**
   - `control`: 4 → **8 batches** (512 KB)
   - `real_time`: 4 → **8 batches** (512 KB)

2. **Приоритетная очередь увеличена на 50%:**
   - `data_high`: 4 → **6 batches** (384 KB)

3. **Низкоприоритетные очереди уменьшены:**
   - `data_low`: 4 → **2 batches** (128 KB)
   - `background`: 4 → **2 batches** (128 KB)

4. **Таймаут увеличен на 50%:**
   - `wait_before_close`: 20с → **30с**

### Затронутые системы

- ✅ Main Pi Zenoh Router (10.1.1.10)
- ✅ Vision Pi Zenoh Router (10.1.1.11)

---

## 🚀 Процедура развёртывания

### Предварительные требования

- SSH доступ к обоим Raspberry Pi
- Пароль: `<ROBOT_PASSWORD>`
- Git репозиторий на `develop` или `main` ветке

### Шаг 1: Обновление Vision Pi

```bash
# Подключение к Vision Pi
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.21

# Переход в директорию проекта
cd ~/rob_box_project

# Получение последних изменений
git pull

# Проверка изменений в конфигурации
git diff HEAD~1 docker/vision/config/zenoh_router_config.json5

# Переход в директорию docker
cd docker/vision

# Перезапуск Zenoh router
docker compose restart zenoh-router

# Ожидание запуска (5 секунд)
sleep 5

# Проверка логов
docker logs zenoh-router --tail 100

# Проверка отсутствия ошибок
docker logs zenoh-router 2>&1 | grep -i "unable to push"
docker logs zenoh-router 2>&1 | grep -i "cannot find link"

# Выход
exit
```

### Шаг 2: Обновление Main Pi

```bash
# Подключение к Main Pi
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.20

# Переход в директорию проекта
cd ~/rob_box_project

# Получение последних изменений
git pull

# Проверка изменений в конфигурации
git diff HEAD~1 docker/main/config/zenoh_router_config.json5

# Переход в директорию docker
cd docker/main

# Перезапуск Zenoh router
docker compose restart zenoh-router

# Ожидание запуска (5 секунд)
sleep 5

# Проверка логов
docker logs zenoh-router --tail 100

# Проверка отсутствия ошибок
docker logs zenoh-router 2>&1 | grep -i "unable to push"
docker logs zenoh-router 2>&1 | grep -i "cannot find link"

# Выход
exit
```

### Шаг 3: Автоматическое развёртывание (альтернатива)

Можно выполнить всё одной командой с локальной машины:

```bash
# Vision Pi - полный процесс
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.21 << 'EOF'
cd ~/rob_box_project
git pull
cd docker/vision
docker compose restart zenoh-router
sleep 5
echo "=== Vision Pi Zenoh Router Status ==="
docker logs zenoh-router --tail 50
echo "=== Checking for errors ==="
docker logs zenoh-router 2>&1 | grep -i "unable to push" || echo "✅ No 'unable to push' errors"
docker logs zenoh-router 2>&1 | grep -i "cannot find link" || echo "✅ No 'cannot find link' errors"
EOF

# Main Pi - полный процесс
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.20 << 'EOF'
cd ~/rob_box_project
git pull
cd docker/main
docker compose restart zenoh-router
sleep 5
echo "=== Main Pi Zenoh Router Status ==="
docker logs zenoh-router --tail 50
echo "=== Checking for errors ==="
docker logs zenoh-router 2>&1 | grep -i "unable to push" || echo "✅ No 'unable to push' errors"
docker logs zenoh-router 2>&1 | grep -i "cannot find link" || echo "✅ No 'cannot find link' errors"
EOF
```

---

## ✅ Проверка результатов

### 1. Проверка статуса роутеров

```bash
# Vision Pi
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.21 'docker ps | grep zenoh'

# Main Pi
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.20 'docker ps | grep zenoh'
```

**Ожидаемый результат:** контейнеры `zenoh-router` в статусе `Up`

### 2. Проверка отсутствия ошибок в логах

```bash
# Vision Pi - последние 200 строк логов
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.21 'docker logs zenoh-router --tail 200 2>&1 | grep -E "(ERROR|WARN)"'

# Main Pi - последние 200 строк логов
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.20 'docker logs zenoh-router --tail 200 2>&1 | grep -E "(ERROR|WARN)"'
```

**Ожидаемый результат:** отсутствие ошибок "Unable to push" и "Cannot find link"

### 3. Проверка работы ROS 2 топиков

```bash
# Подключиться к любому Pi
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.21

# Проверка камеры
ros2 topic hz /camera/rgb/image_raw

# Проверка LiDAR
ros2 topic hz /scan

# Проверка TF
ros2 topic hz /tf

# Выход
exit
```

**Ожидаемый результат:** стабильная частота публикации без пропусков

### 4. Нагрузочное тестирование

Запустить все системы одновременно и наблюдать за логами в течение 10-15 минут:

```bash
# Мониторинг логов Vision Pi в реальном времени
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.21 'docker logs -f zenoh-router 2>&1 | grep -E "(ERROR|transport)"'
```

```bash
# Мониторинг логов Main Pi в реальном времени
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.20 'docker logs -f zenoh-router 2>&1 | grep -E "(ERROR|transport)"'
```

**Ожидаемый результат:** отсутствие ошибок даже при высокой нагрузке

---

## 📊 Мониторинг после развёртывания

### Краткосрочный мониторинг (первые 24 часа)

1. **Каждый час проверять логи:**
   ```bash
   # Скрипт для быстрой проверки
   for pi_ip in 10.1.1.21 10.1.1.20; do
     echo "=== Checking Pi $pi_ip ==="
     sshpass -p '<ROBOT_PASSWORD>' ssh ros2@$pi_ip 'docker logs zenoh-router --since 1h 2>&1 | grep -c "Unable to push" || echo 0'
   done
   ```

2. **Проверка использования памяти:**
   ```bash
   sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.21 'free -h'
   sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.20 'free -h'
   ```

### Долгосрочный мониторинг (неделя)

1. **Еженедельная проверка стабильности:**
   - Нет ошибок "Unable to push"
   - Нет ошибок "Cannot find link"
   - Стабильная работа ROS 2 топиков

2. **Проверка uptime контейнеров:**
   ```bash
   sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.21 'docker ps --format "table {{.Names}}\t{{.Status}}" | grep zenoh'
   sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.20 'docker ps --format "table {{.Names}}\t{{.Status}}" | grep zenoh'
   ```

---

## 🔧 Откат изменений (если требуется)

Если по какой-то причине новая конфигурация вызывает проблемы:

### Вариант 1: Откат к предыдущей конфигурации (20s, queues=4)

```bash
# На обоих Pi
cd ~/rob_box_project
git checkout HEAD~1 docker/main/config/zenoh_router_config.json5
git checkout HEAD~1 docker/vision/config/zenoh_router_config.json5

# Перезапуск
cd docker/main  # или docker/vision
docker compose restart zenoh-router
```

### Вариант 2: Откат к исходной конфигурации (5s, queues=2)

```bash
# Найти коммит с исходной конфигурацией
git log --oneline docker/main/config/zenoh_router_config.json5

# Откатить к нужному коммиту
git checkout <commit_hash> docker/main/config/zenoh_router_config.json5
git checkout <commit_hash> docker/vision/config/zenoh_router_config.json5

# Перезапуск
cd docker/main  # или docker/vision
docker compose restart zenoh-router
```

---

## 📚 Дополнительная информация

- **Полный анализ:** [ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md](../reports/ZENOH_TRANSPORT_ERROR_ANALYSIS_2025-11-09.md)
- **Быстрый справочник:** [ZENOH_TRANSPORT_FIX_QUICKREF.md](ZENOH_TRANSPORT_FIX_QUICKREF.md)
- **Исходный код Zenoh (ошибка "Cannot find link"):** https://github.com/eclipse-zenoh/zenoh/blob/main/zenoh/src/net/protocol/network.rs#L287

---

## ❓ Часто задаваемые вопросы

### Q: Нужно ли перезагружать Raspberry Pi?

**A:** Нет, достаточно перезапустить только контейнер `zenoh-router`.

### Q: Может ли это повлиять на другие контейнеры?

**A:** Нет, изменения затрагивают только конфигурацию Zenoh router. Другие контейнеры (camera, lidar, rtabmap) не требуют перезапуска.

### Q: Сколько дополнительной памяти будет использоваться?

**A:** Примерно +1 MB на каждом Pi (с 2 MB до 3 MB для Zenoh router). Это приемлемо для Raspberry Pi 5.

### Q: Что делать, если ошибки продолжаются?

**A:** См. раздел "Если проблема сохраняется" в [ZENOH_TRANSPORT_FIX_QUICKREF.md](ZENOH_TRANSPORT_FIX_QUICKREF.md). Возможно потребуется дальнейшее увеличение критичных очередей до максимума (16 batches).

### Q: Можно ли применить изменения без остановки системы?

**A:** Нет, `zenoh-router` необходимо перезапустить для применения новой конфигурации. Но перезапуск занимает всего несколько секунд.

---

## ✅ Чеклист успешного развёртывания

- [ ] Vision Pi: выполнен `git pull`
- [ ] Vision Pi: перезапущен `zenoh-router`
- [ ] Vision Pi: логи не содержат ошибок "Unable to push"
- [ ] Vision Pi: логи не содержат ошибок "Cannot find link"
- [ ] Main Pi: выполнен `git pull`
- [ ] Main Pi: перезапущен `zenoh-router`
- [ ] Main Pi: логи не содержат ошибок "Unable to push"
- [ ] Main Pi: логи не содержат ошибок "Cannot find link"
- [ ] ROS 2 топики работают стабильно (camera, lidar, scan, tf)
- [ ] Нагрузочный тест пройден (15 минут работы без ошибок)
- [ ] Запланирован мониторинг на 24 часа

---

**Дата развёртывания:** _______________  
**Выполнил:** _______________  
**Результат:** ⬜ Успешно ⬜ Требуется доработка
