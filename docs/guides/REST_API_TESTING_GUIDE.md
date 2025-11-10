# Руководство по тестированию REST API → ROS

**Дата:** 2025-11-10  
**Цель:** Выяснить почему команды через REST API не доходят до робота

---

## 🎯 План тестирования

1. Включить детальное логирование Zenoh
2. Отправить тестовые команды через REST API
3. Проверить где застревают сообщения
4. Определить правильный формат и ключ

---

## 📝 Шаг 1: Подготовка на локальной машине

### Запустить тестовый скрипт

```bash
cd ~/rob_box_project
./scripts/test_rest_api.sh
```

Скрипт автоматически:
- ✅ Проверит доступность облачного роутера
- ✅ Получит список топиков
- ✅ Найдёт полный ключ cmd_vel_voice
- ✅ Создаст тестовый CDR файл
- ✅ Отправит команды на разные варианты ключей
- ✅ Покажет HTTP коды ответов

Сохраните вывод для анализа!

---

## 🔧 Шаг 2: Включить логирование на роботе

### На Main Pi (10.1.1.20)

```bash
ssh ros2@10.1.1.20
```

### Вариант A: Временное включение debug логов

```bash
# Остановить twist-mux
cd ~/rob_box_project/docker/main
docker-compose stop twist-mux

# Запустить с debug логами Zenoh
docker-compose run --rm \
  -e RUST_LOG=zenoh=debug,rmw_zenoh_cpp=debug \
  twist-mux \
  /scripts/start_twist_mux.sh
```

### Вариант B: Мониторинг через отдельное окно

**Терминал 1: Логи twist_mux**
```bash
docker logs -f --tail 100 twist-mux
```

**Терминал 2: ROS топик**
```bash
docker exec twist-mux ros2 topic echo /cmd_vel_voice
```

**Терминал 3: Zenoh подписки**
```bash
# Проверить подписки на локальном роутере
curl -s http://10.1.1.10:8000/@/local/subscriber | jq '.[] | select(.key | contains("cmd_vel_voice"))'
```

---

## 🧪 Шаг 3: Отправка тестовых команд

### С локальной машины

```bash
# Вариант 1: Упрощённый ключ
curl -X PUT http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_voice \
  -H "Content-Type: application/octet-stream" \
  --data-binary @/tmp/twist.cdr

# Вариант 2: С domain
curl -X PUT http://zenoh.robbox.online/robots/RBXU100001/0/cmd_vel_voice \
  -H "Content-Type: application/octet-stream" \
  --data-binary @/tmp/twist.cdr

# Вариант 3: Полный ключ (если найден)
# Получить полный ключ:
FULL_KEY=$(curl -s http://zenoh.robbox.online/robots/RBXU100001/** | grep cmd_vel_voice | head -1 | awk '{print $1}')
echo "Полный ключ: $FULL_KEY"

curl -X PUT "http://zenoh.robbox.online/$FULL_KEY" \
  -H "Content-Type: application/octet-stream" \
  --data-binary @/tmp/twist.cdr
```

### Наблюдать результаты

После каждой команды проверяйте:
1. HTTP код ответа (200 = принято)
2. Логи twist-mux (появились ли сообщения)
3. ROS топик (получены ли данные)

---

## 📊 Шаг 4: Анализ логов

### Что искать в логах twist-mux

**Хорошие признаки:**
```
[INFO] [twist_mux]: Topic handler 'topics.voice' subscribed to topic 'cmd_vel_voice'
[INFO] [twist_mux]: Received message on 'cmd_vel_voice'
```

**Проблемные признаки:**
```
[WARN] [rmw_zenoh_cpp]: No matching subscription found
[ERROR] [zenoh]: Failed to deserialize message
```

### Что искать в Zenoh логах

**Хорошие признаки:**
```
[DEBUG] zenoh: Received PUT on robots/RBXU100001/...
[DEBUG] rmw_zenoh_cpp: Publishing to local subscribers
```

**Проблемные признаки:**
```
[DEBUG] zenoh: No subscribers for key
[ERROR] zenoh: Failed to match key expression
```

---

## 🔍 Шаг 5: Диагностические команды

### Проверить Zenoh маршрутизацию

```bash
# На Main Pi
curl -s http://10.1.1.10:8000/@/router/status | jq

# Проверить подключения
curl -s http://10.1.1.10:8000/@/router/sessions | jq

# Проверить подписчиков
curl -s http://10.1.1.10:8000/@/local/subscriber | jq '.[] | select(.key | contains("cmd_vel"))'
```

### Проверить ROS топики

```bash
# Список топиков
docker exec twist-mux ros2 topic list | grep cmd_vel

# Информация о топике
docker exec twist-mux ros2 topic info /cmd_vel_voice -v

# Эхо топика (ожидание сообщений)
docker exec twist-mux ros2 topic echo /cmd_vel_voice --once
```

---

## 📝 Шаг 6: Документирование результатов

Сохраните следующую информацию:

### 1. Полный ключ Zenoh
```bash
curl -s http://zenoh.robbox.online/robots/RBXU100001/** | grep cmd_vel_voice
```

### 2. HTTP ответы
Для каждого варианта ключа:
- HTTP код
- Тело ответа (если есть)

### 3. Логи twist-mux
```bash
docker logs twist-mux --tail 50
```

### 4. Zenoh подписки
```bash
curl -s http://10.1.1.10:8000/@/local/subscriber | jq
```

---

## 💡 Возможные сценарии

### Сценарий 1: Сообщения доходят, но неправильный формат

**Симптомы:**
- HTTP 200 OK
- Логи показывают получение в Zenoh
- Ошибки десериализации в rmw_zenoh_cpp

**Решение:** Проверить формат CDR данных

### Сценарий 2: Сообщения не доходят - неправильный ключ

**Симптомы:**
- HTTP 200 OK
- Zenoh принимает, но нет подписчиков
- twist-mux ничего не получает

**Решение:** Использовать полный ключ с типом и хашем

### Сценарий 3: Облачный роутер не маршрутизирует

**Симптомы:**
- HTTP 200 OK
- Облако принимает
- Робот не видит сообщений

**Решение:** Проверить режим роутера и peers_failover_brokering

---

## ✅ Чеклист тестирования

- [ ] Запустить `scripts/test_rest_api.sh`
- [ ] Включить debug логи на роботе
- [ ] Отправить тестовые команды
- [ ] Проверить логи twist-mux
- [ ] Проверить ROS топик
- [ ] Определить правильный ключ Zenoh
- [ ] Определить правильный формат CDR
- [ ] Задокументировать результаты
- [ ] Обновить документацию с правильным решением

---

## 📞 Следующие шаги

После тестирования и сбора логов:

1. Определите правильный формат ключа
2. Определите правильный формат данных
3. Обновите веб-клиент
4. Обновите документацию

---

**Готово к тестированию!** Запустите скрипты и соберите логи.
