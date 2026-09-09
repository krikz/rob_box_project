# Устранение ошибки "Address in use" на Main Pi

## 📋 Проблема

На Main Pi Zenoh роутер падает с ошибкой:
```
Unable to open listener tcp/[::]:7447#iface=eth0: Can not create a new TCP listener 
bound to tcp/[::]:7447#iface=eth0: [[::]:7447: Address in use (os error 98)
```

## 🔍 Причина

Порт 7447 на интерфейсе eth0 уже занят. Возможные причины:

1. **Старый экземпляр zenoh-router не был остановлен**
   - Docker restart policy `unless-stopped` пытается перезапустить упавший контейнер
   - Старый процесс все еще держит порт

2. **Запущено несколько экземпляров zenoh-router**
   - Случайно запущено несколько контейнеров с одинаковым именем
   - docker-compose up был вызван несколько раз без down

3. **Другой процесс использует порт 7447**
   - Редко, но возможно

## 🔧 Решение

### Шаг 1: Диагностика

Запустите диагностический скрипт на Main Pi:

```bash
ssh ros2@10.1.1.20
cd ~/rob_box_project
./scripts/diagnose_zenoh_port_conflict.sh
```

Скрипт покажет:
- Сколько экземпляров zenoh-router запущено
- Кто использует порт 7447
- Есть ли ошибки в логах

### Шаг 2: Остановка всех экземпляров

**Если найдено несколько контейнеров:**

```bash
# Остановить ВСЕ контейнеры zenoh-router
docker stop $(docker ps -aq --filter 'name=zenoh-router')

# Удалить старые контейнеры
docker rm $(docker ps -aq --filter 'name=zenoh-router')

# Подождать 5 секунд для освобождения порта
sleep 5
```

**Если один контейнер, но порт занят:**

```bash
cd ~/rob_box_project/docker/main

# Корректная остановка через docker-compose
docker-compose down zenoh-router

# Подождать 5 секунд
sleep 5
```

### Шаг 3: Проверка освобождения порта

```bash
# Проверить что порт 7447 свободен
sudo netstat -tlnp | grep 7447

# Или через ss
sudo ss -tlnp | grep 7447

# Или через lsof
sudo lsof -i :7447
```

Если команда НЕ выдает результат → порт свободен ✅

Если порт все еще занят → найти и остановить процесс:
```bash
# Найти PID процесса
sudo lsof -i :7447

# Остановить процесс (замените PID на реальный)
sudo kill -9 <PID>
```

### Шаг 4: Запуск zenoh-router

```bash
cd ~/rob_box_project/docker/main

# Запустить zenoh-router
docker-compose up -d zenoh-router

# Проверить статус
docker ps | grep zenoh-router
```

### Шаг 5: Проверка логов

```bash
# Просмотр логов в реальном времени
docker logs -f zenoh-router

# Или последние 50 строк
docker logs zenoh-router --tail 50
```

**Ожидаемый вывод (успешный запуск):**
```
INFO zenoh::api::loader: Successfully started plugin storage_manager
INFO zenoh::net::runtime::orchestrator: Listener tcp/[::]:7447#iface=eth0 started
INFO zenoh::api::session: Session opened
```

**НЕ должно быть:**
```
WARN Unable to open listener tcp/[::]:7447#iface=eth0: Address in use
```

### Шаг 6: Применение исправлений для Vision Pi

После успешного запуска Main Pi роутера, примените исправления на Vision Pi:

```bash
# Подключиться к Vision Pi
ssh ros2@10.1.1.21

# Обновить код
cd ~/rob_box_project/docker/vision
git pull

# Перезапустить с новой конфигурацией
./update_and_restart.sh

# Или вручную:
docker-compose down zenoh-router-vision
sleep 5
docker-compose up -d zenoh-router-vision
```

### Шаг 7: Проверка связи между Pi

**На Main Pi:**
```bash
# Проверить что Vision Pi подключился
docker exec zenoh-router netstat -tnp 2>/dev/null | grep "10.1.1.11.*:7447.*ESTABLISHED"
```

**На Vision Pi:**
```bash
# Проверить подключение к Main Pi
docker exec zenoh-router-vision netstat -tnp 2>/dev/null | grep "10.1.1.10:7447.*ESTABLISHED"
```

## 🎯 Автоматическое решение

Если вы хотите автоматизировать весь процесс:

```bash
#!/bin/bash
# auto_fix_zenoh_port.sh

echo "Останавливаем все zenoh-router контейнеры..."
docker stop $(docker ps -aq --filter 'name=zenoh-router') 2>/dev/null || true
docker rm $(docker ps -aq --filter 'name=zenoh-router') 2>/dev/null || true

echo "Ожидание освобождения порта (5 секунд)..."
sleep 5

echo "Запуск zenoh-router..."
cd ~/rob_box_project/docker/main
docker-compose up -d zenoh-router

echo "Проверка статуса..."
sleep 3
docker logs zenoh-router --tail 20

echo ""
echo "Проверка порта 7447..."
if docker logs zenoh-router 2>&1 | grep -q "Address in use"; then
    echo "❌ ОШИБКА: Порт все еще занят!"
    sudo netstat -tlnp | grep 7447
else
    echo "✅ Zenoh router успешно запущен!"
fi
```

## 📝 Предотвращение в будущем

1. **Всегда используйте docker-compose для управления:**
   ```bash
   # Остановка
   docker-compose down zenoh-router
   
   # Запуск
   docker-compose up -d zenoh-router
   
   # Перезапуск
   docker-compose restart zenoh-router
   ```

2. **Не запускайте несколько экземпляров вручную через `docker run`**

3. **При обновлении конфигурации:**
   ```bash
   # Правильно
   docker-compose down zenoh-router
   git pull
   docker-compose up -d zenoh-router
   
   # НЕ правильно (может создать дубликаты)
   docker-compose up -d zenoh-router  # без down!
   ```

4. **Используйте скрипт `update_and_restart.sh`:**
   ```bash
   cd ~/rob_box_project/docker/main
   ./update_and_restart.sh
   ```
   Этот скрипт корректно останавливает старые контейнеры перед запуском новых.

## 🔍 Дополнительная диагностика

Если проблема сохраняется, проверьте:

1. **Docker daemon log:**
   ```bash
   sudo journalctl -u docker.service -n 100
   ```

2. **Системный лог:**
   ```bash
   sudo dmesg | tail -50
   ```

3. **Все процессы на порту 7447:**
   ```bash
   sudo fuser -v 7447/tcp
   ```

4. **Network namespaces (для продвинутых):**
   ```bash
   sudo ip netns list
   sudo ip netns exec <namespace> netstat -tlnp | grep 7447
   ```

---

**Дата:** 2025-11-10  
**Связанные документы:** 
- `ZENOH_ROUTER_PORT_CONFLICT_FIX_2025-11-10.md`
- `scripts/diagnose_zenoh_port_conflict.sh`
