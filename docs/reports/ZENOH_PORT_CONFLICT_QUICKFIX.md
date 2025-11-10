# Zenoh Port Conflict Fix - Quick Deployment Guide

**Дата:** 2025-11-10  
**Проблема:** ROS service errors + Zenoh transport closures  
**Решение:** Конкретные IP адреса вместо wildcard endpoints  
**Время развёртывания:** ~5 минут

---

## ⚡ Быстрое развёртывание

### 1. Main Pi (10.1.1.20)

```bash
sshpass -p 'open' ssh ros2@10.1.1.20 'cd ~/rob_box_project && git pull origin main && cd docker/main && docker compose restart zenoh-router && sleep 5 && docker compose restart'
```

### 2. Vision Pi (10.1.1.21)

```bash
sshpass -p 'open' ssh ros2@10.1.1.21 'cd ~/rob_box_project && git pull origin main && cd docker/vision && docker compose restart zenoh-router && sleep 5 && docker compose restart'
```

### 3. Проверка

```bash
# Main Pi - проверить что router слушает на 10.1.1.10:7447
sshpass -p 'open' ssh ros2@10.1.1.20 'sudo netstat -tlnp | grep 7447'

# Vision Pi - проверить что router слушает на 10.1.1.11:7447
sshpass -p 'open' ssh ros2@10.1.1.21 'sudo netstat -tlnp | grep 7447'
```

**Ожидаемый вывод:**
```
Main Pi:   tcp 0 0 10.1.1.10:7447 0.0.0.0:* LISTEN <pid>/zenohd
Vision Pi: tcp 0 0 10.1.1.11:7447 0.0.0.0:* LISTEN <pid>/zenohd
```

---

## ✅ Проверка работы

### Проверить что ошибок больше нет

```bash
# Vision Pi - мониторить логи Zenoh router (должно быть БЕЗ ошибок)
sshpass -p 'open' ssh ros2@10.1.1.21 'docker logs -f zenoh-router 2>&1 | grep -i error'

# НЕ должно быть:
# ❌ "Unable to push non droppable network message"
# ❌ "Closing transport"
# ❌ "Query not found"
```

### Проверить ROS топики

```bash
# Main Pi - проверить что видим топики от Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.20 'docker exec rtabmap ros2 topic hz /camera/rgb/image_raw'

# Ожидаемый результат:
# average rate: 5.000
```

---

## 🔧 Что изменилось

| Компонент | Было | Стало |
|-----------|------|-------|
| Main Pi router listen | `tcp/[::]:7447#iface=eth0` | `tcp/10.1.1.10:7447` |
| Vision Pi router listen | `tcp/[::]:7447#iface=eth0` | `tcp/10.1.1.11:7447` |
| Main Pi session connect | `tcp/localhost:7447` | `tcp/10.1.1.10:7447` |
| Vision Pi session connect | `tcp/localhost:7447` | `tcp/10.1.1.11:7447` |

---

## 🐛 Если что-то пошло не так

### Router не запускается

```bash
# Проверить что порт не занят
sshpass -p 'open' ssh ros2@10.1.1.20 'sudo netstat -tlnp | grep 7447'

# Если занят - убить процесс и перезапустить
sshpass -p 'open' ssh ros2@10.1.1.20 'docker compose -f ~/rob_box_project/docker/main/docker-compose.yaml restart zenoh-router'
```

### ROS ноды не подключаются

```bash
# Проверить логи нод
sshpass -p 'open' ssh ros2@10.1.1.21 'docker logs oak-d --tail 50 | grep zenoh'

# Перезапустить все сервисы
sshpass -p 'open' ssh ros2@10.1.1.21 'cd ~/rob_box_project/docker/vision && docker compose restart'
```

---

## 📖 Полная документация

См. [ZENOH_PORT_CONFLICT_FIX_2025-11-10.md](ZENOH_PORT_CONFLICT_FIX_2025-11-10.md)

---

**Статус:** ✅ ГОТОВО  
**Автор:** GitHub Copilot
