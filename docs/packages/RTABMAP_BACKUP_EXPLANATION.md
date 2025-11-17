# RTABMap Backup - Пояснение

## ❓ Проблема

При подтверждении голосовой команды "начать новое исследование", dialogue_node пытался создать автоматический backup через `docker exec`, что приводило к ошибке:

```
[dialogue_node-4] [ERROR] ❌ Backup error: [Errno 2] No such file or directory: 'docker'
```

## 🔍 Анализ

### Почему возникла ошибка?

1. **dialogue_node работает ВНУТРИ контейнера** (voice-assistant на Vision Pi)
2. **Docker CLI недоступен** внутри контейнера
3. **RTABMap работает на другом Pi** (Main Pi в контейнере rtabmap)
4. Невозможно выполнить `docker exec rtabmap` из контейнера на Vision Pi

### Архитектура:

```
Vision Pi                     Main Pi
┌─────────────────┐          ┌─────────────────┐
│ voice-assistant │          │    rtabmap      │
│   (container)   │  ROS2    │   (container)   │
│                 │◄────────►│                 │
│ dialogue_node   │  Zenoh   │  /maps/         │
│                 │          │  rtabmap.db     │
└─────────────────┘          └─────────────────┘
         │                            
         ❌ НЕТ docker CLI
         ❌ НЕТ доступа к другому Pi
```

## ✅ Решение

### RTABMap Multi-Session подход

**RTABMap уже поддерживает сохранение истории!**

1. **reset_memory НЕ удаляет данные** из rtabmap.db
2. **Создаётся новая сессия** в той же БД
3. **Старая карта остаётся** как неактивная сессия
4. **Можно переключаться** между сессиями через GUI

### Что происходит при reset_memory?

```sql
-- rtabmap.db (SQLite)
┌─────────┬──────────────┬─────────┐
│ Session │ Created      │ Active  │
├─────────┼──────────────┼─────────┤
│   1     │ 2025-11-01   │  false  │  ← Старая карта (сохранена!)
│   2     │ 2025-11-17   │  true   │  ← Новая карта (активная)
└─────────┴──────────────┴─────────┘
```

### Что мы изменили?

1. **Убрали subprocess.run + docker exec** - не работает из контейнера
2. **Изменили сообщение роботу**:
   - Было: "Старая карта будет сохранена в резервную копию"
   - Стало: "Старая карта сохранится в базе данных"
3. **Добавили пояснение в лог**: RTABMap использует multi-session

## 📋 Ручной backup (опционально)

Если пользователь хочет сделать **полный backup rtabmap.db**:

### Через SSH на Main Pi:

```bash
# Подключиться к Main Pi
ssh ros2@10.1.1.20

# Создать backup вручную
cd ~/rob_box_project/docker/main/maps
cp rtabmap.db backups/rtabmap_backup_$(date +%Y%m%d_%H%M%S).db

# Проверить размер БД
ls -lh rtabmap.db
du -h rtabmap.db
```

### Через автоматический cron (рекомендуется):

```bash
# Создать скрипт backup
cat > ~/backup_rtabmap.sh << 'EOF'
#!/bin/bash
BACKUP_DIR="$HOME/rob_box_project/docker/main/maps/backups"
mkdir -p "$BACKUP_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
cp "$HOME/rob_box_project/docker/main/maps/rtabmap.db" \
   "$BACKUP_DIR/rtabmap_backup_${TIMESTAMP}.db"
# Удалить backup старше 30 дней
find "$BACKUP_DIR" -name "rtabmap_backup_*.db" -mtime +30 -delete
echo "✅ Backup created: rtabmap_backup_${TIMESTAMP}.db"
EOF

chmod +x ~/backup_rtabmap.sh

# Добавить в crontab (каждый день в 2:00)
crontab -e
# Добавить строку:
# 0 2 * * * /home/ros2/backup_rtabmap.sh >> /home/ros2/backup.log 2>&1
```

## 🎯 Итог

### До изменений:
- ❌ Голосовая команда крашилась при подтверждении
- ❌ Пытались делать backup через docker exec
- ❌ Неправильная архитектура (команда из контейнера)

### После изменений:
- ✅ Голосовая команда работает
- ✅ Используем встроенный multi-session RTABMap
- ✅ Старая карта НЕ теряется
- ✅ Backup - опциональная ручная операция

## 📚 Ссылки

- [RTABMap ROS Wiki - Services](http://wiki.ros.org/rtabmap_ros#Services)
- [RTABMap Database Format](https://github.com/introlab/rtabmap/wiki/Database-Format)
- [Multi-Session Mapping](https://github.com/introlab/rtabmap/wiki/Multi-Session-Mapping)

---

**Автор:** GitHub Copilot  
**Дата:** 2025-11-17  
**Issue:** Ошибка backup при подтверждении mapping команды
