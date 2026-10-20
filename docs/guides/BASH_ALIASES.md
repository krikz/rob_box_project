# 🤖 РОББОКС - Полезные Bash алиасы

Набор алиасов для удобной работы с системой РОББОКС.

## ⚡ Quick Start

После установки узла через `setup_node.sh` алиасы доступны автоматически:

```bash
source ~/.bashrc   # Загрузить алиасы в текущей сессии
rbalias            # Показать список всех доступных команд
```

### 🌟 Самые полезные команды

```bash
rbstatus    # Статус контейнеров
rbhealth    # Полная проверка системы (статус + падающие + ресурсы)
rbupdate    # Обновить всё (stop → git pull → docker pull → start)
rbdebug     # Показать логи проблемных контейнеров
rbalias     # Список всех команд
```

## 📦 Установка

### Автоматически (рекомендуется)

Алиасы устанавливаются автоматически при настройке узла:

```bash
# Vision Pi
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s vision

# Main Pi
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s main
```

### Вручную

Используйте установочный скрипт:

```bash
# Скачать и применить все алиасы
~/rob_box_project/scripts/install_aliases.sh
```

Или добавьте в `~/.bashrc` вручную:

```bash
# Добавить РОББОКС алиасы
if [ -f ~/rob_box_project/scripts/robbox_aliases.sh ]; then
    source ~/rob_box_project/scripts/robbox_aliases.sh
fi
```

## 🐳 Docker Management

### Основные команды

```bash
# Управление Docker контейнерами
alias rbstop='docker compose down'              # Остановить все контейнеры
alias rbstart='docker compose up -d'            # Запустить все контейнеры
alias rbrestart='docker compose restart'        # Перезапустить все контейнеры
alias rbstatus='docker compose ps'              # Статус контейнеров
alias rblogs='docker compose logs -f'           # Все логи (follow)
alias rbpull='docker compose pull'              # Обновить образы

# Для конкретного узла
alias rbvision='cd ~/rob_box_project/docker/vision'   # Перейти к Vision Pi
alias rbsound='cd ~/rob_box_project/docker/sound'     # Перейти к Sound Pi (ADR-0159)
alias rbmain='cd ~/rob_box_project/docker/main'       # Перейти к Main Pi
```

### Логи отдельных сервисов

```bash
# Vision Pi
alias rblog-oak='docker compose logs -f oak-d'
alias rblog-lidar='docker compose logs -f lslidar'
alias rblog-led='docker compose logs -f led-matrix'
alias rblog-apriltag='docker compose logs -f apriltag'

# Sound Pi (ADR-0159: голосовой стек и ТАРС)
alias rblog-voice='docker compose logs -f voice-assistant'
alias rblog-sc='docker compose logs -f supercollider'
alias rblog-supervisor='docker compose logs -f supervisor'

# Main Pi
alias rblog-zenoh='docker compose logs -f zenoh-router'
```

### Полный workflow обновления

```bash
# Полное обновление системы (одна команда)
alias rbupdate='rbstop && git pull origin develop && rbpull && rbstart'

# Поэтапное обновление
alias rbstop-all='cd ~/rob_box_project/docker/vision && docker compose down; cd ~/rob_box_project/docker/sound && docker compose down; cd ~/rob_box_project/docker/main && docker compose down'
alias rbstart-all='cd ~/rob_box_project/docker/vision && docker compose up -d; cd ~/rob_box_project/docker/sound && docker compose up -d; cd ~/rob_box_project/docker/main && docker compose up -d'
```

## 🔍 Мониторинг и диагностика

```bash
# Проверка статуса с деталями
alias rbcheck='~/rob_box_project/docker/diagnose_data_flow.sh'

# Системный мониторинг
alias rbmonitor='~/rob_box_project/docker/monitor_system.sh'

# Проверка проблемных контейнеров (постоянно рестартующихся)
alias rbfailing='docker ps -a --filter "status=restarting" --format "table {{.Names}}\t{{.Status}}"'

# Показать логи проблемных контейнеров
alias rbdebug='for container in $(docker ps -a --filter "status=restarting" --format "{{.Names}}"); do echo "=== $container ==="; docker logs --tail 50 $container; done'

# Использование ресурсов
alias rbresources='docker stats --no-stream'

# Быстрая проверка всего
alias rbhealth='echo "=== Docker Status ===" && docker compose ps && echo "" && echo "=== Failing Containers ===" && rbfailing && echo "" && echo "=== Resource Usage ===" && docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}"'
```

## 📡 ROS2 команды

```bash
# ROS2 топики и ноды
alias rbtopics='ros2 topic list'
alias rbnodes='ros2 node list'
alias rbinfo='ros2 topic info'
alias rbecho='ros2 topic echo'

# Быстрый просмотр данных
alias rbcamera='ros2 topic echo /camera/image_raw --once'
alias rblidar='ros2 topic echo /scan --once'
alias rbimu='ros2 topic echo /imu/data --once'
```

## 🔧 Git Management

```bash
# Навигация
alias rbcd='cd ~/rob_box_project'
alias rbsrc='cd ~/rob_box_project/src'
alias rbdocker='cd ~/rob_box_project/docker'
alias rbdocs='cd ~/rob_box_project/docs'

# Git операции
alias rbgit='cd ~/rob_box_project && git status'
alias rbpull='cd ~/rob_box_project && git pull origin develop'
alias rbpush='cd ~/rob_box_project && git push origin'
alias rbdiff='cd ~/rob_box_project && git diff'
alias rblog='cd ~/rob_box_project && git log --oneline -10'

# Быстрое обновление
alias rbsync='rbstop && rbpull && rbpull && rbstart && rbstatus'
```

## 🛠️ Colcon Build (для разработки)

```bash
# Быстрая сборка
alias rbbuild='cd ~/rob_box_project && colcon build --symlink-install'
alias rbbuild-voice='cd ~/rob_box_project && colcon build --packages-select rob_box_voice --symlink-install'
alias rbbuild-animations='cd ~/rob_box_project && colcon build --packages-select rob_box_animations --symlink-install'

# Очистка и пересборка
alias rbclean='cd ~/rob_box_project && rm -rf build install log'
alias rbrebuild='rbclean && rbbuild'

# Source окружения
alias rbsource='source ~/rob_box_project/install/setup.bash'
```

## 📊 Системные утилиты

```bash
# Проверка USB устройств
alias rbusb='lsusb | grep -E "(2886|03e7|10c4)"'  # ReSpeaker, OAK-D, LSLidar

# Температура и нагрузка
alias rbtemp='vcgencmd measure_temp && vcgencmd measure_volts core'
alias rbload='uptime && free -h && df -h /'

# Сеть
alias rbip='ip -4 addr show | grep inet'
alias rbping='ping -c 3 10.1.1.10 && ping -c 3 10.1.1.11'

# Процессы ROS
alias rbps='ps aux | grep ros2'
alias rbkill='pkill -9 -f ros2'
```

## 🎨 Workflow примеры

### Обновление Vision Pi

```bash
rbvision              # Переходим в директорию
rbstatus              # Проверяем статус
rbstop                # Останавливаем контейнеры
rbgit                 # Проверяем изменения
rbpull                # Обновляем код
rbpull                # Обновляем образы (второй rbpull = docker compose pull)
rbstart               # Запускаем контейнеры
rbstatus              # Проверяем что всё OK
```

### Диагностика проблем

```bash
rbhealth              # Общая проверка здоровья системы
rbfailing             # Какие контейнеры падают?
rbdebug               # Показать логи проблемных контейнеров
rblog-voice           # Смотрим конкретный сервис
```

### Полное обновление системы

```bash
# Main Pi
rbmain
rbstop
git pull origin develop
docker compose pull
rbstart
rbstatus

# Vision Pi
rbvision
rbstop
git pull origin develop
docker compose pull
rbstart
rbstatus

# Или одной командой (если находитесь в нужной директории)
rbupdate
```

## 🔥 Emergency команды

```bash
# Аварийная остановка всего
alias rbemergency='docker stop $(docker ps -q)'

# Полная очистка
alias rbnuke='docker compose down -v && docker system prune -af'

# Перезапуск зависших контейнеров
alias rbfix='for container in $(docker ps -a --filter "status=restarting" --format "{{.Names}}"); do docker compose restart $container; done'
```

## 📝 Примечания

- Все алиасы предполагают что проект находится в `~/rob_box_project`
- Для работы с Docker нужны права (пользователь в группе `docker`)
- Некоторые команды требуют `sudo` на Raspberry Pi

## 🚀 Быстрая справка в терминале

После установки алиасов наберите:

```bash
rbalias   # Показать все доступные алиасы
rbhelp    # Показать эту справку
```

---

**Автор:** AI Agent + krikz  
**Дата:** 2025-10-15  
**Версия:** 1.0
