# 🤖 РОББОКС - MOTD Preview

Так выглядит приветственное сообщение (MOTD) после установки узла через `setup_node.sh`.

## Vision Pi MOTD

```
    ╔═══════════════════════════════════════════════════════════════════╗
    ║                                                                   ║
    ║  ██████╗   ██████╗   ╔██████ ██████╗   ██████╗  ██╗  ██╗ ██████╗  ║ 
    ║  ██╔══██╗ ██╔═══██╗ ╔╩═══╗██ ██╔═══╩╗ ██╔═══██╗ ██║ ██╔╝██╔════╝  ║
    ║  ██████╔╝ ██║   ██║ ╚╗██████ ██████╔╝ ██║   ██║ █████╔╝ ██║       ║
    ║  ██╔═══╝  ██║   ██║ ╔██══╗██ ██╔══██╗ ██║   ██║ ██╔═██╗ ██║       ║
    ║  ██║      ╚██████╔╝ ╚╗██████ ██████╔╝ ╚██████╔╝ ██║  ██╗╚██████╗  ║
    ║  ╚═╝       ╚═════╝   ╚═════╝ ╚═════╝   ╚═════╝  ╚═╝  ╚═╝ ╚═════╝  ║
    ║                                                                   ║
    ╚═══════════════════════════════════════════════════════════════════╝

                    🤖  Vision Pi Node  🤖
                    
═══════════════════════════════════════════════════════════════

NODE INFO
  Node:             🎥 Vision Pi
  IP:               10.1.1.11
  Hostname:         vision-pi
  OS:               Debian GNU/Linux 12 (bookworm)
  Kernel:           6.6.31+rpt-rpi-v8
  Uptime:           3 days, 14:23

HARDWARE
  Platform:         Raspberry Pi 5 Model B Rev 1.0
  CPU:              ARM Cortex-A76 (4 cores) @ 2.4GHz
  Memory:           2.8 / 8.0 GB (35%)
  Temperature:      48.7°C

NETWORK
  Ethernet (eth0):  10.1.1.11/24
  WiFi (wlan0):     192.168.1.105/24

DOCKER CONTAINERS
  Status:           6 running

  ✓ oak-d              Up 2 days (healthy)
  ✓ lslidar            Up 2 days (healthy)
  ✓ led-matrix         Up 2 days
  ✓ voice-assistant    Up 14 hours
  ✓ apriltag           Up 2 days
  ✓ zenoh-bridge       Up 2 days

═══════════════════════════════════════════════════════════════
Quick Commands:
  rbstatus          - Check containers status
  rblogs            - View all container logs
  rbrestart         - Restart all containers
  rbupdate          - Full update (stop → git pull → docker pull → start)
  rbhealth          - Complete system health check
  rbdebug           - Show logs of failing containers

More aliases: Type rbalias to see all available commands
═══════════════════════════════════════════════════════════════
```

## Main Pi MOTD

```
    ╔═══════════════════════════════════════════════════════════════════╗
    ║                                                                   ║
    ║  ██████╗   ██████╗   ╔██████ ██████╗   ██████╗  ██╗  ██╗ ██████╗  ║ 
    ║  ██╔══██╗ ██╔═══██╗ ╔╩═══╗██ ██╔═══╩╗ ██╔═══██╗ ██║ ██╔╝██╔════╝  ║
    ║  ██████╔╝ ██║   ██║ ╚╗██████ ██████╔╝ ██║   ██║ █████╔╝ ██║       ║
    ║  ██╔═══╝  ██║   ██║ ╔██══╗██ ██╔══██╗ ██║   ██║ ██╔═██╗ ██║       ║
    ║  ██║      ╚██████╔╝ ╚╗██████ ██████╔╝ ╚██████╔╝ ██║  ██╗╚██████╗  ║
    ║  ╚═╝       ╚═════╝   ╚═════╝ ╚═════╝   ╚═════╝  ╚═╝  ╚═╝ ╚═════╝  ║
    ║                                                                   ║
    ╚═══════════════════════════════════════════════════════════════════╝

                      🤖  Main Pi Node  🤖
                    
═══════════════════════════════════════════════════════════════

NODE INFO
  Node:             🎮 Main Pi
  IP:               10.1.1.10
  Hostname:         main-pi
  OS:               Debian GNU/Linux 12 (bookworm)
  Kernel:           6.6.31+rpt-rpi-v8
  Uptime:           5 days, 8:15

HARDWARE
  Platform:         Raspberry Pi 5 Model B Rev 1.0
  CPU:              ARM Cortex-A76 (4 cores) @ 2.4GHz
  Memory:           1.2 / 8.0 GB (15%)
  Temperature:      45.2°C

NETWORK
  Ethernet (eth0):  10.1.1.10/24
  WiFi (wlan0):     192.168.1.104/24

DOCKER CONTAINERS
  Status:           2 running

  ✓ micro-ros-agent    Up 5 days (healthy)
  ✓ zenoh-router       Up 5 days (healthy)

═══════════════════════════════════════════════════════════════
Quick Commands:
  rbstatus          - Check containers status
  rblogs            - View all container logs
  rbrestart         - Restart all containers
  rbupdate          - Full update (stop → git pull → docker pull → start)
  rbhealth          - Complete system health check
  rbdebug           - Show logs of failing containers

More aliases: Type rbalias to see all available commands
═══════════════════════════════════════════════════════════════
```

## Алиасы в действии

### Пример использования `rbalias`

```bash
$ rbalias
🤖 РОББОКС - Доступные алиасы:

Docker:
  rbstart, rbstop, rbrestart, rbstatus, rblogs, rbpull
  rbvision, rbmain, rbupdate, rbsync

Мониторинг:
  rbhealth, rbcheck, rbmonitor, rbfailing, rbdebug, rbresources

ROS2:
  rbtopics, rbnodes, rbinfo, rbecho, rbcamera, rblidar, rbimu

Git:
  rbcd, rbgit, rbpullgit, rbpush, rbdiff, rblog

Build:
  rbbuild, rbbuild-voice, rbbuild-animations, rbclean, rbrebuild

System:
  rbusb, rbtemp, rbload, rbip, rbping, rbps, rbkill

Emergency:
  rbemergency, rbnuke, rbfix

Полная документация: ~/rob_box_project/docs/guides/BASH_ALIASES.md
```

### Пример использования `rbhealth`

```bash
$ rbhealth
=== Docker Status ===
NAME                STATUS              PORTS
oak-d               Up 2 days (healthy) 
lslidar             Up 2 days (healthy)
led-matrix          Up 2 days
voice-assistant     Up 14 hours
apriltag            Up 2 days
zenoh-bridge        Up 2 days

=== Failing Containers ===
No failing containers

=== Resource Usage ===
NAME              CPU %    MEM USAGE / LIMIT
oak-d             2.5%     145MB / 8GB
lslidar           1.2%     98MB / 8GB
led-matrix        0.8%     52MB / 8GB
voice-assistant   3.1%     234MB / 8GB
apriltag          1.5%     112MB / 8GB
zenoh-bridge      0.5%     45MB / 8GB
```

### Пример использования `rbupdate`

```bash
$ rbupdate
Останавливаем контейнеры...
[+] Running 6/6
 ✔ Container zenoh-bridge       Stopped
 ✔ Container apriltag           Stopped
 ✔ Container voice-assistant    Stopped
 ✔ Container led-matrix         Stopped
 ✔ Container lslidar            Stopped
 ✔ Container oak-d              Stopped

Обновляем код...
Already up to date.

Обновляем образы...
[+] Pulling 6/6
 ✔ oak-d Pulled
 ✔ lslidar Pulled
 ✔ led-matrix Pulled
 ✔ voice-assistant Pulled
 ✔ apriltag Pulled
 ✔ zenoh-bridge Pulled

Запускаем контейнеры...
[+] Running 6/6
 ✔ Container oak-d              Started
 ✔ Container lslidar            Started
 ✔ Container led-matrix         Started
 ✔ Container voice-assistant    Started
 ✔ Container apriltag           Started
 ✔ Container zenoh-bridge       Started

✓ Готово!

NAME                STATUS
oak-d               Up 2 seconds (health: starting)
lslidar             Up 2 seconds (health: starting)
led-matrix          Up 2 seconds
voice-assistant     Up 2 seconds
apriltag            Up 2 seconds
zenoh-bridge        Up 2 seconds
```

### Пример использования `rbdebug`

```bash
$ rbdebug
=== voice-assistant ===
[2025-10-15 15:42:13] ERROR: Failed to connect to ReSpeaker
[2025-10-15 15:42:13] ERROR: pyaudio.PyAudioError: [Errno -9996] Invalid input device
[2025-10-15 15:42:13] INFO: Retrying in 5 seconds...
[2025-10-15 15:42:18] INFO: ReSpeaker detected on device 2
[2025-10-15 15:42:18] INFO: Audio input initialized successfully

=== led-matrix ===
[2025-10-15 15:42:10] WARNING: Frame rate dropped to 15 FPS
[2025-10-15 15:42:15] INFO: Frame rate recovered: 30 FPS
```

## Типичные сценарии использования

### 1. Утренняя проверка системы

```bash
ssh vision@10.1.1.11
rbhealth              # Проверка состояния
rbfailing             # Есть ли проблемные контейнеры?
rblogs               # Просмотр логов
```

### 2. Обновление проекта

```bash
rbupdate             # Остановка → git pull → docker pull → запуск
# Или по шагам:
rbstop               # Остановить контейнеры
rbpullgit            # Обновить код
rbpull               # Обновить образы
rbstart              # Запустить
```

### 3. Диагностика проблемы

```bash
rbhealth             # Общее состояние
rbfailing            # Проблемные контейнеры
rbdebug              # Логи проблемных контейнеров
rblog-voice          # Детальные логи конкретного сервиса
```

### 4. Разработка и тестирование

```bash
rbcd                 # Перейти в проект
rbgit                # Проверить статус
rbbuild              # Собрать проект
rbtest               # Запустить тесты (если настроены)
```

### 5. Emergency - всё сломалось

```bash
rbemergency          # Остановить всё немедленно
rbnuke               # Полная очистка (volumes + images)
rbfix                # Попытка автоматического исправления
```

## Интеграция с существующими инструментами

Алиасы интегрируются с существующими диагностическими скриптами:

```bash
rbcheck              # → ~/rob_box_project/docker/diagnose_data_flow.sh
rbmonitor            # → ~/rob_box_project/docker/monitor_system.sh
```

## Заключение

Алиасы РОББОКС превращают длинные команды Docker и Git в короткие, запоминающиеся сокращения, ускоряя ежедневную работу с системой.

**Всегда под рукой:** `rbalias` покажет полный список команд! 🚀
