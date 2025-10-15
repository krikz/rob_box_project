#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🤖 РОББОКС - Bash Aliases
# ═══════════════════════════════════════════════════════════════════════════
# Полезные алиасы для работы с системой РОББОКС
#
# Установка:
#   source ~/rob_box_project/scripts/robbox_aliases.sh
#
# Или добавить в ~/.bashrc:
#   if [ -f ~/rob_box_project/scripts/robbox_aliases.sh ]; then
#       source ~/rob_box_project/scripts/robbox_aliases.sh
#   fi
# ═══════════════════════════════════════════════════════════════════════════

# Цвета для вывода
export ROBBOX_COLOR_GREEN='\033[0;32m'
export ROBBOX_COLOR_BLUE='\033[0;34m'
export ROBBOX_COLOR_CYAN='\033[0;36m'
export ROBBOX_COLOR_NC='\033[0m'

# ═══════════════════════════════════════════════════════════════════════════
# 🐳 DOCKER MANAGEMENT
# ═══════════════════════════════════════════════════════════════════════════

alias rbstop='docker compose down'
alias rbstart='docker compose up -d'
alias rbrestart='docker compose restart'
alias rbstatus='docker compose ps'
alias rblogs='docker compose logs -f'
alias rbpull='docker compose pull'

# Навигация
alias rbvision='cd ~/rob_box_project/docker/vision'
alias rbmain='cd ~/rob_box_project/docker/main'

# Логи сервисов Vision Pi
alias rblog-oak='docker compose logs -f oak-d'
alias rblog-lidar='docker compose logs -f lslidar'
alias rblog-led='docker compose logs -f led-matrix'
alias rblog-voice='docker compose logs -f voice-assistant'
alias rblog-apriltag='docker compose logs -f apriltag'

# Логи сервисов Main Pi
alias rblog-agent='docker compose logs -f micro-ros-agent'
alias rblog-zenoh='docker compose logs -f zenoh-router'

# Полное обновление
alias rbupdate='rbstop && git pull origin develop && docker compose pull && rbstart'

# Остановка всех узлов
alias rbstop-all='cd ~/rob_box_project/docker/vision && docker compose down; cd ~/rob_box_project/docker/main && docker compose down'
alias rbstart-all='cd ~/rob_box_project/docker/vision && docker compose up -d; cd ~/rob_box_project/docker/main && docker compose up -d'

# ═══════════════════════════════════════════════════════════════════════════
# 🔍 МОНИТОРИНГ И ДИАГНОСТИКА
# ═══════════════════════════════════════════════════════════════════════════

alias rbcheck='~/rob_box_project/docker/diagnose_data_flow.sh'
alias rbmonitor='~/rob_box_project/docker/monitor_system.sh'
alias rbfailing='docker ps -a --filter "status=restarting" --format "table {{.Names}}\t{{.Status}}"'
alias rbresources='docker stats --no-stream'

# Функция для показа логов проблемных контейнеров
rbdebug() {
    for container in $(docker ps -a --filter "status=restarting" --format "{{.Names}}"); do
        echo -e "${ROBBOX_COLOR_CYAN}=== $container ===${ROBBOX_COLOR_NC}"
        docker logs --tail 50 "$container"
        echo ""
    done
}

# Комплексная проверка здоровья
rbhealth() {
    echo -e "${ROBBOX_COLOR_BLUE}=== Docker Status ===${ROBBOX_COLOR_NC}"
    docker compose ps
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}=== Failing Containers ===${ROBBOX_COLOR_NC}"
    rbfailing
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}=== Resource Usage ===${ROBBOX_COLOR_NC}"
    docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}"
}

# ═══════════════════════════════════════════════════════════════════════════
# 📡 ROS2 КОМАНДЫ
# ═══════════════════════════════════════════════════════════════════════════

alias rbtopics='ros2 topic list'
alias rbnodes='ros2 node list'
alias rbinfo='ros2 topic info'
alias rbecho='ros2 topic echo'

# Быстрый просмотр данных
alias rbcamera='ros2 topic echo /camera/image_raw --once'
alias rblidar='ros2 topic echo /scan --once'
alias rbimu='ros2 topic echo /imu/data --once'

# ═══════════════════════════════════════════════════════════════════════════
# 🔧 GIT MANAGEMENT
# ═══════════════════════════════════════════════════════════════════════════

alias rbcd='cd ~/rob_box_project'
alias rbsrc='cd ~/rob_box_project/src'
alias rbdocker='cd ~/rob_box_project/docker'
alias rbdocs='cd ~/rob_box_project/docs'

alias rbgit='cd ~/rob_box_project && git status'
alias rbpullgit='cd ~/rob_box_project && git pull origin develop'
alias rbpush='cd ~/rob_box_project && git push origin'
alias rbdiff='cd ~/rob_box_project && git diff'
alias rblog='cd ~/rob_box_project && git log --oneline -10'

# Синхронизация с GitHub
rbsync() {
    echo -e "${ROBBOX_COLOR_CYAN}Останавливаем контейнеры...${ROBBOX_COLOR_NC}"
    rbstop
    echo -e "${ROBBOX_COLOR_CYAN}Обновляем код...${ROBBOX_COLOR_NC}"
    cd ~/rob_box_project && git pull origin develop
    echo -e "${ROBBOX_COLOR_CYAN}Обновляем образы...${ROBBOX_COLOR_NC}"
    docker compose pull
    echo -e "${ROBBOX_COLOR_CYAN}Запускаем контейнеры...${ROBBOX_COLOR_NC}"
    rbstart
    echo -e "${ROBBOX_COLOR_GREEN}✓ Готово!${ROBBOX_COLOR_NC}"
    rbstatus
}

# ═══════════════════════════════════════════════════════════════════════════
# 🛠️ COLCON BUILD
# ═══════════════════════════════════════════════════════════════════════════

alias rbbuild='cd ~/rob_box_project && colcon build --symlink-install'
alias rbbuild-voice='cd ~/rob_box_project && colcon build --packages-select rob_box_voice --symlink-install'
alias rbbuild-animations='cd ~/rob_box_project && colcon build --packages-select rob_box_animations --symlink-install'

alias rbclean='cd ~/rob_box_project && rm -rf build install log'
alias rbrebuild='rbclean && rbbuild'
alias rbsource='source ~/rob_box_project/install/setup.bash'

# ═══════════════════════════════════════════════════════════════════════════
# 📊 СИСТЕМНЫЕ УТИЛИТЫ
# ═══════════════════════════════════════════════════════════════════════════

# USB устройства (ReSpeaker, OAK-D, LSLidar)
alias rbusb='lsusb | grep -E "(2886|03e7|10c4)"'

# Температура и напряжение (Raspberry Pi)
rbtemp() {
    if command -v vcgencmd &> /dev/null; then
        echo -e "${ROBBOX_COLOR_BLUE}Temperature:${ROBBOX_COLOR_NC} $(vcgencmd measure_temp | cut -d'=' -f2)"
        echo -e "${ROBBOX_COLOR_BLUE}Core Voltage:${ROBBOX_COLOR_NC} $(vcgencmd measure_volts core | cut -d'=' -f2)"
    else
        echo "vcgencmd not available (not a Raspberry Pi?)"
    fi
}

# Системная нагрузка
rbload() {
    echo -e "${ROBBOX_COLOR_BLUE}=== Uptime ===${ROBBOX_COLOR_NC}"
    uptime
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}=== Memory ===${ROBBOX_COLOR_NC}"
    free -h
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}=== Disk ===${ROBBOX_COLOR_NC}"
    df -h /
}

# Сеть
alias rbip='ip -4 addr show | grep inet'
alias rbping='ping -c 3 10.1.1.10 && ping -c 3 10.1.1.11'

# ROS процессы
alias rbps='ps aux | grep ros2'
alias rbkill='pkill -9 -f ros2'

# ═══════════════════════════════════════════════════════════════════════════
# 🔥 EMERGENCY
# ═══════════════════════════════════════════════════════════════════════════

alias rbemergency='docker stop $(docker ps -q)'
alias rbnuke='docker compose down -v && docker system prune -af'

rbfix() {
    echo -e "${ROBBOX_COLOR_CYAN}Перезапускаем проблемные контейнеры...${ROBBOX_COLOR_NC}"
    for container in $(docker ps -a --filter "status=restarting" --format "{{.Names}}"); do
        echo -e "  Restarting: $container"
        docker compose restart "$container"
    done
    echo -e "${ROBBOX_COLOR_GREEN}✓ Готово!${ROBBOX_COLOR_NC}"
}

# ═══════════════════════════════════════════════════════════════════════════
# 📚 СПРАВКА
# ═══════════════════════════════════════════════════════════════════════════

rbalias() {
    echo -e "${ROBBOX_COLOR_CYAN}🤖 РОББОКС - Доступные алиасы:${ROBBOX_COLOR_NC}"
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}Docker:${ROBBOX_COLOR_NC}"
    echo "  rbstart, rbstop, rbrestart, rbstatus, rblogs, rbpull"
    echo "  rbvision, rbmain, rbupdate, rbsync"
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}Мониторинг:${ROBBOX_COLOR_NC}"
    echo "  rbhealth, rbcheck, rbmonitor, rbfailing, rbdebug, rbresources"
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}ROS2:${ROBBOX_COLOR_NC}"
    echo "  rbtopics, rbnodes, rbinfo, rbecho, rbcamera, rblidar, rbimu"
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}Git:${ROBBOX_COLOR_NC}"
    echo "  rbcd, rbgit, rbpullgit, rbpush, rbdiff, rblog"
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}Build:${ROBBOX_COLOR_NC}"
    echo "  rbbuild, rbbuild-voice, rbbuild-animations, rbclean, rbrebuild"
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}System:${ROBBOX_COLOR_NC}"
    echo "  rbusb, rbtemp, rbload, rbip, rbping, rbps, rbkill"
    echo ""
    echo -e "${ROBBOX_COLOR_BLUE}Emergency:${ROBBOX_COLOR_NC}"
    echo "  rbemergency, rbnuke, rbfix"
    echo ""
    echo -e "Полная документация: ${ROBBOX_COLOR_GREEN}~/rob_box_project/docs/guides/BASH_ALIASES.md${ROBBOX_COLOR_NC}"
}

rbhelp() {
    cat ~/rob_box_project/docs/guides/BASH_ALIASES.md 2>/dev/null || echo "Документация не найдена"
}

# ═══════════════════════════════════════════════════════════════════════════
# 🎉 WELCOME MESSAGE
# ═══════════════════════════════════════════════════════════════════════════

echo -e "${ROBBOX_COLOR_GREEN}✓ РОББОКС aliases loaded${ROBBOX_COLOR_NC}"
echo -e "  Type ${ROBBOX_COLOR_CYAN}rbalias${ROBBOX_COLOR_NC} to see available commands"
