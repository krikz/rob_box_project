# Быстрые алиасы для bash

Добавьте эти алиасы в ваш `~/.bashrc` для быстрого доступа к скриптам мониторинга:

```bash
# Навигация в проект
alias cdproj='cd /home/ros2/rob_box_project'

# Скрипты мониторинга питания
alias power-status='cd /home/ros2/rob_box_project && ./docker/scripts/check_power_status.sh'
alias power-usb='cd /home/ros2/rob_box_project && ./docker/scripts/check_usb_devices.sh'
alias power-live='cd /home/ros2/rob_box_project && ./docker/scripts/monitor_power_live.sh'
alias power-help='cd /home/ros2/rob_box_project && ./docker/scripts/power_help.sh'

# Docker compose shortcuts
alias dc-main='cd /home/ros2/rob_box_project/docker/main && docker-compose'
alias dc-vision='cd /home/ros2/rob_box_project/docker/vision && docker-compose'

# Быстрая проверка контейнеров
alias dc-ps-main='cd /home/ros2/rob_box_project/docker/main && docker-compose ps'
alias dc-ps-vision='cd /home/ros2/rob_box_project/docker/vision && docker-compose ps'

# Логи
alias dc-logs-main='cd /home/ros2/rob_box_project/docker/main && docker-compose logs -f'
alias dc-logs-vision='cd /home/ros2/rob_box_project/docker/vision && docker-compose logs -f'

# Перезапуск сервисов
alias dc-restart-main='cd /home/ros2/rob_box_project/docker/main && docker-compose restart'
alias dc-restart-vision='cd /home/ros2/rob_box_project/docker/vision && docker-compose restart'

# Обновление и перезапуск
alias update-main='cd /home/ros2/rob_box_project/docker/main && git pull && docker-compose pull && docker-compose up -d'
alias update-vision='cd /home/ros2/rob_box_project/docker/vision && git pull && docker-compose pull && docker-compose up -d'

# Мониторинг удаленных Pi
alias power-status-main='ssh ros2@10.1.1.20 "/home/ros2/rob_box_project/docker/scripts/check_power_status.sh"'
alias power-status-vision='ssh ros2@10.1.1.21 "/home/ros2/rob_box_project/docker/scripts/check_power_status.sh"'
alias power-usb-vision='ssh ros2@10.1.1.21 "/home/ros2/rob_box_project/docker/scripts/check_usb_devices.sh"'
```

## Установка

```bash
# Добавить алиасы
cat >> ~/.bashrc << 'EOF'

# ===== rob_box_project алиасы =====
alias cdproj='cd /home/ros2/rob_box_project'
alias power-status='cd /home/ros2/rob_box_project && ./docker/scripts/check_power_status.sh'
alias power-usb='cd /home/ros2/rob_box_project && ./docker/scripts/check_usb_devices.sh'
alias power-live='cd /home/ros2/rob_box_project && ./docker/scripts/monitor_power_live.sh'
alias power-help='cd /home/ros2/rob_box_project && ./docker/scripts/power_help.sh'
alias dc-main='cd /home/ros2/rob_box_project/docker/main && docker-compose'
alias dc-vision='cd /home/ros2/rob_box_project/docker/vision && docker-compose'
EOF

# Применить изменения
source ~/.bashrc

# Проверить
power-help
```

## Использование

После установки вы можете просто вводить короткие команды:

```bash
# Проверка питания
power-status

# Проверка USB устройств
power-usb

# Мониторинг в реальном времени
power-live

# Показать справку
power-help

# Docker compose на Main Pi
dc-main ps
dc-main logs -f rtabmap
dc-main restart

# Docker compose на Vision Pi
dc-vision ps
dc-vision logs -f oak-d
dc-vision restart lslidar

# Обновление системы
update-main
update-vision

# Удаленный мониторинг
power-status-main
power-status-vision
power-usb-vision
```

## Для установки на обоих Pi

```bash
# Скрипт для развертывания алиасов на Main Pi и Vision Pi
for host in 10.1.1.20 10.1.1.21; do
  echo "Установка алиасов на $host..."
  ssh ros2@$host 'cat >> ~/.bashrc << '"'"'EOF'"'"'

# ===== rob_box_project алиасы =====
alias cdproj="cd /home/ros2/rob_box_project"
alias power-status="cd /home/ros2/rob_box_project && ./docker/scripts/check_power_status.sh"
alias power-usb="cd /home/ros2/rob_box_project && ./docker/scripts/check_usb_devices.sh"
alias power-live="cd /home/ros2/rob_box_project && ./docker/scripts/monitor_power_live.sh"
alias power-help="cd /home/ros2/rob_box_project && ./docker/scripts/power_help.sh"

# Определить какой это Pi
if ip addr | grep -q "10.1.1.20"; then
  alias dc="cd /home/ros2/rob_box_project/docker/main && docker-compose"
  echo "Main Pi aliases loaded"
elif ip addr | grep -q "10.1.1.21"; then
  alias dc="cd /home/ros2/rob_box_project/docker/vision && docker-compose"
  echo "Vision Pi aliases loaded"
fi
EOF
'
done

echo "Алиасы установлены на обоих Pi!"
echo "Для применения выполните на каждом Pi: source ~/.bashrc"
```
