#!/bin/bash
# Скрипт настройки сети Vision Pi через SSH
# Использование: ./setup_vision_pi_network.sh [hostname_or_ip]

set -e

VISION_HOST="${1:-VISION.local}"
VISION_USER="ros2"
VISION_PASS="open"

echo "🔧 Настройка сети Vision Pi на $VISION_HOST"

# Проверка доступности
echo "📡 Проверка доступности..."
if ! ping -c 1 -W 2 "$VISION_HOST" &>/dev/null; then
    echo "❌ Vision Pi недоступен по адресу $VISION_HOST"
    exit 1
fi
echo "✓ Vision Pi доступен"

# Создание netplan конфига
echo "📝 Создание конфига netplan..."
cat > /tmp/netplan-vision.yaml << 'EOF'
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      dhcp-identifier: mac
      optional: true
      dhcp4-overrides:
        route-metric: 100
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
  wifis:
    wlan0:
      dhcp4: true
      dhcp-identifier: mac
      optional: true
      access-points:
        "ROS2":
          password: "1234567890"
      dhcp4-overrides:
        route-metric: 600
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
EOF

# Копирование конфига
echo "📤 Копирование конфига на Vision Pi..."
sshpass -p "$VISION_PASS" scp -o StrictHostKeyChecking=no /tmp/netplan-vision.yaml $VISION_USER@$VISION_HOST:/tmp/

# Применение конфига
echo "⚙️  Применение netplan конфига..."
sshpass -p "$VISION_PASS" ssh -o StrictHostKeyChecking=no $VISION_USER@$VISION_HOST \
    "echo '$VISION_PASS' | sudo -S cp /tmp/netplan-vision.yaml /etc/netplan/50-cloud-init.yaml && \
     echo '$VISION_PASS' | sudo -S netplan apply"

# Добавление USB power и SPI
echo "🔌 Настройка USB power и SPI..."
sshpass -p "$VISION_PASS" ssh -o StrictHostKeyChecking=no $VISION_USER@$VISION_HOST \
    "echo '$VISION_PASS' | sudo -S bash -c \"grep -q 'usb_max_current_enable=1' /boot/firmware/config.txt || echo -e '\\nusb_max_current_enable=1\\ndtparam=spi=on' >> /boot/firmware/config.txt\""

# Проверка результата
echo ""
echo "✅ Настройка завершена!"
echo ""
echo "📊 Состояние сети:"
sshpass -p "$VISION_PASS" ssh -o StrictHostKeyChecking=no $VISION_USER@$VISION_HOST \
    'ip -br addr show eth0 wlan0; echo ""; echo "Маршруты:"; ip route show | grep default' 2>&1 | grep -v "warning:"

echo ""
echo "🎯 Доступ:"
echo "  - Ethernet (приоритет): ssh $VISION_USER@\$(ip addr show eth0 | grep 'inet ' | awk '{print \$2}' | cut -d/ -f1)"
echo "  - WiFi (резерв): ssh $VISION_USER@\$(ip addr show wlan0 | grep 'inet ' | awk '{print \$2}' | cut -d/ -f1)"
echo ""
echo "⚠️  Для применения USB power и SPI нужна перезагрузка: ssh $VISION_USER@$VISION_HOST 'sudo reboot'"
