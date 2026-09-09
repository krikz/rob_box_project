#!/bin/bash
# install_host_scripts.sh - Установка host-скриптов и systemd сервисов на Main Pi
#
# Этот скрипт копирует необходимые файлы на Raspberry Pi и настраивает автозапуск.
#
# Использование:
#   sudo ./install_host_scripts.sh

set -e

# Цвета
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}    Rob Box - Main Pi Host Scripts Installation       ${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

# Проверка root прав
if [ "$EUID" -ne 0 ]; then 
  echo -e "${RED}❌ Этот скрипт требует root привилегий${NC}"
  echo "Запустите: sudo $0"
  exit 1
fi

# Определяем директорию установки
INSTALL_DIR="/opt/rob_box"
SYSTEMD_DIR="/etc/systemd/system"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${GREEN}📋 Configuration:${NC}"
echo "  Script directory:  ${SCRIPT_DIR}"
echo "  Install directory: ${INSTALL_DIR}"
echo "  Systemd directory: ${SYSTEMD_DIR}"
echo ""

# ═══════════════════════════════════════════════════════════════
# 1. Создание директории установки
# ═══════════════════════════════════════════════════════════════
echo -e "${YELLOW}📁 Creating installation directory...${NC}"
mkdir -p "${INSTALL_DIR}"
echo -e "${GREEN}✅ Directory created: ${INSTALL_DIR}${NC}"
echo ""

# ═══════════════════════════════════════════════════════════════
# 2. Копирование скриптов
# ═══════════════════════════════════════════════════════════════
echo -e "${YELLOW}📝 Installing scripts...${NC}"

# setup_can.sh
if [ -f "${SCRIPT_DIR}/setup_can.sh" ]; then
    cp "${SCRIPT_DIR}/setup_can.sh" "${INSTALL_DIR}/setup_can.sh"
    chmod +x "${INSTALL_DIR}/setup_can.sh"
    echo -e "${GREEN}✅ Installed: setup_can.sh${NC}"
else
    echo -e "${RED}❌ Error: setup_can.sh not found!${NC}"
    exit 1
fi

echo ""

# ═══════════════════════════════════════════════════════════════
# 3. Установка systemd сервиса
# ═══════════════════════════════════════════════════════════════
echo -e "${YELLOW}🔧 Installing systemd service...${NC}"

if [ -f "${SCRIPT_DIR}/can-setup.service" ]; then
    cp "${SCRIPT_DIR}/can-setup.service" "${SYSTEMD_DIR}/can-setup.service"
    echo -e "${GREEN}✅ Installed: can-setup.service${NC}"
    
    # Reload systemd daemon
    systemctl daemon-reload
    echo -e "${GREEN}✅ Systemd daemon reloaded${NC}"
    
    # Enable service
    systemctl enable can-setup.service
    echo -e "${GREEN}✅ Service enabled (will start on boot)${NC}"
    
    # Start service now
    echo ""
    echo -e "${YELLOW}🚀 Starting CAN setup service...${NC}"
    if systemctl start can-setup.service; then
        echo -e "${GREEN}✅ CAN setup service started successfully${NC}"
    else
        echo -e "${RED}❌ Failed to start CAN setup service${NC}"
        echo "Check logs: sudo journalctl -u can-setup.service"
        exit 1
    fi
else
    echo -e "${RED}❌ Error: can-setup.service not found!${NC}"
    exit 1
fi

echo ""

# ═══════════════════════════════════════════════════════════════
# 4. Проверка статуса
# ═══════════════════════════════════════════════════════════════
echo -e "${YELLOW}🔍 Checking service status...${NC}"
systemctl status can-setup.service --no-pager || true
echo ""

# ═══════════════════════════════════════════════════════════════
# 5. Проверка CAN интерфейсов
# ═══════════════════════════════════════════════════════════════
echo -e "${YELLOW}🔍 Checking CAN interfaces...${NC}"

for iface in can0 can1; do
    if ip link show ${iface} > /dev/null 2>&1; then
        STATE=$(ip -details link show ${iface} | grep -oP 'state \K\w+' || echo "UNKNOWN")
        echo -e "${GREEN}✅ ${iface}: ${STATE}${NC}"
        ip -details link show ${iface} | grep "bitrate"
    else
        echo -e "${YELLOW}⚠️  ${iface}: NOT FOUND${NC}"
        echo "   Check Device Tree configuration in /boot/config.txt"
    fi
done

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Installation complete!${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""
echo "Installed files:"
echo "  ${INSTALL_DIR}/setup_can.sh"
echo "  ${SYSTEMD_DIR}/can-setup.service"
echo ""
echo "Service commands:"
echo "  sudo systemctl status can-setup   # Check status"
echo "  sudo systemctl restart can-setup  # Restart CAN setup"
echo "  sudo systemctl stop can-setup     # Stop CAN"
echo "  sudo journalctl -u can-setup -f   # View logs"
echo ""
echo "Manual CAN setup:"
echo "  sudo ${INSTALL_DIR}/setup_can.sh can0"
echo "  sudo ${INSTALL_DIR}/setup_can.sh can1"
echo ""
