#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🤖 РОББОКС - Установка Bash Aliases
# ═══════════════════════════════════════════════════════════════════════════
# Автоматическая установка РОББОКС алиасов в систему
# ═══════════════════════════════════════════════════════════════════════════

# Цвета
GREEN='\033[0;32m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

PROJECT_DIR="$HOME/rob_box_project"
ALIASES_FILE="$PROJECT_DIR/scripts/maintenance/robbox_aliases.sh"

echo -e "${CYAN}╔═══════════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                  🤖 РОББОКС - Установка Bash Aliases                     ║${NC}"
echo -e "${CYAN}╚═══════════════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Проверка существования файла
if [ ! -f "$ALIASES_FILE" ]; then
    echo -e "${YELLOW}⚠  Файл алиасов не найден: $ALIASES_FILE${NC}"
    exit 1
fi

# Определение shell конфигов
SHELL_CONFIGS=(
    "$HOME/.bashrc"
    "$HOME/.bash_profile"
    "$HOME/.profile"
)

# Найти существующий shell конфиг
SHELL_CONFIG=""
for config in "${SHELL_CONFIGS[@]}"; do
    if [ -f "$config" ]; then
        SHELL_CONFIG="$config"
        break
    fi
done

if [ -z "$SHELL_CONFIG" ]; then
    echo -e "${YELLOW}⚠  Не найден shell конфиг файл. Создаю ~/.bashrc${NC}"
    SHELL_CONFIG="$HOME/.bashrc"
    touch "$SHELL_CONFIG"
fi

echo -e "${BLUE}Используемый shell конфиг:${NC} $SHELL_CONFIG"
echo ""

# Проверка, не установлены ли уже алиасы
if grep -q "robbox_aliases.sh" "$SHELL_CONFIG"; then
    echo -e "${GREEN}✓ РОББОКС aliases уже установлены в $SHELL_CONFIG${NC}"
    echo ""
    read -p "Хотите переустановить? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Установка отменена${NC}"
        exit 0
    fi
    
    # Удаляем старые записи
    echo -e "${CYAN}Удаляю старые записи...${NC}"
    sed -i '/robbox_aliases.sh/d' "$SHELL_CONFIG"
fi

# Добавление в shell конфиг
echo -e "${CYAN}Добавляю РОББОКС aliases в $SHELL_CONFIG...${NC}"

cat >> "$SHELL_CONFIG" << 'EOF'

# ═══════════════════════════════════════════════════════════════════════════
# 🤖 РОББОКС Bash Aliases
# ═══════════════════════════════════════════════════════════════════════════
if [ -f ~/rob_box_project/scripts/maintenance/robbox_aliases.sh ]; then
    source ~/rob_box_project/scripts/maintenance/robbox_aliases.sh
fi
EOF

echo -e "${GREEN}✓ РОББОКС aliases успешно добавлены!${NC}"
echo ""

# Применение изменений
echo -e "${CYAN}Применяю изменения...${NC}"
source "$ALIASES_FILE"

echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                        ✅ Установка завершена!                            ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Доступные команды:${NC}"
echo ""
echo -e "  ${CYAN}rbalias${NC}     - Показать список всех алиасов"
echo -e "  ${CYAN}rbhelp${NC}      - Открыть полную документацию"
echo -e "  ${CYAN}rbhealth${NC}    - Проверить состояние системы"
echo -e "  ${CYAN}rbstatus${NC}    - Показать статус контейнеров"
echo -e "  ${CYAN}rbupdate${NC}    - Обновить проект и перезапустить"
echo ""
echo -e "${YELLOW}Важно:${NC} Для применения в новых терминалах выполните:"
echo -e "  ${CYAN}source $SHELL_CONFIG${NC}"
echo ""
echo -e "Или просто откройте новый терминал"
echo ""
