#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🤖 РОББОКС Vision Pi Setup - Legacy Wrapper
# ═══════════════════════════════════════════════════════════════════════════
# Обертка для обратной совместимости
# Вызывает новый универсальный скрипт setup_node.sh с параметром vision
#
# Использование:
#   curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_vision_pi.sh | bash
# ═══════════════════════════════════════════════════════════════════════════

echo "⚠️  Этот скрипт устарел!"
echo "📢 Перенаправляем на новый универсальный setup_node.sh..."
echo ""

# Определяем путь к новому скрипту
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NEW_SCRIPT="$SCRIPT_DIR/setup_node.sh"

# Если запущено через curl, скачиваем новый скрипт
if [ ! -f "$NEW_SCRIPT" ]; then
    curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s vision
else
    bash "$NEW_SCRIPT" vision
fi
