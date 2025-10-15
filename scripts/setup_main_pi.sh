#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🤖 РОББОКС Main Pi Setup
# ═══════════════════════════════════════════════════════════════════════════
# Быстрая настройка Main Pi узла
#
# Использование:
#   curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_main_pi.sh | bash
# ═══════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NEW_SCRIPT="$SCRIPT_DIR/setup_node.sh"

if [ ! -f "$NEW_SCRIPT" ]; then
    curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s main
else
    bash "$NEW_SCRIPT" main
fi
