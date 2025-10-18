#!/bin/bash
# Launcher для Animation Editor

# Определить директорию проекта
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

echo "🎨 Starting rob_box Animation Editor..."
echo "Project dir: $PROJECT_DIR"

cd "$PROJECT_DIR"

# Проверить зависимости
if ! python3 -c "import numpy, PIL, yaml" 2>/dev/null; then
    echo "📦 Installing dependencies..."
    pip install -q -r tools/animation_editor/requirements.txt
fi

# Запустить редактор
python3 tools/animation_editor/main.py --animations-dir src/rob_box_animations/animations

echo "👋 Animation Editor closed"
