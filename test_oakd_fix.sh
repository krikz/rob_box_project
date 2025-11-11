#!/bin/bash
# Тест для watchdog.sh - проверка базовой логики
# Этот тест проверяет только синтаксис и базовую структуру,
# реальное тестирование возможно только на Vision Pi

set -e

echo "🧪 Тестирование OAK-D watchdog скриптов..."

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
OAKD_SCRIPTS_DIR="$SCRIPT_DIR/docker/vision/scripts/oak-d"

# Тест 1: Проверка наличия всех файлов
echo "📋 Тест 1: Проверка наличия файлов..."
required_files=(
    "setup_usb_power.sh"
    "watchdog.sh"
    "install_watchdog.sh"
    "start_oak_d.sh"
    "oak-d-watchdog.service"
    "README_OAKD_FIX.md"
)

for file in "${required_files[@]}"; do
    if [ ! -f "$OAKD_SCRIPTS_DIR/$file" ]; then
        echo "❌ Файл не найден: $file"
        exit 1
    fi
    echo "  ✅ $file"
done

# Тест 2: Проверка синтаксиса bash скриптов
echo "📋 Тест 2: Проверка синтаксиса bash..."
bash_scripts=(
    "setup_usb_power.sh"
    "watchdog.sh"
    "install_watchdog.sh"
    "start_oak_d.sh"
)

for script in "${bash_scripts[@]}"; do
    if ! bash -n "$OAKD_SCRIPTS_DIR/$script"; then
        echo "❌ Ошибка синтаксиса в: $script"
        exit 1
    fi
    echo "  ✅ $script - синтаксис OK"
done

# Тест 3: Проверка прав на выполнение
echo "📋 Тест 3: Проверка прав на выполнение..."
for script in "${bash_scripts[@]}"; do
    if [ ! -x "$OAKD_SCRIPTS_DIR/$script" ]; then
        echo "❌ Нет прав на выполнение: $script"
        exit 1
    fi
    echo "  ✅ $script - исполняемый"
done

# Тест 4: Проверка структуры systemd service файла
echo "📋 Тест 4: Проверка systemd service..."
service_file="$OAKD_SCRIPTS_DIR/oak-d-watchdog.service"

# Проверяем наличие обязательных секций
required_sections=("\[Unit\]" "\[Service\]" "\[Install\]")
for section in "${required_sections[@]}"; do
    if ! grep -q "$section" "$service_file"; then
        echo "❌ Отсутствует секция в service файле: $section"
        exit 1
    fi
    echo "  ✅ Секция найдена: $section"
done

# Тест 5: Проверка конфигурации камеры
echo "📋 Тест 5: Проверка конфигурации камеры..."
config_file="$SCRIPT_DIR/docker/vision/config/oak-d/oak_d_config.yaml"

if [ ! -f "$config_file" ]; then
    echo "❌ Конфигурационный файл не найден: $config_file"
    exit 1
fi

# Проверяем наличие новых параметров
required_params=("i_usb_chunk_kb" "i_pipeline_dump" "i_calibration_dump")
for param in "${required_params[@]}"; do
    if ! grep -q "$param" "$config_file"; then
        echo "❌ Параметр не найден в конфиге: $param"
        exit 1
    fi
    echo "  ✅ Параметр найден: $param"
done

# Тест 6: Проверка документации
echo "📋 Тест 6: Проверка документации..."
doc_files=(
    "$OAKD_SCRIPTS_DIR/README_OAKD_FIX.md"
    "$SCRIPT_DIR/OAKD_DISCONNECT_FIX_QUICKSTART.md"
)

for doc in "${doc_files[@]}"; do
    if [ ! -f "$doc" ]; then
        echo "❌ Документация не найдена: $doc"
        exit 1
    fi
    # Проверяем что файл не пустой
    if [ ! -s "$doc" ]; then
        echo "❌ Документация пустая: $doc"
        exit 1
    fi
    echo "  ✅ $(basename "$doc")"
done

echo ""
echo "✅ Все тесты пройдены успешно!"
echo ""
echo "📝 Примечание: Полное тестирование возможно только на Vision Pi"
echo "   Для реального тестирования выполните на Vision Pi:"
echo "   cd ~/rob_box_project/docker/vision/scripts/oak-d"
echo "   ./install_watchdog.sh"
