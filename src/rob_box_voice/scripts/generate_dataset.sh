#!/bin/bash
# 🚀 Быстрый запуск генерации датасета для TTS обучения

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "ROBBOX TTS - Dataset Generation Workflow"
echo "=========================================="
echo ""

# Загрузка секретов
SECRETS_FILE="$SCRIPT_DIR/.env.secrets"

if [ ! -f "$SECRETS_FILE" ]; then
    echo "❌ Файл с секретами не найден: $SECRETS_FILE"
    echo ""
    echo "Создайте файл .env.secrets с содержимым:"
    echo ""
    cat << 'EOF'
# Yandex Cloud API
export YANDEX_API_KEY="ваш_ключ"
export YANDEX_FOLDER_ID="ваш_folder_id"

# DeepSeek API
export DEEPSEEK_API_KEY="sk-ваш_ключ"
EOF
    echo ""
    exit 1
fi

echo "✅ Загрузка секретов из .env.secrets"
source "$SECRETS_FILE"

# Проверка ключей
if [ -z "$DEEPSEEK_API_KEY" ]; then
    echo "❌ DEEPSEEK_API_KEY не установлен"
    exit 1
fi

if [ -z "$YANDEX_API_KEY" ] || [ -z "$YANDEX_FOLDER_ID" ]; then
    echo "❌ YANDEX_API_KEY или YANDEX_FOLDER_ID не установлены"
    exit 1
fi

echo "✅ Все ключи найдены"
echo ""

# Параметры по умолчанию
NUM_QUESTIONS=${1:-200}
OUTPUT_DIR="$SCRIPT_DIR/../dataset"
DATASET_NAME="robbox_voice_$(date +%Y%m%d_%H%M%S)"

echo "Параметры:"
echo "  Количество вопросов: $NUM_QUESTIONS"
echo "  Датасет: $DATASET_NAME"
echo ""

# Шаг 1: Генерация фраз через DeepSeek
echo "=========================================="
echo "[1/3] Генерация фраз через DeepSeek"
echo "=========================================="
echo ""

SENTENCES_FILE="$OUTPUT_DIR/${DATASET_NAME}_sentences.txt"
QA_FILE="$OUTPUT_DIR/${DATASET_NAME}_qa.json"

python3 generate_phrases_deepseek.py \
    --master-prompt ../prompts/master_prompt.txt \
    --output "$SENTENCES_FILE" \
    --num-questions "$NUM_QUESTIONS" \
    --save-qa "$QA_FILE" \
    --delay 1.0

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ Ошибка генерации фраз"
    exit 1
fi

echo ""
echo "✅ Фразы сгенерированы: $SENTENCES_FILE"
echo ""

# Показываем статистику
SENTENCE_COUNT=$(wc -l < "$SENTENCES_FILE")
echo "Статистика:"
echo "  Фраз: $SENTENCE_COUNT"
echo ""

# Предпросмотр
echo "Примеры фраз:"
head -10 "$SENTENCES_FILE" | nl
echo "..."
echo ""

read -p "Продолжить запись голоса через Yandex TTS? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Остановлено пользователем"
    echo "Для продолжения запустите:"
    echo "  python3 record_yandex_voice.py --input $SENTENCES_FILE --output ~/robbox_tts_training/datasets/$DATASET_NAME/"
    exit 0
fi

# Шаг 2: Запись голоса через Yandex TTS
echo ""
echo "=========================================="
echo "[2/3] Запись голоса через Yandex TTS"
echo "=========================================="
echo ""

VOICE_DATASET_DIR="$HOME/robbox_tts_training/datasets/$DATASET_NAME"

# Настройки TTS (точные значения из вашего скрипта)
echo "Настройки Yandex TTS v3:"
echo "  Voice: anton (только v3!)"
echo "  Speed: 0.4 (медленная чёткая речь)"
echo ""

python3 record_yandex_voice.py \
    --input "$SENTENCES_FILE" \
    --output "$VOICE_DATASET_DIR" \
    --voice anton \
    --speed 0.4 \
    --delay 1.5

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ Ошибка записи голоса"
    exit 1
fi

echo ""
echo "✅ Голос записан: $VOICE_DATASET_DIR"
echo ""

# Показываем статистику датасета
if [ -f "$VOICE_DATASET_DIR/metadata.csv" ]; then
    AUDIO_COUNT=$(wc -l < "$VOICE_DATASET_DIR/metadata.csv")
    echo "Статистика датасета:"
    echo "  WAV файлов: $AUDIO_COUNT"
    
    # Вычисляем общую длительность (если есть pandas)
    python3 << EOF 2>/dev/null || true
import pandas as pd
try:
    df = pd.read_csv('$VOICE_DATASET_DIR/metadata.csv', sep='|', names=['filename', 'text', 'duration'])
    total_duration = df['duration'].sum()
    print(f"  Общая длительность: {total_duration/60:.1f} минут")
    
    if total_duration < 600:
        print("  Подход: Voice cloning (fast, quality 3-4/5)")
    elif total_duration < 1800:
        print("  Подход: Fine-tuning (balanced, quality 4/5) ⭐")
    else:
        print("  Подход: Full training (slow, quality 4-5/5)")
except:
    pass
EOF
fi

echo ""

# Шаг 3: Информация о следующих шагах
echo "=========================================="
echo "[3/3] Следующие шаги"
echo "=========================================="
echo ""

echo "✅ Датасет готов для обучения!"
echo ""
echo "Для обучения модели выполните:"
echo ""
echo "1. Проверьте систему (на Dell Precision):"
echo "   cd ../training"
echo "   python3 check_system.py"
echo ""
echo "2. Установите окружение (если ещё не установлено):"
echo "   ./setup_training_env.sh"
echo ""
echo "3. Запустите обучение:"
echo "   source ~/robbox_tts_training/activate.sh"
echo "   python3 train_piper.py \\"
echo "     --dataset $VOICE_DATASET_DIR \\"
echo "     --output ~/robbox_tts_training/models/$DATASET_NAME"
echo ""
echo "4. Мониторинг обучения:"
echo "   tensorboard --logdir ~/robbox_tts_training/models/$DATASET_NAME/logs"
echo "   watch -n 1 nvidia-smi"
echo ""

echo "=========================================="
echo "✅ Workflow завершён!"
echo "=========================================="
