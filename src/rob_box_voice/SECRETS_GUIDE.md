# 🔐 Безопасное использование API ключей

Инструкция по работе с секретными ключами для TTS обучения.

## ⚠️ Важно: Безопасность

**НИКОГДА не коммитьте API ключи в Git!**

Все файлы с секретами добавлены в `.gitignore`:
- `*.secrets`
- `.env.secrets`
- `**/secrets/`
- `**/.env.local`

## 📋 Ваши ключи

Создан файл `src/rob_box_voice/.env.secrets` с вашими ключами:

```bash
# Yandex Cloud API
export YANDEX_API_KEY="AQVNxxxxxxxxxxxxxxxxxxxxxxxxxx"
export YANDEX_FOLDER_ID="ajeqxxxxxxxxxxxx"

# DeepSeek API
export DEEPSEEK_API_KEY="sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
```

**Этот файл НЕ будет закоммичен** (защищён .gitignore).

## 🚀 Использование

### Вариант 1: Автоматический workflow (рекомендуется)

Используйте скрипт `generate_dataset.sh` - он автоматически загрузит ключи:

```bash
cd src/rob_box_voice/scripts

# Генерация 200 фраз (для fine-tuning)
chmod +x generate_dataset.sh
./generate_dataset.sh 200

# Или больше для full training
./generate_dataset.sh 500
```

Скрипт автоматически:
1. ✅ Загрузит секреты из `.env.secrets`
2. ✅ Проверит наличие всех ключей
3. ✅ Сгенерирует фразы через DeepSeek
4. ✅ Запишет голос через Yandex TTS
5. ✅ Покажет следующие шаги для обучения

### Вариант 2: Ручной запуск отдельных скриптов

```bash
cd src/rob_box_voice/scripts

# Загрузите секреты в текущую сессию
source ../.env.secrets

# Теперь можете запускать скрипты

# 1. Генерация фраз через DeepSeek
python3 generate_phrases_deepseek.py \
  --master-prompt ../prompts/master_prompt.txt \
  --output ../dataset/my_sentences.txt \
  --num-questions 200

# 2. Запись голоса через Yandex
python3 record_yandex_voice.py \
  --input ../dataset/my_sentences.txt \
  --output ~/robbox_tts_training/datasets/my_voice/

# 3. Обучение модели
cd ../training
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/my_voice \
  --output ~/robbox_tts_training/models/my_model
```

### Вариант 3: Переменные окружения напрямую

```bash
# Экспортируйте ключи вручную (для одной сессии)
export YANDEX_API_KEY="AQVNxxxxxxxxxxxxxxxxxxxxxxxxxx"
export YANDEX_FOLDER_ID="ajeqxxxxxxxxxxxx"
export DEEPSEEK_API_KEY="sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

# Запускайте скрипты
python3 generate_phrases_deepseek.py ...
python3 record_yandex_voice.py ...
```

## 🔍 Проверка ключей

Убедитесь что ключи загружены:

```bash
# Загрузите секреты
source src/rob_box_voice/.env.secrets

# Проверьте
echo "Yandex API Key: ${YANDEX_API_KEY:0:10}..."
echo "Yandex Folder ID: $YANDEX_FOLDER_ID"
echo "DeepSeek API Key: ${DEEPSEEK_API_KEY:0:10}..."
```

Должно вывести:

```
Yandex API Key: AQVNxxxxxx...
Yandex Folder ID: ajeqxxxxxxxxxxxx
DeepSeek API Key: sk-xxxxxxxx...
```

## 🛡️ Безопасность

### Что защищено

Файл `.env.secrets` добавлен в `.gitignore`:

```bash
# Проверьте что файл игнорируется
git status src/rob_box_voice/.env.secrets
# Должно показать: "nothing to commit"
```

### Если случайно закоммитили

Если вы случайно закоммитили секреты:

```bash
# 1. Удалите файл из истории Git
git filter-branch --force --index-filter \
  "git rm --cached --ignore-unmatch src/rob_box_voice/.env.secrets" \
  --prune-empty --tag-name-filter cat -- --all

# 2. Force push (ОСТОРОЖНО!)
git push origin --force --all

# 3. НЕМЕДЛЕННО смените ключи:
#    - Yandex: https://cloud.yandex.ru/docs/iam/operations/api-key/delete
#    - DeepSeek: https://platform.deepseek.com/api_keys
```

### Альтернатива: Используйте переменные окружения системы

Вместо файла `.env.secrets` можно добавить в `~/.bashrc`:

```bash
# Добавьте в конец ~/.bashrc (замените на свои ключи!)
export YANDEX_API_KEY="AQVNxxxxxxxxxxxxxxxxxxxxxxxxxx"
export YANDEX_FOLDER_ID="ajeqxxxxxxxxxxxx"
export DEEPSEEK_API_KEY="sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

# Перезагрузите shell
source ~/.bashrc
```

Теперь ключи будут доступны во всех терминалах.

## 📖 Полный пример использования

```bash
# 1. Перейдите в директорию скриптов
cd ~/rob_box_project/src/rob_box_voice/scripts

# 2. Запустите автоматический workflow
chmod +x generate_dataset.sh
./generate_dataset.sh 200

# Скрипт:
# - Загрузит ключи из .env.secrets
# - Сгенерирует 200 вопросов через DeepSeek (~5 мин, $0.05)
# - Запишет ~300 фраз через Yandex TTS (~15 мин, бесплатно)
# - Покажет команды для обучения

# 3. На Dell Precision 5540 запустите обучение
cd ../training
python3 check_system.py  # Проверьте GPU
./setup_training_env.sh  # Установите окружение (первый раз)
source ~/robbox_tts_training/activate.sh
python3 train_piper.py \
  --dataset ~/robbox_tts_training/datasets/robbox_voice_20250412_120000 \
  --output ~/robbox_tts_training/models/robbox_piper

# Обучение займёт 1-3 дня (зависит от GPU)
```

## 🔄 Обновление ключей

Если нужно обновить ключи:

```bash
# Отредактируйте файл
nano src/rob_box_voice/.env.secrets

# Или создайте новый
cat > src/rob_box_voice/.env.secrets << 'EOF'
export YANDEX_API_KEY="новый_ключ"
export YANDEX_FOLDER_ID="новый_folder"
export DEEPSEEK_API_KEY="sk-новый_ключ"
EOF

# Перезагрузите секреты в текущую сессию
source src/rob_box_voice/.env.secrets
```

## ❓ FAQ

### Q: Можно ли шарить .env.secrets с командой?

**A:** Да, но **НЕ через Git!** Используйте:
- Encrypted USB флешка
- Защищённый мессенджер (Signal, Telegram с секретным чатом)
- Password manager (1Password, Bitwarden)
- Облачное хранилище с шифрованием

### Q: Что делать если ключи утекли?

**A:** Немедленно:
1. Удалите ключи в API консолях (Yandex, DeepSeek)
2. Создайте новые ключи
3. Обновите `.env.secrets`
4. Проверьте нет ли подозрительной активности в биллинге

### Q: Можно ли использовать разные ключи для разных проектов?

**A:** Да, создайте отдельные файлы:
```bash
src/rob_box_voice/.env.secrets.project1
src/rob_box_voice/.env.secrets.project2

# Загрузите нужный
source src/rob_box_voice/.env.secrets.project1
```

## 📚 См. также

- Получение Yandex API key: https://cloud.yandex.ru/docs/iam/operations/api-key/create
- Получение DeepSeek API key: https://platform.deepseek.com/api_keys
- Yandex TTS лимиты: https://cloud.yandex.ru/docs/speechkit/concepts/limits
- DeepSeek pricing: https://platform.deepseek.com/pricing

---

**Помните: Безопасность важнее удобства! 🔐**
