# Deployment Guide - Vision Pi Services

Инструкция по развёртыванию сервисов Vision Pi на Raspberry Pi 5.

## Сборка образов

### Автоматическая сборка (GitHub Actions)

**Основной способ сборки** - через GitHub Actions workflow.

#### Триггеры сборки

1. **Автоматическая сборка при push:**
   ```bash
   git add docker/vision/
   git commit -m "feat: update voice assistant"
   git push origin develop
   ```
   - Сборка запустится автоматически
   - Образ будет: `ghcr.io/krikz/rob_box:voice-assistant-humble-dev`

2. **Ручная сборка:**
   - Перейти на GitHub → Actions → "Build Vision Pi Services"
   - Нажать "Run workflow"
   - Выбрать ветку (develop/main)
   - Нажать "Run workflow"

3. **Сборка при merge:**
   - `develop` → `main`: создаёт тег `-latest`
   - Release branch: создаёт тег `-rc-X.X.X`

#### Теги образов

| Ветка | Тег Docker | Когда использовать |
|-------|------------|-------------------|
| `main` | `humble-latest` | Продакшн деплой |
| `develop` | `humble-dev` | Тестирование новых фич |
| `release/X.X.X` | `humble-rc-X.X.X` | Release candidate |
| SHA commit | `humble-<sha>` | Специфичная версия |

### Локальная сборка (только для тестирования)

```bash
# Voice Assistant
cd /home/ros2/rob_box_project
chmod +x docker/vision/scripts/build_voice_assistant.sh
./docker/vision/scripts/build_voice_assistant.sh

# Результат: rob_box:voice-assistant-humble-test
```

⚠️ **Внимание:** Локально собранные образы НЕ используются в продакшн!

## Развёртывание на Vision Pi

### Первоначальная настройка

1. **Клонирование репозитория:**
   ```bash
   cd ~
   git clone https://github.com/krikz/rob_box_project.git
   cd rob_box_project
   git checkout develop  # или main
   ```

2. **Настройка конфигурации Voice Assistant:**
   ```bash
   cd docker/vision/config/voice
   
   # Скопировать пример секретов
   cp secrets.yaml.example secrets.yaml
   
   # Отредактировать secrets.yaml
   nano secrets.yaml
   
   # Добавить реальные API ключи:
   # - YANDEX_FOLDER_ID
   # - YANDEX_API_KEY
   # - DEEPSEEK_API_KEY
   ```

3. **Проверка конфигурации:**
   ```bash
   # Проверить voice_assistant.yaml
   cat voice_assistant.yaml | grep -A5 "yandex:"
   
   # Убедиться что secrets.yaml не коммитится
   git status  # secrets.yaml должен быть в .gitignore
   ```

### Запуск сервисов

#### Полный стек Vision Pi

```bash
cd ~/rob_box_project/docker/vision

# Скачать последние образы из GitHub Registry
docker-compose pull

# Запуск всех сервисов
docker-compose up -d

# Проверка статуса
docker-compose ps
```

#### Только Voice Assistant

```bash
cd ~/rob_box_project/docker/vision

# Скачать только voice-assistant
docker-compose pull voice-assistant

# Запуск voice-assistant + zenoh-router
docker-compose up -d zenoh-router voice-assistant

# Логи
docker-compose logs -f voice-assistant
```

### Обновление сервисов

#### Обновление до latest версии

```bash
cd ~/rob_box_project/docker/vision

# Остановить сервисы
docker-compose down

# Скачать новые образы
docker-compose pull

# Запустить с новыми образами
docker-compose up -d
```

#### Обновление до dev версии (тестирование)

```bash
cd ~/rob_box_project/docker/vision

# Изменить тег в docker-compose.yaml
sed -i 's/humble-latest/humble-dev/g' docker-compose.yaml

# Скачать dev образы
docker-compose pull

# Запустить
docker-compose up -d

# Вернуть обратно на latest
sed -i 's/humble-dev/humble-latest/g' docker-compose.yaml
```

#### Откат к предыдущей версии

```bash
# Найти SHA предыдущей рабочей версии
docker images | grep voice-assistant

# Изменить тег на конкретный SHA
# В docker-compose.yaml:
# image: ghcr.io/krikz/rob_box:voice-assistant-humble-abc1234

docker-compose up -d voice-assistant
```

### Переключение между ветками

```bash
cd ~/rob_box_project

# Текущая ветка
git branch

# Переключение на develop (новые фичи)
git checkout develop
git pull origin develop
cd docker/vision
docker-compose down
docker-compose pull
docker-compose up -d

# Переключение на main (стабильная)
git checkout main
git pull origin main
cd docker/vision
docker-compose down
docker-compose pull
docker-compose up -d
```

## Мониторинг и отладка

### Проверка статуса

```bash
cd ~/rob_box_project/docker/vision

# Статус контейнеров
docker-compose ps

# Использование ресурсов
docker stats voice-assistant

# Логи real-time
docker-compose logs -f voice-assistant

# Последние 100 строк логов
docker-compose logs --tail 100 voice-assistant
```

### Проверка ReSpeaker

```bash
# Войти в контейнер
docker exec -it voice-assistant bash

# USB устройство
lsusb | grep 2886

# ALSA audio
arecord -l | grep ReSpeaker

# Тест записи (5 секунд)
arecord -D plughw:CARD=ReSpeaker -f S16_LE -r 16000 -c 1 -d 5 test.wav

# Тест LED
python3 << EOF
from rob_box_voice.utils.respeaker_interface import ReSpeakerInterface
dev = ReSpeakerInterface()
print(f"VAD: {dev.read('VOICEACTIVITY')}")
print(f"DoA: {dev.read('DOAANGLE')}")
EOF
```

### ROS2 топики

```bash
# Внутри контейнера или на хосте с ROS2
source /opt/ros/humble/setup.bash

# Список топиков
ros2 topic list

# Voice state
ros2 topic echo /voice/state

# Audio VAD
ros2 topic echo /audio/vad

# Direction (DoA)
ros2 topic echo /audio/direction

# Animation trigger
ros2 topic echo /animations/trigger
```

### Проверка Zenoh

```bash
# Zenoh router status
curl http://localhost:8000/@/local/router

# Список Zenoh peers
curl http://localhost:8000/@/router/local

# ROS2 через Zenoh
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

## Troubleshooting

### Voice Assistant не стартует

```bash
# Проверить логи
docker-compose logs voice-assistant | grep ERROR

# Проверить ReSpeaker подключение
lsusb | grep 2886

# Рестарт контейнера
docker-compose restart voice-assistant

# Пересоздать контейнер
docker-compose down voice-assistant
docker-compose up -d voice-assistant
```

### Нет звука / audio ошибки

```bash
# Проверить ALSA группу
groups | grep audio

# Добавить пользователя в audio группу
sudo usermod -aG audio $USER
# Logout и login

# Проверить audio devices на хосте
aplay -l
arecord -l

# Проверить /dev/snd permissions
ls -la /dev/snd/
```

### ROS2 топики не видны

```bash
# Проверить RMW_IMPLEMENTATION
docker exec voice-assistant env | grep RMW

# Проверить Zenoh router
docker-compose logs zenoh-router

# Restart Zenoh
docker-compose restart zenoh-router
docker-compose restart voice-assistant

# Проверить ROS_DOMAIN_ID
docker exec voice-assistant env | grep ROS_DOMAIN_ID
```

### Out of Memory (OOM)

```bash
# Проверить память
free -h
docker stats

# Увеличить swap (если нужно)
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile  # CONF_SWAPSIZE=2048
sudo dphys-swapfile setup
sudo dphys-swapfile swapon

# Уменьшить memory limit в docker-compose.yaml
# mem_limit: 1.5g
# memswap_limit: 2g
```

### TTS кэш переполнен

```bash
# Проверить размер
du -sh docker/vision/cache/tts/

# Очистить кэш
rm -rf docker/vision/cache/tts/*

# Изменить лимит в voice_assistant.yaml
# cache_max_size_mb: 500
```

## CI/CD Pipeline

### Автоматическая цепочка

1. **Разработка:**
   ```
   feature/voice-assistant → develop (pull request)
   ```
   - GitHub Actions собирает образ с тегом `-dev`
   - Доступен для тестирования на Vision Pi

2. **Release:**
   ```
   develop → release/1.0.0 (pull request)
   ```
   - GitHub Actions собирает образ с тегом `-rc-1.0.0`
   - Release candidate testing

3. **Production:**
   ```
   release/1.0.0 → main (pull request)
   ```
   - GitHub Actions собирает образ с тегом `-latest`
   - Стабильная версия для продакшн

### Мониторинг сборки

1. Перейти на GitHub → Actions
2. Найти workflow "Build Vision Pi Services"
3. Проверить статус последней сборки
4. Просмотреть логи сборки

### Ручной trigger сборки

```bash
# Через GitHub CLI
gh workflow run build-vision-services.yml --ref develop

# Или через web interface
# GitHub → Actions → Build Vision Pi Services → Run workflow
```

## Backup и восстановление

### Backup конфигурации

```bash
cd ~/rob_box_project

# Backup configs + secrets
tar -czf vision-backup-$(date +%Y%m%d).tar.gz \
  docker/vision/config/voice/secrets.yaml \
  docker/vision/config/voice/voice_assistant.yaml \
  docker/vision/cache/tts/

# Скопировать на другую машину
scp vision-backup-*.tar.gz user@backup-server:/backups/
```

### Восстановление

```bash
# Распаковать backup
tar -xzf vision-backup-20251012.tar.gz

# Или восстановить только secrets
cp backup/secrets.yaml docker/vision/config/voice/

# Перезапустить сервисы
cd docker/vision
docker-compose restart voice-assistant
```

## Дополнительная информация

- **Architecture:** [Voice Assistant Architecture](../../src/rob_box_voice/docs/VOICE_ASSISTANT_ARCHITECTURE.md)
- **Hardware:** [Hardware Documentation](../architecture/HARDWARE.md)
- **Package README:** [src/rob_box_voice/README.md](../../src/rob_box_voice/README.md)
- **Docker README:** [docker/vision/README.md](./README.md)
