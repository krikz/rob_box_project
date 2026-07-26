# Voice Assistant Docker Infrastructure - Summary

## Статус: ✅ Готово к сборке через GitHub Actions

Создана полная Docker инфраструктура для Voice Assistant сервиса на Vision Pi.

## Что сделано

### 1. Docker Image (Dockerfile)
- **Файл:** `docker/vision/voice_assistant/Dockerfile`
- **Base:** `ghcr.io/krikz/rob_box_base:ros2-zenoh-kilted-latest`
- **Компоненты:**
  - ROS2 kilted + Zenoh middleware
  - Audio libraries (PyAudio, PortAudio, ALSA)
  - ReSpeaker drivers (usb_4_mic_array, pixel_ring)
  - **STT (Offline-First):**
    - Vosk (vosk-model-small-ru-0.22, 45 MB) - основной
    - Whisper base (74 MB) - альтернатива
    - Yandex SpeechKit - fallback
  - **TTS (Offline-First):**
    - Piper (ru_RU-dmitri-medium, 63 MB) - основной
    - Piper (ru_RU-irina-medium, 63 MB) - женский голос
    - Silero TTS (torch-based) - альтернатива
    - Yandex Cloud TTS - fallback
  - rob_box_voice package (7 nodes)
  - rob_box_animations package
  - USB udev rules для ReSpeaker

### 2. GitHub Actions Workflow
- **Файл:** `.github/workflows/build-vision-services.yml`
- **Добавлен job:** `build-voice-assistant`
- **Платформа:** `linux/arm64` (Raspberry Pi 5)
- **Триггеры:**
  - Push в `develop` → тег `kilted-dev`
  - Push в `main` → тег `kilted-latest`
  - Manual dispatch
  - Изменения в `docker/vision/**`

### 2.1 CI/CD Pipeline (Auto-merge)
- **Файл:** `.github/workflows/auto-merge-feature-to-develop.yml`
- **Логика:**
  - Feature branch push → Build changed services → Auto-merge to develop → Delete feature branch
  - Теги: `*-kilted-dev` (после мерджа в develop)
- **Файл:** `.github/workflows/auto-merge-to-main.yml`
- **Логика:**
  - Develop push → Build ALL services → Auto-merge to main → Create release tag
  - Теги: `*-kilted-latest` (production)
- **Документация:** `docs/CI_CD_PIPELINE.md`

### 3. Docker Compose Integration
- **Файл:** `docker/vision/docker-compose.yaml`
- **Сервис:** `voice-assistant`
- **Зависимости:** `zenoh-router`
- **Устройства:** `/dev/snd` (audio), `/dev/bus/usb` (ReSpeaker)
- **Volumes:**
  - Config: `./config/voice_assistant/`
  - Sound pack: `../../sound_pack/`
  - TTS cache: `./cache/tts/`
- **Memory limits:** 2GB RAM + 2.5GB swap

### 4. Конфигурация
- **Config:** `docker/vision/config/voice_assistant/voice_assistant.yaml`
  - Все параметры для 7 ROS2 nodes
  - Yandex Cloud STT/TTS settings
  - DeepSeek LLM settings
  - ReSpeaker audio parameters
  - LED colors и effects
- **Secrets:** `docker/vision/config/voice_assistant/secrets.yaml.example`
  - Template для API ключей
  - `.gitignore` настроен

### 5. Startup Script
- **Файл:** `docker/vision/scripts/start_voice_assistant.sh`
- **Функции:**
  - Проверка ReSpeaker USB device
  - Ожидание Zenoh router
  - Запуск ROS2 launch file
  - Health checks

### 6. ROS2 Launch File
- **Файл:** `src/rob_box_voice/launch/voice_assistant.launch.py`
- **Запускает:**
  - `audio_node` - Audio capture + VAD + DoA
  - `led_node` - ReSpeaker 12 RGB LED control
  - `voice_animation_player` - LED matrix animations
  - (Остальные nodes закомментированы до реализации)

### 7. Документация
- **Docker README:** `docker/vision/README.md`
  - Описание всех Vision Pi сервисов
  - Voice Assistant capabilities
  - Конфигурация и запуск
  - Troubleshooting
- **Deployment Guide:** `docker/vision/DEPLOYMENT.md`
  - Инструкции по деплою на Vision Pi
  - CI/CD pipeline
  - Мониторинг и отладка
  - Backup и восстановление

### 8. Build Scripts
- **Файл:** `docker/vision/scripts/build_voice_assistant.sh`
- **Назначение:** Локальное тестирование (не для продакшн)
- **Образ:** `rob_box:voice-assistant-kilted-test`

## Следующие шаги

### 1. Протестировать GitHub Actions сборку + Auto-merge
```bash
# Запушить в feature ветку
git add .
git commit -m "feat(voice): complete Docker infrastructure with CI/CD auto-merge"
git push origin feature/voice-assistant

# GitHub Actions автоматически:
# 1. Соберёт voice-assistant образ (linux/arm64)
# 2. Опубликует в ghcr.io с тегом kilted-dev
# 3. Мерджнет feature/voice-assistant → develop
# 4. Удалит feature ветку
```

### 2. Develop автоматически обновится
После успешного мерджа feature → develop:
```bash
# Образ будет доступен как:
# ghcr.io/krikz/rob_box:voice-assistant-kilted-dev

# Автоматически запустится auto-merge-to-main workflow
# (если все сервисы собрались успешно)
```

### 3. Деплой на Vision Pi (тестирование)
```bash
# На Raspberry Pi 5
cd ~/rob_box_project
git checkout develop
git pull

# Настроить secrets
cd docker/vision/config/voice
cp secrets.yaml.example secrets.yaml
nano secrets.yaml  # Добавить API ключи

# Запустить
cd ~/rob_box_project/docker/vision
docker-compose pull voice-assistant
docker-compose up -d zenoh-router voice-assistant
docker-compose logs -f voice-assistant
```

### 4. Реализация оставшихся nodes (Phase 2-6)
По мере реализации nodes раскомментировать в launch file:
- `stt_node` - Speech-to-Text (Yandex Cloud)
- `tts_node` - Text-to-Speech (Yandex Cloud)
- `dialogue_node` - LLM dialogue (DeepSeek)
- `sound_node` - Sound effects playback
- `command_node` - Robot command execution

Каждая новая фича:
```bash
git checkout -b feature/stt-node
# ... разработка ...
git push origin feature/stt-node
# → Автоматический merge в develop
```

### 5. Продакшн деплой (автоматический)
После полного тестирования на develop:
```bash
# Push в develop запустит auto-merge в main
git push origin develop

# GitHub Actions:
# 1. Соберёт ВСЕ сервисы
# 2. Мерджнет develop → main (если всё успешно)
# 3. Создаст release tag
# 4. Опубликует образы с тегом kilted-latest

# На Vision Pi (production)
git checkout main
git pull
docker-compose pull
docker-compose up -d
```

## Проверка перед push

- [x] Dockerfile создан и валиден
- [x] GitHub Actions workflow обновлён (build-vision-services.yml)
- [x] docker-compose.yaml содержит voice-assistant service
- [x] Конфигурационные файлы созданы
- [x] Startup script executable
- [x] Launch file создан
- [x] Документация написана
- [x] .gitignore настроен для secrets
- [x] TTS cache директория создана
- [x] **CI/CD Pipeline:** Auto-merge feature → develop настроен
- [x] **CI/CD Pipeline:** Auto-merge develop → main настроен
- [x] **CI/CD Pipeline:** Документация создана
- [ ] **TODO:** Протестировать GitHub Actions сборку

## Архитектура

```
Vision Pi (Raspberry Pi 5)
├── zenoh-router (Eclipse Zenoh)
│   └── Bridge → Main Pi
│
├── voice-assistant container
│   ├── audio_node (ReSpeaker capture, VAD, DoA)
│   ├── led_node (12× RGB LED control)
│   ├── voice_animation_player (LED matrix)
│   ├── [stt_node] - TODO Phase 2
│   ├── [tts_node] - TODO Phase 3
│   ├── [dialogue_node] - TODO Phase 4
│   ├── [sound_node] - TODO Phase 5
│   └── [command_node] - TODO Phase 6
│
└── Other Vision services
    ├── oak-d
    ├── lslidar
    ├── apriltag
    └── led-matrix
```

## Ресурсы

- **Architecture:** `docs/development/VOICE_ASSISTANT_ARCHITECTURE.md`
- **STT/TTS Research:** `docs/development/STT_TTS_RESEARCH.md` 📚 **NEW!**
  - Сравнение локальных и облачных решений для русского языка
  - Рекомендации: Vosk + Piper (offline-first)
  - Memory budget: ~1.5GB (fits в 2GB ✅)
  - ROS2 интеграция примеры
- **Hardware:** `docs/HARDWARE.md` (section 3.4)
- **Package:** `src/rob_box_voice/README.md`
- **Install:** `src/rob_box_voice/INSTALL.md`
- **Docker:** `docker/vision/README.md`
- **Deploy:** `docker/vision/DEPLOYMENT.md`
- **CI/CD:** `docs/CI_CD_PIPELINE.md`

---

**Дата создания:** 2025-10-12  
**Автор:** GitHub Copilot  
**Ветка:** feature/voice-assistant  
**Статус:** Ready for GitHub Actions build 🚀
