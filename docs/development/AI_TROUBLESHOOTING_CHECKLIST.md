# 🐛 AI Troubleshooting Checklist

**Purpose:** Systematic debugging checklist for common issues in rob_box project.

**How to use:**
1. Find your problem category below
2. Follow checklist step-by-step (don't skip!)
3. Copy relevant terminal commands to AI agent
4. Report results at each step

---

## 📋 Table of Contents

- [🐳 Docker Container Issues](#-docker-container-issues)
- [🤖 ROS2 Node Failures](#-ros2-node-failures)
- [🔧 GitHub Actions CI/CD Failures](#-github-actions-cicd-failures)
- [🎤 Voice Assistant Problems](#-voice-assistant-problems)
- [📹 Camera/OAK-D Issues](#-cameraoak-d-issues)
- [🗺️ RTAB-Map SLAM Issues](#️-rtab-map-slam-issues)
- [🌐 Zenoh Networking Issues](#-zenoh-networking-issues)
- [🎨 Animation Editor Issues](#-animation-editor-issues)
- [⚙️ Build/Compilation Errors](#️-buildcompilation-errors)

---

## 🐳 Docker Container Issues

### Container не запускается или падает сразу

#### ✅ Step 1: Проверить что контейнер существует
```bash
docker ps -a | grep <container_name>
```

**Expected:** Контейнер в списке (может быть Exited)

**If not found:**
- Проверь `docker-compose.yaml` - правильное ли имя сервиса?
- Попробуй `docker images` - есть ли образ?

---

#### ✅ Step 2: Посмотреть логи
```bash
docker logs <container_name> --tail 100
```

**Look for:**
- `Error:` в конце логов
- `ModuleNotFoundError:` - missing Python package
- `RuntimeError:` - configuration issue
- `ConnectionRefused:` - networking issue

**Copy error to AI:**
```
Контейнер <name> падает с ошибкой:
<paste last 20 lines of logs>
```

---

#### ✅ Step 3: Проверить конфигурацию docker-compose.yaml
```bash
cd docker/<main|vision>/
cat docker-compose.yaml | grep -A20 "<service_name>:"
```

**Check:**
- [ ] Образ указан правильно? (`image: ghcr.io/...`)
- [ ] Все volumes смонтированы?
- [ ] Сеть настроена? (`network_mode: host` или custom network)
- [ ] Environment переменные заданы?
- [ ] Для секретов: `env_file: .env.secrets` указан?

---

#### ✅ Step 4: Проверить секреты (если используются)
```bash
ls -la docker/<main|vision>/.env.secrets
cat docker/<main|vision>/.env.secrets
```

**Check:**
- [ ] Файл существует?
- [ ] Все ключи заполнены? (не `your_key_here`)
- [ ] Формат правильный? (`KEY=value` без пробелов)

**If missing:**
```bash
cp docker/<main|vision>/.env.secrets.template docker/<main|vision>/.env.secrets
# Edit .env.secrets with real keys
```

---

#### ✅ Step 5: Пересоздать контейнер
```bash
cd docker/<main|vision>/
docker-compose down <service_name>
docker-compose up -d <service_name>
docker logs <service_name> -f
```

**Expected:** Контейнер стартует без ошибок

---

#### ✅ Step 6: Проверить ресурсы системы
```bash
free -h  # Память
df -h    # Диск
top -n1 -b | head -20  # CPU
```

**Warning signs:**
- RAM > 90% used
- Disk > 95% used
- CPU 100% on all cores

---

### Container запущен но ничего не делает

#### ✅ Step 1: Проверить процессы внутри контейнера
```bash
docker exec <container_name> ps aux
```

**Expected:** Видны процессы `ros2`, `python3`, etc.

**If empty:**
- Entrypoint не запустился
- Проверь `docker logs` - была ли ошибка при старте?

---

#### ✅ Step 2: Проверить ROS2 топики (если ROS контейнер)
```bash
docker exec <container_name> ros2 topic list
```

**Expected:** Список топиков (не пустой)

**If empty:**
- ROS2 не запущен
- Проверь `ROS_DOMAIN_ID` в docker-compose.yaml

---

#### ✅ Step 3: Интерактивная диагностика
```bash
docker exec -it <container_name> bash
# Inside container:
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 node list
exit
```

---

## 🤖 ROS2 Node Failures

### Node падает при запуске

#### ✅ Step 1: Запустить ноду вручную с полными логами
```bash
source ~/voice_ws/install/setup.bash  # или другой workspace
ros2 run <package_name> <node_name> --ros-args --log-level debug
```

**Look for:**
- `ImportError:` - missing Python package
- `ModuleNotFoundError:` - missing dependency
- `ServiceException:` - service not available
- `ParameterException:` - bad config

---

#### ✅ Step 2: Проверить зависимости пакета
```bash
cd ~/voice_ws/src/<package_name>
cat package.xml | grep '<depend>'
```

**Check:**
- [ ] Все `<depend>` пакеты установлены?
- [ ] Версии совместимы с ROS Humble?

**Install missing deps:**
```bash
sudo apt update
sudo apt install ros-humble-<missing-package>
```

---

#### ✅ Step 3: Проверить параметры конфигурации
```bash
cd ~/voice_ws/src/<package_name>/config
ls -la *.yaml
cat config.yaml  # или другой файл конфигурации
```

**Check:**
- [ ] Все обязательные параметры заданы?
- [ ] Пути к файлам правильные?
- [ ] Форматирование YAML корректное? (нет табов, правильные отступы)

---

#### ✅ Step 4: Пересобрать пакет
```bash
cd ~/voice_ws
rm -rf build/<package_name> install/<package_name>
colcon build --packages-select <package_name> --symlink-install
source install/setup.bash
```

**Expected:** `Summary: 1 package finished [X.XXs]`

---

#### ✅ Step 5: Проверить ROS_DOMAIN_ID
```bash
echo $ROS_DOMAIN_ID
```

**Expected:** Число 0-101 (или пусто для default=0)

**If wrong:**
```bash
export ROS_DOMAIN_ID=0
# Add to ~/.bashrc if needed
```

---

### Node запущен но не публикует/не подписывается на топики

#### ✅ Step 1: Проверить список топиков
```bash
ros2 topic list
```

**Expected:** Топик который нужен - в списке

---

#### ✅ Step 2: Проверить тип сообщений
```bash
ros2 topic info /topic_name -v
```

**Check:**
- [ ] Type правильный?
- [ ] Publishers > 0?
- [ ] Subscribers > 0?

---

#### ✅ Step 3: Echo топик
```bash
ros2 topic echo /topic_name --once
```

**Expected:** Сообщение появляется

**If timeout:**
- Publisher не работает
- Проверь логи ноды: `ros2 node info /node_name`

---

#### ✅ Step 4: Проверить QoS профиль
```bash
ros2 topic info /topic_name -v | grep "QoS profile"
```

**Common issue:** Publisher и Subscriber с разными QoS

**Solutions:**
- Используй `sensor_data` для камер/лидаров (best effort)
- Используй `system_default` для команд (reliable)

---

## 🔧 GitHub Actions CI/CD Failures

### Workflow не запустился

#### ✅ Step 1: Проверить что push прошел
```bash
git log --oneline -5
git remote -v
```

**Expected:** Последний коммит = тот что ты запушил

---

#### ✅ Step 2: Проверить имя ветки
```bash
git branch --show-current
```

**Expected:**
- `feature/*` → запустится `build-test` workflow
- `develop` → запустится `build-dev` workflow  
- `main` → запустится `build-main` workflow

**If wrong branch:**
```bash
git checkout -b feature/<name>
git push -u origin feature/<name>
```

---

#### ✅ Step 3: Проверить workflow файл
```bash
cat .github/workflows/*.yml | grep -A5 "on:"
```

**Check:**
- [ ] `push:` секция есть?
- [ ] `branches:` включает твою ветку?

---

### Workflow упал с ошибкой

#### ✅ Step 1: Открыть GitHub Actions
```bash
# В браузере:
https://github.com/krikz/rob_box_project/actions
```

**Click:** На красный/желтый workflow

---

#### ✅ Step 2: Найти failed step

**Look for:**
- ❌ Red X - failed step
- ⚠️ Yellow ! - warning

**Click:** На failed step → expand logs

---

#### ✅ Step 3: Categorize error

**Docker build error:**
```
ERROR [internal] load metadata for docker.io/...
CANCELLED [linux/arm64 X/Y] RUN apt-get install ...
```
→ See "Docker build failures" below

**Lint error:**
```
Error: Process completed with exit code 1
[flake8] ./src/file.py:42: E501 line too long
```
→ Fix code style issue

**Test error:**
```
FAILED tests/test_something.py::test_case
AssertionError: expected X but got Y
```
→ Fix failing test

---

#### ✅ Step 4: Reproduce locally

**For Docker builds:**
```bash
cd docker/<main|vision>/<service>/
docker build -t test .
```

**For linting:**
```bash
pip install flake8 black
flake8 src/
black --check src/
```

**For tests:**
```bash
cd ~/voice_ws
colcon test --packages-select <package>
colcon test-result --verbose
```

---

### Docker build failures in CI

#### ✅ Step 1: Check base image availability
```bash
docker pull ros:humble-ros-base
docker pull ubuntu:22.04
```

**If fails:** Docker Hub может быть недоступен

---

#### ✅ Step 2: Check apt package names
```bash
# В Dockerfile:
grep "apt-get install" docker/*/Dockerfile
```

**Common issues:**
- Package name changed (e.g., `python-` → `python3-`)
- Package removed from Ubuntu 22.04 repos
- Typo in package name

**Test locally:**
```bash
docker run --rm -it ubuntu:22.04 bash
apt-get update
apt-get install <package-name>
```

---

#### ✅ Step 3: Check COPY paths
```bash
grep "COPY" docker/*/Dockerfile
```

**Check:**
- [ ] Файл/директория существует в контексте сборки?
- [ ] Path относительный (не абсолютный)?

---

## 🎤 Voice Assistant Problems

### Dialogue node падает с API key error

#### ✅ Step 1: Проверить .env.secrets
```bash
cat docker/vision/.env.secrets | grep DEEPSEEK
```

**Expected:** `DEEPSEEK_API_KEY=sk-...` (реальный ключ, не placeholder)

**If missing/wrong:**
```bash
nano docker/vision/.env.secrets
# Add: DEEPSEEK_API_KEY=sk-your-real-key
```

---

#### ✅ Step 2: Проверить docker-compose.yaml
```bash
cat docker/vision/docker-compose.yaml | grep -A5 "voice-assistant:"
```

**Check:**
- [ ] `env_file: .env.secrets` указан?

**If missing:**
```yaml
voice-assistant:
  image: ghcr.io/krikz/rob_box:voice-assistant-humble-test
  env_file: .env.secrets  # ADD THIS
```

---

#### ✅ Step 3: Перезапустить контейнер
```bash
cd docker/vision/
docker-compose down voice-assistant
docker-compose up -d voice-assistant
docker logs voice-assistant -f
```

**Expected:** `dialogue_node` стартует без RuntimeError

---

### STT/TTS не работает (Yandex SpeechKit)

#### ✅ Step 1: Проверить Yandex credentials
```bash
cat docker/vision/.env.secrets | grep YANDEX
```

**Expected:**
```
YANDEX_API_KEY=AQVNyour-key-here
YANDEX_FOLDER_ID=b1g...
```

---

#### ✅ Step 2: Проверить интернет из контейнера
```bash
docker exec voice-assistant ping -c3 stt.api.cloud.yandex.net
```

**Expected:** 3 packets received

**If fails:** Networking issue (check Vision Pi internet)

---

#### ✅ Step 3: Тест Yandex API вручную
```bash
export YANDEX_API_KEY="your-key"
export YANDEX_FOLDER_ID="your-folder"

curl -X POST \
  -H "Authorization: Api-Key ${YANDEX_API_KEY}" \
  -d "text=Привет&folderId=${YANDEX_FOLDER_ID}&voice=alena" \
  https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize \
  --output test.ogg

file test.ogg
```

**Expected:** `test.ogg: Ogg data, Opus audio`

---

### Audio device not found (ReSpeaker)

#### ✅ Step 1: Проверить USB устройство на хосте
```bash
lsusb | grep 2886
```

**Expected:** `Bus 001 Device 003: ID 2886:0018 Seeed ReSpeaker`

**If not found:**
- Отключи/подключи USB
- Проверь USB кабель
- Проверь питание Pi

---

#### ✅ Step 2: Проверить ALSA devices
```bash
arecord -l | grep -A2 "ReSpeaker"
```

**Expected:** `card X: ... [seeed-8mic-voicecard]`

---

#### ✅ Step 3: Проверить что контейнер видит устройство
```bash
docker exec voice-assistant ls -la /dev/snd/
```

**Expected:** Список `pcmC*` устройств

**If empty:** Device не пробросился в контейнер
```yaml
# В docker-compose.yaml:
devices:
  - /dev/snd:/dev/snd  # ADD THIS
```

---

## 📹 Camera/OAK-D Issues

### OAK-D контейнер не запускается

#### ✅ Step 1: Проверить USB подключение
```bash
lsusb | grep 03e7
```

**Expected:** `ID 03e7:.... Movidius MyriadX`

---

#### ✅ Step 2: Проверить права доступа
```bash
ls -la /dev/bus/usb/*/*
```

**Expected:** `crw-rw-rw-` (666 permissions)

**If wrong:**
```bash
sudo chmod 666 /dev/bus/usb/*/*
```

---

#### ✅ Step 3: Проверить depthai library
```bash
docker exec oak-d python3 -c "import depthai; print(depthai.__version__)"
```

**Expected:** Version number (e.g., `2.21.0.0`)

---

### Camera топики не публикуются

#### ✅ Step 1: Проверить что нода запущена
```bash
docker exec oak-d ros2 node list
```

**Expected:** `/oak_d_node` в списке

---

#### ✅ Step 2: Проверить топики в контейнере
```bash
docker exec oak-d ros2 topic list | grep camera
```

**Expected:**
```
/camera/rgb/image_raw
/camera/depth/image_rect_raw
/camera/camera_info
```

---

#### ✅ Step 3: Echo топик для теста
```bash
docker exec oak-d ros2 topic echo /camera/rgb/image_raw --once
```

**Expected:** Сообщение с image data

---

## 🗺️ RTAB-Map SLAM Issues

### RTAB-Map не создает карту

#### ✅ Step 1: Проверить входные топики
```bash
docker exec rtabmap ros2 topic list | grep -E "odom|scan|camera"
```

**Required topics:**
- `/odom` (odometry)
- `/scan` (2D lidar) OR `/camera/depth` (3D camera)
- `/camera/rgb/image_raw` (for visual SLAM)

---

#### ✅ Step 2: Проверить параметры RTAB-Map
```bash
docker exec rtabmap ros2 param list /rtabmap
```

**Check:**
- `subscribe_depth: true` (для depth камеры)
- `subscribe_scan: true` (для лидара)
- `frame_id: base_link`

---

#### ✅ Step 3: Проверить TF tree
```bash
docker exec rtabmap ros2 run tf2_ros tf2_echo map base_link
```

**Expected:** Transform matrix

**If fails:** TF не настроен

---

## 🌐 Zenoh Networking Issues

### Топики не перебрасываются через Zenoh

#### ✅ Step 1: Проверить что Zenoh router запущен
```bash
docker ps | grep zenoh
```

**Expected:** `zenoh-router` контейнер Running

---

#### ✅ Step 2: Проверить REST API
```bash
curl http://localhost:8000/@/local/subscriber
```

**Expected:** JSON список подписчиков

---

#### ✅ Step 3: Проверить что ROS2 подключен к Zenoh
```bash
docker exec <container> ros2 topic list
```

**Expected:** Топики с других машин видны

---

#### ✅ Step 4: Проверить Zenoh config
```bash
cat docker/main/zenoh_config.json5
```

**Check:**
- `mode: "router"` или `"peer"`
- `listen: ["tcp/0.0.0.0:7447"]`

---

## 🎨 Animation Editor Issues

### Animation Editor не запускается

#### ✅ Step 1: Проверить зависимости
```bash
python3 -c "import tkinter; import PIL; import numpy; print('✓ OK')"
```

**Expected:** `✓ OK`

**If error:**
```bash
pip install pillow numpy
sudo apt install python3-tk
```

---

#### ✅ Step 2: Проверить путь к анимациям
```bash
ls -la src/rob_box_animations/animations/
```

**Expected:** Директория существует, есть `.json` файлы

---

#### ✅ Step 3: Запустить с debug
```bash
python3 tools/animation_editor/main.py --animations-dir src/rob_box_animations/animations 2>&1 | head -50
```

**Look for:** Tracebacks, Import errors

---

### Анимация не сохраняется

#### ✅ Step 1: Проверить права на запись
```bash
ls -la src/rob_box_animations/animations/
```

**Expected:** Директория writable

---

#### ✅ Step 2: Проверить имя файла
**Rules:**
- Только `a-z`, `0-9`, `_`
- Без пробелов
- Расширение `.json`

---

#### ✅ Step 3: Проверить валидность JSON
```bash
python3 -c "import json; json.load(open('src/rob_box_animations/animations/<name>.json'))"
```

**Expected:** No errors

---

## ⚙️ Build/Compilation Errors

### colcon build fails

#### ✅ Step 1: Проверить что workspace sourced
```bash
echo $AMENT_PREFIX_PATH
```

**Expected:** `/opt/ros/humble` в пути

**If empty:**
```bash
source /opt/ros/humble/setup.bash
```

---

#### ✅ Step 2: Clean build
```bash
cd ~/voice_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

#### ✅ Step 3: Проверить package.xml
```bash
cd ~/voice_ws/src/<package>
xmllint --noout package.xml
```

**Expected:** No errors

---

#### ✅ Step 4: Проверить CMakeLists.txt (для C++ пакетов)
```bash
cat CMakeLists.txt | grep "find_package"
```

**Check:**
- [ ] Все зависимости найдены?
- [ ] Версии правильные?

---

## 💡 General Debugging Tips

### When to ask AI for help

**✅ Ask AI when:**
- Не понимаешь ошибку после 10 минут гугла
- Прошел checklist но проблема не решена
- Нужно объяснение как что-то работает

**❌ Don't ask AI before:**
- Запустил хотя бы одну команду из checklist
- Прочитал логи
- Погуглил error message

---

### How to format question to AI

**Template:**
```
Проблема: <1 sentence>

Компонент: <Docker/ROS2/Build/etc>

Что проверил:
- ✅ Step 1: <result>
- ✅ Step 2: <result>
- ❌ Step 3: <result + error>

Логи:
<paste last 20 lines>

Файлы конфигурации:
<paste relevant yaml/launch/dockerfile>
```

**Example (GOOD):**
```
Проблема: voice-assistant dialogue_node падает при старте

Компонент: Docker container на Vision Pi

Что проверил:
- ✅ Контейнер запущен: docker ps показывает Running
- ✅ .env.secrets существует и содержит DEEPSEEK_API_KEY
- ✅ docker-compose.yaml содержит env_file: .env.secrets
- ❌ Логи показывают: RuntimeError: DEEPSEEK_API_KEY not found

Логи:
[dialogue_node-1] Traceback (most recent call last):
[dialogue_node-1]   File "dialogue_node.py", line 45, in __init__
[dialogue_node-1]     api_key = os.environ["DEEPSEEK_API_KEY"]
[dialogue_node-1] KeyError: 'DEEPSEEK_API_KEY'
...

docker-compose.yaml:
voice-assistant:
  image: ghcr.io/krikz/rob_box:voice-assistant-humble-test
  env_file: .env.secrets
  restart: unless-stopped
```

---

## 🎓 Success Metrics

**After implementing this checklist, you should see:**
- ⏱️ Debug time: 60 min → 15 min (75% reduction)
- 🎯 First-time fix rate: 40% → 80% (2x improvement)
- 🤖 AI context quality: "Ошибка с войсом" → Full diagnostic info

---

## 📚 Related Documentation

- See: [AGENT_GUIDE.md](./AGENT_GUIDE.md) - Основная документация
- See: [AI_CONTEXT_MAP.md](./AI_CONTEXT_MAP.md) - Какие файлы открывать
- See: [AI_DEVELOPMENT_REVIEW.md](./AI_DEVELOPMENT_REVIEW.md) - Общий процесс

---

**Last Updated:** October 16, 2025 (Phase 2, Item 2)
