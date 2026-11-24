# Dockerfile для Telegram Bot — операторский интерфейс через Telegram
#
# Легковесный контейнер: python-telegram-bot + rclpy + aiohttp
# Деплоится на Sound Pi рядом с voice-assistant (до ADR-0159 PR-7 жил на Vision Pi)
#
# ВАЖНО: конфиги и скрипты монтируются через volumes, НЕ COPY!
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh

# Общий образ ROS-сообщений (docker/msgs/Dockerfile): src/rob_box_*_msgs
# компилируются ОДИН раз за прогон, а не в каждом образе (rosidl под qemu —
# 4-14 минут на пакет). Тег по хэшу пакетов вычисляет gen_build_matrix.py и
# передаёт build-arg'ом; без него сборка падает сразу на имени образа.
ARG MSGS_IMAGE=MSGS_IMAGE-build-arg-is-required
FROM ${MSGS_IMAGE} AS msgs

FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

LABEL org.opencontainers.image.title="Telegram Bot for Rob Box"
LABEL org.opencontainers.image.description="Operator interface via Telegram for Rob Box autonomous rover"
LABEL org.opencontainers.image.source="https://github.com/krikz/rob_box_project"

# APT через apt-cacher-ng на katana (APT_PROXY передаёт gen_build_matrix.py).
# Базовый образ удаляет свой 02proxy, поэтому без этого блока apt ходил
# напрямую, а прямой путь с katana ненадёжен: GPG error (run 34387765562),
# Hash Sum mismatch у .deb того же размера (run 36421581380, nav2/voice-base).
# Шаблон и причина Pipeline-Depth 0 — docker/base/Dockerfile.ros2-zenoh.
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            printf '%s\n' "Acquire::http::Proxy \"$APT_PROXY\";" 'Acquire::http::Timeout "10";' \
                'Acquire::Retries "3";' 'Acquire::http::Pipeline-Depth "0";' > /etc/apt/apt.conf.d/02proxy; \
            echo "✅ Using APT proxy: $APT_PROXY"; \
        else \
            echo "⚠️  APT proxy not accessible, will use direct connection"; \
        fi \
    fi

# Системные зависимости (минимум)
# ffmpeg — AV-23 (issue #1915): voice_transcode.ogg_to_pcm16k гоняет
# через него OGG/Opus из Telegram в PCM 16k. Базовый образ
# rob_box_base:ros2-zenoh идёт от ros:humble-ros-base и ffmpeg не
# содержит; ffmpeg стоит только в voice_base, от которого этот
# контейнер НЕ наследуется. Без этой строки _find_ffmpeg() бросает
# VoiceTranscodeError на первом же голосовом с робота.
# python3-msgpack — exec_depend rob_box_supervisor (package.xml). Кодек
# /avatar/state (rob_box_supervisor.core.state, AV-14) импортирует msgpack
# на каждом сообщении топика. База ros2-zenoh и pip-ядро rob-box-dialog-core
# его не ставят (ядро: PyYAML, openai, httpx). Это apt, не новая pip-зависимость
# и не сеть сверх apt-get, который здесь уже был (#5053).
# wget/unzip отсюда убраны вместе со ступенью скачивания Vosk (см. ниже):
# больше их в этом образе не использовал никто — ни start_telegram_bot.sh,
# ни healthcheck, ни код ноды.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-msgpack \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*
RUN rm -f /etc/apt/apt.conf.d/02proxy

# Python зависимости
# vosk — локальный офлайн-фолбек STT (issue найден 2026-09-03: у Yandex
# закончился баланс, голосовые в Telegram молчали вместо деградации).
# Тот же контракт, что у stt_node.py в rob_box_voice: Yandex — primary,
# Vosk — fallback без сети/квоты.
RUN pip3 install --no-cache-dir \
    python-telegram-bot[job-queue]==21.* \
    aiohttp>=3.9 \
    Pillow>=10.0 \
    numpy>=1.24 \
    prometheus_client>=0.20.0 \
    vosk>=0.3.45 \
    && pip3 cache purge

# ============================================================================
# Vosk-модель — БОЛЬШЕ НЕ ЗДЕСЬ (Ресурсный пак, ADR-0125)
# ============================================================================
# Здесь была ступень скачивания vosk-model-small-ru-0.22 (архив 44 МиБ,
# 88 МиБ распакованный) — ровно тот же артефакт, который до 21.09.2026 качал
# и voice_base/Dockerfile. Две сборки ходили за одной моделью, два образа
# хранили её копии.
#
# Почему удалена: сборка telegram-bot ходила в сеть и падала ЦЕЛИКОМ при
# недоступном alphacephei.com (цепочка `wget && unzip && test -f` без
# `|| true`), а версия модели в этой строке могла молча разъехаться с
# манифестом Ресурсного пака. Это был пятый путь доставки, не описанный
# нигде: реестр docs/plans/2026-09-15-resource-pack.md §1.1 перечислял
# четыре (ADR-0125 §8 п.3, план §16.5).
#
# Куда переехало: docker/vision/scripts/resource_pack/manifest.yaml, запись
# `vosk-ru-small` (required: hard). Файл кладёт на ХОСТ шаг деплоя
# «[Sound Pi] Ensure STT/TTS models», контейнер получает его bind-mount'ом
# `/opt/rob_box/models:/models:ro` (docker-compose.yaml, telegram-bot — тот
# же том, что уже был у voice-assistant).
#
# Путь ВНУТРИ контейнера не изменился: voice_processor.py:40 читает
# `/models/vosk-model-small-ru-0.22` (дефолт VOSK_MODEL_PATH) — ни одна
# строка Python не правилась.
#
# `mkdir -p /models` тоже не нужен: каталог создаёт bind-mount.
#
# NB: запуск контейнера БЕЗ этого тома — не молчаливая поломка:
# _load_vosk_model() пишет в лог «Vosk model not found at ...» и возвращает
# None (voice_processor.py:133-140), STT остаётся на одном Yandex.

# Копируем ROS 2 пакеты
# rob_box_dialog_msgs — готовый из общего образа msgs (stage `msgs` в шапке),
# а не компиляцией здесь.
WORKDIR /ws
COPY --from=msgs /ws/install /ws/install

# Ядро диалога из публичного пакета (ADR-0167 Ш3, #4932): rob_box_core и
# rob_box_dialog ставятся pip'ом по тегу, а не colcon'ом из src/. Модули ложатся
# в site-packages; telegram_node импортирует их оттуда (avatar_command, phrases).
# Пин совпадает с voice-assistant: тег v0.7.16. Ставится ДО colcon, чтобы сборка
# хоста rob_box_telegram не затёрла ядро копией из src/.
#
# apt python3-pip на базе ros2-zenoh (Ubuntu 22.04) — это pip 22.0.2. Он не
# читает имя из PEP 621-метаданных ядра (pyproject [project], setuptools>=77)
# и пишет PKG-INFO «Name: unknown», после чего сам отбрасывает direct-URL:
# «filename has 'rob-box-dialog-core', but metadata has 'unknown'»
# (run 44544845438, job 135636006910, #4940). voice-assistant не падает:
# voice_base заранее делает `pip3 install --upgrade pip` (Dockerfile:113).
# Тот же приём здесь, отдельным слоем — до установки ядра.
RUN pip3 install --no-cache-dir --upgrade pip
RUN pip3 install --no-cache-dir \
    "rob-box-dialog-core @ git+https://github.com/krikz/rob_box_dialog_core@v0.7.34"

WORKDIR /ws/src
# ADR-0150 PR-2 (#3687): telegram_node публикует типизированный Utterance
# (rob_box_dialog_msgs) — пакет пришёл из общего образа msgs выше.
COPY src/rob_box_telegram /ws/src/rob_box_telegram

# rob_box_supervisor (ament_python, #5053). Пакет уже exec_depend у
# rob_box_telegram (package.xml, AV-14): face_roster_view.py:17 жёстко
# импортирует rob_box_supervisor.face_roster_tool на старте ноды, а
# handlers/commands.py лениво берёт оттуда ros_people_list для /faces.
# База ros2-zenoh этого пакета не содержит (в отличие от voice-assistant,
# от которого собирается образ supervisor). Паттерн COPY — как в
# docker/sound/supervisor/Dockerfile (package.xml/setup.py/setup.cfg/resource,
# затем исходники). Исходников ядра (src/rob_box_core) здесь нет: pip-установка
# rob-box-dialog-core выше остаётся единственной копией в site-packages,
# colcon её не затирает. rob_box_supervisor_msgs в этот пакет не копируем:
# telegram их не импортирует на старте, а ленивый импорт в supervisor_client
# по-прежнему ловит ImportError (комментарий у SERVICE_EXECUTE).
COPY src/rob_box_supervisor/package.xml /ws/src/rob_box_supervisor/
COPY src/rob_box_supervisor/setup.py   /ws/src/rob_box_supervisor/
COPY src/rob_box_supervisor/setup.cfg  /ws/src/rob_box_supervisor/
COPY src/rob_box_supervisor/resource   /ws/src/rob_box_supervisor/resource
COPY src/rob_box_supervisor/rob_box_supervisor /ws/src/rob_box_supervisor/rob_box_supervisor

# Собираем ROS-хост и пакет, без которого нода не импортируется.
# Ядро уже в site-packages и в --packages-select не входит.
WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws/install/setup.sh && \
    colcon build --packages-select rob_box_supervisor rob_box_telegram && \
    rm -rf /ws/log

WORKDIR /ws
