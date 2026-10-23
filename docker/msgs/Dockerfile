# Общий образ ROS-сообщений робота (src/rob_box_*_msgs).
#
# Собирается ОДИН раз за прогон сборки и раздаётся всем образам, которым
# нужны эти типы: потребитель берёт готовый /ws/install через
#   ARG MSGS_IMAGE
#   FROM ${MSGS_IMAGE} AS msgs
#   COPY --from=msgs /ws/install /ws/install
# Раньше каждый из 6 образов компилировал те же IDL-пакеты сам: rosidl под
# qemu — 4-14 минут на пакет (run 39939981591: rob_box_dialog_msgs 14 мин в
# led-matrix, 8-11 мин в quest/voice-assistant/telegram-bot одновременно).
#
# Тег — msgs-<ros_distro>-<хэш>, хэш считает scripts/ci/gen_build_matrix.py
# по этому Dockerfile, ссылке на базу и всем файлам пакетов из
# docker/build-manifest.yaml (msgs_image.packages). Правка .msg даёт новый
# тег → потребители получают новый MSGS_IMAGE → пересобирают свой слой;
# без правки образ уже лежит в registry и сборку пропускает
# ".github/workflows/L-Build Msgs Image.yml". Список пакетов ниже обязан
# совпадать с манифестом (guard scripts/ci/tests/test_gen_build_matrix.py).
#
# База — ros2-zenoh (ros:humble-ros-base): rosidl-генераторы в ней есть.
# Потребители на базе rtabmap (voice-assistant) — тот же Ubuntu 22.04,
# Python 3.10 и ROS humble, ABI которого стабилен внутри дистрибутива.

ARG BASE_IMAGE=localhost:5000/krikz/rob_box_base:ros2-zenoh-humble

FROM ${BASE_IMAGE} AS build

ARG ROS_DISTRO=humble

WORKDIR /ws
COPY src/rob_box_dialog_msgs /ws/src/rob_box_dialog_msgs
COPY src/rob_box_perception_msgs /ws/src/rob_box_perception_msgs
COPY src/rob_box_supervisor_msgs /ws/src/rob_box_supervisor_msgs

# Без --symlink-install: потребителю уезжает только /ws/install, и в нём
# должны лежать настоящие файлы, а не ссылки в /ws/build (урок led_matrix,
# docs/plans/2026-09-15-builder-runtime-seam.md §2.2).
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select rob_box_dialog_msgs rob_box_perception_msgs rob_box_supervisor_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# В образе — только результат: потребители копируют его COPY --from, базу
# они приносят свою. Так образ весит мегабайты, а не слой ROS целиком.
FROM scratch
COPY --from=build /ws/install /ws/install
