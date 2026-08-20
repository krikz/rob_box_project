# Zenoh Router (Main Pi) — rmw_zenohd из ROS-образа.
# rmw-zenoh-cpp (включая rmw_zenohd) уже установлен в базовом образе ros2-zenoh.
# Использование rmw_zenohd гарантирует совпадение версии zenoh с RMW
# (в отличие от eclipse/zenoh:1.6.2, который отставал от zenoh-c 1.8.0 в RMW).
ARG ROS_DISTRO=lyrical
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh

FROM ${BASE_IMAGE}

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]
