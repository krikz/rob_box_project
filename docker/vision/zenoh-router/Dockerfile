# Zenoh Router (Vision Pi) — rmw_zenohd из ROS-образа.
# rmw-zenoh-cpp (включая rmw_zenohd) уже установлен в базовом образе ros2-zenoh.
ARG ROS_DISTRO=lyrical
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh

FROM ${BASE_IMAGE}

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]
