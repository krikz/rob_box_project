# Docker configuration for FEATURE branches (testing)
# This file is automatically loaded when deploying from feature/* branches

ROS_DISTRO=humble
IMAGE_TAG=test
REGISTRY=ghcr.io
REPOSITORY_OWNER=krikz
BASE_IMAGE_PREFIX=${REGISTRY}/${REPOSITORY_OWNER}/rob_box_base
SERVICE_IMAGE_PREFIX=${REGISTRY}/${REPOSITORY_OWNER}/rob_box

# Robot ID for namespace isolation
ROBOT_ID=RBXU100001

# Zenoh router IPs (FA-4: parametrized instead of hardcoded in configs)
ZENOH_MAIN_PI_IP=10.1.1.10
ZENOH_CLOUD_ROUTER=tcp/zenoh.robbox.online:7447

# Test image tags will be:
# - ghcr.io/krikz/rob_box:voice-assistant-humble-test
# - ghcr.io/krikz/rob_box:oak-d-humble-test
# etc.
