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

# Test image tags will be:
# - ghcr.io/krikz/rob_box:voice-assistant-humble-test
# - ghcr.io/krikz/rob_box:oak-d-humble-test
# etc.
