#!/bin/bash
# Download the pinned packages from the configured ROS APT repository.
# Usage: download_rmw_debs.sh <dest-dir> [arch]
# Refresh the APT index for the requested architecture before running.
set -euo pipefail

DEST="${1:?Usage: $0 <dest-dir> [arch]}"
ARCH="${2:-arm64}"

case "$ARCH" in
  arm64) rmw_version=0.1.11-1jammy.20261106.124730; vendor_version=0.1.11-1jammy.20261027.195405;;
  amd64) rmw_version=0.1.11-1jammy.20261105.191826; vendor_version=0.1.11-1jammy.20261031.173316;;
  *) echo "unsupported arch: $ARCH" >&2; exit 1;;
esac

mkdir -p "$DEST"
cd "$DEST"
apt-get download \
    "ros-humble-rmw-zenoh-cpp:$ARCH=$rmw_version" \
    "ros-humble-zenoh-cpp-vendor:$ARCH=$vendor_version"
