#!/bin/bash
# Download rmw_zenoh 0.1.8 .deb packages from snapshots.ros.org
# HTTP-only mirror — пин 0.1.8 vs 0.1.9 (zenoh 1.6.2 vs 1.8.0)
#
# Usage: download_rmw_debs.sh <dest-dir> [arch]
#   arch default: arm64 (matches CI target). For local amd64 builds pass 'amd64'.
set -euo pipefail

DEST="${1:?Usage: $0 <dest-dir> [arch]}"
ARCH="${2:-arm64}"

mkdir -p "$DEST"

case "$ARCH" in
  amd64) rmw_ts=20260304.205755; vnd_ts=20260304.075447;;
  arm64) rmw_ts=20260307.172628; vnd_ts=20260307.134822;;
  *) echo "unsupported arch: $ARCH"; exit 1;;
esac

RMW_DEB="${DEST}/ros-humble-rmw-zenoh-cpp_0.1.8-1jammy.${rmw_ts}_${ARCH}.deb"
VND_DEB="${DEST}/ros-humble-zenoh-cpp-vendor_0.1.8-1jammy.${vnd_ts}_${ARCH}.deb"

RMW_URL="http://snapshots.ros.org/humble/2026-03-29/ubuntu/pool/main/r/ros-humble-rmw-zenoh-cpp/ros-humble-rmw-zenoh-cpp_0.1.8-1jammy.${rmw_ts}_${ARCH}.deb"
VND_URL="http://snapshots.ros.org/humble/2026-03-29/ubuntu/pool/main/r/ros-humble-zenoh-cpp-vendor/ros-humble-zenoh-cpp-vendor_0.1.8-1jammy.${vnd_ts}_${ARCH}.deb"

echo "Downloading rmw-zenoh-cpp (${ARCH})..."
curl -fsSL --connect-timeout 10 --max-time 120 -o "$RMW_DEB" "$RMW_URL"
echo "  → $RMW_DEB ($(stat -c%s "$RMW_DEB") bytes)"

echo "Downloading zenoh-cpp-vendor (${ARCH})..."
curl -fsSL --connect-timeout 10 --max-time 120 -o "$VND_DEB" "$VND_URL"
echo "  → $VND_DEB ($(stat -c%s "$VND_DEB") bytes)"

# Verify both files are valid Debian archives
for f in "$RMW_DEB" "$VND_DEB"; do
  if head -c 8 "$f" | grep -q "!<arch>"; then
    echo "  ✓ $f: valid Debian archive"
  else
    echo "  ✗ $f: NOT a Debian archive (magic mismatch)"
    head -c 100 "$f" | od -c | head -3
    exit 1
  fi
done

echo "Done."
