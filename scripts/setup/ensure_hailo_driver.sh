#!/usr/bin/env bash
# ensure_hailo_driver.sh — recover Hailo PCIe kernel module for the running kernel.
# Never upgrades the kernel; it builds only for uname -r.
set -euo pipefail

KERNEL="$(uname -r)"
MODULE_CANDIDATES=(hailo_pcie hailo_pci hailo1x_pci)

log() { printf '[hailo-preflight] %s\n' "$*"; }
die() {
  log "ERROR: $*"
  log "kernel=$KERNEL"
  log "modules:"
  dkms status 2>/dev/null || true
  log "hailo packages:"
  dpkg -l 2>/dev/null | grep -E '(^ii|^hi).*hailo|^ii.*hailort' || true
  log "PCI:"
  lspci -nn 2>/dev/null | grep -i hailo || true
  exit 1
}

module_loaded() {
  lsmod | awk '{print $1}' | grep -Eq '^(hailo_pcie|hailo_pci|hailo1x_pci)$'
}

device_ready() { [ -e /dev/hailo0 ]; }

if device_ready && module_loaded; then
  log "Hailo device and driver are already ready for kernel $KERNEL"
  exit 0
fi

log "Hailo device/driver is missing for running kernel $KERNEL; attempting DKMS recovery"

if [ ! -d "/lib/modules/$KERNEL/build" ]; then
  log "Matching kernel headers are missing; installing linux-headers-$KERNEL"
  if ! apt-get install -y "linux-headers-$KERNEL"; then
    apt-get update -qq
    apt-get install -y "linux-headers-$KERNEL" || die "cannot install headers for $KERNEL"
  fi
fi

command -v dkms >/dev/null 2>&1 || die "dkms is not installed"
dkms autoinstall -k "$KERNEL" || die "dkms autoinstall failed for $KERNEL"
depmod -a "$KERNEL"

for module in "${MODULE_CANDIDATES[@]}"; do
  if modprobe "$module" 2>/dev/null; then
    log "loaded module $module"
    break
  fi
done

udevadm settle 2>/dev/null || true

device_ready || die "Hailo module loaded but /dev/hailo0 was not created"

if command -v hailortcli >/dev/null 2>&1; then
  hailortcli scan || die "hailortcli scan cannot see the Hailo device"
fi

log "Hailo PCIe driver is healthy on kernel $KERNEL"
