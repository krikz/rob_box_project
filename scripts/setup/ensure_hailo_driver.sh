#!/usr/bin/env bash
# ensure_hailo_driver.sh — recover Hailo PCIe kernel module for the running kernel.
# Never upgrades the kernel; it builds only for uname -r.
#
# Prevention (#3090, docs/reports/HAILO_DKMS_ROOT_CAUSE_2026-09-27.md §4):
# unattended-upgrades ставит новый linux-image-raspi, но без metapackage
# linux-headers-raspi headers нового kernel не приезжают → DKMS postinst не
# может пересобрать hailo_pci.ko → после ребута /dev/hailo0 нет. Поэтому
# скрипт (идемпотентно) держит linux-headers-raspi установленным — тогда
# headers едут вместе с каждым будущим kernel и DKMS пересобирает модуль сам,
# ещё ДО ребута. Шаг best-effort: его сбой (нет сети / apt lock) пишет
# WARNING и не валит recovery текущего драйвера.
#
# Env (для тестов / других flavour'ов ядра):
#   HAILO_DEVICE            — default /dev/hailo0
#   HAILO_KERNEL_IMAGE_META — default linux-image-raspi (не установлен → шаг headers пропускается)
#   HAILO_HEADERS_META      — default linux-headers-raspi
set -euo pipefail

KERNEL="$(uname -r)"
MODULE_CANDIDATES=(hailo_pcie hailo_pci hailo1x_pci)
HAILO_DEVICE="${HAILO_DEVICE:-/dev/hailo0}"
KERNEL_IMAGE_META="${HAILO_KERNEL_IMAGE_META:-linux-image-raspi}"
HEADERS_META="${HAILO_HEADERS_META:-linux-headers-raspi}"

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

device_ready() { [ -e "$HAILO_DEVICE" ]; }

pkg_installed() {
  # shellcheck disable=SC2016  # ${Status} — формат dpkg-query, не shell-переменная
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q '^install ok installed$'
}

# Prevention (#3090): headers-метапакет, чтобы DKMS пересобирал модуль при
# каждом будущем kernel-апгрейде. Идемпотентно: уже стоит → ничего не делаем.
ensure_headers_metapackage() {
  if ! pkg_installed "$KERNEL_IMAGE_META"; then
    log "$KERNEL_IMAGE_META is not installed; skipping $HEADERS_META (other kernel flavour)"
    return 0
  fi
  if pkg_installed "$HEADERS_META"; then
    log "$HEADERS_META is installed; future kernel upgrades will bring headers for DKMS"
    return 0
  fi
  log "$HEADERS_META is missing: kernel upgrades would arrive without headers and DKMS could not rebuild the Hailo module (#3090); installing"
  if DEBIAN_FRONTEND=noninteractive apt-get install -y "$HEADERS_META" \
     || { apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y "$HEADERS_META"; }; then
    log "installed $HEADERS_META"
  else
    log "WARNING: failed to install $HEADERS_META; the next kernel upgrade may again leave the Hailo module unbuilt"
  fi
}

ensure_headers_metapackage

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

device_ready || die "Hailo module loaded but $HAILO_DEVICE was not created"

if command -v hailortcli >/dev/null 2>&1; then
  hailortcli scan || die "hailortcli scan cannot see the Hailo device"
fi

log "Hailo PCIe driver is healthy on kernel $KERNEL"
