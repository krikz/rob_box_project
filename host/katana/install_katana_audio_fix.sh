#!/usr/bin/env bash
# install_katana_audio_fix.sh — Persistent fix for MSi Katana GF66 silent speaker
#
# Background (issue #1732):
#   PulseAudio reports sink RUNNING, amixer shows 100% on every channel,
#   but the physical speaker produces no sound. Kernel autoconfig tags
#   Node 0x17 (ALC3266) as `line_outs=1` with `speaker_outs=0`, so the
#   speaker path is never built. hda-verb writes return value=0x0 even
#   under sudo — the SOF DSP layer intercepts HDA verbs.
#
# This installer applies the modprobe.d drop-ins shipped in
# host/katana/modprobe.d/ and rebuilds the initramfs so the new modules
# table is loaded on next boot. It does NOT reboot the machine — the
# human operator does that step manually after reviewing the summary.
#
# Usage:
#   sudo ./install_katana_audio_fix.sh                # interactive (recommended)
#   sudo ./install_katana_audio_fix.sh --model        # only model override
#   sudo ./install_katana_audio_fix.sh --blacklist    # only blacklist SOF
#   sudo ./install_katana_audio_fix.sh --combined     # both (most aggressive)
#   sudo ./install_katana_audio_fix.sh --dry-run      # show what would happen
#
# Rollback:
#   sudo ./install_katana_audio_fix.sh --remove
#   sudo ./install_katana_audio_fix.sh --remove && sudo reboot
#
# Issue: https://github.com/krikz/rob_box_project/issues/1732
# See:   docs/fixes/KATANA_SPEAKER_FIX_2026-08-30.md

set -euo pipefail

# ─── Constants ──────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
REPO_MODPROBE_DIR="${SCRIPT_DIR}/../modprobe.d"
readonly REPO_MODPROBE_DIR
readonly TARGET_DIR="/etc/modprobe.d"
readonly STATE_FILE="/var/lib/rob_box/katana-audio-fix.applied"

# Files to install (relative paths inside modprobe.d/)
readonly FILE_MODEL="10-snd-hda-intel-model.conf"
readonly FILE_BLACKLIST="20-blacklist-sof.conf"

# ─── Helpers ────────────────────────────────────────────────────────────────
log()  { printf '[%(%H:%M:%S)T] %s\n' -1 "$*"; }
warn() { printf '[%(%H:%M:%S)T] WARN: %s\n' -1 "$*" >&2; }
err()  { printf '[%(%H:%M:%S)T] ERROR: %s\n' -1 "$*" >&2; }

die() { err "$*"; exit 1; }

require_root() {
    if [[ ${EUID} -ne 0 ]]; then
        die "this installer must run as root (use sudo)"
    fi
}

check_katana() {
    # Soft check: warn if vendor/product doesn't look like Katana, but do not
    # refuse — some operators want to apply the same fix on a sibling MSI
    # chassis with the same ALC3266 codec.
    local vendor=""
    local product=""
    if [[ -r /sys/class/dmi/id/sys_vendor ]]; then
        vendor=$(cat /sys/class/dmi/id/sys_vendor 2>/dev/null || echo "")
    fi
    if [[ -r /sys/class/dmi/id/product_name ]]; then
        product=$(cat /sys/class/dmi/id/product_name 2>/dev/null || echo "")
    fi
    if [[ -z "${vendor}" || -z "${product}" ]]; then
        warn "could not read DMI info; skipping chassis check"
        return 0
    fi
    log "detected chassis: ${vendor} ${product}"
    if [[ "${vendor}" != *"MICRO-STAR"* && "${vendor}" != *"MSI"* ]]; then
        warn "this installer was written for MSi Katana; you are on '${vendor}'"
        warn "ALC3266 + SOF combination is rare; proceed only if you have the same symptoms"
    fi
}

ensure_target_dir() {
    mkdir -p "${TARGET_DIR}"
}

copy_modprobe_file() {
    local name="$1"
    local src="${REPO_MODPROBE_DIR}/${name}"
    local dst="${TARGET_DIR}/${name}"
    if [[ ! -f "${src}" ]]; then
        die "source file not found: ${src}"
    fi
    cp -f "${src}" "${dst}"
    chmod 0644 "${dst}"
    log "installed: ${dst}"
}

record_state() {
    local mode="$1"
    mkdir -p "$(dirname "${STATE_FILE}")"
    local installed_files
    installed_files=""
    for f in "${TARGET_DIR}"/10-snd-hda-intel-model.conf "${TARGET_DIR}"/20-blacklist-sof.conf; do
        if [[ -f "${f}" ]]; then
            if [[ -z "${installed_files}" ]]; then
                installed_files="$(basename "${f}")"
            else
                installed_files="${installed_files},$(basename "${f}")"
            fi
        fi
    done
    cat > "${STATE_FILE}" <<EOF
mode=${mode}
timestamp=$(date -u +%Y-%m-%dT%H:%M:%SZ)
files=${installed_files}
EOF
    chmod 0644 "${STATE_FILE}"
    log "recorded state: ${STATE_FILE}"
}

run_initramfs_update() {
    if command -v update-initramfs >/dev/null 2>&1; then
        log "running update-initramfs -u (Debian/Ubuntu)"
        update-initramfs -u
    elif command -v dracut >/dev/null 2>&1; then
        log "running dracut -f (Fedora/RHEL)"
        dracut -f
    else
        warn "neither update-initramfs nor dracut found; you must rebuild the initramfs manually"
    fi
}

# ─── Action: install (one of three modes) ──────────────────────────────────
install_model() {
    local dry="${1:-no}"
    log "=== Install mode: model override only ==="
    ensure_target_dir
    if [[ "${dry}" == "yes" ]]; then
        log "DRY-RUN: would copy ${FILE_MODEL} -> ${TARGET_DIR}/"
    else
        copy_modprobe_file "${FILE_MODEL}"
        run_initramfs_update
        record_state "model"
    fi
}

install_blacklist() {
    local dry="${1:-no}"
    log "=== Install mode: blacklist SOF only ==="
    ensure_target_dir
    if [[ "${dry}" == "yes" ]]; then
        log "DRY-RUN: would copy ${FILE_BLACKLIST} -> ${TARGET_DIR}/"
    else
        copy_modprobe_file "${FILE_BLACKLIST}"
        run_initramfs_update
        record_state "blacklist"
    fi
}

install_combined() {
    local dry="${1:-no}"
    log "=== Install mode: combined (model + blacklist) ==="
    ensure_target_dir
    if [[ "${dry}" == "yes" ]]; then
        log "DRY-RUN: would copy BOTH files -> ${TARGET_DIR}/"
    else
        copy_modprobe_file "${FILE_MODEL}"
        copy_modprobe_file "${FILE_BLACKLIST}"
        run_initramfs_update
        record_state "combined"
    fi
}

# ─── Action: remove ────────────────────────────────────────────────────────
remove_all() {
    local dry="${1:-no}"
    log "=== Removing katana audio fix ==="
    for f in "${FILE_MODEL}" "${FILE_BLACKLIST}"; do
        local path="${TARGET_DIR}/${f}"
        if [[ -f "${path}" ]]; then
            if [[ "${dry}" == "yes" ]]; then
                log "DRY-RUN: would remove ${path}"
            else
                rm -f "${path}"
                log "removed: ${path}"
            fi
        else
            log "not present (skip): ${path}"
        fi
    done
    if [[ "${dry}" != "yes" ]]; then
        run_initramfs_update
        rm -f "${STATE_FILE}"
        log "state cleared"
    fi
}

# ─── Action: status ────────────────────────────────────────────────────────
print_status() {
    log "=== Current state ==="
    for f in "${FILE_MODEL}" "${FILE_BLACKLIST}"; do
        local path="${TARGET_DIR}/${f}"
        if [[ -f "${path}" ]]; then
            log "PRESENT: ${path}"
            sed 's/^/    /' "${path}"
        else
            log "absent:  ${path}"
        fi
        echo
    done
    if [[ -f "${STATE_FILE}" ]]; then
        log "state file:"
        sed 's/^/    /' "${STATE_FILE}"
    else
        log "no state file at ${STATE_FILE}"
    fi
}

# ─── Interactive picker ────────────────────────────────────────────────────
pick_mode_interactive() {
    local choice=""
    echo "Pick a fix mode:"
    echo "  1) model override only      (lighter: forces alc269vb-headset-intel pin table)"
    echo "  2) blacklist SOF only       (heavier: drops Intel HDA DSP layer)"
    echo "  3) combined (recommended)   (both: most reliable per issue #1732 audit)"
    echo "  4) remove"
    echo "  5) status"
    echo "  q) quit"
    read -r -p "choice [1-5/q]: " choice
    case "${choice}" in
        1) install_model "no" ;;
        2) install_blacklist "no" ;;
        3) install_combined "no" ;;
        4) remove_all "no" ;;
        5) print_status ;;
        q|Q) log "aborted"; exit 0 ;;
        *) die "invalid choice: ${choice}" ;;
    esac
}

# ─── Entry point ───────────────────────────────────────────────────────────
main() {
    require_root
    check_katana

    local mode="${1:-interactive}"
    case "${mode}" in
        --model)     install_model "no" ;;
        --blacklist) install_blacklist "no" ;;
        --combined)  install_combined "no" ;;
        --remove)    remove_all "no" ;;
        --status)    print_status ;;
        --dry-run)
            shift
            case "${1:-}" in
                --model)     install_model "yes" ;;
                --blacklist) install_blacklist "yes" ;;
                --combined)  install_combined "yes" ;;
                --remove)    remove_all "yes" ;;
                *) die "--dry-run requires one of: --model | --blacklist | --combined | --remove" ;;
            esac
            ;;
        interactive|"")
            pick_mode_interactive
            ;;
        *)
            die "unknown mode: ${mode} (try --model | --blacklist | --combined | --remove | --status | --dry-run)"
            ;;
    esac

    cat <<'EOF'

──────────────────────────────────────────────────────────────────────
Next step: reboot so the new modprobe.d table takes effect.

    sudo reboot

After reboot, verify with:

    bash host/katana/scripts/audit_katana_audio.sh

Issue: https://github.com/krikz/rob_box_project/issues/1732
──────────────────────────────────────────────────────────────────────
EOF
}

main "$@"
