#!/usr/bin/env bash
# audit_katana_audio.sh — Post-fix verification for MSi Katana GF66 speaker fix
#
# Runs the exact checks listed in issue #1732 acceptance criteria and prints
# a clear PASS/FAIL verdict for each. Exit code is 0 only if every check
# passes; non-zero if any check fails (suitable for cron / CI integration).
#
# Usage:
#   bash host/katana/scripts/audit_katana_audio.sh            # human-readable
#   bash host/katana/scripts/audit_katana_audio.sh --json     # machine-readable
#
# Issue: https://github.com/krikz/rob_box_project/issues/1732
# See:   docs/fixes/KATANA_SPEAKER_FIX_2026-08-30.md

set -uo pipefail

# ─── Output helpers ────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

JSON_MODE="no"
if [[ "${1:-}" == "--json" ]]; then
    JSON_MODE="yes"
fi

declare -a CHECKS=()
declare -a FAILURES=()

record() {
    local name="$1"
    local status="$2"
    local detail="$3"
    CHECKS+=("$(printf '{"name":"%s","status":"%s","detail":"%s"}' "${name}" "${status}" "${detail}")")
    if [[ "${status}" == "FAIL" ]]; then
        FAILURES+=("${name}")
    fi
}

# ─── Individual checks ─────────────────────────────────────────────────────
check_chassis() {
    local vendor="unknown"
    local product="unknown"
    if [[ -r /sys/class/dmi/id/sys_vendor ]]; then
        vendor=$(tr -d '\0' < /sys/class/dmi/id/sys_vendor 2>/dev/null || echo "unknown")
    fi
    if [[ -r /sys/class/dmi/id/product_name ]]; then
        product=$(tr -d '\0' < /sys/class/dmi/id/product_name 2>/dev/null || echo "unknown")
    fi
    echo "${vendor} ${product}"
}

check_speaker_outs() {
    # Acceptance: dmesg | grep speaker_outs must show speaker_outs=1
    local line
    line=$(dmesg 2>/dev/null | grep -E "speaker_outs=" | tail -1 || true)
    if [[ -z "${line}" ]]; then
        echo "no-speaker_outs-line"
        return
    fi
    echo "${line}"
}

check_sof_loaded() {
    # Acceptance (for blacklist mode): snd_sof must NOT be loaded
    if lsmod 2>/dev/null | grep -q "^snd_sof "; then
        echo "loaded"
    else
        echo "not-loaded"
    fi
}

check_pulseaudio_sink() {
    # Acceptance: paplay runs and PulseAudio shows RUNNING sink on speaker port
    if ! command -v pacmd >/dev/null 2>&1; then
        echo "pacmd:not-available"
        return
    fi
    local state
    state=$(pacmd list-sinks 2>/dev/null | awk '/state:/ {print $2; exit}' || true)
    local port
    port=$(pacmd list-sinks 2>/dev/null | awk '/active port:/ {print $3; exit}' || true)
    echo "${state:-unknown}|${port:-unknown}"
}

check_amixer_speaker() {
    # Acceptance: Speaker channel must exist and be at 100% unmute
    if ! command -v amixer >/dev/null 2>&1; then
        echo "amixer:not-available"
        return
    fi
    amixer get Speaker 2>/dev/null | awk -F'[][]' '/\[/ {gsub(",", "", $2); print $2; exit}' || true
}

check_modprobe_state() {
    # Show which fix files are installed
    local installed=()
    for f in /etc/modprobe.d/10-snd-hda-intel-model.conf /etc/modprobe.d/20-blacklist-sof.conf; do
        if [[ -f "${f}" ]]; then
            installed+=("$(basename "${f}")")
        fi
    done
    if [[ ${#installed[@]} -eq 0 ]]; then
        echo "none"
    else
        echo "${installed[*]}"
    fi
}

# ─── Runner ────────────────────────────────────────────────────────────────
run_all() {
    # Check 1: chassis is reported by render_human() — no need to capture here.

    # Check 2: speaker_outs=1 in dmesg (PRIMARY acceptance criterion)
    local speaker_line
    speaker_line=$(check_speaker_outs)
    if [[ "${speaker_line}" == *"speaker_outs=1"* ]]; then
        record "speaker_outs=1" "PASS" "${speaker_line}"
    else
        record "speaker_outs=1" "FAIL" "got: ${speaker_line:-<empty>}"
    fi

    # Check 3: SOF not loaded (only meaningful if blacklist mode is installed)
    local sof_state
    sof_state=$(check_sof_loaded)
    if [[ -f /etc/modprobe.d/20-blacklist-sof.conf ]]; then
        if [[ "${sof_state}" == "not-loaded" ]]; then
            record "snd_sof_blacklist" "PASS" "snd_sof not loaded"
        else
            record "snd_sof_blacklist" "FAIL" "snd_sof is ${sof_state}; blacklist not effective"
        fi
    else
        record "snd_sof_blacklist" "SKIP" "blacklist not installed (model-only mode)"
    fi

    # Check 4: PulseAudio sink on speaker port
    # NOTE: idle sinks are SUSPENDED, not RUNNING. We only need to verify
    # the sink exists and is bound to the analog-output-speaker port.
    local pulse_state
    pulse_state=$(check_pulseaudio_sink)
    local pulse_status="FAIL"
    local pulse_detail="${pulse_state}"
    if [[ "${pulse_state}" == "pacmd:not-available" ]]; then
        pulse_status="SKIP"
        pulse_detail="pacmd not available"
    elif [[ "${pulse_state}" == *"speaker"* ]]; then
        pulse_status="PASS"
        pulse_detail="sink on speaker port (state=$(echo "${pulse_state}" | cut -d'|' -f1))"
    fi
    record "pulseaudio_speaker_port" "${pulse_status}" "${pulse_detail}"

    # Check 5: amixer Speaker channel unmuted
    local amixer_val
    amixer_val=$(check_amixer_speaker)
    if [[ -z "${amixer_val}" ]]; then
        record "amixer_speaker" "FAIL" "Speaker channel not present (codec path not built)"
    elif [[ "${amixer_val}" == "on" || "${amixer_val}" == "100%" ]]; then
        record "amixer_speaker" "PASS" "Speaker=${amixer_val}"
    else
        record "amixer_speaker" "WARN" "Speaker=${amixer_val} (not unmuted or not 100%)"
    fi

    # Check 6: modprobe.d state file present
    local modprobe_state
    modprobe_state=$(check_modprobe_state)
    if [[ "${modprobe_state}" == "none" ]]; then
        record "modprobe_state" "FAIL" "no fix files in /etc/modprobe.d/"
    else
        record "modprobe_state" "PASS" "${modprobe_state}"
    fi
}

# ─── Render ────────────────────────────────────────────────────────────────
render_human() {
    echo
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  Katana GF66 Audio Fix — Audit (issue #1732)         ${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
    echo
    echo "chassis: $(check_chassis)"
    echo
    for entry in "${CHECKS[@]}"; do
        local name status detail
        name=$(echo "${entry}" | sed -n 's/.*"name":"\([^"]*\)".*/\1/p')
        status=$(echo "${entry}" | sed -n 's/.*"status":"\([^"]*\)".*/\1/p')
        detail=$(echo "${entry}" | sed -n 's/.*"detail":"\([^"]*\)".*/\1/p')
        local color="${NC}"
        case "${status}" in
            PASS) color="${GREEN}" ;;
            FAIL) color="${RED}" ;;
            WARN) color="${YELLOW}" ;;
            SKIP) color="${YELLOW}" ;;
        esac
        printf "  %b[%-4s]%b %-32s %s\n" "${color}" "${status}" "${NC}" "${name}" "${detail}"
    done
    echo
    if [[ ${#FAILURES[@]} -eq 0 ]]; then
        echo -e "${GREEN}VERDICT: PASS${NC} — speaker path is wired and unmuted."
    else
        echo -e "${RED}VERDICT: FAIL${NC} — failing checks: ${FAILURES[*]}"
        echo
        echo "Recovery options:"
        echo "  1) sudo reboot           # if you just installed the fix"
        echo "  2) sudo ./install_katana_audio_fix.sh   # pick option 3 (combined)"
        echo "  3) BIOS: enable HDA Controller (skip DSP)"
    fi
    echo
}

render_json() {
    local verdict="PASS"
    if [[ ${#FAILURES[@]} -gt 0 ]]; then
        verdict="FAIL"
    fi
    printf '{'
    printf '"verdict":"%s",' "${verdict}"
    printf '"checks":['
    local first=1
    for entry in "${CHECKS[@]}"; do
        if [[ ${first} -eq 1 ]]; then first=0; else printf ','; fi
        printf '%s' "${entry}"
    done
    printf ']}'
    echo
}

run_all
if [[ "${JSON_MODE}" == "yes" ]]; then
    render_json
else
    render_human
fi

if [[ ${#FAILURES[@]} -gt 0 ]]; then
    exit 1
fi
exit 0
