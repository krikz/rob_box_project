#!/bin/sh
set -eu

SOURCE_DIR="${SOURCE_DIR:-/renardo_samples_bundle}"
TARGET_DIR="${TARGET_DIR:-/target}"
MARKER_FILE="${TARGET_DIR}/.initialized"

mkdir -p "${TARGET_DIR}"

if [ "${FORCE_REINIT:-false}" = "true" ]; then
    rm -f "${MARKER_FILE}"
fi

if [ -f "${MARKER_FILE}" ]; then
    echo "Renardo samples already initialized"
    exit 0
fi

if [ ! -d "${SOURCE_DIR}" ] || [ -z "$(find "${SOURCE_DIR}" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]; then
    echo "WARNING: bundled Renardo samples not found — leaving volume empty"
    exit 0
fi

echo "Initializing Renardo samples volume..."
cp -rp "${SOURCE_DIR}/." "${TARGET_DIR}/"
touch "${MARKER_FILE}"
echo "✓ Renardo samples initialized ($(find "${TARGET_DIR}" -name '*.wav' | wc -l) WAV files)"
