#!/bin/bash

# Перемещение документов согласно структуре проекта

echo "📁 Reorganizing documentation..."

# Development docs
echo "Moving development docs..."
mv -v ANIMATION_*.md docs/development/animations/ 2>/dev/null || true
mv -v TALKING_ANIMATION_V2.md docs/development/animations/ 2>/dev/null || true
mv -v CODE_REVIEW_2025-10-11.md docs/development/ 2>/dev/null || true

# Reference docs
echo "Moving reference docs..."
mv -v FUSION360_*.md docs/reference/ 2>/dev/null || true
mv -v SENSOR_HUB_INTEGRATION.md docs/reference/ 2>/dev/null || true
mv -v VESC_INTEGRATION_PROGRESS.md docs/reference/ 2>/dev/null || true

# Quick reference stays in root (it's for developers)
echo "✅ QUICK_REFERENCE.md stays in root"
echo "✅ CONTRIBUTING.md stays in root"
echo "✅ README.md stays in root"

# Clean up screencasts
echo ""
echo "�� Moving screencasts to archive..."
mkdir -p archive/screencasts
mv -v "Screencast from"*.webm archive/screencasts/ 2>/dev/null || true
mv -v convert_screencasts.sh archive/screencasts/ 2>/dev/null || true

echo ""
echo "✅ Documentation reorganization complete!"
