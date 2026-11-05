#!/bin/bash

# Скрипт для обновления тегов Docker образов в workflows

WORKFLOWS=(
  ".github/workflows/build-base-images.yml"
  ".github/workflows/build-main-services.yml"
  ".github/workflows/build-vision-services.yml"
  ".github/workflows/build-all.yml"
)

for workflow in "${WORKFLOWS[@]}"; do
  echo "Updating $workflow..."
  
  # Добавить needs: determine-tag ко всем jobs (кроме determine-tag)
  # Обновить теги образов с динамическим суффиксом
  
  # Это нужно сделать вручную для каждого workflow
  # так как структура может отличаться
done

echo "Done! Please review changes before committing."
