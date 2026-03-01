#!/bin/bash
# Аудит документации проекта
# Проверяет naming conventions и дубликаты

echo "🔍 Аудит документации Rob Box Project"
echo "======================================"
echo ""

# 1. Проверка lowercase файлов в docs/
echo "1️⃣ Проверка lowercase файлов в docs/:"
lowercase_files=$(find docs -name "*.md" -type f | grep -v "^docs/README.md$" | grep -E '[a-z]' | grep -v '[A-Z_]')
if [ -z "$lowercase_files" ]; then
    echo "   ✅ Нет lowercase файлов"
else
    echo "   ❌ Найдены lowercase файлы:"
    echo "$lowercase_files" | sed 's/^/      /'
fi
echo ""

# 2. Проверка MD файлов вне docs/ (кроме корня)
echo "2️⃣ Проверка MD файлов вне docs/:"
outside_docs=$(find . -maxdepth 3 -name "*.md" -type f \
    ! -path "./docs/*" \
    ! -path "./README.md" \
    ! -path "./CONTRIBUTING.md" \
    ! -path "./QUICK_REFERENCE.md" \
    ! -path "./src/*/README.md" \
    ! -path "./host/*/README.md" \
    ! -path "./archive/*" \
    ! -path "./.git/*")
if [ -z "$outside_docs" ]; then
    echo "   ✅ Все документы в docs/"
else
    echo "   ⚠️  Файлы вне docs/:"
    echo "$outside_docs" | sed 's/^/      /'
fi
echo ""

# 3. Поиск возможных дубликатов по имени
echo "3️⃣ Проверка дубликатов по имени:"
duplicates_found=false
cd docs
for file in $(find . -name "*.md" -type f | sed 's|^\./||'); do
    basename=$(basename "$file" .md)
    # Ищем файлы с похожим именем (игнорируя дату)
    base_pattern=$(echo "$basename" | sed 's/_[0-9-]*$//')
    similar=$(find . -name "${base_pattern}*.md" -type f | wc -l)
    if [ $similar -gt 1 ]; then
        echo "   ⚠️  Похожие файлы для $base_pattern:"
        find . -name "${base_pattern}*.md" -type f | sed 's|^\./|      docs/|'
        duplicates_found=true
    fi
done
cd ..
if [ "$duplicates_found" = false ]; then
    echo "   ✅ Дубликаты не найдены"
fi
echo ""

# 4. Статистика по категориям
echo "4️⃣ Статистика документации:"
echo "   📁 getting-started: $(find docs/getting-started -name "*.md" 2>/dev/null | wc -l) файлов"
echo "   📁 guides:          $(find docs/guides -name "*.md" 2>/dev/null | wc -l) файлов"
echo "   📁 reference:       $(find docs/reference -name "*.md" 2>/dev/null | wc -l) файлов"
echo "   📁 development:     $(find docs/development -name "*.md" 2>/dev/null | wc -l) файлов"
echo "   📄 Всего в docs:    $(find docs -name "*.md" 2>/dev/null | wc -l) файлов"
echo ""

# 5. Проверка битых ссылок на документы
echo "5️⃣ Проверка внутренних ссылок (sample):"
broken_links=0
for file in $(find docs -name "*.md" -type f | head -10); do
    # Ищем markdown ссылки на локальные .md файлы
    grep -oE '\[.*\]\([^)]*\.md\)' "$file" 2>/dev/null | while read -r link; do
        target=$(echo "$link" | sed -E 's/.*\(([^)]*\.md)\).*/\1/')
        dir=$(dirname "$file")
        full_path="$dir/$target"
        if [ ! -f "$full_path" ]; then
            echo "   ⚠️  Битая ссылка: $file -> $target"
            broken_links=$((broken_links + 1))
        fi
    done
done
if [ $broken_links -eq 0 ]; then
    echo "   ✅ Ссылки корректны (sample check)"
fi
echo ""

echo "======================================"
echo "✅ Аудит завершен"
