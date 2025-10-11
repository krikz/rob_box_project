#!/bin/bash
# Проверка битых ссылок в документации
# Ищет все .md ссылки и проверяет существование файлов

echo "🔗 Проверка внутренних ссылок в документации"
echo "============================================="
echo ""

broken_count=0
checked_count=0

# Функция для резолва относительных путей
resolve_path() {
    local base_dir="$1"
    local target="$2"
    
    # Убрать якорь (#...)
    target=$(echo "$target" | sed 's/#.*//')
    
    # Если путь начинается с /, это абсолютный путь от корня проекта
    if [[ "$target" == /* ]]; then
        echo "$target"
    else
        # Относительный путь
        echo "$(cd "$base_dir" && pwd)/$target"
    fi
}

# Проверить все MD файлы в docs/
while IFS= read -r file; do
    # Извлечь все markdown ссылки на .md файлы
    grep -oE '\[([^\]]+)\]\(([^)]+\.md[^)]*)\)' "$file" 2>/dev/null | while read -r match; do
        # Извлечь URL из ссылки
        url=$(echo "$match" | sed -E 's/.*\(([^)]+)\).*/\1/')
        
        # Пропустить внешние ссылки
        if [[ "$url" =~ ^https?:// ]]; then
            continue
        fi
        
        checked_count=$((checked_count + 1))
        
        # Резолвить путь
        dir=$(dirname "$file")
        full_path=$(resolve_path "$dir" "$url")
        
        # Проверить существование
        if [ ! -f "$full_path" ]; then
            echo "❌ $file"
            echo "   → $url"
            echo "   Ожидается: $full_path"
            echo ""
            broken_count=$((broken_count + 1))
        fi
    done
done < <(find docs -name "*.md" -type f)

echo "============================================="
echo "Проверено ссылок: $checked_count"
if [ $broken_count -eq 0 ]; then
    echo "✅ Все ссылки корректны!"
else
    echo "⚠️  Найдено битых ссылок: $broken_count"
fi
