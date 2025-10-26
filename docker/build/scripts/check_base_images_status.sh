#!/bin/bash
# Проверка статуса сборки базовых образов

echo "🔍 Checking base images build status..."
echo ""

# Проверка процесса сборки
if pgrep -f "build_and_push_base_images.sh" > /dev/null; then
    echo "✅ Build process is RUNNING"
    echo "   PID: $(pgrep -f 'build_and_push_base_images.sh')"
    echo ""
else
    echo "❌ Build process is NOT running"
    echo ""
fi

# Проверка логов
if [ -f /tmp/build_base_images.log ]; then
    echo "📋 Last 10 lines from build log:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    tail -10 /tmp/build_base_images.log
    echo ""
fi

# Проверка локального registry
echo "📦 Images in local registry:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
curl -s http://192.168.1.125:5000/v2/krikz/rob_box_base/tags/list | jq -r '.tags[]' 2>/dev/null || echo "❌ Failed to query registry"
echo ""

# Статистика
echo "📊 Expected base images:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
EXPECTED=("ros2-zenoh" "rtabmap" "pcl" "depthai")
FOUND=0

for img in "${EXPECTED[@]}"; do
    if curl -s "http://192.168.1.125:5000/v2/krikz/rob_box_base/tags/list" | grep -q "\"$img\""; then
        echo "✅ $img - FOUND"
        ((FOUND++))
    else
        echo "⏳ $img - BUILDING or MISSING"
    fi
done

echo ""
echo "Progress: $FOUND / ${#EXPECTED[@]} images ready"
echo ""

# Рекомендации
if [ "$FOUND" -eq "${#EXPECTED[@]}" ]; then
    echo "🎉 All base images are ready!"
    echo "   Next step: Restart GitHub runners to use new images"
    echo "   Command: cd ~/rob_box_project/docker/build && docker-compose restart"
elif pgrep -f "build_and_push_base_images.sh" > /dev/null; then
    echo "⏳ Build in progress. Check again in a few minutes."
    echo "   Watch logs: tail -f /tmp/build_base_images.log"
else
    echo "❌ Build not running and images incomplete."
    echo "   Restart build: cd ~/rob_box_project/docker/build && sudo bash scripts/build_and_push_base_images.sh"
fi
