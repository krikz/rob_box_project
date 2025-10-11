#!/bin/bash

# Конвертация screencast'ов в оптимизированные GIF
# Анимации идут в порядке:
animations=(
    "police"
    "ambulance"
    "fire_truck"
    "road_service"
    "turn_left"
    "turn_right"
    "braking"
    "accelerating"
    "idle"
    "happy"
    "sad"
    "angry"
    "surprised"
    "thinking"
    "talking"
    "victory"
    "sleep"
    "wakeup"
    "charging"
    "low_battery"
    "error"
)

i=0
for webm in "Screencast from"*.webm; do
    if [ $i -lt ${#animations[@]} ]; then
        anim_name="${animations[$i]}"
        output="docs/assets/animations/${anim_name}.gif"
        
        echo "Converting: $webm -> $output"
        
        # Конвертация с палитрой для лучшего качества и меньшего размера
        ffmpeg -i "$webm" \
            -vf "fps=10,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
            -loop 0 \
            "$output" \
            -y \
            -loglevel error
        
        echo "✅ Created: $output"
        ((i++))
    fi
done

echo ""
echo "✅ Conversion complete! Created ${i} GIF files."
ls -lh docs/assets/animations/*.gif | wc -l
