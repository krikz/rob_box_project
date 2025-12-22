#!/usr/bin/env python3
"""
Тест LED драйвера напрямую - бегущий огонек по всем 253 LED
"""
import time
import sys

try:
    from pi5neo import Pi5Neo
except ImportError:
    print("ERROR: pi5neo not installed!")
    print("Install: pip install pi5neo")
    sys.exit(1)

# Конфигурация
SPI_DEVICE = '/dev/spidev0.0'
NUM_LEDS = 253
SPI_SPEED = 1000  # kHz
BRIGHTNESS = 25  # 0-255

def main():
    print("=" * 50)
    print("LED Matrix Direct Test - Running Light")
    print("=" * 50)
    print(f"Device: {SPI_DEVICE}")
    print(f"Total LEDs: {NUM_LEDS}")
    print(f"Speed: {SPI_SPEED} kHz")
    print(f"Brightness: {BRIGHTNESS}/255 ({BRIGHTNESS*100//255}%)")
    print()
    print("LED Layout:")
    print("  [0-24]:   Main Display Panel 0 (5×5)")
    print("  [25-49]:  Main Display Panel 1 (5×5)")
    print("  [50-74]:  Main Display Panel 2 (5×5)")
    print("  [75-99]:  Main Display Panel 3 (5×5)")
    print("  [100-124]: Main Display Panel 4 (5×5)")
    print("  [125-188]: Front Left Panel (8×8)")
    print("  [189-252]: Front Right Panel (8×8)")
    print("=" * 50)
    print()

    # Инициализация
    try:
        neo = Pi5Neo(SPI_DEVICE, NUM_LEDS, SPI_SPEED)
        print("✅ LED matrix initialized")
    except Exception as e:
        print(f"❌ Failed to initialize: {e}")
        sys.exit(1)

    # Цвета для разных секций
    colors = [
        (BRIGHTNESS, 0, 0),      # Красный - Main Panel 0
        (0, BRIGHTNESS, 0),      # Зеленый - Main Panel 1
        (0, 0, BRIGHTNESS),      # Синий - Main Panel 2
        (BRIGHTNESS, BRIGHTNESS, 0),  # Желтый - Main Panel 3
        (BRIGHTNESS, 0, BRIGHTNESS),  # Пурпурный - Main Panel 4
        (0, BRIGHTNESS, BRIGHTNESS),  # Голубой - Front Left
        (BRIGHTNESS, BRIGHTNESS, BRIGHTNESS),  # Белый - Front Right
    ]

    panel_ranges = [
        (0, 25, "Main Panel 0"),
        (25, 50, "Main Panel 1"),
        (50, 75, "Main Panel 2"),
        (75, 100, "Main Panel 3"),
        (100, 125, "Main Panel 4"),
        (125, 189, "Front Left (8×8)"),
        (189, 253, "Front Right (8×8)"),
    ]

    print("Starting running light test...")
    print("Press Ctrl+C to stop")
    print()

    try:
        iteration = 0
        while True:
            iteration += 1
            print(f"\n🔄 Iteration {iteration}")
            
            for panel_idx, (start, end, name) in enumerate(panel_ranges):
                color = colors[panel_idx % len(colors)]
                panel_leds = end - start
                
                print(f"  📍 {name}: LEDs {start}-{end-1} ({panel_leds} LEDs) - Color RGB{color}")
                
                for led in range(start, end):
                    # Очистить все
                    for i in range(NUM_LEDS):
                        neo.set_led_color(i, 0, 0, 0)
                    
                    # Зажечь текущий
                    neo.set_led_color(led, *color)
                    neo.update_strip()
                    
                    # Небольшая задержка
                    time.sleep(0.02)  # 20ms = 50 FPS
                
                # Пауза между панелями
                time.sleep(0.1)
            
            # Пауза между итерациями
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n⏹️  Stopping...")
        
        # Выключить все LED
        print("Turning off all LEDs...")
        for i in range(NUM_LEDS):
            neo.set_led_color(i, 0, 0, 0)
        neo.update_strip()
        
        print("✅ Done!")

if __name__ == '__main__':
    main()
