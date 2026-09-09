#!/usr/bin/env python3
"""Measure duration of all MP3 files in sound_pack directory."""

import json
from pathlib import Path
from mutagen.mp3 import MP3


def measure_all_durations():
    """Measure duration of all MP3 files and save to JSON."""
    sound_pack_dir = Path(__file__).parent
    durations = {}
    
    mp3_files = sorted(sound_pack_dir.glob("*.mp3"))
    
    print(f"\n{'='*60}")
    print(f"  ИЗМЕРЕНИЕ ДЛИТЕЛЬНОСТИ ЗВУКОВ 🎵")
    print(f"{'='*60}\n")
    print(f"Найдено MP3 файлов: {len(mp3_files)}\n")
    
    for mp3_file in mp3_files:
        try:
            audio = MP3(mp3_file)
            duration = audio.info.length
            durations[mp3_file.name] = round(duration, 3)
            print(f"✓ {mp3_file.name:<35} {duration:6.3f}s")
        except Exception as e:
            print(f"✗ {mp3_file.name:<35} ERROR: {e}")
            durations[mp3_file.name] = 0.0
    
    # Save to JSON
    json_path = sound_pack_dir / "durations.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(durations, f, indent=2, ensure_ascii=False)
    
    # Calculate statistics
    valid_durations = [d for d in durations.values() if d > 0]
    total_duration = sum(valid_durations)
    avg_duration = total_duration / len(valid_durations) if valid_durations else 0
    
    print(f"\n{'='*60}")
    print(f"📊 СТАТИСТИКА:")
    print(f"  • Всего файлов: {len(durations)}")
    print(f"  • Общая длительность: {total_duration:.2f}s ({total_duration/60:.2f} минут)")
    print(f"  • Средняя длительность: {avg_duration:.3f}s")
    print(f"  • Минимум: {min(valid_durations):.3f}s")
    print(f"  • Максимум: {max(valid_durations):.3f}s")
    print(f"\n✅ Сохранено в: {json_path.name}")
    print(f"{'='*60}\n")
    
    return durations


if __name__ == "__main__":
    measure_all_durations()
