#!/usr/bin/env python3
"""
Генератор AprilTag коллекции tag36h11 для печати
Генерирует все 587 тегов (0-586) с ID в углу
Размер для печати: 180мм x 180мм

Метод: скачивает официальные изображения из apriltag-imgs репозитория
"""

import numpy as np
import cv2
from pathlib import Path
import subprocess
import sys


def download_official_tags(base_dir: Path, family: str = "tag36h11"):
    """Скачивает официальные AprilTag изображения с GitHub"""
    print("📥 Скачиваем официальные AprilTag изображения...")
    
    # Временная папка
    temp_dir = base_dir / "apriltag_temp"
    
    if temp_dir.exists():
        print(f"🗑️  Удаляем старую временную папку...")
        subprocess.run(["rm", "-rf", str(temp_dir)], check=True)
    
    print(f"📦 Клонируем apriltag-imgs репозиторий...")
    try:
        subprocess.run([
            "git", "clone", "--depth", "1", "--filter=blob:none", "--sparse",
            "https://github.com/AprilRobotics/apriltag-imgs.git",
            str(temp_dir)
        ], check=True, capture_output=True)
        
        subprocess.run([
            "git", "-C", str(temp_dir), "sparse-checkout", "set", family
        ], check=True, capture_output=True)
        
        print(f"✅ Репозиторий склонирован")
        return temp_dir / family
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Ошибка клонирования: {e}")
        return None


def process_tags(source_dir: Path, output_dir: Path, 
                 target_size: int = 1100, border_size: int = 100):
    """Обрабатывает скачанные теги: добавляет рамку и ID"""
    output_dir.mkdir(parents=True, exist_ok=True)
    
    png_files = sorted(source_dir.glob("*.png"))
    
    if not png_files:
        print(f"❌ Не найдено PNG файлов в {source_dir}")
        return
    
    print(f"\n🖼️  Обработка {len(png_files)} изображений...")
    
    for idx, png_file in enumerate(png_files):
        img = cv2.imread(str(png_file), cv2.IMREAD_GRAYSCALE)
        
        if img is None:
            continue
        
        try:
            tag_id = int(png_file.stem.split('_')[-1])
        except ValueError:
            continue
        
        tag_size = target_size - 2 * border_size
        img_resized = cv2.resize(img, (tag_size, tag_size), 
                                interpolation=cv2.INTER_NEAREST)
        
        final_img = add_border_and_label(img_resized, tag_id, border_size)
        
        out_filename = output_dir / f"tag36h11_id{tag_id:03d}.png"
        cv2.imwrite(str(out_filename), final_img)
        
        if (idx + 1) % 50 == 0:
            print(f"  ✓ Обработано: {idx + 1}/{len(png_files)}")
    
    print(f"\n✅ Готово! {len(png_files)} тегов в {output_dir}")


def add_border_and_label(tag_img: np.ndarray, tag_id: int, 
                         border_size: int) -> np.ndarray:
    """Добавляет белую рамку и текст с ID тега"""
    h, w = tag_img.shape
    
    bordered = np.ones((h + 2*border_size, w + 2*border_size), dtype=np.uint8) * 255
    bordered[border_size:border_size+h, border_size:border_size+w] = tag_img
    
    text = f"ID: {tag_id}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = max(0.8, border_size / 60.0)
    thickness = max(2, int(border_size / 30))
    
    (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    
    margin = max(8, border_size // 10)
    text_x = margin
    text_y = text_h + margin
    
    pad = 4
    cv2.rectangle(bordered, 
                  (text_x - pad, text_y - text_h - pad),
                  (text_x + text_w + pad, text_y + baseline + pad),
                  255, -1)
    
    cv2.putText(bordered, text, (text_x, text_y), 
                font, font_scale, 0, thickness, cv2.LINE_AA)
    
    return bordered


def main():
    """Основная функция"""
    print("=" * 70)
    print("  AprilTag Generator for Printing (180x180mm)")
    print("  Family: tag36h11 (587 tags)")
    print("=" * 70)
    
    family = "tag36h11"
    base_dir = Path("/home/ros2/rob_box_project")
    output_dir = base_dir / "apriltags_printable"
    
    target_size = 1100
    border_size = 55  # 20% ВСЕГО на рамку: 1100*0.2=220px на обе стороны = по 55px
    
    print(f"\n📂 Выходная папка: {output_dir}")
    print(f"📏 Размер: {target_size}x{target_size}px (рамка {border_size}px)")
    
    source_dir = download_official_tags(base_dir, family)
    
    if not source_dir or not source_dir.exists():
        print("\n❌ Не удалось скачать изображения!")
        sys.exit(1)
    
    process_tags(source_dir, output_dir, target_size, border_size)
    
    temp_dir = base_dir / "apriltag_temp"
    if temp_dir.exists():
        print(f"\n🗑️  Удаляем временные файлы...")
        subprocess.run(["rm", "-rf", str(temp_dir)])
    
    print("\n" + "=" * 70)
    print("📋 ИНСТРУКЦИЯ ПО ПЕЧАТИ:")
    print("=" * 70)
    print("1. Откройте PNG файлы в редакторе/принтере")
    print("2. Установите размер: 180мм x 180мм (БЕЗ автомасштабирования!)")
    print("3. Печатайте на белой матовой бумаге")
    print("4. Качество печати: 300+ DPI")
    print("5. Проверьте размер линейкой после печати!")
    print("=" * 70)
    
    print(f"\n✨ Все готово! Файлы в {output_dir}")


if __name__ == "__main__":
    main()
