#!/usr/bin/env python3
"""
analyze_stl_meshes.py - Анализ STL файлов из Fusion 360

Этот скрипт:
1. Читает все STL файлы из FROM_FUSION_360/meshes/
2. Вычисляет bounding box (размеры)
3. Вычисляет центр масс (центроид)
4. Извлекает позицию относительно origin (0,0,0)
5. Создает отчет для обновления URDF

Установка зависимостей:
    pip install numpy-stl
"""

import os
import sys
from pathlib import Path
import numpy as np

try:
    from stl import mesh
except ImportError:
    print("ERROR: numpy-stl not installed!")
    print("Install: pip install numpy-stl")
    sys.exit(1)


def analyze_stl(stl_path):
    """Анализирует STL файл и возвращает геометрические параметры"""
    try:
        # Загружаем mesh
        stl_mesh = mesh.Mesh.from_file(str(stl_path))
        
        # Вычисляем bounding box
        min_coords = stl_mesh.vectors.reshape(-1, 3).min(axis=0)
        max_coords = stl_mesh.vectors.reshape(-1, 3).max(axis=0)
        
        # Размеры
        size = max_coords - min_coords
        
        # Центр bounding box
        center = (min_coords + max_coords) / 2.0
        
        # Центроид (центр масс, если плотность равномерная)
        centroid = stl_mesh.vectors.mean(axis=(0, 1))
        
        # Объем (приблизительный, через triangles)
        volume = stl_mesh.get_mass_properties()[0]
        
        return {
            'min': min_coords,
            'max': max_coords,
            'size': size,
            'center': center,
            'centroid': centroid,
            'volume': volume,
            'triangles': len(stl_mesh.vectors)
        }
    except Exception as e:
        return {'error': str(e)}


def format_coords(coords, unit='m'):
    """Форматирует координаты в читаемый вид"""
    # Предполагаем что STL в миллиметрах, конвертируем в метры
    if unit == 'm':
        coords = coords / 1000.0
    return f"x={coords[0]:.4f}, y={coords[1]:.4f}, z={coords[2]:.4f}"


def main():
    # Путь к mesh файлам
    meshes_dir = Path(__file__).parent.parent / "FROM_FUSION_360" / "meshes"
    
    if not meshes_dir.exists():
        print(f"ERROR: Directory not found: {meshes_dir}")
        sys.exit(1)
    
    print("=" * 80)
    print("  STL Mesh Analysis - Rob Box from Fusion 360")
    print("=" * 80)
    print()
    
    # Находим все STL файлы
    stl_files = sorted(meshes_dir.glob("*.stl"))
    
    if not stl_files:
        print("No STL files found!")
        sys.exit(1)
    
    print(f"Found {len(stl_files)} STL files\n")
    
    # Группируем по категориям
    sensors = ['LSLIDAR', 'OAK_D_LITE', 'RP_CAM']
    wheels = ['wheel']
    rockers = ['rocker']
    body = ['base_link', 'body', 'cover', 'assembly']
    
    results = {}
    
    for stl_file in stl_files:
        name = stl_file.stem
        print(f"Analyzing: {name}...")
        
        result = analyze_stl(stl_file)
        
        if 'error' in result:
            print(f"  ERROR: {result['error']}\n")
            continue
        
        results[name] = result
        
        # Определяем категорию
        category = "other"
        if any(s.lower() in name.lower() for s in sensors):
            category = "sensor"
        elif any(w in name.lower() for w in wheels):
            category = "wheel"
        elif any(r in name.lower() for r in rockers):
            category = "rocker"
        elif any(b in name.lower() for b in body):
            category = "body"
        
        print(f"  Category: {category}")
        print(f"  Triangles: {result['triangles']}")
        print(f"  Size (mm): {format_coords(result['size']*1000, 'mm')}")
        print(f"  Size (m):  {format_coords(result['size'], 'm')}")
        print(f"  Center:    {format_coords(result['center'], 'm')}")
        print(f"  Centroid:  {format_coords(result['centroid'], 'm')}")
        print()
    
    # Генерируем отчет
    print("=" * 80)
    print("  Summary Report - Key Components")
    print("=" * 80)
    print()
    
    # Сенсоры
    print("SENSORS (положение относительно origin Fusion 360):")
    print("-" * 80)
    for name, data in results.items():
        if any(s.lower() in name.lower() for s in sensors):
            print(f"\n{name}:")
            print(f"  Position: {format_coords(data['center'], 'm')}")
            print(f"  Size:     {format_coords(data['size'], 'm')}")
    
    # Колеса
    print("\n\nWHEELS:")
    print("-" * 80)
    wheel_data = {}
    for name, data in results.items():
        if 'wheel' in name.lower():
            print(f"\n{name}:")
            print(f"  Position: {format_coords(data['center'], 'm')}")
            print(f"  Radius:   {data['size'][2]/2000:.4f} m")  # Z размер / 2
            wheel_data[name] = data
    
    # Вычисляем wheelbase и track_width
    if len(wheel_data) >= 4:
        print("\n\nROBOT GEOMETRY:")
        print("-" * 80)
        
        # Ищем передние и задние колеса
        front_wheels = [d for n, d in wheel_data.items() if 'front' in n.lower()]
        rear_wheels = [d for n, d in wheel_data.items() if 'rear' in n.lower()]
        left_wheels = [d for n, d in wheel_data.items() if 'left' in n.lower()]
        right_wheels = [d for n, d in wheel_data.items() if 'right' in n.lower()]
        
        if front_wheels and rear_wheels:
            # Wheelbase - расстояние по X между передней и задней осью
            front_x = np.mean([w['center'][0] for w in front_wheels])
            rear_x = np.mean([w['center'][0] for w in rear_wheels])
            wheelbase = abs(front_x - rear_x) / 1000.0
            print(f"Wheelbase (X axis): {wheelbase:.4f} m")
        
        if left_wheels and right_wheels:
            # Track width - расстояние по Y между левыми и правыми колесами
            left_y = np.mean([w['center'][1] for w in left_wheels])
            right_y = np.mean([w['center'][1] for w in right_wheels])
            track_width = abs(left_y - right_y) / 1000.0
            print(f"Track Width (Y axis): {track_width:.4f} m")
    
    # Base link
    print("\n\nBASE LINK:")
    print("-" * 80)
    if 'base_link' in results:
        data = results['base_link']
        print(f"Size:   {format_coords(data['size'], 'm')}")
        print(f"Center: {format_coords(data['center'], 'm')}")
    
    print("\n" + "=" * 80)
    print("Analysis complete! Use this data to update URDF files.")
    print("=" * 80)


if __name__ == "__main__":
    main()
