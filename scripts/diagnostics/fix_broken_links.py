#!/usr/bin/env python3
"""
Скрипт для автоматического исправления битых ссылок в markdown файлах
"""

import os
import re
from pathlib import Path

# Маппинг старых путей на новые
FIXES = {
    # Гифки переехали из src в docs
    'src/rob_box_animations/assets/animations/': 'docs/assets/animations/',
    '../assets/animations/': '../../../docs/assets/animations/',  # Для src/rob_box_animations/docs/ANIMATIONS.md
    
    # Документация
    'docs/reference/ARCHITECTURE.md': 'docs/architecture/SYSTEM_OVERVIEW.md',
    'docs/HARDWARE.md': 'docs/architecture/HARDWARE.md',
    'ARCHITECTURE.md': 'SYSTEM_OVERVIEW.md',
    '../docs/': '../',  # Убираем дублирующиеся docs/docs/
    
    # Getting started
    'docs/getting-started/QUICK_START_RU.md': 'docs/guides/QUICK_START.md',
    'docs/getting-started/CHECKLIST.md': 'docs/guides/VISION_PI_SETUP.md',
    
    # Guides
    'docs/guides/POWER_MONITORING_SCRIPTS.md': 'docker/monitor_system.sh',
    'docs/guides/BASH_ALIASES.md': 'docs/deployment/VISION_PI_DEPLOYMENT.md',
    
    # Development
    'docs/development/VOICE_ASSISTANT_ARCHITECTURE.md': 'src/rob_box_voice/README.md',
    'docs/reference/RTABMAP_LIDAR_CONFIG.md': 'docs/guides/VISUALIZATION.md',
    'docs/reference/FUSION360_MEASUREMENTS.md': 'docs/architecture/HARDWARE.md',
    'docs/reference/VESC_INTEGRATION.md': 'src/vesc_nexus/README.md',
    'docs/reference/OPTIMIZATION.md': 'docs/development/BUILD_OPTIMIZATION.md',
    'docs/reference/OPTIMIZATION_SUMMARY.md': 'docs/development/BUILD_OPTIMIZATION.md',
    
    # Docker
    'docker/POWER_MANAGEMENT.md': 'docs/deployment/VISION_PI_DEPLOYMENT.md',
    'docker/POWER_MONITORING_SCRIPTS.md': 'docker/monitor_system.sh',
    'docker/OPTIMIZATION_README.md': 'docs/development/BUILD_OPTIMIZATION.md',
    'docker/ARCHITECTURE.md': 'docs/architecture/SYSTEM_OVERVIEW.md',
    'docker/DOCKER_STANDARDS.md': 'docs/CI_CD_PIPELINE.md',
    'docker/AGENT_GUIDE.md': 'docs/deployment/VISION_PI_DEPLOYMENT.md',
    'docker/TROUBLESHOOTING.md': 'docs/deployment/VISION_PI_DEPLOYMENT.md',
    
    # Other
    './.WORKFLOWS_GUIDE.md': './README.md',
    '../LICENSE': '../../LICENSE',
    '../CONTRIBUTING.md': '../../CONTRIBUTING.md',
    'docs/guides/ARCHITECTURE.md': 'docs/architecture/SYSTEM_OVERVIEW.md',
    'docs/guides/HARDWARE.md': 'docs/architecture/HARDWARE.md',
    'docs/guides/SOFTWARE.md': 'docs/architecture/SOFTWARE.md',
    'docs/guides/DEPLOYMENT.md': 'docs/deployment/VISION_PI_DEPLOYMENT.md',
    'docs/guides/API_REFERENCE.md': 'docs/architecture/SYSTEM_OVERVIEW.md',
    'docs/architecture/DEPLOYMENT.md': 'docs/deployment/VISION_PI_DEPLOYMENT.md',
    'docs/architecture/API_REFERENCE.md': 'docs/architecture/SYSTEM_OVERVIEW.md',
    'docs/architecture/development/DOCKER_STANDARDS.md': 'docs/CI_CD_PIPELINE.md',
    '../../docs/guides/CAN_SETUP.md': 'docs/guides/VISUALIZATION.md',
    '../../docs/reference/VESC_INTEGRATION.md': 'src/vesc_nexus/README.md',
    '../../docs/reference/ROS2_CONTROL_ARCHITECTURE.md': 'src/vesc_nexus/README.md',
    '../../docs/reference/LED_MATRIX_INTEGRATION.md': 'src/rob_box_animations/README.md',
    '../rob_box_description/animations/ANIMATIONS_LIST.md': 'animations/ANIMATIONS_LIST.md',
    '../docs/reference/LED_MATRIX_INTEGRATION.md': '../../README.md',
    '../../src/rob_box_animations/README.md': '../README.md',
    '../SOFTWARE.md': '../../../docs/architecture/SOFTWARE.md',
    'docs/reference/vesc-integration.md': 'src/vesc_nexus/README.md',
    '../../../docs/guides/RASPBERRY_PI_USB_POWER_FIX.md': 'RASPBERRY_PI_USB_POWER_FIX.md',
    '../../../docs/guides/TROUBLESHOOTING.md': 'VISION_PI_SETUP.md',
    '../../../docs/guides/QUICK_START.md': 'QUICK_START.md',
    '../../../docs/deployment/VISION_PI_DEPLOYMENT.md': '../deployment/VISION_PI_DEPLOYMENT.md',
    '../../../docs/architecture/DOCKER_ARCHITECTURE.md': '../architecture/SYSTEM_OVERVIEW.md',
    './docs/VOICE_FORMATTING_RULES.md': '../VOICE_FORMATTING_RULES.md',
    './docs/VOICE_FORMATTING_QUICK.md': '../VOICE_FORMATTING_QUICK.md',
}

def fix_link(match, file_path):
    """Исправляет одну ссылку"""
    text = match.group(1)
    url = match.group(2)
    
    # Пропускаем внешние ссылки
    if url.startswith(('http://', 'https://', 'ftp://', 'mailto:')):
        return match.group(0)
    
    # Пробуем найти замену
    original_url = url
    for old, new in FIXES.items():
        if old in url:
            url = url.replace(old, new)
            print(f"  Исправлено: {old} → {new}")
            break
    
    if url == original_url:
        return match.group(0)  # Не изменилось
    
    return f'[{text}]({url})'

def fix_file(file_path):
    """Исправляет все ссылки в одном файле"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except Exception as e:
        print(f"⚠️  Ошибка чтения {file_path}: {e}")
        return False
    
    original_content = content
    
    # Заменяем ссылки
    pattern = r'!?\[([^\]]*)\]\(([^)]+)\)'
    content = re.sub(pattern, lambda m: fix_link(m, file_path), content)
    
    if content != original_content:
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
            return True
        except Exception as e:
            print(f"⚠️  Ошибка записи {file_path}: {e}")
            return False
    
    return False

def main():
    print("🔧 Автоматическое исправление битых ссылок")
    print("=" * 70)
    print()
    
    root_dir = Path.cwd()
    md_files = []
    
    for root, dirs, files in os.walk(root_dir):
        dirs[:] = [d for d in dirs if d not in ['.git', 'node_modules', 'build', 'install', 'log']]
        for file in files:
            if file.endswith('.md'):
                md_files.append(Path(root) / file)
    
    fixed_count = 0
    
    for md_file in md_files:
        rel_path = md_file.relative_to(root_dir)
        print(f"Проверяю: {rel_path}")
        if fix_file(md_file):
            fixed_count += 1
            print(f"  ✓ Файл исправлен")
    
    print()
    print("=" * 70)
    print(f"Исправлено файлов: {fixed_count}")

if __name__ == '__main__':
    main()
