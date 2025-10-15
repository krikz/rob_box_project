#!/usr/bin/env python3
"""
Проверка битых ссылок в markdown файлах
Находит все внутренние ссылки и проверяет их существование
"""

import os
import re
from pathlib import Path
from collections import defaultdict

# Цвета для вывода
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
NC = '\033[0m'

def find_markdown_files(root_dir):
    """Находит все .md файлы"""
    md_files = []
    for root, dirs, files in os.walk(root_dir):
        # Пропускаем системные директории
        dirs[:] = [d for d in dirs if d not in ['.git', 'node_modules', 'build', 'install', 'log']]
        
        for file in files:
            if file.endswith('.md'):
                md_files.append(Path(root) / file)
    return md_files

def extract_links(content):
    """Извлекает все markdown ссылки из текста"""
    # Ищем [text](url) и ![alt](url)
    pattern = r'!?\[([^\]]*)\]\(([^)]+)\)'
    return re.findall(pattern, content)

def is_external_link(url):
    """Проверяет является ли ссылка внешней"""
    return url.startswith(('http://', 'https://', 'ftp://', 'mailto:'))

def resolve_link(base_file, link_url):
    """Преобразует относительную ссылку в абсолютный путь"""
    # Убираем якорь
    clean_url = link_url.split('#')[0]
    if not clean_url:  # Только якорь
        return None
    
    base_dir = base_file.parent
    
    # Абсолютный путь от корня проекта
    if clean_url.startswith('/'):
        return Path(os.getcwd()) / clean_url.lstrip('/')
    
    # Относительный путь
    return (base_dir / clean_url).resolve()

def check_links():
    """Основная функция проверки"""
    print(f"{BLUE}🔗 Проверка внутренних ссылок в документации{NC}")
    print("=" * 70)
    print()
    
    root_dir = Path.cwd()
    md_files = find_markdown_files(root_dir)
    
    print(f"Найдено {len(md_files)} markdown файлов")
    print()
    
    broken_links = defaultdict(list)
    total_checked = 0
    
    for md_file in md_files:
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
        except Exception as e:
            print(f"{YELLOW}⚠️  Не удалось прочитать {md_file}: {e}{NC}")
            continue
        
        links = extract_links(content)
        
        for text, url in links:
            # Пропускаем внешние ссылки
            if is_external_link(url):
                continue
            
            total_checked += 1
            
            # Проверяем существование файла
            target_path = resolve_link(md_file, url)
            
            if target_path and not target_path.exists():
                broken_links[md_file].append((text, url, target_path))
    
    # Выводим результаты
    print("=" * 70)
    print()
    
    if broken_links:
        print(f"{RED}❌ Найдено битых ссылок: {sum(len(v) for v in broken_links.values())}{NC}")
        print()
        
        for file, links in sorted(broken_links.items()):
            rel_file = file.relative_to(root_dir)
            print(f"{YELLOW}📄 {rel_file}{NC}")
            for text, url, target in links:
                print(f"   {RED}✗{NC} [{text}]({url})")
                print(f"     → Не найден: {target.relative_to(root_dir) if target.is_relative_to(root_dir) else target}")
            print()
    else:
        print(f"{GREEN}✅ Все ссылки корректны!{NC}")
    
    print()
    print(f"Проверено ссылок: {total_checked}")
    print(f"Файлов с битыми ссылками: {len(broken_links)}")
    
    return len(broken_links) > 0

if __name__ == '__main__':
    import sys
    has_broken = check_links()
    sys.exit(1 if has_broken else 0)
