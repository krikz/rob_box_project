#!/usr/bin/env python3
"""
Главный entry point для Animation Editor
"""

import sys
import argparse
from pathlib import Path
import tkinter as tk

from animation_editor.app import AnimationEditorApp


def main():
    """Главная функция"""
    parser = argparse.ArgumentParser(
        description='rob_box LED Animation Editor'
    )
    parser.add_argument(
        '--animations-dir',
        type=Path,
        default='src/rob_box_animations/animations',
        help='Path to animations directory (default: src/rob_box_animations/animations)'
    )
    
    args = parser.parse_args()
    
    # Проверить директорию
    animations_dir = Path(args.animations_dir)
    if not animations_dir.exists():
        print(f"Creating animations directory: {animations_dir}")
        animations_dir.mkdir(parents=True, exist_ok=True)
        (animations_dir / 'manifests').mkdir(exist_ok=True)
        (animations_dir / 'frames').mkdir(exist_ok=True)
    
    # Создать Tkinter root
    root = tk.Tk()
    
    # Создать и запустить приложение
    try:
        app = AnimationEditorApp(root, animations_dir)
        app.run()
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
