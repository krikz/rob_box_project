#!/usr/bin/env python3
"""
LED Animation Visualizer

Просмотр LED матричных анимаций с схематичным представлением робота.
Визуализирует 4 матрицы колёс (8×8) и главный дисплей (25×5).
"""

import sys
import os
import argparse
from pathlib import Path
import yaml
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time


class RobotVisualizer:
    """Визуализатор анимаций LED матриц робота"""
    
    # Размеры панелей (в пикселях LED)
    WHEEL_SIZE = (8, 8)
    DISPLAY_SIZE = (25, 5)
    
    # Масштаб для отображения (пикселей на LED)
    SCALE = 20
    
    # Цвета
    BG_COLOR = '#1e1e1e'
    ROBOT_COLOR = '#3a3a3a'
    LED_OFF_COLOR = '#2a2a2a'
    BORDER_COLOR = '#555555'
    
    def __init__(self, master, animations_dir):
        """
        Инициализация визуализатора
        
        Args:
            master: Tkinter root
            animations_dir: Путь к директории с анимациями
        """
        self.master = master
        self.animations_dir = Path(animations_dir)
        
        master.title("rob_box LED Animation Visualizer")
        master.configure(bg=self.BG_COLOR)
        
        # Состояние
        self.current_animation = None
        self.is_playing = False
        self.animation_thread = None
        self.frame_data = {}  # Кэш загруженных кадров
        
        # GUI элементы
        self._create_widgets()
        
        # Загрузить список анимаций
        self._load_animations_list()
    
    def _create_widgets(self):
        """Создать GUI элементы"""
        # Верхняя панель управления
        control_frame = tk.Frame(self.master, bg=self.BG_COLOR)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        
        # Выбор анимации
        tk.Label(control_frame, text="Animation:", bg=self.BG_COLOR, 
                fg='white').pack(side=tk.LEFT, padx=5)
        
        self.animation_var = tk.StringVar()
        self.animation_combo = ttk.Combobox(
            control_frame, 
            textvariable=self.animation_var,
            width=30,
            state='readonly'
        )
        self.animation_combo.pack(side=tk.LEFT, padx=5)
        self.animation_combo.bind('<<ComboboxSelected>>', self._on_animation_selected)
        
        # Кнопки управления
        self.play_btn = tk.Button(
            control_frame, 
            text="▶ Play", 
            command=self._play_animation,
            width=10
        )
        self.play_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = tk.Button(
            control_frame, 
            text="⏹ Stop", 
            command=self._stop_animation,
            width=10,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Инфо
        self.info_label = tk.Label(
            control_frame, 
            text="No animation loaded", 
            bg=self.BG_COLOR,
            fg='#888888'
        )
        self.info_label.pack(side=tk.LEFT, padx=20)
        
        # Canvas для отрисовки робота
        self.canvas = tk.Canvas(
            self.master,
            bg=self.BG_COLOR,
            width=800,
            height=600,
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Нарисовать схему робота
        self._draw_robot_schematic()
    
    def _draw_robot_schematic(self):
        """Нарисовать схематичное изображение робота с матрицами"""
        # Размеры в пикселях
        wheel_w = self.WHEEL_SIZE[0] * self.SCALE
        wheel_h = self.WHEEL_SIZE[1] * self.SCALE
        display_w = self.DISPLAY_SIZE[0] * self.SCALE
        display_h = self.DISPLAY_SIZE[1] * self.SCALE
        
        # Центр canvas
        cx = 400
        cy = 300
        
        # Корпус робота (прямоугольник)
        body_w = 300
        body_h = 200
        body_x1 = cx - body_w // 2
        body_y1 = cy - body_h // 2
        body_x2 = cx + body_w // 2
        body_y2 = cy + body_h // 2
        
        self.canvas.create_rectangle(
            body_x1, body_y1, body_x2, body_y2,
            fill=self.ROBOT_COLOR,
            outline=self.BORDER_COLOR,
            width=2
        )
        
        # Текст "FRONT" сверху
        self.canvas.create_text(
            cx, body_y1 - 20,
            text="FRONT",
            fill='white',
            font=('Arial', 12, 'bold')
        )
        
        # Передние колёса (сверху)
        # Левое переднее колесо
        fl_x = body_x1 - wheel_w - 20
        fl_y = body_y1 + 20
        self.wheel_fl_rect = self.canvas.create_rectangle(
            fl_x, fl_y, fl_x + wheel_w, fl_y + wheel_h,
            fill=self.LED_OFF_COLOR,
            outline=self.BORDER_COLOR,
            width=2
        )
        self.wheel_fl_leds = self._create_led_grid(fl_x, fl_y, *self.WHEEL_SIZE)
        self.canvas.create_text(
            fl_x + wheel_w // 2, fl_y - 15,
            text="FL",
            fill='#888888',
            font=('Arial', 10)
        )
        
        # Правое переднее колесо
        fr_x = body_x2 + 20
        fr_y = body_y1 + 20
        self.wheel_fr_rect = self.canvas.create_rectangle(
            fr_x, fr_y, fr_x + wheel_w, fr_y + wheel_h,
            fill=self.LED_OFF_COLOR,
            outline=self.BORDER_COLOR,
            width=2
        )
        self.wheel_fr_leds = self._create_led_grid(fr_x, fr_y, *self.WHEEL_SIZE)
        self.canvas.create_text(
            fr_x + wheel_w // 2, fr_y - 15,
            text="FR",
            fill='#888888',
            font=('Arial', 10)
        )
        
        # Главный дисплей (рот) - между передними колёсами
        mouth_x = cx - display_w // 2
        mouth_y = body_y1 - display_h - 30
        self.display_rect = self.canvas.create_rectangle(
            mouth_x, mouth_y, mouth_x + display_w, mouth_y + display_h,
            fill=self.LED_OFF_COLOR,
            outline=self.BORDER_COLOR,
            width=2
        )
        self.display_leds = self._create_led_grid(mouth_x, mouth_y, *self.DISPLAY_SIZE)
        self.canvas.create_text(
            mouth_x + display_w // 2, mouth_y - 15,
            text="MAIN DISPLAY (mouth)",
            fill='#888888',
            font=('Arial', 10)
        )
        
        # Задние колёса (снизу)
        # Левое заднее колесо
        rl_x = body_x1 - wheel_w - 20
        rl_y = body_y2 - wheel_h - 20
        self.wheel_rl_rect = self.canvas.create_rectangle(
            rl_x, rl_y, rl_x + wheel_w, rl_y + wheel_h,
            fill=self.LED_OFF_COLOR,
            outline=self.BORDER_COLOR,
            width=2
        )
        self.wheel_rl_leds = self._create_led_grid(rl_x, rl_y, *self.WHEEL_SIZE)
        self.canvas.create_text(
            rl_x + wheel_w // 2, rl_y + wheel_h + 15,
            text="RL",
            fill='#888888',
            font=('Arial', 10)
        )
        
        # Правое заднее колесо
        rr_x = body_x2 + 20
        rr_y = body_y2 - wheel_h - 20
        self.wheel_rr_rect = self.canvas.create_rectangle(
            rr_x, rr_y, rr_x + wheel_w, rr_y + wheel_h,
            fill=self.LED_OFF_COLOR,
            outline=self.BORDER_COLOR,
            width=2
        )
        self.wheel_rr_leds = self._create_led_grid(rr_x, rr_y, *self.WHEEL_SIZE)
        self.canvas.create_text(
            rr_x + wheel_w // 2, rr_y + wheel_h + 15,
            text="RR",
            fill='#888888',
            font=('Arial', 10)
        )
        
        # Текст "REAR" снизу
        self.canvas.create_text(
            cx, body_y2 + 20,
            text="REAR",
            fill='white',
            font=('Arial', 12, 'bold')
        )
    
    def _create_led_grid(self, x, y, width, height):
        """Создать сетку LED индикаторов"""
        leds = []
        for row in range(height):
            led_row = []
            for col in range(width):
                led_x = x + col * self.SCALE + self.SCALE // 2
                led_y = y + row * self.SCALE + self.SCALE // 2
                led = self.canvas.create_oval(
                    led_x - 8, led_y - 8,
                    led_x + 8, led_y + 8,
                    fill=self.LED_OFF_COLOR,
                    outline=self.BORDER_COLOR,
                    width=1
                )
                led_row.append(led)
            leds.append(led_row)
        return leds
    
    def _load_animations_list(self):
        """Загрузить список доступных анимаций"""
        manifests_dir = self.animations_dir / 'manifests'
        if not manifests_dir.exists():
            messagebox.showerror("Error", f"Manifests directory not found: {manifests_dir}")
            return
        
        animations = []
        for file in manifests_dir.glob('*.yaml'):
            animations.append(file.stem)
        
        if animations:
            self.animation_combo['values'] = sorted(animations)
            self.animation_combo.current(0)
            self._on_animation_selected(None)
        else:
            messagebox.showwarning("Warning", "No animations found")
    
    def _on_animation_selected(self, event):
        """Обработчик выбора анимации"""
        animation_name = self.animation_var.get()
        if not animation_name:
            return
        
        # Загрузить манифест
        manifest_path = self.animations_dir / 'manifests' / f'{animation_name}.yaml'
        try:
            with open(manifest_path, 'r') as f:
                manifest = yaml.safe_load(f)
            
            self.current_animation = manifest
            self.frame_data.clear()  # Очистить кэш кадров
            
            # Показать информацию
            info = (
                f"{manifest['name']} | "
                f"{manifest.get('description', 'No description')} | "
                f"Duration: {manifest.get('duration_ms', 0)}ms | "
                f"FPS: {manifest.get('fps', 10)}"
            )
            self.info_label.config(text=info, fg='white')
            
            # Очистить дисплей
            self._clear_all_panels()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load animation: {e}")
    
    def _clear_all_panels(self):
        """Очистить все панели"""
        for panel_leds in [self.wheel_fl_leds, self.wheel_fr_leds, 
                          self.wheel_rl_leds, self.wheel_rr_leds, 
                          self.display_leds]:
            for row in panel_leds:
                for led in row:
                    self.canvas.itemconfig(led, fill=self.LED_OFF_COLOR)
    
    def _play_animation(self):
        """Начать воспроизведение анимации"""
        if not self.current_animation:
            messagebox.showwarning("Warning", "No animation selected")
            return
        
        if self.is_playing:
            return
        
        self.is_playing = True
        self.play_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        
        # Запустить поток воспроизведения
        self.animation_thread = threading.Thread(target=self._animation_loop, daemon=True)
        self.animation_thread.start()
    
    def _stop_animation(self):
        """Остановить воспроизведение"""
        self.is_playing = False
        self.play_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self._clear_all_panels()
    
    def _animation_loop(self):
        """Главный цикл воспроизведения анимации"""
        manifest = self.current_animation
        loop = manifest.get('loop', True)
        
        try:
            while self.is_playing:
                # Воспроизвести один цикл
                self._play_cycle()
                
                # Если не зациклена - остановиться
                if not loop:
                    break
        
        except Exception as e:
            print(f"Animation error: {e}")
        
        finally:
            self.master.after(0, self._stop_animation)
    
    def _play_cycle(self):
        """Воспроизвести один цикл анимации"""
        manifest = self.current_animation
        panels = manifest.get('panels', [])
        
        # Построить временную шкалу
        timeline = self._build_timeline(panels)
        
        start_time = time.time()
        
        for timestamp_ms, panel_frames in timeline:
            if not self.is_playing:
                break
            
            # Ждать до нужного времени
            target_time = start_time + (timestamp_ms / 1000.0)
            sleep_time = target_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Отобразить кадры
            for logical_group, frame_path in panel_frames:
                self._display_frame(logical_group, frame_path)
    
    def _build_timeline(self, panels):
        """Построить временную шкалу кадров"""
        timeline = {}
        
        for panel in panels:
            logical_group = panel['logical_group']
            offset_ms = panel.get('offset_ms', 0)
            current_time = offset_ms
            
            for frame in panel['frames']:
                if current_time not in timeline:
                    timeline[current_time] = []
                
                frame_path = frame['image']
                # Разрешить относительный путь
                if not os.path.isabs(frame_path):
                    frame_path = self.animations_dir / frame_path
                
                timeline[current_time].append((logical_group, str(frame_path)))
                current_time += frame['duration_ms']
        
        return sorted(timeline.items())
    
    def _display_frame(self, logical_group, frame_path):
        """Отобразить кадр на панели"""
        # Загрузить изображение
        try:
            img_array = self._load_frame_image(frame_path)
        except Exception as e:
            print(f"Failed to load frame {frame_path}: {e}")
            return
        
        # Определить панель(и) для отображения
        if logical_group == 'wheel_front_left':
            self._update_panel(self.wheel_fl_leds, img_array)
        elif logical_group == 'wheel_front_right':
            self._update_panel(self.wheel_fr_leds, img_array)
        elif logical_group == 'wheel_rear_left':
            self._update_panel(self.wheel_rl_leds, img_array)
        elif logical_group == 'wheel_rear_right':
            self._update_panel(self.wheel_rr_leds, img_array)
        elif logical_group == 'main_display':
            self._update_panel(self.display_leds, img_array)
        elif logical_group == 'wheels_front':
            # Разделить на две панели 8×8
            if img_array.shape[1] >= 16:
                left = img_array[:, :8]
                right = img_array[:, 8:16]
                self._update_panel(self.wheel_fl_leds, left)
                self._update_panel(self.wheel_fr_leds, right)
            else:
                # Если всего 8 пикселей - отобразить на обеих
                self._update_panel(self.wheel_fl_leds, img_array)
                self._update_panel(self.wheel_fr_leds, img_array)
        elif logical_group == 'wheels_rear':
            if img_array.shape[1] >= 16:
                left = img_array[:, :8]
                right = img_array[:, 8:16]
                self._update_panel(self.wheel_rl_leds, left)
                self._update_panel(self.wheel_rr_leds, right)
            else:
                self._update_panel(self.wheel_rl_leds, img_array)
                self._update_panel(self.wheel_rr_leds, img_array)
        elif logical_group == 'wheels_all':
            # 2×2 сетка
            if img_array.shape[0] >= 16 and img_array.shape[1] >= 16:
                fl = img_array[:8, :8]
                fr = img_array[:8, 8:16]
                rl = img_array[8:16, :8]
                rr = img_array[8:16, 8:16]
                self._update_panel(self.wheel_fl_leds, fl)
                self._update_panel(self.wheel_fr_leds, fr)
                self._update_panel(self.wheel_rl_leds, rl)
                self._update_panel(self.wheel_rr_leds, rr)
            else:
                # На всех отобразить одно и то же
                self._update_panel(self.wheel_fl_leds, img_array)
                self._update_panel(self.wheel_fr_leds, img_array)
                self._update_panel(self.wheel_rl_leds, img_array)
                self._update_panel(self.wheel_rr_leds, img_array)
    
    def _load_frame_image(self, frame_path):
        """Загрузить изображение кадра"""
        if frame_path in self.frame_data:
            return self.frame_data[frame_path]
        
        img = Image.open(frame_path)
        if img.mode != 'RGB':
            img = img.convert('RGB')
        
        img_array = np.array(img)
        self.frame_data[frame_path] = img_array
        
        return img_array
    
    def _update_panel(self, panel_leds, img_array):
        """Обновить LED панель из массива изображения"""
        height, width = img_array.shape[:2]
        
        for row in range(min(height, len(panel_leds))):
            for col in range(min(width, len(panel_leds[0]))):
                r, g, b = img_array[row, col]
                color = f'#{r:02x}{g:02x}{b:02x}'
                
                # Обновить в главном потоке
                self.master.after(0, self._update_led, panel_leds[row][col], color)
    
    def _update_led(self, led_id, color):
        """Обновить цвет LED"""
        self.canvas.itemconfig(led_id, fill=color)


def main():
    parser = argparse.ArgumentParser(description='LED Animation Visualizer')
    parser.add_argument(
        '--animations-dir',
        default='src/rob_box_animations/animations',
        help='Path to animations directory'
    )
    
    args = parser.parse_args()
    
    # Проверить директорию
    animations_dir = Path(args.animations_dir)
    if not animations_dir.exists():
        print(f"Error: Animations directory not found: {animations_dir}")
        return 1
    
    # Создать GUI
    root = tk.Tk()
    app = RobotVisualizer(root, animations_dir)
    root.mainloop()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
