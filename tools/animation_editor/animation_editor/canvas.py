"""
Robot Canvas - визуализация и редактирование LED матриц
"""

import tkinter as tk
from typing import Dict, Tuple, Optional, Callable
from .models import Frame


class RobotCanvas(tk.Canvas):
    """Canvas для отображения и редактирования LED матриц робота"""
    
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
    SELECTED_BORDER = '#00ff00'
    
    def __init__(self, master, on_pixel_click: Optional[Callable] = None, **kwargs):
        """
        Инициализация canvas
        
        Args:
            master: Родительский виджет
            on_pixel_click: Callback при клике на пиксель (panel_name, x, y)
        """
        super().__init__(
            master,
            bg=self.BG_COLOR,
            width=800,
            height=700,
            highlightthickness=0,
            **kwargs
        )
        
        self.on_pixel_click = on_pixel_click
        
        # LED элементы для каждой панели
        self.panels_leds: Dict[str, list] = {}
        
        # Координаты панелей (для определения кликов)
        self.panels_bounds: Dict[str, Tuple[int, int, int, int]] = {}
        
        # Текущая выбранная панель
        self.selected_panel: Optional[str] = None
        
        # Создать схему робота
        self._draw_robot_schematic()
        
        # Bind события мыши
        self.bind('<Button-1>', self._on_mouse_click)
        self.bind('<B1-Motion>', self._on_mouse_drag)
    
    def _draw_robot_schematic(self):
        """Нарисовать схематичное изображение робота с матрицами"""
        # Размеры в пикселях
        wheel_w = self.WHEEL_SIZE[0] * self.SCALE
        wheel_h = self.WHEEL_SIZE[1] * self.SCALE
        display_w = self.DISPLAY_SIZE[0] * self.SCALE
        display_h = self.DISPLAY_SIZE[1] * self.SCALE
        
        # Центр canvas
        cx = 400
        cy = 400
        
        # Корпус робота
        body_w = 300
        body_h = 400
        body_x1 = cx - body_w // 2
        body_y1 = cy - body_h // 2
        body_x2 = cx + body_w // 2
        body_y2 = cy + body_h // 2
        
        self.create_rectangle(
            body_x1, body_y1, body_x2, body_y2,
            fill=self.ROBOT_COLOR,
            outline=self.BORDER_COLOR,
            width=2
        )
        
        # Текст "FRONT"
        self.create_text(
            cx, body_y1 - 40,
            text="↑ FRONT ↑",
            fill='#00ff00',
            font=('Arial', 14, 'bold')
        )
        
        # Левое переднее колесо
        fl_x = body_x1 - wheel_w - 30
        fl_y = body_y1
        self._create_panel('wheel_front_left', fl_x, fl_y, *self.WHEEL_SIZE, 
                          'FL (8×8)', '#00ff00', -15)
        
        # Правое переднее колесо
        fr_x = body_x2 + 30
        fr_y = body_y1
        self._create_panel('wheel_front_right', fr_x, fr_y, *self.WHEEL_SIZE,
                          'FR (8×8)', '#00ff00', -15)
        
        # Главный дисплей (рот)
        mouth_x = cx - display_w // 2
        mouth_y = body_y1 - display_h - 50
        self._create_panel('main_display', mouth_x, mouth_y, *self.DISPLAY_SIZE,
                          'MOUTH (25×5)', '#ffff00', -15)
        
        # Левое заднее колесо
        rl_x = body_x1 - wheel_w - 30
        rl_y = body_y2 - wheel_h
        self._create_panel('wheel_rear_left', rl_x, rl_y, *self.WHEEL_SIZE,
                          'RL (8×8)', '#ff0000', None)
        
        # Правое заднее колесо
        rr_x = body_x2 + 30
        rr_y = body_y2 - wheel_h
        self._create_panel('wheel_rear_right', rr_x, rr_y, *self.WHEEL_SIZE,
                          'RR (8×8)', '#ff0000', None)
        
        # Текст "REAR"
        self.create_text(
            cx, body_y2 + 80,
            text="↓ REAR ↓",
            fill='#ff0000',
            font=('Arial', 14, 'bold')
        )
        
        # Легенда
        self.create_text(
            50, 30,
            text="LED MATRICES EDITOR",
            fill='white',
            font=('Arial', 12, 'bold'),
            anchor='w'
        )
        
        # Инструкция
        self.create_text(
            50, 55,
            text="Click on panel to select • Click on LEDs to paint",
            fill='#888888',
            font=('Arial', 9),
            anchor='w'
        )
    
    def _create_panel(self, name: str, x: int, y: int, width: int, height: int,
                     label: str, label_color: str, label_offset: Optional[int]):
        """Создать панель с LED сеткой"""
        w_px = width * self.SCALE
        h_px = height * self.SCALE
        
        # Прямоугольник панели
        rect = self.create_rectangle(
            x, y, x + w_px, y + h_px,
            fill=self.LED_OFF_COLOR,
            outline=self.BORDER_COLOR,
            width=2,
            tags=(name, f'{name}_rect')
        )
        
        # Сохранить границы для определения кликов
        self.panels_bounds[name] = (x, y, x + w_px, y + h_px)
        
        # Label
        if label_offset is not None:
            label_y = y + label_offset
        else:
            label_y = y + h_px + 15
        
        self.create_text(
            x + w_px // 2, label_y,
            text=label,
            fill=label_color,
            font=('Arial', 11, 'bold'),
            tags=(name, f'{name}_label')
        )
        
        # Создать LED сетку
        leds = []
        for row in range(height):
            led_row = []
            for col in range(width):
                led_x = x + col * self.SCALE + self.SCALE // 2
                led_y = y + row * self.SCALE + self.SCALE // 2
                
                led = self.create_oval(
                    led_x - 8, led_y - 8,
                    led_x + 8, led_y + 8,
                    fill=self.LED_OFF_COLOR,
                    outline=self.BORDER_COLOR,
                    width=1,
                    tags=(name, f'{name}_led', f'{name}_led_{row}_{col}')
                )
                led_row.append(led)
            leds.append(led_row)
        
        self.panels_leds[name] = leds
    
    def update_panel_from_frame(self, panel_name: str, frame: Frame):
        """Обновить панель из кадра"""
        if panel_name not in self.panels_leds:
            return
        
        leds = self.panels_leds[panel_name]
        
        for row in range(min(frame.height, len(leds))):
            for col in range(min(frame.width, len(leds[0]))):
                r, g, b = frame.get_pixel(col, row)
                color = f'#{r:02x}{g:02x}{b:02x}'
                self.itemconfig(leds[row][col], fill=color)
    
    def clear_panel(self, panel_name: str):
        """Очистить панель"""
        if panel_name not in self.panels_leds:
            return
        
        leds = self.panels_leds[panel_name]
        for row in leds:
            for led in row:
                self.itemconfig(led, fill=self.LED_OFF_COLOR)
    
    def clear_all_panels(self):
        """Очистить все панели"""
        for panel_name in self.panels_leds.keys():
            self.clear_panel(panel_name)
    
    def set_selected_panel(self, panel_name: Optional[str]):
        """Установить выбранную панель"""
        # Снять выделение со старой панели
        if self.selected_panel:
            rect_tag = f'{self.selected_panel}_rect'
            self.itemconfig(rect_tag, outline=self.BORDER_COLOR, width=2)
        
        # Выделить новую панель
        self.selected_panel = panel_name
        if panel_name:
            rect_tag = f'{panel_name}_rect'
            self.itemconfig(rect_tag, outline=self.SELECTED_BORDER, width=3)
    
    def _on_mouse_click(self, event):
        """Обработка клика мыши"""
        x, y = event.x, event.y
        
        # Определить на какую панель кликнули
        clicked_panel = None
        for panel_name, (px1, py1, px2, py2) in self.panels_bounds.items():
            if px1 <= x <= px2 and py1 <= y <= py2:
                clicked_panel = panel_name
                break
        
        if not clicked_panel:
            return
        
        # Выделить панель
        self.set_selected_panel(clicked_panel)
        
        # Определить на какой пиксель кликнули
        px1, py1, px2, py2 = self.panels_bounds[clicked_panel]
        
        # Относительные координаты внутри панели
        rel_x = x - px1
        rel_y = y - py1
        
        # Индексы пикселя
        pixel_x = int(rel_x // self.SCALE)
        pixel_y = int(rel_y // self.SCALE)
        
        # Проверить границы
        leds = self.panels_leds[clicked_panel]
        if 0 <= pixel_y < len(leds) and 0 <= pixel_x < len(leds[0]):
            if self.on_pixel_click:
                self.on_pixel_click(clicked_panel, pixel_x, pixel_y)
    
    def _on_mouse_drag(self, event):
        """Обработка перетаскивания мыши (рисование)"""
        # Использовать тот же обработчик что и для клика
        self._on_mouse_click(event)
    
    def set_pixel_color(self, panel_name: str, x: int, y: int, color: Tuple[int, int, int]):
        """Установить цвет пикселя"""
        if panel_name not in self.panels_leds:
            return
        
        leds = self.panels_leds[panel_name]
        if 0 <= y < len(leds) and 0 <= x < len(leds[0]):
            r, g, b = color
            hex_color = f'#{r:02x}{g:02x}{b:02x}'
            self.itemconfig(leds[y][x], fill=hex_color)
