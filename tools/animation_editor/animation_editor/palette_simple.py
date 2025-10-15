"""
Simple Color Palette Widget - минимальная версия без багов
"""

import tkinter as tk
from typing import Tuple, Callable, Optional


class ColorPalette(tk.Frame):
    """Упрощённая палитра цветов"""
    
    def __init__(self, master, on_color_change: Optional[Callable] = None, **kwargs):
        """
        Инициализация палитры
        
        Args:
            master: Родительский виджет
            on_color_change: Callback при изменении цвета (color: Tuple[int, int, int])
        """
        super().__init__(master, **kwargs)
        
        self.on_color_change = on_color_change
        self.current_color = (255, 255, 255)  # Белый по умолчанию
        
        self._create_widgets()
    
    def _create_widgets(self):
        """Создать виджеты"""
        self.configure(bg='#2a2a2a', relief=tk.RAISED, borderwidth=2)
        
        # Заголовок
        title = tk.Label(
            self,
            text="COLOR PALETTE",
            bg='#2a2a2a',
            fg='white'
        )
        title.pack(pady=5)
        
        # Текущий цвет
        self.current_color_canvas = tk.Canvas(
            self,
            width=60,
            height=60,
            bg='white',
            highlightthickness=2,
            highlightbackground='#555555'
        )
        self.current_color_canvas.pack(pady=5)
        
        # RGB label
        self.rgb_label = tk.Label(
            self,
            text="RGB: 255,255,255",
            bg='#2a2a2a',
            fg='#aaaaaa'
        )
        self.rgb_label.pack(pady=5)
        
        # Основные цвета
        colors = [
            ("White", (255, 255, 255)),
            ("Red", (255, 0, 0)),
            ("Green", (0, 255, 0)),
            ("Blue", (0, 0, 255)),
            ("Yellow", (255, 255, 0)),
            ("Cyan", (0, 255, 255)),
            ("Magenta", (255, 0, 255)),
            ("Black", (0, 0, 0)),
        ]
        
        for name, rgb in colors:
            btn = tk.Button(
                self,
                text=name,
                command=lambda c=rgb: self.set_color(c),
                bg=self._rgb_to_hex(rgb),
                fg='white' if sum(rgb) < 384 else 'black',
                width=15
            )
            btn.pack(padx=5, pady=2, fill=tk.X)
    
    def _rgb_to_hex(self, rgb: Tuple[int, int, int]) -> str:
        """Конвертировать RGB в hex"""
        return f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}"
    
    def set_color(self, color: Tuple[int, int, int]):
        """Установить текущий цвет"""
        self.current_color = color
        self._update_current_color_display()
        
        if self.on_color_change:
            self.on_color_change(color)
    
    def get_color(self) -> Tuple[int, int, int]:
        """Получить текущий цвет"""
        return self.current_color
    
    def _update_current_color_display(self):
        """Обновить отображение текущего цвета"""
        hex_color = self._rgb_to_hex(self.current_color)
        self.current_color_canvas.configure(bg=hex_color)
        self.rgb_label.configure(
            text=f"RGB: {self.current_color[0]},{self.current_color[1]},{self.current_color[2]}"
        )
