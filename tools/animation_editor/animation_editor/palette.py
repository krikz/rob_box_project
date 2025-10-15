"""
Color Palette Widget - выбор цвета для рисования
"""

import tkinter as tk
from tkinter import ttk, colorchooser
from typing import Tuple, Callable, Optional


class ColorPalette(tk.Frame):
    """Палитра цветов с предустановками и custom picker"""
    
    # Предустановленные цвета
    PRESET_COLORS = [
        # Базовые
        ('Black', '#000000'),
        ('White', '#FFFFFF'),
        ('Red', '#FF0000'),
        ('Green', '#00FF00'),
        ('Blue', '#0000FF'),
        ('Yellow', '#FFFF00'),
        ('Cyan', '#00FFFF'),
        ('Magenta', '#FF00FF'),
        
        # Дополнительные
        ('Orange', '#FF8800'),
        ('Purple', '#8800FF'),
        ('Pink', '#FF0088'),
        ('Lime', '#88FF00'),
        
        # Тёмные
        ('Dark Red', '#880000'),
        ('Dark Green', '#008800'),
        ('Dark Blue', '#000088'),
        ('Gray', '#888888'),
    ]
    
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
            text="🎨 COLOR PALETTE",
            bg='#2a2a2a',
            fg='white',
            font=('Arial', 11, 'bold')
        )
        title.pack(pady=5)
        
        # Текущий цвет (большой квадрат)
        current_frame = tk.Frame(self, bg='#2a2a2a')
        current_frame.pack(pady=5)
        
        tk.Label(
            current_frame,
            text="Current:",
            bg='#2a2a2a',
            fg='#aaaaaa'
        ).pack(side=tk.LEFT, padx=5)
        
        self.current_color_canvas = tk.Canvas(
            current_frame,
            width=60,
            height=60,
            bg='white',
            highlightthickness=2,
            highlightbackground='#555555'
        )
        self.current_color_canvas.pack(side=tk.LEFT, padx=5)
        
        # RGB значения
        self.rgb_label = tk.Label(
            current_frame,
            text="RGB:\n255,255,255",
            bg='#2a2a2a',
            fg='#aaaaaa',
            font=('Courier', 9),
            justify=tk.LEFT
        )
        self.rgb_label.pack(side=tk.LEFT, padx=5)
        
        # Separator
        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)
        
        # Preset colors (grid)
        presets_label = tk.Label(
            self,
            text="Presets:",
            bg='#2a2a2a',
            fg='#aaaaaa'
        )
        presets_label.pack(anchor=tk.W, padx=5)
        
        presets_frame = tk.Frame(self, bg='#2a2a2a')
        presets_frame.pack(padx=5, pady=5)
        
        # Создать кнопки для preset цветов (4 колонки)
        for i, (name, hex_color) in enumerate(self.PRESET_COLORS):
            row = i // 4
            col = i % 4
            
            btn = tk.Canvas(
                presets_frame,
                width=35,
                height=35,
                bg=hex_color,
                highlightthickness=1,
                highlightbackground='#555555',
                cursor='hand2'
            )
            btn.grid(row=row, column=col, padx=2, pady=2)
            
            # RGB из hex
            r = int(hex_color[1:3], 16)
            g = int(hex_color[3:5], 16)
            b = int(hex_color[5:7], 16)
            color_rgb = (r, g, b)
            
            # Bind click
            btn.bind('<Button-1>', lambda e, c=color_rgb: self.set_color(c))
            
            # Tooltip (имя цвета)
            self._create_tooltip(btn, name)
        
        # Separator
        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)
        
        # Custom color picker
        custom_btn = tk.Button(
            self,
            text="🎨 Custom Color...",
            command=self._pick_custom_color,
            bg='#3a3a3a',
            fg='white',
            activebackground='#4a4a4a',
            relief=tk.RAISED,
            cursor='hand2'
        )
        custom_btn.pack(padx=5, pady=5, fill=tk.X)
        
        # Eyedropper tool (будущая фича)
        eyedropper_btn = tk.Button(
            self,
            text="💧 Eyedropper",
            command=self._eyedropper,
            bg='#3a3a3a',
            fg='#888888',
            activebackground='#4a4a4a',
            relief=tk.RAISED,
            state=tk.DISABLED,
            cursor='hand2'
        )
        eyedropper_btn.pack(padx=5, pady=2, fill=tk.X)
        
        # Eraser (черный цвет)
        eraser_btn = tk.Button(
            self,
            text="🧹 Eraser (Black)",
            command=lambda: self.set_color((0, 0, 0)),
            bg='#3a3a3a',
            fg='white',
            activebackground='#4a4a4a',
            relief=tk.RAISED,
            cursor='hand2'
        )
        eraser_btn.pack(padx=5, pady=2, fill=tk.X)
        
        # Обновить отображение текущего цвета
        self._update_current_color_display()
    
    def _create_tooltip(self, widget, text):
        """Создать tooltip для виджета"""
        def on_enter(event):
            x, y, _, _ = widget.bbox("insert")
            x += widget.winfo_rootx() + 25
            y += widget.winfo_rooty() + 25
            
            # Создать toplevel окно
            self.tooltip = tk.Toplevel(widget)
            self.tooltip.wm_overrideredirect(True)
            self.tooltip.wm_geometry(f"+{x}+{y}")
            
            label = tk.Label(
                self.tooltip,
                text=text,
                background="#ffffe0",
                relief=tk.SOLID,
                borderwidth=1,
                font=("Arial", 8)
            )
            label.pack()
        
        def on_leave(event):
            if hasattr(self, 'tooltip'):
                self.tooltip.destroy()
                del self.tooltip
        
        widget.bind('<Enter>', on_enter)
        widget.bind('<Leave>', on_leave)
    
    def _update_current_color_display(self):
        """Обновить отображение текущего цвета"""
        r, g, b = self.current_color
        hex_color = f'#{r:02x}{g:02x}{b:02x}'
        
        self.current_color_canvas.configure(bg=hex_color)
        self.rgb_label.configure(text=f"RGB:\n{r},{g},{b}\n{hex_color}")
    
    def set_color(self, color: Tuple[int, int, int]):
        """Установить текущий цвет"""
        self.current_color = color
        self._update_current_color_display()
        
        if self.on_color_change:
            self.on_color_change(color)
    
    def get_color(self) -> Tuple[int, int, int]:
        """Получить текущий цвет"""
        return self.current_color
    
    def _pick_custom_color(self):
        """Открыть диалог выбора цвета"""
        # Текущий цвет в hex
        r, g, b = self.current_color
        initial_color = f'#{r:02x}{g:02x}{b:02x}'
        
        # Открыть color chooser
        color = colorchooser.askcolor(
            initialcolor=initial_color,
            title="Choose Custom Color"
        )
        
        if color and color[0]:  # color[0] = (r, g, b), color[1] = hex
            r, g, b = color[0]
            self.set_color((int(r), int(g), int(b)))
    
    def _eyedropper(self):
        """Eyedropper tool (пока не реализован)"""
        # TODO: Implement eyedropper functionality
        pass
