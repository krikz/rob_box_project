"""
Timeline Widget - управление кадрами анимации
"""

import tkinter as tk
from tkinter import ttk
from typing import Optional, Callable, List
from .models import Animation, Frame


class Timeline(tk.Frame):
    """Timeline для управления кадрами анимации"""
    
    FRAME_WIDTH = 60  # Ширина кадра на timeline
    FRAME_HEIGHT = 50  # Высота кадра
    
    def __init__(self, master, animation: Optional[Animation] = None,
                 on_frame_select: Optional[Callable] = None,
                 on_frame_change: Optional[Callable] = None,
                 **kwargs):
        """
        Инициализация timeline
        
        Args:
            master: Родительский виджет
            animation: Анимация для отображения
            on_frame_select: Callback при выборе кадра (panel_name, frame_index)
            on_frame_change: Callback при изменении кадров
        """
        super().__init__(master, **kwargs)
        
        self.animation = animation
        self.on_frame_select = on_frame_select
        self.on_frame_change = on_frame_change
        
        # Текущая выбранная панель и кадр
        self.selected_panel: Optional[str] = None
        self.selected_frame_index: Optional[int] = None
        
        self._create_widgets()
    
    def _create_widgets(self):
        """Создать виджеты"""
        self.configure(bg='#2a2a2a', relief=tk.SUNKEN, borderwidth=2)
        
        # Заголовок
        header = tk.Frame(self, bg='#2a2a2a')
        header.pack(fill=tk.X, pady=5)
        
        title = tk.Label(
            header,
            text="🎬 TIMELINE",
            bg='#2a2a2a',
            fg='white',
            font=('Arial', 11, 'bold')
        )
        title.pack(side=tk.LEFT, padx=10)
        
        # Кнопки управления
        btn_frame = tk.Frame(header, bg='#2a2a2a')
        btn_frame.pack(side=tk.RIGHT, padx=10)
        
        self.add_btn = tk.Button(
            btn_frame,
            text="➕ Add Frame",
            command=self._add_frame,
            bg='#3a3a3a',
            fg='white',
            relief=tk.RAISED
        )
        self.add_btn.pack(side=tk.LEFT, padx=2)
        
        self.duplicate_btn = tk.Button(
            btn_frame,
            text="📋 Duplicate",
            command=self._duplicate_frame,
            bg='#3a3a3a',
            fg='white',
            relief=tk.RAISED,
            state=tk.DISABLED
        )
        self.duplicate_btn.pack(side=tk.LEFT, padx=2)
        
        self.delete_btn = tk.Button(
            btn_frame,
            text="🗑️ Delete",
            command=self._delete_frame,
            bg='#3a3a3a',
            fg='white',
            relief=tk.RAISED,
            state=tk.DISABLED
        )
        self.delete_btn.pack(side=tk.LEFT, padx=2)
        
        # Separator
        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X)
        
        # Выбор панели
        panel_frame = tk.Frame(self, bg='#2a2a2a')
        panel_frame.pack(fill=tk.X, pady=5, padx=10)
        
        tk.Label(
            panel_frame,
            text="Panel:",
            bg='#2a2a2a',
            fg='#aaaaaa'
        ).pack(side=tk.LEFT, padx=5)
        
        self.panel_var = tk.StringVar()
        self.panel_combo = ttk.Combobox(
            panel_frame,
            textvariable=self.panel_var,
            width=20,
            state='readonly'
        )
        self.panel_combo.pack(side=tk.LEFT, padx=5)
        self.panel_combo.bind('<<ComboboxSelected>>', self._on_panel_change)
        
        # Populate panels
        if self.animation:
            self.panel_combo['values'] = list(self.animation.panels.keys())
            if self.panel_combo['values']:
                self.panel_combo.current(0)
        
        # Frames container (scrollable)
        frames_container = tk.Frame(self, bg='#2a2a2a')
        frames_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Scrollbar
        scrollbar = tk.Scrollbar(frames_container, orient=tk.HORIZONTAL)
        scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Canvas для frames
        self.frames_canvas = tk.Canvas(
            frames_container,
            bg='#1e1e1e',
            height=80,
            xscrollcommand=scrollbar.set,
            highlightthickness=0
        )
        self.frames_canvas.pack(fill=tk.BOTH, expand=True)
        
        scrollbar.config(command=self.frames_canvas.xview)
        
        # Frame для кадров внутри canvas
        self.frames_inner = tk.Frame(self.frames_canvas, bg='#1e1e1e')
        self.frames_canvas.create_window((0, 0), window=self.frames_inner, anchor='nw')
        
        self.frames_inner.bind('<Configure>', 
                              lambda e: self.frames_canvas.configure(
                                  scrollregion=self.frames_canvas.bbox("all")))
        
        # Info панель внизу
        info_frame = tk.Frame(self, bg='#2a2a2a')
        info_frame.pack(fill=tk.X, pady=5, padx=10)
        
        self.info_label = tk.Label(
            info_frame,
            text="No frame selected",
            bg='#2a2a2a',
            fg='#888888',
            font=('Arial', 9)
        )
        self.info_label.pack(side=tk.LEFT)
        
        # Duration control
        duration_frame = tk.Frame(info_frame, bg='#2a2a2a')
        duration_frame.pack(side=tk.RIGHT)
        
        tk.Label(
            duration_frame,
            text="Duration:",
            bg='#2a2a2a',
            fg='#aaaaaa'
        ).pack(side=tk.LEFT, padx=5)
        
        self.duration_var = tk.IntVar(value=100)
        self.duration_spinbox = tk.Spinbox(
            duration_frame,
            from_=10,
            to=5000,
            increment=10,
            textvariable=self.duration_var,
            width=8,
            command=self._on_duration_change,
            state=tk.DISABLED
        )
        self.duration_spinbox.pack(side=tk.LEFT)
        
        tk.Label(
            duration_frame,
            text="ms",
            bg='#2a2a2a',
            fg='#aaaaaa'
        ).pack(side=tk.LEFT, padx=2)
        
        # Обновить отображение
        self._refresh_frames()
    
    def set_animation(self, animation: Animation):
        """Установить анимацию"""
        self.animation = animation
        
        # Обновить список панелей
        self.panel_combo['values'] = list(animation.panels.keys())
        if self.panel_combo['values']:
            self.panel_combo.current(0)
            self.selected_panel = self.panel_var.get()
        
        self._refresh_frames()
    
    def _on_panel_change(self, event=None):
        """Обработка изменения панели"""
        self.selected_panel = self.panel_var.get()
        self.selected_frame_index = None
        self._refresh_frames()
    
    def _refresh_frames(self):
        """Обновить отображение кадров"""
        # Очистить старые кадры
        for widget in self.frames_inner.winfo_children():
            widget.destroy()
        
        if not self.animation or not self.selected_panel:
            return
        
        panel = self.animation.panels.get(self.selected_panel)
        if not panel:
            return
        
        # Отобразить кадры
        for i, frame in enumerate(panel.frames):
            frame_widget = self._create_frame_widget(i, frame)
            frame_widget.pack(side=tk.LEFT, padx=5, pady=5)
    
    def _create_frame_widget(self, index: int, frame: Frame) -> tk.Frame:
        """Создать виджет для кадра"""
        is_selected = index == self.selected_frame_index
        
        container = tk.Frame(
            self.frames_inner,
            bg='#00ff00' if is_selected else '#3a3a3a',
            relief=tk.RAISED,
            borderwidth=2,
            cursor='hand2'
        )
        
        # Превью кадра (маленький canvas)
        preview = tk.Canvas(
            container,
            width=self.FRAME_WIDTH,
            height=self.FRAME_HEIGHT,
            bg='#1e1e1e',
            highlightthickness=0
        )
        preview.pack(padx=2, pady=2)
        
        # Нарисовать упрощенное превью
        if frame.image is not None:
            self._draw_frame_preview(preview, frame)
        
        # Label с номером кадра и длительностью
        label = tk.Label(
            container,
            text=f"#{index}\n{frame.duration_ms}ms",
            bg='#3a3a3a',
            fg='white',
            font=('Arial', 8)
        )
        label.pack()
        
        # Bind click
        container.bind('<Button-1>', lambda e, i=index: self._select_frame(i))
        preview.bind('<Button-1>', lambda e, i=index: self._select_frame(i))
        label.bind('<Button-1>', lambda e, i=index: self._select_frame(i))
        
        return container
    
    def _draw_frame_preview(self, canvas: tk.Canvas, frame: Frame):
        """Нарисовать превью кадра"""
        if frame.image is None:
            return
        
        height, width = frame.image.shape[:2]
        
        # Масштаб для отображения
        scale_x = self.FRAME_WIDTH / width
        scale_y = self.FRAME_HEIGHT / height
        scale = min(scale_x, scale_y)
        
        pixel_size = max(1, int(scale))
        
        # Отцентрировать
        offset_x = (self.FRAME_WIDTH - width * pixel_size) // 2
        offset_y = (self.FRAME_HEIGHT - height * pixel_size) // 2
        
        # Нарисовать пиксели
        for y in range(height):
            for x in range(width):
                r, g, b = frame.get_pixel(x, y)
                color = f'#{r:02x}{g:02x}{b:02x}'
                
                x1 = offset_x + x * pixel_size
                y1 = offset_y + y * pixel_size
                x2 = x1 + pixel_size
                y2 = y1 + pixel_size
                
                canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='')
    
    def _select_frame(self, index: int):
        """Выбрать кадр"""
        self.selected_frame_index = index
        
        # Обновить отображение
        self._refresh_frames()
        
        # Обновить info и duration
        if self.animation and self.selected_panel:
            panel = self.animation.panels[self.selected_panel]
            if 0 <= index < len(panel.frames):
                frame = panel.frames[index]
                
                self.info_label.config(
                    text=f"Frame #{index} | {frame.width}×{frame.height} | {frame.duration_ms}ms",
                    fg='white'
                )
                
                self.duration_var.set(frame.duration_ms)
                self.duration_spinbox.config(state=tk.NORMAL)
                
                # Enable buttons
                self.duplicate_btn.config(state=tk.NORMAL)
                self.delete_btn.config(state=tk.NORMAL)
                
                # Callback
                if self.on_frame_select:
                    self.on_frame_select(self.selected_panel, index)
    
    def _add_frame(self):
        """Добавить новый кадр"""
        if not self.animation or not self.selected_panel:
            return
        
        panel = self.animation.panels[self.selected_panel]
        
        # Создать новый кадр
        frame = self.animation.create_new_frame(self.selected_panel)
        
        # Добавить после текущего или в конец
        if self.selected_frame_index is not None:
            panel.add_frame(frame, self.selected_frame_index + 1)
            self.selected_frame_index += 1
        else:
            panel.add_frame(frame)
            self.selected_frame_index = len(panel.frames) - 1
        
        # Обновить
        self._refresh_frames()
        self._select_frame(self.selected_frame_index)
        
        if self.on_frame_change:
            self.on_frame_change()
    
    def _duplicate_frame(self):
        """Дублировать кадр"""
        if not self.animation or not self.selected_panel or self.selected_frame_index is None:
            return
        
        panel = self.animation.panels[self.selected_panel]
        panel.duplicate_frame(self.selected_frame_index)
        
        self.selected_frame_index += 1
        self._refresh_frames()
        self._select_frame(self.selected_frame_index)
        
        if self.on_frame_change:
            self.on_frame_change()
    
    def _delete_frame(self):
        """Удалить кадр"""
        if not self.animation or not self.selected_panel or self.selected_frame_index is None:
            return
        
        panel = self.animation.panels[self.selected_panel]
        
        if len(panel.frames) <= 1:
            # Не удалять последний кадр
            return
        
        panel.remove_frame(self.selected_frame_index)
        
        # Выбрать соседний кадр
        if self.selected_frame_index >= len(panel.frames):
            self.selected_frame_index = len(panel.frames) - 1
        
        self._refresh_frames()
        if panel.frames:
            self._select_frame(self.selected_frame_index)
        
        if self.on_frame_change:
            self.on_frame_change()
    
    def _on_duration_change(self):
        """Обработка изменения длительности"""
        if not self.animation or not self.selected_panel or self.selected_frame_index is None:
            return
        
        panel = self.animation.panels[self.selected_panel]
        if 0 <= self.selected_frame_index < len(panel.frames):
            frame = panel.frames[self.selected_frame_index]
            frame.duration_ms = self.duration_var.get()
            
            # Обновить info
            self.info_label.config(
                text=f"Frame #{self.selected_frame_index} | {frame.width}×{frame.height} | {frame.duration_ms}ms"
            )
            
            # Обновить превью
            self._refresh_frames()
            
            if self.on_frame_change:
                self.on_frame_change()
    
    def get_current_frame(self) -> Optional[Frame]:
        """Получить текущий выбранный кадр"""
        if not self.animation or not self.selected_panel or self.selected_frame_index is None:
            return None
        
        panel = self.animation.panels[self.selected_panel]
        if 0 <= self.selected_frame_index < len(panel.frames):
            return panel.frames[self.selected_frame_index]
        
        return None
    
    def refresh(self):
        """Обновить отображение"""
        self._refresh_frames()
