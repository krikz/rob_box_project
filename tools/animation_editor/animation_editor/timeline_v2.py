"""
Timeline Widget v2 - временная шкала с ключевыми кадрами
Более современный подход - single timeline с keyframes
"""

import tkinter as tk
from tkinter import ttk
from typing import Optional, Callable, Dict
from .models import KeyframeAnimation, Keyframe


class TimelineV2(tk.Frame):
    """Timeline с единой временной шкалой и ключевыми кадрами"""
    
    MARKER_SIZE = 10  # Размер маркера ключевого кадра
    
    def __init__(self, master, animation: Optional[KeyframeAnimation] = None,
                 on_time_change: Optional[Callable] = None,
                 on_keyframe_change: Optional[Callable] = None,
                 panel_checkboxes_container: Optional[tk.Widget] = None,
                 **kwargs):
        """
        Инициализация timeline
        
        Args:
            master: Родительский виджет
            animation: KeyframeAnimation для отображения
            on_time_change: Callback при изменении времени (time_ms)
            on_keyframe_change: Callback при изменении ключевых кадров
            panel_checkboxes_container: Контейнер для чекбоксов панелей (если None, создаст свой)
        """
        super().__init__(master, **kwargs)
        
        self.animation = animation
        self.on_time_change = on_time_change
        self.on_keyframe_change = on_keyframe_change
        self.panel_checkboxes_container = panel_checkboxes_container
        
        # Текущее время
        self.current_time_ms: int = 0
        self.current_keyframe: Optional[Keyframe] = None
        
        self._create_widgets()
    
    def _create_widgets(self):
        """Создать виджеты"""
        self.configure(bg='#2a2a2a', relief=tk.SUNKEN, borderwidth=2)
        
        # Заголовок
        header = tk.Frame(self, bg='#2a2a2a')
        header.pack(fill=tk.X, pady=5)
        
        title = tk.Label(
            header,
            text="TIMELINE",  # Убрали emoji
            bg='#2a2a2a',
            fg='white'
        )
        title.pack(side=tk.LEFT, padx=10)
        
        # Текущее время
        self.time_label = tk.Label(
            header,
            text="0 ms",
            bg='#2a2a2a',
            fg='#00ff00',
            font=('Courier', 10, 'bold')
        )
        self.time_label.pack(side=tk.LEFT, padx=10)
        
        # Кнопки управления
        btn_frame = tk.Frame(header, bg='#2a2a2a')
        btn_frame.pack(side=tk.RIGHT, padx=10)
        
        self.add_keyframe_btn = tk.Button(
            btn_frame,
            text="+ Add Keyframe",
            command=self._add_keyframe,
            bg='#3a3a3a',
            fg='white',
            relief=tk.RAISED
        )
        self.add_keyframe_btn.pack(side=tk.LEFT, padx=2)
        
        self.delete_keyframe_btn = tk.Button(
            btn_frame,
            text="X Delete",
            command=self._delete_keyframe,
            bg='#3a3a3a',
            fg='white',
            relief=tk.RAISED,
            state=tk.DISABLED
        )
        self.delete_keyframe_btn.pack(side=tk.LEFT, padx=2)
        
        # Временная шкала (canvas)
        timeline_frame = tk.Frame(self, bg='#1e1e1e')
        timeline_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.timeline_canvas = tk.Canvas(
            timeline_frame,
            height=40,
            bg='#1e1e1e',
            highlightthickness=0
        )
        self.timeline_canvas.pack(fill=tk.X)
        
        # Bind клик на canvas
        self.timeline_canvas.bind('<Button-1>', self._on_timeline_click)
        
        # Слайдер времени
        slider_frame = tk.Frame(self, bg='#2a2a2a')
        slider_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(
            slider_frame,
            text="Time:",
            bg='#2a2a2a',
            fg='#aaaaaa'
        ).pack(side=tk.LEFT, padx=5)
        
        self.time_var = tk.IntVar(value=0)
        self.time_slider = tk.Scale(
            slider_frame,
            from_=0,
            to=1000 if not self.animation else self.animation.duration_ms,
            orient=tk.HORIZONTAL,
            variable=self.time_var,
            command=self._on_slider_change,
            bg='#2a2a2a',
            fg='white',
            highlightthickness=0,
            troughcolor='#1e1e1e',
            activebackground='#00ff00'
        )
        self.time_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Duration control
        duration_frame = tk.Frame(self, bg='#2a2a2a')
        duration_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(
            duration_frame,
            text="Duration:",
            bg='#2a2a2a',
            fg='#aaaaaa'
        ).pack(side=tk.LEFT, padx=5)
        
        self.duration_var = tk.IntVar(
            value=1000 if not self.animation else self.animation.duration_ms
        )
        duration_spinbox = tk.Spinbox(
            duration_frame,
            from_=100,
            to=60000,
            increment=100,
            textvariable=self.duration_var,
            command=self._on_duration_change,
            width=10,
            bg='#3a3a3a',
            fg='white',
            buttonbackground='#3a3a3a',
            readonlybackground='#3a3a3a'
        )
        duration_spinbox.pack(side=tk.LEFT, padx=5)
        
        tk.Label(
            duration_frame,
            text="ms",
            bg='#2a2a2a',
            fg='#aaaaaa'
        ).pack(side=tk.LEFT)
        
        # Panel checkboxes - создаем в переданном контейнере или здесь
        if self.panel_checkboxes_container:
            # Используем внешний контейнер (в правой панели)
            checkboxes_parent = self.panel_checkboxes_container
        else:
            # Создаем свой контейнер в timeline (fallback)
            panels_frame = tk.LabelFrame(
                self,
                text="🎨 ACTIVE PANELS",
                bg='#2a2a2a',
                fg='#00ff00',
                font=('Arial', 10, 'bold')
            )
            panels_frame.pack(fill=tk.X, padx=10, pady=5)
            checkboxes_parent = panels_frame
        
        # Подсказка
        self.hint_label = tk.Label(
            checkboxes_parent,
            text="💡 Add a keyframe first!",
            bg='#2a2a2a',
            fg='#ffaa00',
            font=('Arial', 9, 'italic')
        )
        self.hint_label.pack(pady=5)
        
        self.panel_vars: Dict[str, tk.BooleanVar] = {}
        self.panel_checkboxes: Dict[str, tk.Checkbutton] = {}
        
        # Контейнер для чекбоксов
        checkboxes_container = tk.Frame(checkboxes_parent, bg='#2a2a2a')
        checkboxes_container.pack(pady=5, fill=tk.X)
        
        panel_names = [
            ('wheel_front_left', '🔴 FL Wheel'),
            ('wheel_front_right', '🔴 FR Wheel'),
            ('wheel_rear_left', '🔵 RL Wheel'),
            ('wheel_rear_right', '🔵 RR Wheel'),
            ('main_display', '🟢 Main Display')
        ]
        
        for panel_name, label in panel_names:
            var = tk.BooleanVar(value=False)
            self.panel_vars[panel_name] = var
            
            cb = tk.Checkbutton(
                checkboxes_container,
                text=label,
                variable=var,
                command=lambda p=panel_name: self._on_panel_toggle(p),
                bg='#2a2a2a',
                fg='white',
                selectcolor='#00aa00',
                activebackground='#2a2a2a',
                activeforeground='#00ff00',
                font=('Arial', 10, 'bold'),
                relief=tk.FLAT,
                bd=1,
                padx=5,
                pady=5,
                cursor='hand2',
                anchor=tk.W
            )
            cb.pack(fill=tk.X, padx=10, pady=3)
            self.panel_checkboxes[panel_name] = cb
        
        # Обновить отображение
        self._update_timeline()
    
    def _update_timeline(self):
        """Обновить отображение временной шкалы"""
        self.timeline_canvas.delete('all')
        
        if not self.animation:
            return
        
        canvas_width = self.timeline_canvas.winfo_width()
        if canvas_width <= 1:
            canvas_width = 800
        
        duration = self.animation.duration_ms
        
        # Нарисовать линию времени
        self.timeline_canvas.create_line(
            10, 20, canvas_width - 10, 20,
            fill='#555555', width=2
        )
        
        # Нарисовать маркеры ключевых кадров
        for kf in self.animation.keyframes:
            x = 10 + (canvas_width - 20) * (kf.time_ms / duration)
            
            # Маркер (используем id для сравнения, не ==)
            color = '#00ff00' if kf is self.current_keyframe else '#ffaa00'
            self.timeline_canvas.create_oval(
                x - self.MARKER_SIZE//2, 20 - self.MARKER_SIZE//2,
                x + self.MARKER_SIZE//2, 20 + self.MARKER_SIZE//2,
                fill=color, outline='white', width=2,
                tags=('keyframe', f'kf_{id(kf)}')
            )
            
            # Время
            self.timeline_canvas.create_text(
                x, 35,
                text=f"{kf.time_ms}",
                fill='#aaaaaa',
                font=('Courier', 8)
            )
        
        # Текущая позиция (красная линия)
        x_current = 10 + (canvas_width - 20) * (self.current_time_ms / duration)
        self.timeline_canvas.create_line(
            x_current, 0, x_current, 40,
            fill='#ff0000', width=2,
            tags='current_pos'
        )
    
    def _on_timeline_click(self, event):
        """Обработка клика на timeline"""
        if not self.animation:
            return
        
        canvas_width = self.timeline_canvas.winfo_width()
        duration = self.animation.duration_ms
        
        # Вычислить время по клику
        x = event.x
        time_ms = int((x - 10) / (canvas_width - 20) * duration)
        time_ms = max(0, min(time_ms, duration))
        
        # Проверить, кликнули ли на ключевой кадр
        for kf in self.animation.keyframes:
            kf_x = 10 + (canvas_width - 20) * (kf.time_ms / duration)
            if abs(x - kf_x) < self.MARKER_SIZE:
                # Выбрать этот ключевой кадр
                self.set_time(kf.time_ms)
                return
        
        # Просто перейти к времени
        self.set_time(time_ms)
    
    def _on_slider_change(self, value):
        """Обработка изменения слайдера"""
        time_ms = int(float(value))
        self.set_time(time_ms)
    
    def _on_duration_change(self):
        """Обработка изменения длительности"""
        if self.animation:
            self.animation.duration_ms = self.duration_var.get()
            self.time_slider.config(to=self.animation.duration_ms)
            self._update_timeline()
    
    def _add_keyframe(self):
        """Добавить ключевой кадр"""
        if not self.animation:
            return
        
        # Добавить на текущей позиции
        kf = self.animation.add_keyframe(self.current_time_ms)
        self.current_keyframe = kf
        
        self._update_timeline()
        self._update_panel_checkboxes()
        
        if self.on_keyframe_change:
            self.on_keyframe_change()
    
    def _delete_keyframe(self):
        """Удалить текущий ключевой кадр"""
        if not self.animation or not self.current_keyframe:
            return
        
        if len(self.animation.keyframes) <= 1:
            return  # Нельзя удалить последний
        
        self.animation.remove_keyframe(self.current_keyframe)
        self.current_keyframe = self.animation.keyframes[0] if self.animation.keyframes else None
        
        self._update_timeline()
        self._update_panel_checkboxes()
        
        if self.on_keyframe_change:
            self.on_keyframe_change()
    
    def _on_panel_toggle(self, panel_name: str):
        """Обработка переключения чекбокса панели"""
        if not self.current_keyframe:
            return
        
        active = self.panel_vars[panel_name].get()
        self.current_keyframe.active_panels[panel_name] = active
        
        if self.on_keyframe_change:
            self.on_keyframe_change()
    
    def set_time(self, time_ms: int):
        """Установить текущее время"""
        if not self.animation:
            return
        
        self.current_time_ms = max(0, min(time_ms, self.animation.duration_ms))
        self.time_var.set(self.current_time_ms)
        self.time_label.config(text=f"{self.current_time_ms} ms")
        
        # Найти ближайший ключевой кадр
        kf = self.animation.get_keyframe_at(self.current_time_ms)
        if kf:
            self.current_keyframe = kf
            self.delete_keyframe_btn.config(state=tk.NORMAL)
        else:
            # Показываем ближайший предыдущий
            self.current_keyframe = None
            for k in reversed(self.animation.keyframes):
                if k.time_ms <= self.current_time_ms:
                    self.current_keyframe = k
                    break
            if not self.current_keyframe:
                self.current_keyframe = self.animation.keyframes[0] if self.animation.keyframes else None
            self.delete_keyframe_btn.config(state=tk.DISABLED)
        
        self._update_timeline()
        self._update_panel_checkboxes()
        
        if self.on_time_change:
            self.on_time_change(self.current_time_ms)
    
    def _update_panel_checkboxes(self):
        """Обновить состояние чекбоксов панелей"""
        if not self.current_keyframe:
            # Нет ключевого кадра - показать подсказку
            self.hint_label.config(
                text="⚠️  Add a keyframe first! Click '+ Add Keyframe' button above.",
                fg='#ff4444'
            )
            self.hint_label.pack(pady=5)
            for var in self.panel_vars.values():
                var.set(False)
            for cb in self.panel_checkboxes.values():
                cb.config(state=tk.DISABLED, fg='#666666')
            return
        
        # Есть ключевой кадр - скрыть подсказку, показать инструкцию
        self.hint_label.config(
            text="✓ Click checkboxes below to enable panels, then click on robot panels to draw",
            fg='#00ff00'
        )
        self.hint_label.pack(pady=5)
        
        # Включить чекбоксы
        for cb in self.panel_checkboxes.values():
            cb.config(state=tk.NORMAL, fg='white')
        
        # Установить значения
        for panel_name, var in self.panel_vars.items():
            active = self.current_keyframe.is_panel_active(panel_name)
            var.set(active)
            # Подсветить активные
            if active:
                self.panel_checkboxes[panel_name].config(fg='#00ff00')
            else:
                self.panel_checkboxes[panel_name].config(fg='white')
    
    def set_animation(self, animation: KeyframeAnimation):
        """Установить анимацию"""
        self.animation = animation
        self.current_time_ms = 0
        self.current_keyframe = animation.keyframes[0] if animation.keyframes else None
        
        self.duration_var.set(animation.duration_ms)
        self.time_slider.config(to=animation.duration_ms)
        
        self._update_timeline()
        self._update_panel_checkboxes()
    
    def get_current_keyframe(self) -> Optional[Keyframe]:
        """Получить текущий ключевой кадр"""
        return self.current_keyframe
