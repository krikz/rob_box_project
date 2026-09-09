"""
Animation Editor Application - главное приложение
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from pathlib import Path
from typing import Optional

from .models import KeyframeAnimation, Keyframe, Frame
from .canvas import RobotCanvas
from .timeline_v2 import TimelineV2
from .palette_simple import ColorPalette


class AnimationEditorApp:
    """Главное приложение редактора анимаций"""
    
    def __init__(self, root: tk.Tk, animations_dir: Path):
        """
        Инициализация приложения
        
        Args:
            root: Tkinter root window
            animations_dir: Путь к директории с анимациями
        """
        self.root = root
        self.animations_dir = animations_dir
        
        # Текущая анимация (новая архитектура с ключевыми кадрами)
        self.animation: Optional[KeyframeAnimation] = None
        self.animation_file: Optional[Path] = None
        self.is_modified = False
        
        # Playback control
        self._playback_after_id: Optional[str] = None
        self._is_playing = False
        
        # Настройка окна
        self.root.title("rob_box LED Animation Editor")
        self.root.geometry("1400x900")
        self.root.configure(bg='#1e1e1e')
        
        # Создать GUI
        self._create_menu()
        self._create_widgets()
        
        # Bind закрытие окна
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        
        # Создать новую анимацию по умолчанию
        self._new_animation()
    
    def _create_menu(self):
        """Создать меню"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        
        file_menu.add_command(label="New Animation", command=self._new_animation, accelerator="Ctrl+N")
        file_menu.add_command(label="Open...", command=self._open_animation, accelerator="Ctrl+O")
        file_menu.add_command(label="Save", command=self._save_animation, accelerator="Ctrl+S")
        file_menu.add_command(label="Save As...", command=self._save_animation_as, accelerator="Ctrl+Shift+S")
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self._on_closing, accelerator="Ctrl+Q")
        
        # Edit menu
        edit_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Edit", menu=edit_menu)
        
        edit_menu.add_command(label="Clear Frame", command=self._clear_frame)
        edit_menu.add_command(label="Fill Frame", command=self._fill_frame)
        
        # View menu
        view_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="View", menu=view_menu)
        
        view_menu.add_command(label="Play Animation", command=self._play_animation)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        
        help_menu.add_command(label="About", command=self._show_about)
        
        # Bind клавиши
        self.root.bind('<Control-n>', lambda e: self._new_animation())
        self.root.bind('<Control-o>', lambda e: self._open_animation())
        self.root.bind('<Control-s>', lambda e: self._save_animation())
        self.root.bind('<Control-Shift-S>', lambda e: self._save_animation_as())
        self.root.bind('<Control-q>', lambda e: self._on_closing())
    
    def _create_widgets(self):
        """Создать виджеты"""
        # Главный container
        main_container = tk.Frame(self.root, bg='#1e1e1e')
        main_container.pack(fill=tk.BOTH, expand=True)
        
        # Toolbar
        toolbar = self._create_toolbar(main_container)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        # Центральная область (3 колонки)
        center = tk.Frame(main_container, bg='#1e1e1e')
        center.pack(fill=tk.BOTH, expand=True, padx=5)
        
        # Левая панель - Palette
        left_panel = tk.Frame(center, bg='#1e1e1e', width=250)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        left_panel.pack_propagate(False)
        
        self.palette = ColorPalette(left_panel, on_color_change=self._on_color_change)
        self.palette.pack(fill=tk.BOTH, expand=True)
        
        # Центральная панель - Canvas
        center_panel = tk.Frame(center, bg='#1e1e1e')
        center_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        self.canvas = RobotCanvas(center_panel, on_pixel_click=self._on_pixel_click)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Правая панель - Properties
        right_panel = tk.Frame(center, bg='#1e1e1e', width=300)
        right_panel.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        right_panel.pack_propagate(False)
        
        self._create_properties_panel(right_panel)
        
        # Нижняя панель - Timeline v2 (уменьшена, т.к. чекбоксы в правой панели)
        bottom_panel = tk.Frame(main_container, bg='#1e1e1e', height=180)
        bottom_panel.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
        bottom_panel.pack_propagate(False)
        
        self.timeline = TimelineV2(
            bottom_panel,
            animation=self.animation,
            on_time_change=self._on_time_change,
            on_keyframe_change=self._on_keyframe_change,
            panel_checkboxes_container=self.panel_checkboxes_container
        )
        self.timeline.pack(fill=tk.BOTH, expand=True)
        
        # Статус бар
        self.status_bar = tk.Label(
            self.root,
            text="Ready",
            bg='#2a2a2a',
            fg='#aaaaaa',
            anchor=tk.W,
            padx=10
        )
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
    
    def _create_toolbar(self, parent) -> tk.Frame:
        """Создать toolbar"""
        toolbar = tk.Frame(parent, bg='#2a2a2a', relief=tk.RAISED, borderwidth=1)
        
        # Инфо о текущей анимации
        self.anim_name_label = tk.Label(
            toolbar,
            text="Untitled Animation",
            bg='#2a2a2a',
            fg='white',
            font=('Arial', 12, 'bold')
        )
        self.anim_name_label.pack(side=tk.LEFT, padx=10)
        
        self.modified_label = tk.Label(
            toolbar,
            text="",
            bg='#2a2a2a',
            fg='#ff8800',
            font=('Arial', 10)
        )
        self.modified_label.pack(side=tk.LEFT)
        
        # Separator
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        
        # Кнопки быстрого доступа
        tk.Button(
            toolbar,
            text="▶ Play",
            command=self._play_animation,
            bg='#3a3a3a',
            fg='white'
        ).pack(side=tk.LEFT, padx=2)
        
        tk.Button(
            toolbar,
            text="💾 Save",
            command=self._save_animation,
            bg='#3a3a3a',
            fg='white'
        ).pack(side=tk.LEFT, padx=2)
        
        return toolbar
    
    def _create_properties_panel(self, parent):
        """Создать панель свойств"""
        props_frame = tk.Frame(parent, bg='#2a2a2a', relief=tk.RAISED, borderwidth=2)
        props_frame.pack(fill=tk.BOTH, expand=True)
        
        # Заголовок
        tk.Label(
            props_frame,
            text="⚙️ PROPERTIES",
            bg='#2a2a2a',
            fg='white',
            font=('Arial', 11, 'bold')
        ).pack(pady=5)
        
        ttk.Separator(props_frame, orient=tk.HORIZONTAL).pack(fill=tk.X)
        
        # Animation properties
        anim_frame = tk.LabelFrame(
            props_frame,
            text="Animation",
            bg='#2a2a2a',
            fg='white',
            font=('Arial', 10, 'bold')
        )
        anim_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Name
        tk.Label(anim_frame, text="Name:", bg='#2a2a2a', fg='#aaaaaa').grid(
            row=0, column=0, sticky=tk.W, padx=5, pady=5
        )
        self.name_entry = tk.Entry(anim_frame, bg='#3a3a3a', fg='white', width=20)
        self.name_entry.grid(row=0, column=1, padx=5, pady=5)
        self.name_entry.bind('<KeyRelease>', lambda e: self._on_property_change())
        
        # Description
        tk.Label(anim_frame, text="Description:", bg='#2a2a2a', fg='#aaaaaa').grid(
            row=1, column=0, sticky=tk.W, padx=5, pady=5
        )
        self.desc_entry = tk.Entry(anim_frame, bg='#3a3a3a', fg='white', width=20)
        self.desc_entry.grid(row=1, column=1, padx=5, pady=5)
        self.desc_entry.bind('<KeyRelease>', lambda e: self._on_property_change())
        
        # FPS
        tk.Label(anim_frame, text="FPS:", bg='#2a2a2a', fg='#aaaaaa').grid(
            row=2, column=0, sticky=tk.W, padx=5, pady=5
        )
        self.fps_var = tk.IntVar(value=10)
        tk.Spinbox(
            anim_frame,
            from_=1,
            to=60,
            textvariable=self.fps_var,
            width=18,
            bg='#3a3a3a',
            fg='white',
            command=self._on_property_change
        ).grid(row=2, column=1, padx=5, pady=5)
        
        # Loop
        self.loop_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            anim_frame,
            text="Loop",
            variable=self.loop_var,
            bg='#2a2a2a',
            fg='white',
            selectcolor='#3a3a3a',
            command=self._on_property_change
        ).grid(row=3, column=0, columnspan=2, padx=5, pady=5)
        
        # Info
        info_frame = tk.LabelFrame(
            props_frame,
            text="Info",
            bg='#2a2a2a',
            fg='white',
            font=('Arial', 10, 'bold')
        )
        info_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.info_text = tk.Label(
            info_frame,
            text="No animation loaded",
            bg='#2a2a2a',
            fg='#aaaaaa',
            justify=tk.LEFT,
            anchor=tk.W
        )
        self.info_text.pack(padx=10, pady=10, fill=tk.X)
        
        # ACTIVE PANELS - Чекбоксы панелей (перемещены сюда из Timeline)
        panels_frame = tk.LabelFrame(
            props_frame,
            text="🎨 Active Panels",
            bg='#2a2a2a',
            fg='#00ff00',
            font=('Arial', 10, 'bold')
        )
        panels_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Хранилища для чекбоксов (будут заполнены timeline)
        self.panel_checkboxes_container = panels_frame
    
    def _new_animation(self):
        """Создать новую анимацию"""
        if self.is_modified:
            result = messagebox.askyesnocancel(
                "Unsaved Changes",
                "Save changes to current animation?"
            )
            if result is True:
                self._save_animation()
            elif result is None:
                return
        
        # Создать новую анимацию с ключевыми кадрами
        self.animation = KeyframeAnimation(name="untitled", description="New animation")
        self.animation_file = None
        self.is_modified = False
        
        # Обновить timeline
        self.timeline.set_animation(self.animation)
        
        # Обновить GUI
        self._update_gui()
        self._update_title()
        
        self.status_bar.config(text="New animation created")
    
    def _open_animation(self):
        """Открыть анимацию"""
        if self.is_modified:
            result = messagebox.askyesnocancel(
                "Unsaved Changes",
                "Save changes to current animation?"
            )
            if result is True:
                self._save_animation()
            elif result is None:
                return
        
        manifests_dir = self.animations_dir / 'manifests'
        manifests_dir.mkdir(parents=True, exist_ok=True)
        
        filename = filedialog.askopenfilename(
            title="Open Animation",
            initialdir=manifests_dir,
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        
        if filename:
            self._load_from_file(Path(filename))
    
    def _load_from_file(self, filepath: Path):
        """Загрузить анимацию из файла"""
        try:
            # Загрузить анимацию (поддерживает оба формата)
            self.animation = KeyframeAnimation.load_from_manifest(filepath, self.animations_dir)
            self.animation_file = filepath
            self.is_modified = False
            
            # Обновить GUI
            self.name_entry.delete(0, tk.END)
            self.name_entry.insert(0, self.animation.name)
            
            self.desc_entry.delete(0, tk.END)
            self.desc_entry.insert(0, self.animation.description)
            
            self.fps_var.set(self.animation.fps)
            self.loop_var.set(self.animation.loop)
            
            # Обновить timeline
            self.timeline.set_animation(self.animation)
            
            # Обновить canvas
            self._refresh_canvas()
            
            # Обновить info
            self._update_info()
            
            # Обновить заголовок
            self.anim_name_label.config(text=f"📁 {self.animation.name}")
            self.status_bar.config(text=f"Loaded: {filepath.name}")
            
            messagebox.showinfo("Success", f"Animation '{self.animation.name}' loaded successfully!\n\nKeyframes: {len(self.animation.keyframes)}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load animation:\n{str(e)}")
            import traceback
            traceback.print_exc()
    
    def _save_animation(self):
        """Сохранить анимацию"""
        if self.animation_file:
            self._save_to_file(self.animation_file)
        else:
            self._save_animation_as()
    
    def _save_animation_as(self):
        """Сохранить анимацию как..."""
        manifests_dir = self.animations_dir / 'manifests'
        manifests_dir.mkdir(parents=True, exist_ok=True)
        
        filename = filedialog.asksaveasfilename(
            title="Save Animation As",
            initialdir=manifests_dir,
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        
        if filename:
            self._save_to_file(Path(filename))
    
    def _save_to_file(self, filepath: Path):
        """Сохранить в файл"""
        try:
            # Обновить свойства из GUI
            self.animation.name = self.name_entry.get()
            self.animation.description = self.desc_entry.get()
            self.animation.fps = self.fps_var.get()
            self.animation.loop = self.loop_var.get()
            
            # Создать директорию для кадров
            frames_dir = self.animations_dir / 'frames' / self.animation.name
            
            # Сохранить
            self.animation.save_to_manifest(filepath, frames_dir)
            
            self.animation_file = filepath
            self.is_modified = False
            self._update_title()
            
            self.status_bar.config(text=f"Saved: {filepath.name}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save animation:\n{e}")
    
    def _clear_frame(self):
        """Очистить текущий кадр"""
        frame = self.timeline.get_current_frame()
        if frame and frame.image is not None:
            frame.image.fill(0)
            self._refresh_canvas()
            self.is_modified = True
    
    def _fill_frame(self):
        """Заполнить кадр текущим цветом"""
        frame = self.timeline.get_current_frame()
        if frame and frame.image is not None:
            color = self.palette.get_color()
            frame.image[:, :] = color
            self._refresh_canvas()
            self.is_modified = True
    
    def _play_animation(self):
        """Воспроизвести анимацию (preview)"""
        if self._is_playing:
            # Если уже играет, остановить
            self._stop_playback()
            return
        
        if not self.animation:
            messagebox.showwarning("Warning", "No animation to play")
            return
        
        # В новой архитектуре играем все активные панели
        if not self.animation.keyframes:
            messagebox.showwarning("Warning", "No keyframes to play")
            return
        
        # Запустить проигрывание
        self._is_playing = True
        self.status_bar.config(text="Playing animation...")
        self._play_keyframes(0)
    
    def _play_keyframes(self, time_ms: int):
        """Проигрывать анимацию по времени"""
        if not self._is_playing:
            return
        
        if time_ms >= self.animation.duration_ms:
            # Если loop включен, начать сначала
            if self.animation.loop:
                time_ms = 0
            else:
                self._is_playing = False
                self.status_bar.config(text="Playback finished")
                return
        
        # Получить состояние всех панелей в текущий момент
        panel_states = self.animation.get_state_at_time(time_ms)
        
        # Обновить canvas
        for panel_name, frame in panel_states.items():
            self.canvas.update_panel_from_frame(panel_name, frame)
        
        self.status_bar.config(text=f"Playing: {time_ms} / {self.animation.duration_ms} ms")
        
        # Следующий кадр через 1000/fps миллисекунд
        delay_ms = int(1000 / self.animation.fps)
        self._playback_after_id = self.root.after(
            delay_ms,
            lambda: self._play_keyframes(time_ms + delay_ms)
        )
    
    def _stop_playback(self):
        """Остановить воспроизведение"""
        if self._playback_after_id:
            self.root.after_cancel(self._playback_after_id)
            self._playback_after_id = None
        self._is_playing = False
        self.status_bar.config(text="Playback stopped")
    
    def _show_about(self):
        """Показать информацию о программе"""
        messagebox.showinfo(
            "About",
            "rob_box LED Animation Editor\nVersion 1.0.0\n\n"
            "© 2025 rob_box team"
        )
    
    def _on_pixel_click(self, panel_name: str, x: int, y: int):
        """Обработка клика на пиксель"""
        keyframe = self.timeline.get_current_keyframe()
        if not keyframe:
            return
        
        # Проверить что панель активна
        if not keyframe.is_panel_active(panel_name):
            self.status_bar.config(text=f"Panel '{panel_name}' is not active. Enable it first!")
            return
        
        # Получить frame панели
        frame = keyframe.get_panel_state(panel_name)
        if not frame:
            return
        
        # Получить текущий цвет
        color = self.palette.get_color()
        
        # Установить пиксель
        frame.set_pixel(x, y, color)
        
        # Обновить canvas
        self.canvas.set_pixel_color(panel_name, x, y, color)
        
        self.is_modified = True
        self.status_bar.config(text=f"Pixel set: {panel_name} ({x},{y}) = {color}")
    
    def _on_color_change(self, color):
        """Обработка изменения цвета"""
        pass  # Цвет уже обновлён в палитре
    
    def _on_time_change(self, time_ms: int):
        """Обработка изменения времени на timeline"""
        if not self.animation:
            return
        
        # Получить состояние всех панелей в текущий момент
        panel_states = self.animation.get_state_at_time(time_ms)
        
        # Обновить canvas
        for panel_name, frame in panel_states.items():
            self.canvas.update_panel_from_frame(panel_name, frame)
        
        self.status_bar.config(text=f"Time: {time_ms} ms")
    
    def _on_keyframe_change(self):
        """Обработка изменения ключевых кадров"""
        self.is_modified = True
        self._update_info()
        
        # Обновить canvas с текущим ключевым кадром
        keyframe = self.timeline.get_current_keyframe()
        if keyframe:
            for panel_name, frame in keyframe.panel_states.items():
                if keyframe.is_panel_active(panel_name):
                    self.canvas.update_panel_from_frame(panel_name, frame)
    
    def _on_property_change(self):
        """Обработка изменения свойств"""
        self.is_modified = True
        self._update_title()
    
    def _refresh_canvas(self):
        """Обновить canvas - показать все активные панели текущего keyframe"""
        keyframe = self.timeline.get_current_keyframe()
        if keyframe:
            for panel_name, frame in keyframe.panel_states.items():
                if keyframe.is_panel_active(panel_name):
                    self.canvas.update_panel_from_frame(panel_name, frame)

    
    def _update_gui(self):
        """Обновить весь GUI"""
        if not self.animation:
            return
        
        # Обновить свойства
        self.name_entry.delete(0, tk.END)
        self.name_entry.insert(0, self.animation.name)
        
        self.desc_entry.delete(0, tk.END)
        self.desc_entry.insert(0, self.animation.description)
        
        self.fps_var.set(self.animation.fps)
        self.loop_var.set(self.animation.loop)
        
        # Обновить timeline
        self.timeline.set_animation(self.animation)
        
        # Обновить info
        self._update_info()
    
    def _update_info(self):
        """Обновить info панель"""
        if not self.animation:
            return
        
        # Для KeyframeAnimation считаем ключевые кадры
        total_keyframes = len(self.animation.keyframes)
        duration = self.animation.duration_ms
        
        info = (
            f"Keyframes: {total_keyframes}\n"
            f"Duration: {duration}ms ({duration/1000:.2f}s)\n"
            f"FPS: {self.animation.fps}\n"
            f"Loop: {'Yes' if self.animation.loop else 'No'}"
        )
        
        self.info_text.config(text=info, fg='white')
    
    def _update_title(self):
        """Обновить заголовок окна"""
        if not self.animation:
            return
        
        name = self.animation.name
        modified = "*" if self.is_modified else ""
        
        self.root.title(f"rob_box LED Animation Editor - {name}{modified}")
        self.anim_name_label.config(text=name)
        self.modified_label.config(text=modified)
    
    def _on_closing(self):
        """Обработка закрытия окна"""
        # Остановить воспроизведение если активно
        self._stop_playback()
        
        if self.is_modified:
            result = messagebox.askyesnocancel(
                "Unsaved Changes",
                "Save changes before closing?"
            )
            if result is True:
                self._save_animation()
            elif result is None:
                return
        
        self.root.destroy()
    
    def run(self):
        """Запустить приложение"""
        self.root.mainloop()
