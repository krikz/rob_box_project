"""
Animation Editor Application - главное приложение
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from pathlib import Path
from typing import Optional

from .models import Animation, Frame
from .canvas import RobotCanvas
from .timeline import Timeline
from .palette import ColorPalette


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
        
        # Текущая анимация
        self.animation: Optional[Animation] = None
        self.animation_file: Optional[Path] = None
        self.is_modified = False
        
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
        
        # Нижняя панель - Timeline
        bottom_panel = tk.Frame(main_container, bg='#1e1e1e', height=200)
        bottom_panel.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
        bottom_panel.pack_propagate(False)
        
        self.timeline = Timeline(
            bottom_panel,
            animation=self.animation,
            on_frame_select=self._on_frame_select,
            on_frame_change=self._on_frame_change
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
        
        self.animation = Animation(name="untitled", description="New animation")
        self.animation_file = None
        self.is_modified = False
        
        # Обновить GUI
        self._update_gui()
        self._update_title()
        
        self.status_bar.config(text="New animation created")
    
    def _open_animation(self):
        """Открыть анимацию"""
        manifests_dir = self.animations_dir / 'manifests'
        
        filename = filedialog.askopenfilename(
            title="Open Animation",
            initialdir=manifests_dir,
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        
        if not filename:
            return
        
        try:
            self.animation = Animation.load_from_manifest(
                Path(filename),
                self.animations_dir
            )
            self.animation_file = Path(filename)
            self.is_modified = False
            
            # Обновить GUI
            self._update_gui()
            self._update_title()
            
            self.status_bar.config(text=f"Opened: {self.animation.name}")
        
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open animation:\n{e}")
    
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
        """Воспроизвести анимацию (заглушка)"""
        # TODO: Implement animation playback
        messagebox.showinfo("Play Animation", "Animation playback not yet implemented")
    
    def _show_about(self):
        """Показать информацию о программе"""
        messagebox.showinfo(
            "About",
            "rob_box LED Animation Editor\nVersion 1.0.0\n\n"
            "© 2025 rob_box team"
        )
    
    def _on_pixel_click(self, panel_name: str, x: int, y: int):
        """Обработка клика на пиксель"""
        frame = self.timeline.get_current_frame()
        if not frame:
            return
        
        # Получить текущий цвет
        color = self.palette.get_color()
        
        # Установить пиксель
        frame.set_pixel(x, y, color)
        
        # Обновить canvas
        self.canvas.set_pixel_color(panel_name, x, y, color)
        
        # Обновить timeline preview
        self.timeline.refresh()
        
        self.is_modified = True
    
    def _on_color_change(self, color):
        """Обработка изменения цвета"""
        pass  # Цвет уже обновлён в палитре
    
    def _on_frame_select(self, panel_name: str, frame_index: int):
        """Обработка выбора кадра"""
        if not self.animation:
            return
        
        panel = self.animation.panels[panel_name]
        if frame_index < len(panel.frames):
            frame = panel.frames[frame_index]
            
            # Обновить canvas
            self.canvas.set_selected_panel(panel_name)
            self.canvas.update_panel_from_frame(panel_name, frame)
            
            self.status_bar.config(text=f"Selected: {panel_name} frame #{frame_index}")
    
    def _on_frame_change(self):
        """Обработка изменения кадров"""
        self.is_modified = True
        self._update_info()
    
    def _on_property_change(self):
        """Обработка изменения свойств"""
        self.is_modified = True
        self._update_title()
    
    def _refresh_canvas(self):
        """Обновить canvas"""
        if self.timeline.selected_panel:
            frame = self.timeline.get_current_frame()
            if frame:
                self.canvas.update_panel_from_frame(self.timeline.selected_panel, frame)
    
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
        
        total_frames = sum(len(p.frames) for p in self.animation.panels.values())
        duration = self.animation.duration_ms
        
        info = (
            f"Total frames: {total_frames}\n"
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
