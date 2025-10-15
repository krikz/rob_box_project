"""
Data models для LED анимаций
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
from pathlib import Path
import numpy as np
from PIL import Image
import yaml


@dataclass
class Frame:
    """Один кадр анимации"""
    duration_ms: int = 100  # Длительность кадра в миллисекундах
    image: Optional[np.ndarray] = None  # RGB данные (H, W, 3)
    image_path: Optional[Path] = None  # Путь к файлу изображения
    
    def __post_init__(self):
        """Инициализация после создания"""
        if self.image is None and self.image_path is None:
            # Создать пустое изображение по умолчанию (8x8 black)
            self.image = np.zeros((8, 8, 3), dtype=np.uint8)
    
    @property
    def width(self) -> int:
        """Ширина кадра"""
        if self.image is not None:
            return self.image.shape[1]
        return 0
    
    @property
    def height(self) -> int:
        """Высота кадра"""
        if self.image is not None:
            return self.image.shape[0]
        return 0
    
    def load_image(self):
        """Загрузить изображение из файла"""
        if self.image_path and self.image_path.exists():
            img = Image.open(self.image_path)
            if img.mode != 'RGB':
                img = img.convert('RGB')
            self.image = np.array(img)
    
    def save_image(self, path: Path):
        """Сохранить изображение в файл"""
        if self.image is not None:
            img = Image.fromarray(self.image)
            img.save(path)
            self.image_path = path
    
    def get_pixel(self, x: int, y: int) -> Tuple[int, int, int]:
        """Получить цвет пикселя"""
        if self.image is not None and 0 <= y < self.height and 0 <= x < self.width:
            return tuple(self.image[y, x])
        return (0, 0, 0)
    
    def set_pixel(self, x: int, y: int, color: Tuple[int, int, int]):
        """Установить цвет пикселя"""
        if self.image is not None and 0 <= y < self.height and 0 <= x < self.width:
            self.image[y, x] = color
    
    def resize(self, width: int, height: int):
        """Изменить размер кадра"""
        if self.image is not None:
            img = Image.fromarray(self.image)
            img = img.resize((width, height), Image.NEAREST)
            self.image = np.array(img)
        else:
            self.image = np.zeros((height, width, 3), dtype=np.uint8)
    
    def copy(self) -> 'Frame':
        """Создать копию кадра"""
        return Frame(
            duration_ms=self.duration_ms,
            image=self.image.copy() if self.image is not None else None,
            image_path=self.image_path
        )


@dataclass
class Panel:
    """Панель анимации (один logical_group)"""
    logical_group: str  # wheel_front_left, wheel_front_right, etc.
    frames: List[Frame] = field(default_factory=list)
    offset_ms: int = 0  # Смещение начала воспроизведения
    
    @property
    def total_duration_ms(self) -> int:
        """Общая длительность панели"""
        return sum(f.duration_ms for f in self.frames)
    
    def add_frame(self, frame: Frame, index: Optional[int] = None):
        """Добавить кадр"""
        if index is None:
            self.frames.append(frame)
        else:
            self.frames.insert(index, frame)
    
    def remove_frame(self, index: int):
        """Удалить кадр"""
        if 0 <= index < len(self.frames):
            del self.frames[index]
    
    def duplicate_frame(self, index: int):
        """Дублировать кадр"""
        if 0 <= index < len(self.frames):
            frame_copy = self.frames[index].copy()
            self.frames.insert(index + 1, frame_copy)
    
    def get_frame_at_time(self, time_ms: int) -> Optional[Tuple[Frame, int]]:
        """Получить кадр в указанное время (относительно offset)"""
        time_ms -= self.offset_ms
        if time_ms < 0:
            return None
        
        current_time = 0
        for i, frame in enumerate(self.frames):
            if current_time <= time_ms < current_time + frame.duration_ms:
                return (frame, i)
            current_time += frame.duration_ms
        
        return None


@dataclass
class Animation:
    """Анимация (коллекция панелей)"""
    name: str = "untitled"
    description: str = ""
    fps: int = 10
    loop: bool = True
    panels: Dict[str, Panel] = field(default_factory=dict)
    
    # Поддерживаемые logical groups
    LOGICAL_GROUPS = [
        'wheel_front_left',
        'wheel_front_right',
        'wheel_rear_left',
        'wheel_rear_right',
        'main_display',
        'wheels_front',
        'wheels_rear',
        'wheels_all',
    ]
    
    # Размеры панелей
    PANEL_SIZES = {
        'wheel_front_left': (8, 8),
        'wheel_front_right': (8, 8),
        'wheel_rear_left': (8, 8),
        'wheel_rear_right': (8, 8),
        'main_display': (25, 5),
        'wheels_front': (16, 8),
        'wheels_rear': (16, 8),
        'wheels_all': (16, 16),
    }
    
    def __post_init__(self):
        """Инициализация"""
        # Создать пустые панели для всех групп
        for group in self.LOGICAL_GROUPS:
            if group not in self.panels:
                self.panels[group] = Panel(logical_group=group)
    
    @property
    def duration_ms(self) -> int:
        """Общая длительность анимации"""
        max_duration = 0
        for panel in self.panels.values():
            duration = panel.offset_ms + panel.total_duration_ms
            max_duration = max(max_duration, duration)
        return max_duration
    
    def get_panel_size(self, logical_group: str) -> Tuple[int, int]:
        """Получить размер панели (width, height)"""
        return self.PANEL_SIZES.get(logical_group, (8, 8))
    
    @classmethod
    def load_from_manifest(cls, manifest_path: Path, animations_dir: Path) -> 'Animation':
        """Загрузить анимацию из manifest файла"""
        with open(manifest_path, 'r') as f:
            data = yaml.safe_load(f)
        
        anim = cls(
            name=data.get('name', manifest_path.stem),
            description=data.get('description', ''),
            fps=data.get('fps', 10),
            loop=data.get('loop', True),
        )
        
        # Загрузить панели
        for panel_data in data.get('panels', []):
            logical_group = panel_data['logical_group']
            offset_ms = panel_data.get('offset_ms', 0)
            
            panel = Panel(logical_group=logical_group, offset_ms=offset_ms)
            
            for frame_data in panel_data.get('frames', []):
                duration_ms = frame_data.get('duration_ms', 100)
                image_path = animations_dir / frame_data['image']
                
                frame = Frame(duration_ms=duration_ms, image_path=image_path)
                frame.load_image()
                
                panel.add_frame(frame)
            
            anim.panels[logical_group] = panel
        
        return anim
    
    def save_to_manifest(self, manifest_path: Path, frames_dir: Path):
        """Сохранить анимацию в manifest файл"""
        # Создать директорию для кадров
        frames_dir.mkdir(parents=True, exist_ok=True)
        
        manifest = {
            'name': self.name,
            'description': self.description,
            'duration_ms': self.duration_ms,
            'fps': self.fps,
            'loop': self.loop,
            'panels': []
        }
        
        # Сохранить панели
        for logical_group, panel in self.panels.items():
            if not panel.frames:
                continue
            
            panel_data = {
                'logical_group': logical_group,
                'offset_ms': panel.offset_ms,
                'frames': []
            }
            
            # Сохранить кадры
            for i, frame in enumerate(panel.frames):
                # Имя файла: логическая группа + индекс
                frame_filename = f"{logical_group}_frame_{i:03d}.png"
                frame_path = frames_dir / frame_filename
                
                # Сохранить изображение
                frame.save_image(frame_path)
                
                # Относительный путь от animations_dir
                relative_path = f"frames/{self.name}/{frame_filename}"
                
                frame_data = {
                    'image': relative_path,
                    'duration_ms': frame.duration_ms
                }
                
                panel_data['frames'].append(frame_data)
            
            manifest['panels'].append(panel_data)
        
        # Сохранить manifest
        with open(manifest_path, 'w') as f:
            yaml.dump(manifest, f, default_flow_style=False, sort_keys=False)
    
    def create_new_frame(self, logical_group: str) -> Frame:
        """Создать новый кадр для панели"""
        width, height = self.get_panel_size(logical_group)
        frame = Frame(duration_ms=int(1000 / self.fps))
        frame.resize(width, height)
        return frame
