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
    
    def load_image(self, path: Optional[Path] = None):
        """
        Загрузить изображение из файла
        
        Args:
            path: Путь к файлу (если None, использует self.image_path)
        """
        image_path = path or self.image_path
        if image_path and image_path.exists():
            img = Image.open(image_path)
            if img.mode != 'RGB':
                img = img.convert('RGB')
            self.image = np.array(img)
            if path:
                self.image_path = path
    
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


@dataclass
class Keyframe:
    """
    Ключевой кадр - момент времени с состоянием всех панелей
    """
    time_ms: int  # Время в миллисекундах от начала анимации
    panel_states: Dict[str, Frame] = field(default_factory=dict)  # Состояние каждой панели
    active_panels: Dict[str, bool] = field(default_factory=dict)  # Какие панели активны
    
    def set_panel_state(self, panel_name: str, frame: Frame, active: bool = True):
        """Установить состояние панели"""
        self.panel_states[panel_name] = frame
        self.active_panels[panel_name] = active
    
    def get_panel_state(self, panel_name: str) -> Optional[Frame]:
        """Получить состояние панели"""
        return self.panel_states.get(panel_name)
    
    def is_panel_active(self, panel_name: str) -> bool:
        """Проверить активна ли панель"""
        return self.active_panels.get(panel_name, False)
    
    def copy(self) -> 'Keyframe':
        """Создать копию ключевого кадра"""
        kf = Keyframe(time_ms=self.time_ms)
        for panel_name, frame in self.panel_states.items():
            kf.panel_states[panel_name] = frame.copy()
            kf.active_panels[panel_name] = self.active_panels.get(panel_name, False)
        return kf


class KeyframeAnimation:
    """
    Анимация на основе ключевых кадров
    Более современный подход - единая временная шкала
    """
    def __init__(self, name: str = "Untitled", description: str = ""):
        self.name = name
        self.description = description
        self.keyframes: List[Keyframe] = []
        self.duration_ms: int = 1000  # Общая длительность
        self.fps: int = 10
        self.loop: bool = False
        
        # Дополнительные метаданные (опционально)
        self.version: Optional[str] = None
        self.author: Optional[str] = None
        self.metadata: Dict[str, any] = {}  # Для любых других полей
        
        # Создать первый ключевой кадр
        self.add_keyframe(0)
    
    def add_keyframe(self, time_ms: int) -> Keyframe:
        """Добавить ключевой кадр"""
        kf = Keyframe(time_ms=time_ms)
        
        # Инициализировать все стандартные панели
        for panel_name in ['wheel_front_left', 'wheel_front_right', 
                          'wheel_rear_left', 'wheel_rear_right', 'main_display']:
            width, height = Animation.PANEL_SIZES.get(panel_name, (8, 8))
            frame = Frame()
            frame.resize(width, height)
            kf.set_panel_state(panel_name, frame, active=False)
        
        self.keyframes.append(kf)
        self.keyframes.sort(key=lambda k: k.time_ms)
        return kf
    
    def remove_keyframe(self, keyframe: Keyframe):
        """Удалить ключевой кадр"""
        if keyframe in self.keyframes and len(self.keyframes) > 1:
            self.keyframes.remove(keyframe)
    
    def get_keyframe_at(self, time_ms: int) -> Optional[Keyframe]:
        """Получить ключевой кадр в конкретный момент времени"""
        for kf in self.keyframes:
            if kf.time_ms == time_ms:
                return kf
        return None
    
    def get_nearest_keyframe(self, time_ms: int) -> Optional[Keyframe]:
        """Получить ближайший ключевой кадр"""
        if not self.keyframes:
            return None
        return min(self.keyframes, key=lambda kf: abs(kf.time_ms - time_ms))
    
    def get_state_at_time(self, time_ms: int) -> Dict[str, Frame]:
        """
        Получить состояние всех панелей в конкретный момент времени
        (интерполяция между ключевыми кадрами)
        """
        if not self.keyframes:
            return {}
        
        # Найти ближайший предыдущий ключевой кадр
        prev_kf = None
        for kf in reversed(self.keyframes):
            if kf.time_ms <= time_ms:
                prev_kf = kf
                break
        
        if not prev_kf:
            prev_kf = self.keyframes[0]
        
        # Вернуть состояние активных панелей
        result = {}
        for panel_name, frame in prev_kf.panel_states.items():
            if prev_kf.is_panel_active(panel_name):
                result[panel_name] = frame
        
        return result

    def save_to_manifest(self, manifest_path: Path, frames_dir: Path):
        """Сохранить keyframe-анимацию в manifest файл с сохранением метаданных"""
        frames_dir.mkdir(parents=True, exist_ok=True)
        
        manifest = {
            'name': self.name,
            'description': self.description,
            'duration_ms': self.duration_ms,
            'fps': self.fps,
            'loop': self.loop,
            'format': 'keyframe',  # Маркер нового формата
        }
        
        # Добавить опциональные метаданные если они есть
        if self.version:
            manifest['version'] = self.version
        if self.author:
            manifest['author'] = self.author
        
        # Добавить любые дополнительные метаданные
        for key, value in self.metadata.items():
            if key not in manifest:  # Не перезаписывать основные поля
                manifest[key] = value
        
        # Добавить keyframes
        manifest['keyframes'] = []
        
        # Сохранить ключевые кадры
        for kf_idx, kf in enumerate(self.keyframes):
            kf_data = {
                'time_ms': kf.time_ms,
                'panels': []
            }
            
            # Сохранить состояние каждой панели
            for panel_name, frame in kf.panel_states.items():
                if not kf.is_panel_active(panel_name):
                    continue  # Пропустить неактивные панели
                
                # Имя файла: keyframe_панель_индекс
                frame_filename = f"kf{kf_idx:03d}_{panel_name}.png"
                frame_path = frames_dir / frame_filename
                
                # Сохранить изображение
                frame.save_image(frame_path)
                
                # Относительный путь
                relative_path = f"frames/{self.name}/{frame_filename}"
                
                panel_data = {
                    'panel': panel_name,
                    'image': relative_path,
                    'active': True
                }
                
                kf_data['panels'].append(panel_data)
            
            manifest['keyframes'].append(kf_data)
        
        # Сохранить manifest
        with open(manifest_path, 'w') as f:
            yaml.dump(manifest, f, default_flow_style=False, sort_keys=False)
    
    @classmethod
    def load_from_manifest(cls, manifest_path: Path, animations_dir: Path) -> 'KeyframeAnimation':
        """
        Загрузить keyframe-анимацию из manifest файла
        Поддерживает как новый keyframe формат, так и старый frame-based формат
        """
        with open(manifest_path, 'r') as f:
            data = yaml.safe_load(f)
        
        name = data.get('name', 'Untitled')
        description = data.get('description', '')
        
        # Проверить формат
        if data.get('format') == 'keyframe':
            # Новый keyframe-based формат
            return cls._load_keyframe_format(data, animations_dir)
        else:
            # Старый frame-based формат - конвертировать
            return cls._convert_from_animation(data, animations_dir)
    
    @classmethod
    def _load_keyframe_format(cls, data: dict, animations_dir: Path) -> 'KeyframeAnimation':
        """Загрузить из нового keyframe формата"""
        anim = cls.__new__(cls)  # Создать без вызова __init__
        anim.name = data.get('name', 'Untitled')
        anim.description = data.get('description', '')
        anim.duration_ms = data.get('duration_ms', 1000)
        anim.fps = data.get('fps', 10)
        anim.loop = data.get('loop', False)
        anim.keyframes = []
        
        # Загрузить опциональные метаданные
        anim.version = data.get('version')
        anim.author = data.get('author')
        
        # Сохранить дополнительные поля в metadata
        anim.metadata = {}
        skip_keys = {'name', 'description', 'duration_ms', 'fps', 'loop', 'format', 
                     'keyframes', 'panels', 'version', 'author'}
        for key, value in data.items():
            if key not in skip_keys:
                anim.metadata[key] = value
        
        # Загрузить ключевые кадры
        for kf_data in data.get('keyframes', []):
            time_ms = kf_data.get('time_ms', 0)
            kf = Keyframe(time_ms=time_ms)
            
            # Инициализировать все панели пустыми кадрами
            for panel_name in ['wheel_front_left', 'wheel_front_right',
                              'wheel_rear_left', 'wheel_rear_right', 'main_display']:
                width, height = Animation.PANEL_SIZES.get(panel_name, (8, 8))
                frame = Frame()
                frame.resize(width, height)
                kf.set_panel_state(panel_name, frame, active=False)
            
            # Загрузить состояние активных панелей
            for panel_data in kf_data.get('panels', []):
                panel_name = panel_data.get('panel')
                image_path = animations_dir / panel_data['image']
                
                frame = Frame()
                frame.load_image(image_path)
                
                is_active = panel_data.get('active', True)
                kf.set_panel_state(panel_name, frame, active=is_active)
            
            anim.keyframes.append(kf)
        
        # Если нет ключевых кадров, создать дефолтный
        if not anim.keyframes:
            anim.add_keyframe(0)
        
        return anim
    
    @classmethod
    def _convert_from_animation(cls, data: dict, animations_dir: Path) -> 'KeyframeAnimation':
        """
        Конвертировать из старого Animation формата в KeyframeAnimation
        Создает ключевые кадры из последовательности кадров
        Сохраняет все метаданные из оригинального файла
        """
        anim = cls.__new__(cls)
        anim.name = data.get('name', 'Untitled')
        anim.description = data.get('description', '')
        anim.fps = data.get('fps', 10)
        anim.loop = data.get('loop', False)
        anim.keyframes = []
        
        # Сохранить метаданные из старого файла
        anim.version = data.get('version')
        anim.author = data.get('author')
        
        # Сохранить любые дополнительные поля
        anim.metadata = {}
        skip_keys = {'name', 'description', 'duration_ms', 'fps', 'loop', 'format', 
                     'keyframes', 'panels', 'version', 'author'}
        for key, value in data.items():
            if key not in skip_keys:
                anim.metadata[key] = value
        
        # Собрать информацию о всех панелях
        panels_info = {}
        for panel_data in data.get('panels', []):
            logical_group = panel_data.get('logical_group')
            offset_ms = panel_data.get('offset_ms', 0)
            frames = panel_data.get('frames', [])
            
            panels_info[logical_group] = {
                'offset_ms': offset_ms,
                'frames': frames
            }
        
        # Вычислить общую длительность
        max_duration = 0
        for panel_name, info in panels_info.items():
            panel_duration = info['offset_ms']
            for frame_data in info['frames']:
                panel_duration += frame_data.get('duration_ms', 100)
            max_duration = max(max_duration, panel_duration)
        
        anim.duration_ms = max_duration if max_duration > 0 else 1000
        
        # Создать ключевые кадры для каждого уникального момента времени
        time_points = set([0])  # Всегда есть начальный кадр
        
        for panel_name, info in panels_info.items():
            current_time = info['offset_ms']
            for frame_data in info['frames']:
                time_points.add(current_time)
                current_time += frame_data.get('duration_ms', 100)
        
        time_points = sorted(time_points)
        
        # Создать ключевые кадры
        for time_ms in time_points:
            if time_ms > anim.duration_ms:
                break
            
            kf = Keyframe(time_ms=time_ms)
            
            # Для каждой панели определить какой кадр показывать в этот момент
            for panel_name in ['wheel_front_left', 'wheel_front_right',
                              'wheel_rear_left', 'wheel_rear_right', 'main_display']:
                width, height = Animation.PANEL_SIZES.get(panel_name, (8, 8))
                frame = Frame()
                frame.resize(width, height)
                
                # Найти соответствующий кадр из старой анимации
                if panel_name in panels_info:
                    info = panels_info[panel_name]
                    current_time = info['offset_ms']
                    
                    for frame_data in info['frames']:
                        frame_end = current_time + frame_data.get('duration_ms', 100)
                        if current_time <= time_ms < frame_end:
                            # Это нужный кадр
                            image_path = animations_dir / frame_data['image']
                            if image_path.exists():
                                frame.load_image(image_path)
                            kf.set_panel_state(panel_name, frame, active=True)
                            break
                        current_time = frame_end
                    else:
                        # Кадр не найден, оставить пустым
                        kf.set_panel_state(panel_name, frame, active=False)
                else:
                    kf.set_panel_state(panel_name, frame, active=False)
            
            anim.keyframes.append(kf)
        
        # Если нет ключевых кадров, создать дефолтный
        if not anim.keyframes:
            anim.add_keyframe(0)
        
        return anim
