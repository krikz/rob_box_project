"""
Audio utilities для работы с PyAudio и аудио данными
"""

import pyaudio
import numpy as np
from typing import Optional, List, Tuple


def find_respeaker_device(p: pyaudio.PyAudio) -> Optional[int]:
    """
    Найти индекс ReSpeaker устройства в PyAudio
    
    Args:
        p: PyAudio instance
        
    Returns:
        Device index или None если не найден
    """
    info = p.get_host_api_info_by_index(0)
    num_devices = info.get('deviceCount', 0)
    
    for i in range(num_devices):
        device_info = p.get_device_info_by_host_api_device_index(0, i)
        device_name = device_info.get('name', '')
        
        # Проверяем по имени
        if 'ReSpeaker' in device_name or 'ArrayUAC10' in device_name:
            # Проверяем что есть input каналы
            if device_info.get('maxInputChannels', 0) > 0:
                return i
    
    return None


def list_audio_devices(p: pyaudio.PyAudio) -> List[dict]:
    """
    Список всех доступных аудио устройств
    
    Returns:
        Список словарей с информацией об устройствах
    """
    devices = []
    info = p.get_host_api_info_by_index(0)
    num_devices = info.get('deviceCount', 0)
    
    for i in range(num_devices):
        try:
            device_info = p.get_device_info_by_host_api_device_index(0, i)
            if device_info.get('maxInputChannels', 0) > 0:
                devices.append({
                    'index': i,
                    'name': device_info.get('name', 'Unknown'),
                    'channels': device_info.get('maxInputChannels', 0),
                    'sample_rate': int(device_info.get('defaultSampleRate', 0)),
                })
        except Exception:
            continue
    
    return devices


def pcm16_to_float32(data: bytes) -> np.ndarray:
    """
    Конвертировать PCM 16-bit в float32 [-1.0, 1.0]
    
    Args:
        data: Bytes с PCM16 данными
        
    Returns:
        numpy array float32
    """
    samples = np.frombuffer(data, dtype=np.int16)
    return samples.astype(np.float32) / 32768.0


def float32_to_pcm16(samples: np.ndarray) -> bytes:
    """
    Конвертировать float32 [-1.0, 1.0] в PCM 16-bit
    
    Args:
        samples: numpy array float32
        
    Returns:
        Bytes с PCM16 данными
    """
    # Clamp to [-1.0, 1.0]
    samples = np.clip(samples, -1.0, 1.0)
    # Scale to int16 range
    samples = (samples * 32767).astype(np.int16)
    return samples.tobytes()


def extract_channel(data: bytes, channel: int, total_channels: int) -> bytes:
    """
    Извлечь один канал из многоканального аудио
    
    Args:
        data: Bytes с PCM16 данными (interleaved)
        channel: Номер канала (0-based)
        total_channels: Общее количество каналов
        
    Returns:
        Bytes с одним каналом
    """
    samples = np.frombuffer(data, dtype=np.int16)
    # Deinterleave
    channel_data = samples[channel::total_channels]
    return channel_data.tobytes()


def calculate_rms(data: bytes) -> float:
    """
    Рассчитать RMS (Root Mean Square) уровень аудио
    
    Args:
        data: Bytes с PCM16 данными
        
    Returns:
        RMS уровень (0.0 - 1.0)
    """
    samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
    rms = np.sqrt(np.mean(samples ** 2))
    return float(rms)


def calculate_db(rms: float) -> float:
    """
    Конвертировать RMS в децибелы
    
    Args:
        rms: RMS уровень (0.0 - 1.0)
        
    Returns:
        Уровень в dB
    """
    if rms < 1e-10:  # Avoid log(0)
        return -100.0
    return 20.0 * np.log10(rms)


def apply_gain(data: bytes, gain_db: float) -> bytes:
    """
    Применить усиление к аудио данным
    
    Args:
        data: Bytes с PCM16 данными
        gain_db: Усиление в dB
        
    Returns:
        Bytes с усиленными данными
    """
    samples = pcm16_to_float32(data)
    gain_linear = 10.0 ** (gain_db / 20.0)
    samples = samples * gain_linear
    return float32_to_pcm16(samples)


class AudioBuffer:
    """Кольцевой буфер для аудио данных"""
    
    def __init__(self, max_duration: float, sample_rate: int, channels: int = 1):
        """
        Args:
            max_duration: Максимальная длительность в секундах
            sample_rate: Частота дискретизации (Hz)
            channels: Количество каналов
        """
        self.sample_rate = sample_rate
        self.channels = channels
        self.max_samples = int(max_duration * sample_rate * channels)
        self.buffer = np.zeros(self.max_samples, dtype=np.int16)
        self.write_pos = 0
        self.size = 0
    
    def write(self, data: bytes):
        """Записать данные в буфер"""
        samples = np.frombuffer(data, dtype=np.int16)
        n_samples = len(samples)
        
        if n_samples > self.max_samples:
            # Если данных больше чем буфер, берём последние
            samples = samples[-self.max_samples:]
            n_samples = self.max_samples
        
        # Запись с циклическим переполнением
        space_to_end = self.max_samples - self.write_pos
        
        if n_samples <= space_to_end:
            self.buffer[self.write_pos:self.write_pos + n_samples] = samples
            self.write_pos = (self.write_pos + n_samples) % self.max_samples
        else:
            # Разбиваем на две части
            self.buffer[self.write_pos:] = samples[:space_to_end]
            remaining = n_samples - space_to_end
            self.buffer[:remaining] = samples[space_to_end:]
            self.write_pos = remaining
        
        self.size = min(self.size + n_samples, self.max_samples)
    
    def read_last(self, duration: float) -> bytes:
        """
        Прочитать последние N секунд
        
        Args:
            duration: Длительность в секундах
            
        Returns:
            Bytes с аудио данными
        """
        n_samples = int(duration * self.sample_rate * self.channels)
        n_samples = min(n_samples, self.size)
        
        if n_samples == 0:
            return b''
        
        # Читаем последние n_samples
        if n_samples <= self.write_pos:
            samples = self.buffer[self.write_pos - n_samples:self.write_pos]
        else:
            # Читаем в два приёма
            part1_size = n_samples - self.write_pos
            part1 = self.buffer[-part1_size:]
            part2 = self.buffer[:self.write_pos]
            samples = np.concatenate([part1, part2])
        
        return samples.tobytes()
    
    def clear(self):
        """Очистить буфер"""
        self.buffer.fill(0)
        self.write_pos = 0
        self.size = 0
    
    def is_empty(self) -> bool:
        """Проверить пустой ли буфер"""
        return self.size == 0
    
    def get_duration(self) -> float:
        """Получить текущую длительность данных в секундах"""
        return self.size / (self.sample_rate * self.channels)
