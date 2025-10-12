"""
ReSpeaker Interface - wrapper для работы с ReSpeaker Mic Array v2.0
"""

import usb.core
import usb.util
from typing import Optional, Tuple
import time


class ReSpeakerInterface:
    """Интерфейс для работы с ReSpeaker Mic Array v2.0 через USB"""
    
    # USB IDs ReSpeaker
    VENDOR_ID = 0x2886
    PRODUCT_ID = 0x0018
    
    # Параметры для tuning
    PARAMETERS = {
        'DOAANGLE': 21,        # Direction of Arrival (0-360°)
        'VOICEACTIVITY': 19,   # Voice Activity Detection (0/1)
        'SPEECHDETECTED': 19,  # Speech detection status
        'AGCONOFF': 1,         # AGC on/off
        'GAMMAVAD_SR': 27,     # VAD threshold
        'STATNOISEONOFF': 5,   # Stationary noise suppression
        'NONSTATNOISEONOFF': 8, # Non-stationary noise suppression
    }
    
    def __init__(self):
        self.dev: Optional[usb.core.Device] = None
        self._connected = False
        
    def connect(self) -> bool:
        """Подключиться к ReSpeaker устройству"""
        try:
            self.dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)
            if self.dev is None:
                return False
            
            self._connected = True
            return True
        except Exception as e:
            print(f"Ошибка подключения к ReSpeaker: {e}")
            return False
    
    def is_connected(self) -> bool:
        """Проверить подключено ли устройство"""
        return self._connected and self.dev is not None
    
    def read_parameter(self, param_name: str) -> Optional[int]:
        """
        Прочитать параметр из ReSpeaker
        
        Args:
            param_name: Имя параметра (например 'DOAANGLE', 'VOICEACTIVITY')
            
        Returns:
            Значение параметра или None при ошибке
        """
        if not self.is_connected():
            return None
            
        try:
            param_id = self.PARAMETERS.get(param_name)
            if param_id is None:
                return None
            
            # Чтение через USB control transfer
            data = self.dev.ctrl_transfer(
                usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
                0,  # request
                param_id,  # value (parameter ID)
                0x1C,  # index
                64,  # length
                timeout=1000
            )
            
            # Парсинг значения из ответа
            if len(data) >= 8:
                # Параметры возвращаются как int или float
                if param_name in ['VOICEACTIVITY', 'SPEECHDETECTED', 'AGCONOFF']:
                    # Булевы параметры
                    return int(data[0])
                elif param_name == 'DOAANGLE':
                    # Угол 0-360°
                    return int.from_bytes(data[0:4], byteorder='little')
                else:
                    # Другие параметры
                    return int.from_bytes(data[0:4], byteorder='little')
            
            return None
            
        except Exception as e:
            print(f"Ошибка чтения параметра {param_name}: {e}")
            return None
    
    def write_parameter(self, param_name: str, value: int) -> bool:
        """
        Записать параметр в ReSpeaker
        
        Args:
            param_name: Имя параметра
            value: Значение
            
        Returns:
            True если успешно
        """
        if not self.is_connected():
            return False
            
        try:
            param_id = self.PARAMETERS.get(param_name)
            if param_id is None:
                return False
            
            # Запись через USB control transfer
            data = value.to_bytes(4, byteorder='little')
            self.dev.ctrl_transfer(
                usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
                0,  # request
                param_id,  # value (parameter ID)
                0x1C,  # index
                data,
                timeout=1000
            )
            
            return True
            
        except Exception as e:
            print(f"Ошибка записи параметра {param_name}: {e}")
            return False
    
    def get_doa(self) -> Optional[int]:
        """Получить Direction of Arrival (0-360°)"""
        return self.read_parameter('DOAANGLE')
    
    def get_vad(self) -> Optional[bool]:
        """Получить Voice Activity Detection status"""
        value = self.read_parameter('VOICEACTIVITY')
        return bool(value) if value is not None else None
    
    def set_vad_threshold(self, threshold: float) -> bool:
        """
        Установить порог VAD
        
        Args:
            threshold: Порог в dB (обычно 3.5)
        """
        # GAMMAVAD_SR expects value in specific format
        int_value = int(threshold * 10)  # Convert to internal format
        return self.write_parameter('GAMMAVAD_SR', int_value)
    
    def configure_audio_processing(self, 
                                   agc: bool = True,
                                   noise_suppression: bool = True) -> bool:
        """
        Настроить параметры обработки звука
        
        Args:
            agc: Включить Automatic Gain Control
            noise_suppression: Включить подавление шума
        """
        success = True
        
        # AGC
        success &= self.write_parameter('AGCONOFF', 1 if agc else 0)
        
        # Noise suppression
        if noise_suppression:
            success &= self.write_parameter('STATNOISEONOFF', 1)
            success &= self.write_parameter('NONSTATNOISEONOFF', 1)
        else:
            success &= self.write_parameter('STATNOISEONOFF', 0)
            success &= self.write_parameter('NONSTATNOISEONOFF', 0)
        
        return success
    
    def get_device_info(self) -> Optional[dict]:
        """Получить информацию об устройстве"""
        if not self.is_connected():
            return None
        
        try:
            return {
                'vendor_id': f"0x{self.dev.idVendor:04x}",
                'product_id': f"0x{self.dev.idProduct:04x}",
                'manufacturer': usb.util.get_string(self.dev, self.dev.iManufacturer) if self.dev.iManufacturer else None,
                'product': usb.util.get_string(self.dev, self.dev.iProduct) if self.dev.iProduct else None,
            }
        except Exception as e:
            print(f"Ошибка получения информации: {e}")
            return None
    
    def disconnect(self):
        """Отключиться от устройства"""
        if self.dev is not None:
            try:
                usb.util.dispose_resources(self.dev)
            except:
                pass
        self._connected = False
        self.dev = None


class ReSpeakerTuning:
    """
    Совместимость с оригинальным tuning.py из respeaker/usb_4_mic_array
    Обёртка для более удобного использования
    """
    
    def __init__(self, device: Optional[usb.core.Device] = None):
        if device is None:
            device = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        
        self.dev = device
        if self.dev is None:
            raise RuntimeError("ReSpeaker device not found")
    
    @property
    def direction(self) -> int:
        """Направление на источник звука (0-360°)"""
        interface = ReSpeakerInterface()
        interface.dev = self.dev
        interface._connected = True
        return interface.get_doa() or 0
    
    @property  
    def is_voice(self) -> int:
        """Voice activity detection (0 или 1)"""
        interface = ReSpeakerInterface()
        interface.dev = self.dev
        interface._connected = True
        vad = interface.get_vad()
        return 1 if vad else 0
