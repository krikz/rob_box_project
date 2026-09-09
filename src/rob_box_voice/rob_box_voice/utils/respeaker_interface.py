"""
ReSpeaker Interface - wrapper для работы с ReSpeaker Mic Array v2.0
"""

import logging
import usb.core
import usb.util
from typing import Optional, Tuple
import time

_log = logging.getLogger(__name__)

# Pixel ring для LED индикации
# Optional dependency: `pixel_ring` (ReSpeaker 4-Mic Array LED control) requires
# the `spidev` kernel binding, which is unavailable in many deployment environments
# (containers without /dev/spidev, non-Pi hosts). When missing, we degrade gracefully
# to INFO so the LED ring is silently disabled while the rest of the audio stack
# keeps running.
try:
    from pixel_ring import usb_pixel_ring_v2
    PIXEL_RING_AVAILABLE = True
except (ImportError, IOError) as e:
    _log.info(
        "pixel_ring not available on this platform "
        "(ReSpeaker LED functions disabled, but software animation continues): %s",
        e,
    )
    PIXEL_RING_AVAILABLE = False


class ReSpeakerInterface:
    """Интерфейс для работы с ReSpeaker Mic Array v2.0 через USB."""

    # USB IDs ReSpeaker
    VENDOR_ID = 0x2886
    PRODUCT_ID = 0x0018

    # Параметры для tuning
    # Формат как в jsk-ros-pkg: parameter_name -> (parameter_id, offset, type)
    PARAMETERS = {
        'DOAANGLE': (21, 0, 'int'),        # Direction of Arrival (0-360°)
        'VOICEACTIVITY': (19, 32, 'int'),  # Voice Activity Detection (0/1)
        'SPEECHDETECTED': (19, 22, 'int'), # Speech detection status
        'AGCONOFF': (19, 0, 'int'),        # AGC on/off
        'AECFREEZEONOFF': (18, 7, 'int'),  # AEC Freeze control (0=adaptive, 1=frozen)
        'ECHOONOFF': (18, 6, 'int'),       # Echo cancellation on/off
        'NLATTENONOFF': (18, 1, 'int'),    # Non-linear AEC attenuation on/off
        # Issue #1117 round-2: high-pass filter.
        # Параметр group=18, offset=27 (см. /tmp/issue_1117_assets/
        # usb_4_mic_array_tuning.py PARAMETERS['HPFONOFF']).
        # 0=OFF, 1=70Hz, 2=125Hz, 3=180Hz (firmware default). Дефолт
        # firmware = 3 режет тихое «Р» в «Робот» (-26 dBFS), STT получает
        # «обот» / «меня зовут саша» без wake word → dialogue отбрасывает
        # (no_wake_word). 1 = 70 Hz сохраняет основной тон «Р» (≈60 Hz).
        'HPFONOFF': (18, 27, 'int'),
        # Issue #1117 round-2: переключаемся на 1-канальный processed ASR
        # (Ch0) — Ch1..Ch4 в 6-channel firmware = сырые микрофоны, нам не
        # нужны (DSP уже сделал AEC+beamforming+NS+AGC на XVF-3000).
        # MIX_SELECT: 0=mic1 (raw), 1=mic2 (raw), ... 4=mic4 (raw),
        # 5=AEC-processed beamforming reference (firmware default в
        # 1_channel_firmware), 6-7=playback reference. См. PARAMETERS в
        # upstream usb_4_mic_array_tuning.py. В нашем случае
        # пишем 0 — фактически привязываем аппаратную обработку к Ch1
        # прошивки (которая в 6-канальной раскладке становится Ch0 в
        # PyAudio data). Это парный аппаратный сигнал для mix_channels=[0].
        # Но фактически мы НЕ используем этот параметр (применяется
        # автоматом в 1_channel_firmware); оставлен для полноты датакы.
        # 'MIX_SELECT': (18, ??, 'int'),
    }

    def __init__(self):
        self.dev: Optional[usb.core.Device] = None
        self.pixel_ring = None
        self._connected = False

    def connect(self) -> bool:
        """Подключиться к ReSpeaker устройству."""
        try:
            self.dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)
            if self.dev is None:
                return False

            # НЕ делаем dev.reset()! Это убивает audio interface!
            # В test_jsk_simple.py работало БЕЗ reset, а с ним - Pipe errors

            # Инициализация pixel_ring для LED (если доступен)
            if PIXEL_RING_AVAILABLE:
                try:
                    self.pixel_ring = usb_pixel_ring_v2.PixelRing(self.dev)
                    # "Думающий" режим во время инициализации
                    self.pixel_ring.set_brightness(10)
                    self.pixel_ring.think()
                    print("✓ Pixel ring инициализирован")
                except Exception as e:
                    print(f"Pixel ring ошибка: {e}")

            # Sleep для стабилизации (как в jsk, но БЕЗ reset)
            print("Ожидание стабилизации USB (5 сек)...")
            time.sleep(5)

            # Переключить LED в режим "трассировки" (DoA индикация)
            if self.pixel_ring:
                try:
                    self.pixel_ring.set_brightness(20)
                    self.pixel_ring.trace()
                    print("✓ Pixel ring в режиме трассировки")
                except Exception as e:
                    print(f"Pixel ring trace ошибка: {e}")

            self._connected = True
            return True
        except Exception as e:
            print(f"Ошибка подключения к ReSpeaker: {e}")
            return False

    def is_connected(self) -> bool:
        """Проверить подключено ли устройство."""
        return self._connected and self.dev is not None

    def read_parameter(self, param_name: str) -> Optional[int]:
        """
        Прочитать параметр из ReSpeaker (jsk-ros-pkg compatible)

        Args:
            param_name: Имя параметра (например 'DOAANGLE', 'VOICEACTIVITY')

        Returns:
            Значение параметра или None при ошибке
        """
        if not self.is_connected():
            return None

        try:
            param_data = self.PARAMETERS.get(param_name)
            if param_data is None:
                return None

            param_id = param_data[0]    # parameter ID (19, 21, etc)
            offset = param_data[1]      # offset (0, 22, 32, etc)
            param_type = param_data[2]  # 'int' or 'float'

            # КРИТИЧНО: Формируем cmd как в jsk-ros-pkg!
            cmd = 0x80 | offset  # Для чтения: 0x80 | offset
            if param_type == 'int':
                cmd |= 0x40      # Для int: добавляем 0x40

            # Чтение через USB control transfer (КАК В JSK!)
            import struct
            response = self.dev.ctrl_transfer(
                usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
                0,        # request = 0
                cmd,      # value = (0x80 | offset) [| 0x40 for int]
                param_id, # index = parameter ID
                8,        # length = 8 bytes
                timeout=1000
            )

            # Парсинг ответа как в jsk-ros-pkg
            if len(response) >= 8:
                result = struct.unpack(b'ii', response.tobytes())
                if param_type == 'int':
                    return result[0]
                else:
                    # float: result[0] * (2 ** result[1])
                    return int(result[0] * (2.0 ** result[1]))

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
        """Получить Direction of Arrival (0-360°)."""
        return self.read_parameter('DOAANGLE')

    def get_vad(self) -> Optional[bool]:
        """Получить Voice Activity Detection status."""
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

    def configure_aec(self,
                     aec_on: bool = True,
                     aec_freeze: bool = False,
                     nlp_on: bool = True) -> bool:
        """
        Настроить Acoustic Echo Cancellation (AEC)

        Args:
            aec_on: Включить эхоподавление (ECHOONOFF)
            aec_freeze: Заморозить адаптацию AEC (AECFREEZEONOFF) - обычно False для адаптации
            nlp_on: Включить нелинейное подавление (NLATTENONOFF)

        Returns:
            True если все параметры успешно установлены
        """
        success = True

        # AECFREEZEONOFF: 0 = adaptive (рекомендуется), 1 = frozen
        success &= self.write_parameter('AECFREEZEONOFF', 1 if aec_freeze else 0)

        # ECHOONOFF: 1 = on, 0 = off
        success &= self.write_parameter('ECHOONOFF', 1 if aec_on else 0)

        # NLATTENONOFF: 1 = on (рекомендуется), 0 = off
        success &= self.write_parameter('NLATTENONOFF', 1 if nlp_on else 0)

        return success

    def configure_dsp(self, hpf_on: int = 1) -> bool:
        """
        Настроить параметры DSP XVF-3000 (issue #1117 round-2).

        Сейчас: high-pass filter (HPFONOFF). При необходимости можно
        добавить другие параметры (GAMMAVAD_SR и пр.) — единая точка
        для batch-настройки на старте audio_node.

        Args:
            hpf_on: 0=OFF, 1=70Hz, 2=125Hz, 3=180Hz. Дефолт firmware
                    = 3 (180Hz) режет тихое «Р» в «Робот». 1 — сохраняет.
                    (group=18, offset=27, type=int)

        Returns:
            True если запись прошла успешно.
        """
        return self.write_parameter('HPFONOFF', int(hpf_on))

    def get_device_info(self) -> Optional[dict]:
        """Получить информацию об устройстве."""
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
        """Отключиться от устройства."""
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
        """Направление на источник звука (0-360°)."""
        interface = ReSpeakerInterface()
        interface.dev = self.dev
        interface._connected = True
        return interface.get_doa() or 0

    @property
    def is_voice(self) -> int:
        """Voice activity detection (0 или 1)."""
        interface = ReSpeakerInterface()
        interface.dev = self.dev
        interface._connected = True
        vad = interface.get_vad()
        return 1 if vad else 0
