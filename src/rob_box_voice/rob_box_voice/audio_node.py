#!/usr/bin/env python3
"""
AudioNode - захват аудио с ReSpeaker Mic Array v2.0
Публикует: /audio/audio, /audio/vad, /audio/direction, /audio/speech_detected
"""

import queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Int32, String
from audio_common_msgs.msg import AudioData
import pyaudio
import threading
import time
import os
import sys
from contextlib import contextmanager
from typing import Optional

from .utils.audio_utils import find_respeaker_device, list_audio_devices, calculate_rms, calculate_db
from .utils.respeaker_interface import ReSpeakerInterface
from .utils.wakeword_detector import WakeWordDetector, WakeWordResult


@contextmanager
def ignore_stderr(enable=True):
    """
    Подавить ALSA ошибки от PyAudio (как в jsk-ros-pkg)
    https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/respeaker_ros/src/respeaker_ros/__init__.py
    """
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield


class AudioNode(Node):
    """Нода для захвата аудио и публикации VAD/DoA с ReSpeaker"""
    
    def __init__(self):
        super().__init__('audio_node')
        
        # Параметры
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('vad_threshold', 3.5)
        self.declare_parameter('publish_rate', 10)
        self.declare_parameter('device_index', -1)  # -1 = auto-detect
        self.declare_parameter('device_name', 'ReSpeaker 4 Mic Array')
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.device_index = self.get_parameter('device_index').value
        self.device_name = self.get_parameter('device_name').value
        
        # QoS для аудио потока
        audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.audio_pub = self.create_publisher(AudioData, '/audio/audio', audio_qos)
        self.speech_audio_pub = self.create_publisher(AudioData, '/audio/speech_audio', audio_qos)
        self.vad_pub = self.create_publisher(Bool, '/audio/vad', 10)
        self.direction_pub = self.create_publisher(Int32, '/audio/direction', 10)
        self.state_pub = self.create_publisher(String, '/audio/state', 10)
        self.tts_control_pub = self.create_publisher(String, '/voice/tts/control', 10)  # Для прерывания TTS
        self.wake_word_pub = self.create_publisher(String, '/audio/wake_word', 10)  # Wake word events

        # Subscriber: dialogue state (to know when in active dialogue for DOA window)
        self.create_subscription(String, '/voice/dialogue/state', self._on_dialogue_state, 10)

        # Subscriber: TTS state (to mute wake word while robot is speaking)
        self.create_subscription(String, '/voice/tts/state', self._on_tts_state, 10)
        
        # ReSpeaker interface
        self.respeaker = ReSpeakerInterface()
        
        # PyAudio
        self.pyaudio_instance: Optional[pyaudio.PyAudio] = None
        self.stream: Optional[pyaudio.Stream] = None
        
        # Состояние
        self.is_running = False
        self.audio_thread: Optional[threading.Thread] = None
        
        # Wake word engine параметры
        self.declare_parameter('use_wake_word_engine', False)  # False = fallback (text-based)
        self.declare_parameter('wake_word_model_paths', [''])  # Пути к .tflite/.onnx моделям
        self.declare_parameter('wake_word_threshold', 0.5)     # Порог детекции (0.0-1.0)
        self.declare_parameter('wake_word_timeout_sec', 8.0)   # Окно после wake word (сек)
        self.declare_parameter('wake_word_min_interval_sec', 4.0)  # Минимальный интервал между событиями WW

        # DOA lock параметры
        self.declare_parameter('doa_lock_enabled', True)          # Включить DOA фильтрацию
        self.declare_parameter('doa_tolerance_degrees', 45)       # Допуск угла ±deg
        self.declare_parameter('dialogue_window_seconds', 30.0)   # Окно диалога без wake word

        self._use_wake_word_engine: bool = self.get_parameter('use_wake_word_engine').value
        self._wake_word_threshold: float = self.get_parameter('wake_word_threshold').value
        self._wake_word_timeout_sec: float = self.get_parameter('wake_word_timeout_sec').value
        self._wake_word_min_interval_sec: float = self.get_parameter('wake_word_min_interval_sec').value
        self._doa_lock_enabled: bool = self.get_parameter('doa_lock_enabled').value
        self._doa_tolerance_degrees: int = self.get_parameter('doa_tolerance_degrees').value
        self._dialogue_window_seconds: float = self.get_parameter('dialogue_window_seconds').value

        # Wake word state
        self._last_wake_word_time: float = 0.0   # время последнего срабатывания WW
        self._last_wake_word_event_time: float = 0.0  # время последней ПУБЛИКАЦИИ события WW (cooldown)
        self._tts_active: bool = False            # True пока TTS воспроизводит речь
        self._locked_doa_angle: Optional[int] = None  # угол при последней активации
        self._last_dialogue_response_time: float = 0.0  # время последнего ответа бота
        self._in_active_dialogue: bool = False    # True пока state = DIALOGUE/LISTENING
        self._current_doa: int = 0               # последний прочитанный DOA

        # Wake word engine (инициализируется после PyAudio)
        self._wakeword_detector: Optional[WakeWordDetector] = None
        self._ww_chunk_buffer: bytes = b''  # накопитель для 512-sample чанков
        # Background thread для ONNX инференса (чтобы не блокировать PyAudio callback)
        self._ww_queue: queue.Queue = queue.Queue(maxsize=100)
        self._ww_thread: Optional[threading.Thread] = None

        # Параметры VAD
        self.declare_parameter('speech_continuation', 1.5)  # Время после окончания речи (секунды)
        self.declare_parameter('speech_prefetch', 0.5)      # Буфер перед началом речи
        self.declare_parameter('speech_min_duration', 0.3)  # Минимальная длительность
        self.declare_parameter('speech_max_duration', 15.0) # Максимальная длительность (секунды)
        
        self.speech_continuation = self.get_parameter('speech_continuation').value
        self.speech_prefetch = self.get_parameter('speech_prefetch').value
        self.speech_min_duration = self.get_parameter('speech_min_duration').value
        self.speech_max_duration = self.get_parameter('speech_max_duration').value
        
        # Буферы для VAD
        self.is_speeching = False
        self.speech_stopped_time = self.get_clock().now()
        self.speech_audio_buffer = b""
        self.speech_prefetch_buffer = b""
        self.speech_prefetch_bytes = int(
            self.speech_prefetch * self.sample_rate * 2)  # 16-bit = 2 bytes
        self.prev_vad = False
        
        # Таймер для VAD/DoA (после sleep(5) безопасно!)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.check_vad_and_doa)

        # Инициализация
        self.get_logger().info('AudioNode инициализирован')
        self.initialize_hardware()
    
    def initialize_hardware(self):
        """Инициализация ReSpeaker и PyAudio"""
        # Подключиться к ReSpeaker для VAD/DoA через USB
        if self.respeaker.connect():
            self.get_logger().info('✓ ReSpeaker USB подключен для VAD/DoA')
            device_info = self.respeaker.get_device_info()
            if device_info:
                self.get_logger().info(f"  Устройство: {device_info['product']}")
            
            # НЕ настраиваем параметры - только ЧИТАЕМ VAD!
            # Любая USB запись может заблокировать PyAudio!
            # self.respeaker.configure_audio_processing(agc=True, noise_suppression=True)
            # self.respeaker.set_vad_threshold(self.vad_threshold)
            self.get_logger().info(f'  VAD threshold: {self.vad_threshold} dB (по умолчанию)')
        else:
            self.get_logger().warn('⚠ ReSpeaker USB не найден для VAD/DoA')

        # Инициализировать wake word engine (если включён)
        if self._use_wake_word_engine:
            raw_paths = list(self.get_parameter('wake_word_model_paths').value)
            model_paths = [p for p in raw_paths if p.strip()]
            self._wakeword_detector = WakeWordDetector(
                model_paths=model_paths or None,
                threshold=self._wake_word_threshold,
            )
            if self._wakeword_detector.available:
                self.get_logger().info(
                    f'✓ WakeWordDetector запущен: '  
                    f'threshold={self._wake_word_threshold}, '
                    f'models={model_paths or "pre-packaged"}'
                )
            else:
                self.get_logger().warn(
                    '⚠ WakeWordDetector недоступен (openWakeWord не установлен) — '
                    'используем text-based fallback'
                )
        else:
            self.get_logger().info('WakeWordDetector отключён (use_wake_word_engine=false) — text fallback')
        
        # Инициализация PyAudio (теперь ReSpeaker должен быть виден как аудио устройство)
        # Глушим ALSA ошибки как в jsk-ros-pkg
        with ignore_stderr(enable=True):
            self.pyaudio_instance = pyaudio.PyAudio()
        
        # Найти устройство
        if self.device_index < 0:
            self.device_index = find_respeaker_device(self.pyaudio_instance)
            if self.device_index is None:
                self.get_logger().error('❌ ReSpeaker аудио устройство не найдено!')
                self.list_available_devices()
                self.publish_state('error_no_device')
                return
        
        self.get_logger().info(f'✓ Используется аудио устройство index={self.device_index}')
        
        # Открыть аудио поток
        try:
            self.stream = self.pyaudio_instance.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=self.chunk_size,
                stream_callback=self.audio_callback
            )
            
            self.get_logger().info(f'✓ Аудио поток открыт: {self.sample_rate}Hz, {self.channels}ch')
            self.publish_state('ready')
            
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка открытия аудио потока: {e}')
            self.publish_state('error_stream')
            return
        
        # Запустить поток
        self.is_running = True
        self.stream.start_stream()
        self.get_logger().info('▶ Захват аудио запущен')
        self.publish_state('running')

        # Запустить background thread для ONNX инференса (off main/callback thread)
        if self._use_wake_word_engine and self._wakeword_detector and self._wakeword_detector.available:
            self._ww_thread = threading.Thread(
                target=self._ww_worker, name='ww-inference', daemon=True
            )
            self._ww_thread.start()
            self.get_logger().info('▶ Wake word inference thread запущен (background)')
    
    def _ww_worker(self) -> None:
        """Фоновый тред для ONNX инференса wake word (не засоряет PyAudio callback)."""
        while self.is_running:
            try:
                chunk = self._ww_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if chunk is None:  # sentinel → выход
                break
            try:
                result = self._wakeword_detector.process_chunk(chunk)
                if result:
                    self._on_wake_word_detected(result)
            except Exception as e:
                self.get_logger().warn(f'WW inference error: {e}')

    def audio_callback(self, in_data, frame_count, time_info, status):
        """Callback для PyAudio stream"""
        if status:
            self.get_logger().warn(f'PyAudio status: {status}')
        
        # Публиковать RAW аудио данные
        if in_data and self.is_running:
            msg = AudioData()
            
            # Если многоканальное аудио - конвертируем в моно
            if self.channels > 1:
                import numpy as np
                # Конвертируем bytes в numpy массив int16
                audio_data = np.frombuffer(in_data, dtype=np.int16)
                # Разделяем на каналы: [ch1, ch2, ..., ch6, ch1, ch2, ...]
                audio_data = audio_data.reshape(-1, self.channels)
                # Усреднение по каналам для получения моно
                mono_data = audio_data.mean(axis=1).astype(np.int16)
                # Конвертируем обратно в bytes
                audio_bytes = mono_data.tobytes()
            else:
                audio_bytes = in_data
            
            msg.data = list(audio_bytes)
            self.audio_pub.publish(msg)

            # --- Wake word engine: enqueue chunks (inference runs in background thread) ---
            if self._use_wake_word_engine and self._wakeword_detector and self._wakeword_detector.available:
                self._ww_chunk_buffer += audio_bytes
                chunk_bytes = WakeWordDetector.CHUNK_SAMPLES * 2
                while len(self._ww_chunk_buffer) >= chunk_bytes:
                    chunk = self._ww_chunk_buffer[:chunk_bytes]
                    self._ww_chunk_buffer = self._ww_chunk_buffer[chunk_bytes:]
                    try:
                        self._ww_queue.put_nowait(chunk)
                    except queue.Full:
                        pass  # Дропаем старый чанк если очередь переполнена

            # Буферизация для speech_audio
            if self.is_speeching:
                # Во время речи - добавляем в speech buffer
                if len(self.speech_audio_buffer) == 0:
                    # Первый чанк - добавляем prefetch
                    self.speech_audio_buffer = self.speech_prefetch_buffer
                self.speech_audio_buffer += audio_bytes
            else:
                # Вне речи - обновляем prefetch buffer
                self.speech_prefetch_buffer += audio_bytes
                self.speech_prefetch_buffer = self.speech_prefetch_buffer[-self.speech_prefetch_bytes:]
        
        return (None, pyaudio.paContinue)
    
    def check_vad_and_doa(self):
        """Проверка VAD и DoA от ReSpeaker"""
        if not self.respeaker.is_connected():
            return
        
        try:
            # Получить текущее время
            now = self.get_clock().now()
            
            # VAD - читаем с обработкой ошибок
            try:
                vad = self.respeaker.get_vad()
            except Exception as e:
                # Pipe error или другая USB ошибка - пропускаем этот цикл
                # (такое может быть если PyAudio активно использует устройство)
                return
            
            if vad is None:
                return  # Ошибка чтения, пропускаем
            
            if vad != self.prev_vad:
                # Публикуем только при изменении
                msg = Bool()
                msg.data = vad
                self.vad_pub.publish(msg)
                self.get_logger().info(f'🎙️  VAD: {"речь" if vad else "тишина"}')
                self.prev_vad = vad
            
            # Обработка состояния речи
            if vad:
                # Речь обнаружена - обновляем время остановки
                self.speech_stopped_time = now
            
            # Проверяем время с момента окончания речи
            time_since_stop = (now - self.speech_stopped_time).nanoseconds / 1e9
            
            if time_since_stop < self.speech_continuation:
                # Речь продолжается (или недавно закончилась)
                if not self.is_speeching:
                    self.get_logger().info('🗣️  Начало речи')
                self.is_speeching = True
            elif self.is_speeching:
                # Речь закончилась - публикуем накопленный буфер
                buf = self.speech_audio_buffer
                self.speech_audio_buffer = b""
                self.is_speeching = False
                
                # Вычисляем длительность
                duration = len(buf) / (self.sample_rate * 2)  # 16-bit = 2 bytes
                
                if self.speech_min_duration <= duration <= self.speech_max_duration:
                    # --- Wake word + DOA gate ---
                    if self._should_accept_phrase(self._current_doa):
                        self.get_logger().info(f'✅ Речь распознана: {duration:.2f}с (doa={self._current_doa}°)')
                        msg = AudioData()
                        msg.data = list(buf)
                        self.speech_audio_pub.publish(msg)
                    else:
                        self.get_logger().info(
                            f'🚫 Речь отфильтрована (WW/DOA gate): {duration:.2f}с, '
                            f'doa={self._current_doa}°, '
                            f'locked={self._locked_doa_angle}°, '
                            f'ww_age={time.time()-self._last_wake_word_time:.1f}s'
                        )
                else:
                    self.get_logger().warn(f'❌ Речь отклонена: {duration:.2f}с (min={self.speech_min_duration}, max={self.speech_max_duration})')
            
            # DoA - читаем с обработкой ошибок
            try:
                direction = self.respeaker.get_direction()
                if direction is not None:
                    self._current_doa = direction  # Обновляем текущий DOA
                    msg = Int32()
                    msg.data = direction
                    self.direction_pub.publish(msg)
            except Exception:
                # Pipe error - пропускаем
                pass
        
        except Exception as e:
            # Общая ошибка - логируем только если это не Pipe error
            if 'Pipe error' not in str(e):
                self.get_logger().warn(f'VAD/DoA ошибка: {e}')
    
    # ------------------------------------------------------------------
    # Wake word + DOA gate
    # ------------------------------------------------------------------

    def _on_wake_word_detected(self, result: WakeWordResult) -> None:
        """Called (from audio callback thread) when wake word engine fires."""
        now = time.time()

        # Подавление во время TTS: игнорируем пока робот говорит
        if self._tts_active:
            return

        # Cooldown: не публикуем событие чаще чем раз в N секунд
        if now - self._last_wake_word_event_time < self._wake_word_min_interval_sec:
            # НЕ обновляем _last_wake_word_time — иначе фоновый звук (YouTube)
            # каждые 4с обновляет окно и VAD-фразы вечно принимаются
            return

        self._last_wake_word_time = result.timestamp
        self._last_wake_word_event_time = now
        self._locked_doa_angle = self._current_doa
        msg = String()
        msg.data = result.model_name
        self.wake_word_pub.publish(msg)
        self.get_logger().info(
            f'🔔 Wake word: model={result.model_name} score={result.score:.2f} '
            f'doa={self._locked_doa_angle}°'
        )

    def _on_tts_state(self, msg: String) -> None:
        """Track TTS state to mute wake word detection while robot speaks."""
        state = msg.data.lower()
        self._tts_active = state in ('playing', 'synthesizing')

    def _on_dialogue_state(self, msg: String) -> None:
        """Track dialogue state for DOA window logic."""
        state = msg.data.upper()
        was_active = self._in_active_dialogue
        self._in_active_dialogue = state in ('DIALOGUE', 'LISTENING')
        # When bot finishes a response (transitions out of DIALOGUE), record time
        # so the DOA dialogue window starts counting from bot's last reply.
        if was_active and not self._in_active_dialogue:
            self._last_dialogue_response_time = time.time()

    def _should_accept_phrase(self, phrase_doa: int) -> bool:
        """
        Decide whether to forward a captured phrase to STT.

        Logic (short-circuit evaluation):
        1. If wake word engine is disabled → always accept (legacy behaviour).
        2. If wake word engine unavailable (not installed) → always accept.
        3. If wake word fired within wake_word_timeout_sec → accept and lock DOA.
        4. If in active dialogue (DIALOGUE/LISTENING state) AND DOA matches → accept.
        5. If dialogue window open (last bot response < dialogue_window_seconds ago)
           AND DOA matches locked angle → accept continuation.
        6. Otherwise → reject.

        Args:
            phrase_doa: DOA angle of the speech phrase (0-359°).

        Returns:
            True if phrase should be forwarded to STT.
        """
        # 1. Engine disabled — passthrough (text-based wake word still active in dialogue_node)
        if not self._use_wake_word_engine:
            return True

        # 2. Engine installed but unavailable — graceful degradation
        if self._wakeword_detector is None or not self._wakeword_detector.available:
            return True

        now = time.time()

        # 3. Wake word recently fired
        if now - self._last_wake_word_time <= self._wake_word_timeout_sec:
            # Lock DOA on first accepted phrase after wake word
            if self._locked_doa_angle is None:
                self._locked_doa_angle = phrase_doa
            return True

        # DOA check helper
        def _doa_matches(current: int, locked: int) -> bool:
            if not self._doa_lock_enabled:
                return True
            diff = abs(current - locked)
            diff = min(diff, 360 - diff)  # handle 359° vs 1° wraparound
            return diff <= self._doa_tolerance_degrees

        if self._locked_doa_angle is None:
            return False  # No lock established yet

        # 4. Active dialogue — accept from same direction
        if self._in_active_dialogue:
            return _doa_matches(phrase_doa, self._locked_doa_angle)

        # 5. Dialogue window after bot response
        if now - self._last_dialogue_response_time <= self._dialogue_window_seconds:
            return _doa_matches(phrase_doa, self._locked_doa_angle)

        # 6. No valid gate — reject
        return False

    # ------------------------------------------------------------------

    def publish_state(self, state: str):
        """Публиковать состояние ноды"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
    
    def list_available_devices(self):
        """Вывести список доступных аудио устройств"""
        if self.pyaudio_instance:
            devices = list_audio_devices(self.pyaudio_instance)
            self.get_logger().info('Доступные аудио устройства:')
            for dev in devices:
                self.get_logger().info(
                    f"  [{dev['index']}] {dev['name']} "
                    f"({dev['channels']}ch, {dev['sample_rate']}Hz)"
                )
    
    def shutdown(self):
        """Корректное завершение работы"""
        self.get_logger().info('Остановка AudioNode...')
        self.is_running = False

        # Остановить WW inference thread
        try:
            if self._ww_thread and self._ww_thread.is_alive():
                self._ww_queue.put_nowait(None)  # sentinel
                self._ww_thread.join(timeout=2.0)
        except Exception:
            pass
        
        try:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
        except:
            pass
        
        try:
            if self.pyaudio_instance:
                self.pyaudio_instance.terminate()
        except:
            pass
        
        try:
            if self.respeaker and self.respeaker.is_connected():
                self.respeaker.disconnect()
        except:
            pass
        
        self.publish_state('stopped')
        self.get_logger().info('✓ AudioNode остановлен')


def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
