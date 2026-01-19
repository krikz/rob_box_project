#!/usr/bin/env python3
"""
AudioNode - захват аудио с ReSpeaker Mic Array v2.0
Публикует: /audio/audio, /audio/vad, /audio/direction, /audio/speech_detected
"""

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
        
        # ReSpeaker interface
        self.respeaker = ReSpeakerInterface()
        
        # PyAudio
        self.pyaudio_instance: Optional[pyaudio.PyAudio] = None
        self.stream: Optional[pyaudio.Stream] = None
        
        # Состояние
        self.is_running = False
        self.audio_thread: Optional[threading.Thread] = None
        
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
                    self.get_logger().info(f'✅ Речь распознана: {duration:.2f}с')
                    # Публикуем speech_audio
                    msg = AudioData()
                    msg.data = list(buf)
                    self.speech_audio_pub.publish(msg)
                else:
                    self.get_logger().warn(f'❌ Речь отклонена: {duration:.2f}с (min={self.speech_min_duration}, max={self.speech_max_duration})')
            
            # DoA - читаем с обработкой ошибок
            try:
                direction = self.respeaker.get_direction()
                if direction is not None:
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
