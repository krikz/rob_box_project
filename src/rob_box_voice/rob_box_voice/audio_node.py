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
from typing import Optional

from .utils.audio_utils import find_respeaker_device, list_audio_devices, calculate_rms, calculate_db
from .utils.respeaker_interface import ReSpeakerInterface


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
        self.vad_pub = self.create_publisher(Bool, '/audio/vad', 10)
        self.direction_pub = self.create_publisher(Int32, '/audio/direction', 10)
        self.speech_detected_pub = self.create_publisher(Bool, '/audio/speech_detected', 10)
        self.state_pub = self.create_publisher(String, '/audio/state', 10)
        
        # ReSpeaker interface
        self.respeaker = ReSpeakerInterface()
        
        # PyAudio
        self.pyaudio_instance: Optional[pyaudio.PyAudio] = None
        self.stream: Optional[pyaudio.Stream] = None
        
        # Состояние
        self.is_running = False
        self.audio_thread: Optional[threading.Thread] = None
        self.last_vad = False
        self.vad_start_time: Optional[float] = None
        
        # Таймер для публикации VAD/DoA
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_sensor_data)
        
        # Инициализация
        self.get_logger().info('AudioNode инициализирован')
        self.initialize_hardware()
    
    def initialize_hardware(self):
        """Инициализация ReSpeaker и PyAudio"""
        # Подключиться к ReSpeaker для VAD/DoA
        if self.respeaker.connect():
            self.get_logger().info('✓ ReSpeaker подключен для VAD/DoA')
            device_info = self.respeaker.get_device_info()
            if device_info:
                self.get_logger().info(f"  Устройство: {device_info['product']}")
            
            # Настроить параметры обработки звука
            self.respeaker.configure_audio_processing(agc=True, noise_suppression=True)
            self.respeaker.set_vad_threshold(self.vad_threshold)
            self.get_logger().info(f'  VAD threshold: {self.vad_threshold} dB')
        else:
            self.get_logger().warn('⚠ ReSpeaker не найден для VAD/DoA')
        
        # Инициализация PyAudio
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
        
        # Публиковать аудио данные
        if in_data and self.is_running:
            msg = AudioData()
            msg.data = list(in_data)
            self.audio_pub.publish(msg)
        
        return (None, pyaudio.paContinue)
    
    def publish_sensor_data(self):
        """Публикация VAD и DoA (вызывается таймером)"""
        if not self.respeaker.is_connected():
            return
        
        # VAD
        vad = self.respeaker.get_vad()
        if vad is not None:
            msg = Bool()
            msg.data = vad
            self.vad_pub.publish(msg)
            
            # Детектировать начало речи
            if vad and not self.last_vad:
                # Переход False → True
                self.vad_start_time = time.time()
                speech_msg = Bool()
                speech_msg.data = True
                self.speech_detected_pub.publish(speech_msg)
                self.get_logger().debug('🎤 Speech detected')
            
            self.last_vad = vad
        
        # DoA
        doa = self.respeaker.get_doa()
        if doa is not None:
            msg = Int32()
            msg.data = doa
            self.direction_pub.publish(msg)
    
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
        
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        
        if self.pyaudio_instance:
            self.pyaudio_instance.terminate()
        
        if self.respeaker.is_connected():
            self.respeaker.disconnect()
        
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
