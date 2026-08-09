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
    """Нода для захвата аудио и публикации VAD/DoA с ReSpeaker."""

    def __init__(self):
        super().__init__('audio_node')

        # Параметры
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        # Issue 1076: какие каналы усреднять для моно (0-based). ReSpeaker
        # в 6-канальном RAW-режиме: Ch1-4 = сырые микрофоны, Ch5-6 =
        # playback-референс (AEC-референс, то, что играет робот). Усреднять
        # референс в STT-сигнал НЕЛЬЗЯ — собственный голос/музыка робота
        # смешиваются в сигнал → Yandex `empty`, vosk-галлюцинации.
        # Пустой список = все каналы (legacy). Для ReSpeaker 6ch укажите
        # [0, 1, 2, 3] (только микрофоны).
        self.declare_parameter('mix_channels', [])
        # Issue 1050: 1024 → 4096. frames_per_buffer=1024 (64ms @16kHz) слишком
        # мал для Python-callback — GIL/публикация/USB VAD приводили к
        # paInputOverflow (PyAudio status 2) и потере сэмплов. 4096 = 256ms.
        self.declare_parameter('chunk_size', 4096)
        self.declare_parameter('vad_threshold', 3.5)
        self.declare_parameter('publish_rate', 10)
        self.declare_parameter('device_index', -1)  # -1 = auto-detect
        self.declare_parameter('device_name', 'ReSpeaker 4 Mic Array')

        # Issue 989 Fix B: grace period после окончания TTS — не начинать
        # накопление речи (и не публиковать VAD=True) пока робот говорит
        # или в течение tts_grace_s секунд после. Иначе эхо собственного
        # голоса/музыки захватывается как «речь» и замыкает петлю.
        self.declare_parameter('tts_grace_s', 2.5)
        # Issue 989 Fix C: при активной музыке поднимаем порог VAD на
        # ReSpeaker (GAMMAVAD_SR) и дополнительно гейтим по уровню RMS —
        # музыка не должна триггерить VAD как речь.
        self.declare_parameter('music_vad_threshold', 6.0)
        self.declare_parameter('music_vad_min_db', -35.0)

        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.mix_channels = list(self.get_parameter('mix_channels').value or [])
        self.chunk_size = self.get_parameter('chunk_size').value
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.device_index = self.get_parameter('device_index').value
        self.device_name = self.get_parameter('device_name').value
        # Issue 989: параметры анти-эхо.
        self.tts_grace_s = float(self.get_parameter('tts_grace_s').value)
        self.music_vad_threshold = float(self.get_parameter('music_vad_threshold').value)
        self.music_vad_min_db = float(self.get_parameter('music_vad_min_db').value)

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

        # Issue 989: подписки на состояние TTS и музыки для анти-эхо гейтов.
        # /voice/tts/state — "synthesizing"/"playing" пока робот говорит,
        # "ready"/"idle" после. Используем для grace period (Fix B).
        self.create_subscription(String, '/voice/tts/state', self._on_tts_state, 10)
        # /voice/music/state — "playing"/"idle" от mcp_server (Fix C): при
        # активной музыке поднимаем порог VAD, чтобы бит не триггерил речь.
        self.create_subscription(String, '/voice/music/state', self._on_music_state, 10)

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

        # Issue 1050: счётчики paInputOverflow для rate-limited диагностики.
        # Не пишем в лог каждый overflow (спам), а раз в окно — с числом
        # случаев и размером чанка, чтобы по live-логам видеть динамику.
        self._overflow_count = 0
        self._overflow_last_logged = 0.0
        self._overflow_log_window_s = 60.0

        # Issue 989: анти-эхо состояние.
        # TTS: True пока робот говорит (synthesizing/playing); после перехода
        # в ready/idle запоминаем момент окончания для grace period.
        self.tts_active: bool = False
        self._tts_ended_at: float = 0.0  # монотонное время окончания TTS
        # Музыка: True когда mcp_server сообщил "playing" на /voice/music/state.
        self.music_active: bool = False
        # Текущий уровень сигнала (dB) для программного гейта VAD (Fix C).
        # Обновляется в audio_callback; стартуем как «тишина».
        self._current_db: float = -100.0

        # Таймер для VAD/DoA (после sleep(5) безопасно!)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.check_vad_and_doa)

        # Инициализация
        self.get_logger().info('AudioNode инициализирован')
        self.initialize_hardware()

    def initialize_hardware(self):
        """Инициализация ReSpeaker и PyAudio."""
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
        """Callback для PyAudio stream."""
        if status:
            # Issue 1050: status=2 (paInputOverflow) — входной буфер
            # переполнен, сэмплы потеряны. Rate-limited лог + диагностика.
            self._log_overflow(frame_count, in_data)

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
                # Issue 1076: усредняем ТОЛЬКО микрофонные каналы.
                # ReSpeaker в 6-канальном RAW-режиме: Ch1-4 = микрофоны,
                # Ch5-6 = playback-референс (AEC-референс — то, что играет
                # робот). Если усреднять ВСЕ каналы, собственный голос/музыка
                # робота смешиваются в STT-сигнал → Yandex `empty`, vosk
                # галлюцинирует. mix_channels=[0,1,2,3] исключает референс.
                if self.mix_channels:
                    # Берём только запрошенные каналы (валидные индексы)
                    channels = [c for c in self.mix_channels if 0 <= c < self.channels]
                    if channels:
                        audio_data = audio_data[:, channels]
                # Усреднение по выбранным каналам для получения моно
                mono_data = audio_data.mean(axis=1).astype(np.int16)
                # Конвертируем обратно в bytes
                audio_bytes = mono_data.tobytes()
            else:
                audio_bytes = in_data

            msg.data = list(audio_bytes)
            self.audio_pub.publish(msg)

            # Issue 989 Fix C: обновляем текущий уровень сигнала для
            # программного гейта VAD при активной музыке.
            try:
                self._current_db = calculate_db(calculate_rms(audio_bytes))
            except Exception:  # noqa: BLE001 — не критично для захвата
                pass

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

    def _log_overflow(self, frame_count: int, in_data) -> None:
        """Rate-limited лог paInputOverflow (issue #1050).

        Статус 2 (paInputOverflow) означает, что входной буфер переполнен
        и сэмплы потеряны — обычно Python-callback не успел за периодом
        (GIL, тяжёлая публикация AudioData, блокирующие USB VAD/DoA чтения
        в check_vad_and_doa). Считаем каждый случай, но пишем в лог не
        чаще раза в ``_overflow_log_window_s`` секунд: строка не спамит,
        а по числу случаев и размеру чанка видно динамику после фикса
        (увеличение frames_per_buffer 1024 → 4096).
        """
        self._overflow_count += 1
        now = time.monotonic()
        if now - self._overflow_last_logged < self._overflow_log_window_s:
            return
        self._overflow_last_logged = now
        expected = frame_count * self.channels * 2
        got = len(in_data) if in_data else 0
        lost = max(0, expected - got)
        self.get_logger().warning(
            f"[issue 1050] PyAudio paInputOverflow (status=2): "
            f"{self._overflow_count} случаев за окно "
            f"{self._overflow_log_window_s:.0f}с, "
            f"chunk {got}/{expected} байт (потеряно ~{lost} байт)"
        )

    # ------------------------------------------------------------------
    # Issue 989: анти-эхо гейты (Fix B — grace после TTS, Fix C — музыка)
    # ------------------------------------------------------------------
    def _on_tts_state(self, msg: String) -> None:
        """Следим за состоянием TTS: пока робот говорит — не трогаем VAD.

        Issue 989 Fix B: при переходе в speaking-состояние сбрасываем
        накопленный буфер (это почти наверняка эхо предыдущей фразы) и
        запоминаем момент окончания TTS для grace period (tts_grace_s).

        Issue 993 barge-in: НЕ гейтим VAD пока робот говорит — wake-word
        gate в dialogue_node отсекает эхо собственного голоса, а
        пользователь может перебить рэп/ответ командой «робот, добавь
        музыку».
        """
        state = (msg.data or "").strip()
        if state in ("synthesizing", "playing"):
            if not self.tts_active:
                self.get_logger().info(
                    "🔇 [issue 993] TTS активен — VAD пропускает речь "
                    "(barge-in через wake-word gate)"
                )
                # Сбрасываем незавершённое накопление — это эхо, не речь.
                self.speech_audio_buffer = b""
                self.is_speeching = False
            self.tts_active = True
        elif state in ("ready", "idle", "stopped"):
            if self.tts_active:
                self._tts_ended_at = time.monotonic()
                self.get_logger().info(
                    f"🔇 [issue 989] TTS закончился — grace {self.tts_grace_s}s"
                )
            self.tts_active = False

    def _on_music_state(self, msg: String) -> None:
        """Состояние музыки от mcp_server (Fix C).

        При активной музыке поднимаем эффективный порог VAD (3.5 → 6-8 dB),
        чтобы бит/мелодия не триггерили «речь». Порог применяется к железу
        best-effort (set_vad_threshold может не поддерживаться на всех
        прошивках ReSpeaker) и к программному гейту по RMS.
        """
        state = (msg.data or "").strip()
        was_active = self.music_active
        self.music_active = state == "playing"
        if self.music_active == was_active:
            return
        if self.music_active:
            self.get_logger().info(
                f"🎵 [issue 989] Музыка активна — VAD threshold {self.vad_threshold} → "
                f"{self.music_vad_threshold} dB (strict mode)"
            )
            self._apply_vad_threshold(self.music_vad_threshold)
        else:
            self.get_logger().info(
                f"🎵 [issue 989] Музыка остановлена — VAD threshold → {self.vad_threshold} dB"
            )
            self._apply_vad_threshold(self.vad_threshold)

    def _apply_vad_threshold(self, threshold: float) -> None:
        """Best-effort применение порога VAD к железу ReSpeaker."""
        if not self.respeaker.is_connected():
            self.get_logger().debug(
                f"[issue 989] ReSpeaker не подключён — порог {threshold} dB только программный"
            )
            return
        try:
            ok = self.respeaker.set_vad_threshold(threshold)
            if ok:
                self.get_logger().info(f"✅ [issue 989] ReSpeaker VAD threshold = {threshold} dB")
            else:
                self.get_logger().warning(
                    f"⚠️ [issue 989] ReSpeaker не принял threshold {threshold} dB — программный гейт остаётся"
                )
        except Exception as exc:  # noqa: BLE001 — USB write может блокировать PyAudio
            self.get_logger().warning(
                f"⚠️ [issue 989] Не удалось set_vad_threshold({threshold}): {exc}"
            )

    def _vad_gated(self, vad: bool) -> bool:
        """Применить анти-эхо гейты к аппаратному VAD.

        Возвращает эффективное значение VAD с учётом:
        - TTS grace period (issue 989 Fix B): в течение tts_grace_s после
          окончания TTS — False (эхо собственного голоса).
          НЕ гейтим VAD пока робот говорит (issue 993 barge-in): эхо
          отсекается wake-word gate в dialogue_node._on_stt_result
          (строка ~411 «has_wake_word»), поэтому во время рэпа/ответа
          пользователь может перебить робот командой «робот, …» —
          barge-in работает.
        - активной музыки (Fix C): требуем программный порог по RMS
        """
        # Issue 993 barge-in: пока TTS активен — НЕ гейтим. Эхо
        # отсекается wake-word gate в dialogue_node.
        if self.tts_active:
            # VAD дальше пройдёт проверки (если vad=True и музыка не
            # слишком громкая). Это разрешает пользователю перебить рэп.
            pass
        elif time.monotonic() - self._tts_ended_at < self.tts_grace_s:
            # После TTS в течение tts_grace_s — это эхо.
            return False
        if not vad:
            return False
        if self.music_active:
            # Музыка активна: аппаратный VAD может ловить бит как «речь».
            # Программный гейт: RMS должен быть выше music_vad_min_db,
            # иначе это музыка/шум, а не голос поверх музыки.
            if self._current_db < self.music_vad_min_db:
                self.get_logger().debug(
                    f"🎵 [issue 989] VAD подавлен музыкой: dB={self._current_db:.1f} < "
                    f"{self.music_vad_min_db} (strict)"
                )
                return False
        return True

    def check_vad_and_doa(self):
        """Проверка VAD и DoA от ReSpeaker."""
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

            # Issue 989: анти-эхо гейты (TTS grace + музыка strict mode).
            vad_eff = self._vad_gated(bool(vad))

            if vad_eff != self.prev_vad:
                # Публикуем только при изменении
                msg = Bool()
                msg.data = vad_eff
                self.vad_pub.publish(msg)
                self.get_logger().info(f'🎙️  VAD: {"речь" if vad_eff else "тишина"}')
                self.prev_vad = vad_eff

            # Обработка состояния речи
            if vad_eff:
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
                    # Issue 1076 (телеметрия): честный «замолчал → фраза готова»
                    # = time_since_stop (>= speech_continuation), а НЕ 0. Раньше
                    # T_accept считался от «Получена фраза» (после 3с паузы) —
                    # итоговый «замолчал → акцепт» занижался на speech_continuation.
                    # Этот лог + «фраза→ПРИНЯТО» в stt_node дают честный полный замер.
                    self.get_logger().info(
                        f"📊 [telemetry] silence_to_phrase_s={time_since_stop:.2f} "
                        f"(speech_continuation={self.speech_continuation})"
                    )
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
        """Публиковать состояние ноды."""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def list_available_devices(self):
        """Вывести список доступных аудио устройств."""
        if self.pyaudio_instance:
            devices = list_audio_devices(self.pyaudio_instance)
            self.get_logger().info('Доступные аудио устройства:')
            for dev in devices:
                self.get_logger().info(
                    f"  [{dev['index']}] {dev['name']} "
                    f"({dev['channels']}ch, {dev['sample_rate']}Hz)"
                )

    def shutdown(self):
        """Корректное завершение работы."""
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
