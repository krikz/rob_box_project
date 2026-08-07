#!/usr/bin/env python3
"""
AudioPlaybackManager - централизованное управление воспроизведением звука

Предотвращает конфликты ALSA между sound_node и tts_node
используя threading.Lock для синхронизации доступа к аудио устройству.
"""

import threading
import time
from typing import Optional
import numpy as np
import sounddevice as sd


class AudioPlaybackManager:
    """
    Singleton менеджер для синхронизации воспроизведения звука.

    Использует threading.Lock для предотвращения одновременного
    доступа к аудио устройству от разных узлов.

    Example:
        >>> manager = AudioPlaybackManager.get_instance()
        >>> manager.play_audio(audio_data, sample_rate, device_index)
    """

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(AudioPlaybackManager, cls).__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self._playback_lock = threading.Lock()
        self._current_stream = None
        self._initialized = True

    @classmethod
    def get_instance(cls):
        """Получить singleton instance менеджера."""
        return cls()

    def play_audio(
        self,
        audio_data: np.ndarray,
        sample_rate: int,
        device_index: Optional[int] = None,
        blocking: bool = False,
        timeout: float = 5.0,
        node_name: str = "unknown"
    ) -> bool:
        """
        Воспроизвести аудио с блокировкой устройства.

        Args:
            audio_data: Numpy array с аудио данными
            sample_rate: Частота дискретизации
            device_index: Индекс аудио устройства (None = default)
            blocking: Если True, блокирует до окончания воспроизведения
            timeout: Максимальное время ожидания блокировки (сек)
            node_name: Имя узла для логирования

        Returns:
            True если воспроизведение успешно, False если timeout или ошибка
        """
        # Попытка захватить блокировку с таймаутом
        acquired = self._playback_lock.acquire(timeout=timeout)

        if not acquired:
            print(f"⚠️  [{node_name}] Аудио устройство занято, пропускаем воспроизведение")
            return False

        try:
            # Остановить предыдущее воспроизведение если есть
            if self._current_stream is not None:
                try:
                    sd.stop()
                except Exception:
                    pass
                self._current_stream = None

            # Воспроизвести аудио
            sd.play(audio_data, sample_rate, device=device_index, blocking=False)
            self._current_stream = True

            if blocking:
                # Ждать окончания воспроизведения
                sd.wait()
                self._current_stream = None
                # ВАЖНО: небольшая задержка чтобы ALSA успел закрыть stream
                # Без этого следующий play() может получить "Device unavailable"
                time.sleep(0.05)
            else:
                # Для non-blocking: подождать немного чтобы stream стартовал
                time.sleep(0.05)

            return True

        except Exception as e:
            print(f"❌ [{node_name}] Ошибка воспроизведения: {e}")
            return False

        finally:
            # Если blocking - блокировка уже освобождена после sd.wait()
            # Если non-blocking - освобождаем сразу (но stream продолжает играть)
            if blocking:
                self._playback_lock.release()
            else:
                # Для non-blocking: создаем таймер который освободит блокировку
                # после примерной длительности аудио
                duration = len(audio_data) / sample_rate
                threading.Timer(duration + 0.1, self._release_lock).start()

    def _release_lock(self):
        """Освободить блокировку воспроизведения (внутренний метод)."""
        try:
            self._current_stream = None
            self._playback_lock.release()
        except Exception:
            # Блокировка уже освобождена
            pass

    def stop_playback(self, node_name: str = "unknown"):
        """
        Остановить текущее воспроизведение.

        Args:
            node_name: Имя узла для логирования
        """
        try:
            if self._current_stream is not None:
                sd.stop()
                self._current_stream = None
                print(f"🔇 [{node_name}] Воспроизведение остановлено")
        except Exception as e:
            print(f"⚠️  [{node_name}] Ошибка остановки воспроизведения: {e}")

    def is_playing(self) -> bool:
        """Проверить, воспроизводится ли сейчас аудио."""
        return self._playback_lock.locked()

    def wait_until_available(self, timeout: float = 10.0, node_name: str = "unknown") -> bool:
        """
        Ждать пока устройство освободится.

        Args:
            timeout: Максимальное время ожидания
            node_name: Имя узла для логирования

        Returns:
            True если устройство освободилось, False при timeout
        """
        start_time = time.time()

        while self.is_playing():
            if time.time() - start_time > timeout:
                print(f"⏱️  [{node_name}] Timeout ожидания освобождения устройства")
                return False
            time.sleep(0.1)

        return True
