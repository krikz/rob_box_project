#!/usr/bin/env python3
"""
AudioPlaybackManager - централизованное управление воспроизведением звука

Предотвращает конфликты ALSA между sound_node и tts_node
используя threading.Lock для синхронизации доступа к аудио устройству.
"""

import threading
import time
from typing import Any, Callable, Optional, Tuple
import numpy as np

# ``sounddevice`` опционален для CI (issue #1133: Unit Tests ROS2 Humble
# ставит только requests/pyyaml/pytz, без системных ALSA-зависимостей).
# Если модуль недоступен — sd=None, play_audio делает ранний return.
try:
    import sounddevice as sd  # type: ignore[import-untyped]
except (ImportError, OSError):
    sd = None  # type: ignore[assignment]


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
        self._abort_stream = threading.Event()
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
        # Issue #1133: sounddevice опционален для CI. Без sd play_audio
        # невозможен — ранний return с диагностикой.
        if sd is None:
            print(f"❌ [{node_name}] sounddevice недоступен — невозможно воспроизвести")
            return False
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
                # Ждать окончания воспроизведения С ТАЙМАУТОМ (issue #1133:
                # sd.wait() зависал навечно при PaErrorCode -9985 / dmix
                # race — ALSA-девайс занят. Без таймаута worker в
                # ThreadPoolExecutor залипал на sd.wait(), семафор
                # _synthesis_slots не освобождался, через max_workers+max_queue
                # запросов tts_node полностью переставал обрабатывать callback
                # → watchdog (PR #1133) был единственной защитой).
                # Таймаут = duration + grace, минимум 8.0 с — больше чем
                # duration=4s (типичный TTS-чанк) + ALSA tail.
                _duration = len(audio_data) / float(sample_rate)
                _wait_timeout = max(8.0, _duration + 3.0)
                _deadline = time.monotonic() + _wait_timeout
                _completed = False
                # Issue #1133: цикл с poll, а не голый sd.wait() — последний
                # может зависнуть, если ALSA-stream не сообщает о конце.
                # _current_stream выставляется в None ниже после фактического
                # завершения, поэтому проверяем состояние polling'ом.
                while time.monotonic() < _deadline:
                    try:
                        if not sd.get_stream().active:
                            _completed = True
                            break
                    except Exception:
                        # sd.get_stream() может бросить если stream уже
                        # закрыт — это и есть сигнал завершения.
                        _completed = True
                        break
                    time.sleep(0.05)
                if not _completed:
                    print(
                        f"⚠️  [{node_name}] sd.wait() завис на {_wait_timeout:.1f}с "
                        f"(ALSA Device unavailable / dmix race) — принудительный sd.stop()"
                    )
                    try:
                        sd.stop()
                    except Exception:
                        pass
                    # Не возвращаем False сразу — попробуем дождаться ещё чуть-чуть
                    time.sleep(0.1)
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

    def play_stream(
        self,
        source: Any,
        sample_rate: int,
        device_index: Optional[int] = None,
        *,
        gain: float = 1.0,
        poll: Callable[[], None] = lambda: None,
        should_stop: Callable[[], bool] = lambda: False,
        timeout: float = 5.0,
        node_name: str = "unknown",
    ) -> Tuple[str, int]:
        """Потоковое воспроизведение (#3930): кадры берутся из ``source.read(n)`` по мере прихода.

        ``source.read(n)`` - до ``n`` кадров mono float32 (меньше - пока нет), ``source.done`` - больше не
        будет. Нехватка кадров посреди реплики - тишина, её длина в кадрах возвращается (честный учёт
        провалов); хвост буфера доигрывается. ``poll`` зовётся каждые 20 мс (огибающая рта),
        ``should_stop()`` или :meth:`abort_stream` - обрыв сразу. Тот же вывод, что у ``play_audio``:
        ``device_index``, 16 кГц стерео. Исход: ``done`` / ``stopped`` / ``failed``.
        """
        if sd is None:
            print(f"❌ [{node_name}] sounddevice недоступен — невозможно воспроизвести")
            return "failed", 0
        if not self._playback_lock.acquire(timeout=timeout):
            print(f"⚠️  [{node_name}] Аудио устройство занято, пропускаем воспроизведение")
            return "failed", 0
        self._abort_stream.clear()
        underrun = [0]
        ended = threading.Event()

        def fill(outdata, frames, _time, _status):
            mono = source.read(frames) * gain
            outdata[: mono.size, 0] = mono
            outdata[: mono.size, 1] = mono
            outdata[mono.size:] = 0.0
            if mono.size < frames:
                if source.done:
                    raise sd.CallbackStop
                underrun[0] += frames - mono.size

        try:
            stream = sd.OutputStream(samplerate=sample_rate, channels=2, dtype="float32", device=device_index,
                                     callback=fill, finished_callback=ended.set)
            with stream:
                while not ended.wait(0.02):
                    poll()
                    if should_stop() or self._abort_stream.is_set():
                        stream.abort()
                        return "stopped", underrun[0]
            poll()
            return "done", underrun[0]
        except Exception as e:  # noqa: BLE001 - устройство занято/пропало: честный failed
            print(f"❌ [{node_name}] Ошибка потокового воспроизведения: {e}")
            return "failed", underrun[0]
        finally:
            self._playback_lock.release()

    def abort_stream(self) -> None:
        """Оборвать потоковое воспроизведение (барж-ин / STOP): ``play_stream`` выходит за ≤ 20 мс."""
        self._abort_stream.set()

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
