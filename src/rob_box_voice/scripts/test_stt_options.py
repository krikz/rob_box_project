#!/usr/bin/env python3
"""
Тестирование различных STT решений для ROBBOX

Проверяем:
1. Vosk (offline, быстрый, русский)
2. Whisper (offline, точный, медленный)
3. Yandex SpeechKit (online, точный, требует интернет)

Используем ReSpeaker Mic Array v2.0
"""

import pyaudio
import wave
import time
import os
from typing import Optional, Tuple
import threading
import queue


class AudioRecorder:
    """Запись аудио с ReSpeaker"""
    
    def __init__(self, device_name: str = "ReSpeaker 4 Mic Array"):
        self.device_name = device_name
        self.sample_rate = 16000
        self.channels = 1
        self.chunk = 1024
        self.format = pyaudio.paInt16
        
        self.pyaudio = pyaudio.PyAudio()
        self.device_index = self._find_device()
        
        if self.device_index is None:
            print(f"❌ {device_name} не найден!")
            print("\n📋 Доступные устройства:")
            self._list_devices()
            raise RuntimeError("ReSpeaker not found")
        
        print(f"✅ {device_name} найден (index: {self.device_index})")
    
    def _find_device(self) -> Optional[int]:
        """Найти ReSpeaker устройство"""
        for i in range(self.pyaudio.get_device_count()):
            info = self.pyaudio.get_device_info_by_index(i)
            if self.device_name.lower() in info['name'].lower():
                return i
        return None
    
    def _list_devices(self):
        """Список всех аудио устройств"""
        for i in range(self.pyaudio.get_device_count()):
            info = self.pyaudio.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"  [{i}] {info['name']} ({info['maxInputChannels']} channels)")
    
    def record(self, duration: int = 5) -> bytes:
        """Записать аудио"""
        print(f"🎤 Запись {duration} секунд...")
        
        stream = self.pyaudio.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=self.chunk
        )
        
        frames = []
        for i in range(0, int(self.sample_rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)
        
        stream.stop_stream()
        stream.close()
        
        print("✅ Запись завершена")
        return b''.join(frames)
    
    def save_wav(self, audio_data: bytes, filename: str):
        """Сохранить WAV файл"""
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.pyaudio.get_sample_size(self.format))
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_data)
        print(f"💾 Сохранено: {filename}")
    
    def cleanup(self):
        """Закрыть PyAudio"""
        self.pyaudio.terminate()


class VoskSTT:
    """Vosk Speech-to-Text (offline)"""
    
    def __init__(self, model_path: str = "/models/vosk-model-small-ru-0.22"):
        print("\n🔄 Загрузка Vosk STT...")
        try:
            from vosk import Model, KaldiRecognizer
            import json
            
            self.Model = Model
            self.KaldiRecognizer = KaldiRecognizer
            self.json = json
            
            if not os.path.exists(model_path):
                print(f"❌ Модель не найдена: {model_path}")
                print("Скачайте модель:")
                print("  wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip")
                print("  unzip vosk-model-small-ru-0.22.zip -d /models/")
                raise FileNotFoundError(model_path)
            
            self.model = Model(model_path)
            print("✅ Vosk загружен")
            
        except ImportError:
            print("❌ Vosk не установлен!")
            print("Установите: pip install vosk")
            raise
    
    def recognize(self, audio_data: bytes, sample_rate: int = 16000) -> str:
        """Распознать речь"""
        print("🔍 Vosk распознаёт...")
        start = time.time()
        
        rec = self.KaldiRecognizer(self.model, sample_rate)
        rec.AcceptWaveform(audio_data)
        result = self.json.loads(rec.FinalResult())
        
        elapsed = time.time() - start
        text = result.get('text', '')
        
        print(f"⏱️  Время: {elapsed:.2f}s")
        print(f"📝 Результат: {text}")
        
        return text


class WhisperSTT:
    """Whisper Speech-to-Text (offline)"""
    
    def __init__(self, model_size: str = "base"):
        print(f"\n🔄 Загрузка Whisper STT ({model_size})...")
        try:
            import whisper
            self.whisper = whisper
            
            self.model = whisper.load_model(model_size)
            print(f"✅ Whisper {model_size} загружен")
            
        except ImportError:
            print("❌ Whisper не установлен!")
            print("Установите: pip install openai-whisper")
            raise
    
    def recognize(self, audio_file: str) -> str:
        """Распознать речь из WAV файла"""
        print("🔍 Whisper распознаёт...")
        start = time.time()
        
        result = self.model.transcribe(audio_file, language='ru')
        
        elapsed = time.time() - start
        text = result['text'].strip()
        
        print(f"⏱️  Время: {elapsed:.2f}s")
        print(f"📝 Результат: {text}")
        
        return text


class YandexSTT:
    """Yandex SpeechKit STT (online)"""
    
    def __init__(self):
        print("\n🔄 Инициализация Yandex STT...")
        
        self.api_key = os.getenv('YANDEX_API_KEY')
        self.folder_id = os.getenv('YANDEX_FOLDER_ID')
        
        if not self.api_key or not self.folder_id:
            print("❌ Yandex API ключи не найдены!")
            print("Установите переменные:")
            print("  export YANDEX_API_KEY='...'")
            print("  export YANDEX_FOLDER_ID='...'")
            raise RuntimeError("Yandex credentials missing")
        
        print("✅ Yandex STT готов")
    
    def recognize(self, audio_file: str) -> str:
        """Распознать речь через Yandex API"""
        print("🔍 Yandex распознаёт...")
        import requests
        
        start = time.time()
        
        url = "https://stt.api.cloud.yandex.net/speech/v1/stt:recognize"
        
        headers = {
            "Authorization": f"Api-Key {self.api_key}"
        }
        
        params = {
            "folderId": self.folder_id,
            "lang": "ru-RU",
            "format": "lpcm",
            "sampleRateHertz": "16000"
        }
        
        with open(audio_file, 'rb') as f:
            # Пропускаем WAV заголовок (44 байта)
            f.seek(44)
            audio_data = f.read()
        
        response = requests.post(url, headers=headers, params=params, data=audio_data)
        
        elapsed = time.time() - start
        
        if response.status_code == 200:
            result = response.json()
            text = result.get('result', '')
            print(f"⏱️  Время: {elapsed:.2f}s (+ сеть)")
            print(f"📝 Результат: {text}")
            return text
        else:
            print(f"❌ Ошибка: {response.status_code}")
            print(response.text)
            return ""


def test_stt_engine(engine_name: str, recorder: AudioRecorder):
    """Тест одного STT движка"""
    print("\n" + "="*70)
    print(f"   ТЕСТ: {engine_name}")
    print("="*70)
    
    # Запись аудио
    print("\n🎙️  Говорите после сигнала...")
    time.sleep(1)
    print("🔴 ЗАПИСЬ!")
    
    audio_data = recorder.record(duration=5)
    
    # Сохраняем WAV для тестов
    wav_file = f"/tmp/test_stt_{engine_name.lower()}.wav"
    recorder.save_wav(audio_data, wav_file)
    
    # Распознавание
    try:
        if engine_name == "Vosk":
            stt = VoskSTT()
            text = stt.recognize(audio_data)
        
        elif engine_name == "Whisper":
            stt = WhisperSTT(model_size="base")
            text = stt.recognize(wav_file)
        
        elif engine_name == "Yandex":
            stt = YandexSTT()
            text = stt.recognize(wav_file)
        
        else:
            print(f"❌ Неизвестный движок: {engine_name}")
            return
        
        print("\n" + "="*70)
        print(f"✅ {engine_name} ЗАВЕРШЁН")
        print("="*70)
        
    except Exception as e:
        print(f"\n❌ Ошибка {engine_name}: {e}")


def main():
    """Главная функция"""
    print("="*70)
    print("   🎤 ТЕСТИРОВАНИЕ STT ДЛЯ ROBBOX")
    print("="*70)
    print("\nПроверяем:")
    print("  1. Vosk (offline, быстрый)")
    print("  2. Whisper (offline, точный)")
    print("  3. Yandex (online, точный)")
    print()
    
    # Инициализация записи
    try:
        recorder = AudioRecorder()
    except Exception as e:
        print(f"\n❌ Ошибка инициализации: {e}")
        return
    
    # Меню
    while True:
        print("\n" + "="*70)
        print("ВЫБЕРИТЕ ТЕСТ:")
        print("  1 - Vosk (быстрый, offline)")
        print("  2 - Whisper (точный, offline, медленный)")
        print("  3 - Yandex (точный, online)")
        print("  4 - Все подряд")
        print("  0 - Выход")
        print("="*70)
        
        choice = input("\nВыбор: ").strip()
        
        if choice == '1':
            test_stt_engine("Vosk", recorder)
        elif choice == '2':
            test_stt_engine("Whisper", recorder)
        elif choice == '3':
            test_stt_engine("Yandex", recorder)
        elif choice == '4':
            test_stt_engine("Vosk", recorder)
            input("\nНажмите Enter для следующего теста...")
            test_stt_engine("Whisper", recorder)
            input("\nНажмите Enter для следующего теста...")
            test_stt_engine("Yandex", recorder)
        elif choice == '0':
            break
        else:
            print("❌ Неверный выбор!")
    
    recorder.cleanup()
    print("\n👋 До свидания!")


if __name__ == '__main__':
    main()
