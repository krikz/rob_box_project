#!/usr/bin/env python3
"""
Тестовый скрипт для подбора голоса и настроек TTS
Скачивает все доступные модели на русском языке и позволяет прослушать их

Поддерживаемые TTS движки:
1. Piper (РЕКОМЕНДУЕТСЯ) - быстрый, качественный, offline
2. Silero - несколько голосов, offline
3. RHVoice - легковесный, но робот (для сравнения)

Использование:
    python3 test_tts_voices.py
    
    Или с конкретным движком:
    python3 test_tts_voices.py --engine piper
    python3 test_tts_voices.py --engine silero
    python3 test_tts_voices.py --engine rhvoice
"""

import os
import sys
import argparse
import subprocess
import json
from pathlib import Path
from typing import List, Dict, Optional
import wave
import tempfile

# Добавляем путь к rob_box_voice для импорта
sys.path.insert(0, str(Path(__file__).parent.parent))

# Цвета для терминала
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_header(text: str):
    """Печать заголовка"""
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*60}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{text:^60}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'='*60}{Colors.ENDC}\n")


def print_success(text: str):
    """Успех"""
    print(f"{Colors.OKGREEN}✓{Colors.ENDC} {text}")


def print_info(text: str):
    """Информация"""
    print(f"{Colors.OKCYAN}ℹ{Colors.ENDC} {text}")


def print_warning(text: str):
    """Предупреждение"""
    print(f"{Colors.WARNING}⚠{Colors.ENDC} {text}")


def print_error(text: str):
    """Ошибка"""
    print(f"{Colors.FAIL}✗{Colors.ENDC} {text}")


# ============================================================================
# Piper TTS
# ============================================================================

PIPER_VOICES = {
    "dmitri": {
        "name": "Dmitri (male)",
        "quality": "medium",
        "url": "https://github.com/rhasspy/piper/releases/download/v1.2.0/ru_RU-dmitri-medium.onnx",
        "config_url": "https://github.com/rhasspy/piper/releases/download/v1.2.0/ru_RU-dmitri-medium.onnx.json",
        "size_mb": 63,
        "description": "Чёткий мужской голос, спокойный"
    },
    "irina": {
        "name": "Irina (female)",
        "quality": "medium",
        "url": "https://github.com/rhasspy/piper/releases/download/v1.2.0/ru_RU-irina-medium.onnx",
        "config_url": "https://github.com/rhasspy/piper/releases/download/v1.2.0/ru_RU-irina-medium.onnx.json",
        "size_mb": 63,
        "description": "Тёплый женский голос, дружелюбный"
    }
}


def download_piper_voice(voice_id: str, models_dir: Path) -> Optional[Path]:
    """Скачать модель Piper"""
    voice_info = PIPER_VOICES[voice_id]
    model_path = models_dir / f"ru_RU-{voice_id}-medium.onnx"
    config_path = models_dir / f"ru_RU-{voice_id}-medium.onnx.json"
    
    if model_path.exists() and config_path.exists():
        print_info(f"Модель {voice_info['name']} уже скачана")
        return model_path
    
    print_info(f"Скачиваю {voice_info['name']} (~{voice_info['size_mb']} MB)...")
    
    try:
        # Скачать модель
        subprocess.run(
            ["wget", "-q", "-O", str(model_path), voice_info["url"]],
            check=True
        )
        
        # Скачать конфиг
        subprocess.run(
            ["wget", "-q", "-O", str(config_path), voice_info["config_url"]],
            check=True
        )
        
        print_success(f"Модель {voice_info['name']} скачана")
        return model_path
        
    except subprocess.CalledProcessError as e:
        print_error(f"Ошибка при скачивании: {e}")
        return None


def test_piper_voice(voice_id: str, models_dir: Path, text: str, speed: float = 1.0):
    """Тестировать голос Piper"""
    try:
        from piper import PiperVoice
    except ImportError:
        print_error("Piper не установлен. Установите: pip install piper-tts")
        return
    
    voice_info = PIPER_VOICES[voice_id]
    print(f"\n{Colors.BOLD}Голос:{Colors.ENDC} {voice_info['name']}")
    print(f"{Colors.BOLD}Описание:{Colors.ENDC} {voice_info['description']}")
    
    # Скачать модель если нужно
    model_path = download_piper_voice(voice_id, models_dir)
    if not model_path:
        return
    
    print_info(f"Синтезирую речь (скорость: {speed})...")
    
    try:
        # Загрузить модель
        voice = PiperVoice.load(str(model_path))
        
        # Синтезировать
        audio = voice.synthesize(text, length_scale=1.0/speed)
        
        # Сохранить во временный файл
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = tmp.name
            
            with wave.open(tmp_path, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(22050)
                wav_file.writeframes(audio.tobytes())
        
        print_success("Синтез завершён")
        
        # Воспроизвести
        print_info("Воспроизвожу...")
        subprocess.run(["aplay", "-q", tmp_path])
        
        # Удалить временный файл
        os.unlink(tmp_path)
        
    except Exception as e:
        print_error(f"Ошибка: {e}")


# ============================================================================
# Silero TTS
# ============================================================================

SILERO_VOICES = {
    "aidar": {
        "name": "Aidar (male)",
        "gender": "male",
        "description": "Мужской, нейтральный"
    },
    "baya": {
        "name": "Baya (female)",
        "gender": "female",
        "description": "Женский, тёплый"
    },
    "kseniya": {
        "name": "Kseniya (female)",
        "gender": "female",
        "description": "Женский, энергичный"
    },
    "xenia": {
        "name": "Xenia (female)",
        "gender": "female",
        "description": "Молодой женский, дружелюбный"
    }
}


def download_silero_model(models_dir: Path) -> bool:
    """Скачать модель Silero (однократно для всех голосов)"""
    try:
        import torch
        
        model_path = models_dir / "silero_model.pt"
        
        if model_path.exists():
            print_info("Модель Silero уже скачана")
            return True
        
        print_info("Скачиваю модель Silero (~300 MB, может занять время)...")
        
        # Загрузить модель из torch hub
        model, _ = torch.hub.load(
            repo_or_dir='snakers4/silero-models',
            model='silero_tts',
            language='ru',
            speaker='v3_1_ru'
        )
        
        # Сохранить модель
        torch.save(model, model_path)
        
        print_success("Модель Silero скачана")
        return True
        
    except Exception as e:
        print_error(f"Ошибка при скачивании Silero: {e}")
        return False


def test_silero_voice(voice_id: str, models_dir: Path, text: str, speed: float = 1.0):
    """Тестировать голос Silero"""
    try:
        import torch
    except ImportError:
        print_error("PyTorch не установлен. Установите: pip install torch torchaudio")
        return
    
    voice_info = SILERO_VOICES[voice_id]
    print(f"\n{Colors.BOLD}Голос:{Colors.ENDC} {voice_info['name']}")
    print(f"{Colors.BOLD}Описание:{Colors.ENDC} {voice_info['description']}")
    
    # Скачать модель если нужно
    if not download_silero_model(models_dir):
        return
    
    print_info(f"Синтезирую речь (скорость: {speed})...")
    
    try:
        # Загрузить модель
        model_path = models_dir / "silero_model.pt"
        device = torch.device('cpu')
        model = torch.load(model_path, map_location=device)
        model.to(device)
        
        # Синтезировать
        audio = model.apply_tts(
            text=text,
            speaker=voice_id,
            sample_rate=48000,
            put_accent=True,
            put_yo=True
        )
        
        # Сохранить во временный файл
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = tmp.name
            
            import torchaudio
            torchaudio.save(tmp_path, audio.unsqueeze(0), 48000)
        
        print_success("Синтез завершён")
        
        # Воспроизвести
        print_info("Воспроизвожу...")
        subprocess.run(["aplay", "-q", tmp_path])
        
        # Удалить временный файл
        os.unlink(tmp_path)
        
    except Exception as e:
        print_error(f"Ошибка: {e}")


# ============================================================================
# RHVoice TTS
# ============================================================================

RHVOICE_VOICES = {
    "aleksandr": {
        "name": "Aleksandr (male)",
        "description": "Мужской, классический"
    },
    "anna": {
        "name": "Anna (female)",
        "description": "Женский, нейтральный"
    },
    "elena": {
        "name": "Elena (female)",
        "description": "Женский, приятный"
    },
    "irina": {
        "name": "Irina (female)",
        "description": "Женский, тёплый"
    }
}


def test_rhvoice_voice(voice_id: str, text: str, speed: float = 1.0):
    """Тестировать голос RHVoice"""
    voice_info = RHVOICE_VOICES[voice_id]
    print(f"\n{Colors.BOLD}Голос:{Colors.ENDC} {voice_info['name']}")
    print(f"{Colors.BOLD}Описание:{Colors.ENDC} {voice_info['description']}")
    
    # Проверить установлен ли RHVoice
    try:
        subprocess.run(["which", "RHVoice-test"], check=True, capture_output=True)
    except subprocess.CalledProcessError:
        print_warning("RHVoice не установлен")
        print_info("Установите: sudo apt-get install rhvoice rhvoice-russian")
        return
    
    print_info(f"Синтезирую речь (скорость: {speed})...")
    
    try:
        # Создать временный файл для текста
        with tempfile.NamedTemporaryFile(mode='w', suffix=".txt", delete=False) as tmp_txt:
            tmp_txt.write(text)
            tmp_txt_path = tmp_txt.name
        
        # Создать временный файл для аудио
        tmp_wav_path = tempfile.mktemp(suffix=".wav")
        
        # Синтезировать
        rate = int(100 * speed)  # RHVoice использует rate 0-200
        subprocess.run([
            "RHVoice-test",
            "-p", f"ru:{voice_id}",
            "-r", str(rate),
            "-o", tmp_wav_path,
            tmp_txt_path
        ], check=True, capture_output=True)
        
        print_success("Синтез завершён")
        
        # Воспроизвести
        print_info("Воспроизвожу...")
        subprocess.run(["aplay", "-q", tmp_wav_path])
        
        # Удалить временные файлы
        os.unlink(tmp_txt_path)
        os.unlink(tmp_wav_path)
        
    except Exception as e:
        print_error(f"Ошибка: {e}")


# ============================================================================
# Главное меню
# ============================================================================

def interactive_menu(engine: str, models_dir: Path):
    """Интерактивное меню для тестирования голосов"""
    
    # Тестовые фразы
    test_phrases = [
        "Привет! Я голосовой ассистент робота Роббокс.",
        "Еду вперёд. Скорость полметра в секунду.",
        "Заряд батареи семьдесят восемь процентов.",
        "Обнаружено препятствие на расстоянии один метр.",
        "Поворачиваю налево на девяносто градусов.",
        "Система навигации готова. Ожидаю команды."
    ]
    
    current_phrase_idx = 0
    current_speed = 1.0
    
    while True:
        print_header(f"Тестирование TTS - {engine.upper()}")
        
        # Текущие настройки
        print(f"{Colors.BOLD}Текущая фраза:{Colors.ENDC}")
        print(f"  {test_phrases[current_phrase_idx]}")
        print(f"\n{Colors.BOLD}Скорость:{Colors.ENDC} {current_speed}x")
        
        print(f"\n{Colors.BOLD}Доступные голоса:{Colors.ENDC}")
        
        if engine == "piper":
            voices = PIPER_VOICES
        elif engine == "silero":
            voices = SILERO_VOICES
        elif engine == "rhvoice":
            voices = RHVOICE_VOICES
        
        for i, (voice_id, info) in enumerate(voices.items(), 1):
            print(f"  {i}. {info['name']} - {info['description']}")
        
        print(f"\n{Colors.BOLD}Опции:{Colors.ENDC}")
        print(f"  p - Сменить фразу")
        print(f"  s - Изменить скорость")
        print(f"  q - Выход")
        
        choice = input(f"\n{Colors.OKCYAN}Выберите голос (1-{len(voices)}) или опцию:{Colors.ENDC} ").strip()
        
        if choice == 'q':
            break
        elif choice == 'p':
            # Сменить фразу
            current_phrase_idx = (current_phrase_idx + 1) % len(test_phrases)
            continue
        elif choice == 's':
            # Изменить скорость
            speed_input = input(f"Введите скорость (0.5-2.0, текущая {current_speed}): ").strip()
            try:
                new_speed = float(speed_input)
                if 0.5 <= new_speed <= 2.0:
                    current_speed = new_speed
                else:
                    print_warning("Скорость должна быть от 0.5 до 2.0")
            except ValueError:
                print_warning("Неверный формат")
            continue
        
        # Проверить номер голоса
        try:
            voice_num = int(choice)
            if 1 <= voice_num <= len(voices):
                voice_id = list(voices.keys())[voice_num - 1]
                
                # Тестировать голос
                if engine == "piper":
                    test_piper_voice(voice_id, models_dir, test_phrases[current_phrase_idx], current_speed)
                elif engine == "silero":
                    test_silero_voice(voice_id, models_dir, test_phrases[current_phrase_idx], current_speed)
                elif engine == "rhvoice":
                    test_rhvoice_voice(voice_id, test_phrases[current_phrase_idx], current_speed)
                
                input(f"\n{Colors.OKCYAN}Нажмите Enter для продолжения...{Colors.ENDC}")
            else:
                print_warning("Неверный номер")
        except ValueError:
            print_warning("Неверный ввод")


def main():
    parser = argparse.ArgumentParser(
        description="Тестирование голосов TTS для rob_box_voice",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--engine",
        choices=["piper", "silero", "rhvoice", "all"],
        default="all",
        help="TTS движок для тестирования (по умолчанию: all)"
    )
    parser.add_argument(
        "--models-dir",
        type=Path,
        default=Path.home() / ".cache" / "rob_box_voice" / "tts_models",
        help="Директория для сохранения моделей"
    )
    
    args = parser.parse_args()
    
    # Создать директорию для моделей
    args.models_dir.mkdir(parents=True, exist_ok=True)
    
    print_header("Rob Box Voice - TTS Voice Testing")
    
    print(f"{Colors.BOLD}Директория моделей:{Colors.ENDC} {args.models_dir}")
    print(f"{Colors.BOLD}Размер кэша:{Colors.ENDC} {sum(f.stat().st_size for f in args.models_dir.rglob('*') if f.is_file()) / 1024 / 1024:.1f} MB")
    
    if args.engine == "all":
        print(f"\n{Colors.BOLD}Выберите TTS движок:{Colors.ENDC}")
        print("  1. Piper (рекомендуется) - быстрый, качественный")
        print("  2. Silero - несколько голосов, медленнее")
        print("  3. RHVoice - легковесный, но звучит как робот")
        
        choice = input(f"\n{Colors.OKCYAN}Ваш выбор (1-3):{Colors.ENDC} ").strip()
        
        if choice == "1":
            engine = "piper"
        elif choice == "2":
            engine = "silero"
        elif choice == "3":
            engine = "rhvoice"
        else:
            print_error("Неверный выбор")
            return
    else:
        engine = args.engine
    
    # Запустить интерактивное меню
    try:
        interactive_menu(engine, args.models_dir)
    except KeyboardInterrupt:
        print(f"\n\n{Colors.WARNING}Прервано пользователем{Colors.ENDC}")
    
    print_header("Testing Complete")
    print_success(f"Модели сохранены в: {args.models_dir}")
    print_info("Вы можете запустить скрипт снова для тестирования других голосов")


if __name__ == "__main__":
    main()
