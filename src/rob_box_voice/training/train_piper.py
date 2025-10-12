#!/usr/bin/env python3
"""
Обучение TTS модели для ROBBOX с использованием Piper.
Piper - быстрый и качественный TTS на базе VITS.
"""

import os
import sys
import json
import argparse
import subprocess
from pathlib import Path
from typing import Optional

def check_requirements():
    """Проверка установленных зависимостей"""
    try:
        import torch
        import piper_train
        print(f"✅ PyTorch {torch.__version__}")
        print(f"✅ Piper training installed")
        print(f"✅ CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            print(f"   GPU: {torch.cuda.get_device_name(0)}")
            print(f"   Memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")
        return True
    except ImportError as e:
        print(f"❌ Missing dependency: {e}")
        print("\nInstall with:")
        print("  cd ~/piper/src/python && pip install -e .")
        return False

def prepare_dataset(dataset_dir: str, output_dir: str):
    """
    Подготовка датасета для Piper.
    Преобразует metadata.csv в формат Piper.
    """
    dataset_path = Path(dataset_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    metadata_file = dataset_path / "metadata.csv"
    if not metadata_file.exists():
        print(f"❌ Metadata file not found: {metadata_file}")
        print("Run record_yandex_voice.py first to create the dataset!")
        return False
    
    print(f"Reading metadata from: {metadata_file}")
    
    # Читаем metadata.csv и создаём Piper формат
    # Формат Piper: audio_file|text|speaker_id
    # Наш формат: filename|text|duration
    
    piper_metadata = []
    with open(metadata_file, 'r', encoding='utf-8') as f:
        for line in f:
            if not line.strip():
                continue
            
            parts = line.strip().split('|')
            if len(parts) < 2:
                continue
            
            filename = parts[0]
            text = parts[1]
            
            # Полный путь к аудио файлу
            audio_path = dataset_path / filename
            if not audio_path.exists():
                print(f"⚠️  Audio file not found: {audio_path}")
                continue
            
            # Формат Piper: относительный путь|текст|speaker_id
            piper_metadata.append(f"{filename}|{text}|robbox")
    
    # Сохраняем в формате Piper
    piper_metadata_file = output_path / "metadata.csv"
    with open(piper_metadata_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(piper_metadata))
    
    print(f"✅ Prepared {len(piper_metadata)} samples")
    print(f"   Piper metadata: {piper_metadata_file}")
    
    # Создаём символьный линк на аудио файлы
    audio_link = output_path / "wavs"
    if not audio_link.exists():
        audio_link.symlink_to(dataset_path, target_is_directory=True)
        print(f"✅ Created audio link: {audio_link} -> {dataset_path}")
    
    return True

def create_config(output_dir: str, dataset_dir: str, language: str = "ru"):
    """Создание конфигурационного файла для Piper"""
    output_path = Path(output_dir)
    
    config = {
        "audio": {
            "sample_rate": 22050,
            "filter_length": 1024,
            "hop_length": 256,
            "win_length": 1024
        },
        "dataset": {
            "path": str(Path(dataset_dir).absolute()),
            "metadata_file": "metadata.csv",
            "language": language,
            "speakers": ["robbox"]
        },
        "model": {
            "type": "vits",
            "hidden_channels": 192,
            "inter_channels": 192,
            "filter_channels": 768,
            "n_heads": 2,
            "n_layers": 6,
            "kernel_size": 3,
            "p_dropout": 0.1,
            "resblock": "1",
            "resblock_kernel_sizes": [3, 7, 11],
            "resblock_dilation_sizes": [[1, 3, 5], [1, 3, 5], [1, 3, 5]],
            "upsample_rates": [8, 8, 2, 2],
            "upsample_initial_channel": 512,
            "upsample_kernel_sizes": [16, 16, 4, 4]
        },
        "training": {
            "batch_size": 16,  # Уменьшите до 8 если мало GPU памяти
            "learning_rate": 0.0002,
            "num_epochs": 1000,
            "seed": 1234,
            "checkpoint_interval": 100,
            "validation_interval": 100,
            "num_workers": 4
        }
    }
    
    config_file = output_path / "config.json"
    with open(config_file, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)
    
    print(f"✅ Config created: {config_file}")
    return config_file

def train(dataset_dir: str, output_dir: str, resume_from: Optional[str] = None):
    """Запуск обучения Piper модели"""
    
    print("\n" + "=" * 60)
    print("PIPER TTS TRAINING - ROBBOX")
    print("=" * 60)
    print()
    
    # Проверка зависимостей
    if not check_requirements():
        return 1
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Подготовка датасета
    print("\n[1/4] Preparing dataset...")
    if not prepare_dataset(dataset_dir, output_path):
        return 1
    
    # Создание конфигурации
    print("\n[2/4] Creating configuration...")
    config_file = create_config(output_path, dataset_dir)
    
    # Проверка GPU
    print("\n[3/4] Checking GPU...")
    import torch
    if not torch.cuda.is_available():
        print("⚠️  GPU not available! Training will be very slow on CPU.")
        response = input("Continue anyway? (y/n): ")
        if response.lower() != 'y':
            return 1
    else:
        gpu_name = torch.cuda.get_device_name(0)
        gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3
        print(f"✅ GPU: {gpu_name} ({gpu_memory:.1f} GB)")
    
    # Запуск обучения
    print("\n[4/4] Starting training...")
    print("=" * 60)
    
    # Команда для обучения Piper
    cmd = [
        "python3", "-m", "piper_train",
        "--dataset-dir", str(output_path),
        "--checkpoint-dir", str(output_path / "checkpoints"),
        "--config", str(config_file),
    ]
    
    if resume_from:
        cmd.extend(["--resume-from", resume_from])
    
    print(f"Command: {' '.join(cmd)}")
    print()
    
    # Запуск
    try:
        subprocess.run(cmd, check=True)
        print("\n" + "=" * 60)
        print("✅ Training completed!")
        print("=" * 60)
        print(f"\nModel saved to: {output_path / 'checkpoints'}")
        print(f"Logs saved to: {output_path / 'logs'}")
        print("\nTo test your model:")
        print(f"  python3 test_trained_model.py --model {output_path / 'checkpoints' / 'model.pt'}")
        return 0
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Training failed: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n\n⚠️  Training interrupted by user")
        print(f"To resume training, use: --resume-from {output_path / 'checkpoints'}")
        return 1

def main():
    parser = argparse.ArgumentParser(
        description="Train Piper TTS model for ROBBOX",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:

  # Basic training (20-30 min dataset for fine-tuning)
  python3 train_piper.py \\
    --dataset ~/robbox_tts_training/datasets/robbox_voice \\
    --output ~/robbox_tts_training/models/robbox_piper

  # Resume training from checkpoint
  python3 train_piper.py \\
    --dataset ~/robbox_tts_training/datasets/robbox_voice \\
    --output ~/robbox_tts_training/models/robbox_piper \\
    --resume-from ~/robbox_tts_training/models/robbox_piper/checkpoints/checkpoint_1000.pt

Dataset requirements:
  - metadata.csv with format: filename|text|duration
  - WAV files (22050Hz recommended)
  - 20-30 minutes for fine-tuning
  - 3-5 hours for full training

GPU requirements:
  - 6+ GB VRAM for batch_size=8
  - 12+ GB VRAM for batch_size=16 (recommended)
  - 24+ GB VRAM for batch_size=32

Training time:
  - Fine-tuning (20-30 min): ~24-48 hours on RTX 3060
  - Full training (3-5 h): ~5-14 days on RTX 3060
        """
    )
    
    parser.add_argument(
        "--dataset",
        required=True,
        help="Path to dataset directory with metadata.csv and WAV files"
    )
    
    parser.add_argument(
        "--output",
        required=True,
        help="Output directory for model, checkpoints, and logs"
    )
    
    parser.add_argument(
        "--resume-from",
        help="Resume training from checkpoint"
    )
    
    args = parser.parse_args()
    
    # Проверка датасета
    dataset_path = Path(args.dataset)
    if not dataset_path.exists():
        print(f"❌ Dataset directory not found: {dataset_path}")
        print("\nCreate dataset first:")
        print("  cd src/rob_box_voice/scripts")
        print("  python3 record_yandex_voice.py --input dataset/sentences.txt --output ~/robbox_tts_training/datasets/robbox_voice/")
        return 1
    
    metadata_file = dataset_path / "metadata.csv"
    if not metadata_file.exists():
        print(f"❌ Metadata file not found: {metadata_file}")
        return 1
    
    # Запуск обучения
    return train(args.dataset, args.output, args.resume_from)

if __name__ == "__main__":
    sys.exit(main())
