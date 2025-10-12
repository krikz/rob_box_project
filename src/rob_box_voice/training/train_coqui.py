#!/usr/bin/env python3
"""
Обучение TTS модели для ROBBOX с использованием Coqui TTS.
Альтернатива Piper с большим количеством настроек.
"""

import os
import sys
import json
import argparse
from pathlib import Path
from typing import Optional

def check_requirements():
    """Проверка установленных зависимостей"""
    try:
        import torch
        from TTS.utils.manage import ModelManager
        print(f"✅ PyTorch {torch.__version__}")
        print(f"✅ Coqui TTS installed")
        print(f"✅ CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            print(f"   GPU: {torch.cuda.get_device_name(0)}")
            print(f"   Memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")
        return True
    except ImportError as e:
        print(f"❌ Missing dependency: {e}")
        print("\nInstall with:")
        print("  pip install TTS")
        return False

def prepare_dataset_coqui(dataset_dir: str, output_dir: str):
    """Подготовка датасета для Coqui TTS"""
    dataset_path = Path(dataset_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    metadata_file = dataset_path / "metadata.csv"
    if not metadata_file.exists():
        print(f"❌ Metadata file not found: {metadata_file}")
        return False
    
    print(f"Reading metadata from: {metadata_file}")
    
    # Coqui TTS формат: audio_file|text
    coqui_metadata = []
    with open(metadata_file, 'r', encoding='utf-8') as f:
        for line in f:
            if not line.strip():
                continue
            
            parts = line.strip().split('|')
            if len(parts) < 2:
                continue
            
            filename = parts[0]
            text = parts[1]
            
            audio_path = dataset_path / filename
            if not audio_path.exists():
                print(f"⚠️  Audio file not found: {audio_path}")
                continue
            
            # Относительный путь для Coqui
            coqui_metadata.append(f"{filename}|{text}")
    
    # Разделение на train/val (90/10)
    train_size = int(len(coqui_metadata) * 0.9)
    train_data = coqui_metadata[:train_size]
    val_data = coqui_metadata[train_size:]
    
    # Сохранение
    train_file = output_path / "metadata_train.txt"
    val_file = output_path / "metadata_val.txt"
    
    with open(train_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(train_data))
    
    with open(val_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(val_data))
    
    print(f"✅ Prepared dataset:")
    print(f"   Train: {len(train_data)} samples ({train_file})")
    print(f"   Val: {len(val_data)} samples ({val_file})")
    
    # Символьная ссылка на аудио
    audio_link = output_path / "wavs"
    if not audio_link.exists():
        audio_link.symlink_to(dataset_path, target_is_directory=True)
        print(f"✅ Audio link: {audio_link} -> {dataset_path}")
    
    return True

def create_config_coqui(output_dir: str, batch_size: int = 16):
    """Создание конфигурации для Coqui TTS"""
    output_path = Path(output_dir)
    
    config = {
        "run_name": "robbox_tts",
        "model": "vits",
        "batch_size": batch_size,
        "eval_batch_size": 8,
        "num_loader_workers": 4,
        "num_eval_loader_workers": 2,
        "run_eval": True,
        "test_delay_epochs": 5,
        "epochs": 1000,
        "print_step": 25,
        "print_eval": True,
        "save_step": 1000,
        "save_best_after": 5000,
        "mixed_precision": True,
        
        "audio": {
            "sample_rate": 22050,
            "fft_size": 1024,
            "win_length": 1024,
            "hop_length": 256,
            "num_mels": 80,
            "mel_fmin": 0,
            "mel_fmax": None
        },
        
        "dataset": [{
            "formatter": "ljspeech",
            "meta_file_train": "metadata_train.txt",
            "meta_file_val": "metadata_val.txt",
            "path": str(output_path),
            "language": "ru"
        }],
        
        "phoneme_language": "ru",
        "use_phonemes": True,
        "phoneme_cache_path": str(output_path / "phoneme_cache"),
        
        "output_path": str(output_path / "output")
    }
    
    config_file = output_path / "config.json"
    with open(config_file, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)
    
    print(f"✅ Config created: {config_file}")
    return config_file

def train_coqui(dataset_dir: str, output_dir: str, batch_size: int = 16, 
                resume_from: Optional[str] = None):
    """Запуск обучения Coqui TTS"""
    
    print("\n" + "=" * 60)
    print("COQUI TTS TRAINING - ROBBOX")
    print("=" * 60)
    print()
    
    if not check_requirements():
        return 1
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Подготовка датасета
    print("\n[1/4] Preparing dataset...")
    if not prepare_dataset_coqui(dataset_dir, output_path):
        return 1
    
    # Создание конфигурации
    print("\n[2/4] Creating configuration...")
    config_file = create_config_coqui(output_path, batch_size)
    
    # Проверка GPU
    print("\n[3/4] Checking GPU...")
    import torch
    if not torch.cuda.is_available():
        print("⚠️  GPU not available! Training will be very slow.")
        response = input("Continue anyway? (y/n): ")
        if response.lower() != 'y':
            return 1
    
    # Запуск обучения
    print("\n[4/4] Starting training...")
    print("=" * 60)
    
    try:
        from TTS.bin.train_tts import main as train_main
        
        # Подготовка аргументов
        sys.argv = [
            "train_tts",
            "--config_path", str(config_file),
            "--coqpit.output_path", str(output_path / "output"),
        ]
        
        if resume_from:
            sys.argv.extend(["--restore_path", resume_from])
        
        print(f"Command: {' '.join(sys.argv)}")
        print()
        
        # Запуск
        train_main()
        
        print("\n" + "=" * 60)
        print("✅ Training completed!")
        print("=" * 60)
        print(f"\nModel saved to: {output_path / 'output'}")
        print("\nTo test your model:")
        print("  tts --text 'Привет, я ROBBOX' --model_path output/best_model.pth --config_path config.json")
        return 0
        
    except Exception as e:
        print(f"\n❌ Training failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

def main():
    parser = argparse.ArgumentParser(
        description="Train Coqui TTS model for ROBBOX",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:

  # Basic training
  python3 train_coqui.py \\
    --dataset ~/robbox_tts_training/datasets/robbox_voice \\
    --output ~/robbox_tts_training/models/robbox_coqui

  # Training with custom batch size
  python3 train_coqui.py \\
    --dataset ~/robbox_tts_training/datasets/robbox_voice \\
    --output ~/robbox_tts_training/models/robbox_coqui \\
    --batch-size 8

  # Resume training
  python3 train_coqui.py \\
    --dataset ~/robbox_tts_training/datasets/robbox_voice \\
    --output ~/robbox_tts_training/models/robbox_coqui \\
    --resume-from ~/robbox_tts_training/models/robbox_coqui/output/best_model.pth

Note: Coqui TTS provides more configuration options than Piper,
      but may require more memory and tuning.
        """
    )
    
    parser.add_argument(
        "--dataset",
        required=True,
        help="Path to dataset directory with metadata.csv"
    )
    
    parser.add_argument(
        "--output",
        required=True,
        help="Output directory for model and logs"
    )
    
    parser.add_argument(
        "--batch-size",
        type=int,
        default=16,
        help="Batch size (reduce to 8 if GPU memory is low)"
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
        return 1
    
    if not (dataset_path / "metadata.csv").exists():
        print(f"❌ Metadata file not found: {dataset_path / 'metadata.csv'}")
        return 1
    
    return train_coqui(args.dataset, args.output, args.batch_size, args.resume_from)

if __name__ == "__main__":
    sys.exit(main())
