#!/usr/bin/env python3
"""
Проверка системы для обучения TTS модели.
Проверяет GPU, CUDA, память и создаёт отчёт о возможностях системы.
"""

import sys
import subprocess
import platform

def run_command(cmd):
    """Выполнить команду и вернуть результат"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.stdout.strip()
    except Exception as e:
        return f"Error: {e}"

def check_python():
    """Проверка Python"""
    print("=" * 60)
    print("PYTHON")
    print("=" * 60)
    print(f"Version: {sys.version}")
    print(f"Executable: {sys.executable}")
    print()

def check_os():
    """Проверка ОС"""
    print("=" * 60)
    print("OPERATING SYSTEM")
    print("=" * 60)
    print(f"System: {platform.system()}")
    print(f"Release: {platform.release()}")
    print(f"Version: {platform.version()}")
    print(f"Machine: {platform.machine()}")
    print()

def check_cpu():
    """Проверка CPU"""
    print("=" * 60)
    print("CPU")
    print("=" * 60)
    
    if platform.system() == "Linux":
        cpu_info = run_command("lscpu | grep -E '(Model name|Core|Thread|MHz)'")
        print(cpu_info)
    else:
        print(f"Processor: {platform.processor()}")
    print()

def check_memory():
    """Проверка памяти"""
    print("=" * 60)
    print("MEMORY")
    print("=" * 60)
    
    if platform.system() == "Linux":
        mem_info = run_command("free -h | grep -E '(Mem|Swap)'")
        print(mem_info)
    else:
        print("Memory check not implemented for this OS")
    print()

def check_gpu():
    """Проверка GPU"""
    print("=" * 60)
    print("GPU / CUDA")
    print("=" * 60)
    
    # Проверка nvidia-smi
    nvidia_smi = run_command("nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader")
    if "Error" not in nvidia_smi and nvidia_smi:
        print("✅ NVIDIA GPU detected!")
        print(nvidia_smi)
        print()
        
        # Дополнительная информация
        gpu_util = run_command("nvidia-smi --query-gpu=utilization.gpu,utilization.memory,temperature.gpu --format=csv,noheader")
        print(f"Current status: {gpu_util}")
    else:
        print("❌ NVIDIA GPU not detected or nvidia-smi not available")
        print("Install NVIDIA drivers: sudo apt install nvidia-driver-535")
    print()

def check_pytorch():
    """Проверка PyTorch и CUDA"""
    print("=" * 60)
    print("PYTORCH")
    print("=" * 60)
    
    try:
        import torch
        print(f"✅ PyTorch version: {torch.__version__}")
        print(f"CUDA available: {torch.cuda.is_available()}")
        
        if torch.cuda.is_available():
            print(f"CUDA version: {torch.version.cuda}")
            print(f"cuDNN version: {torch.backends.cudnn.version()}")
            print(f"Number of GPUs: {torch.cuda.device_count()}")
            
            for i in range(torch.cuda.device_count()):
                print(f"\nGPU {i}: {torch.cuda.get_device_name(i)}")
                props = torch.cuda.get_device_properties(i)
                print(f"  Compute capability: {props.major}.{props.minor}")
                print(f"  Total memory: {props.total_memory / 1024**3:.1f} GB")
                
                # Проверка доступной памяти
                torch.cuda.set_device(i)
                mem_allocated = torch.cuda.memory_allocated(i) / 1024**3
                mem_reserved = torch.cuda.memory_reserved(i) / 1024**3
                mem_free = (props.total_memory - torch.cuda.memory_reserved(i)) / 1024**3
                print(f"  Memory allocated: {mem_allocated:.2f} GB")
                print(f"  Memory reserved: {mem_reserved:.2f} GB")
                print(f"  Memory free: {mem_free:.2f} GB")
        else:
            print("❌ CUDA not available")
            print("\nTo install PyTorch with CUDA:")
            print("pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118")
    except ImportError:
        print("❌ PyTorch not installed")
        print("\nTo install PyTorch:")
        print("pip3 install torch torchvision torchaudio")
    print()

def check_tts_packages():
    """Проверка TTS библиотек"""
    print("=" * 60)
    print("TTS LIBRARIES")
    print("=" * 60)
    
    packages = {
        'piper_train': 'Piper Training',
        'TTS': 'Coqui TTS',
        'soundfile': 'SoundFile',
        'librosa': 'Librosa',
        'phonemizer': 'Phonemizer',
        'torchaudio': 'TorchAudio',
    }
    
    for pkg, name in packages.items():
        try:
            __import__(pkg)
            print(f"✅ {name}: installed")
        except ImportError:
            print(f"❌ {name}: not installed")
    print()

def estimate_training_capability(gpu_memory_gb):
    """Оценка возможностей обучения"""
    print("=" * 60)
    print("TRAINING CAPABILITY ESTIMATE")
    print("=" * 60)
    
    if gpu_memory_gb >= 24:
        print("🚀 Excellent! Can train large models")
        print("   - Full TTS training: ✅")
        print("   - Fine-tuning: ✅")
        print("   - Voice cloning: ✅")
        print("   - Batch size: 32-64")
    elif gpu_memory_gb >= 12:
        print("✅ Good! Can train medium models")
        print("   - Full TTS training: ✅ (slower)")
        print("   - Fine-tuning: ✅")
        print("   - Voice cloning: ✅")
        print("   - Batch size: 16-32")
    elif gpu_memory_gb >= 8:
        print("⚠️  Fair! Can fine-tune and clone")
        print("   - Full TTS training: ⚠️ (very slow, small models only)")
        print("   - Fine-tuning: ✅")
        print("   - Voice cloning: ✅")
        print("   - Batch size: 8-16")
    elif gpu_memory_gb >= 6:
        print("⚠️  Limited! Voice cloning recommended")
        print("   - Full TTS training: ❌")
        print("   - Fine-tuning: ⚠️ (difficult)")
        print("   - Voice cloning: ✅")
        print("   - Batch size: 4-8")
    else:
        print("❌ Insufficient GPU memory")
        print("   - Consider cloud GPU (Colab, Vast.ai)")
    print()

def print_recommendations():
    """Рекомендации по установке"""
    print("=" * 60)
    print("RECOMMENDED SETUP")
    print("=" * 60)
    print("""
1. Install NVIDIA drivers (if not installed):
   sudo apt update
   sudo apt install nvidia-driver-535 nvidia-cuda-toolkit

2. Install PyTorch with CUDA 11.8:
   pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

3. Install Piper training (recommended):
   git clone https://github.com/rhasspy/piper.git
   cd piper/src/python
   pip3 install -e .

4. Alternative: Install Coqui TTS:
   pip3 install TTS

5. Install audio libraries:
   sudo apt install espeak-ng libsndfile1
   pip3 install soundfile librosa phonemizer

6. Reboot after driver installation:
   sudo reboot
""")

def main():
    print("\n" + "=" * 60)
    print("ROBBOX TTS TRAINING - SYSTEM CHECK")
    print("=" * 60)
    print()
    
    check_python()
    check_os()
    check_cpu()
    check_memory()
    check_gpu()
    check_pytorch()
    check_tts_packages()
    
    # Оценка возможностей
    try:
        import torch
        if torch.cuda.is_available():
            gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3
            estimate_training_capability(gpu_memory)
    except:
        pass
    
    print_recommendations()
    
    print("=" * 60)
    print("Check completed!")
    print("=" * 60)

if __name__ == "__main__":
    main()
