#!/bin/bash
# Установка окружения для обучения TTS модели на Dell Precision 5540

set -e

echo "=========================================="
echo "ROBBOX TTS Training Environment Setup"
echo "=========================================="
echo ""

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Проверка ОС
if [ ! -f /etc/os-release ]; then
    echo -e "${RED}Error: Cannot detect OS${NC}"
    exit 1
fi

source /etc/os-release
echo "OS: $PRETTY_NAME"
echo ""

# Проверка NVIDIA GPU
echo "Checking for NVIDIA GPU..."
if ! command -v nvidia-smi &> /dev/null; then
    echo -e "${YELLOW}Warning: nvidia-smi not found${NC}"
    echo "Installing NVIDIA drivers..."
    
    read -p "Install NVIDIA drivers? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt update
        sudo apt install -y nvidia-driver-535 nvidia-cuda-toolkit
        echo -e "${GREEN}NVIDIA drivers installed. Please reboot and run this script again.${NC}"
        exit 0
    fi
else
    echo -e "${GREEN}✓ NVIDIA GPU detected${NC}"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
    echo ""
fi

# Обновление системы
echo "Updating system packages..."
sudo apt update
sudo apt install -y \
    build-essential \
    git \
    wget \
    curl \
    python3-dev \
    python3-pip \
    python3-venv \
    libsndfile1 \
    ffmpeg \
    espeak-ng \
    sox \
    libsox-fmt-all

echo ""

# Создание виртуального окружения
VENV_DIR="$HOME/robbox_tts_env"

if [ -d "$VENV_DIR" ]; then
    echo -e "${YELLOW}Virtual environment already exists: $VENV_DIR${NC}"
    read -p "Remove and recreate? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$VENV_DIR"
    else
        echo "Using existing environment"
    fi
fi

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment: $VENV_DIR"
    python3 -m venv "$VENV_DIR"
fi

# Активация окружения
source "$VENV_DIR/bin/activate"

# Обновление pip
echo "Upgrading pip..."
pip install --upgrade pip setuptools wheel

# Установка PyTorch с CUDA
echo ""
echo "Installing PyTorch with CUDA 11.8..."
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Проверка CUDA
echo ""
echo "Checking PyTorch CUDA..."
python3 -c "import torch; print(f'PyTorch version: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}'); print(f'CUDA version: {torch.version.cuda if torch.cuda.is_available() else \"N/A\"}')"

# Установка аудио библиотек
echo ""
echo "Installing audio libraries..."
pip install \
    soundfile \
    librosa \
    phonemizer \
    pydub \
    scipy \
    numpy \
    pandas \
    tqdm \
    matplotlib \
    tensorboard

# Выбор TTS фреймворка
echo ""
echo "=========================================="
echo "Choose TTS framework:"
echo "=========================================="
echo "1) Piper (recommended for ROBBOX)"
echo "2) Coqui TTS (more features, heavier)"
echo "3) Both"
echo "4) Skip"
echo ""
read -p "Enter choice [1-4]: " CHOICE

case $CHOICE in
    1)
        echo "Installing Piper training..."
        
        # Клонируем Piper если ещё нет
        PIPER_DIR="$HOME/piper"
        if [ ! -d "$PIPER_DIR" ]; then
            git clone https://github.com/rhasspy/piper.git "$PIPER_DIR"
        fi
        
        cd "$PIPER_DIR/src/python"
        pip install -e .
        cd -
        
        echo -e "${GREEN}✓ Piper installed${NC}"
        ;;
    2)
        echo "Installing Coqui TTS..."
        pip install TTS
        echo -e "${GREEN}✓ Coqui TTS installed${NC}"
        ;;
    3)
        echo "Installing both Piper and Coqui TTS..."
        
        # Piper
        PIPER_DIR="$HOME/piper"
        if [ ! -d "$PIPER_DIR" ]; then
            git clone https://github.com/rhasspy/piper.git "$PIPER_DIR"
        fi
        cd "$PIPER_DIR/src/python"
        pip install -e .
        cd -
        
        # Coqui
        pip install TTS
        
        echo -e "${GREEN}✓ Both frameworks installed${NC}"
        ;;
    4)
        echo "Skipping TTS framework installation"
        ;;
    *)
        echo -e "${RED}Invalid choice${NC}"
        ;;
esac

# Создание структуры директорий
echo ""
echo "Creating directory structure..."

mkdir -p ~/robbox_tts_training/{datasets,models,checkpoints,logs,configs}

echo "Directory structure:"
tree -L 2 ~/robbox_tts_training/ 2>/dev/null || ls -R ~/robbox_tts_training/

# Создание скрипта активации
ACTIVATE_SCRIPT="$HOME/robbox_tts_training/activate.sh"
cat > "$ACTIVATE_SCRIPT" << 'EOF'
#!/bin/bash
# Активация окружения для обучения TTS

export ROBBOX_TTS_ROOT="$HOME/robbox_tts_training"
export ROBBOX_VENV="$HOME/robbox_tts_env"

# Активация виртуального окружения
source "$ROBBOX_VENV/bin/activate"

# Переход в рабочую директорию
cd "$ROBBOX_TTS_ROOT"

echo "=========================================="
echo "ROBBOX TTS Training Environment"
echo "=========================================="
echo "Root: $ROBBOX_TTS_ROOT"
echo "Python: $(which python3)"
echo "PyTorch: $(python3 -c 'import torch; print(torch.__version__)')"
echo "CUDA: $(python3 -c 'import torch; print(torch.cuda.is_available())')"
echo ""
echo "Directories:"
echo "  - datasets/     : Training data"
echo "  - models/       : Trained models"
echo "  - checkpoints/  : Training checkpoints"
echo "  - logs/         : Training logs"
echo "  - configs/      : Configuration files"
echo ""
echo "Quick commands:"
echo "  check_system    : Check system capabilities"
echo "  train_piper     : Train Piper model"
echo "  train_coqui     : Train Coqui model"
echo "=========================================="

# Алиасы для быстрых команд
alias check_system="python3 $ROBBOX_TTS_ROOT/../rob_box_project/src/rob_box_voice/training/check_system.py"
alias train_piper="python3 $ROBBOX_TTS_ROOT/../rob_box_project/src/rob_box_voice/training/train_piper.py"
alias train_coqui="python3 $ROBBOX_TTS_ROOT/../rob_box_project/src/rob_box_voice/training/train_coqui.py"

EOF

chmod +x "$ACTIVATE_SCRIPT"

# Итоговая информация
echo ""
echo "=========================================="
echo -e "${GREEN}Installation completed!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Activate training environment:"
echo "   source $ACTIVATE_SCRIPT"
echo ""
echo "2. Check system capabilities:"
echo "   python3 check_system.py"
echo ""
echo "3. Prepare dataset (record Yandex voice):"
echo "   cd /path/to/rob_box_project/src/rob_box_voice/scripts"
echo "   python3 record_yandex_voice.py --input dataset/sentences.txt --output ~/robbox_tts_training/datasets/robbox_voice/"
echo ""
echo "4. Train model:"
echo "   See training/train_piper.py or training/train_coqui.py"
echo ""
echo "Virtual environment location: $VENV_DIR"
echo "Training directory: ~/robbox_tts_training/"
echo "Activation script: $ACTIVATE_SCRIPT"
echo ""
echo -e "${YELLOW}Note: If you installed NVIDIA drivers for the first time, please reboot your system.${NC}"
echo ""
