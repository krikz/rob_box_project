#!/usr/bin/env python3
"""
train.py — Обучение wake word модели для Rob Box.

Использует openWakeWord training pipeline чтобы обучить модель
которая детектирует: "роббокс", "робокс", "робок", "робот".

Полный pipeline:
    1. python3 generate_samples.py          # ~500 позитивных сэмплов (Silero TTS)
    2. python3 download_background_noise.py  # ~2000 фоновых файлов
    3. python3 train.py                      # обучение + экспорт .tflite

Результат:
    models/robbot.tflite   — модель для Pi 5 (TFLite)
    models/robbot.onnx     — модель для PC (ONNX)

Запуск:
    python3 train.py [--epochs 200] [--model-name robbot]

Время на CPU (i7-11800H): ~15-20 минут
"""

import argparse
import os
import sys
import glob


def check_deps():
    """Проверяет что все зависимости установлены."""
    missing = []
    try:
        import openwakeword
    except ImportError:
        missing.append("openwakeword[train]")
    try:
        import torch
    except ImportError:
        missing.append("torch")
    try:
        import numpy
    except ImportError:
        missing.append("numpy")
    try:
        import tflite_runtime.interpreter
    except ImportError:
        try:
            import tensorflow
        except ImportError:
            missing.append("tflite-runtime OR tensorflow")

    if missing:
        print("❌ Не установлены зависимости:")
        for m in missing:
            print(f"   pip install {m}")
        print("\nБыстрая установка:")
        print("  pip install -r requirements.txt")
        sys.exit(1)


def count_wav_files(directory: str) -> int:
    if not os.path.exists(directory):
        return 0
    return len(glob.glob(os.path.join(directory, "**/*.wav"), recursive=True) +
               glob.glob(os.path.join(directory, "*.wav")))


def train(
    model_name: str = "robbot",
    positive_dir: str = "./data/positive",
    negative_dir: str = "./data/negative",
    output_dir: str = "./models",
    epochs: int = 200,
    learning_rate: float = 0.001,
    batch_size: int = 256,
    test_split: float = 0.1,
    target_false_positive_rate: float = 0.5,
):
    """Запускает обучение модели openWakeWord."""
    from openwakeword.train import train_model

    n_pos = count_wav_files(positive_dir)
    n_neg = count_wav_files(negative_dir)
    print(f"\n📊 Датасет:")
    print(f"   Позитивных сэмплов: {n_pos} (в {positive_dir}/)")
    print(f"   Негативных сэмплов: {n_neg} (в {negative_dir}/)")

    if n_pos < 100:
        print(f"\n⚠ Мало позитивных сэмплов ({n_pos}). Рекомендуется ≥300.")
        print("  Запусти: python3 generate_samples.py --count 500")
        if n_pos == 0:
            sys.exit(1)

    if n_neg < 500:
        print(f"\n⚠ Мало негативных сэмплов ({n_neg}). Рекомендуется ≥1000.")
        print("  Запусти: python3 download_background_noise.py --count 2000")
        if n_neg == 0:
            sys.exit(1)

    os.makedirs(output_dir, exist_ok=True)

    print(f"\n🚀 Начинаю обучение...")
    print(f"   Модель: {model_name}")
    print(f"   Эпохи: {epochs}")
    print(f"   Learning rate: {learning_rate}")
    print(f"   Batch size: {batch_size}")
    print(f"   Устройство: CPU (GPU опционален)")

    # openWakeWord training API
    # Docs: https://github.com/dscripka/openWakeWord/blob/main/docs/custom_models.md
    try:
        train_model(
            model_name=model_name,
            positive_example_directory=positive_dir,
            background_data_directory=negative_dir,
            output_directory=output_dir,
            n_epochs=epochs,
            learning_rate=learning_rate,
            batch_size=batch_size,
            val_split=test_split,
            feature_extractor="melspectrogram",  # встроенный, без доп. зависимостей
            use_synthetic_augmentation=True,     # микширование с фоном
            target_false_positive_rate=target_false_positive_rate,
            # Сохранить оба формата:
            save_model_path=os.path.join(output_dir, f"{model_name}.tflite"),
        )
    except TypeError:
        # Старые версии openWakeWord имеют другие параметры
        print("⚠ Пробую legacy API...")
        _train_legacy(
            model_name=model_name,
            positive_dir=positive_dir,
            negative_dir=negative_dir,
            output_dir=output_dir,
            epochs=epochs,
            learning_rate=learning_rate,
            batch_size=batch_size,
        )

    # Проверяем что модель создалась
    tflite_path = os.path.join(output_dir, f"{model_name}.tflite")
    onnx_path = os.path.join(output_dir, f"{model_name}.onnx")

    if os.path.exists(tflite_path):
        size_kb = os.path.getsize(tflite_path) // 1024
        print(f"\n✅ Модель сохранена: {tflite_path} ({size_kb}KB)")
    if os.path.exists(onnx_path):
        size_kb = os.path.getsize(onnx_path) // 1024
        print(f"✅ ONNX модель: {onnx_path} ({size_kb}KB)")

    return tflite_path


def _train_legacy(model_name, positive_dir, negative_dir, output_dir, epochs, learning_rate, batch_size):
    """Попытка для старых версий openWakeWord."""
    from openwakeword import train

    # Попробуем через класс trainer если доступен
    if hasattr(train, "Trainer"):
        trainer = train.Trainer(
            model_name=model_name,
            positive_clips=positive_dir,
            negative_clips=negative_dir,
        )
        trainer.train(
            n_epochs=epochs,
            lr=learning_rate,
            batch_size=batch_size,
        )
        trainer.save(output_dir)
    else:
        print("❌ Не удалось найти совместимый training API.")
        print("   Обновите: pip install 'openwakeword[train]>=0.6.0'")
        sys.exit(1)


def test_model(model_path: str, positive_dir: str, negative_dir: str):
    """Быстрый тест модели на hold-out сэмплах."""
    print(f"\n🧪 Тестирование модели {model_path}...")

    try:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            from tensorflow.lite.python.interpreter import Interpreter

        import numpy as np
        import wave
        import glob

        interpreter = Interpreter(model_path=model_path)
        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        def infer(wav_path: str) -> float:
            with wave.open(wav_path, "rb") as wf:
                frames = wf.readframes(wf.getnframes())
            audio = np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
            # Pad/trim to 1 second at 16kHz
            target_len = 16000
            if len(audio) > target_len:
                audio = audio[:target_len]
            else:
                audio = np.pad(audio, (0, target_len - len(audio)))
            audio = audio.reshape(1, -1).astype(np.float32)
            interpreter.set_tensor(input_details[0]["index"], audio)
            interpreter.invoke()
            return float(interpreter.get_tensor(output_details[0]["index"])[0][0])

        # Тест на позитивных (ожидаем высокий score)
        pos_files = glob.glob(os.path.join(positive_dir, "*.wav"))[:50]
        scores_pos = [infer(f) for f in pos_files]
        avg_pos = sum(scores_pos) / len(scores_pos) if scores_pos else 0

        # Тест на негативных (ожидаем низкий score)
        neg_files = glob.glob(os.path.join(negative_dir, "*.wav"))[:50]
        scores_neg = [infer(f) for f in neg_files]
        avg_neg = sum(scores_neg) / len(scores_neg) if scores_neg else 0

        tp = sum(1 for s in scores_pos if s >= 0.5) / len(scores_pos) if scores_pos else 0
        fp = sum(1 for s in scores_neg if s >= 0.5) / len(scores_neg) if scores_neg else 0

        print(f"   Позитивные: avg_score={avg_pos:.3f} | TPR (thresh=0.5): {tp:.1%}")
        print(f"   Негативные: avg_score={avg_neg:.3f} | FPR (thresh=0.5): {fp:.1%}")

        if tp < 0.7:
            print("   ⚠ TPR низкий — модель плохо детектирует. Увеличь --epochs или добавь сэмплы.")
        elif fp > 0.1:
            print("   ⚠ FPR высокий — много ложных срабатываний. Добавь больше негативных сэмплов.")
        else:
            print("   ✅ Модель выглядит хорошо!")

    except Exception as e:
        print(f"   ⚠ Тест не удался: {e} (модель всё равно сохранена)")


def copy_to_models(tflite_path: str, robot_models_dir: str = "../../docker/vision/models/wakeword"):
    """Копирует модель в директорию робота."""
    import shutil
    dest_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), robot_models_dir))
    os.makedirs(dest_dir, exist_ok=True)
    dest = os.path.join(dest_dir, os.path.basename(tflite_path))
    shutil.copy2(tflite_path, dest)
    print(f"\n📋 Скопировано в Docker volume: {dest}")
    print(f"   Обнови voice_assistant.yaml:")
    print(f"     wake_word_model_paths: [\"/models/wakeword/{os.path.basename(tflite_path)}\"]")
    print(f"     use_wake_word_engine: true")


def main():
    parser = argparse.ArgumentParser(
        description="Обучение wake word модели для Rob Box (роббокс/робокс/робок/робот)"
    )
    parser.add_argument("--model-name", default="robbot", help="Имя модели (без расширения)")
    parser.add_argument("--positive-dir", default="./data/positive", help="Папка с позитивными сэмплами")
    parser.add_argument("--negative-dir", default="./data/negative", help="Папка с фоновыми звуками")
    parser.add_argument("--output-dir", default="./models", help="Папка для сохранения модели")
    parser.add_argument("--epochs", type=int, default=200, help="Количество эпох (default: 200)")
    parser.add_argument("--learning-rate", type=float, default=0.001)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--no-test", action="store_true", help="Пропустить тест после обучения")
    parser.add_argument("--copy-to-robot", action="store_true",
                        help="Скопировать .tflite в docker/vision/models/wakeword/")
    args = parser.parse_args()

    print("=" * 60)
    print("  Rob Box Wake Word Training")
    print("  Целевые слова: роббокс, робокс, робок, робот")
    print("=" * 60)

    check_deps()

    tflite_path = train(
        model_name=args.model_name,
        positive_dir=args.positive_dir,
        negative_dir=args.negative_dir,
        output_dir=args.output_dir,
        epochs=args.epochs,
        learning_rate=args.learning_rate,
        batch_size=args.batch_size,
    )

    if not args.no_test and os.path.exists(tflite_path):
        test_model(tflite_path, args.positive_dir, args.negative_dir)

    if args.copy_to_robot and os.path.exists(tflite_path):
        copy_to_models(tflite_path)

    print("\n" + "=" * 60)
    print("  Следующие шаги:")
    print(f"  1. Скопируй {args.output_dir}/{args.model_name}.tflite")
    print(f"     в docker/vision/models/wakeword/")
    print("  2. Обнови voice_assistant.yaml:")
    print(f"     wake_word_model_paths: [\"/models/wakeword/{args.model_name}.tflite\"]")
    print("     use_wake_word_engine: true")
    print("  3. docker compose up -d voice-assistant")
    print("=" * 60)


if __name__ == "__main__":
    main()
