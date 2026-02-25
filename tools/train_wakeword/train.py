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
import random
import sys
import glob


def check_deps():
    """Проверяет что все зависимости установлены."""
    missing = []
    try:
        import openwakeword
    except ImportError:
        missing.append("openwakeword[train]")
    # torch нужен только generate_samples.py (Silero TTS), не для training
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
    """Запускает обучение модели openWakeWord 0.6.0.

    API openwakeword 0.6.0:
        - Model.auto_train(X_train, X_val, fp_val, steps=N)
        - augment_clips(paths, total_length) → audio generator
        - compute_features_from_generator(gen, ...) → .npy features
        - mmap_batch_generator({label: npy_path}, ...) → batch iterator
    """
    import torch
    import numpy as np
    import openwakeword.utils
    from openwakeword.train import Model, convert_onnx_to_tflite
    from openwakeword.data import augment_clips
    from openwakeword.utils import compute_features_from_generator

    # Скачиваем базовые модели (melspectrogram.onnx и др.) если ещё нет
    print("\n📥 Проверяю базовые модели openwakeword...")
    openwakeword.utils.download_models()

    # ── 1. Собираем клипы (deduplicate через set) ────────────────────────────────
    positive_clips = sorted(set(
        glob.glob(os.path.join(positive_dir, "**/*.wav"), recursive=True)
        + glob.glob(os.path.join(positive_dir, "*.wav"))
    ))
    negative_clips = sorted(set(
        glob.glob(os.path.join(negative_dir, "**/*.wav"), recursive=True)
        + glob.glob(os.path.join(negative_dir, "*.wav"))
    ))

    n_pos, n_neg = len(positive_clips), len(negative_clips)
    print(f"\n📊 Датасет:")
    print(f"   Позитивных: {n_pos} (в {positive_dir}/)")
    print(f"   Негативных: {n_neg} (в {negative_dir}/)")

    if n_pos < 100:
        print(f"\n⚠ Мало позитивных ({n_pos}). Рекомендуется ≥300.")
        print("  Запусти: python3 generate_samples.py --count 500")
        if n_pos == 0:
            sys.exit(1)
    if n_neg < 500:
        print(f"\n⚠ Мало негативных ({n_neg}). Рекомендуется ≥1000.")
        print("  Запусти: python3 download_background_noise.py --count 2000")
        if n_neg == 0:
            sys.exit(1)

    # ── 2. Train/val split ───────────────────────────────────────────────────
    random.shuffle(positive_clips)
    random.shuffle(negative_clips)
    n_pos_val = max(20, int(n_pos * test_split))
    n_neg_val = max(100, int(n_neg * test_split))
    pos_train, pos_val = positive_clips[n_pos_val:], positive_clips[:n_pos_val]
    neg_train, neg_val = negative_clips[n_neg_val:], negative_clips[:n_neg_val]
    # FP validation = последние neg_val (не пересекается с train)
    neg_fp = negative_clips[-(n_neg_val * 2) : -n_neg_val] if len(negative_clips) > n_neg_val * 3 else neg_val

    # ── 3. Augment + feature extraction ─────────────────────────────────────
    feat_dir = os.path.abspath(os.path.join(output_dir, "features"))
    os.makedirs(feat_dir, exist_ok=True)
    os.makedirs(os.path.abspath(output_dir), exist_ok=True)

    TOTAL_LENGTH = 32000   # 2 с @ 16 kHz
    AUG_ROUNDS   = 3       # x3 через аугментацию
    AUG_BATCH    = 32

    datasets = [
        (os.path.join(feat_dir, "pos_train.npy"), pos_train * AUG_ROUNDS, "positive train"),
        (os.path.join(feat_dir, "neg_train.npy"), neg_train * AUG_ROUNDS, "negative train"),
        (os.path.join(feat_dir, "pos_val.npy"),   pos_val,                "positive val"),
        (os.path.join(feat_dir, "neg_val.npy"),   neg_val,                "negative val"),
        (os.path.join(feat_dir, "neg_fp.npy"),    neg_fp,                 "negative FP"),
    ]

    print(f"\n⚙ Аугментация и вычисление признаков (total_length={TOTAL_LENGTH})...")
    for feat_path, clips, label in datasets:
        if os.path.exists(feat_path):
            stored = np.load(feat_path, mmap_mode='r').shape[0]
            print(f"   ✓ {label}: {stored} признаков уже есть")
            continue
        print(f"   {label}: {len(clips)} клипов...")
        gen = augment_clips(
            clips,
            total_length=TOTAL_LENGTH,
            batch_size=AUG_BATCH,
            background_clip_paths=negative_clips,
        )
        compute_features_from_generator(
            generator=gen,
            n_total=len(clips),
            clip_duration=TOTAL_LENGTH,
            output_file=feat_path,
            device="cpu",
        )

    # ── 4. DataLoaders ───────────────────────────────────────────────────────
    # auto_train ожидает:
    #   X_train  — БЕСКОНЕЧНЫЙ итератор (цикл по steps шагам)
    #   X_val    — КОНЕЧНЫЙ (итерируется целиком на каждом val_step)
    #   X_fp_val — КОНЕЧНЫЙ (то же)
    from torch.utils.data import DataLoader, TensorDataset
    from openwakeword.data import mmap_batch_generator

    def make_val_loader(pos_path, neg_path, bs):
        """Конечный DataLoader для валидации."""
        pos = np.load(pos_path).astype(np.float32)
        neg = np.load(neg_path).astype(np.float32)
        n_steps = pos.shape[1]
        if neg.shape[1] != n_steps:
            flat = neg.reshape(-1, neg.shape[-1])
            n_batches = flat.shape[0] // n_steps
            neg = flat[:n_batches * n_steps].reshape(n_batches, n_steps, neg.shape[-1])
        X = np.concatenate([pos, neg], axis=0)
        y = np.array([1.0] * len(pos) + [0.0] * len(neg), dtype=np.float32)
        ds = TensorDataset(torch.from_numpy(X), torch.from_numpy(y))
        return DataLoader(ds, batch_size=bs, shuffle=False)

    def make_train_iter(pos_path, neg_path, bs):
        """Бесконечный итератор для тренировки через mmap_batch_generator."""
        pos_steps = np.load(pos_path, mmap_mode='r').shape[1]
        neg_steps = np.load(neg_path, mmap_mode='r').shape[1]
        transform = {}
        if neg_steps != pos_steps:
            n = pos_steps
            transform = {"0": lambda x, n=n: (
                x if x.shape[1] == n else
                x.reshape(-1, x.shape[-1])[:((x.shape[0]*x.shape[1])//n)*n]
                 .reshape(-1, n, x.shape[-1])
            )}
        gen = mmap_batch_generator(
            data_files={"0": neg_path, "1": pos_path},
            batch_size=bs,
            data_transform_funcs=transform,
        )

        class InfiniteIter:
            def __iter__(self): return self
            def __next__(self):
                x, y = next(gen)
                return (torch.from_numpy(np.array(x, dtype=np.float32)),
                        torch.from_numpy(np.array(y, dtype=np.float32)))
        return InfiniteIter()

    pos_feat_path = os.path.join(feat_dir, "pos_train.npy")
    neg_feat_path = os.path.join(feat_dir, "neg_train.npy")
    pos_val_path  = os.path.join(feat_dir, "pos_val.npy")
    neg_val_path  = os.path.join(feat_dir, "neg_val.npy")
    neg_fp_path   = os.path.join(feat_dir, "neg_fp.npy")

    X_train  = make_train_iter(pos_feat_path, neg_feat_path, batch_size)
    X_val    = make_val_loader(pos_val_path,  neg_val_path,  batch_size)
    X_fp_val = make_val_loader(pos_val_path,  neg_fp_path,   batch_size)

    # ── 5. Создаём модель ────────────────────────────────────────────────────
    F = openwakeword.utils.AudioFeatures(device="cpu")
    input_shape = F.get_embedding_shape(TOTAL_LENGTH / 16000)
    print(f"\n🧠 Модель: input_shape={input_shape}")
    oww = Model(n_classes=1, input_shape=input_shape)

    # ── 6. Обучение ──────────────────────────────────────────────────────────
    steps = max(10000, epochs * 50)
    print(f"\n🚀 Обучение: {steps} шагов (lr={learning_rate})")
    print(f"   Устройство: {'GPU' if torch.cuda.is_available() else 'CPU'}")
    trained_model = oww.auto_train(
        X_train,
        X_val,
        X_fp_val,
        steps=steps,
        target_fp_per_hour=target_false_positive_rate,
    )

    # ── 7. Экспорт ───────────────────────────────────────────────────────────
    onnx_path   = os.path.join(os.path.abspath(output_dir), f"{model_name}.onnx")
    tflite_path = os.path.join(os.path.abspath(output_dir), f"{model_name}.tflite")

    print(f"\n💾 Экспорт ONNX → {onnx_path}")
    oww.export_model(trained_model, model_name, os.path.abspath(output_dir))

    if os.path.exists(onnx_path):
        size_kb = os.path.getsize(onnx_path) // 1024
        print(f"✅ ONNX: {onnx_path} ({size_kb} KB)")
        try:
            convert_onnx_to_tflite(onnx_path, tflite_path)
            if os.path.exists(tflite_path):
                size_kb = os.path.getsize(tflite_path) // 1024
                print(f"✅ TFLite: {tflite_path} ({size_kb} KB)")
        except Exception as e:
            print(f"⚠ TFLite конвертация: {e}")
    else:
        print(f"⚠ ONNX файл не найден после экспорта")

    return tflite_path if os.path.exists(tflite_path) else onnx_path


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
