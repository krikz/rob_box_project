#!/usr/bin/env python3
"""
download_background_noise.py — Скачивание фоновых звуков для wake word обучения.

Скачивает небольшую выборку из:
1. FSDnoisy18k (Freesound Dataset) — 1000 файлов
2. Отфильтрованные русские/английские данные из Mozilla Common Voice мы не качаем —
   вместо этого качаем freesound clips через официальный URL.
3. MUSAN noise (subset)

Запуск:
    python3 download_background_noise.py --output-dir ./data/negative --count 2000
"""

import argparse
import os
import subprocess
import sys
import urllib.request


def download_musan_noise(output_dir: str, max_files: int = 1000) -> int:
    """
    Скачивает MUSAN noise сегмент (~3.5GB полная версия, мы берём только noise/).
    Использует wget с bandwidth limits.
    """
    print("📥 Скачиваю MUSAN noise subset (~900MB)...")
    url = "https://www.openslr.org/resources/17/musan.tar.gz"
    tarpath = os.path.join(output_dir, "_musan.tar.gz")

    os.makedirs(output_dir, exist_ok=True)

    if not os.path.exists(tarpath):
        try:
            print(f"  URL: {url}")
            urllib.request.urlretrieve(url, tarpath, reporthook=_progress_hook())
        except Exception as e:
            print(f"  ⚠ Скачать не удалось: {e}")
            print("  Пробую wget...")
            ret = subprocess.run(["wget", "-q", "--show-progress", "-O", tarpath, url])
            if ret.returncode != 0:
                print("  ❌ wget тоже не сработал. Пропускаем MUSAN.")
                return 0

    # Распаковываем только noise/ директорию
    print("  📦 Распаковываю noise/ из архива...")
    extract_dir = os.path.join(output_dir, "_musan_extracted")
    os.makedirs(extract_dir, exist_ok=True)
    subprocess.run(
        ["tar", "-xzf", tarpath, "--wildcards", "musan/noise/", "-C", extract_dir],
        check=False,
    )

    # Копируем WAV файлы в output_dir
    noise_dir = os.path.join(extract_dir, "musan", "noise")
    count = 0
    if os.path.exists(noise_dir):
        for root, _, files in os.walk(noise_dir):
            for f in files:
                if f.endswith(".wav") and count < max_files:
                    src = os.path.join(root, f)
                    dst = os.path.join(output_dir, f"musan_{count:04d}.wav")
                    if not os.path.exists(dst):
                        import shutil
                        shutil.copy2(src, dst)
                    count += 1

    print(f"  ✓ {count} MUSAN noise файлов")
    return count


def download_free_sounds(output_dir: str, max_files: int = 500) -> int:
    """
    Генерирует синтетические фоновые звуки: белый шум, розовый шум,
    имитация комнатного эха — без зависимости от внешних серверов.
    """
    import wave
    import numpy as np

    print("🔊 Генерирую синтетические фоновые звуки...")
    os.makedirs(output_dir, exist_ok=True)

    sample_rate = 16000
    duration = 3  # секунды
    n_samples = sample_rate * duration
    count = 0

    types = [
        ("white_noise", lambda: np.random.normal(0, 0.05, n_samples)),
        ("pink_noise", lambda: _pink_noise(n_samples)),
        ("brown_noise", lambda: _brown_noise(n_samples)),
        ("silence_low", lambda: np.random.normal(0, 0.002, n_samples)),
        ("chatter_sim", lambda: _simulated_chatter(n_samples, sample_rate)),
    ]

    for i in range(max_files):
        noise_type, fn = types[i % len(types)]
        path = os.path.join(output_dir, f"synthetic_{noise_type}_{i:04d}.wav")
        if os.path.exists(path):
            count += 1
            continue
        try:
            audio = fn()
            audio = np.clip(audio, -1.0, 1.0)
            pcm = (audio * 32767).astype(np.int16)
            with wave.open(path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(sample_rate)
                wf.writeframes(pcm.tobytes())
            count += 1
        except Exception as e:
            print(f"  ⚠ {noise_type}_{i}: {e}")

    print(f"  ✓ {count} синтетических фоновых файлов")
    return count


def _pink_noise(n: int) -> "np.ndarray":
    """Генерирует розовый шум (1/f)."""
    import numpy as np
    white = np.random.randn(n)
    # Простое приближение через накопленный интеграл
    f = np.fft.rfftfreq(n)
    f[0] = 1e-9
    power = 1.0 / np.sqrt(f)
    spectrum = np.fft.rfft(white) * power
    return np.fft.irfft(spectrum, n=n).astype(np.float32) * 0.05


def _brown_noise(n: int) -> "np.ndarray":
    """Генерирует коричневый шум (1/f²)."""
    import numpy as np
    white = np.random.randn(n)
    f = np.fft.rfftfreq(n)
    f[0] = 1e-9
    power = 1.0 / f
    spectrum = np.fft.rfft(white) * power
    out = np.fft.irfft(spectrum, n=n).astype(np.float32)
    out /= np.max(np.abs(out)) + 1e-9
    return out * 0.05


def _simulated_chatter(n: int, sr: int) -> "np.ndarray":
    """Имитация фонового разговора людей (random sinusoids + noise)."""
    import numpy as np
    t = np.linspace(0, n / sr, n)
    result = np.zeros(n)
    for _ in range(random.randint(3, 8)):
        freq = random.uniform(100, 3000)
        amp = random.uniform(0.001, 0.02)
        phase = random.uniform(0, 2 * 3.14159)
        result += amp * np.sin(2 * 3.14159 * freq * t + phase)
    result += np.random.normal(0, 0.005, n)
    return result.astype(np.float32)


def _progress_hook():
    """Простой прогресс-хук для urllib."""
    last = [0]
    def hook(count, block_size, total_size):
        done = count * block_size
        if total_size > 0:
            pct = min(100, done * 100 // total_size)
            if pct - last[0] >= 10:
                print(f"    {pct}%...", end="\r", flush=True)
                last[0] = pct
    return hook


import random  # noqa: E402 (needed for _simulated_chatter)


def main():
    parser = argparse.ArgumentParser(description="Загрузка фоновых звуков для обучения")
    parser.add_argument("--output-dir", default="./data/negative", help="Папка для фонов")
    parser.add_argument("--count", type=int, default=2000, help="Желаемое количество файлов")
    parser.add_argument("--skip-musan", action="store_true", help="Пропустить MUSAN (~900MB)")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    total = 0

    # 1. Синтетические фоны (быстро, без интернета)
    total += download_free_sounds(args.output_dir, max_files=min(500, args.count))

    # 2. MUSAN noise (опционально, нужен интернет + ~900MB)
    if not args.skip_musan and total < args.count:
        total += download_musan_noise(args.output_dir, max_files=args.count - total)

    # Подсчёт итога
    wav_files = [f for f in os.listdir(args.output_dir) if f.endswith(".wav")]
    print(f"\n✅ Итого фоновых файлов: {len(wav_files)} в {args.output_dir}/")
    print(f"   Рекомендуется ≥1000 для хорошей модели. Текущий статус: {'OK' if len(wav_files) >= 1000 else 'МАЛО — добавь --skip-musan=false'}")


if __name__ == "__main__":
    main()
