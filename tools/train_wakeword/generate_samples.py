#!/usr/bin/env python3
"""
generate_samples.py — Генерация позитивных аудио-сэмплов для wake word обучения.

Поддерживает два TTS-движка:
  1. Silero TTS v5 (локально, 5 голосов)
  2. Yandex SpeechKit v1 REST API (6 голосов, высокое качество)

Запуск:
    # Silero (локально, без интернета):
    python3 generate_samples.py --engine silero --count 300

    # Yandex (нужен ключ):
    YANDEX_API_KEY=<key> python3 generate_samples.py --engine yandex --count 300

    # Оба движка вместе (максимальное разнообразие):
    YANDEX_API_KEY=<key> python3 generate_samples.py --engine both --count 600

    # Ключ явно:
    python3 generate_samples.py --engine yandex --yandex-api-key <key> --count 300

Что генерируется:
    ./data/positive/
        sample_0000.wav ... sample_XXXX.wav   (16kHz mono WAV)
"""

import argparse
import itertools
import os
import random
import sys
import wave

# ──────────────────────────────────────────────────────────────────────────────
# Целевые фразы: все варианты + окружение
# ──────────────────────────────────────────────────────────────────────────────
TARGET_PHRASES = [
    # Основные
    "роббокс",
    "робокс",
    "робок",
    "робот",
    # С паузой-контекстом (имитирует реальные обращения)
    "роббокс привет",
    "эй роббокс",
    "ну роббокс",
    "робот привет",
    "эй робот",
    # Вариации произношения (немного другие написания = другое TTS-произношение)
    "роб бокс",
    "роб окс",
]

# Голоса Silero v5
SILERO_SPEAKERS = ["aidar", "baya", "kseniya", "xenia", "eugene"]

# Голоса Yandex SpeechKit v1 (русские)
# https://yandex.cloud/ru/docs/speechkit/tts/voices
YANDEX_SPEAKERS = ["alena", "filipp", "jane", "ermil", "zahar", "omazh"]

# Скорости (нормальная, чуть быстрее, чуть медленнее)
SPEEDS = [1.0, 0.9, 1.1, 0.85, 1.15]

# URL Yandex SpeechKit REST API v1
YANDEX_TTS_URL = "https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize"


def setup_silero(sample_rate: int = 16000):
    """Загружает модель Silero TTS."""
    import torch
    model, _ = torch.hub.load(
        repo_or_dir="snakers4/silero-models",
        model="silero_tts",
        language="ru",
        speaker="v3_1_ru",
    )
    model.eval()
    return model


def synthesize(model, text: str, speaker: str, speed: float, sample_rate: int = 16000) -> bytes:
    """
    Синтезирует речь и возвращает PCM-16 bytes.
    Returns bytes of 16kHz mono int16 PCM.
    """
    import torch
    import numpy as np

    with torch.no_grad():
        audio = model.apply_tts(
            text=text,
            speaker=speaker,
            sample_rate=sample_rate,
            put_accent=True,
            put_yo=True,
        )
    # audio is a torch tensor
    if hasattr(audio, "numpy"):
        audio_np = audio.numpy()
    else:
        audio_np = audio

    # Normalize to int16
    audio_np = np.clip(audio_np, -1.0, 1.0)
    pcm = (audio_np * 32767).astype(np.int16)
    return pcm.tobytes()


def synthesize_yandex(text: str, speaker: str, speed: float, api_key: str, sample_rate: int = 16000) -> bytes:
    """
    Синтезирует речь через Yandex SpeechKit REST API v1.

    Args:
        text:        Текст для синтеза
        speaker:     Голос (alena, filipp, jane, ermil, zahar, omazh)
        speed:       Скорость речи 0.1-3.0 (1.0 = нормальная)
        api_key:     Yandex API Key (из YANDEX_API_KEY)
        sample_rate: Целевая частота (API отдаёт 48000, мы ресэмплируем до 16000)

    Returns:
        PCM-16 mono bytes на нужной частоте дискретизации.

    Raises:
        RuntimeError: При HTTP ошибке или недоступности requests/scipy.
    """
    try:
        import urllib.request
        import urllib.parse
        import numpy as np
    except ImportError as e:
        raise RuntimeError(f"Не хватает пакета: {e}") from e

    # Yandex lpcm v1: 16-bit LE mono, 48000 Hz (единственный поддерживаемый SR в lpcm)
    yandex_sr = 48000

    payload = urllib.parse.urlencode({
        "text": text,
        "lang": "ru-RU",
        "voice": speaker,
        "speed": str(speed),
        "format": "lpcm",
        "sampleRateHertz": str(yandex_sr),
    }).encode("utf-8")

    req = urllib.request.Request(
        YANDEX_TTS_URL,
        data=payload,
        headers={
            "Authorization": f"Api-Key {api_key}",
            "Content-Type": "application/x-www-form-urlencoded",
        },
    )

    with urllib.request.urlopen(req, timeout=30) as resp:
        if resp.status != 200:
            body = resp.read(256)
            raise RuntimeError(f"Yandex API вернул {resp.status}: {body}")
        raw = resp.read()

    # raw - это raw LPCM 16-bit LE 48kHz mono
    audio_np = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32767.0

    # Ресэмплирование 48000 → 16000 (ratio = 1/3)
    if yandex_sr != sample_rate:
        try:
            from scipy.signal import resample_poly
            # 48000 → 16000: downsample 3x
            gcd = _gcd(yandex_sr, sample_rate)
            up = sample_rate // gcd
            down = yandex_sr // gcd
            audio_np = resample_poly(audio_np, up, down).astype(np.float32)
        except Exception:  # ImportError или AttributeError (scipy скомпилирован под numpy 1.x)
            # Простой децимация без scipy — берём каждый 3-й семпл (если ratio целое)
            ratio = yandex_sr // sample_rate
            if yandex_sr % sample_rate == 0:
                audio_np = audio_np[::ratio]
            else:
                # Линейная интерполяция через numpy
                old_len = len(audio_np)
                new_len = int(old_len * sample_rate / yandex_sr)
                indices = np.linspace(0, old_len - 1, new_len)
                audio_np = np.interp(indices, np.arange(old_len), audio_np).astype(np.float32)

    pcm = np.clip(audio_np, -1.0, 1.0)
    return (pcm * 32767).astype(np.int16).tobytes()


def _gcd(a: int, b: int) -> int:
    """Наибольший общий делитель."""
    while b:
        a, b = b, a % b
    return a


def write_wav(path: str, pcm: bytes, sample_rate: int = 16000) -> None:
    """Записывает PCM-16 mono bytes в WAV файл."""
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(pcm)


def add_silence(pcm: bytes, before_ms: int = 100, after_ms: int = 200, sample_rate: int = 16000) -> bytes:
    """Добавляет тишину до и после фразы."""
    before = bytes(int(before_ms / 1000 * sample_rate) * 2)
    after = bytes(int(after_ms / 1000 * sample_rate) * 2)
    return before + pcm + after


def add_noise(pcm: bytes, noise_level: float = 0.005) -> bytes:
    """Добавляет лёгкий белый гаусс-шум чтобы модель была более robustной."""
    import numpy as np
    audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
    noise = np.random.normal(0, noise_level * 32767, len(audio)).astype(np.float32)
    augmented = np.clip(audio + noise, -32767, 32767).astype(np.int16)
    return augmented.tobytes()


def main():
    parser = argparse.ArgumentParser(description="Генерация wake word сэмплов через Silero TTS и/или Yandex SpeechKit")
    parser.add_argument("--output-dir", default="./data/positive", help="Папка для сэмплов")
    parser.add_argument("--count", type=int, default=500,
                        help="Количество сэмплов на движок (рекомендуется 300-1000)")
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--engine",
        choices=["silero", "yandex", "both"],
        default="silero",
        help="TTS движок: silero (локально), yandex (требует API ключ), both (оба)",
    )
    parser.add_argument(
        "--yandex-api-key",
        default=os.getenv("YANDEX_API_KEY", ""),
        help="Yandex API Key (или задай YANDEX_API_KEY в env). "
             "Ключ лежит в .env.secrets на Vision Pi (10.1.1.21).",
    )
    args = parser.parse_args()

    random.seed(args.seed)
    os.makedirs(args.output_dir, exist_ok=True)

    use_silero = args.engine in ("silero", "both")
    use_yandex = args.engine in ("yandex", "both")

    if use_yandex and not args.yandex_api_key:
        print("❌ Для Yandex нужен API ключ:")
        print("   export YANDEX_API_KEY=<ключ>")
        print("   # или --yandex-api-key <ключ>")
        print("   # ключ в .env.secrets на Vision Pi: sshpass -p open ssh ros2@10.1.1.21 'grep YANDEX /home/ros2/rob_box_project/docker/vision/.env.secrets'")
        sys.exit(1)

    # ──────────────────────────────────────────────────────────────────────────
    # Silero генерация
    # ──────────────────────────────────────────────────────────────────────────
    silero_model = None
    if use_silero:
        print("📦 Загружаю Silero TTS...")
        try:
            silero_model = setup_silero(args.sample_rate)
            print("✓ Silero TTS загружен")
        except Exception as e:
            print(f"❌ Ошибка загрузки Silero: {e}")
            print("  Попробуй: pip install torch==2.2.0")
            if not use_yandex:
                sys.exit(1)
            print("  ⚠ Продолжаю только с Yandex...")
            use_silero = False

    # ──────────────────────────────────────────────────────────────────────────
    # Построение списка заданий
    # ──────────────────────────────────────────────────────────────────────────
    jobs: list[tuple[str, str, float, str]] = []  # (phrase, speaker, speed, engine)

    if use_silero:
        silero_combos = list(itertools.product(TARGET_PHRASES, SILERO_SPEAKERS, SPEEDS))
        random.shuffle(silero_combos)
        while len(silero_combos) < args.count:
            silero_combos.extend(silero_combos)
        for phrase, speaker, speed in silero_combos[:args.count]:
            jobs.append((phrase, speaker, speed, "silero"))

    if use_yandex:
        yandex_combos = list(itertools.product(TARGET_PHRASES, YANDEX_SPEAKERS, SPEEDS))
        random.shuffle(yandex_combos)
        while len(yandex_combos) < args.count:
            yandex_combos.extend(yandex_combos)
        for phrase, speaker, speed in yandex_combos[:args.count]:
            jobs.append((phrase, speaker, speed, "yandex"))

    random.shuffle(jobs)
    total = len(jobs)

    engines_str = "+".join(filter(None, [
        f"Silero({len(SILERO_SPEAKERS)} голосов)" if use_silero else "",
        f"Yandex({len(YANDEX_SPEAKERS)} голосов)" if use_yandex else "",
    ]))
    print(f"\n🎤 Генерирую {total} сэмплов в {args.output_dir}/")
    print(f"   Движки: {engines_str}")
    print(f"   Фразы: {len(TARGET_PHRASES)}, скорости: {len(SPEEDS)}")
    if use_yandex:
        print(f"   Yandex voices: {', '.join(YANDEX_SPEAKERS)}")

    # ──────────────────────────────────────────────────────────────────────────
    # Генерация
    # ──────────────────────────────────────────────────────────────────────────
    errors = 0
    for idx, (phrase, speaker, speed, engine) in enumerate(jobs):
        path = os.path.join(args.output_dir, f"sample_{idx:05d}.wav")

        try:
            if engine == "silero":
                pcm = synthesize(silero_model, phrase, speaker, speed, args.sample_rate)
            else:
                pcm = synthesize_yandex(phrase, speaker, speed, args.yandex_api_key, args.sample_rate)

            # Аугментация: случайная тишина + лёгкий шум
            pcm = add_silence(
                pcm,
                before_ms=random.randint(50, 300),
                after_ms=random.randint(100, 400),
                sample_rate=args.sample_rate,
            )
            if random.random() > 0.5:
                pcm = add_noise(pcm, noise_level=random.uniform(0.001, 0.01))

            write_wav(path, pcm, args.sample_rate)

            if (idx + 1) % 50 == 0:
                print(f"  [{idx+1}/{total}] {engine}: '{phrase}' ({speaker}, x{speed})")

        except Exception as e:
            errors += 1
            print(f"  ⚠ [{idx}] {engine} ошибка '{phrase}' {speaker}: {e}")
            # Yandex rate limit: небольшая задержка
            if engine == "yandex" and "429" in str(e):
                import time
                print("  ⏳ Rate limit — жду 2с...")
                time.sleep(2)

    good = total - errors
    print(f"\n✅ Готово: {good}/{total} сэмплов в {args.output_dir}/")
    if errors:
        print(f"⚠ {errors} ошибок — проверь лог выше")


if __name__ == "__main__":
    main()
