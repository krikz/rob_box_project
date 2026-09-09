# E2E Voice Golden Recordings

Каталог эталонных .wav файлов для **baseline comparison** (issue #1396, артефакт `e2e-voice-baseline-diff-<run_id>.json`).

## Как используются

`e2e_baseline_diff.py` ищет golden-файл по **slug голосовой команды**:
`Робот, привет меня зовут Саша` → `robot_privet_menya_zovut_sasha.wav`.

Если файл найден — сравнивается `recording.wav` (свежий e2e прогон) с
golden по RMS (допуск ±3dB) и длительности (допуск ±30%). Расхождения
FAILят `e2e-voice-baseline-diff` артефакт; pass=ok без отличий или в
пределах порога.

Если файл НЕ найден — сравнение идёт с синтетическим baseline
(1s sine -23dB FS); pass=mic_working (recording громче -50dB FS).

## Как добавить новый golden

1. Прогоните e2e на сценарии, который дал **зелёный PASS** в
   `L-E2E Voice Test` (робот ответил корректно, RMS в норме,
   длительность соответствует ожиданию).
2. Скачайте артефакт `e2e-voice-recording-<run_id>` → `recording.wav`.
3. Положите под slug-именем в этот каталог:
   ```bash
   cp recording.wav .github/e2e/golden/robot_privet_privet.wav
   git add .github/e2e/golden/robot_privet_privet.wav
   git commit -m "feat(e2e): add golden recording for 'Робот, привет'"
   ```

## Конвенция slug

`e2e_baseline_diff.py:_slug_from_voice_text()` — простая транслитерация
русского в ascii + `[a-z0-9_]+`. Полный список символов — в этой функции.

## Чего НЕ делать

- **Не коммитить .wav с личными данными** (голос Шифу — публичный ОК,
  но если в golden окажется чувствительный фрагмент диалога — лучше
  переснять с короткой командой).
- **Не править golden-файлы молча** — каждое изменение должно быть
  в отдельном PR с обоснованием (был регресс микрофона / обновлён
  голос TTS / etc.).
