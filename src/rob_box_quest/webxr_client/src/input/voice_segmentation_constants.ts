// webxr_client/src/input/voice_segmentation_constants.ts
//
// Зеркало серверного rob_box_core/config/speech_segmentation.yaml
// (issue #2199: единый сегментатор вместо четырёх правил).
//
// Эти константы ДОЛЖНЫ быть согласованы с профилем ``wake`` на
// сервере — иначе клиент будет слать фразу, пока сервер ещё не
// считает её завершённой (или наоборот). Проверка при деплое:
//   PYTHONPATH=src/rob_box_core python3 -m rob_box_core.tools.check_ts_sync
// (см. ``rob_box_core/tools/check_ts_sync.py``, добавим в CI).
//
// Прямо сейчас значения скопированы вручную из YAML (профиль ``wake``,
// секция ``client_vad``); при правке YAML обязательно править и этот файл.
// В перспективе — генерировать TS-файл из YAML на этапе сборки
// (tools/generate_ts_voice_constants.py, ADR-0024 в очереди).

/**
 * RMS int16 PCM: кадр считается речью, если его RMS ≥ этого порога.
 * Зеркало ``profiles.wake.client_vad.rms_threshold`` в
 * ``rob_box_core/config/speech_segmentation.yaml``.
 *
 * 200 int16 единиц ≈ 0.0061 от максимума — отсеивает микрофонный шум,
 * оставляет голос (формула из voice_capture.ts:rmsInt16).
 */
export const VOICE_RMS_THRESHOLD_DEFAULT = 200;

/**
 * Hangover: после последнего голоса шлём ещё N мс тишины, чтобы
 * не рвать слоги внутри слова. Зеркало
 * ``profiles.wake.client_vad.hangover_ms``.
 *
 * Серверный ``gap_timeout_s`` (400 мс) включает этот hangover + запас
 * на сетевой джиттер; править оба согласованно.
 */
export const VOICE_HANGOVER_MS_DEFAULT = 200;
