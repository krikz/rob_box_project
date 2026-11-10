# Огибающая озвучки для рта (ADR-0150 §5.5.4). Публикует tts_node перед каждым чанком.

# Реплика.
string speech_id
# Ход диалога.
string turn_id
# Момент старта воспроизведения чанка.
builtin_interfaces/Time t0
# Шаг окна огибающей, мс (по умолчанию 40).
uint16 frame_ms
# RMS 0..1 по окнам frame_ms.
float32[] levels
# Подсказка гласной по окнам (0 нет, 1 A, 2 O, 3 E); пусто, если не считается.
uint8[] vowel_hint
