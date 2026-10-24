# Запрос на озвучку (ADR-0150 §10.1). Заменяет /voice/tts/request и /voice/dialogue/response.

# Идентификатор реплики; по нему приходят SpeechEvent и SpeechCancel.
string speech_id
# Ход диалога, породивший реплику ("" - реплика вне хода).
string turn_id
# Эпоха сессии на момент запроса; реплика устаревшей эпохи отбрасывается.
uint32 epoch
# Агент-автор реплики (robot, tars, telegram).
string agent
# Приёмник звука (speaker, telegram); "" - колонки робота.
string sink
# Приоритет очереди: больше - важнее.
uint8 priority
# Можно ли прервать реплику перебиванием.
bool interruptible
# Текст реплики (или SSML, если провайдер его поддерживает).
string text
# Запрошенный голос; "" - голос сессии.
string voice
# Язык текста (ru, en).
string language
# Эмоция озвучки из enum знания диалога; "" - нейтрально.
string emotion
# Время старта, секунды ROS-часов; 0 - сразу.
float64 at
uint8 BOUNDARY_NONE=0
uint8 BOUNDARY_SENTENCE=1
# Граница, на которой реплику можно разрезать.
uint8 boundary
# false - только синтез (прогрев кэша), true - играть.
bool commit
# Время создания запроса.
builtin_interfaces/Time stamp
