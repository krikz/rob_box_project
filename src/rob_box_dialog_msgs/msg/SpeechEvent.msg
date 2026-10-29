# Событие жизни реплики (ADR-0150 §10.1). Заменяет /voice/tts/state, /finished, /batch_*, /current_voice.

uint8 KIND_QUEUED=0
uint8 KIND_SYNTHESIZED=1
uint8 KIND_STARTED=2
uint8 KIND_PROGRESS=3
uint8 KIND_FINISHED=4
uint8 KIND_CANCELLED=5
uint8 KIND_VOICE_CHANGED=6
uint8 KIND_REJECTED=7
# Что произошло с репликой.
uint8 kind
# Длительность синтезированного звука, мс (0 - неизвестна).
uint32 duration_ms
# Реплика, к которой относится событие.
string speech_id
# Ход диалога.
string turn_id
# Эпоха реплики.
uint32 epoch
# Агент-автор.
string agent
# Сколько символов текста реально произнесено (cancelled, progress).
uint32 spoken_chars
# Длина исходного текста, символов.
uint32 text_len
# Секунды воспроизведения от начала реплики.
float32 seconds
# Голос, который просили.
string voice_requested
# Голос, который реально применён (честная деградация).
string voice_applied
# Провайдер синтеза, который отработал.
string provider
# Причина (cancelled, rejected, voice_changed).
string reason
# Время события.
builtin_interfaces/Time stamp
