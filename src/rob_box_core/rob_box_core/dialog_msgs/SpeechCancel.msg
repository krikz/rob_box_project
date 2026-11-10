# Отмена реплики (ADR-0150 §10.1). Заменяет /voice/tts/control STOP.

uint8 BOUNDARY_AS_REQUESTED=0
uint8 BOUNDARY_NOW=1
# Какую реплику отменить; "" - все реплики агента с эпохой не новее указанной.
string speech_id
# Эпоха отмены: отменяются реплики с эпохой не новее.
uint32 epoch
# Чьи реплики отменяются.
string agent
# Причина отмены (barge_in, silenced, stop, replaced).
string reason
# Где резать звучащую реплику: как в её SpeechRequest.boundary (дочитать предложение) или сразу («стой», PR-7c).
uint8 boundary
