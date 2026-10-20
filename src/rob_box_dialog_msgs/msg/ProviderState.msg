# Состояние провайдера LLM/TTS (ADR-0150 §10.1). Заменяет /voice/tts/provider_state и llm_health.json.

# Имя провайдера.
string name
# Вид (llm, tts, stt).
string kind
# Здоровье: ok | degraded | down.
string health
# С какого времени состояние.
builtin_interfaces/Time since
# Умеет вызывать тулы.
bool tool_calls
# Умеет принудительный выбор тула.
bool tool_choice
# Умеет JSON-режим.
bool json_mode
# Умеет структурированный вывод.
bool structured_output
# Умеет рассуждение.
bool thinking
# Понимает SSML.
bool ssml
# Умеет потоковый вывод.
bool streaming
# Доступные голоса (tts).
string[] voices
# Код последней ошибки; "" - нет.
string last_error_code
