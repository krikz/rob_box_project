# Вывод диалога в канал (ADR-0150 §10.1). Для telegram заменяет /voice/dialogue/response.

# Агент.
string agent
# Ссылка на канал (например telegram:495039871).
string channel_ref
# Текст.
string text
# Реплика речи, к которой привязан текст ("" - только текст).
string speech_id
