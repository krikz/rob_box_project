# Распознанная реплика (ADR-0150 §10.1, ADR-0131).
# Заменяет String /voice/stt/result и JSON /voice/stt/utterance.
# Пишут stt_node (respeaker) и telegram_node (telegram), читает dialogue_node.

# Детерминированный id фразы по PCM; "" - у Telegram и операторского текста id нет.
string utterance_id
# Текст реплики без префиксов.
string text
string SOURCE_RESPEAKER="respeaker"
string SOURCE_TELEGRAM="telegram"
string SOURCE_OPERATOR="operator"
# Источник реплики.
string source
# Идентификатор чата Telegram; 0 - не Telegram.
int64 chat_id
# Метка диктора от STT-провайдера (Yandex speaker_tag); "" - нет.
string speaker_tag
# Начало фразы (время ROS); нулевое - неизвестно.
builtin_interfaces/Time started_at
# Конец фразы (время ROS); нулевое - неизвестно.
builtin_interfaces/Time ended_at
# Провайдер STT (yandex, vosk, telegram).
string provider
