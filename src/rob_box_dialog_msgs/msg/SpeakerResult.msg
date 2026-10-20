# Результат голосовой биометрии и служебные ответы speaker_id_node (ADR-0131).
# Заменяет String-JSON /voice/speaker/result.

uint8 EVENT_MATCH=0
uint8 EVENT_REGISTERED=1
uint8 EVENT_REGISTER_ERROR=2
uint8 EVENT_RENAMED=3
uint8 EVENT_MERGED=4
# Вид сообщения: результат биометрии или ответ на служебную команду.
uint8 event
# Фраза, к которой относится результат; "" - нет.
string utterance_id
# Узнан ли диктор (EVENT_MATCH).
bool is_known
# Биометрическое id диктора.
string speaker_id
# Имя диктора; "" - имя скрыто или неизвестно (низкая уверенность).
string name
# Уверенность совпадения 0..1.
float32 confidence
# Внутренняя кличка диктора; "" - ещё нет.
string epithet
# Источник результата (face_hint); "" - голос.
string source
# Биометрия фразу не оценила (мало речи, нет эмбеддинга).
bool inconclusive
# Причина inconclusive.
string reason
# Предполагаемое имя при низкой уверенности; "" - нет.
string tentative_name
# Уверенность предположения.
float32 tentative_conf
# Вид предположения (tentative_single); "" - нет.
string tentative_kind
# Исход служебной команды (renamed, merged).
bool ok
# Код ошибки регистрации (too_short, no_utterance_context, utterance_not_found).
string error
# Длина реплики при too_short, с.
float32 duration_s
# Требуемая длина реплики, с.
float32 min_required_s
# Профиль переиспользован при регистрации.
bool reused_profile
# Диктор уже был узнан до регистрации.
bool already_known
# Конфликт голоса: имя профиля с похожим голосом; "" - конфликта нет.
string conflict_name
# Конфликт голоса: id профиля.
string conflict_speaker_id
# Конфликт голоса: оценка сходства.
float32 conflict_score
# Тёзка: имя совпало, голос не дотянул до порога слияния; "" - нет.
string twin_name
# Тёзка: id профиля.
string twin_speaker_id
# Тёзка: оценка сходства; NaN - нет оценки.
float32 twin_score
# Склейка: исходный профиль.
string src_speaker_id
# Склейка: целевой профиль.
string dst_speaker_id
# Склейка: перенесено эмбеддингов.
uint32 embeddings_moved
# Склейка: перенесено фактов памяти.
uint32 facts_moved
# Переименование: новое имя.
string new_name
