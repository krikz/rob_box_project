# Событие хода (ADR-0150 §10.1). Заменяет /harness/task_events и /dialogue/control_ack.

# Ход диалога.
string turn_id
# Эпоха.
uint32 epoch
# Агент.
string agent
# Стадия хода (heard, thinking, acting, speaking, done).
string stage
# Статус стадии (started, finished, failed, skipped).
string status
# Причина (failed, skipped).
string reason
# Сколько вызовов LLM сделано в ходе.
uint8 llm_calls
# Провайдер LLM последнего вызова.
string llm_provider
# Задержка стадии, мс.
uint32 latency_ms
# Источник реплики (respeaker, telegram, operator).
string source
# Время события.
builtin_interfaces/Time stamp
