# Результат исполнения тула (ADR-0150 §10.1). Заменяет /mcp/result.

# Ход диалога.
string turn_id
# Имя тула.
string tool
# Статус (ok, failed, timeout, rejected).
string status
# Причина (не-ok).
string reason
# Данные результата как JSON: открытый словарь.
string data_json
# Ссылка на событие плана, породившее запрос.
string event_ref
