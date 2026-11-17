# Запрос исполнения тула (ADR-0150 §10.1). Заменяет /mcp/execute.

# Ход диалога.
string turn_id
# Эпоха.
uint32 epoch
# Агент.
string agent
# Имя тула.
string tool
# Аргументы тула как JSON: открытый словарь, схему проверяет каталог тулов.
string args_json
# Класс ресурса тула (motion, audio, light).
string cls
# Таймаут исполнения, с.
float32 timeout_s
