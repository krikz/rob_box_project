# Намерение-команда (ADR-0150 §10.1). Для command_node вместо разбора текста /voice/stt/result.

# Ход диалога.
string turn_id
# Эпоха.
uint32 epoch
# Вид намерения (stop_motion, move).
string kind
# Аргументы как JSON: открытый словарь.
string args_json
