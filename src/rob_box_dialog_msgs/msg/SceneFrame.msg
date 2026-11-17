# Кадр сцены сессии (ADR-0150 §7). Часть SessionState.

# Вид сцены (identity_ask, music).
string kind
# Кто поставил кадр.
string set_by
# Кадр живёт до этого времени; 0 - без срока.
builtin_interfaces/Time ttl_until
# Данные кадра как JSON: открытый словарь, схему проверяет код сцены.
string payload_json
