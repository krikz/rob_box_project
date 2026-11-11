# Запрос экспрессии (ADR-0150 §10.1, §5.5.9). Заменяет /voice/animation/request.

# Ход диалога.
string turn_id
# Эпоха.
uint32 epoch
# Агент.
string agent
# Эмоция из enum знания диалога; "" - не менять.
string emotion
# Жест из enum; "" - нет.
string gesture
# Короткий звуковой сигнал из enum; "" - нет.
string earcon
# Время жизни экспрессии, мс; 0 - до следующей стадии.
uint32 ttl_ms
# Сцена лица (FaceScene, enum-слоты, §5.5.9) как JSON; "" - нет. Открытый словарь слотов.
string face_json
