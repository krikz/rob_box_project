# rob_box_supervisor_msgs/Response.msg
#
# Ответ на ExecuteCommand (ADR-0051 §2.1).
#
# Зеркалит структуру legacy-ответов std_srvs/Trigger (success + JSON в
# ``message``), но в **типизированном** виде — клиент видит ``accepted``/
# ``applied``/``reason`` как отдельные поля, без JSON-парсинга на каждом
# вызове. НЕ молчим при отказе (ADR-0018): ``reason`` всегда заполнен,
# ``accepted`` отличает «сервис принял запрос» от «состояние реально
# изменилось» (монитор-режим: ``accepted=true, applied=false,
# reason="supervisor_in_monitor_mode"``).

bool   accepted              # сервис принял запрос (≠ applied!)
bool   applied               # состояние реально изменилось
string reason                # human-readable: "held_by_other",
                             # "supervisor_in_monitor_mode",
                             # "invalid_voice_mode: 'foo'", ...
string held_by               # для floor-команд: client_id текущего держателя
string actual_mode           # для SET_AVATAR_MODE: текущий режим после применения
bool   contacted_service     # false → fallback / монитор; для диагностики