# Событие плана хода (ADR-0150 §10.1, §5.6.1): планировщик ресурсов rob_box_dialog.plan (PR-7b).

uint8 KIND_BLOCK_START=0
uint8 KIND_BLOCK_END=1
uint8 KIND_SEGMENT_STARTED=2
uint8 KIND_SEGMENT_FINISHED=3
uint8 KIND_SEGMENT_REPLACED=4
uint8 KIND_QUEUED=5
uint8 KIND_PREDICTION=6
uint8 KIND_REJECTED=7
# Идентификатор блока плана.
string block_id
# Ход диалога.
string turn_id
# Эпоха.
uint32 epoch
# Агент.
string agent
# Тип события.
uint8 kind
# Номер сегмента внутри блока.
uint16 segment_no
# Блок, после которого запланирован этот.
string after_block
# Ожидаемое время до старта, мс.
uint32 eta_ms
# Старт синхронизирован с тактом.
bool synced
# Причина (rejected, segment_replaced).
string reason
# Время события.
builtin_interfaces/Time stamp
