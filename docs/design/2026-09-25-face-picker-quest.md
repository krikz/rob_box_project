# Дизайн: лицевые команды на капитанском мостике (Quest)

> Краткая рабочая заметка к ADR-0135. Решения и обоснования — в ADR. Здесь —
> только то, что нужно держать под рукой во время реализации: API-форма,
> ссылки на образцы, чеклисты.

## Контекст в одном абзаце

Парная задача к #3025 (Telegram `/faces` + `/face <id>` с коллажем 4×3). В Quest
нужен тот же функционал в 3D-интерфейсе «капитанский мостик» на Meta Quest.
ADR-0123 §9 пункт 8 («Просмотр в Quest») — наша карточка. Полное обоснование
выбора транспорта, гейта приватности и UI — ADR-0135.

## Выбранный транспорт (резюме ADR §3)

Расширяем существующий WSS-канал `rob_box_quest` по образцу `voice_picker` —
**НЕ** вводим HTTP-эндпоинты и **НЕ** заводим stream в `STREAM_CATALOG.

### Wire-каталог

Добавить в `rob_box_core.bridge_protocol` (SSOT, ADR-0080 §2.2):

```python
# Команды
CommandSpec("face_list",    payload=EmptyPayload,      result="face_list"),     # пустое тело
CommandSpec("face_get",     payload={person_id: str},  result="face_get_ack"),  # запрашивает коллаж

# События
EventSpec("face_list",    payload={"persons": list[FaceSummaryDTO]})
EventSpec("face_get_ack", payload={"person_id": str, "summary": FaceSummaryDTO, "stream_id": int})
EventSpec("face_get_done", payload={"person_id": str, "stream_id": int, "bytes": int})
EventSpec("face_get_nack", payload={"person_id": str, "reason": str})
# reason ∈ {"unknown_person", "strict_mode", "exhibition_stranger", "no_photos"}
```

Регенерировать `protocol_generated.ts` через `tools/gen_bridge_protocol_ts.py`.

### DTO

```python
class FaceSummaryDTO(TypedDict):
    person_id: str           # короткий id из meta.json (например, "4ff0ddc5")
    name: str | None         # None для незнакомцев
    speaker_id: str | None   # привязка к голосу (ADR-0106 шов «Знакомый»)
    encounter_count: int
    photo_count: int         # сколько JPEG реально есть на диске
    gallery_cohesion: float  # из meta.json
    mode_recorded: str       # "workshop" | "exhibition" | "strict" — для UI-честности (nack рисует на его основе)
    has_collage: bool        # сервер уже решил; False в strict
```

### Клиент-сервер поток `face_get`

```
Client                                    Server (rob_box_quest)
  │                                                │
  │ JSON_CMD{cmd:"face_get", person_id:"4ff0ddc5"} │
  │ ─────────────────────────────────────────────► │
  │                                                │ FaceStore.gallery(person)
  │                                                │ → build_face_collage(person_dir) (PIL)
  │ JSON_EVENT{type:"face_get_ack",                 │
  │   person_id, summary, stream_id=0x1000+1}       │
  │ ◄───────────────────────────────────────────── │
  │                                                │
  │ BINARY_FRAME stream_id=0x1000+1 (JPEG bytes)   │
  │ ◄───────────────────────────────────────────── │
  │                                                │
  │ JSON_EVENT{type:"face_get_done",                │
  │   person_id, stream_id, bytes}                  │
  │ ◄───────────────────────────────────────────── │
```

Nack-путь:

```
Client                                    Server
  │                                                │
  │ JSON_CMD{cmd:"face_get", person_id:"..."}      │
  │ ─────────────────────────────────────────────► │
  │                                                │ FaceStore says: no / strict / unknown
  │ JSON_EVENT{type:"face_get_nack",                │
  │   person_id, reason}                            │
  │ ◄───────────────────────────────────────────── │
```

## Сборка коллажа (резюме ADR §4)

Общий модуль (предложение места — `src/rob_box_core/face_collage.py`,
импортируется и Telegram-ботом из #3025, и `rob_box_quest` из этой карточки):

```python
def build_face_collage(person_dir: Path, *, grid: tuple[int, int] = (4, 3)) -> bytes:
    """JPEG bytes. Raises FaceCollageUnavailable(reason) если нечего показать."""
```

Тесты — чистый pytest, без `rclpy`/`aiohttp`. **Импортируется без ROS** —
это уже требование #3025 («отдельная чистая функция, юнит-тестируемая
без ROS/без железа»), повторяем его для обеих фаз.

Формат коллажа — по #3025 §«Коллаж» (эталон Шифу): сетка 4×3, первая плитка
`reference.jpg`, дальше кропы встреч; пустые ячейки — белый фон; подпись
`<имя> WxH` внизу каждой плитки; встреча с `face_snapshot=null` пропускается.

## UI (резюме ADR §5)

Файлы для реализации:

| Файл | Зачем | Образец |
|---|---|---|
| `webxr_client/src/scene/face_picker_menu.ts` | Список лиц (плавающая панель, строки + pager + кнопки) | `tts_picker_menu.ts` (1:1 по структуре) |
| `webxr_client/src/scene/face_card_panel.ts` | Карточка с коллажем | `video_panel.ts` (одна текстура + canvas-overlay для подписей) |
| `webxr_client/src/state/face_picker_state.ts` | Чистый редьюсер, 5 фаз: loading/empty/ready/loading-card/loaded-card | `tts_picker_state.ts` |

Targets регистрируются в PointerSystem с префиксом `face:*` (как
`tts:*`/`stream:*`).

## Гейт приватности (резюме ADR §6)

Никакого нового гейта. Уже существующая WSS-PIN-аутентификация =
эквивалент `@authorized` для Telegram. Режим (`workshop`/`exhibition`/
`strict`) — ответственность `FaceStore` (ADR-0123 §5), не Quest.

Nack-причины на стороне сервера:

| Причина | Когда | Что рисует Quest |
|---|---|---|
| `unknown_person` | `person_id` нет в `/data/faces/` | toast: «лицо не найдено» |
| `strict_mode` | `mode_recorded == "strict"` | toast: «снимки отключены режимом приватности» |
| `exhibition_stranger` | незнакомец в `exhibition` (на диске его нет) | toast: «незнакомцы не сохраняются в режиме выставки» |
| `no_photos` | знакомый есть, но фото = 0 (например, только-что созданная запись) | пустая карточка с подписью «нет фото» |

## Docker-compose (резюме ADR §10 пункт 6)

В сервисе `rob_box_quest` добавить read-only том:

```yaml
services:
  rob_box_quest:
    volumes:
      - ./data/faces:/data/faces:ro   # ← новая строка
```

Тот же приём, что в #3025 для `telegram-bot`.

## Чеклист для реализации (по карточкам из ADR §10)

- [ ] Карточка 1: `face_collage.py` + pytest (без ROS)
- [ ] Карточка 2: правки в `rob_box_core.bridge_protocol`, регенерация TS
- [ ] Карточка 3: FaceStore-readonly-фасад + `ws_server.py` JSON_CMD-handler + nack-тесты
- [ ] Карточка 4: `face_picker_menu.ts` + `face_card_panel.ts` + `face_picker_state.ts`
- [ ] Карточка 5: e2e voice-команда «Робот, покажи лица» в `.github/e2e/voice_commands/`
- [ ] Карточка 6: `./data/faces:/data/faces:ro` в docker-compose

## Открытые вопросы к владельцу (ADR §13)

1. Сортировка в списке — «именованные первыми» как в Telegram?
2. `MAX_VISIBLE_ROWS` — 6 или 8?
3. Голосовая активация — только «открыть/закрыть» (рекомендация), или ещё «выбрать лицо голосом»?
4. Миниатюра в строке списка (рекомендация: нет)?

После ответа на эти вопросы конкретные карточки реализации уйдут в
kanban через `kanban_create`.
