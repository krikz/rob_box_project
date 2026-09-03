## Why

Панель выбора стиля речи и языка (AV-28 §P7) была сдана как DOM-оверлей
(`ui/voice_presets_panel.ts`), но по замыслу на капитанском мостике
**весь голосовой пайплайн оператора** должен жить в 3D-сцене
(`docs/design/dialogue-mode-spec-2026-08-28.md` §3.6, W6-2), а в HTML
остаётся только ввод PIN. В immersive-vr DOM не виден вовсе — оператор
в очках не мог настраивать голос. На мостике уже была заготовка
`styleStub` («Стиль: скоро») в 3D-панели режимов — её нужно было довести.

## What changes

- Удалён DOM-оверлей `ui/voice_presets_panel.ts` (+ его CSS и тесты).
- Новая 3D-панель `scene/voice_pipeline_panel.ts` (canvas-текстура +
  кнопки-меши на слое указателя, как `supervisor_panel.ts`): пайплайн
  `голос → STT → LLM → TTS → динамик` + быстрые настройки ступеней.
- `captain_bridge.ts`: панель всегда видима справа (+105°), клики
  маршрутизируются через `onPipelineAction` (сцена не знает транспорт).
- `main.ts`: убран DOM-panel wiring; стиль/язык → `set_voice`, тумблеры
  STT/LLM → `voice_mode`, кнопка TTS → открывает TTS picker (AV-27).
- Чистые функции (`renderHud`, `computePipelineLayout`, `hitTest`,
  `parsePipelineTargetId`) вынесены и покрыты unit-тестами.

## Impact

- WebXR-клиент: голос настраивается лучом контроллера в VR.
- Сервер (`rob_box_quest`) не менялся — wire-контракт `set_voice` /
  `voice_mode` уже существовал.
- Живая телеметрия ступеней (§3.7) — отдельная карточка, не входит.

Refs: krikz/rob_box_project#1920 (AV-28),
[W6-2](../../../docs/plans/2026-08-29-wave2-worker-prompts.md),
[спека §3.6–§3.7](../../../docs/design/dialogue-mode-spec-2026-08-28.md),
[ADR-0027 §3.4.1](../../../docs/adr/0027-meta-quest-ar-control.md).

## Out of scope

- Живое состояние пайплайна (§3.7): нет wire-событий со стадией/задержкой.
- STT/LLM-тумблеры дают корректный `voice_mode`, но реальное применение
  на `dialogue_node` зависит от W3-1/W3-2 (супервизор вне `monitor`).
