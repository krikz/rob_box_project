# ADR-0121: `_LazyHailoInit` mixin для дублирующейся ленивой инициализации Hailo loader'ов

- Status: Proposed
- Date: 2026-09-16
- Deciders: architect
- Relates: [ADR-0089 §2.1 Phase 2](../0089-vision-pipeline.md) (issue #2657)
- Touches: `src/rob_box_perception/rob_box_perception/vision_hailo_loader.py`,
  `src/rob_box_perception/rob_box_perception/vision_face_loader.py`

## Контекст

Lazy-init pattern (`_init_failed: Optional[BaseException]` + `_ensure_initialized` +
`_init_locked`) скопирован 1:1 из `RealHEFLoader` в `RetinaFaceLoader` (issue #2657).

**Объём дубля (raw evidence):**

| Шаблон                                   | `vision_hailo_loader.py` | `vision_face_loader.py` |
| ---------------------------------------- | ------------------------ | ----------------------- |
| `self._init_failed: Optional[BaseException] = None` | L258 | L388 |
| `_ensure_initialized()` body             | L264–296 (~33 строки)    | L394–403 (~10 строк)    |
| `_init_locked()` (без HEF-специфики)     | L298–379 (~82 строки)    | L405–468 (~64 строки)   |

Из ~82 строк `_init_locked` **совпадают побайтово** ~58 строк (VDevice create_params +
ROUND_ROBIN fallback + input/output buffers + `create_bindings` + activate/last_output).
Различия только в:

1. `self._output_name = self._infer_model.output_names[0]` vs
   `self._output_names = list(self._infer_model.output_names)` (single vs multi output).
2. Сообщение `FileNotFoundError` (yolov8 vs retinaface HEF).
3. (Опционально) `_do_init` у RetinaFaceLoader должен уметь **не** звать `activate()`,
   потому что RetinaFace HEF (single-batch inference, no-async-pipeline в PoC-варианте)
   не требует activate. Сейчас код RetinaFace **не** вызывает `activate()` — это
   часть контракта миксина.

**Регресс, который дефект уже причинил:** commit `f54b6adc` ("fix(vision #2599):
шов «Ускоритель» + SHA-регистр") вынужденно правил ОБА файла одним и тем же патчем.
Если бы общий код жил в одном месте, fix внёсся бы в одну строку.

## Решение

Ввести **mixin `_LazyHailoInit`** в `vision_hailo_loader.py` (где уже живёт
"настоящий" loader — естественное место для общего базового класса). Mixin:

```python
class _LazyHailoInit:
    """Lazy HailoRT init + fail-fast caching.

    Требования к subclass'у:
      - поле `_configured: Any` (None до init, non-None после).
      - поле `_infer_model: Any` (sub-init use).
      - метод `_do_init() -> None`, реализующий конкретную логику
        открытия VDevice / configure / create_bindings для данной HEF.
        Может поднять ImportError / FileNotFoundError / RuntimeError —
        mixin их закеширует и re-raise'нет на повторные вызовы.
    """

    _init_failed: Optional[BaseException]

    def _ensure_initialized(self) -> None:
        if self._configured is not None:
            return
        if self._init_failed is not None:
            raise self._init_failed
        try:
            self._do_init()
        except BaseException as exc:  # noqa: BLE001 (capability-honest)
            self._init_failed = exc
            raise

    def _do_init(self) -> None:  # pragma: no cover - abstract
        raise NotImplementedError(
            f'{type(self).__name__} обязан реализовать _do_init()'
        )
```

### Subclass-обязательства

`RealHEFLoader(..., _LazyHailoInit)`:

- удаляет своё `_init_failed`, `_ensure_initialized`, `_init_locked`.
- реализует `_do_init() -> None` с конкретной логикой YOLOv8n HEF
  (включая `self._configured.activate()` и
  `self._output_name = self._infer_model.output_names[0]`).

`RetinaFaceLoader(..., _LazyHailoInit)`:

- удаляет своё `_init_failed`, `_ensure_initialized`, `_init_locked`.
- реализует `_do_init() -> None` с конкретной логикой retinaface HEF
  (БЕЗ `activate()` для PoC-варианта, и с
  `self._output_names = list(self._infer_model.output_names)`).

### Альтернативы, отвергнутые

1. **`ABC`/`Generic`**: тяжелее mixin'а, запрещает множественное наследование
   (а subclass'ы могут в будущем унаследоваться от чего-то ещё, например от
   общего `BaseVisionLoader` в Phase 3). Mixin — минимальный контракт.
2. **Composition (`has-a`) через wrapping loader'а**: усложняет ownership полей
   (`_vdevice`, `_infer_model`...) — больше boilerplate, чем само сокращение.
3. **Копипастить, но документировать дубль в комментарии**: уже было — регресс
   `f54b6adc` показал, что комментарии не спасают, общий код нужен.
4. **Вынести `_LazyHailoInit` в отдельный модуль** (`vision_hailo_mixin.py`):
   преждевременное разделение — пока один файл-потребитель (`vision_hailo_loader.py`)
   и второй — рядом (`vision_face_loader.py`). Разделение оправдано когда
   появится третий loader (ArcFace, Phase 2 PR-B) — но **тогда** и сделаем
   split; сейчас YAGNI.

## Trade-offs

- **Pro:** −~58 строк дубля (миксин ~20 строк + 2 subclass'а по ~10 строк
  переименований), следующий loader (ArcFace, PR-B) добавляется через
  тривиальную реализацию `_do_init`, fix `f54b6adc`-style правится один раз.
- **Pro:** контракт суб-класса (`_configured`, `_do_init`) — минимальный и
  стабильный; mixin не знает о HEF-специфике (yolov8 vs retinaface vs arcface).
- **Pro:** `noqa: BLE001` остаётся в **одном** месте — проще аудитить.
- **Con:** +1 уровень косвенности (subclass → mixin → ABC object). Для
  двух loader'ов это не критично, но при добавлении 4+ стоит пересмотреть
  в сторону `BaseVisionLoader` ABC с `Template Method`.
- **Con:** mixin не сможет гарантировать, что subclass объявил `_configured`
  (Type hint есть, runtime-проверки нет). Митигация: добавить тест, что
  `RealHEFLoader()._ensure_initialized()` падает `NotImplementedError` если
  subclass забыл переопределить `_do_init` — но **после** merge'а в develop.

## Acceptance (для implementer'а)

- [ ] В `vision_hailo_loader.py` объявлен `class _LazyHailoInit:` с
      полем `_init_failed` и методом `_ensure_initialized`. Метод `_do_init`
      объявлен с `raise NotImplementedError`.
- [ ] `RealHEFLoader` и `RetinaFaceLoader`:
      - унаследованы от `_LazyHailoInit` (порядок MRO: subclass первым,
        mixin последним — `class RealHEFLoader(..., _LazyHailoInit):`);
      - **не** содержат собственных `_ensure_initialized` / `_init_failed`
        / `_init_locked`;
      - реализуют `_do_init()` (а не переопределяют `_init_locked`).
- [ ] `grep -nE 'self\._init_failed|_ensure_initialized|_init_locked' src/rob_box_perception/rob_box_perception/vision_*_loader.py`
      даёт ровно **одно** определение `_ensure_initialized` (в `_LazyHailoInit`)
      и **ноль** определений `_init_locked`.
- [ ] `pytest src/rob_box_perception/test/unit/test_vision_hailo_phase15.py -v`
      — все 10 тестов Phase 1.5 зелёные (loaders всё ещё импортируются,
      `_do_init` остаётся приватным, `infer(image=None)` → [] продолжает работать).
- [ ] Никаких изменений в публичном API `RealHEFLoader.infer` /
      `RetinaFaceLoader.infer` / `make_loader` — только внутренний refactor.

## План реализации (для implementer'а)

1. В `vision_hailo_loader.py` **перед** классом `RealHEFLoader` объявить
   `class _LazyHailoInit:` (по образцу из этого ADR).
2. В `RealHEFLoader.__init__` убрать `self._init_failed = None` (mixin инициализирует
   через class-level аннотацию). Убедиться, что в `__init__` mixin'а или class-body
   стоит `_init_failed: Optional[BaseException] = None`.
3. В `RealHEFLoader` удалить методы `_ensure_initialized` и `_init_locked`.
   Переименовать тело `_init_locked` → `_do_init` (без изменений).
4. В `RetinaFaceLoader.__init__` убрать `self._init_failed = None`,
   добавить mixin в базы. Удалить `_ensure_initialized` и `_init_locked`,
   переименовать тело `_init_locked` → `_do_init`.
5. Прогнать unit-тесты `test_vision_hailo_phase15.py` + `test_vision_hailo_node.py`.
6. Коммит одним PR (atomic refactor, без behaviour change).

## Что НЕ делаем

- Не выносим `_LazyHailoInit` в отдельный модуль (см. альтернативу 4).
- Не делаем `BaseVisionLoader` ABC (Phase 3+, когда появится 3-й loader).
- Не трогаем `make_loader` / публичный API / тесты, которые не падают.
- Не правим privacy-issue (`self._init_failed` хранит `BaseException`, не PII — OK).
