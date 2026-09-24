#!/usr/bin/env python3
"""Узнавание лица — склейка детектора, эмбеддера, трекера и хранилища.

Issue #2599 PR-B, ADR-0123 §3/§4/§6.

Детектор (``vision_face_loader.RetinaFaceLoader``) отдаёт bbox'ы — по
~5 штук в секунду на одно лицо (живой замер 16.09.2026: 88 событий
``face`` за минуту от одного человека). Этот модуль превращает поток
кадровых детекций в осмысленные ответы на два разных вопроса:

1. **«Кто сейчас в кадре?»** — на КАЖДОМ кадре, дёшево: эмбеддинг лица
   сравнивается с галереями в :class:`FaceStore` (косинус в памяти, без
   диска). Результат едет в ``VisionEvent.embedding_id`` /
   ``display_name``, чтобы Личность видела имя, а не «face-блоб».

2. **«Кто-то ПОЯВИЛСЯ»** — редко, по правилу Встречи (ADR-0123 §3):
   трек живёт ``min_track_sec`` и лицо хоть раз было крупнее
   ``min_face_px``. Только в этот момент пишется запись в хранилище и
   поднимается маркер ``encounter=start`` в ``attributes_json`` — повод
   заговорить (ADR-0102, ``Occasion(kind="meeting")``).

Почему два вопроса, а не один: если публиковать повод на каждом кадре,
робот будет здороваться пять раз в секунду; если узнавать только в
момент Встречи, имя пропадёт из контекста через кадр после неё.

**Разделение ответственности.** Здесь нет ни ROS, ни HailoRT, ни
файловой системы — только оркестрация. Железо живёт в
``face_embedding``, диск и режимы приватности — в ``face_store``
(ADR-0123 §5: режим проверяется в одном месте, в хранилище), правило
Встречи — в ``face_tracker``. Поэтому модуль тестируется подставными
объектами, без камеры и без Hailo.

**Выдумка не доезжает до Личности** (ADR-0089 §2.2, #2583): в stub-режиме
детектор публикует ``event_type="stub"``, а не ``"face"``, и сюда такие
события просто не попадают — узнавать нечего, записи не создаются.

**Ворота качества кропа** (issue #2749): ``confidence_threshold`` и
``min_face_px`` проверяют только геометрию детекции, а не то, что внутри
бокса. RetinaFace может полчаса честно детектировать тень в тёмной
комнате — бокс большой, confidence высокий, а кроп почти чёрный; ArcFace
на таком входе не падает, а стабильно отдаёт один и тот же вырожденный
эмбеддинг, который ложится в галерею как "человек". Поэтому здесь же, в
:meth:`FaceRecognizer._crop_all`, кроп проверяется на яркость/контраст
(:func:`~rob_box_perception.face_embedding.crop_brightness_contrast`,
``DEFAULT_MIN_CROP_MEAN``/``DEFAULT_MIN_CROP_CONTRAST``) ДО эмбеддинга —
отбракованный кроп трактуется как отсутствующий (тот же путь, что и
мелкое лицо ниже ``min_embed_px``): не эмбеддится, не идёт в снимок,
трек без единого годного кадра просто не даёт эмбеддинга на Встрече
(см. ``_on_encounter``) и запись в ``FaceStore`` не создаётся. Это
согласуется с границей ``FaceStore`` (ADR-0123 §5: хранилище снимки
принципиально не разглядывает) — фильтр стоит строго ВЫШЕ неё, пока
кроп ещё живой массив, а не готовые JPEG-байты.

**Ворота качества кропа, часть 2: геометрия и резкость** (issue #2774).
Яркость/контраст ловят «кроп есть, но там ничего не видно» (тень,
пересвет). Они НЕ ловят другой, не менее опасный отказ: кроп ЕСТЬ, он
яркий и контрастный, но геометрически обрублен. Живой пример на
роботе (issue #2774) — запись ``b49470e1-...``, встреча №19: бокс
упёрся в верхний край кадра, ``crop_face`` (``face_embedding.py``)
честно склампил его к границе и молча отдал результат — в кадре
остались подбородок, рот и шея, глаз нет вообще. Такой вектор
проходит любые ворота яркости/контраста и работает отмычкой в галерее,
потому что счёт узнавания в ``FaceStore`` — max по галерее (issue
#2774: «один мусорный вектор похож на что угодно чуть больше, чем
надо»). Поэтому здесь же, в :meth:`FaceRecognizer._crop_all`,
пересчитывается — от исходного ``bbox`` и того же ``margin``, что уйдёт
в ``crop_face`` — какая доля ЗАПРОШЕННОГО (bbox+margin) прямоугольника
реально попадает в кадр (:func:`_crop_coverage`, ``DEFAULT_MIN_CROP_
COVERAGE``), и отдельно — не срезана ли ИМЕННО верхняя граница
(``DEFAULT_MAX_TOP_CLIP_FRAC``): верх не равноценен низу и бокам, там
глаза. Мы не можем поправить ``crop_face``, чтобы он сообщал об этом
сам (issue #2773 сейчас параллельно переписывает и его, и декодер
landmarks в ``vision_face_loader``) — поэтому геометрия клампа
продублирована здесь по формуле из докстринга ``crop_face``.

Резкость (:func:`~rob_box_perception.face_embedding.sharpness`)
до этой карточки считалась, но НЕ была воротами — она входила только
слагаемым в скоринг «лучшего кадра трека» (``face_tracker.
_observation_score``, ``SCORE_WEIGHT_SHARPNESS``). Смазанный, но яркий
и геометрически целый кроп такие ворота раньше проходил свободно.
Теперь та же величина используется дважды, с двумя разными ролями,
явно разведёнными (см. :meth:`FaceRecognizer._pixel_quality_reject_
reason` про ворота и ``face_tracker.py`` про скоринг): здесь —
порог «пускать/не пускать» (``DEFAULT_MIN_SHARPNESS``), там — не
изменившееся слагаемое выбора лучшего из УЖЕ пропущенных кадров.

Все причины отказа (``dark``/``flat``/``clipped``/``blurry``) считаются
в один словарь и отдаются в :meth:`stats` — issue #2749 и #2774 прямо
требуют, чтобы этот фильтр был видимым, а не вторым тихим источником
«почему робот меня не видит».

**Выравнивание по landmark'ам** (issue #2773, соседняя карточка).
Когда детектор отдаёт 5 точек лица (``det['landmarks']`` — плоский
список из 10 float, normalized [0,1] в координатах кадра, порядок
left_eye/right_eye/nose/mouth_left/mouth_right; либо ``None``, если
точек нет), :meth:`FaceRecognizer._crop_all` пробует выровнять лицо по
шаблону ArcFace через ``face_embedding.align_face`` — канонический
similarity-transform вместо простого прямоугольного кропа с
анизотропным ресайзом (issue #2773: без выравнивания внутриперсонный
разброс эмбеддингов 0.3–0.85, порог узнавания пришлось опустить до
0.45, отсюда и спутанные имена). Если landmarks нет, ``align_face``
недоступна (модуль face_embedding у соседнего агента ещё не готов —
импорт устойчив к этому, см. ниже) или вернула ``None`` — используется
прежний путь ``crop_face``; частота фолбека считается отдельно и тоже
уходит в :meth:`stats`.

Важно: выровненная 112x112 картинка идёт ТОЛЬКО эмбеддеру. Снимок
встречи, который видят люди в Telegram/Quest (ADR-0123 §8), обязан
остаться человекочитаемым прямоугольным кропом ``crop_face`` — не
квадратом ArcFace. Поэтому с этой карточки на каждую детекцию есть два
разных артефакта: вход эмбеддера (``embed_crops`` в ``_crop_all``) и
снимок встречи (``snapshot_crops``, из которого же считаются и ворота
качества, и итоговый JPEG в :meth:`_on_encounter`).
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, List, Optional, Tuple

from rob_box_perception.face_embedding import (
    crop_brightness_contrast,
    crop_face,
    encode_jpeg,
    sharpness,
)

try:
    # align_face — issue #2773, пишет параллельно другой агент в этом же
    # PR-окне. Импорт устойчив ровно к той же ситуации, для которой
    # face_tracker._cosine_similarity уже держит fallback на face_embedding:
    # на момент запуска ЭТИХ тестов функции может ещё не быть в файле
    # соседа (или файл ещё не импортируется вовсе) — модуль не имеет права
    # упасть на импорте из-за чужой незаконченной работы. Если align_face
    # недоступна, узнавание просто всегда идёт фолбеком на crop_face (см.
    # ``FaceRecognizer._align_or_fallback``), а не падает.
    from rob_box_perception.face_embedding import (  # type: ignore[import-not-found]
        align_face,
    )
except ImportError:
    align_face = None  # type: ignore[assignment]

from rob_box_perception.face_tracker import FaceObservation, FaceTracker

#: Запас вокруг bbox'а лица при кропе (ADR-0123 §4.2: «40 % по краям,
#: видно причёску, очки»). Тот же кроп идёт и в ArcFace, и в снимок —
#: ArcFace ресайзит его до 112x112 сам.
DEFAULT_CROP_MARGIN = 0.4

#: Лица мельче этого порога (короткая сторона, px) не эмбеддятся вовсе.
#: Смысл не в приватности, а в цене: ArcFace на мелком кропе даёт шум,
#: который только пачкает галерею, а NPU тратится на каждом кадре.
DEFAULT_MIN_EMBED_PX = 32.0

#: Порог средней яркости кропа (0..255, см. ``face_embedding.
#: crop_brightness_contrast``) — ворота качества кропа перед эмбеддингом
#: (issue #2749). Ниже него кроп не эмбеддится и не идёт в снимок встречи:
#: трактуется так же, как мелкое лицо (см. ``DEFAULT_MIN_EMBED_PX``) —
#: наблюдение остаётся, эмбеддинга у него нет.
#:
#: Число — живой замер на Vision Pi 22.09.2026 (issue #2749): фантомная
#: запись ``8ffc2641-...`` (тень в тёмной комнате, детектор держал её
#: полчаса, 130 "встреч") — mean 6.4–7.4/255 по 11 снимкам подряд; та же
#: база, живой человек (``b49470e1-...``) — mean 76.6–185.4/255 по 10
#: снимкам. Порог 20 садится с большим запасом ОТ ОБЕИХ границ: втрое
#: выше максимума фантома и вчетверо ниже минимума живого кропа.
DEFAULT_MIN_CROP_MEAN = 20.0

#: Порог контраста кропа (``p95 - p5``, 0..255) — второй, независимый от
#: яркости сигнал ворот качества (issue #2749). Ловит другой отказ, чем
#: ``DEFAULT_MIN_CROP_MEAN``: не "слишком тёмный", а "плоский" кроп —
#: пересвеченная в сплошной белый кадром матрица, где средняя яркость в
#: норме, а разброса нет вовсе. Для замеренного 22.09.2026 фантома
#: (``8ffc2641-...``) контраст 28–31 — ЧУТЬ ВЫШЕ этого порога (``p5``
#: там упирается в шумовой пол матрицы 0, а не в настоящий чёрный, отсюда
#: обманчиво не такой уж маленький разброс); именно эту фантомную запись
#: останавливает ``DEFAULT_MIN_CROP_MEAN``, а не этот порог — контраст
#: остаётся про другой (пока не пойманный вживую) случай отказа. Живой
#: человек в той же базе (``b49470e1-...``) — контраст 170–208.
DEFAULT_MIN_CROP_CONTRAST = 25.0

#: Минимальная доля ЗАПРОШЕННОГО (bbox с запасом ``crop_margin``,
#: сторона умножается на ``1 + 2*margin`` — формула из докстринга
#: ``face_embedding.crop_face``) прямоугольника, которая обязана
#: попасть в кадр, иначе кроп отбраковывается как ``clipped`` (issue
#: #2774). ``crop_face`` клампит бокс к границам кадра молча, ничего не
#: сообщая наверх о том, что и сколько отрезано — правкой этого самого
#: кламп-сообщения занимается issue #2773 (файл сейчас параллельно
#: переписывает другой агент, трогать нельзя), поэтому здесь же, зная
#: тот же bbox/margin/размер кадра, что уйдут в ``crop_face``, геометрия
#: запроса пересчитывается заново (:func:`_crop_coverage`).
#:
#: Живой пример issue #2774: запись ``b49470e1-...``, встреча №19 —
#: бокс упёрся в верхний край кадра, в кропе остались подбородок/рот/шея,
#: глаз нет вообще. Точных чисел кламп-прямоугольника этого случая нет
#: (на диске сохранился только итоговый JPEG, не геометрия запроса), порог
#: 0.85 подобран из здравого смысла запаса: ``DEFAULT_CROP_MARGIN=0.4``
#: делает запрошенный прямоугольник почти вдвое шире самой детекции —
#: потеря больше 15% его площади уже заметно режет именно ту рамку, ради
#: которой margin вообще заводили (причёска/уши, ADR-0123 §4.2), а не
#: только "лишний" запас по краям.
DEFAULT_MIN_CROP_COVERAGE = 0.85

#: Максимальная доля высоты запроса, которую можно срезать ИМЕННО
#: сверху, прежде чем кроп отбраковывается — отдельная, более строгая
#: проверка (issue #2774), а не частный случай ``DEFAULT_MIN_CROP_
#: COVERAGE``. Верх не равноценен низу и бокам: там глаза, и без них
#: эмбеддинг ArcFace бессмыслен, даже если формально ``coverage`` кропа
#: ещё выше 0.85 (крупный запрошенный прямоугольник может потерять
#: сверху немного площади, но ровно ту полосу, где были глаза). Порог
#: заметно строже общего покрытия (5% против допустимых 15% по
#: ``DEFAULT_MIN_CROP_COVERAGE``) — в этой asимметрии весь смысл
#: отдельной проверки.
DEFAULT_MAX_TOP_CLIP_FRAC = 0.05

#: Порог ворот резкости (variance-of-Laplacian, см. ``face_embedding.
#: sharpness``) — issue #2774. У той же величины теперь ДВЕ разные роли,
#: явно разведённые (см. докстринг :meth:`FaceRecognizer.
#: _pixel_quality_reject_reason`): здесь — порог "пускать/не пускать" ДО
#: эмбеддинга, а не слагаемое скоринга "лучшего кадра трека"
#: (``face_tracker._observation_score``, ``SCORE_WEIGHT_SHARPNESS`` —
#: тот код не менялся и не должен).
#:
#: Число подобрано по нормировке ``face_tracker.SCORE_SHARPNESS_NORM``
#: (=500.0; докстринг там же: "резкие кадры веб/USB-камер обычно дают
#: значения в районе сотен-полутора тысяч", то есть sharpness_term
#: резкого кадра там близок к 1.0). Порог ворот здесь — на порядок ниже
#: этой нормировки (500/10): заведомо ниже вклада сколько-нибудь резкого
#: кадра в скоринг, но далеко не ноль, который даёт статичный/сильно
#: смазанный кроп — некалиброванных живых чисел смазанного кропа с
#: issue #2774 нет, порог не претендует на точность, только на то, чтобы
#: не пропускать явную мазню.
DEFAULT_MIN_SHARPNESS = 50.0

#: Окно, внутри которого голосовое опознание считается относящимся к
#: лицу в кадре (ADR-0123 §6 «в окне встречи»). Шире — и робот привяжет
#: имя к тому, кто зашёл следом.
DEFAULT_VOICE_MERGE_WINDOW_SEC = 6.0

#: Сколько лиц эмбеддить за один кадр.
#:
#: Бюджет замерен на Vision Pi 22.09.2026, оба HEF в ОДНОМ процессе
#: (RetinaFace + ArcFace через шов «Ускоритель», как и работает нода):
#:
#:   * первый прогон — ~950 мс детекция и ~150 мс эмбеддинг: это
#:     cold-start, configure network group;
#:   * дальше, в установившемся режиме — детекция 120–150 мс,
#:     **эмбеддинг 8–25 мс** на лицо.
#:
#: Тик ноды идёт раз в 0.5 с (``vision_hailo_node``:
#: ``max(0.1, stub_period_sec / 4)``), детекция съедает ~140 мс, так что
#: на эмбеддинги остаётся ~350 мс — это десяток лиц, а не два.
#:
#: Ограничение поэтому не про NPU, а про здравый смысл: в кадре редко
#: бывает больше нескольких лиц, которые нас интересуют, а хвост мелких
#: на заднем плане только пачкает галерею. Эмбеддим САМЫЕ КРУПНЫЕ —
#: у них выше шанс дать годный вектор (ADR-0123 §3 выбирает на Встречу
#: кадры покрупнее), остальные ждут следующего кадра.
#:
#: Если поток лиц окажется плотнее — смотреть ``embed_ms_avg`` и
#: ``embed_skipped_budget`` в :meth:`FaceRecognizer.stats`.
DEFAULT_MAX_EMBEDS_PER_FRAME = 4


class FaceRecognizer:
    """Оркестратор узнавания: детекции → Встреча → имя.

    Args:
        embedder: :class:`~rob_box_perception.face_embedding.ArcFaceEmbedder`
            или совместимый объект с ``embed(crops) -> list``. ``None`` —
            узнавание выключено, модуль работает как no-op (детекция
            продолжает публиковаться, но без имён).
        store: :class:`~rob_box_perception.face_store.FaceStore`.
        tracker: :class:`~rob_box_perception.face_tracker.FaceTracker`.
        crop_margin: запас вокруг bbox'а (см. ``DEFAULT_CROP_MARGIN``).
        min_embed_px: ниже этого размера лицо не эмбеддится.
        min_crop_mean: ворота качества кропа — порог средней яркости
            (issue #2749, см. ``DEFAULT_MIN_CROP_MEAN``).
        min_crop_contrast: ворота качества кропа — порог контраста
            ``p95-p5`` (issue #2749, см. ``DEFAULT_MIN_CROP_CONTRAST``).
        min_crop_coverage: ворота качества кропа — минимальная доля
            запрошенного (bbox+margin) прямоугольника, попавшая в кадр
            (issue #2774, см. ``DEFAULT_MIN_CROP_COVERAGE``).
        max_top_clip_frac: ворота качества кропа — максимальная доля
            высоты, срезанная именно сверху (issue #2774, см.
            ``DEFAULT_MAX_TOP_CLIP_FRAC``).
        min_sharpness: ворота качества кропа — порог резкости
            (variance-of-Laplacian); НЕ путать со слагаемым скоринга
            лучшего кадра трека в ``face_tracker`` (issue #2774, см.
            ``DEFAULT_MIN_SHARPNESS``).
        voice_merge_window_sec: окно слияния с голосом (ADR-0123 §6).
        store_snapshots: писать ли снимок встречи. Решение «лечь ли ему
            на диск» всё равно принимает ``FaceStore`` по режиму — здесь
            только экономия на JPEG-кодировании в ``strict``.
        log_fn: ``callable(level: str, msg: str)`` для логов ноды.
    """

    def __init__(
        self,
        *,
        embedder: Any,
        store: Any,
        tracker: Optional[FaceTracker] = None,
        crop_margin: float = DEFAULT_CROP_MARGIN,
        min_embed_px: float = DEFAULT_MIN_EMBED_PX,
        min_crop_mean: float = DEFAULT_MIN_CROP_MEAN,
        min_crop_contrast: float = DEFAULT_MIN_CROP_CONTRAST,
        min_crop_coverage: float = DEFAULT_MIN_CROP_COVERAGE,
        max_top_clip_frac: float = DEFAULT_MAX_TOP_CLIP_FRAC,
        min_sharpness: float = DEFAULT_MIN_SHARPNESS,
        voice_merge_window_sec: float = DEFAULT_VOICE_MERGE_WINDOW_SEC,
        max_embeds_per_frame: int = DEFAULT_MAX_EMBEDS_PER_FRAME,
        store_snapshots: bool = True,
        log_fn: Optional[Any] = None,
    ) -> None:
        self._embedder = embedder
        self._store = store
        self._tracker = tracker if tracker is not None else FaceTracker()
        self._crop_margin = float(crop_margin)
        self._min_embed_px = float(min_embed_px)
        self._min_crop_mean = float(min_crop_mean)
        self._min_crop_contrast = float(min_crop_contrast)
        self._min_crop_coverage = float(min_crop_coverage)
        self._max_top_clip_frac = float(max_top_clip_frac)
        self._min_sharpness = float(min_sharpness)
        self._voice_merge_window_sec = float(voice_merge_window_sec)
        self._max_embeds_per_frame = max(1, int(max_embeds_per_frame))
        self._store_snapshots = bool(store_snapshots)
        self._log_fn = log_fn

        # Последний кадр — для правила ADR-0123 §6 «голос + РОВНО ОДНО
        # лицо в кадре». Без этого в галерею Дениса однажды попадёт лицо
        # того, кто стоял рядом.
        self._last_frame_ts: float = 0.0
        self._last_frame_person_ids: List[Optional[str]] = []

        # Счётчики для ``stats()`` и для честного health (ADR-0018).
        self._encounters_total = 0
        self._embed_failures = 0
        self._recognized_total = 0
        self._new_people_total = 0
        self._voice_merges_total = 0
        self._embed_calls = 0
        self._embed_ms_total = 0.0
        self._embed_skipped_budget = 0
        #: Кропы, отброшенные воротами качества (issue #2749, #2774) — не
        #: дошли даже до эмбеддера. Отдельно от ``embed_failures`` (там
        #: ArcFace/HailoRT реально падает). ``_crop_rejected_total`` —
        #: сумма по всем причинам, ``_crop_rejected_by_reason`` — та же
        #: сумма с разбивкой (issue #2774: «тихий фильтр обязан быть
        #: видимым» — тот же аргумент, что issue #2748 уже применило к
        #: причинам пропуска слияния с голосом ниже).
        self._crop_rejected_total = 0
        self._crop_rejected_by_reason: Dict[str, int] = {
            'dark': 0,      # средняя яркость ниже min_crop_mean (issue #2749)
            'flat': 0,      # контраст p95-p5 ниже min_crop_contrast (issue #2749)
            'clipped': 0,   # coverage/top-clip ниже порога (issue #2774)
            'blurry': 0,    # sharpness ниже min_sharpness (issue #2774)
        }

        # Выравнивание по landmark'ам (issue #2773) — как часто эмбеддер
        # реально получил выровненный 112x112 вход, и как часто пришлось
        # откатиться на старый путь crop_face (нет landmarks, align_face
        # недоступна/ещё не приземлилась у соседа, либо вернула None).
        self._align_used_total = 0
        self._align_fallback_total = 0

        # Issue #2748 — «слияний=0» само по себе не говорит, ПОЧЕМУ: некого
        # было сливать (голос не опознан) или не с чем (в кадре не одно
        # лицо). Раздельные счётчики причин пропуска note_voice_identification
        # — см. докстринг метода и periodic-сводку vision_face_node._log_stats.
        self._voice_merge_skip_no_face = 0
        self._voice_merge_skip_multi_face = 0
        self._voice_merge_skip_stale = 0
        self._voice_merge_skip_conflict = 0

    # ------------------------------------------------------------------
    # Логирование
    # ------------------------------------------------------------------

    def _log(self, level: str, msg: str) -> None:
        if self._log_fn is None:
            return
        try:
            self._log_fn(level, msg)
        except Exception:  # noqa: BLE001 — лог не имеет права ронять узнавание
            pass

    # ------------------------------------------------------------------
    # Основной вход
    # ------------------------------------------------------------------

    def process(
        self,
        detections: List[Dict[str, Any]],
        image: Any,
        now: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """Обогатить детекции именами и поднять маркер Встречи.

        Мутирует и возвращает те же словари, что отдал детектор
        (``vision_face_loader`` уже заполнил в них bbox/confidence) —
        нода публикует их без разбора, кто и что туда дописал.

        Args:
            detections: список dict'ов VisionEvent-полей от детектора.
            image: RGB uint8 кадр, на котором эти детекции найдены.
            now: время для тестов; ``None`` → ``time.monotonic()``.

        Returns:
            Тот же список. Поля ``embedding_id`` / ``display_name`` /
            ``attributes_json`` заполнены там, где узнавание получилось.
        """
        ts = time.monotonic() if now is None else float(now)

        if not detections or image is None or self._embedder is None:
            # Нет кадра или узнавание выключено — треки всё равно надо
            # состарить, иначе «ушедший» человек останется живым треком
            # и следующая Встреча не поднимется.
            self._tracker.expire(ts)
            self._last_frame_ts = ts
            self._last_frame_person_ids = []
            return detections

        frame_h, frame_w = int(image.shape[0]), int(image.shape[1])

        # Два разных артефакта на детекцию с #2773/#2774: embed_crops —
        # вход эмбеддера (выровненный по landmarks 112x112, если получилось,
        # иначе фолбек-кроп crop_face), snapshot_crops — человекочитаемый
        # прямоугольный кроп crop_face, который и только который идёт в
        # снимок встречи (ADR-0123 §8) и в скоринг "лучшего кадра трека".
        embed_crops, snapshot_crops, face_px_list = self._crop_all(
            detections, image, frame_w, frame_h
        )
        embeddings = self._embed_all(embed_crops)

        observations = self._make_observations(
            detections, snapshot_crops, embeddings, face_px_list, frame_w, frame_h, ts
        )

        # 1. Кто в кадре — на каждом кадре, только чтение.
        person_ids = self._annotate_identities(detections, embeddings)
        self._last_frame_ts = ts
        self._last_frame_person_ids = person_ids

        # 2. Кто появился — редко, по правилу Встречи.
        promoted = self._tracker.update(observations, ts)
        self._tracker.expire(ts)
        for encounter in promoted:
            self._on_encounter(encounter, detections, snapshot_crops)

        return detections

    # ------------------------------------------------------------------
    # Шаги конвейера
    # ------------------------------------------------------------------

    def _crop_all(
        self,
        detections: List[Dict[str, Any]],
        image: Any,
        frame_w: int,
        frame_h: int,
    ) -> Tuple[List[Any], List[Any], List[float]]:
        """Два артефакта на детекцию + размер лица в пикселях (issue #2773/#2774).

        Returns:
            ``(embed_crops, snapshot_crops, face_px_list)``:
              - ``embed_crops`` — вход эмбеддера: выровненный по
                landmarks кроп (``face_embedding.align_face``), либо,
                если выравнивание не сработало/недоступно, тот же кроп,
                что и ``snapshot_crops`` (фолбек, issue #2773).
              - ``snapshot_crops`` — человекочитаемый прямоугольный кроп
                ``crop_face`` с запасом; именно он, и только он, уходит в
                снимок встречи (ADR-0123 §8) и в скоринг «лучшего кадра
                трека» (``face_tracker``). Ворота качества (яркость/
                контраст/резкость/геометрия) тоже считаются на НЁМ —
                это единственный артефакт, который гарантированно
                существует независимо от того, есть ли landmarks.
            Оба списка той же длины и с ``None`` на тех же позициях, что
            и раньше: budget-пропуск (``DEFAULT_MAX_EMBEDS_PER_FRAME``)
            или отказ ворот качества трактуются как отсутствующий кроп.
        """
        bboxes: List[Any] = []
        face_px_list: List[float] = []
        for det in detections:
            bbox = (
                float(det.get('bbox_cx', 0.0)),
                float(det.get('bbox_cy', 0.0)),
                float(det.get('bbox_w', 0.0)),
                float(det.get('bbox_h', 0.0)),
            )
            bboxes.append(bbox)
            # Короткая сторона лица в пикселях — по ней решается и
            # «эмбеддить ли», и «годится ли на Встречу» (ADR-0123 §3).
            face_px_list.append(float(min(bbox[2] * frame_w, bbox[3] * frame_h)))

        # Бюджет NPU: кропим (а значит и эмбеддим) только N самых крупных
        # лиц — см. DEFAULT_MAX_EMBEDS_PER_FRAME. Остальные в этом кадре
        # останутся без эмбеддинга; их треки живут дальше и получат его на
        # следующем кадре, когда порядок по размеру может смениться.
        eligible = [
            i for i, px in enumerate(face_px_list) if px >= self._min_embed_px
        ]
        eligible.sort(key=lambda i: face_px_list[i], reverse=True)
        chosen = set(eligible[: self._max_embeds_per_frame])
        if len(eligible) > len(chosen):
            self._embed_skipped_budget += len(eligible) - len(chosen)

        embed_crops: List[Any] = []
        snapshot_crops: List[Any] = []
        for idx, bbox in enumerate(bboxes):
            if idx not in chosen:
                embed_crops.append(None)
                snapshot_crops.append(None)
                continue

            # Ворота геометрии (issue #2774) — ДО вызова crop_face: если
            # запрошенный прямоугольник заведомо обрублен кадром, нет
            # смысла даже резать пиксели.
            coverage, top_clip_frac = _crop_coverage(
                bbox, self._crop_margin, frame_w, frame_h
            )
            if (
                coverage < self._min_crop_coverage
                or top_clip_frac > self._max_top_clip_frac
            ):
                self._reject_crop('clipped')
                embed_crops.append(None)
                snapshot_crops.append(None)
                continue

            snap = crop_face(image, bbox, margin=self._crop_margin)
            if snap is None:
                # Геометрия формально прошла, но crop_face всё равно не
                # дал кропа (округление на границе пикселя) — тот же
                # отказ по смыслу, что и явный клам выше.
                self._reject_crop('clipped')
                embed_crops.append(None)
                snapshot_crops.append(None)
                continue

            reason = self._pixel_quality_reject_reason(snap)
            if reason is not None:
                # Ворота качества (issue #2749/#2774): кроп живой, но
                # почти чёрный/плоский/смазанный — не эмбеддим и не
                # кладём в снимок, трактуем как отсутствующий кроп (тот
                # же путь, что и бюджетный/геометрический пропуск выше).
                self._reject_crop(reason)
                embed_crops.append(None)
                snapshot_crops.append(None)
                continue

            snapshot_crops.append(snap)
            landmarks = detections[idx].get('landmarks')
            embed_crops.append(self._align_or_fallback(image, landmarks, snap))

        return embed_crops, snapshot_crops, face_px_list

    def _reject_crop(self, reason: str) -> None:
        """Счётчики отказа ворот качества кропа — сумма + причина (issue #2774)."""
        self._crop_rejected_total += 1
        self._crop_rejected_by_reason[reason] = (
            self._crop_rejected_by_reason.get(reason, 0) + 1
        )

    def _pixel_quality_reject_reason(self, crop: Any) -> Optional[str]:
        """Ворота качества кропа перед эмбеддингом: пиксельная часть (issue #2749/#2774).

        Фильтровать нужно ЗДЕСЬ, пока кроп ещё живой numpy-массив:
        ``FaceStore`` ниже по стеку принципиально не разглядывает снимки
        (ADR-0123 §5, см. его докстринг) — граница режимов приватности
        проведена там намеренно, и нарушать её нельзя; а после кодирования
        в JPEG для записи встречи уже поздно — пиксели надо смотреть
        раньше.

        Три независимых проверки одного и того же кропа, каждая ловит
        свой отказ:
          - ``mean < min_crop_mean`` -> ``'dark'`` (issue #2749: тень,
            забитая экспозиция);
          - ``contrast < min_crop_contrast`` -> ``'flat'`` (issue #2749:
            пересвет в сплошной кадр);
          - ``sharpness < min_sharpness`` -> ``'blurry'`` (issue #2774:
            смазанный кадр — раньше эта же величина участвовала только в
            скоринге "лучшего кадра трека" в ``face_tracker``, здесь у
            неё ДРУГАЯ роль — порог "пускать/не пускать" ДО эмбеддинга;
            обе роли используют один и тот же ``face_embedding.
            sharpness()``, но не путают друг друга).

        ``crop_brightness_contrast`` и ``sharpness`` используют один и
        тот же ленивый импорт ``cv2`` (см. их докстринги в
        ``face_embedding.py``) — то есть если недоступен один, недоступен
        и другой. Поэтому единственный сигнал "cv2 нет" — ``None`` от
        ``crop_brightness_contrast`` — открывает ВСЕ три ворот сразу, а
        не только яркость/контраст: пробовать ``sharpness`` отдельным
        вызовом при недоступном cv2 бессмысленно, она в этом случае
        всегда возвращает ``0.0`` (не ``None``!) и ложно провалила бы
        ворота резкости на КАЖДОМ кропе в окружении без cv2 — тогда как
        деградировать до "не эмбеддим вообще" хуже, чем пропустить один
        некалиброванный кроп через ворота (тот же аргумент, что и в
        #2749).

        Returns:
            ``None``, если кроп прошёл все ворота (или метрики посчитать
            не удалось — cv2 недоступен); иначе строка причины отказа.
        """
        metrics = crop_brightness_contrast(crop)
        if metrics is None:
            return None
        mean, contrast = metrics
        if mean < self._min_crop_mean:
            return 'dark'
        if contrast < self._min_crop_contrast:
            return 'flat'
        if sharpness(crop) < self._min_sharpness:
            return 'blurry'
        return None

    def _align_or_fallback(
        self,
        image: Any,
        landmarks: Optional[List[float]],
        fallback_crop: Any,
    ) -> Any:
        """Выровненный вход эмбеддера по landmarks, либо фолбек на ``crop_face`` (issue #2773).

        ``align_face`` может отсутствовать (соседний агент ещё не
        дописал ``face_embedding.py`` — см. импорт в начале модуля) или
        вернуть ``None`` (битые/отсутствующие точки, cv2 недоступен —
        контракт ``align_face``). Любой из этих случаев — фолбек на
        прежний путь: тот же ``fallback_crop`` (``crop_face``), что и так
        уже посчитан и прошёл ворота качества, идёт эмбеддеру напрямую
        (``ArcFaceEmbedder.embed`` сам приводит его к 112x112).

        Исключение из самого ``align_face`` ловится здесь же и не
        поднимается выше: один кадр с битыми landmarks не должен ронять
        узнавание остальных лиц кадра (тот же принцип capability-honest,
        что и у ``_embed_all`` для сбоя ArcFace, но на уровень раньше).
        """
        if landmarks is not None and align_face is not None:
            try:
                aligned = align_face(image, landmarks)
            except Exception as exc:  # noqa: BLE001 — один плохой набор точек не должен ронять кадр
                aligned = None
                self._log(
                    'warn',
                    f'align_face упал на landmarks: {exc!r} — фолбек на '
                    f'crop_face для этой детекции (issue #2773).',
                )
            if aligned is not None:
                self._align_used_total += 1
                return aligned

        self._align_fallback_total += 1
        return fallback_crop

    def _embed_all(self, crops: List[Any]) -> List[Any]:
        """Один батч в ArcFace на кадр, а не по вызову на лицо."""
        wanted = [c for c in crops if c is not None]
        if not wanted:
            return [None] * len(crops)
        started = time.monotonic()
        try:
            vectors = self._embedder.embed(wanted)
        except Exception as exc:  # noqa: BLE001
            # Capability-honest (ADR-0018): не выдумываем эмбеддинги.
            # Детекция продолжает публиковаться — лицо видно, имени нет.
            self._embed_failures += 1
            if self._embed_failures <= 3:
                self._log(
                    'error',
                    f'ArcFace embed failed: {exc!r}. Лица без имён '
                    f'(попытка {self._embed_failures}).',
                )
            return [None] * len(crops)

        # Замер стоимости ArcFace на живом потоке: бюджет тика — 0.5 с,
        # и если среднее подберётся к нему, надо резать
        # max_embeds_per_frame (или эмбеддить не каждый кадр).
        elapsed_ms = (time.monotonic() - started) * 1000.0 / max(1, len(wanted))
        self._embed_calls += len(wanted)
        self._embed_ms_total += elapsed_ms * len(wanted)

        out: List[Any] = []
        it = iter(vectors)
        for crop in crops:
            out.append(next(it, None) if crop is not None else None)
        return out

    def _make_observations(
        self,
        detections: List[Dict[str, Any]],
        snapshot_crops: List[Any],
        embeddings: List[Any],
        face_px_list: List[float],
        frame_w: int,
        frame_h: int,
        ts: float,
    ) -> List[FaceObservation]:
        """Построить наблюдения трекера — ``crop`` здесь ВСЕГДА snapshot (crop_face), не вход эмбеддера.

        Это то, что уйдёт в JPEG снимка встречи (ADR-0123 §8) и в скоринг
        «лучшего кадра трека» в ``face_tracker`` — сознательно не
        выровненный 112x112 квадрат, который мог получить эмбеддер
        (issue #2773/#2774, см. докстринг ``_crop_all``).
        """
        observations: List[FaceObservation] = []
        for idx, det in enumerate(detections):
            crop = snapshot_crops[idx]
            observations.append(
                FaceObservation(
                    ts=ts,
                    bbox_cx=float(det.get('bbox_cx', 0.0)),
                    bbox_cy=float(det.get('bbox_cy', 0.0)),
                    bbox_w=float(det.get('bbox_w', 0.0)),
                    bbox_h=float(det.get('bbox_h', 0.0)),
                    confidence=float(det.get('confidence', 0.0)),
                    face_px=face_px_list[idx],
                    embedding=embeddings[idx],
                    sharpness=sharpness(crop) if crop is not None else 0.0,
                    crop=crop,
                    frame_w=frame_w,
                    frame_h=frame_h,
                )
            )
        return observations

    def _annotate_identities(
        self,
        detections: List[Dict[str, Any]],
        embeddings: List[Any],
    ) -> List[Optional[str]]:
        """Заполнить ``embedding_id``/``display_name`` — только чтение.

        Хранилище здесь НЕ пишет: запись — событие уровня Встречи
        (ADR-0123 §3), а не кадра.
        """
        person_ids: List[Optional[str]] = []
        for idx, det in enumerate(detections):
            emb = embeddings[idx]
            if emb is None:
                person_ids.append(None)
                continue
            try:
                match = self._store.identify(emb)
            except Exception as exc:  # noqa: BLE001
                self._log('warn', f'FaceStore.identify failed: {exc!r}')
                person_ids.append(None)
                continue
            if match is None:
                person_ids.append(None)
                continue
            person_ids.append(match.person_id)
            det['embedding_id'] = match.person_id
            det['display_name'] = match.name or ''
        return person_ids

    def _on_encounter(
        self,
        encounter: Any,
        detections: List[Dict[str, Any]],
        snapshot_crops: List[Any],
    ) -> None:
        """Встреча состоялась: записать её и поднять повод заговорить."""
        self._encounters_total += 1

        if encounter.embedding is None:
            # Трек дожил до Встречи, но ни один кадр не дал эмбеддинга
            # (мелкое лицо / ArcFace в degraded). Записывать нечего, и
            # выдумывать «кого-то» мы не будем.
            self._log(
                'warn',
                f'Встреча track={encounter.track_id} без эмбеддинга — '
                f'запись пропущена (лицо мельче {self._min_embed_px:.0f}px '
                f'или ArcFace недоступен).',
            )
            return

        best_crop = encounter.best[0].crop if encounter.best else None
        snapshot = None
        if self._store_snapshots and best_crop is not None:
            snapshot = encode_jpeg(best_crop)

        try:
            match = self._store.record_encounter(
                encounter.embedding,
                snapshot=snapshot,
                meta={
                    'track_id': int(encounter.track_id),
                    'max_face_px': float(encounter.max_face_px),
                    'mean_confidence': float(encounter.mean_confidence),
                    'observation_count': int(encounter.observation_count),
                    'source_camera': str(
                        detections[0].get('source_camera', 'unknown')
                    )
                    if detections
                    else 'unknown',
                },
            )
        except Exception as exc:  # noqa: BLE001
            self._log('error', f'FaceStore.record_encounter failed: {exc!r}')
            return

        if match.is_new:
            self._new_people_total += 1
        else:
            self._recognized_total += 1

        # Второй кандидат и зазор до него (issue #2771). Голосовой
        # speaker_id_node печатает их с самого начала
        # (`best=... second=... gap=...`), лицевой канал — нет, и именно
        # поэтому ложное принятие 22.09.2026 (тёща опознана как «Деньчик»
        # при sim=0.483) в логе ничем не отличалось от настоящего
        # узнавания: одно число без контекста нечем поверить. Зазор —
        # первое, на что смотрят при калибровке порога (ADR-0123 §6).
        runner_up = ''
        if getattr(match, 'runner_up_similarity', None) is not None:
            runner_up = ' second={rid} rsim={rsim:.3f} gap={gap:.3f}'.format(
                rid=(match.runner_up_person_id or '?')[:8],
                rsim=match.runner_up_similarity,
                gap=match.similarity - match.runner_up_similarity,
            )

        # issue #2771: помечаем в самой строке "Встреча", когда FaceStore
        # разрешил неоднозначность "безымянный дубль против именованной
        # записи" в пользу имени (см. FaceMatch.disambiguated,
        # FaceStore._disambiguate) - без этой пометки лог выглядел бы как
        # обычное узнавание, и калибровка disambiguation_gap (issue #2771,
        # sweep ADR-0123 §6) не имела бы по чему считать частоту срабатывания.
        disambig = (
            ' устранена_неоднозначность=True'
            if getattr(match, 'disambiguated', False) else ''
        )

        self._log(
            'info',
            '👤 Встреча: {who} (person={pid} sim={sim:.3f} new={new} '
            'encounters={n} face={px:.0f}px{runner_up}{disambig})'.format(
                who=match.name or 'незнакомец',
                pid=match.person_id[:8],
                sim=match.similarity,
                new=match.is_new,
                n=match.encounter_count,
                px=encounter.max_face_px,
                runner_up=runner_up,
                disambig=disambig,
            ),
        )

        self._mark_encounter_start(encounter, detections, match)

    def _mark_encounter_start(
        self,
        encounter: Any,
        detections: List[Dict[str, Any]],
        match: Any,
    ) -> None:
        """Поставить маркер Встречи на ту детекцию, из которой она выросла.

        Маркер едет в ``attributes_json`` — отдельного топика и msg под
        Встречу нет и не будет (ADR-0105: «ни ноды, ни msg, ни топика»).
        Потребитель (``dialogue_node``) реагирует ИМЕННО на маркер, а не
        на каждое ``event_type="face"``, иначе робот поздоровается пять
        раз в секунду.
        """
        idx = self._best_matching_detection(encounter, detections)
        if idx is None:
            return
        det = detections[idx]
        det['embedding_id'] = match.person_id
        det['display_name'] = match.name or ''
        det['attributes_json'] = json.dumps(
            {
                'encounter': 'start',
                'person_id': match.person_id,
                'name': match.name or '',
                'is_new': bool(match.is_new),
                'similarity': round(float(match.similarity), 4),
                'encounter_count': int(match.encounter_count),
                'face_px': round(float(encounter.max_face_px), 1),
                'privacy_mode': getattr(self._store, 'mode', ''),
            },
            ensure_ascii=False,
        )

    @staticmethod
    def _best_matching_detection(
        encounter: Any,
        detections: List[Dict[str, Any]],
    ) -> Optional[int]:
        """Индекс детекции, ближайшей к лучшему кадру Встречи (по IoU)."""
        if not detections or not encounter.best:
            return None
        ref = encounter.best[0]
        best_idx, best_iou = None, -1.0
        for idx, det in enumerate(detections):
            iou = _iou_cxcywh(
                (ref.bbox_cx, ref.bbox_cy, ref.bbox_w, ref.bbox_h),
                (
                    float(det.get('bbox_cx', 0.0)),
                    float(det.get('bbox_cy', 0.0)),
                    float(det.get('bbox_w', 0.0)),
                    float(det.get('bbox_h', 0.0)),
                ),
            )
            if iou > best_iou:
                best_idx, best_iou = idx, iou
        # Даже при нулевом IoU возвращаем ближайшую: Встреча реальна,
        # и потерять повод из-за сместившегося на кадр bbox'а хуже, чем
        # повесить маркер на соседнюю детекцию того же человека.
        return best_idx

    # ------------------------------------------------------------------
    # Слияние с голосом (ADR-0123 §6)
    # ------------------------------------------------------------------

    def note_voice_identification(
        self,
        *,
        speaker_id: str,
        name: str,
        now: Optional[float] = None,
    ) -> Optional[str]:
        """Привязать имя к лицу, когда голос уверенно опознал человека.

        Правило ADR-0123 §6 — строгое и намеренно: привязка происходит
        ТОЛЬКО если в кадре **ровно одно** лицо. Два и больше — слияния
        нет; это открытый вопрос ADR-0105 §3 п.4, и здесь он не решается.
        Иначе в галерею Дениса однажды попадёт лицо того, кто стоял рядом.

        Issue #2748 — каждый ранний ``return None`` инкрементирует счётчик
        причины (``_voice_merge_skip_*``), чтобы periodic-сводка ноды могла
        сказать не просто «слияний=0», а ПОЧЕМУ: нет свежего кадра / не
        одно лицо в кадре / голос уже привязан к другому лицу.

        Args:
            speaker_id: стабильный биометрический id голоса.
            name: имя из голосового профиля.
            now: время для тестов.

        Returns:
            ``person_id`` лицевой записи, если имя привязано; иначе ``None``.
        """
        ts = time.monotonic() if now is None else float(now)

        if not name or not speaker_id:
            # Голос не опознан (is_known=false) — на этот уровень такой
            # payload вообще не должен доходить (vision_face_node фильтрует
            # по is_known до вызова), поэтому счётчика здесь нет: это
            # программная ошибка вызывающего кода, а не штатный «нечего
            # сливать». См. VisionFaceNode._speaker_unknown_total — ИМЕННО
            # там считается «голос без опознанного имени».
            return None
        if ts - self._last_frame_ts > self._voice_merge_window_sec:
            self._voice_merge_skip_stale += 1
            return None  # голос без свежего кадра — не с чем сливать
        if len(self._last_frame_person_ids) == 0:
            self._voice_merge_skip_no_face += 1
            return None  # никого в кадре — сливать некого
        if len(self._last_frame_person_ids) > 1:
            self._voice_merge_skip_multi_face += 1
            return None  # двое и больше в кадре — см. докстринг (ADR-0105 §3 п.4)
        person_id = self._last_frame_person_ids[0]
        if person_id is None:
            self._voice_merge_skip_no_face += 1
            return None

        # Уже привязан к другому голосу — не перебиваем: разбор дублей
        # голосовых профилей (ADR-0123 §6, карточка §9.1) живёт отдельно.
        try:
            existing = self._store.find_by_speaker(speaker_id)
            if existing is not None and existing != person_id:
                self._voice_merge_skip_conflict += 1
                self._log(
                    'warn',
                    f'Голос {speaker_id[:8]} уже привязан к лицу '
                    f'{existing[:8]}, а в кадре {person_id[:8]} — '
                    f'слияние пропущено (дубли профилей, ADR-0123 §9.1).',
                )
                return None
            ok = self._store.attach_name(person_id, name, speaker_id=speaker_id)
        except Exception as exc:  # noqa: BLE001
            self._log('error', f'FaceStore.attach_name failed: {exc!r}')
            return None

        if not ok:
            return None
        self._voice_merges_total += 1
        self._log(
            'info',
            f'🔗 Лицо {person_id[:8]} = «{name}» (по голосу '
            f'{speaker_id[:8]}, одно лицо в кадре).',
        )
        return person_id

    # ------------------------------------------------------------------
    # Health
    # ------------------------------------------------------------------

    def stats(self) -> Dict[str, Any]:
        """Счётчики для лога ноды и health (ADR-0123 §2: режим обязан быть виден)."""
        store_stats: Dict[str, Any] = {}
        try:
            store_stats = self._store.stats()
        except Exception:  # noqa: BLE001
            pass
        return {
            'encounters_total': self._encounters_total,
            'recognized_total': self._recognized_total,
            'new_people_total': self._new_people_total,
            'voice_merges_total': self._voice_merges_total,
            # Issue #2748 — причины, по которым note_voice_identification()
            # НЕ привязала имя, чтобы «слияний=0» в сводке ноды не было
            # немым нулём (см. VisionFaceNode._log_stats).
            'voice_merge_skip_no_face': self._voice_merge_skip_no_face,
            'voice_merge_skip_multi_face': self._voice_merge_skip_multi_face,
            'voice_merge_skip_stale': self._voice_merge_skip_stale,
            'voice_merge_skip_conflict': self._voice_merge_skip_conflict,
            'embed_failures': self._embed_failures,
            'embed_calls': self._embed_calls,
            'embed_ms_avg': round(
                self._embed_ms_total / self._embed_calls, 1
            ) if self._embed_calls else 0.0,
            'embed_skipped_budget': self._embed_skipped_budget,
            'crop_rejected_total': self._crop_rejected_total,
            # Issue #2774 — разбивка причин, тем же аргументом, что и
            # voice_merge_skip_* выше: «отклонено=N» не говорит, ПОЧЕМУ.
            'crop_rejected_by_reason': dict(self._crop_rejected_by_reason),
            # Issue #2773 — как часто эмбеддер получил выровненный по
            # landmarks вход против фолбека на crop_face.
            'align_used_total': self._align_used_total,
            'align_fallback_total': self._align_fallback_total,
            'active_tracks': self._tracker.active_track_count(),
            'store': store_stats,
        }


def _iou_cxcywh(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> float:
    """IoU двух боксов в формате (cx, cy, w, h), нормализованных 0..1."""
    ax1, ay1 = a[0] - a[2] / 2.0, a[1] - a[3] / 2.0
    ax2, ay2 = a[0] + a[2] / 2.0, a[1] + a[3] / 2.0
    bx1, by1 = b[0] - b[2] / 2.0, b[1] - b[3] / 2.0
    bx2, by2 = b[0] + b[2] / 2.0, b[1] + b[3] / 2.0

    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    union = max(0.0, a[2] * a[3]) + max(0.0, b[2] * b[3]) - inter
    return float(inter / union) if union > 0.0 else 0.0


def _crop_coverage(
    bbox: Tuple[float, float, float, float],
    margin: float,
    frame_w: int,
    frame_h: int,
) -> Tuple[float, float]:
    """Доля запрошенного (bbox+margin) прямоугольника в кадре + доля, срезанная сверху (issue #2774).

    Дублирует геометрию клампа ``face_embedding.crop_face`` (см. его
    докстринг: сторона расширяется до ``1 + 2*margin`` от исходного
    bbox). ``crop_face`` клампит бокс к границам кадра молча, не сообщая
    наверх, что и сколько отрезано — а поправить это можно только внутри
    ``face_embedding.py``, который сейчас параллельно переписывает другой
    агент (issue #2773), трогать нельзя. Поэтому геометрия запроса
    пересчитывается заново здесь же, от тех же входов (``bbox``,
    ``margin``, размер кадра), что уйдут в ``crop_face`` следом.

    Args:
        bbox: ``(cx, cy, w, h)``, normalized [0, 1] — как отдаёт
            ``vision_face_loader`` (``bbox_cx``/``bbox_cy``/``bbox_w``/
            ``bbox_h``).
        margin: тот же параметр, что уйдёт в ``crop_face(..., margin=)``.
        frame_w, frame_h: размер кадра в пикселях.

    Returns:
        ``(coverage, top_clip_frac)``:
          - ``coverage`` — площадь пересечения запрошенного прямоугольника
            с кадром, делённая на площадь запрошенного прямоугольника.
            ``1.0`` — ничего не срезано, ``0.0`` — бокс вырожден или
            целиком снаружи кадра.
          - ``top_clip_frac`` — доля ВЫСОТЫ запрошенного прямоугольника,
            срезанная именно сверху (``0.0``, если верх не срезан). Верх
            не равноценен низу/бокам: там глаза, без которых эмбеддинг
            ArcFace бессмыслен (issue #2774, живой пример на роботе —
            запись ``b49470e1-...``, встреча №19: подбородок/рот/шея без
            единого глаза).
    """
    cx, cy, bw, bh = bbox
    if bw <= 0.0 or bh <= 0.0 or frame_w <= 0 or frame_h <= 0:
        return 0.0, 1.0

    exp_w = bw * (1.0 + 2.0 * margin)
    exp_h = bh * (1.0 + 2.0 * margin)

    x1 = (cx - exp_w / 2.0) * frame_w
    y1 = (cy - exp_h / 2.0) * frame_h
    x2 = (cx + exp_w / 2.0) * frame_w
    y2 = (cy + exp_h / 2.0) * frame_h

    req_w = x2 - x1
    req_h = y2 - y1
    req_area = req_w * req_h
    if req_area <= 0.0:
        return 0.0, 1.0

    x1i = max(0.0, x1)
    y1i = max(0.0, y1)
    x2i = min(float(frame_w), x2)
    y2i = min(float(frame_h), y2)

    clamped_w = max(0.0, x2i - x1i)
    clamped_h = max(0.0, y2i - y1i)
    coverage = (clamped_w * clamped_h) / req_area

    top_clip_frac = max(0.0, (y1i - y1) / req_h) if req_h > 0.0 else 1.0
    return coverage, top_clip_frac
