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
        #: Кропы, отброшенные воротами качества (issue #2749) — почти
        #: чёрные/плоские, не дошли даже до эмбеддера. Отдельно от
        #: ``embed_failures`` (там ArcFace/HailoRT реально падает).
        self._crop_rejected_total = 0

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

        crops, face_px_list = self._crop_all(detections, image, frame_w, frame_h)
        embeddings = self._embed_all(crops)

        observations = self._make_observations(
            detections, crops, embeddings, face_px_list, frame_w, frame_h, ts
        )

        # 1. Кто в кадре — на каждом кадре, только чтение.
        person_ids = self._annotate_identities(detections, embeddings)
        self._last_frame_ts = ts
        self._last_frame_person_ids = person_ids

        # 2. Кто появился — редко, по правилу Встречи.
        promoted = self._tracker.update(observations, ts)
        self._tracker.expire(ts)
        for encounter in promoted:
            self._on_encounter(encounter, detections, crops)

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
    ) -> Tuple[List[Any], List[float]]:
        """Кропы лиц с запасом + размер лица в пикселях."""
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

        crops: List[Any] = []
        for idx, bbox in enumerate(bboxes):
            if idx not in chosen:
                crops.append(None)
                continue
            crop = crop_face(image, bbox, margin=self._crop_margin)
            if crop is not None and not self._crop_quality_ok(crop):
                # Ворота качества (issue #2749): кроп живой, но почти
                # чёрный/плоский — не эмбеддим и не кладём в снимок,
                # трактуем как отсутствующий кроп (тот же путь, что и
                # бюджетный пропуск выше).
                self._crop_rejected_total += 1
                crop = None
            crops.append(crop)
        return crops, face_px_list

    def _crop_quality_ok(self, crop: Any) -> bool:
        """Ворота качества кропа перед эмбеддингом (issue #2749).

        Фильтровать нужно ЗДЕСЬ, пока кроп ещё живой numpy-массив:
        ``FaceStore`` ниже по стеку принципиально не разглядывает снимки
        (ADR-0123 §5, см. его докстринг) — граница режимов приватности
        проведена там намеренно, и нарушать её нельзя; а после кодирования
        в JPEG для записи встречи уже поздно — пиксели надо смотреть
        раньше. Возвращает ``True``, если метрики посчитать не удалось
        (cv2 недоступен, см. ``crop_brightness_contrast``) — деградировать
        до "не эмбеддим вообще" из-за отсутствующего cv2 хуже, чем
        пропустить один некалиброванный кроп через ворота.
        """
        metrics = crop_brightness_contrast(crop)
        if metrics is None:
            return True
        mean, contrast = metrics
        return mean >= self._min_crop_mean and contrast >= self._min_crop_contrast

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
        crops: List[Any],
        embeddings: List[Any],
        face_px_list: List[float],
        frame_w: int,
        frame_h: int,
        ts: float,
    ) -> List[FaceObservation]:
        observations: List[FaceObservation] = []
        for idx, det in enumerate(detections):
            crop = crops[idx]
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
        crops: List[Any],
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

        self._log(
            'info',
            '👤 Встреча: {who} (person={pid} sim={sim:.3f} new={new} '
            'encounters={n} face={px:.0f}px)'.format(
                who=match.name or 'незнакомец',
                pid=match.person_id[:8],
                sim=match.similarity,
                new=match.is_new,
                n=match.encounter_count,
                px=encounter.max_face_px,
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

        Args:
            speaker_id: стабильный биометрический id голоса.
            name: имя из голосового профиля.
            now: время для тестов.

        Returns:
            ``person_id`` лицевой записи, если имя привязано; иначе ``None``.
        """
        ts = time.monotonic() if now is None else float(now)

        if not name or not speaker_id:
            return None
        if ts - self._last_frame_ts > self._voice_merge_window_sec:
            return None  # голос без свежего кадра — не с чем сливать
        if len(self._last_frame_person_ids) != 1:
            return None  # ноль или двое в кадре — см. докстринг
        person_id = self._last_frame_person_ids[0]
        if person_id is None:
            return None

        # Уже привязан к другому голосу — не перебиваем: разбор дублей
        # голосовых профилей (ADR-0123 §6, карточка §9.1) живёт отдельно.
        try:
            existing = self._store.find_by_speaker(speaker_id)
            if existing is not None and existing != person_id:
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
            'embed_failures': self._embed_failures,
            'embed_calls': self._embed_calls,
            'embed_ms_avg': round(
                self._embed_ms_total / self._embed_calls, 1
            ) if self._embed_calls else 0.0,
            'embed_skipped_budget': self._embed_skipped_budget,
            'crop_rejected_total': self._crop_rejected_total,
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
