#!/usr/bin/env python3
"""face_tracker.py — склейка детекций лица во «Встречу» (ADR-0123 §3, #2599 PR-B).

Проблема, которую решает модуль: RetinaFace отдаёт по кадру детекцию на
лицо, а кадров — несколько в секунду (живой замер ADR-0123 §3: 88 событий
``face`` за минуту от ОДНОГО лица). Писать по записи на детекцию нельзя —
получится мусор вместо истории встреч. Единица хранения — **встреча** в
смысле ADR-0096/ADR-0105: лицо, которое было в кадре достаточно долго и
достаточно крупно, чтобы это не было мельканием или шумом детектора.

Модуль не знает про ROS, Hailo или диск — он получает список
``FaceObservation`` за кадр и текущее время, ведёт треки внутри процесса
и решает, когда трек стал встречей. Хранение (``FaceStore``, ADR-0123 §5),
узнавание (ADR-0123 §6) и снимки (§4) — заботы других модулей.

**Важное отклонение от буквального чтения ADR-0123 §3.** Текст ADR говорит
«на встречу выбираются 1–3 лучших кадра», что при поверхностном чтении
можно понять как «решение принимается после того, как трек закончился».
Это не так: цель — чтобы робот поздоровался с человеком, пока тот ещё
стоит перед камерой, а не когда он уже вышел из кадра. Поэтому трек
**повышается до встречи в тот момент, когда он впервые удовлетворяет
условию промоушна** (см. ``_should_promote``), и ровно один раз за жизнь
трека. После промоушна трек продолжает накапливать наблюдения и уточнять
лучшие кадры — это не второй промоушн, а обновление уже существующей
встречи (метод ``update`` возвращает только вновь-повышенные встречи,
``snapshot`` — актуальное состояние любого живого трека).

Ключевые понятия ADR-0123 §3:
  - **min_track_sec** — трек должен прожить не меньше этого времени
    (стартовое 2 с), иначе мелькнувшее лицо не становится встречей;
  - **min_face_px** — хотя бы одно наблюдение должно быть не мельче этого
    по короткой стороне в пикселях (стартовое 48 px) — иначе это лицо на
    заднем плане, не тот, с кем встретились;
  - **best_n лучших кадров** — по размеру, резкости и confidence
    (фронтальность по landmarks сейчас недоступна — RetinaFace-декодер в
    ``vision_face_loader`` их не отдаёт, см. ниже ``SCORE_WEIGHT_*``);
  - **эмбеддинг встречи** — среднее эмбеддингов лучших кадров, нормированное.

Touchpoints:
- ADR-0123 §3 «Встреча, а не кадр» — спецификация этого модуля.
- ADR-0105 (in-process «Встреча») / ADR-0096 (§6 edge-кейсы, §7 lifecycle) —
  вокабуляр «встречи»: этот модуль — источник встреч лицевого канала,
  который со временем будет питать ``EncounterSeam`` через адаптер (по
  аналогии с ``VoiceEncounterAdapter``), но сам шов не трогает.
- issue #2599 PR-B — ArcFace-эмбеддинги + лицевой трекер.
- rob_box_perception.vision_face_loader — источник геометрии наблюдений
  (``bbox_cx/bbox_cy/bbox_w/bbox_h`` normalized 0..1, ``confidence``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

# ============================================================================
# Константы (ADR-0123 §3 — стартовые значения владельца, калибруются позже)
# ============================================================================

#: Трек должен прожить не меньше этого времени, чтобы стать встречей.
DEFAULT_MIN_TRACK_SEC = 2.0

#: Хотя бы одно наблюдение трека должно быть не мельче этого (короткая
#: сторона bbox в пикселях), иначе это фон, а не встреча.
DEFAULT_MIN_FACE_PX = 48.0

#: Трек считается прерванным (не продолжением), если пропуск между
#: наблюдениями превысил это время.
DEFAULT_MAX_GAP_SEC = 1.5

#: Порог IoU для ассоциации наблюдения с существующим треком.
DEFAULT_IOU_THRESHOLD = 0.3

#: Порог косинусной близости эмбеддингов для "спасения" ассоциации, когда
#: IoU её не даёт (человек сдвинулся быстро между кадрами).
DEFAULT_EMBEDDING_THRESHOLD = 0.5

#: Сколько лучших наблюдений хранить на трек.
DEFAULT_BEST_N = 3

#: Максимум одновременно живых треков (защита от неограниченного роста
#: при шумном детекторе / толпе).
DEFAULT_MAX_TRACKS = 16

#: Веса скоринга лучших кадров (ADR-0123 §3: «крупнее, фронтальнее по
#: landmarks, резче»). Фронтальность сегодня недоступна — RetinaFace HEF
#: в vision_face_loader landmarks не декодирует (PR-A scope, см. модуль),
#: поэтому вес фронтальности зарезервирован (0.0) и вынесен отдельной
#: константой, чтобы её включение в будущем было однострочным изменением,
#: а не переписыванием скоринга.
SCORE_WEIGHT_SIZE = 0.5
SCORE_WEIGHT_SHARPNESS = 0.3
SCORE_WEIGHT_CONFIDENCE = 0.2
SCORE_WEIGHT_FRONTALITY = 0.0  # зарезервировано: landmarks пока не приходят

#: Нормировочная константа для face_px в скоринге (типичный крупный план
#: лица с фронтальной камеры мастерской — порядка 200 px по короткой
#: стороне; используется только чтобы привести размер к [0, ~1] масштабу
#: наравне с sharpness/confidence, само число не является порогом).
SCORE_SIZE_NORM_PX = 200.0

#: Нормировочная константа для sharpness (variance-of-Laplacian). Резкие
#: кадры веб/USB-камер обычно дают значения в районе сотен-полутора тысяч;
#: как и SCORE_SIZE_NORM_PX — это масштабирующий делитель, не порог.
SCORE_SHARPNESS_NORM = 500.0


# ============================================================================
# Публичные типы
# ============================================================================

@dataclass
class FaceObservation:
    """Одна детекция лица за один кадр (вход трекера).

    Геометрия — normalized bbox 0..1, как отдаёт ``vision_face_loader``
    (``bbox_cx/bbox_cy/bbox_w/bbox_h``). ``face_px`` считает вызывающий
    код из размера кадра — трекер сам размер кадра в пикселях не знает
    (и не обязан: он видит только то, что ему передали).
    """

    ts: float
    bbox_cx: float
    bbox_cy: float
    bbox_w: float
    bbox_h: float
    confidence: float
    face_px: float
    embedding: Optional[Any] = None
    sharpness: float = 0.0
    crop: Optional[Any] = None
    frame_w: int = 0
    frame_h: int = 0


@dataclass
class FaceEncounter:
    """Снимок состояния трека, повышенного (или ещё не повышенного) до встречи.

    ``promoted_at`` имеет смысл только после промоушна; до него в него
    пишется момент последнего обновления снапшота — потребители обязаны
    полагаться на факт присутствия в возвращаемом списке ``update()``,
    а не гадать по полю.
    """

    track_id: int
    started_at: float
    promoted_at: float
    last_seen_at: float
    observation_count: int
    max_face_px: float
    mean_confidence: float
    embedding: Optional[Any]
    best: List[FaceObservation] = field(default_factory=list)


# ============================================================================
# Внутреннее состояние трека
# ============================================================================

class _Track:
    """Живой трек — накопитель наблюдений одного лица между кадрами."""

    __slots__ = (
        'track_id', 'started_at', 'last_seen_at', 'promoted', 'promoted_at',
        'observation_count', 'confidence_sum', 'max_face_px', 'last_obs',
        'best',
    )

    def __init__(self, track_id: int, obs: FaceObservation, now: float) -> None:
        self.track_id = track_id
        self.started_at = now
        self.last_seen_at = now
        self.promoted = False
        self.promoted_at = 0.0
        self.observation_count = 0
        self.confidence_sum = 0.0
        self.max_face_px = 0.0
        self.last_obs: FaceObservation = obs
        self.best: List[FaceObservation] = []
        self.add_observation(obs, now)

    def add_observation(self, obs: FaceObservation, now: float) -> None:
        self.last_seen_at = now
        self.last_obs = obs
        self.observation_count += 1
        self.confidence_sum += obs.confidence
        if obs.face_px > self.max_face_px:
            self.max_face_px = obs.face_px
        self._insert_best(obs)

    def _insert_best(self, obs: FaceObservation) -> None:
        # Обрезка до best_n происходит в FaceTracker._trim_best сразу
        # после вызова add_observation — best_n конфигурируется на
        # трекере, а не на треке, поэтому здесь список только растёт
        # и остаётся отсортированным по убыванию скора.
        self.best.append(obs)
        self.best.sort(key=_observation_score, reverse=True)

    def alive_sec(self, now: float) -> float:
        return now - self.started_at

    def mean_confidence(self) -> float:
        if self.observation_count == 0:
            return 0.0
        return self.confidence_sum / self.observation_count

    def snapshot(self, best_n: int) -> FaceEncounter:
        best = self.best[:best_n]
        embedding = _mean_embedding([o.embedding for o in best])
        return FaceEncounter(
            track_id=self.track_id,
            started_at=self.started_at,
            promoted_at=self.promoted_at,
            last_seen_at=self.last_seen_at,
            observation_count=self.observation_count,
            max_face_px=self.max_face_px,
            mean_confidence=self.mean_confidence(),
            embedding=embedding,
            best=list(best),
        )


def _observation_score(obs: FaceObservation) -> float:
    """Скор кадра для отбора «лучших» (ADR-0123 §3): размер, резкость, confidence.

    Фронтальность по landmarks не учитывается — детектор их не отдаёт
    (см. комментарий у ``SCORE_WEIGHT_FRONTALITY``). Веса и нормировки —
    именованные модульные константы, чтобы добавление фронтальности в
    будущем было изменением одной строки, а не переписыванием функции.
    """
    size_term = min(obs.face_px / SCORE_SIZE_NORM_PX, 1.0)
    sharpness_term = min(obs.sharpness / SCORE_SHARPNESS_NORM, 1.0)
    confidence_term = obs.confidence
    return (
        SCORE_WEIGHT_SIZE * size_term
        + SCORE_WEIGHT_SHARPNESS * sharpness_term
        + SCORE_WEIGHT_CONFIDENCE * confidence_term
    )


def _mean_embedding(embeddings: List[Optional[Any]]) -> Optional[Any]:
    """Среднее эмбеддингов лучших кадров, ре-нормированное (ADR-0123 §3).

    None, если ни одно наблюдение не несло эмбеддинг — вызывающий код
    (узнавание) в этом случае пропускает идентификацию, а не выдумывает
    эмбеддинг из нуля.
    """
    import numpy as np

    present = [e for e in embeddings if e is not None]
    if not present:
        return None
    stacked = np.stack([np.asarray(e, dtype=np.float32) for e in present], axis=0)
    mean = stacked.mean(axis=0)
    norm = float(np.linalg.norm(mean))
    if norm <= 0.0:
        return mean.astype(np.float32)
    return (mean / norm).astype(np.float32)


def _iou(a: FaceObservation, b: FaceObservation) -> float:
    """IoU двух normalized bbox (cx, cy, w, h)."""
    ax1, ay1 = a.bbox_cx - a.bbox_w / 2.0, a.bbox_cy - a.bbox_h / 2.0
    ax2, ay2 = a.bbox_cx + a.bbox_w / 2.0, a.bbox_cy + a.bbox_h / 2.0
    bx1, by1 = b.bbox_cx - b.bbox_w / 2.0, b.bbox_cy - b.bbox_h / 2.0
    bx2, by2 = b.bbox_cx + b.bbox_w / 2.0, b.bbox_cy + b.bbox_h / 2.0

    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, a.bbox_w) * max(0.0, a.bbox_h)
    area_b = max(0.0, b.bbox_w) * max(0.0, b.bbox_h)
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def _cosine_similarity(a: Any, b: Any) -> float:
    """Косинусная близость двух эмбеддингов.

    Пытается переиспользовать ``rob_box_perception.face_embedding`` —
    но ИМПОРТ ЛЕНИВЫЙ и с фоллбэком: над этим модулем параллельно
    работает другой агент (issue #2599 PR-B, ArcFace-эмбеддинги), и на
    момент запуска этих тестов ``face_embedding`` может ещё не
    импортироваться (или импортироваться с другой сигнатурой). Трекер
    не должен падать/тормозить из-за чужого недописанного модуля —
    поэтому есть локальный numpy-фоллбэк, идентичный по смыслу.
    """
    try:
        from rob_box_perception.face_embedding import (  # type: ignore[import-not-found]
            cosine_similarity as _external_cosine,
        )
        return float(_external_cosine(a, b))
    except Exception:
        import numpy as np

        va = np.asarray(a, dtype=np.float32).reshape(-1)
        vb = np.asarray(b, dtype=np.float32).reshape(-1)
        na = float(np.linalg.norm(va))
        nb = float(np.linalg.norm(vb))
        if na <= 0.0 or nb <= 0.0:
            return 0.0
        return float(np.dot(va, vb) / (na * nb))


# ============================================================================
# FaceTracker
# ============================================================================

class FaceTracker:
    """Склеивает детекции лица в треки и повышает их до встреч (ADR-0123 §3).

    Полностью синхронный, без часов внутри — время подаёт вызывающий код
    через ``now`` в ``update``/``expire``. Это сделано намеренно: узел
    (ROS) или тест (pytest) сами решают, что такое «сейчас», а трекер не
    обязан знать про ``time.monotonic()`` или ROS clock.
    """

    def __init__(
        self,
        *,
        min_track_sec: float = DEFAULT_MIN_TRACK_SEC,
        min_face_px: float = DEFAULT_MIN_FACE_PX,
        max_gap_sec: float = DEFAULT_MAX_GAP_SEC,
        iou_threshold: float = DEFAULT_IOU_THRESHOLD,
        embedding_threshold: float = DEFAULT_EMBEDDING_THRESHOLD,
        best_n: int = DEFAULT_BEST_N,
        max_tracks: int = DEFAULT_MAX_TRACKS,
    ) -> None:
        self._min_track_sec = min_track_sec
        self._min_face_px = min_face_px
        self._max_gap_sec = max_gap_sec
        self._iou_threshold = iou_threshold
        self._embedding_threshold = embedding_threshold
        self._best_n = best_n
        self._max_tracks = max_tracks

        self._tracks: Dict[int, _Track] = {}
        self._next_track_id = 1
        self._promoted_ids: set = set()

    # ------------------------------------------------------------------
    # Основной цикл
    # ------------------------------------------------------------------

    def update(self, observations: List[FaceObservation], now: float) -> List[FaceEncounter]:
        """Ассоциировать наблюдения кадра с треками, повысить готовые встречи.

        Возвращает встречи, повышенные ИМЕННО НА ЭТОМ вызове (обычно
        пусто; не больше одной на трек за всю жизнь трека) — см. модульный
        docstring про то, почему промоушн происходит здесь, а не в
        ``expire``.
        """
        assignments = self._associate(observations, now)

        matched_track_ids = set()
        for obs, track_id in assignments:
            if track_id is None:
                track_id = self._start_track(obs, now)
            else:
                self._tracks[track_id].add_observation(obs, now)
            matched_track_ids.add(track_id)
            self._trim_best(self._tracks[track_id])

        self._enforce_max_tracks()

        promoted: List[FaceEncounter] = []
        for track_id in matched_track_ids:
            track = self._tracks.get(track_id)
            if track is None:
                continue
            if not track.promoted and self._should_promote(track, now):
                track.promoted = True
                track.promoted_at = now
                self._promoted_ids.add(track_id)
                promoted.append(track.snapshot(self._best_n))
        return promoted

    def _should_promote(self, track: _Track, now: float) -> bool:
        """ADR-0123 §3: жив >= min_track_sec И хотя бы раз лицо >= min_face_px."""
        return (
            track.alive_sec(now) >= self._min_track_sec
            and track.max_face_px >= self._min_face_px
        )

    def _trim_best(self, track: _Track) -> None:
        if len(track.best) > self._best_n:
            del track.best[self._best_n:]

    # ------------------------------------------------------------------
    # Ассоциация
    # ------------------------------------------------------------------

    def _associate(
        self,
        observations: List[FaceObservation],
        now: float,
    ) -> List[Any]:
        """Сопоставить наблюдения кадра трекам: IoU, с эмбеддинг-спасением.

        Один трек — не больше одного наблюдения за кадр (конфликт решает
        лучший суммарный score = IoU, либо cosine similarity, когда она и
        дала спасение). Несопоставленные наблюдения получают track_id=None
        (новый трек создаст вызывающий код после этого прохода).
        """
        candidate_tracks = list(self._tracks.values())

        # score_matrix[i][j] = (score, track_id) для observations[i] x candidate_tracks[j]
        pairs: List[Any] = []  # (score, obs_idx, track_id)
        for i, obs in enumerate(observations):
            for track in candidate_tracks:
                iou = _iou(obs, track.last_obs)
                score = iou
                matched_by_embedding = False
                if obs.embedding is not None and track.last_obs.embedding is not None:
                    sim = _cosine_similarity(obs.embedding, track.last_obs.embedding)
                    if sim >= self._embedding_threshold:
                        matched_by_embedding = True
                        # Эмбеддинг может как спасти пропущенный IoU, так и
                        # разрешить связку сильнее — берём максимум, чтобы
                        # эмбеддинг-совпадение выигрывало тай-брейк у
                        # слабого IoU-совпадения другого трека.
                        score = max(iou, sim)
                if iou >= self._iou_threshold or matched_by_embedding:
                    pairs.append((score, i, track.track_id))

        pairs.sort(key=lambda p: p[0], reverse=True)

        assigned_obs = set()
        assigned_tracks = set()
        result_by_obs: Dict[int, Optional[int]] = {i: None for i in range(len(observations))}
        for score, obs_idx, track_id in pairs:
            if obs_idx in assigned_obs or track_id in assigned_tracks:
                continue
            assigned_obs.add(obs_idx)
            assigned_tracks.add(track_id)
            result_by_obs[obs_idx] = track_id

        return [(observations[i], result_by_obs[i]) for i in range(len(observations))]

    def _start_track(self, obs: FaceObservation, now: float) -> int:
        track_id = self._next_track_id
        self._next_track_id += 1
        self._tracks[track_id] = _Track(track_id, obs, now)
        return track_id

    def _enforce_max_tracks(self) -> None:
        while len(self._tracks) > self._max_tracks:
            # Вытесняем наименее недавно виденный — он с наибольшей
            # вероятностью уже ушёл из кадра.
            oldest_id = min(self._tracks, key=lambda tid: self._tracks[tid].last_seen_at)
            del self._tracks[oldest_id]

    # ------------------------------------------------------------------
    # Истечение
    # ------------------------------------------------------------------

    def expire(self, now: float) -> List[int]:
        """Удалить треки, не виденные дольше ``max_gap_sec``. Вернуть их id."""
        expired = [
            tid for tid, track in self._tracks.items()
            if now - track.last_seen_at > self._max_gap_sec
        ]
        for tid in expired:
            del self._tracks[tid]
        return expired

    # ------------------------------------------------------------------
    # Инспекция
    # ------------------------------------------------------------------

    def snapshot(self, track_id: int) -> Optional[FaceEncounter]:
        """Текущее уточнённое состояние живого трека (read-only)."""
        track = self._tracks.get(track_id)
        if track is None:
            return None
        return track.snapshot(self._best_n)

    def active_track_count(self) -> int:
        return len(self._tracks)

    def promoted_track_count(self) -> int:
        return len(self._promoted_ids)
