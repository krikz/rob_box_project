"""Unit-тесты для ``FaceRecognizer`` (ADR-0123 §3/§4/§6, issue #2599 PR-B).

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_face_recognition.py -q

``FaceRecognizer`` — чистая оркестрация: ни ROS, ни Hailo, ни файловой
системы. Он склеивает эмбеддер (``ArcFaceEmbedder``), хранилище
(``FaceStore``) и трекер (``FaceTracker``), поэтому тестируется подставными
объектами — фейковым эмбеддером и фейковым хранилищем; реальный
``FaceTracker`` используем как есть (это его контракт мы и опираем на
промоушн-тесты). Время — только через ``now=`` (никаких sleep).

Что покрываем (см. докстринг face_recognition.py):
  1. Пустые детекции / отсутствие кадра — трек всё равно стареет (expire),
     хранилище не трогается.
  2. Покадровое узнавание: ``identify`` вызывается, ``record_encounter`` —
     нет (запись — событие уровня Встречи, не кадра).
  3. Промоушн Встречи: ``record_encounter`` вызывается РОВНО ОДИН раз,
     маркер ``encounter=start`` появляется ровно на одной детекции кадра.
  4. Долгое удержание лица после Встречи не плодит вторых записей/маркеров.
  5. Лица мельче ``min_embed_px`` не эмбеддятся вовсе.
  6. Падение эмбеддера — детекции живы, имени нет, счётчик ошибок растёт,
     нода не падает.
  7. Встреча без эмбеддинга (мелкое лицо) не пишется в хранилище —
     "выдумка не доезжает до Личности" (ADR-0089 §2.2).
  8. Слияние с голосом (ADR-0123 §6) — самая ценная группа тестов.
  9. ``stats()`` — форма и устойчивость к падению ``store.stats()``.
  10. Ворота качества кропа (issue #2749): почти чёрный кроп не доезжает
      до эмбеддера, не создаёт запись в ``FaceStore`` даже если трек
      промоутится геометрией (воспроизводит фантомную запись из 130
      "встреч" тени в тёмной комнате), счётчик ``crop_rejected_total``
      растёт, пороги настраиваемы через параметры конструктора.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import pytest

# ---------- import target under test (тот же приём, что test_face_tracker.py) --

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

face_recognition_mod = importlib.import_module('rob_box_perception.face_recognition')
face_tracker_mod = importlib.import_module('rob_box_perception.face_tracker')
face_store_mod = importlib.import_module('rob_box_perception.face_store')

FaceRecognizer = face_recognition_mod.FaceRecognizer
FaceTracker = face_tracker_mod.FaceTracker
FaceMatch = face_store_mod.FaceMatch


# ============================================================================
# Фейки
# ============================================================================

class FakeEmbedder:
    """Подставной эмбеддер: не трогает Hailo, отдаёт заранее заданные векторы.

    ``embed_calls`` — сколько кропов было запрошено на каждый вызов
    ``embed()`` (для проверки, что мелкие лица отфильтрованы ДО эмбеддера).
    """

    def __init__(self, vector_fn: Optional[Any] = None, raise_exc: Optional[BaseException] = None) -> None:
        self.embed_calls: List[int] = []
        self._vector_fn = vector_fn or (lambda i: np.array([1.0, 0.0, 0.0], dtype=np.float32))
        self._raise_exc = raise_exc

    def embed(self, crops: List[Any]) -> List[Any]:
        self.embed_calls.append(len(crops))
        if self._raise_exc is not None:
            raise self._raise_exc
        return [self._vector_fn(i) for i in range(len(crops))]


class FakeStore:
    """Подставное хранилище — полный контроль над ответами без косинусов.

    Реализует ровно тот контракт, которым пользуется ``FaceRecognizer``:
    ``identify``, ``record_encounter``, ``attach_name``, ``find_by_speaker``,
    ``stats``, атрибут ``mode``.
    """

    def __init__(self, mode: str = 'workshop') -> None:
        self.mode = mode
        self.identify_calls: List[Any] = []
        self.identify_return: Optional[FaceMatch] = None
        self.record_encounter_calls: List[Dict[str, Any]] = []
        self.record_encounter_return: Any = None  # FaceMatch | Exception
        self.attach_name_calls: List[Any] = []
        self.attach_name_return: bool = True
        self.find_by_speaker_map: Dict[str, str] = {}
        self.stats_return: Dict[str, Any] = {'people': 0}
        self.stats_raises: bool = False

    def identify(self, embedding: Any) -> Optional[FaceMatch]:
        self.identify_calls.append(embedding)
        return self.identify_return

    def record_encounter(self, embedding: Any, *, snapshot=None, meta=None) -> FaceMatch:
        self.record_encounter_calls.append({'embedding': embedding, 'snapshot': snapshot, 'meta': meta})
        if isinstance(self.record_encounter_return, BaseException):
            raise self.record_encounter_return
        return self.record_encounter_return

    def attach_name(self, person_id: str, name: str, *, speaker_id: Optional[str] = None) -> bool:
        self.attach_name_calls.append((person_id, name, speaker_id))
        return self.attach_name_return

    def find_by_speaker(self, speaker_id: str) -> Optional[str]:
        return self.find_by_speaker_map.get(speaker_id)

    def stats(self) -> Dict[str, Any]:
        if self.stats_raises:
            raise RuntimeError('face_store.stats() упал (тестовый сбой)')
        return self.stats_return


class RecordingLog:
    """Собирает лог-вызовы ``(level, msg)`` для проверки предупреждений."""

    def __init__(self) -> None:
        self.entries: List[Any] = []

    def __call__(self, level: str, msg: str) -> None:
        self.entries.append((level, msg))

    def has(self, level: str, substring: str = '') -> bool:
        return any(lvl == level and substring in msg for lvl, msg in self.entries)


# ============================================================================
# Хелперы
# ============================================================================

def make_frame(size: int = 200) -> np.ndarray:
    """RGB uint8 кадр size x size с детерминированным шумом.

    Раньше это был чистый чёрный кадр (``np.zeros``) — годился, пока
    ``FaceRecognizer`` не смотрел в пиксели кропа вообще. После ворот
    качества кропа (issue #2749, ``_crop_quality_ok``) чёрный кроп
    отбраковывается ПО ЗАМЫСЛУ (это и есть фильтруемый случай) — чёрный
    кадр как "нейтральный" фикстур больше не годится, иначе он ломает
    все тесты, не имеющие отношения к воротам качества.

    Шум с фиксированным seed — не ``np.zeros`` и не случайный: даёт
    высокую яркость/контраст в ЛЮБОМ кропе кадра независимо от его
    размера и положения (в отличие, например, от плавного градиента,
    где узкий кроп у края мог бы случайно оказаться низкоконтрастным), и
    при этом воспроизводим между запусками тестов.
    """
    rng = np.random.RandomState(20260922)
    return rng.randint(0, 256, size=(size, size, 3)).astype(np.uint8)


def make_black_frame(size: int = 200) -> np.ndarray:
    """RGB uint8 кадр size x size, полностью чёрный.

    Имитирует фантомную детекцию issue #2749 (тень/забитая экспозиция в
    тёмной комнате). Живые цифры на Vision Pi 22.09.2026: mean 6.4-7.4 из
    255, контраст (p95-p5) 28-31 — здесь используется предельный случай
    0/0, который заведомо дальше за обоими порогами по умолчанию
    (``DEFAULT_MIN_CROP_MEAN=20``, ``DEFAULT_MIN_CROP_CONTRAST=25``) и не
    требует точной имитации шумового пола матрицы.
    """
    return np.zeros((size, size, 3), dtype=np.uint8)


def make_detection(
    cx: float = 0.5,
    cy: float = 0.5,
    w: float = 0.5,
    h: float = 0.5,
    confidence: float = 0.9,
    source_camera: str = 'main_camera',
) -> Dict[str, Any]:
    """Детекция VisionEvent-полей, как отдаёт ``vision_face_loader``."""
    return {
        'bbox_cx': cx,
        'bbox_cy': cy,
        'bbox_w': w,
        'bbox_h': h,
        'confidence': confidence,
        'source_camera': source_camera,
    }


def make_match(person_id: str = 'p1', name: str = '', is_new: bool = False, similarity: float = 0.9, encounter_count: int = 1) -> FaceMatch:
    return FaceMatch(person_id=person_id, name=name, similarity=similarity, is_new=is_new, encounter_count=encounter_count)


# ============================================================================
# 1. Пустые детекции / нет кадра — треки стареют, хранилище не трогается
# ============================================================================

def test_empty_detections_ages_tracks_without_touching_store():
    tracker = FaceTracker(min_track_sec=10.0, min_face_px=10.0, max_gap_sec=1.0)
    embedder = FakeEmbedder()
    store = FakeStore()
    rec = FaceRecognizer(embedder=embedder, store=store, tracker=tracker)
    frame = make_frame()

    # Стартуем трек крупным лицом.
    rec.process([make_detection(w=0.5, h=0.5)], frame, now=0.0)
    assert tracker.active_track_count() == 1

    # Пустые детекции при разрыве > max_gap_sec — трек обязан истечь.
    out = rec.process([], frame, now=2.0)
    assert out == []
    assert tracker.active_track_count() == 0
    assert store.record_encounter_calls == []


def test_none_image_ages_tracks_without_touching_store():
    tracker = FaceTracker(min_track_sec=10.0, min_face_px=10.0, max_gap_sec=1.0)
    embedder = FakeEmbedder()
    store = FakeStore()
    rec = FaceRecognizer(embedder=embedder, store=store, tracker=tracker)
    frame = make_frame()

    dets = [make_detection(w=0.5, h=0.5)]
    rec.process(dets, frame, now=0.0)
    assert tracker.active_track_count() == 1
    identify_calls_before = len(store.identify_calls)

    # image=None — тот же путь раннего выхода, что и пустые детекции.
    dets2 = [make_detection(w=0.5, h=0.5)]
    result = rec.process(dets2, None, now=2.0)
    assert result is dets2  # тот же список вернулся как есть, немутированный
    assert tracker.active_track_count() == 0
    assert len(store.identify_calls) == identify_calls_before, 'без кадра узнавание не запускается'
    assert store.record_encounter_calls == []


# ============================================================================
# 2. Покадровое узнавание: identify() да, record_encounter() нет
# ============================================================================

def test_per_frame_identification_calls_identify_not_record_encounter():
    embedder = FakeEmbedder()
    store = FakeStore()
    store.identify_return = make_match(person_id='denis-1', name='Денис', encounter_count=5)
    rec = FaceRecognizer(embedder=embedder, store=store)
    frame = make_frame()

    dets = [make_detection(w=0.5, h=0.5)]
    out = rec.process(dets, frame, now=0.0)

    assert out[0]['embedding_id'] == 'denis-1'
    assert out[0]['display_name'] == 'Денис'
    assert len(store.identify_calls) == 1
    assert store.record_encounter_calls == [], (
        'запись в хранилище — событие уровня Встречи, а не кадра'
    )


def test_per_frame_no_match_leaves_fields_unset():
    embedder = FakeEmbedder()
    store = FakeStore()
    store.identify_return = None  # незнакомец / не узнан
    rec = FaceRecognizer(embedder=embedder, store=store)
    frame = make_frame()

    dets = [make_detection(w=0.5, h=0.5)]
    out = rec.process(dets, frame, now=0.0)

    assert 'embedding_id' not in out[0]
    assert 'display_name' not in out[0]
    assert store.record_encounter_calls == []


# ============================================================================
# 3. Промоушн Встречи: запись ровно один раз, маркер ровно на одной детекции
# ============================================================================

def test_encounter_promotion_records_once_and_marks_one_detection():
    tracker = FaceTracker(min_track_sec=1.0, min_face_px=48.0, max_gap_sec=5.0)
    embedder = FakeEmbedder()
    store = FakeStore()
    store.identify_return = None  # ещё не знаем, кто это, до Встречи
    store.record_encounter_return = make_match(
        person_id='new-person-1', name='', is_new=True, similarity=1.0, encounter_count=1
    )
    rec = FaceRecognizer(embedder=embedder, store=store, tracker=tracker)
    frame = make_frame()

    last_out: List[Dict[str, Any]] = []
    for t in (0.0, 0.5, 1.0):
        dets = [make_detection(w=0.5, h=0.5)]
        last_out = rec.process(dets, frame, now=t)

    assert len(store.record_encounter_calls) == 1

    marked = [d for d in last_out if 'attributes_json' in d]
    assert len(marked) == 1
    import json
    payload = json.loads(marked[0]['attributes_json'])
    assert payload['encounter'] == 'start'
    assert payload['person_id'] == 'new-person-1'
    assert payload['is_new'] is True

    assert rec.stats()['encounters_total'] == 1
    assert rec.stats()['new_people_total'] == 1


def test_holding_face_longer_does_not_produce_second_encounter_or_marker():
    tracker = FaceTracker(min_track_sec=1.0, min_face_px=48.0, max_gap_sec=5.0)
    embedder = FakeEmbedder()
    store = FakeStore()
    store.identify_return = None
    store.record_encounter_return = make_match(person_id='p1', is_new=True)
    rec = FaceRecognizer(embedder=embedder, store=store, tracker=tracker)
    frame = make_frame()

    # Доводим трек до Встречи.
    for t in (0.0, 0.5, 1.0):
        rec.process([make_detection(w=0.5, h=0.5)], frame, now=t)
    assert len(store.record_encounter_calls) == 1

    # Держим лицо в кадре ещё много кадров.
    for t in (1.5, 2.0, 2.5, 3.0, 3.5, 4.0):
        out = rec.process([make_detection(w=0.5, h=0.5)], frame, now=t)
        assert 'attributes_json' not in out[0], 'второй маркер encounter=start недопустим'

    assert len(store.record_encounter_calls) == 1, 'вторая запись Встречи недопустима'
    assert rec.stats()['encounters_total'] == 1


# ============================================================================
# 5. Лица мельче min_embed_px не эмбеддятся
# ============================================================================

def test_small_face_alone_never_reaches_embedder():
    embedder = FakeEmbedder()
    store = FakeStore()
    rec = FaceRecognizer(embedder=embedder, store=store, min_embed_px=32.0)
    frame = make_frame()

    # face_px = min(0.05*200, 0.05*200) = 10 < 32
    dets = [make_detection(w=0.05, h=0.05)]
    rec.process(dets, frame, now=0.0)

    assert embedder.embed_calls == [], 'эмбеддер вообще не должен вызываться'


def test_mixed_small_and_large_face_only_large_goes_to_embedder():
    embedder = FakeEmbedder()
    store = FakeStore()
    rec = FaceRecognizer(embedder=embedder, store=store, min_embed_px=32.0)
    frame = make_frame()

    dets = [
        make_detection(cx=0.2, w=0.05, h=0.05),  # face_px=10 — мелкое
        make_detection(cx=0.8, w=0.5, h=0.5),    # face_px=100 — крупное
    ]
    rec.process(dets, frame, now=0.0)

    assert embedder.embed_calls == [1], 'в ArcFace должен уйти только один кроп из двух'


# ============================================================================
# 6. Падение эмбеддера — не роняет ноду, детекции живы, счётчик растёт
# ============================================================================

def test_embedder_failure_is_swallowed_and_counted():
    embedder = FakeEmbedder(raise_exc=RuntimeError('HailoRT boom'))
    store = FakeStore()
    log = RecordingLog()
    rec = FaceRecognizer(embedder=embedder, store=store, log_fn=log)
    frame = make_frame()

    dets = [make_detection(w=0.5, h=0.5)]
    out = rec.process(dets, frame, now=0.0)

    assert out is dets  # детекции продолжают публиковаться
    assert 'embedding_id' not in out[0]
    assert 'display_name' not in out[0]
    assert rec.stats()['embed_failures'] == 1
    assert store.identify_calls == []  # эмбеддинга нет — identify не вызывался
    assert log.has('error')

    # Второй сбой — счётчик растёт дальше, нода по-прежнему не падает.
    rec.process([make_detection(w=0.5, h=0.5)], frame, now=1.0)
    assert rec.stats()['embed_failures'] == 2


# ============================================================================
# 7. Встреча без эмбеддинга не пишется в хранилище
# ============================================================================

def test_encounter_without_embedding_does_not_record():
    # min_embed_px выше, чем размер лица, который всё же промоутит трекер:
    # трек становится Встречей по геометрии, а эмбеддинга у него нет.
    tracker = FaceTracker(min_track_sec=1.0, min_face_px=48.0, max_gap_sec=5.0)
    embedder = FakeEmbedder()
    store = FakeStore()
    log = RecordingLog()
    rec = FaceRecognizer(embedder=embedder, store=store, tracker=tracker, min_embed_px=200.0, log_fn=log)
    frame = make_frame()  # 200x200 -> face_px=100 при w=h=0.5, ниже min_embed_px=200

    for t in (0.0, 0.5, 1.0):
        rec.process([make_detection(w=0.5, h=0.5)], frame, now=t)

    assert store.record_encounter_calls == [], 'без эмбеддинга запись — выдумка, недопустимо'
    assert rec.stats()['encounters_total'] == 1  # счётчик встреч растёт, но без записи
    assert log.has('warn')


# ============================================================================
# 8. Слияние с голосом (ADR-0123 §6)
# ============================================================================

def _prime_last_frame(rec: FaceRecognizer, store: FakeStore, person_ids: List[Optional[str]], now: float, frame=None) -> None:
    """Прогнать один кадр так, чтобы ``_last_frame_person_ids`` стал ``person_ids``.

    Каждой детекции соответствует один person_id (``None`` — не узнан).
    Хелпер настраивает ``store.identify`` через побочный список ответов.
    """
    frame = frame if frame is not None else make_frame()
    dets = [make_detection(cx=0.1 + 0.2 * i, w=0.5, h=0.5) for i in range(len(person_ids))]
    answers = iter(
        make_match(person_id=pid) if pid is not None else None for pid in person_ids
    )
    original_identify = store.identify

    def fake_identify(embedding):
        store.identify_calls.append(embedding)
        return next(answers)

    store.identify = fake_identify  # type: ignore[method-assign]
    rec.process(dets, frame, now=now)
    store.identify = original_identify  # type: ignore[method-assign]


def test_voice_merge_single_known_face_attaches_name():
    tracker = FaceTracker(min_track_sec=1000.0)  # промоушн не мешает тесту
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker, voice_merge_window_sec=6.0)

    _prime_last_frame(rec, store, ['p1'], now=10.0)

    result = rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.0)

    assert result == 'p1'
    assert store.attach_name_calls == [('p1', 'Денис', 'spk-1')]


def test_voice_merge_two_faces_in_frame_no_merge():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker)

    _prime_last_frame(rec, store, ['p1', 'p2'], now=10.0)

    result = rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.0)

    assert result is None
    assert store.attach_name_calls == []


def test_voice_merge_zero_faces_no_merge():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker)

    rec.process([], make_frame(), now=10.0)

    result = rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.0)

    assert result is None
    assert store.attach_name_calls == []


def test_voice_merge_stale_frame_no_merge():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker, voice_merge_window_sec=6.0)

    _prime_last_frame(rec, store, ['p1'], now=10.0)

    # 100с спустя — кадр давно устарел (окно 6с).
    result = rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=110.0)

    assert result is None
    assert store.attach_name_calls == []


def test_voice_merge_empty_name_no_merge():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker)

    _prime_last_frame(rec, store, ['p1'], now=10.0)

    result = rec.note_voice_identification(speaker_id='spk-1', name='', now=11.0)

    assert result is None
    assert store.attach_name_calls == []


def test_voice_merge_empty_speaker_id_no_merge():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker)

    _prime_last_frame(rec, store, ['p1'], now=10.0)

    result = rec.note_voice_identification(speaker_id='', name='Денис', now=11.0)

    assert result is None
    assert store.attach_name_calls == []


def test_voice_merge_speaker_bound_to_different_person_no_merge_and_warns():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    store.find_by_speaker_map = {'spk-1': 'other-person'}
    log = RecordingLog()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker, log_fn=log)

    _prime_last_frame(rec, store, ['p1'], now=10.0)

    result = rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.0)

    assert result is None
    assert store.attach_name_calls == []
    assert log.has('warn')


# ============================================================================
# 8b. Причины пропуска слияния — issue #2748 («слияний=0» само по себе не
#     говорит, ПОЧЕМУ). Каждый ранний return из note_voice_identification()
#     обязан инкрементировать СВОЙ, отдельный счётчик в stats().
# ============================================================================


def test_voice_merge_skip_counted_as_multi_face():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker)
    _prime_last_frame(rec, store, ['p1', 'p2'], now=10.0)

    rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.0)

    s = rec.stats()
    assert s['voice_merge_skip_multi_face'] == 1
    assert s['voice_merge_skip_no_face'] == 0
    assert s['voice_merge_skip_stale'] == 0
    assert s['voice_merge_skip_conflict'] == 0


def test_voice_merge_skip_counted_as_no_face():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker)
    rec.process([], make_frame(), now=10.0)

    rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.0)

    s = rec.stats()
    assert s['voice_merge_skip_no_face'] == 1
    assert s['voice_merge_skip_multi_face'] == 0


def test_voice_merge_skip_counted_as_stale():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker, voice_merge_window_sec=6.0)
    _prime_last_frame(rec, store, ['p1'], now=10.0)

    rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=110.0)

    s = rec.stats()
    assert s['voice_merge_skip_stale'] == 1
    assert s['voice_merge_skip_no_face'] == 0


def test_voice_merge_skip_counted_as_conflict():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    store.find_by_speaker_map = {'spk-1': 'other-person'}
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker)
    _prime_last_frame(rec, store, ['p1'], now=10.0)

    rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.0)

    s = rec.stats()
    assert s['voice_merge_skip_conflict'] == 1


def test_voice_merge_skip_counters_accumulate_across_calls():
    """Несколько пропусков ПОДРЯД суммируются, а не перезаписываются —
    иначе periodic-сводка (раз в 60с) видела бы только последнюю причину."""
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker)

    rec.process([], make_frame(), now=10.0)
    rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.0)
    rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.5)

    assert rec.stats()['voice_merge_skip_no_face'] == 2


def test_successful_merge_does_not_touch_skip_counters():
    tracker = FaceTracker(min_track_sec=1000.0)
    store = FakeStore()
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store, tracker=tracker, voice_merge_window_sec=6.0)
    _prime_last_frame(rec, store, ['p1'], now=10.0)

    result = rec.note_voice_identification(speaker_id='spk-1', name='Денис', now=11.0)

    assert result == 'p1'
    s = rec.stats()
    assert s['voice_merges_total'] == 1
    assert s['voice_merge_skip_no_face'] == 0
    assert s['voice_merge_skip_multi_face'] == 0
    assert s['voice_merge_skip_stale'] == 0
    assert s['voice_merge_skip_conflict'] == 0


# ============================================================================
# 9. stats() — форма и устойчивость к падению store.stats()
# ============================================================================

def test_stats_shape():
    store = FakeStore()
    store.stats_return = {'mode': 'workshop', 'people': 3}
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store)

    s = rec.stats()

    expected_keys = {
        'encounters_total', 'recognized_total', 'new_people_total',
        'voice_merges_total', 'embed_failures', 'active_tracks', 'store',
    }
    assert expected_keys <= set(s.keys())
    assert s['store'] == {'mode': 'workshop', 'people': 3}
    assert s['encounters_total'] == 0
    assert s['embed_failures'] == 0


def test_stats_survives_store_stats_exception():
    store = FakeStore()
    store.stats_raises = True
    rec = FaceRecognizer(embedder=FakeEmbedder(), store=store)

    s = rec.stats()  # не должно бросить исключение наружу

    assert s['store'] == {}
    assert 'active_tracks' in s


# ============================================================================
# 10. Ворота качества кропа (issue #2749)
# ============================================================================
#
# Контекст: на живом Vision Pi 22.09.2026 RetinaFace полчаса детектировал
# тень в тёмной комнате (confidence/размер бокса в норме, сам кроп
# почти чёрный) — ArcFace на таком входе стабильно отдавал один и тот же
# вырожденный эмбеддинг, и в базу легла запись из 130 "встреч" с попарной
# близостью эмбеддингов 0.90+ (настоящий человек в той же базе — 0.17-0.81).
# Ворота ``_crop_quality_ok`` ловят это ДО эмбеддера, на живом numpy-массиве
# кропа, не трогая границу ``FaceStore`` (ADR-0123 §5).

def test_black_crop_rejected_before_embedder():
    embedder = FakeEmbedder()
    store = FakeStore()
    rec = FaceRecognizer(embedder=embedder, store=store)
    frame = make_black_frame()

    dets = [make_detection(w=0.5, h=0.5)]
    rec.process(dets, frame, now=0.0)

    assert embedder.embed_calls == [], 'почти чёрный кроп не должен доезжать до ArcFace'
    assert rec.stats()['crop_rejected_total'] == 1


def test_normal_crop_not_rejected_by_quality_gate():
    embedder = FakeEmbedder()
    store = FakeStore()
    rec = FaceRecognizer(embedder=embedder, store=store)
    frame = make_frame()  # шумный кадр — по умолчанию проходит ворота

    dets = [make_detection(w=0.5, h=0.5)]
    rec.process(dets, frame, now=0.0)

    assert embedder.embed_calls == [1], 'годный кроп обязан дойти до ArcFace'
    assert rec.stats()['crop_rejected_total'] == 0


def test_phantom_shadow_track_never_records_encounter():
    """Прямое воспроизведение issue #2749: трек промоутится геометрией
    (Встреча состоялась по правилам ADR-0123 §3), но ни один кроп трека
    не проходит ворота качества -> эмбеддинга у Встречи нет -> запись в
    FaceStore не создаётся ("выдумка не доезжает до Личности").
    """
    tracker = FaceTracker(min_track_sec=1.0, min_face_px=48.0, max_gap_sec=5.0)
    embedder = FakeEmbedder()
    store = FakeStore()
    log = RecordingLog()
    rec = FaceRecognizer(embedder=embedder, store=store, tracker=tracker, log_fn=log)
    frame = make_black_frame()

    for t in (0.0, 0.5, 1.0):
        rec.process([make_detection(w=0.5, h=0.5)], frame, now=t)

    assert store.record_encounter_calls == [], (
        'фантом issue #2749: почти чёрный кроп не должен родить запись в базе'
    )
    assert rec.stats()['encounters_total'] == 1, 'трек всё же промоутнулся геометрией'
    assert rec.stats()['crop_rejected_total'] >= 1
    assert log.has('warn')


def test_crop_rejected_total_accumulates_across_frames():
    embedder = FakeEmbedder()
    store = FakeStore()
    rec = FaceRecognizer(embedder=embedder, store=store)
    frame = make_black_frame()

    for t in (0.0, 0.5, 1.0):
        rec.process([make_detection(w=0.5, h=0.5)], frame, now=t)

    assert rec.stats()['crop_rejected_total'] == 3


def test_crop_quality_thresholds_are_configurable_not_hardcoded():
    """Пороги — параметры конструктора (issue #2749), а не константа в
    коде: выставив их в 0, даже чёрный кроп обязан пройти ворота.
    """
    embedder = FakeEmbedder()
    store = FakeStore()
    rec = FaceRecognizer(
        embedder=embedder, store=store, min_crop_mean=0.0, min_crop_contrast=0.0,
    )
    frame = make_black_frame()

    dets = [make_detection(w=0.5, h=0.5)]
    rec.process(dets, frame, now=0.0)

    assert embedder.embed_calls == [1], 'при min_crop_mean=0/min_crop_contrast=0 ворота открыты'
    assert rec.stats()['crop_rejected_total'] == 0
