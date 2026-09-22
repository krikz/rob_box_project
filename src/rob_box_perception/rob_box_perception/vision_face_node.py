#!/usr/bin/env python3
"""vision_face_node — детекция и узнавание лица на AI HAT+ (ADR-0089 Phase 2).

Сабкласс ``VisionHailoNode``: переиспользует всю инфраструктуру ноды
(шов «Взгляд» gaze.py, поллинг кадров, degraded-state policy, публикацию
``VisionEvent``), меняя только три вещи:

  1. HEF loader — ``make_face_loader`` (RetinaFace вместо YOLOv8n),
  2. имя ноды и дефолтный confidence threshold,
  3. **узнавание** (PR-B): поверх детектора надевается
     :class:`RecognizingFaceLoader`, который считает ArcFace-эмбеддинг,
     ведёт треки и превращает их во Встречи (ADR-0123 §3).

Публикует ``VisionEvent`` с ``event_type="face"`` + bbox на тот же топик
``/vision/hailo/events``. Начиная с PR-B там же приезжают:

* ``embedding_id`` — стабильный id человека в лицевом хранилище;
* ``display_name`` — имя, если оно уже привязано;
* ``attributes_json`` — на кадре, где ТРЕК СТАЛ ВСТРЕЧЕЙ, лежит маркер
  ``{"encounter": "start", ...}``. Это и есть повод заговорить
  (ADR-0102, ``Occasion(kind="meeting")``); потребитель обязан
  реагировать на маркер, а не на каждое ``event_type="face"``, иначе
  робот поздоровается пять раз в секунду (ADR-0123 §3).

**Почему узнавание живёт здесь, а не в шве идентичности.** Лицевая
биометрия — такой же адаптер, как голосовая (``VoiceIdentitySeam`` в
``rob_box_voice``): пакет, который владеет сырыми данными, владеет и их
сопоставлением. ``last_seen``/``seen_count`` эта нода НЕ ведёт — они
принадлежат шву «Знакомый» (ADR-0089 §8, ADR-0106, ADR-0123 §4.1), и
дублировать их в ``/data/faces/`` прямо запрещено: ровно так голос и
лицо разъехались бы по несвязанным ключам.

**Слияние с голосом** (ADR-0123 §6): нода слушает
``/voice/speaker/result`` и, если голос уверенно опознал человека, а в
кадре ровно одно лицо, привязывает имя к лицевой записи. Два лица в
кадре — слияния нет (открытый вопрос ADR-0105 §3 п.4).

Stub-режим (hailo_enabled=false): ``StubHEFLoader`` публикует
``event_type="stub"`` (маркер выдуманного события, ADR-0089 §2.2) —
выдумка не должна доезжать до Личности. Узнавание поверх stub'а не
надевается вовсе: узнавать нечего.

Touchpoints:
- ADR-0089 §2.1 Phase 2, ADR-0123 (режимы приватности, Встреча, слияние).
- Issue #2599 PR-A (детекция) / PR-B (эмбеддинги и узнавание).
- rob_box_perception.face_recognition / face_store / face_tracker / face_embedding.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

from rob_box_perception.vision_face_loader import make_face_loader
from rob_box_perception.vision_hailo_loader import HEFLoader
from rob_box_perception.vision_hailo_node import VisionHailoNode

#: Топик голосовой биометрии (std_msgs/String, JSON SpeakerMatch).
SPEAKER_RESULT_TOPIC = '/voice/speaker/result'

#: Как часто печатать сводку узнавания в лог ноды. Режим приватности
#: обязан быть виден (ADR-0123 §2), а «сколько встреч и кого узнали» —
#: единственный способ понять, работает ли узнавание, не лазая в /data.
STATS_LOG_PERIOD_SEC = 60.0


class RecognizingFaceLoader(HEFLoader):
    """Декоратор детектора: bbox'ы от RetinaFace + имена от ArcFace.

    Почему декоратор, а не правка ``_tick`` ноды: базовый класс уже
    отдаёт лоадеру и ``frame_id``, и сам кадр, а его degraded-state
    policy (ADR-0018, issue #2538 п.6) умеет гасить шторм ошибок. Если
    узнавание упадёт, оно упадёт внутри ``infer`` — и нода обойдётся с
    ним ровно так же, как с падением детектора, без второго набора
    правил.

    Args:
        inner: реальный детектор (``RetinaFaceLoader``).
        recognizer: :class:`~rob_box_perception.face_recognition.FaceRecognizer`.
    """

    def __init__(self, inner: HEFLoader, recognizer: Any) -> None:
        self._inner = inner
        self._recognizer = recognizer

    def infer(self, frame_id: str, image: Any) -> List[Dict[str, Any]]:
        detections = self._inner.infer(frame_id=frame_id, image=image)
        if not detections:
            # Пустая комната — треки всё равно надо состарить, иначе
            # ушедший человек останется «живым» и следующая Встреча
            # не поднимется.
            self._recognizer.process([], image)
            return detections
        return self._recognizer.process(detections, image)

    @property
    def recognizer(self) -> Any:
        return self._recognizer


class VisionFaceNode(VisionHailoNode):
    """Нода лица: детекция (RetinaFace) + узнавание (ArcFace)."""

    NODE_NAME: str = 'vision_face'

    #: RetinaFace confidence threshold (hailo_models.yaml: 0.6).
    DEFAULT_CONFIDENCE_THRESHOLD: float = 0.6

    def __init__(self, **kwargs: Any) -> None:
        # Параметры узнавания объявляем ДО super().__init__(): базовый
        # класс собирает loader внутри себя (`_make_loader`), и к этому
        # моменту значения уже должны быть на руках.
        self._face_params_declared = False
        super().__init__(**kwargs)

        self._recognizer: Any = getattr(self._loader, 'recognizer', None)
        self._speaker_sub: Any = None
        self._stats_timer: Any = None
        # Issue #2748 — «слияний=0» в сводке не отличало «некого сливать»
        # (голос ещё не опознан / is_known=false) от «не с чем» (причины
        # внутри note_voice_identification — считает face_recognition.py).
        # Этот счётчик — та самая недостающая ПЕРВАЯ причина: она
        # обрывается ЗДЕСЬ, до вызова note_voice_identification, поэтому
        # recognizer её никогда не видит.
        self._speaker_unknown_total: int = 0

        if self._recognizer is not None:
            self._subscribe_speaker_result()
            self._stats_timer = self.create_timer(
                STATS_LOG_PERIOD_SEC, self._log_stats
            )

    # ------------------------------------------------------------------
    # Параметры
    # ------------------------------------------------------------------

    def _declare_face_params(self) -> None:
        """Параметры узнавания (start_vision_face.sh отдаёт их из ENV/YAML)."""
        if self._face_params_declared:
            return
        self._face_params_declared = True
        self.declare_parameter('arcface_enabled', False)
        self.declare_parameter('arcface_hef_path', '')
        self.declare_parameter('face_store_root', '/data/faces')
        self.declare_parameter('face_privacy_mode', 'workshop')
        # ЗАГЛУШКА до sweep по ADR-0123 §6 (issue #2771), не калиброванные
        # числа. Прежний дефолт 0.45 пускал чужих (тёща опознана как
        # «Деньчик» при score=0.483, issue #2771). identify — «похож
        # достаточно, чтобы назвать имя»; enroll — заметно строже,
        # «похож достаточно, чтобы дописать эмбеддинг в галерею» (issue
        # #2772) — см. докстринг конструктора FaceStore.
        self.declare_parameter('face_identify_threshold', 0.6)
        self.declare_parameter('face_enroll_threshold', 0.75)
        self.declare_parameter('min_track_sec', 2.0)
        self.declare_parameter('min_face_px', 48.0)
        # Ворота качества кропа (issue #2749): почти чёрный/плоский кроп
        # не эмбеддится и не пишется в FaceStore — см. докстринг и
        # DEFAULT_MIN_CROP_MEAN/DEFAULT_MIN_CROP_CONTRAST в
        # face_recognition.py про то, откуда взяты числа 20.0/25.0.
        self.declare_parameter('min_crop_mean', 20.0)
        self.declare_parameter('min_crop_contrast', 25.0)
        self.declare_parameter('max_embeds_per_frame', 4)
        self.declare_parameter('max_embeddings', 20)
        self.declare_parameter('keep_encounters', 10)
        self.declare_parameter('max_strangers', 500)

    # ------------------------------------------------------------------
    # Loader
    # ------------------------------------------------------------------

    def _make_loader(self) -> HEFLoader:
        self._declare_face_params()

        detector = make_face_loader(
            hailo_enabled=self.hailo_enabled,
            hef_path=self.hef_path,
            stub_period_sec=self.stub_period_sec,
            confidence_threshold=self.confidence_threshold,
            nms_iou_threshold=self.nms_iou_threshold,
        )

        arcface_enabled = bool(self.get_parameter('arcface_enabled').value)
        arcface_hef = str(self.get_parameter('arcface_hef_path').value).strip()

        if not self.hailo_enabled:
            # Stub-режим: узнавать нечего, выдумку метить именами нельзя.
            self.get_logger().info(
                'Узнавание лица выключено: нода в stub-режиме '
                '(hailo_enabled=false) — событий "face" не будет.'
            )
            return detector

        if not arcface_enabled or not arcface_hef:
            self.get_logger().warning(
                'Узнавание лица выключено (arcface_enabled=%s, hef=%r): '
                'детекция работает, имён не будет. Включается в '
                'docker/vision/.env → FACE_ARCFACE_ENABLED.'
                % (arcface_enabled, arcface_hef)
            )
            return detector

        try:
            recognizer = self._build_recognizer(arcface_hef)
        except Exception as exc:  # noqa: BLE001
            # Capability-honest (ADR-0018): не падаем и не выдумываем —
            # нода остаётся детектором, и в логе написано почему.
            self.get_logger().error(
                f'Не удалось поднять узнавание лица: {exc!r}. '
                f'Нода работает как детектор (bbox без имён).'
            )
            return detector

        return RecognizingFaceLoader(detector, recognizer)

    def _build_recognizer(self, arcface_hef: str) -> Any:
        """Собрать эмбеддер + хранилище + трекер. Импорты — локальные.

        Локальные, потому что тянут numpy/HailoRT: юнит-тесты ноды и
        ``--help`` не должны требовать железа.
        """
        from rob_box_perception.face_embedding import ArcFaceEmbedder
        from rob_box_perception.face_recognition import FaceRecognizer
        from rob_box_perception.face_store import FaceStore
        from rob_box_perception.face_tracker import FaceTracker

        mode = str(self.get_parameter('face_privacy_mode').value).strip() or 'workshop'
        root = str(self.get_parameter('face_store_root').value).strip() or '/data/faces'
        identify_threshold = float(
            self.get_parameter('face_identify_threshold').value
        )
        enroll_threshold = float(
            self.get_parameter('face_enroll_threshold').value
        )
        min_track_sec = float(self.get_parameter('min_track_sec').value)
        min_face_px = float(self.get_parameter('min_face_px').value)
        min_crop_mean = float(self.get_parameter('min_crop_mean').value)
        min_crop_contrast = float(self.get_parameter('min_crop_contrast').value)

        store = FaceStore(
            root=root,
            mode=mode,
            identify_threshold=identify_threshold,
            enroll_threshold=enroll_threshold,
            max_embeddings=int(self.get_parameter('max_embeddings').value),
            keep_encounters=int(self.get_parameter('keep_encounters').value),
            max_strangers=int(self.get_parameter('max_strangers').value),
        )
        tracker = FaceTracker(
            min_track_sec=min_track_sec,
            min_face_px=min_face_px,
        )
        embedder = ArcFaceEmbedder(hef_path=arcface_hef)

        recognizer = FaceRecognizer(
            embedder=embedder,
            store=store,
            tracker=tracker,
            max_embeds_per_frame=int(
                self.get_parameter('max_embeds_per_frame').value
            ),
            min_crop_mean=min_crop_mean,
            min_crop_contrast=min_crop_contrast,
            store_snapshots=(mode != 'strict'),
            log_fn=self._recognizer_log,
        )

        self.get_logger().info(
            'Узнавание лица включено: hef=%s, режим приватности=%s, '
            'хранилище=%s, identify_threshold=%.2f enroll_threshold=%.2f '
            '(заглушка до sweep #2771), Встреча≥%.1fс и ≥%.0fpx '
            '(ADR-0123). Людей в базе: %s.'
            % (
                arcface_hef,
                mode,
                root,
                identify_threshold,
                enroll_threshold,
                min_track_sec,
                min_face_px,
                store.stats().get('people', '?'),
            )
        )
        return recognizer

    def _recognizer_log(self, level: str, msg: str) -> None:
        logger = self.get_logger()
        if level == 'error':
            logger.error(msg)
        elif level == 'warn':
            logger.warning(msg)
        else:
            logger.info(msg)

    # ------------------------------------------------------------------
    # Слияние с голосом (ADR-0123 §6)
    # ------------------------------------------------------------------

    def _subscribe_speaker_result(self) -> None:
        """Подписка на голосовую биометрию — источник имени для лица.

        Имя лицу неоткуда взяться самому: ArcFace знает «это тот же
        человек, что и вчера», но не знает, что его зовут Денис. Имя
        приходит от голоса — это и есть слияние каналов ADR-0123 §6.
        """
        try:
            from std_msgs.msg import String
        except ImportError:  # pragma: no cover — вне ROS-окружения
            return
        try:
            self._speaker_sub = self.create_subscription(
                String, SPEAKER_RESULT_TOPIC, self._on_speaker_result, 10
            )
            self.get_logger().info(
                f'[слияние] подписка на {SPEAKER_RESULT_TOPIC} — имя для '
                f'лица приходит от голоса (ADR-0123 §6).'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'Не удалось подписаться на {SPEAKER_RESULT_TOPIC}: {exc!r}. '
                f'Лица будут узнаваться, но без имён.'
            )

    def _on_speaker_result(self, msg: Any) -> None:
        if self._recognizer is None:
            return
        try:
            payload = json.loads(msg.data)
        except (ValueError, AttributeError):
            return
        if not isinstance(payload, dict):
            return
        # Registration-ack (``{"event": "registered", ...}``) без is_known —
        # служебное сообщение speaker_id_node, не сигнал присутствия
        # (тот же контракт, что у dialogue_node/voice_adapter). Не считаем
        # его как «голос не опознан» — это отдельный, третий случай.
        if payload.get('event') == 'registered':
            return
        if not payload.get('is_known'):
            # Issue #2748 — причина №1, почему «слияний=0»: голос ЕЩЁ не
            # опознан (обычное identify() ниже порога, issue #2747) —
            # note_voice_identification() в этом случае вообще не
            # вызывается, поэтому считаем здесь, а не там.
            self._speaker_unknown_total += 1
            return
        speaker_id = str(payload.get('speaker_id') or '')
        name = str(payload.get('name') or '')
        if not speaker_id or not name:
            return
        # Issue #2748 — источник ``source="register"`` (speaker_id_node,
        # момент регистрации) принимается НАРАВНЕ с обычным узнаванием:
        # имя названо самим человеком, доверия к нему больше, чем к
        # косинусу identify(). Условие «ровно одно лицо в кадре»
        # (ADR-0105 §3 п.4) не отличает источники — оно живёт внутри
        # note_voice_identification() и остаётся строгим для ОБОИХ.
        try:
            self._recognizer.note_voice_identification(
                speaker_id=speaker_id, name=name
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Слияние голос→лицо упало: {exc!r}')

    # ------------------------------------------------------------------
    # Health
    # ------------------------------------------------------------------

    def _log_stats(self) -> None:
        if self._recognizer is None:
            return
        try:
            stats = self._recognizer.stats()
        except Exception:  # noqa: BLE001
            return
        store = stats.get('store', {}) or {}
        # crop_rejected_total (issue #2749): кропы, отброшенные воротами
        # качества (почти чёрные/плоские) ДО эмбеддинга — без счётчика в
        # сводке тихий фильтр стал бы вторым источником "робот меня не
        # видит" (см. докстринг FaceRecognizer._crop_quality_ok).
        #
        # Issue #2748 — «слияний=0» само по себе не говорит, ПОЧЕМУ. Причины
        # печатаем ТОЛЬКО когда есть хоть один пропуск — чтобы в спокойном
        # режиме (слияния идут штатно) строка не разбухала диагностикой,
        # которая никому не нужна.
        skip_reasons = (
            ('голос_не_опознан', self._speaker_unknown_total),
            ('нет_лица', stats.get('voice_merge_skip_no_face', 0)),
            ('два+_лица', stats.get('voice_merge_skip_multi_face', 0)),
            ('устарело', stats.get('voice_merge_skip_stale', 0)),
            ('конфликт_профилей', stats.get('voice_merge_skip_conflict', 0)),
        )
        skip_suffix = ''
        active_reasons = [f'{label}={count}' for label, count in skip_reasons if count]
        if active_reasons:
            skip_suffix = ' (пропуски: ' + ' '.join(active_reasons) + ')'
        # gallery_cohesion (issue #2772/#2775): медиана попарного косинуса
        # внутри галерей — сторожевой показатель отравления. None ("н/д")
        # значит «пока нет ни одной записи с ≥2 эмбеддингами», а не ошибку.
        cohesion = store.get('gallery_cohesion')
        cohesion_str = f'{cohesion:.3f}' if cohesion is not None else 'н/д'
        self.get_logger().info(
            '[лицо] режим=%s встреч=%d узнано=%d новых=%d слияний=%d%s '
            'ошибок_эмбеддинга=%d кропов_отброшено=%d треков=%d | '
            'в базе: людей=%s с_именем=%s gallery_cohesion=%s '
            'enroll_отклонено=%d'
            % (
                store.get('mode', '?'),
                stats.get('encounters_total', 0),
                stats.get('recognized_total', 0),
                stats.get('new_people_total', 0),
                stats.get('voice_merges_total', 0),
                skip_suffix,
                stats.get('embed_failures', 0),
                stats.get('crop_rejected_total', 0),
                stats.get('active_tracks', 0),
                store.get('people', '?'),
                store.get('named', '?'),
                cohesion_str,
                store.get('enroll_rejected_total', 0),
            )
        )


def main(args: Optional[List[str]] = None) -> None:
    """Entrypoint console_script (см. setup.py)."""
    import rclpy

    rclpy.init(args=args)
    node = VisionFaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
