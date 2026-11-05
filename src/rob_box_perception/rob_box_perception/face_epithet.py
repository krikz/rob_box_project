#!/usr/bin/env python3
"""face_epithet.py — кличка лица при рождении записи (ADR-0164 §4.1, этап 6).

Шифу 05.11 (В2): новое лицо получает кличку сразу, по внешности. Порядок:

1. Сразу, синхронно — словарная кличка (одно слово, ``persona.fallback_epithet``). У записи всегда есть
   метка, даже если облака нет (тот же порядок, что у голоса: словарь в БД до запроса к LLM).
2. Если режим разрешает облако (``FaceStore.epithet_policy == vision``, только ``workshop``), снимок прошёл
   фильтр качества, vision-LLM настроена и потолок вызовов в час (``persona.CallBudget``) не выбран — в
   фоне описание по внешности и кличка из него. Ответ проверяет и решает ``rob_box_core.persona``, пишет
   ``FaceStore.set_epithet`` (ADR-0148).

Каждое решение — строка ``[лицо] кличка …`` в логе с причиной: тихой деградации нет (capability-honest).

Фильтр качества (ADR-0164 §4.1 «Цена и мусор»): вызывать модель только по снимку, на котором есть пять
точек лица и лицо не мельче ``min_face_px``. AprilTag и пустой кадр точек лица не дают.

Вызов модели не блокирует конвейер камеры: он уходит в ``runner`` (по умолчанию — один фоновый поток).
"""

from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Callable, Dict, Iterable, Optional

from rob_box_core import persona

from rob_box_perception.face_store import EPITHET_POLICY_NONE, EPITHET_POLICY_VISION

#: ``describer(jpeg, prompt, schema) -> ответ модели`` (строка JSON или dict).
Describer = Callable[[bytes, str, Dict[str, Any]], Any]

#: Длина плоского списка точек лица RetinaFace: 5 точек × (x, y).
LANDMARK_VALUES = 10
#: Минимальная короткая сторона лица для описания по внешности, px. Совпадает с порогом Встречи
#: ``min_face_px`` ноды (48): мельче — описывать нечего. Не калибровано на роботе.
DEFAULT_MIN_FACE_PX = 48.0

SKIP_MODE = 'mode'
SKIP_QUALITY = 'quality'
SKIP_UNCONFIGURED = 'unconfigured'
SKIP_BUDGET = 'budget'


def snapshot_quality_ok(snapshot: Optional[bytes], landmarks: Any, face_px: float,
                        min_face_px: float = DEFAULT_MIN_FACE_PX) -> bool:
    """Снимок годится для описания: JPEG есть, пять точек лица, лицо не мельче порога."""
    if not snapshot:
        return False
    try:
        if landmarks is None or len(landmarks) < LANDMARK_VALUES:
            return False
    except TypeError:
        return False
    return float(face_px) >= float(min_face_px)


class _ThreadRunner:
    """Один фоновый поток: вызовы модели идут по одному и не держат кадр."""

    def __init__(self) -> None:
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='face-epithet')

    def __call__(self, fn: Callable[[], None]) -> None:
        self._pool.submit(fn)

    def close(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)


class FaceEpithetNamer:
    """Выдаёт кличку новой лицевой записи и, где можно, уточняет её по внешности."""

    def __init__(
        self,
        store: Any,
        *,
        describer: Optional[Describer] = None,
        budget: Optional[persona.CallBudget] = None,
        session_id: str = '',
        min_face_px: float = DEFAULT_MIN_FACE_PX,
        foreign_taken: Optional[Callable[[], Iterable[str]]] = None,
        runner: Optional[Callable[[Callable[[], None]], None]] = None,
        clock: Callable[[], float] = time.time,
        log_fn: Optional[Callable[[str, str], None]] = None,
    ) -> None:
        self._store = store
        self._describer = describer
        self._budget = budget if budget is not None else persona.CallBudget()
        self._session_id = session_id or f'vision-face-{int(clock())}'
        self._min_face_px = float(min_face_px)
        # Клички голосовых профилей (второе хранилище). Обмен списком — этап 9; до него пусто.
        self._foreign_taken = foreign_taken or (lambda: ())
        self._runner = runner if runner is not None else _ThreadRunner()
        self._clock = clock
        self._log_fn = log_fn
        self._lock = threading.Lock()
        self._counts: Dict[str, int] = {
            'epithet_dictionary_total': 0,
            'epithet_vision_calls': 0,
            'epithet_vision_accepted': 0,
            'epithet_vision_rejected': 0,
            'epithet_vision_errors': 0,
            f'epithet_skip_{SKIP_MODE}': 0,
            f'epithet_skip_{SKIP_QUALITY}': 0,
            f'epithet_skip_{SKIP_UNCONFIGURED}': 0,
            f'epithet_skip_{SKIP_BUDGET}': 0,
        }

    # ------------------------------------------------------------------

    def _log(self, level: str, msg: str) -> None:
        if self._log_fn is not None:
            try:
                self._log_fn(level, f'[лицо] {msg}')
            except Exception:  # noqa: BLE001 — лог не роняет узнавание
                pass

    def _count(self, key: str) -> None:
        with self._lock:
            self._counts[key] += 1

    def _epithets(self) -> Dict[str, Optional[str]]:
        """Клички записей хранилища (``people()``) — без отдельного метода в FaceStore (ADR-0145)."""
        return {p['person_id']: p.get('epithet') for p in self._store.people()}

    def _current(self, person_id: str) -> Optional[str]:
        return self._epithets().get(person_id)

    def _taken(self, person_id: str) -> list:
        own = {pid: label for pid, label in self._epithets().items() if pid != person_id}
        return persona.taken_union(own.values(), self._foreign_taken())

    def _material(self, person_id: str, encounter_count: int, has_snapshot: bool) -> Any:
        return persona.collect_material(
            person_id, self._session_id, has_snapshot=has_snapshot, encounter_count=encounter_count,
        )

    def _vision_skip(self, policy: str, quality_ok: bool) -> Optional[str]:
        if policy != EPITHET_POLICY_VISION:
            return SKIP_MODE
        if not quality_ok:
            return SKIP_QUALITY
        if self._describer is None:
            return SKIP_UNCONFIGURED
        if not self._budget.try_acquire(self._clock()):
            return SKIP_BUDGET
        return None

    # ------------------------------------------------------------------

    def on_encounter(self, match: Any, encounter: Any, snapshot: Optional[bytes]) -> str:
        """Встреча из ``FaceRecognizer``: новой записи — кличка сразу, известной — её кличка ('' — нет).

        Ошибка именования не роняет Встречу: строка в лог, маркер уходит без клички.
        """
        if not match.is_new:
            return getattr(match, 'epithet', None) or ''
        best = encounter.best[0] if encounter.best else None
        try:
            label = self.on_new_face(
                match.person_id, snapshot=snapshot, landmarks=getattr(best, 'landmarks', None),
                face_px=float(encounter.max_face_px), encounter_count=int(match.encounter_count),
            )
        except Exception as exc:  # noqa: BLE001
            self._log('error', f'кличка {match.person_id[:8]}: {exc!r}')
            return ''
        return label or ''

    def on_new_face(
        self,
        person_id: str,
        *,
        snapshot: Optional[bytes],
        landmarks: Any,
        face_px: float,
        encounter_count: int = 1,
    ) -> Optional[str]:
        """Новая запись лица: словарная кличка сразу; описание по внешности — в фоне, если можно.

        Returns:
            Кличка, записанная сразу (``None`` — в этом режиме кличек нет или запись уже исчезла).
        """
        policy = self._store.epithet_policy
        if policy == EPITHET_POLICY_NONE:
            return None
        quality_ok = snapshot_quality_ok(snapshot, landmarks, face_px, self._min_face_px)
        skip = self._vision_skip(policy, quality_ok)
        material = self._material(person_id, encounter_count, has_snapshot=quality_ok)
        unavailable = f'описание по внешности не запрошено: {skip}' if skip else 'до ответа vision-LLM'
        decision = persona.decide(
            material, None, current=self._current(person_id),
            taken=self._taken(person_id), unavailable=unavailable,
        )
        if not self._store.set_epithet(person_id, decision.epithet, decision.reason):
            return None
        if decision.changed:
            self._count('epithet_dictionary_total')
        if skip:
            self._count(f'epithet_skip_{skip}')
        self._log('info', persona.decision_log_line(person_id[:8], decision))
        if skip is None:
            self._runner(lambda: self._describe(person_id, snapshot, material))
        return decision.epithet

    def _describe(self, person_id: str, snapshot: bytes, material: Any) -> None:
        """Фон: vision-LLM по снимку → проверка → решение → запись. Ошибка — словарная кличка остаётся."""
        self._count('epithet_vision_calls')
        current = self._current(person_id)
        try:
            raw = self._describer(
                snapshot, persona.build_proposal_prompt(material, current_epithet=current),
                persona.PROPOSAL_SCHEMA,
            )
        except Exception as exc:  # noqa: BLE001 — сеть/деньги/таймаут: честная строка, кличка прежняя
            self._count('epithet_vision_errors')
            self._log('warn', f'кличка {person_id[:8]}: vision-LLM не ответила ({exc!r}), '
                              f'остаётся «{current}»')
            return
        verdict = persona.check_proposal(
            raw, material, taken=self._taken(person_id), now=self._clock(),
        )
        decision = persona.decide(material, verdict, current=current, taken=self._taken(person_id))
        self._count('epithet_vision_accepted' if verdict.epithet else 'epithet_vision_rejected')
        description = decision.description.to_meta() if decision.description else None
        if self._store.set_epithet(person_id, decision.epithet, decision.reason,
                                   persona_meta=description):
            self._log('info', persona.decision_log_line(person_id[:8], decision))

    def stats(self) -> Dict[str, int]:
        with self._lock:
            out = dict(self._counts)
        out['epithet_vision_budget_left'] = self._budget.remaining(self._clock())
        return out

    def close(self) -> None:
        close = getattr(self._runner, 'close', None)
        if close is not None:
            close()


class _NoNamer:
    """Клички выключены: новым записям не выдаём, у известных отдаём ту, что уже есть."""

    def on_encounter(self, match: Any, encounter: Any, snapshot: Optional[bytes]) -> str:
        return getattr(match, 'epithet', None) or ''

    def stats(self) -> Dict[str, int]:
        return {}


#: Неймер по умолчанию у ``FaceRecognizer`` — без ветвлений в самом распознавателе.
NO_NAMER = _NoNamer()


__all__ = [
    'DEFAULT_MIN_FACE_PX',
    'Describer',
    'FaceEpithetNamer',
    'NO_NAMER',
    'snapshot_quality_ok',
]
