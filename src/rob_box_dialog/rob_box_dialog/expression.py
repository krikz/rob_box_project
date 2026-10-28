"""Рефлексы: стадия хода -> экспрессия без LLM (ADR-0150 §5.5.1-5.5.3; ADR-0148 «код решает»).

:func:`reflex` читает таблицу :data:`.knowledge.REFLEXES` и отдаёт :class:`ExpressionPlan` - что показать лицу и какой
earcon сыграть на этой стадии. :func:`merge` накладывает эмоцию, жест и сцену лица команды (в v1 - аргумент
``animation`` тулов речи, в v2 - ``command.expression`` и сцена исполненного ``play_animation``, #3997, #4049): на
``speaking`` и ``done`` она красит рефлекс, на ``failed`` и ``timeout`` рефлекс побеждает - «радостно» о провале не
бывает.
Кольцо LED в план не входит: его режим исполнитель берёт из ``REFLEXES[stage].ring`` по событию стадии.

Показ по просьбе человека (#4083): время сцены - :func:`show_seconds` (не меньше ``FACE_SHOW_MIN_S``), пачка сцен
одной команды - :func:`queued`, по очереди, каждая своё время, а не перезаписью последней; сцена показа -
:func:`shown_scene` (#4103: «нейтральное лицо» - лицо покоя на своё время, а не рефлекс ``happy``).

Чистый модуль, без ROS: план превращается в ``ExpressionRequest`` клиентом ``rob_box_core.expression_client``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from . import knowledge as kn

#: Эмоции команды, которые «ничего не просят»: нейтраль и сцены, которыми лицо и так живёт (покой, рот при речи).
NO_EMOTION = frozenset({"", "neutral", "idle", "talking"})
#: Стадии, на которые эмоция команды накладывается; на остальных (в том числе на провале) решает рефлекс.
EMOTION_STAGES = ("speaking", "done")
#: Сцена лица на ``executing`` движения по направлению из аргументов тула; без направления - ``MOTION_DEFAULT_FACE``.
MOTION_FACE = {"left": "turn_left", "right": "turn_right"}
MOTION_DEFAULT_FACE = "accelerating"


@dataclass(frozen=True)
class ExpressionPlan:
    """Что код просит у исполнителей на стадии: сцена лица, жест, earcon, время жизни, слоты LLM; пусто - не менять."""

    emotion: str = ""
    gesture: str = ""
    earcon: str = ""
    ttl_ms: int = 0
    #: Сцена лица от LLM (``FaceScene.to_dict()``, уже проверенная); пусто - нет (§5.5.9).
    face: Optional[Mapping[str, object]] = None


def music_scene(bpm: Optional[float], track_no: int = 0, elapsed_s: float = 0.0) -> str:
    """Музыкальная сцена лица (#4010) - единственное место, где она выбирается (``face_director`` и ``executing`` ниже).

    Трек начинает сцена по темпу (``MUSIC_SCENE_BPM``; темпа нет - ``MUSIC_DEFAULT_BPM``), дальше сцены ходят по кругу
    ``MUSIC_SCENES`` независимо от темпа: ``track_no`` - сколько раз трек сменился с начала музыки (каждый - шаг),
    ``elapsed_s`` - сколько играет текущий (шаг каждые ``MUSIC_SCENE_HOLD_S``). Чистая функция: тот же вход - та же
    сцена, ``face_director`` только держит пол ``MUSIC_SCENE_MIN_S``."""
    tempo = bpm if bpm and bpm > 0 else kn.MUSIC_DEFAULT_BPM
    first = next(i for i, (limit, _) in enumerate(kn.MUSIC_SCENE_BPM) if tempo < limit)
    step = max(0, int(track_no)) + int(max(0.0, elapsed_s) // kn.MUSIC_SCENE_HOLD_S)
    return kn.MUSIC_SCENES[(first + step) % len(kn.MUSIC_SCENES)]


def _executing_face(cls: Optional[str], snapshot: Mapping[str, object]) -> str:
    """Лицо на ``executing``: движение - по направлению, музыка - по темпу после ``started``, прочее - как «думает»."""
    if cls == "action.motion":
        return MOTION_FACE.get(str(snapshot.get("direction") or ""), MOTION_DEFAULT_FACE)
    if cls == "action.media" and snapshot.get("started"):
        return music_scene(snapshot.get("bpm"))  # type: ignore[arg-type]
    return kn.REFLEXES["executing"].face[0]


def _face(stage: str, cls: Optional[str], snapshot: Mapping[str, object]) -> str:
    """Сцена лица стадии; несколько вариантов в таблице чередуются по номеру хода (``snapshot['turn_no']``)."""
    if stage == "executing":
        return _executing_face(cls, snapshot)
    faces = kn.REFLEXES[stage].face
    if not faces:
        return ""
    return faces[int(snapshot.get("turn_no") or 0) % len(faces)]  # type: ignore[call-overload]


def reflex(stage: str, cls: Optional[str] = None, snapshot: Optional[Mapping[str, object]] = None
           ) -> Optional[ExpressionPlan]:
    """Экспрессия рефлекса стадии хода; ``None`` - у стадии рефлекса нет (или ему нечего показать и сыграть)."""
    row = kn.REFLEXES.get(stage)
    if row is None or stage not in kn.TURN_STAGES:
        return None
    face = _face(stage, cls, snapshot or {})
    if not face and not row.earcon:
        return None
    return ExpressionPlan(emotion=face, earcon=row.earcon or "", ttl_ms=row.ttl_ms or 0)


def silent_face() -> str:
    """Лицо в тишине (#3984): сцена из ``REFLEXES["silenced"]`` (заклеенный рот) - вместо «слушаю/думаю/радость»."""
    return kn.REFLEXES["silenced"].face[0]


def silent_earcon(earcon: Optional[str]) -> str:
    """Earcon стадии в тишине (#3984): по ``SILENCED_EARCONS`` - «услышал» становится тихим ``muffled``, «готово» молчит.

    Неизвестный earcon в тишине не играет (``""``): громкий звук нельзя протащить мимо таблицы."""
    return kn.SILENCED_EARCONS.get(earcon or "", "")


def session_silence(state: object) -> Optional[bool]:
    """Снимок сессии (``SessionState``) -> «в кадрах сцена ``Silence``?» для голоса личности; чужой агент - ``None``
    (его снимок о голосе не говорит). Одно правило для лица, звука и кольца (как ``rob_box_core.voice_state``)."""
    if getattr(state, "agent", "") != kn.AGENTS[0]:
        return None
    return any(getattr(scene, "kind", "") == "Silence" for scene in getattr(state, "scenes", ()))


def command_scene(emotion: Optional[str]) -> str:
    """Сцена лица для эмоции команды: ``EMOTIONS`` переводит эмоцию в сцену, имя сцены проходит как есть."""
    name = (emotion or "").strip().lower()
    if name in NO_EMOTION:
        return ""
    return kn.EMOTIONS.get(name, name)


def shown_scene(animation: Optional[str]) -> str:
    """Сцена лица исполненного ``play_animation`` - сцены, которую человек попросил показать (#4103). Нейтраль
    (``idle``, ``neutral``: «покажи по очереди нейтральное лицо и грусть») - лицо покоя (первая сцена рефлекса
    ``idle``) на своё время, а не «нечего показать» (:data:`NO_EMOTION` - правило экспрессии команды, где нейтраль -
    значение по умолчанию): иначе итог ``done`` красил её рефлексом ``happy`` (живая проверка 27.10 18:55). Остальные
    имена - как есть: их уже проверил тул по словарю ``knowledge.KNOWN_ANIMATIONS``."""
    name = (animation or "").strip().lower()
    return kn.REFLEXES["idle"].face[0] if kn.EMOTIONS.get(name, name) == kn.EMOTIONS["neutral"] else name


def shows(emotion: Optional[str], gesture: Optional[str] = None, face: Optional[object] = None) -> bool:
    """Покажет ли экспрессия команды что-то на лице: эмоция сверх ``NO_EMOTION``, жест из ``GESTURES`` или сцена LLM.
    Нейтраль без жеста и сцены - нечего показать (команда из неё - пустая, а не «лицо без слов», #3997)."""
    return bool(command_scene(emotion) or (gesture and gesture in kn.GESTURES) or face)


def face_label(emotion: Optional[str], gesture: Optional[str] = None, face: Optional[object] = None) -> str:
    """Лицо, которое покажет экспрессия команды, словами кода для окна LLM (``knowledge.WINDOW_FACE_REPLY``, #4073): сцена
    эмоции (:func:`command_scene`), жест из ``GESTURES``, «сцена лица» для сцены LLM - через запятую. Показывать нечего
    (:func:`shows` ложно) - пусто."""
    parts = [command_scene(emotion), gesture if gesture in kn.GESTURES else "", kn.WINDOW_FACE_SCENE if face else ""]
    return ", ".join(dict.fromkeys(part for part in parts if part))  # «victory» бывает и эмоцией, и жестом


def merge(plan: Optional[ExpressionPlan], stage: str, emotion: Optional[str] = None, gesture: Optional[str] = None,
          face: Optional[Mapping[str, object]] = None, ttl_ms: Optional[int] = None) -> Optional[ExpressionPlan]:
    """Наложить эмоцию, жест и сцену лица команды (``FaceScene.to_dict()``, уже проверенная) на рефлекс; на
    ``failed``/``timeout`` и вне ``EMOTION_STAGES`` рефлекс как есть. ``ttl_ms`` - сколько держать сцену команды
    (``duration`` исполненного ``play_animation``, #4049; ``0`` - по умолчанию лица): только вместе с её сценой - время
    рефлекса без сцены команды не меняется."""
    if plan is None or stage not in EMOTION_STAGES:
        return plan
    scene = command_scene(emotion)
    if scene:
        plan = replace(plan, emotion=scene)
    if gesture and gesture in kn.GESTURES:
        plan = replace(plan, gesture=gesture)
    if face:
        plan = replace(plan, face=dict(face))
    if ttl_ms is not None and (scene or face):
        plan = replace(plan, ttl_ms=max(0, int(ttl_ms)))
    return plan


def show_seconds(requested: object = None) -> float:
    """Сколько лицо держит сцену, которую человек попросил показать (#4083, Шифу 27.10: «хотя бы 5–10 секунд»):
    заказанное время (``play_animation.duration``, ``expression.duration_s``, ``face.duration_s``) в пределах
    ``[FACE_SHOW_MIN_S, FACE_SCENE_MAX_S]``; не задано (не число, не больше нуля) - ``FACE_SHOW_DEFAULT_S``.
    Единственное правило времени показа: его применяет ход (``turn.command_face``), время доходит до
    ``ExpressionRequest.ttl_ms`` и ``duration_s`` сцены лица."""
    if isinstance(requested, bool) or not isinstance(requested, (int, float)) or not requested > 0:
        return kn.FACE_SHOW_DEFAULT_S
    return min(max(float(requested), kn.FACE_SHOW_MIN_S), kn.FACE_SCENE_MAX_S)


def held_face(face: Optional[Any], seconds: Optional[float]) -> Optional[Dict[str, Any]]:
    """Сцена лица LLM (``FaceScene``) для ``ExpressionRequest.face_json``: со временем сцены команды ``seconds``, если
    оно задано (оверлей гаснет вместе со сценой, а не раньше и не позже); нет сцены - ``None``."""
    if face is None:
        return None
    out = dict(face.to_dict())
    if seconds is not None:
        out["duration_s"] = seconds
    return out


def hold_ms(seconds: Optional[float]) -> Optional[int]:
    """Время сцены команды в ``ttl_ms``; ``None`` - время рефлекса (обычный ответ, показ не просили)."""
    return None if seconds is None or not math.isfinite(seconds) else int(round(seconds * 1000))


def queued(start_s: float, scenes: Sequence[Any]) -> Tuple[Tuple[float, ExpressionPlan], ...]:
    """Пачка сцен одной команды по очереди (#4083: «покажи по очереди радость, грусть и удивление» - три
    ``play_animation`` исполнялись разом, последняя перекрывала остальные). ``(через сколько секунд после итога,
    план)`` для сцен после первой: следующая начинается, когда кончилось время предыдущей (``start_s`` - время первой,
    уже на лице с итогом ``done``). Время сцены - :func:`show_seconds` (не меньше ``FACE_SHOW_MIN_S``). Earcon и жест -
    только у итога: сцена очереди - эмоция (имя сцены), её сцена лица и время. Нейтраль показа приходит сюда лицом
    покоя (:func:`shown_scene`, #4103) и шлётся, как любая сцена: «нейтральное лицо» держится своё время. Сцена, которой
    нечего показать (:func:`shows`: ``talking`` без речи, экспрессия без эмоции и сцены лица), запроса не шлёт, но своё
    время держит: следующая - после неё в срок."""
    out, at = [], show_seconds(start_s)
    for scene in scenes:
        seconds = show_seconds(getattr(scene, "duration_s", None))
        emotion, face = command_scene(getattr(scene, "emotion", "")), held_face(getattr(scene, "face", None), seconds)
        if emotion or face:
            out.append((at, ExpressionPlan(emotion=emotion, face=face, ttl_ms=hold_ms(seconds) or 0)))
        at += seconds
    return tuple(out)
