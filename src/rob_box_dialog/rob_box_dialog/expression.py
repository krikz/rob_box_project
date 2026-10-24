"""Рефлексы: стадия хода -> экспрессия без LLM (ADR-0150 §5.5.1-5.5.3; ADR-0148 «код решает»).

:func:`reflex` читает таблицу :data:`.knowledge.REFLEXES` и отдаёт :class:`ExpressionPlan` - что показать лицу и какой
earcon сыграть на этой стадии. :func:`merge` накладывает эмоцию команды (в v1 - аргумент ``animation`` тулов речи): на
``speaking`` и ``done`` она красит рефлекс, на ``failed`` и ``timeout`` рефлекс побеждает -
«радостно» о провале не бывает.
Кольцо LED в план не входит: его режим исполнитель берёт из ``REFLEXES[stage].ring`` по событию стадии.

Чистый модуль, без ROS: план превращается в ``ExpressionRequest`` клиентом ``rob_box_core.expression_client``.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Mapping, Optional

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


def _bpm_scene(bpm: Optional[float]) -> Optional[str]:
    if not bpm or bpm <= 0:
        return None
    return next(scene for limit, scene in kn.MUSIC_SCENE_BPM if bpm < limit)


def _executing_face(cls: Optional[str], snapshot: Mapping[str, object]) -> str:
    """Лицо на ``executing``: движение - по направлению, музыка - по темпу после ``started``, прочее - как «думает»."""
    if cls == "action.motion":
        return MOTION_FACE.get(str(snapshot.get("direction") or ""), MOTION_DEFAULT_FACE)
    if cls == "action.media" and snapshot.get("started"):
        return _bpm_scene(snapshot.get("bpm")) or kn.EXECUTING_FACE[cls][0]  # type: ignore[arg-type]
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


def merge(plan: Optional[ExpressionPlan], stage: str, emotion: Optional[str] = None,
          gesture: Optional[str] = None) -> Optional[ExpressionPlan]:
    """Наложить эмоцию и жест команды на рефлекс; на ``failed``/``timeout`` и вне ``EMOTION_STAGES`` рефлекс как есть."""
    if plan is None or stage not in EMOTION_STAGES:
        return plan
    scene = command_scene(emotion)
    if scene:
        plan = replace(plan, emotion=scene)
    if gesture and gesture in kn.GESTURES:
        plan = replace(plan, gesture=gesture)
    return plan
