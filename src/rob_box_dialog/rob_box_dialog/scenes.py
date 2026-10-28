"""Кадры стека сцен по событиям владельцев (ADR-0150 §7.2, §6.3; PR-10, A9, A10).

* ``DJ(set_id)`` и ``Persona(name, voice)`` кладёт код по latched-снимку плеера (``dj.enabled``, ``set_id``,
  ``persona``) и снимает, когда сет кончился (``idle``/``dj_set stop``). Снимок — те же события плеера в виде, который
  не теряется: потерянное ``idle`` не оставит кадр висеть.
* Снятие ``Persona`` (конец сета, явная команда, TTL, ``reset``) возвращает голос, который был до неё; ``reset`` —
  всегда голос робота по умолчанию (A10: голос и персона не переживают сессию). Голос применяет владелец речи
  (``tts_node``): здесь только просьба ``restore``, а какой голос реально звучит, сессия узнаёт из ``voice_changed``.
* :func:`apply_control` — ``DialogControl`` (``hold``/``resume``/``reset``): оператор держит и снимает только свой
  кадр ``OperatorHold``; пользовательскую тишину оператор не снимает, и наоборот (§6.3).

Чистый модуль: ROS и часы — снаружи (``Session`` держит свои).
"""

from __future__ import annotations

from typing import Any, Callable, Mapping, Optional, Tuple

from rob_box_dialog.session import SceneFrame, Session, SessionChanged, VoiceState

#: ``(провайдер, голос)`` — голос робота по умолчанию (владелец реестра голосов; сюда его передают).
Voice = Tuple[str, str]
#: Просьба владельцу речи применить голос ``(провайдер, голос)``.
Restore = Callable[[str, str], None]

HOLD, RESUME, RESET = "hold", "resume", "reset"
ACTIONS: Tuple[str, ...] = (HOLD, RESUME, RESET)
#: Кто может держать и снимать ``OperatorHold`` через ``DialogControl`` (``set_by`` в ``.srv``: ``operator, tars``).
#: ТАРС — тоже оператор: кадр у них один. Пользователь (``user``) сервисом не управляет (§6.3).
CONTROL_SETTERS = {"operator": "operator", "tars": "operator"}


class SceneDirector:
    """Кадры ``DJ``/``Persona`` и возврат голоса. Пишет в сессию только через её owner-методы."""

    def __init__(self, session: Session, restore: Restore, default_voice: Voice) -> None:
        self._session = session
        self._restore = restore
        self._default = default_voice
        self._persona: Optional[SceneFrame] = None
        session.subscribe(self._on_change)

    def on_music(self, dj: bool, info: Mapping[str, Any]) -> None:
        """Снимок плеера: сет пошёл — ``DJ`` и ``Persona``; сет кончился — оба кадра снимаются.

        Новый ``set_id`` без паузы (сет заменили) меняет только ``DJ``: голос «до персоны» остаётся первым."""
        snap = self._session.snapshot
        if not dj:
            self._session.pop_scene("DJ")
            self._session.pop_scene("Persona")
            return
        set_id = str(info.get("set_id") or "")
        frame = snap.top("DJ")
        if frame is not None and frame.payload.get("set_id") == set_id:
            return
        if frame is not None:
            self._session.pop_scene("DJ")
        self._session.push_scene("DJ", "code", payload={"set_id": set_id})
        if snap.top("Persona") is None:
            before = snap.voice
            self._session.push_scene("Persona", "code", payload={
                "name": str(info.get("persona") or ""), "provider": before.provider, "voice": before.applied})

    def _on_change(self, event: SessionChanged) -> None:
        persona = event.snapshot.top("Persona")
        gone, self._persona = self._persona, persona
        if event.change == "reset":
            self._give_back(self._default, event.snapshot.voice)
        elif gone is not None and persona is None:
            provider, voice = gone.payload.get("provider"), gone.payload.get("voice")
            target = self._default if provider is None or voice is None else (provider, voice)
            self._give_back(target, event.snapshot.voice)

    def _give_back(self, target: Voice, current: VoiceState) -> None:
        """Голос не менялся с тех пор (или не известен) — просить нечего; иначе просьба владельцу речи."""
        if current.applied is None or (current.provider, current.applied) == target:
            return
        self._restore(*target)


def apply_control(session: Session, action: str, set_by: str, ttl_s: float = 0.0) -> bool:
    """``DialogControl``: ``hold`` — кадр ``OperatorHold`` (повторный ``hold`` обновляет срок, а не копится),
    ``resume`` — снять его, ``reset`` — новая сессия. ``False`` — отказ (чужой ``set_by`` или неизвестное действие):
    честный ``ok=false`` в ответе, а не тихое «принято»."""
    owner = CONTROL_SETTERS.get(set_by)
    if owner is None or action not in ACTIONS:
        return False
    if action == HOLD:
        session.pop_scene("OperatorHold", owner)
        session.push_scene("OperatorHold", owner, ttl_s=ttl_s if ttl_s > 0 else None)
    elif action == RESUME:
        session.pop_scene("OperatorHold", owner)
    else:
        session.reset()
    return True


__all__ = ["ACTIONS", "CONTROL_SETTERS", "HOLD", "RESET", "RESUME", "SceneDirector", "apply_control"]
