"""Один инструмент «скажи / спой / сыграй Химерой» (ADR-0175 §2.1).

Схема узкая: действие из списка профиля, не свободный текст. Смысл просьбы и фраза исхода — ``chimera_decide``;
устройство, рендер и отмена — хост. Короткий вызов идёт клипом через ``play_sound``/``SoundEvent``, длинное пение,
речь и игра — владельцу исполнения, стоп которого уже есть (``perform_host.PerformHost.stop``).
"""

from typing import Any, List, Optional

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from . import chimera_decide as decide

ACTIONS = ("call", "speak", "sing", "play")  # литерал для каталога; профиль хоста сужает его на исполнении
TIMBRES = ("male", "female", "child")


class ChimeraPerformTool(MCPTool):
    """Разовое выступление Химерой: вызов, речь, пение или игра по названию."""

    # литерал: каталог читает event_source из тела класса, не из имени константы
    event_source = "inproc:chimera-host"

    def __init__(self, node, host=None, library=None, profile=None):
        super().__init__(node)
        self._host = host if host is not None else node_host(node)
        self._library = library
        self._profile = profile if profile is not None else decide.stub_profile()

    @property
    def name(self) -> str:
        return "chimera_perform"

    @property
    def description(self) -> str:
        return (
            "Разовое выступление Химерой. action=call — разрешённый вызов существа (короткий клип); "
            "speak — подготовленная фраза (phrase_id) проверенным тембром; sing — мелодия по названию со слогами; "
            "play — та же мелодия без текста, нейтральным вокализом. Название ищет библиотека партитур. "
            "«Спел» — только после конца проигрывания. Произвольный русский текст не поддержан."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action",
                type="string",
                description="Что сделать Химерой",
                required=True,
                enum=["call", "speak", "sing", "play"],
            ),
            MCPToolParameter(name="creature", type="string", description="Существо для action=call", required=False),
            MCPToolParameter(name="call", type="string", description="Вызов существа для action=call", required=False),
            MCPToolParameter(
                name="phrase_id", type="string", description="Id подготовленной фразы для action=speak", required=False
            ),
            MCPToolParameter(
                name="timbre",
                type="string",
                description="Человеческий тембр речи или пения",
                required=False,
                enum=["male", "female", "child"],
            ),
            MCPToolParameter(
                name="title", type="string", description="Название пьесы для sing или play", required=False
            ),
            MCPToolParameter(
                name="syllables", type="string", description="Слоги пения; без них sing поёт вокализ", required=False
            ),
        ]

    @property
    def slice(self) -> str:
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    def execute(
        self,
        action: str,
        creature: str = "",
        call: str = "",
        phrase_id: str = "",
        timbre: str = "",
        title: str = "",
        syllables: str = "",
    ) -> MCPToolResult:
        """Решить просьбу и спросить хост. Фраза — из исхода, одна на запрос."""
        plan = decide.decide(
            action,
            self._profile,
            creature=creature,
            call_name=call,
            phrase_id=phrase_id,
            timbre=timbre,
            title=title,
            syllables=syllables,
            library=self._library_of(),
        )
        if not plan.plays:
            return self._result(decide.outcome_of(plan))
        request_id, stage = self._host.perform(plan)
        return self._result(decide.outcome_of(plan, stage, request_id))

    def _library_of(self):
        if self._library is not None:
            return self._library
        from ..engine.score_library import ScoreLibrary

        return ScoreLibrary()

    def _result(self, outcome: decide.ChimeraOutcome) -> MCPToolResult:
        from rob_box_dialog import phrases

        said = decide.sang_phrase(outcome, phrases.text, phrases.first_key)
        self.log_info(f"{outcome.action} {outcome.status} {outcome.reason} sang={outcome.sang}")
        return MCPToolResult(
            success=outcome.status == "done",
            data=outcome.as_data(),
            error=None if outcome.status == "done" else said,
            message=said,
        )


def node_host(node: Any) -> "ChimeraHostPort":
    """Хост ноды: один на ноду, чтобы стоп попал в тот же плеер, что и старт."""
    host = getattr(node, "_chimera_host", None)
    if host is None:
        host = ChimeraHostPort(node)
        try:
            node._chimera_host = host
        except Exception:  # noqa: BLE001 — нода без атрибутов: порт живёт только в туле
            pass
    return host


class ChimeraHostPort:
    """Мост к хосту робота: короткий клип — ``SoundEvent``, длинное — владелец исполнения и его стоп.

    В тесте вместо этого порта стоит фальшивый хост. Здесь нет второго плеера: стоп длинного действия зовёт
    ``PerformHost.stop``, если выступление уже идёт, иначе отменяет собственный рендер.
    """

    def __init__(self, node: Any, perform_host: Any = None) -> None:
        self._node = node
        self._perform = perform_host
        self._active: Optional[str] = None

    def perform(self, plan: decide.ChimeraPlan):
        """Спросить хост и вернуть ``(request_id, стадия)``. Нет хоста — честный timeout, не выдуманный успех."""
        if plan.route == decide.ROUTE_CLIP:
            return self._clip(plan)
        return self._long(plan)

    def stop(self, when: str = "playback") -> bool:
        """Остановить идущее. Длинное действие, отданное владельцу исполнения, гасит его ``stop``."""
        if self._perform is not None and self._perform.stop():
            self._active = None
            return True
        if self._active is None:
            return False
        self._active = None
        return True

    def _clip(self, plan: decide.ChimeraPlan):
        from .sound import EVENT_TOPIC, TRIGGER_TOPIC, sound_deadline_s

        return self._ask_sound(plan.clip, TRIGGER_TOPIC, EVENT_TOPIC, sound_deadline_s())

    def _long(self, plan: decide.ChimeraPlan):
        """Длинное действие. Владелец исполнения на ноде есть — стоп его; рендера пакета 0.1/0.2 ещё нет."""
        self._active = plan.material_id or plan.action
        if self._perform is not None:
            return "", {"stage": "started"}
        return "", {"stage": "missing_file", "reason": "missing_file"}

    def _ask_sound(self, clip: str, trigger: str, event: str, wait_s: float):
        import json

        from std_msgs.msg import String

        from ..owner_event import OwnerEvents, owner_events

        pub = self._node.create_publisher(String, trigger, 10)

        def publish(request_id: str) -> None:
            msg = String()
            msg.data = json.dumps({"sound": clip, "request_id": request_id}, ensure_ascii=False)
            pub.publish(msg)

        from rob_box_dialog_msgs.msg import SoundEvent

        events = owner_events(
            self._node,
            "chimera-clip",
            lambda: OwnerEvents(self._node, SoundEvent, event, key_of=lambda m: m.request_id),
        )
        request_id, got = events.ask(publish, wait_s)
        return request_id, _sound_stage(got)


def _sound_stage(event: Optional[Any]):
    """``SoundEvent`` → стадия хоста. ``started`` ещё не конец: «спел» ждёт ``finished``."""
    if event is None:
        return None
    if event.kind == event.KIND_FINISHED:
        return {"stage": "finished"}
    if event.kind == event.KIND_SKIPPED:
        reason = event.reason or "error"
        if reason == "not_found":
            return {"stage": "missing_file"}
        return {"stage": "skipped", "reason": reason}
    if event.kind == event.KIND_STARTED:
        return {"stage": "started"}
    return {"stage": "skipped", "reason": "error"}


__all__ = ["ACTIONS", "TIMBRES", "ChimeraPerformTool", "ChimeraHostPort", "node_host"]
