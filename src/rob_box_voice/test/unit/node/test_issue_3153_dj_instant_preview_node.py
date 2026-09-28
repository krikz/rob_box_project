"""Issue #3153 — адаптер ``DialogueNode``: мгновенное превью DJ-сета.

Тот же харнесс, что ``test_issue_3134_media_router_node.py`` (без ROS2,
``object.__new__``): реальные ``_on_stt`` → ``_route_media_command`` →
``_execute_media_plan``; мок — исполнитель тулов (вместо
``SchedulerToolExecutor`` → ``/mcp/execute``), DJ-контроллер и TTS.
Проверяется порядок вызовов, честные фразы при отказе тулов и заявка
«превью — трек #1» DJ-контроллеру.
"""

from __future__ import annotations

import json

from rob_box_llm.provider import ToolResult

from rob_box_voice.core.media_router import (
    DJ_MODE_FAIL_AFTER_PREVIEW_TEXT,
    DJ_PREVIEW_FAIL_TEXT,
    DJ_PREVIEW_FORM_SEC,
)
from rob_box_voice.core.music_player_state import MusicPlayerState

from .test_issue_3134_media_router_node import _make_node, _stt, run_plans  # noqa: F401


class _OrderedExecutor:
    """Исполнитель, который пишет вызовы в общий журнал и валит нужные тулы."""

    def __init__(self, journal: list, fail: frozenset = frozenset()) -> None:
        self.journal = journal
        self.calls: list = []
        self._fail = fail

    def begin_turn(self) -> None:
        self.journal.append(("begin_turn",))

    async def execute(self, call):
        self.journal.append(("tool", call.name))
        self.calls.append((call.name, dict(call.arguments)))
        failed = call.name in self._fail
        return ToolResult(
            tool_call_id=call.id,
            content=json.dumps({"success": not failed}),
            is_error=failed,
        )


def _node(journal: list, *, fail=frozenset(), **state):
    n = _make_node(**state)
    n._scheduler_executor = _OrderedExecutor(journal, fail=frozenset(fail))
    n._dj.claim_preview.side_effect = lambda root: journal.append(("claim", root))
    n._dj.drop_preview_claim.side_effect = lambda: journal.append(("drop",))
    return n


def test_silence_preview_first_then_dj_mode(run_plans):  # noqa: F811
    journal: list = []
    n = _node(journal)
    _stt(n, "Робот, ты диджей Снупдог")
    run_plans()
    n._dispatch_turn.assert_not_called()  # LLM не вызывается
    tools = [e[1] for e in journal if e[0] == "tool"]
    assert tools == ["compose_music", "set_dj_mode"]
    compose_args = n._scheduler_executor.calls[0][1]
    assert compose_args["style"] == "club" and compose_args["repeat"] is True
    assert n._scheduler_executor.calls[1][1]["next_transition_sec"] == DJ_PREVIEW_FORM_SEC
    # Заявка и сброс лимита «один трек за ход» — ДО первого тула.
    first_tool = journal.index(("tool", "compose_music"))
    assert journal.index(("begin_turn",)) < first_tool
    assert journal.index(("claim", compose_args["root"])) < first_tool
    assert ("drop",) not in journal
    n._speak_direct.assert_called_once_with("Я диджей Снупдог, запускаю сет.")


def test_preview_failure_does_not_enable_dj_and_says_so(run_plans):  # noqa: F811
    journal: list = []
    n = _node(journal, fail={"compose_music"})
    _stt(n, "Робот, ты диджей Снупдог")
    run_plans()
    assert [c[0] for c in n._scheduler_executor.calls] == ["compose_music"]
    assert ("drop",) in journal
    n._speak_direct.assert_called_once_with(DJ_PREVIEW_FAIL_TEXT)


def test_dj_mode_failure_after_preview_is_said_honestly(run_plans):  # noqa: F811
    journal: list = []
    n = _node(journal, fail={"set_dj_mode"})
    _stt(n, "Робот, ты диджей Снупдог")
    run_plans()
    assert [c[0] for c in n._scheduler_executor.calls] == ["compose_music", "set_dj_mode"]
    assert ("drop",) in journal
    n._speak_direct.assert_called_once_with(DJ_MODE_FAIL_AFTER_PREVIEW_TEXT)


def test_running_set_only_switches_persona(run_plans):  # noqa: F811
    journal: list = []
    n = _node(journal, playing=True, dj=True, track="Still Dre", state_name="DIALOGUE")
    _stt(n, "Робот, ты диджей Снупдог")
    run_plans()
    assert n._scheduler_executor.calls == [
        ("set_dj_mode", {"enabled": True, "persona": "диджей Снупдог"})
    ]
    assert not [e for e in journal if e[0] in ("claim", "begin_turn")]
    assert n._speak_direct.call_args[0][0].startswith("Теперь я диджей Снупдог")


def test_over_playing_track_no_compose_but_claims_track_one(run_plans):  # noqa: F811
    """Issue #3153 (доп.) — живой прогон 28.09.2026 22:30: играющий обычный
    трек не получает свой ``compose_music`` (он уже звучит), но роутер
    заявляет его DJ-контроллеру как трек #1 сета (пустая тоника — роутер
    её не знает), чтобы переход #1 не шёл по ветке «СТАРТ ВЕЧЕРИНКИ»."""
    journal: list = []
    n = _node(journal, playing=True, track="Still Dre")
    _stt(n, "Робот, ты диджей Снупдог")
    run_plans()
    assert [c[0] for c in n._scheduler_executor.calls] == ["set_dj_mode"]
    assert journal.index(("claim", "")) < journal.index(("tool", "set_dj_mode"))
    assert ("drop",) not in journal


def test_preview_after_idle_snapshot(run_plans):  # noqa: F811
    """Тишина — по снимку плеера (ADR-0141), а не по памяти ноды."""
    journal: list = []
    n = _node(journal, playing=True)
    n._music_player_state = MusicPlayerState(state="idle")
    _stt(n, "Робот, запусти диджей-сет")
    run_plans()
    assert [c[0] for c in n._scheduler_executor.calls] == ["compose_music", "set_dj_mode"]
