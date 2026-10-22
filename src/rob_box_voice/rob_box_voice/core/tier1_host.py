"""Tier-1 в старом пути v1 (ADR-0150 PR-6 #3696): снимок ноды → :func:`rob_box_dialog.grammar.match` → исполнение.

Решает грамматика (чистая, в ``rob_box_dialog``); здесь — только снимок из состояния ``DialogueNode`` и исполнение
вида команды её же механизмами: отмена хода и речи (``_cancel_run``), тишина (``_handle_silence``), FSM, сброс
сессии, MCP-тулы громкости/голоса тем же исполнителем, что у медиароутера. Фразы — ``rob_box_dialog/phrases``;
в SILENCED не звучит ничего (§6.3). Спорный стоп (В19) — :class:`StopChooser` в loop'е ноды, речь до решения не
рвётся; «часть команды» — реплика продолжает приём с шага после Tier-1.

Атрибуты ноды читаются через ``getattr`` с умолчанием: юнит-тесты собирают ноду через ``object.__new__``.
"""

from __future__ import annotations

import asyncio
import time
from typing import TYPE_CHECKING, Any, Callable, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.phrases import text as phrase
from rob_box_dialog.session import MusicStateRef, SceneFrame, SessionSnapshot
from rob_box_dialog.words import policy_word
from rob_box_dialog_msgs.msg import Intent as IntentMsg
from rob_box_harness.core.dialogue_state_machine import DialogueEvent, DialogueStateKind
from rob_box_voice.core.command_intent import fill_intent
from rob_box_voice.core import perform_ask
from rob_box_voice.core.command_parser import Command, IntentType
from rob_box_voice.core.media_router import DJ_SET_TOOL, MediaToolCall
from rob_box_voice.core.stop_chooser import STOP
from rob_box_voice.core.turn_context import tts_state
from rob_box_voice.tts_voice_registry import VOICE_METADATA, voices_for

if TYPE_CHECKING:  # pragma: no cover
    from rob_box_voice.core.stt_admission import SttContext

#: Реплика Telegram «/clear» — новая сессия (транспортное правило, не слова человека).
TG_CLEAR = "/clear"
#: Виды, которым нужен MCP-исполнитель: без него реплика не Tier-1 (отвечает LLM, как раньше).
_NEEDS_TOOLS = ("volume", "set_voice")
#: Счётчик пропусков LLM по виду (``LLMSkipReason``); остальные виды — ``tier1``.
_SKIP_KEY = {"silence": "silence_command", "new_session": "new_session", "unsilence": "unsilence"}


class Tier1Runner:
    """Снимок, сверка и исполнение Tier-1 команды для одной ``DialogueNode``."""

    def __init__(self, node: Any) -> None:
        self._node = node
        self.volume_channel = "tts"

    # -- снимок -------------------------------------------------------------------------------------------------

    def snapshot(self) -> SessionSnapshot:
        node = self._node
        dsm = getattr(node, "_dsm", None)
        silenced = getattr(dsm, "current_state", None) == DialogueStateKind.SILENCED
        player = getattr(node, "_music_player_state", None)
        playing = bool(player is not None and player.is_playing())
        title = str((player.dj_info or {}).get("title") or "") if player is not None else ""
        watch = getattr(node, "_speaking_watch", None)
        return SessionSnapshot(
            agent="personality", session_id="v1", epoch=0, started_at=0.0,
            scenes=(SceneFrame("Silence", "user"),) if silenced else (),
            music=MusicStateRef("playing" if playing else "idle", title=title or None,
                                dj=bool(player is not None and player.dj)),
            speaking=bool(watch is not None and watch.speaking), volume_channel=self.volume_channel,
            pending_ask=perform_ask.ASK if perform_ask.window(node) is not None else None)

    def voices(self) -> list:
        provider, _ = tts_state(self._node)
        return [(vid, VOICE_METADATA.get(f"{provider}:{vid}", {}).get("display_name", vid))
                for vid in voices_for(provider)]

    def match(self, ctx: "SttContext") -> Optional[Tier1Command]:
        """Tier-1 команда реплики или ``None`` (дальше медиа-грамматика / LLM)."""
        node = self._node
        if ctx.tg_chat_id is not None and ctx.text.strip().lower() == TG_CLEAR:
            command: Optional[Tier1Command] = Tier1Command("new_session")
        else:
            command = match(ctx.text, self.snapshot(), self.voices())
        if command is None:
            return None
        if command.kind == "new_session" and not getattr(node, "_new_session_enabled", True):
            return None
        if command.kind in _NEEDS_TOOLS and getattr(node, "_tool_executor", None) is None:
            return None
        node.get_logger().info(
            f"🧩 [tier1] kind={command.kind} disputed={command.disputed} args={dict(command.args)} "
            f"rest={command.rest[:40]!r} silenced={self._silenced()} text={ctx.text[:60]!r}")
        return command

    def _silenced(self) -> bool:
        return getattr(getattr(self._node, "_dsm", None), "current_state", None) == DialogueStateKind.SILENCED

    # -- исполнение ---------------------------------------------------------------------------------------------

    def run(self, command: Tier1Command, text: str) -> None:
        """Исполнить вид команды; звучащие фразы — только вне SILENCED."""
        node = self._node
        counter = getattr(node, "_llm_skipped_counter", None)
        if counter is not None:
            key = _SKIP_KEY.get(command.kind, "tier1")
            counter[key] = counter.get(key, 0) + 1
        getattr(self, f"_run_{command.kind}")(command, text)

    def _run_stop_all(self, _command: Tier1Command, text: str) -> None:
        """Стоп всего (§4.2): ход и речь, движение (``/dialog/intent stop_motion``), музыка — молча. Идёт выступление
        (ADR-0150 PR-7c): «хватит» — дочитать предложение, бит выступления снимет оно само на такте (outro); «стой» —
        всё сразу."""
        node = self._node
        outro = self._stop_performance(text)
        node._cancel_run("tier1 stop", stop_tts=True)
        pub = getattr(node, "_intent_pub", None)
        if pub is not None:
            gate = getattr(node, "_session_epoch_gate", None)
            stop = Command(intent=IntentType.STOP, text=text, entities={}, confidence=1.0)
            pub.publish(fill_intent(IntentMsg(), stop, epoch=gate().current if gate else 0))
        player = getattr(node, "_music_player_state", None)
        if player is not None and (player.is_playing() or player.dj) and not outro:
            self.stop_music()

    def _stop_performance(self, text: str) -> bool:
        """Остановить выступление по слову (``policy_word``: «стой» — halt, иначе — до конца предложения). ``True`` —
        музыка, что играет, — бит выступления, и его снимет само выступление (outro на такте)."""
        performer = getattr(getattr(self._node, "_tool_executor", None), "performer", None)
        if performer is None:
            return False
        if policy_word(text) == "halt":
            performer.halt()
            return False
        owns_music = performer.owns_backing
        return performer.stop() and owns_music

    def stop_music(self) -> bool:
        """Стоп деки/сета и mp3 без фразы (стоп всего; «выключи музыку» в SILENCED, #3696). Нет исполнителя — ``False``."""
        if getattr(self._node, "_tool_executor", None) is None:
            return False
        self._node.get_logger().info("🔇 [tier1] стоп музыки без фразы: dj_set(stop) + stop_music")
        self._spawn(self._tools((MediaToolCall(DJ_SET_TOOL, {"action": "stop"}), MediaToolCall("stop_music", {}))))
        return True

    def _run_silence(self, _command: Tier1Command, text: str) -> None:
        """«Хватит/замолчи»: тишина; идущее выступление дочитывает предложение и снимает свой бит на такте (X5)."""
        self._stop_performance(text)
        if self._silenced():
            self._node._cancel_run("tier1 silence (уже тишина)", stop_tts=True)
            return
        self._node._handle_silence()

    def _run_unsilence(self, _command: Tier1Command, _text: str) -> None:
        node = self._node
        node._dsm.on_event(DialogueEvent.UNSILENCE)
        node._publish_state()

    def _run_new_session(self, _command: Tier1Command, text: str) -> None:
        self._node._reset_dialogue_session()
        self._node.get_logger().info(f"🧹 [new-session] session reset: text={text[:60]!r}")

    def _run_volume(self, command: Tier1Command, _text: str) -> None:
        channel, action = command.args["channel"], command.args["action"]
        self.volume_channel = channel
        prefix = "music" if channel == "music" else "voice"
        quiet = self._silenced()

        async def go() -> None:
            ok = await self._tools((MediaToolCall(kn.VOLUME_CHANNELS[channel].tool, {"action": action}),))
            said = phrase(f"{prefix}.volume.{action}" if ok else f"{prefix}.volume_fail")
            self._node.get_logger().info(f"🔊 [tier1] volume {channel}:{action} ok={ok} quiet={quiet} say={said!r}")
            if not quiet:
                self._node._speak_direct(said)

        self._spawn(go())

    def _run_set_voice(self, command: Tier1Command, _text: str) -> None:
        # Фразу «Говорю голосом X» / «Голос X недоступен» строит сам тул по SpeechEvent voice_changed (PR-4).
        self._spawn(self._tools((MediaToolCall("set_voice", {"voice": command.args["voice"]}),)))

    def _run_answer(self, command: Tier1Command, _text: str) -> None:
        """«Да»/«нет» на вопрос выступления «выключить и зачитать?» (О9′, #3700): ответ — выступлению, окно — снять."""
        node = self._node
        perform_ask.close(node)
        performer = getattr(getattr(node, "_tool_executor", None), "performer", None)
        yes = command.args.get("value") == "yes"
        taken = performer is not None and performer.answer(yes)
        node.get_logger().info(f"🧩 [tier1] answer {'да' if yes else 'нет'} → выступление: {taken}")

    async def _tools(self, calls: tuple) -> bool:
        node = self._node
        executor = getattr(node, "_tool_executor", None)
        ok = executor is not None
        for call in calls if ok else ():
            call_ok, _content = await node._execute_media_tool_result(executor, call)
            ok = ok and call_ok
        return ok

    def _spawn(self, coro: Any) -> None:
        loop = getattr(self._node, "_loop", None)
        if loop is None:
            coro.close()
            return
        asyncio.run_coroutine_threadsafe(coro, loop)

    # -- спорный стоп (В19) -------------------------------------------------------------------------------------

    def dispute(self, command: Tier1Command, ctx: "SttContext", resume: Optional[Callable[[str], None]]) -> None:
        """Спросить Chooser в loop'е: ``stop`` — исполнить команду; иначе реплика продолжает приём (``resume``)."""
        chooser = getattr(self._node, "_stop_chooser", None)

        async def go() -> None:
            started = time.monotonic()
            verdict = (await asyncio.to_thread(chooser.decide, ctx.text))[0] if chooser is not None else ""
            self._node.get_logger().info(
                f"🤔 [tier1] спорный стоп → {verdict or 'part_of_command'} за {(time.monotonic() - started):.2f} с")
            if verdict == STOP:
                self.run(command, ctx.text)
            elif callable(resume):
                resume(ctx.text)

        self._spawn(go())


def runner_of(node: Any) -> Tier1Runner:
    """Раннер Tier-1 ноды (один на ноду, создаётся при первом обращении)."""
    runner = getattr(node, "_tier1", None)
    if runner is None:
        runner = node._tier1 = Tier1Runner(node)
    return runner


__all__ = ["TG_CLEAR", "Tier1Runner", "runner_of"]
