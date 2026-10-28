"""Правка ИГРАЮЩЕГО трека на ходу (#4029, ADR-0148): решение — ``rob_box_music.track_edit``, исполнение — здесь.

Свой синт-трек (``request_music`` intent=track и бит выступления, оба — ``RequestMusicTool._stage_club``) собирается
по рецепту (:class:`LiveRecipe`: план, номер трека плана, мелодии и материалы темы, лид инструмента, дека). Правка
пересобирает тот же трек с другим темпом, семьёй тембров или уровнем баса — форма та же — и владелец плеера
(``PlayerOwner.retune``) ставит программу в те же слоты на ближайшей границе такта: без остановки, без нового
``started`` и без генерации, выступление поверх бита держит сетку. Готовая запись, пьеса, сет, тишина — честный
отказ кодом (``knowledge.EDIT_REFUSALS``). Фраза исхода — из исхода (``track_edit.edit_text``).
"""

from __future__ import annotations

import logging
import threading
from collections import OrderedDict
from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple

from rob_box_music.arrange.compose import compose
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import SetPlan
from rob_box_music.track_edit import (
    FILE,
    NONE,
    OTHER,
    PIECE,
    SET,
    SYNTH,
    EditPlan,
    EditTarget,
    edit_text,
    music_edits,
    plan_edit,
    with_bass,
)

_LOG = logging.getLogger(__name__)
#: Сколько ждать, пока правка встанет на такт: такт при 60 BPM — 4 с, плюс запас на доставку (как ``started``).
EDIT_WAIT_S = 6.0
#: Рецептов своих треков в памяти: играющий, подложка и пара последних заказов.
RECIPES_KEPT = 8
#: Источник играющего (факт ``source`` снимка плеера) → вид для отказа правки.
_SOURCE_KIND = {"generated": FILE, "piece": PIECE, "melody": PIECE}
#: ``(событие «встала», срок) -> встала ли``: настоящий поток ждёт ``threading.Event``, тест крутит клок симулятора.
Wait = Callable[[threading.Event, float], bool]


def synth_title(label: str, bpm: float) -> str:
    """Название своего синт-трека в снимке плеера: тема и темп («клубный трек · 138 BPM»)."""
    return f"{label or 'клубный трек'} · {bpm:g} BPM"


@dataclass(frozen=True)
class LiveRecipe:
    """Как собран свой синт-трек деки: пересборка с правкой даёт тот же трек (форма, гармония, рисунки) с другим
    темпом, тембром или басом. ``code`` — программа, что звучит (без строки-заголовка: сверка «звук изменился»)."""

    plan: SetPlan
    track_no: int
    melodies: Mapping[str, str]
    materials: Mapping[str, Any]
    lead_synth: Optional[str]
    deck: str
    label: str = ""
    bass_db: float = 0.0
    code: str = ""


def _body(code: str) -> str:
    """Программа без первой строки-комментария (в ней ``track_id`` с хэшем партий: уровень баса на потолке ``amp``
    меняет хэш, но не звук)."""
    return code.split("\n", 1)[1] if "\n" in code else code


def _target(snap: Mapping[str, Any], recipe: Optional[LiveRecipe]) -> EditTarget:
    """Что играет — по снимку владельца плеера (``PlayerOwner.now_playing``) и рецепту своего трека."""
    if not snap.get("playing"):
        return EditTarget(NONE)
    if recipe is not None:
        return EditTarget(
            SYNTH,
            float(recipe.plan.bpm),
            recipe.plan.style,
            recipe.plan.family,
            recipe.bass_db,
            bool(snap.get("backing")),
        )
    if snap.get("enabled"):
        return EditTarget(SET)
    return EditTarget(_SOURCE_KIND.get(str(snap.get("source") or ""), OTHER))


class TrackEditor:
    """Рецепты своих синт-треков и правка играющего по ним."""

    def __init__(
        self,
        owner: Any,
        adapter: Any = None,
        log: Callable[[], Any] = lambda: _LOG,
        wait: Wait = lambda done, seconds: done.wait(seconds),
        wait_s: float = EDIT_WAIT_S,
    ) -> None:
        self._owner = owner
        #: Клок и деки Renardo (``RenardoAdapter``): ставит ``mcp_server``; нет — правка звука честно недоступна.
        self.adapter = adapter
        self._log = log
        self._wait = wait
        self._wait_s = wait_s
        self._lock = threading.Lock()
        self._recipes: "OrderedDict[str, LiveRecipe]" = OrderedDict()

    def remember(self, track_id: str, recipe: LiveRecipe) -> None:
        """Собран свой синт-трек ``track_id`` — его рецепт (правка, если он заиграет)."""
        with self._lock:
            self._recipes[track_id] = replace(recipe, code=_body(recipe.code))
            while len(self._recipes) > RECIPES_KEPT:
                self._recipes.popitem(last=False)

    def edit(self, kinds: Sequence[str], heard_text: Optional[str] = None) -> Dict[str, Any]:
        """Правка играющего: виды — из слов человека про играющую музыку (``heard_text``, ``track_edit.music_edits``
        — то же чтение, что решило путь хода), а без них — из узкого слота LLM (``kinds``). Итог — данные результата
        тула с фразой исхода ``message``."""
        named = music_edits(heard_text or "")
        asked = named or tuple(kinds)
        snap = self._owner.now_playing()
        track_id = str(snap.get("track_id") or "")
        with self._lock:
            recipe = self._recipes.get(track_id)
        target = _target(snap, recipe)
        plan = plan_edit(asked, target)
        reason = ""
        if plan.changes_sound and recipe is not None:
            plan, reason = self._apply(track_id, recipe, plan)
        self._log().info(
            f"🎚️ [edit] {'слова' if named else 'слот'} {list(asked)} играет={target.kind}"
            f"{' (подложка)' if target.backing else ''} track_id={track_id or '—'} → "
            f"сделано={list(plan.applied)} предел={list(plan.limited)} дальше={list(plan.kept)} "
            f"отказ={plan.refused or reason or '—'}"
        )
        return self._result(plan, target, track_id, reason)

    def _apply(self, track_id: str, recipe: LiveRecipe, plan: EditPlan) -> Tuple[EditPlan, str]:
        """Пересобрать трек с правкой и поставить его на такт; ``(исход, причина сбоя или «»)``. Программа звучит так
        же, как играющая (бас на потолке), — правка не меняет звук: предел, деку не трогаем."""
        new_plan = replace(recipe.plan, bpm=int(plan.bpm), timbre=plan.timbre)
        try:
            model = with_bass(
                compose(
                    new_plan,
                    recipe.track_no,
                    melodies=recipe.melodies,
                    deck=recipe.deck,
                    materials=recipe.materials,
                    lead_synth=recipe.lead_synth,
                ),
                plan.bass_db,
            )
            program = render(model, recipe.deck)
        except Exception as exc:  # noqa: BLE001 — отказ громкий, трек играет как было
            self._log().warning(f"⚠️ [edit] пересборка {track_id} упала: {type(exc).__name__}: {exc}")
            return plan, "compose_error"
        tempo = float(plan.bpm) if plan.bpm != recipe.plan.bpm else None
        if tempo is None and _body(program.code) == recipe.code:
            return plan.limit_all(), ""
        reason = self._retune(track_id, program, tempo)
        if reason:
            return plan, reason
        self._owner.update_dj(
            track_id, {"bpm": int(plan.bpm), "title": synth_title(recipe.label, plan.bpm)}, set_only=False
        )
        with self._lock:
            self._recipes[track_id] = replace(recipe, plan=new_plan, bass_db=plan.bass_db, code=_body(program.code))
        return plan, ""

    def _retune(self, track_id: str, program: Any, tempo: Optional[float]) -> str:
        """Программа правки — в слоты играющей деки на ближайшей границе такта формы (``RenardoAdapter.retune``), с
        ней темп ``tempo`` (``None`` — тот же); ждём, пока встанет. ``track_id``, доля старта и форма — прежние:
        ``started`` не публикуется, выступление поверх трека держит сетку, у подложки и сета — свои концы. «» —
        встала; иначе причина (звук не тронут)."""
        where = self._owner.position(track_id)
        if where is None or self.adapter is None:
            return "not_playing" if where is None else "unavailable"
        problem = self.adapter.check(program)
        if problem is not None:
            self._log().warning(f"⚠️ [music v2] edit track_id={track_id} отказ {problem[0]}: {problem[1]}")
            return str(problem[0])
        done = threading.Event()
        outcome: Dict[str, Any] = {}

        def on_done(info: Dict[str, Any]) -> None:
            outcome.update(info)
            done.set()

        at = self.adapter.retune(program, where[1], tempo, lambda: self._owner.position(track_id) is not None, on_done)
        self._log().info(f"🎚️ [music v2] edit track_id={track_id} at_beat={at:g} bpm→{tempo or 'тот же'} — ждём такта")
        if not self._wait(done, self._wait_s) or not outcome.get("ok"):
            self._log().warning(f"⚠️ [music v2] edit track_id={track_id} не встала: {outcome or 'нет такта в срок'}")
            return str(outcome.get("reason") or "not_applied")
        self._log().info(
            f"🎚️ [music v2] edit applied track_id={track_id} at_beat={outcome.get('at_beat'):g} "
            f"bpm={outcome.get('bpm'):g}"
        )
        return ""

    @staticmethod
    def _result(plan: EditPlan, target: EditTarget, track_id: str, reason: str) -> Dict[str, Any]:
        """Данные результата тула: что сделано, предел, отказ и фраза исхода. Успех — что-то сделано (или «играю
        дальше»); иначе отказ с причиной, звук не тронут."""
        if reason:
            return {
                "ok": False,
                "reason": f"edit_{reason}",
                "track_id": track_id,
                "backing": target.backing,
                "message": edit_text(EditPlan(plan.kinds, refused="failed")),
            }
        data: Dict[str, Any] = {
            "ok": bool(plan.applied or plan.limited or plan.kept),
            "track_id": track_id,
            "changes": list(plan.applied),
            "limited": list(plan.limited),
            "kept": list(plan.kept),
            "backing": target.backing,
            "message": edit_text(plan),
            # бит выступления поменялся на такте, и сказать больше нечего: исход слышен сам
            "heard": target.backing and bool(plan.applied) and not (plan.limited or plan.refused),
        }
        if target.kind == SYNTH:
            data["bpm"] = int(plan.bpm)
        if plan.refused:
            data["refused"] = plan.refused
        if not data["ok"]:
            data["reason"] = f"edit_{plan.refused or 'none'}"
        return data


__all__ = ["EDIT_WAIT_S", "LiveRecipe", "RECIPES_KEPT", "TrackEditor", "synth_title"]
