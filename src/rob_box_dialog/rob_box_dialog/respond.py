"""Фраза об исполнении — из события владельца, а не из слов LLM (ADR-0150 §2.1 п.3, §5.2, §5.4; ADR-0148).

Исполнитель отвечает :class:`ActionResult`; :func:`phrase_from_result` строит по нему реплику из каталога
``phrases/ru.yaml`` по ключу ``{класс}.{статус}[.{причина}]`` (или особому ключу тула, ``knowledge.TOOL_PHRASES``),
:func:`join` собирает фразы нескольких действий хода в одну реплику. ``query`` с ``done`` фразы не даёт: данные
уходят обратно LLM. Провал (:func:`failed`) грустит; отказ тула, который - вопрос человеку (:func:`asks`,
``knowledge.TOOL_QUESTIONS``: «Удалить «X»?»), - не провал (#4062).
"""

from __future__ import annotations

import zlib
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Any, Mapping, Optional, Sequence

from rob_box_core.utterance import Sink, Utterance
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases

#: Статусы, при которых рефлекс побеждает эмоцию команды (§5.5.1: нельзя «радостно» о провале).
_FAILED = frozenset({"rejected", "timeout"})


@dataclass(frozen=True)
class ActionResult:
    """Ответ исполнителя (§5.2): статус по событию владельца ресурса, причина — ключ фраз, данные — слоты."""

    turn_id: str
    epoch: int
    agent: str
    tool: str
    cls: str
    status: str
    reason: Optional[str] = None
    data: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))
    event_ref: Optional[str] = None


def asks(result: ActionResult) -> bool:
    """Отказ тула - вопрос человеку (``knowledge.TOOL_QUESTIONS``, #4062): «Удалить «X»? Скажи «да, удаляй».» -
    ничего не провалилось, тул ждёт ответа."""
    return result.status == "rejected" and (result.tool, result.reason) in kn.TOOL_QUESTIONS


def failed(result: ActionResult) -> bool:
    """Исход - провал (``rejected``/``timeout``): фраза о нём грустит, лицо команды его не красит (§5.5.1). Вопрос
    тула (:func:`asks`) - не провал: лицо нейтральное, а не ``sad`` (#4062)."""
    return result.status in _FAILED and not asks(result)


def classed(result: ActionResult, args: Mapping[str, Any]) -> ActionResult:
    """Результат с классом ВЫЗОВА (``knowledge.call_class``): исполнитель клеит класс тула, а класс ``music_library``
    зависит от ``action`` (#4020) - list/search/info это ``query`` (данные идут LLM), а не «Включаю музыку.».
    ``args`` - аргументы вызова (для позднего исхода - данные результата: аргументы лежат в них поверх ответа тула)."""
    cls = kn.call_class(result.tool, args)
    return result if cls == result.cls else replace(result, cls=cls)


def _tool_key(result: ActionResult) -> Optional[str]:
    """Особый ключ тула (``knowledge.TOOL_PHRASES``): готовый или выбранный по данным результата
    (``knowledge.ByData``)."""
    entry = kn.TOOL_PHRASES.get(result.tool, {}).get(result.status)
    return entry.key(result.data) if isinstance(entry, kn.ByData) else entry


def _imitation_key(result: ActionResult) -> Optional[str]:
    """Успех тихого действия молчит, кроме звукоподражания (``knowledge.SOUND_IMITATIONS``): ключ по звуку из данных
    результата или ``None``."""
    if result.tool != kn.SOUND_TOOL:
        return None
    return kn.SOUND_IMITATIONS.get(str(result.data.get("sound")))


def phrase_key(result: ActionResult) -> Optional[str]:
    """Ключ фразы результата или ``None``, если о таком исходе шаблон не говорит (``query.done`` — речь LLM;
    успех молчаливого действия ``knowledge.SILENT_DONE_TOOLS`` — звук сам себе доказательство, кроме звукоподражания;
    исход с причиной из ``knowledge.VOICED_BY_OTHER_REASONS`` — о нём говорит другой результат команды, #4054: факт-имя
    и ``register_speaker``). Фраза берётся из РЕЗУЛЬТАТА, а не из класса тула (ADR-0148): ``stop_music`` при
    ``was_playing=False`` - «ничего не играет», а не «Выключил музыку»; действия ``music_library`` - свои фразы
    (``knowledge.TOOL_PHRASES``)."""
    if result.reason in kn.VOICED_BY_OTHER_REASONS:
        return None
    if result.status == "done" and result.tool in kn.SILENT_DONE_TOOLS:
        return _imitation_key(result)
    if result.reason == kn.PLAN_WAIT_REASON:  # не дождался очереди плана: исход не у владельца — фраза плана
        return "plan.timeout"
    base = _tool_key(result) or kn.ACTION_CLASSES[result.cls].phrases.get(result.status)
    if base is None:
        return None
    return phrases.first_key(f"{base}.{result.reason}" if result.reason else None, base)


def _slots(result: ActionResult) -> Mapping[str, Any]:
    """Слоты фразы: данные события; у фраз библиотеки ``{name}`` - название трека из результата (``title``) в кавычках,
    как у ``media_phrases.library_*_text`` (тексты - один каталог ``ru.yaml``)."""
    slots = dict(result.data)
    if result.tool == kn.LIBRARY_TOOL:
        title = str(slots.get("title") or "").strip()
        slots["name"] = f"«{title}»" if title else kn.LIBRARY_UNNAMED
    return slots


def phrase_from_result(result: ActionResult, snapshot: Any = None) -> Optional[Utterance]:
    """Реплика по результату исполнителя. Вариант фразы выбирается по ``turn_id`` (повторяемо в тестах), адресат —
    наушник для агента оператора, иначе динамики (телеграм-агент берёт из реплики только текст)."""
    key = phrase_key(result)
    if key is None:
        return None
    text = phrases.text(key, zlib.crc32(result.turn_id.encode("utf-8")), **_slots(result))
    agent = getattr(snapshot, "agent", None) or result.agent
    return Utterance(text=text, sink=Sink.HEADSET if agent == "operator" else Sink.SPEAKERS,
                     emotion="sad" if failed(result) else None)


def audible(results: Sequence[ActionResult], speaks: bool) -> Sequence[ActionResult]:
    """Исходы к озвучке. Звучит речь модели рядом с тихими действиями (``knowledge.QUIET_WITH_SPEECH_TOOLS``) - их
    простой успех (``done`` без причины) молчит: «Запомнил тебя, Борис» поверх «Приятно познакомиться, Борис» -
    две реплики об одном. Провал и успех с причиной (``already_known``, ``renamed``: исход не тот, что мог
    подразумевать ответ модели) остаются: слова модели не выдают несделанное за сделанное (Шифу 23.10, #3953)."""
    if not speaks:
        return results
    return [r for r in results if not (r.status == "done" and not r.reason and r.tool in kn.QUIET_WITH_SPEECH_TOOLS)]


def join(results: Sequence[ActionResult], snapshot: Any = None) -> Optional[Utterance]:
    """Фразы всех действий хода — одной репликой в порядке результатов; провал хоть одного — грустная эмоция."""
    said = [u for u in (phrase_from_result(r, snapshot) for r in results) if u is not None]
    if not said:
        return None
    sad = any(u.emotion == "sad" for u in said)
    return Utterance(text=" ".join(u.text for u in said), sink=said[0].sink, emotion="sad" if sad else None)
