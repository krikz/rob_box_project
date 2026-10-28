"""Фраза об исполнении — из события владельца, а не из слов LLM (ADR-0150 §2.1 п.3, §5.2, §5.4; ADR-0148).

Исполнитель отвечает :class:`ActionResult`; :func:`phrase_from_result` строит по нему реплику из каталога
``phrases/ru.yaml`` по ключу ``{класс}.{статус}[.{причина}]`` (или особому ключу тула, ``knowledge.TOOL_PHRASES``),
:func:`join` собирает фразы нескольких действий хода в одну реплику. ``query`` с ``done`` фразы не даёт: данные
уходят обратно LLM. Провал (:func:`failed`) грустит; отказ тула, который - вопрос человеку (:func:`asks`,
``knowledge.TOOL_QUESTIONS``: «Удалить «X»?»), - не провал (#4062); эмоция фразы об исходах - :func:`outcome_emotion`.
"""

from __future__ import annotations

import zlib
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Any, Mapping, Optional, Sequence, Tuple

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


def outcome_emotion(results: Sequence[ActionResult], default: Optional[str] = None) -> Optional[str]:
    """Эмоция фразы кода об исходах - одно правило для всех путей хода (#4062): провал - ``sad``; вопрос тула -
    эмоция вопроса (``ToolQuestion.emotion``: не грусть провала и не настроение модели, ревью PR #4100); иначе
    ``default`` (лицо команды модели или ``None``)."""
    if any(failed(r) for r in results):
        return "sad"
    question = next((r for r in results if asks(r)), None)
    return default if question is None else kn.TOOL_QUESTIONS[(question.tool, str(question.reason))].emotion


def classed(result: ActionResult, args: Mapping[str, Any]) -> ActionResult:
    """Результат с классом ВЫЗОВА (``knowledge.call_class``): исполнитель клеит класс тула, а класс ``music_library``
    зависит от ``action`` (#4020) - list/search/info это ``query`` (данные идут LLM), а не «Включаю музыку.».
    ``args`` - аргументы вызова (для позднего исхода - данные результата: аргументы лежат в них поверх ответа тула)."""
    cls = kn.call_class(result.tool, args)
    return result if cls == result.cls else replace(result, cls=cls)


def _stops_music(result: ActionResult) -> bool:
    """Результат тула, что гасит музыку (``knowledge.MUSIC_STOP_TOOLS``), исполненный (``done``)."""
    if result.status != "done" or result.tool not in kn.MUSIC_STOP_TOOLS:
        return False
    marker = kn.MUSIC_STOP_TOOLS[result.tool]
    return marker is None or result.data.get(marker[0]) == marker[1]


def reconcile_stops(stopped: bool, results: Sequence[ActionResult]) -> Tuple[bool, Tuple[ActionResult, ...]]:
    """Исходы стопа музыки по ходу целиком, а не по одному тулу (#4173, ADR-0148: фраза об исходе - правда о ходе).
    ``stopped`` - музыку в этом ходе уже выключил стоп прошлого вызова; результаты - по порядку вызовов. Стоп, что
    застал тишину (``was_playing=False``) после стопа этого же хода, - не «ничего не играет»: музыку выключил ход, и
    результату ставится ``knowledge.STOPPED_EARLIER_FIELD`` (фраза «Выключил музыку.»). Первый стоп хода, заставший
    тишину, остаётся «ничего не играет». Возвращает ``(музыку ход выключил, результаты)``."""
    out = []
    for result in results:
        if _stops_music(result):
            if result.data.get("was_playing") is False:
                if stopped:
                    result = replace(result, data=MappingProxyType({**result.data, kn.STOPPED_EARLIER_FIELD: True}))
            else:
                stopped = True
        out.append(result)
    return stopped, tuple(out)


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
    reason = _phrase_reason(result)
    return phrases.first_key(f"{base}.{reason}" if reason else None, base)


def _phrase_reason(result: ActionResult) -> Optional[str]:
    """Причина для ключа фразы: ``no_person`` агента без собеседника-голоса (чат Telegram, ТАРС - не
    ``knowledge.PERSON_AGENTS``) - своя, ``knowledge.NO_PERSON_UNVOICED_REASON``: «Не узнаю твой голос» им неправда,
    их голос никто не узнаёт (ревью PR #4121). Остальные - причина результата как есть."""
    if result.reason == kn.NO_PERSON_REASON and result.agent not in kn.PERSON_AGENTS:
        return kn.NO_PERSON_UNVOICED_REASON
    return result.reason


def _slots(result: ActionResult) -> Mapping[str, Any]:
    """Слоты фразы: данные события; у фраз библиотеки ``{name}`` - название трека из результата (``title``) в кавычках,
    как у ``media_phrases.library_*_text`` (тексты - один каталог ``ru.yaml``)."""
    slots = dict(result.data)
    if result.tool == kn.LIBRARY_TOOL:
        title = str(slots.get("title") or "").strip()
        slots["name"] = f"«{title}»" if title else kn.LIBRARY_UNNAMED
    return slots


def _owner_text(result: ActionResult) -> Tuple[str, str]:
    """``(whole, note)`` — фраза кода владельца из данных успешного исхода (``knowledge.OUTCOME_OWNER_TEXT``, #4090).
    Пусто, когда об исходе говорит таблица диалога по данным (``knowledge.TOOL_PHRASES``: «песня пишется» —
    ``knowledge.GEN_WAIT``, #4104): её фраза — единственная об этом факте, владелец её не подменяет и не дополняет."""
    fields = kn.OUTCOME_OWNER_TEXT.get(result.tool)
    if result.status != "done" or fields is None or _tool_key(result) is not None:
        return "", ""
    return (str(result.data.get(fields.whole) or "").strip(), str(result.data.get(fields.note) or "").strip())


def phrase_from_result(result: ActionResult, snapshot: Any = None) -> Optional[Utterance]:
    """Реплика по результату исполнителя. Вариант фразы выбирается по ``turn_id`` (повторяемо в тестах), адресат —
    наушник для агента оператора, иначе динамики (телеграм-агент берёт из реплики только текст). Фраза кода владельца
    из данных исхода (:func:`_owner_text`) — целиком (``whole``) вместо фразы исхода или перед ней (``note``)."""
    key = phrase_key(result)
    if key is None:
        return None
    whole, note = _owner_text(result)
    outcome = phrases.text(key, zlib.crc32(result.turn_id.encode("utf-8")), **_slots(result))
    text = whole or " ".join(filter(None, (note, outcome)))
    agent = getattr(snapshot, "agent", None) or result.agent
    return Utterance(text=text, sink=Sink.HEADSET if agent == "operator" else Sink.SPEAKERS,
                     emotion=outcome_emotion((result,)))


def quiet_in_performance(result: ActionResult) -> bool:
    """Успех запуска музыки, который при идущем выступлении молчит (``knowledge.QUIET_IN_PERFORMANCE_KEYS``, #4075):
    общая фраза класса «Включаю музыку.» поверх рэпа - шум, а не исход. Решает ключ фразы результата: ожидание
    генерации и стоп (фразы по данным) и любой провал остаются."""
    return result.status == "done" and phrase_key(result) in kn.QUIET_IN_PERFORMANCE_KEYS


def audible(results: Sequence[ActionResult], speaks: bool, performing: bool = False) -> Sequence[ActionResult]:
    """Исходы к озвучке. Звучит речь модели рядом с тихими действиями (``knowledge.QUIET_WITH_SPEECH_TOOLS``) - их
    простой успех (``done`` без причины) молчит: «Запомнил тебя, Борис» поверх «Приятно познакомиться, Борис» -
    две реплики об одном. Провал и успех с причиной (``already_known``, ``renamed``: исход не тот, что мог
    подразумевать ответ модели) остаются: слова модели не выдают несделанное за сделанное (Шифу 23.10, #3953).
    Идёт выступление (``performing``) - общий успех запуска музыки молчит (:func:`quiet_in_performance`, #4075)."""
    if not speaks and not performing:
        return results
    return [r for r in results
            if not (speaks and r.status == "done" and not r.reason and r.tool in kn.QUIET_WITH_SPEECH_TOOLS)
            and not (performing and quiet_in_performance(r))]


def join(results: Sequence[ActionResult], snapshot: Any = None) -> Optional[Utterance]:
    """Фразы всех действий хода — одной репликой в порядке результатов; провал хоть одного — грустная эмоция.

    Одинаковые исходы говорятся один раз: «помягче» - ``set_pitch lower`` и ``set_speed slower``, оба ``local.done`` -
    это «Готово.», а не «Готово. Готово.» (живой прогон 27.10). Одинаковость - по тексту фразы: разные ключи каталога с
    общим текстом (``local.done`` и его псевдонимы) тоже один исход, а фразы со своими слотами (название трека, имя)
    различаются и звучат обе. Порядок - по первому появлению."""
    said = [u for u in (phrase_from_result(r, snapshot) for r in results) if u is not None]
    if not said:
        return None
    sad = any(u.emotion == "sad" for u in said)
    texts = tuple(dict.fromkeys(u.text for u in said))
    return Utterance(text=" ".join(texts), sink=said[0].sink, emotion="sad" if sad else None)
