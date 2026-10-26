"""Tier-1: закрытая грамматика команд без LLM (ADR-0150 §4.2, §6.3, §18.3 В19; карточка #3696).

:func:`match` — стоп, тишина и её снятие, громкость по каналу, голос, новая сессия, да/нет на вопрос робота.
Команда должна быть покрыта грамматикой ЦЕЛИКОМ: лишнее слово — не Tier-1, решает следующий шаг (медиа-грамматика
v1 или LLM). Работает в любом состоянии, включая SILENCED, — тогда отдаёт только незвучащие виды
(``knowledge.TIER1_KINDS``). Слова — из :mod:`.knowledge`, сверка по границам слов — :mod:`.words`.

Стоп внутри другой команды (В19): «спой песню про стоп» — :attr:`Tier1Command.disputed`; решает Chooser хоста
(``{stop, part_of_command}``, дедлайн ``STOP_CHOOSER_DEADLINE_MS``, таймаут → ``part_of_command``), а не этот модуль.
Музыка и диджей — медиа-грамматика v1 (``media_command_grammar``), здесь их нет.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, List, Mapping, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog.words import STOP_CLEAR, STOP_DISPUTED, has_form, has_word, stop_command, strip_wake_word

_TOKEN = re.compile(r"\w+")
#: ``(voice_id, display_name)`` голосов активного провайдера (реестр ``tts_voice_registry`` — у вызывающего).
Voices = Sequence[Tuple[str, str]]


@dataclass(frozen=True)
class Tier1Command:
    """Команда Tier-1: вид (``knowledge.TIER1_KINDS``), аргументы, спорность (В19) и остаток фразы.

    ``rest`` — новая команда после ясного стопа («хватит, спой другую» → ``stop_all`` + «спой другую»): её решают
    следующие шаги приёма. ``disputed`` — стоп-слово внутри другой команды: исполнять только по ответу Chooser.
    """

    kind: str
    args: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))
    disputed: bool = False
    rest: str = ""


def match(text: str, snapshot: SessionSnapshot, voices: Voices = ()) -> Optional[Tier1Command]:
    """Tier-1 команда во фразе (вейк уже снят или ещё стоит — без разницы) или ``None``."""
    silenced = snapshot.top("Silence") is not None
    body = strip_wake_word(text or "")
    for rule in (_stop, _new_session, _unsilence, _volume, _voice, _answer):
        command = rule((text or "") if rule is _stop else body, snapshot, voices)
        if command is not None and not (silenced and kn.TIER1_KINDS[command.kind]):
            return command
    return None


def volume_channel(snapshot: SessionSnapshot, named: Optional[str] = None) -> str:
    """Канал громкости (§4.2): названный в фразе; играет музыка → музыка; звучит речь → голос; иначе последний."""
    if named:
        return named
    if snapshot.music.state == "playing":
        return "music"
    if snapshot.speaking:
        return "tts"
    return snapshot.volume_channel if snapshot.volume_channel in kn.VOLUME_CHANNELS else "tts"


def _words(text: str) -> List[str]:
    return [t.lower().replace("ё", "е") for t in _TOKEN.findall(text)]


def _starts(word: str, stems: Sequence[str]) -> bool:
    return any(word.startswith(stem.replace("ё", "е")) for stem in stems)


def _stop(text: str, snapshot: SessionSnapshot, _voices: Voices) -> Optional[Tier1Command]:
    """Ясный стоп: тишина (в серии стоп-слов есть основа ``SILENCE_WORDS`` — «замолчи», «помолчи») или ``stop_all``
    (остановить текущие звуки, режим тишины НЕ включается: «хватит», «стоп»; решение Шифу 22.10, #3700); после
    стопа ещё команда — ``stop_all`` + ``rest`` (тишина такую команду бы заглушила). Спорный — тот же вид с
    ``disputed``.

    Охват (#3994, ``words.stop_scope``): названа речь («хватит читать/говорить/петь», «…а музыку оставь») —
    ``stop_speech`` (``args.keep_music``): речь и выступление гаснут, плеер музыки и движение — нет. Названа музыка
    («хватит музыки») или ничего («хватит», «стоп») — ``stop_all``, как было (В20). Во фразе есть «стой» (политика
    ``halt`` — стоп и движения) — тоже ``stop_all``, даже при названной речи. Оговорка «музыку оставь» без названной
    речи («стой, а музыку оставь», «хватит, бит не трогай») — ``stop_all``/``silence`` с ``args.keep_music``: гаснет всё,
    кроме музыки (движение — тоже). Покрытая охватом часть в ``rest`` не попадает."""
    call = stop_command(text)
    if call.verdict not in (STOP_CLEAR, STOP_DISPUTED):
        return None
    scope = call.scope
    silence = not call.rest and not scope.music and any(_starts(word, kn.SILENCE_WORDS) for word in call.words)
    disputed = call.verdict == STOP_DISPUTED
    halt = has_form(text, kn.TIER1_POLICY["halt"].words)
    if not silence and scope.speech and not scope.music and not halt:
        args = MappingProxyType({"keep_music": scope.keep_music})
        return Tier1Command("stop_speech", args, disputed=disputed, rest=call.rest)
    keep = MappingProxyType({"keep_music": True} if scope.keep_music else {})
    return Tier1Command("silence" if silence else "stop_all", keep, disputed=disputed, rest=call.rest)


def _new_session(text: str, _snapshot: SessionSnapshot, _voices: Voices) -> Optional[Tier1Command]:
    low = text.lower()
    return Tier1Command("new_session") if any(p in low for p in kn.NEW_SESSION_PHRASES) else None


def _unsilence(text: str, snapshot: SessionSnapshot, _voices: Voices) -> Optional[Tier1Command]:
    """«говори», «отвечай», «включайся» — только в тишине (вне её «включи музыку» — медиакоманда)."""
    if snapshot.top("Silence") is None or not has_word(text, kn.UNSILENCE_WORDS):
        return None
    return Tier1Command("unsilence")


def _drop_voice_negation(words: Sequence[str]) -> List[str]:
    """«играй громче, а не говори громче» (#3125): «не говори …» — отказ от канала голоса, а не просьба о нём."""
    out: List[str] = []
    skip = 0
    for i, word in enumerate(words):
        if skip:
            skip -= 1
            continue
        if word == "не" and i + 1 < len(words) and _starts(words[i + 1], kn.VOLUME_CHANNEL_STEMS["tts"]):
            skip = 2 if i + 2 < len(words) and _volume_intent(words[i + 2]) else 1
            continue
        out.append(word)
    return out


def _volume_intent(word: str) -> Optional[str]:
    return next((intent for intent, stems in kn.VOLUME_STEMS.items() if _starts(word, stems)), None)


def _names_track(extras: Sequence[str], title: Optional[str]) -> bool:
    """Лишние слова — название играющего трека («горный король погромче» ↔ «В пещере горного короля»)."""
    if not title or len(extras) > 5:
        return False
    named = _words(title)
    return all(any(t.startswith(w[:4]) for t in named) for w in extras)


def _volume(text: str, snapshot: SessionSnapshot, _voices: Voices) -> Optional[Tier1Command]:
    """Направление громкости + канал: названный словом («музыку», «говори») или по снимку (:func:`volume_channel`)."""
    words = _drop_voice_negation(_words(text))
    intent = _one_intent(words)
    named = {ch for w in words for ch, stems in kn.VOLUME_CHANNEL_STEMS.items() if _starts(w, stems)}
    if intent is None or len(named) > 1:
        return None
    music_title = snapshot.music.title if snapshot.music.state == "playing" else None
    extras = [w for w in words if not _volume_word(w)]
    if extras and (set(words) & set(kn.PLAY_VERBS) or not _names_track(extras, music_title)):
        return None  # «сыграй X погромче» — заказ; «расскажи анекдот погромче» — не громкость
    channel = volume_channel(snapshot, next(iter(named), None))
    return Tier1Command("volume", MappingProxyType({"action": intent, "channel": channel}))


def _one_intent(words: Sequence[str]) -> Optional[str]:
    """Одно намерение громкости во фразе: «на полную» — max; «громче или тише», отрицание — нет."""
    intents = {i for i in map(_volume_intent, words) if i}
    if "max" in intents and "quieter" not in intents:
        intents = {"max"}  # «выкрути музыку на полную»
    if len(intents) != 1 or "не" in words:
        return None
    if intents == {"normal"} and not any(w.startswith("громк") for w in words):
        return None  # «говори нормально» — не про громкость; «нормальная громкость» — про неё
    return intents.pop()


def _volume_word(word: str) -> bool:
    """Слово команды громкости: направление, канал, «громкость/звук», служебное."""
    return bool(_volume_intent(word) or word in kn.GRAMMAR_FILLER or word in kn.VOLUME_FILLER
                or _starts(word, kn.VOLUME_TOPIC_STEMS)
                or any(_starts(word, stems) for stems in kn.VOLUME_CHANNEL_STEMS.values()))


def _voice_tokens(voice_id: str, display: str) -> List[str]:
    split = re.sub(r"([a-z])([A-Z])", r"\1 \2", voice_id.split("_", 1)[-1])
    return _words(f"{display} {split}")


def _is_default_voice(names: Sequence[str]) -> bool:
    """«верни свой обычный голос»: есть слово-признак «обычного» и больше ничего, кроме служебных (#3803)."""
    marked = any(_starts(w, kn.VOICE_DEFAULT_STEMS) for w in names)
    return marked and all(_starts(w, kn.VOICE_DEFAULT_STEMS) or w in kn.VOICE_DEFAULT_FILLER for w in names)


def _voice(text: str, _snapshot: SessionSnapshot, voices: Voices) -> Optional[Tier1Command]:
    """«включи голос надёжного мужика» → ``set_voice{voice}``: каждое слово вне служебных совпадает (первые
    ``VOICE_STEM_LEN`` букв) со словом ОДНОГО голоса реестра; не нашлось или двое — не Tier-1 (решает LLM)."""
    words = _words(text)
    if not any(w.startswith(kn.VOICE_NOUN_STEM) for w in words):
        return None
    names = [w for w in words if not (w.startswith(kn.VOICE_NOUN_STEM) or w in kn.VOICE_FILLER)]
    if not names:
        return None
    if _is_default_voice(names):
        return Tier1Command("set_voice", MappingProxyType({"voice": kn.VOICE_DEFAULT_ARG}))
    n = kn.VOICE_STEM_LEN
    hits = [vid for vid, display in voices
            if all(any(len(w) >= n and t[:n] == w[:n] for t in _voice_tokens(vid, display)) for w in names)]
    return Tier1Command("set_voice", MappingProxyType({"voice": hits[0]})) if len(hits) == 1 else None


def _answer(text: str, snapshot: SessionSnapshot, _voices: Voices) -> Optional[Tier1Command]:
    """Да/нет на вопрос робота ``Ask(expects="yes_no")`` (U9): вся фраза — одно слово ответа."""
    ask = snapshot.pending_ask
    words = _words(text)
    if ask is None or ask.expects != "yes_no" or len(words) != 1:
        return None
    value = "yes" if words[0] in kn.YES_WORDS else "no" if words[0] in kn.NO_WORDS else None
    return Tier1Command("answer", MappingProxyType({"value": value})) if value else None


__all__ = ["Tier1Command", "Voices", "match", "volume_channel"]
